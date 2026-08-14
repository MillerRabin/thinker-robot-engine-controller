#pragma once

#include "../config/config.h"
#include "../periodic/periodic.h"
#include "../speedBuffer/speedBuffer.h"
#include "../logQueue/logQueue.h"
#include "hardware/clocks.h"
#include "pico/divider.h"
#include "pico/stdlib.h"
#include <FreeRTOS.h>
#include <algorithm>
#include <hardware/pwm.h>
#include <iostream>
#include <math.h>
#include <task.h>

#define SERVO_OK 0
#define SERVO_DEGREE_IS_BELOW_MINIMUM 1
#define SERVO_DEGREE_IS_ABOVE_MAXIMUM 2
#define SERVO_DEGREE_IS_NAN 3
#define SERVO_NOT_INITIALIZED 4
#define SERVO_FREQUENCY_OUT_OF_RANGE 5
#define SERVO_WRONG_PERIODS 6

class Range {
public:
  const float from;
  const float to;
  Range(const float from, const float to) : from(from), to(to) {}
};

struct PWMParameters {
  uint8_t divInt;
  uint8_t divFrac;
  uint32_t wrap;
};

class Servo {
private:
  uint pin;
  uint slice;
  uint channel;
  
  uint8_t clk_divider = 1;
  const float lowPeriod;
  const float highPeriod;
    
  uint16_t lowSlices = 0;
  uint16_t highSlices = 0;
  uint16_t delta = 0;
  float pulseStep = 0.0f;
  bool pulseStepLogged = false;
  bool initialized = false;
  TickType_t positionTime = 0;
  const TickType_t positionInterval = pdMS_TO_TICKS(1000);
  constexpr static float tickInterval = (float)ENGINE_TASK_LOOP_TIMEOUT_US;
  // =========================================================
  // Servo / movement control settings
  // =========================================================
  float deadZone = 1.0f; // допустимая ошибка по углу (deg)
  const uint64_t stabilizationTimeUs = 500000;
  
  float stabilizationSpeed = 1.0f; // deg/s

  const float maxAngularSpeed = 60.0f;   // deg/s
  float maxAngularAccelerationCmd = 180.0f; // deg/s^2
  float appliedSpeedDegPerSec = 0.0f; // signed, ramped toward the desired speed by maxAngularAccelerationCmd
  const float maxAngleStepPerTick = 30.0f; // deg, to prevent too large steps in case of long delays
  int64_t timeUs = 1000000; // default 1 second
  absolute_time_t moveStarted = 0;
  absolute_time_t lastTickTime;
  float velocityKp = 0.3f;  // velocity loop gain
  
  float imuAngle = NAN;      // real angle from IMU, low-pass filtered in setIMUAngle()
  const float imuFilterAlpha = 0.2f; // lower = smoother but laggier
  float lastIMUAngle = NAN;  // last angle from IMU, used for acceleration estimation
  float physicalAngle = NAN; // commanded servo angle
  float targetAngle = NAN;   // target angle

  // Divergence guard: freeze if tracking error grows instead of shrinking for a sustained period.
  float divergenceCheckError = NAN;
  absolute_time_t divergenceCheckTime = 0;
  uint8_t divergingIntervals = 0;
  bool diverging = false;
  static constexpr int64_t divergenceCheckIntervalUs = 300000;
  static constexpr uint8_t divergenceMaxIntervals = 6;
  static constexpr float divergenceMarginDeg = 2.0f;

  // Catches the case a growing-error check misses: pinned at a limit with a large but non-growing error.
  absolute_time_t limitPinnedSince = 0;
  static constexpr int64_t limitPinnedMaxUs = divergenceCheckIntervalUs * divergenceMaxIntervals;

  // Anchor pair, captured once per move (cleared in resetDivergence(), lazily re-captured on the
  // first tick() after). physicalAngle and imuAngle are NOT assumed to share the same numeric
  // frame/zero-point - confirmed live: true for shoulder/wrist (same scale), but NOT for elbow,
  // which has a large, legitimate, fixed offset between its PWM scale and its gravity-sensed scale.
  // Both checks below compare physicalAngle against how far it *should* have moved given imuAngle's
  // own real progress since this anchor, not against imuAngle's raw value - this generalizes
  // correctly to any fixed per-joint offset, since only relative progress is ever compared.
  float leadAnchorPhysical = NAN;
  float leadAnchorImu = NAN;

  // Catches a third case neither check above sees: a persistent (not growing) small bias in
  // imuAngle relative to targetAngle - error stays small and non-growing, so the checks above
  // never trip, but tick() still nudges physicalAngle a little further every tick since it never
  // exactly reaches the (tight) dead zone. Nothing ties physicalAngle's own displacement back to
  // imuAngle's real progress, so it can drift unboundedly - confirmed live (shoulder ran
  // physicalAngle from ~91 to its 180 range limit over several seconds while imuAngle sat at a
  // near-constant ~90). Sustained, not instantaneous - a single-tick gap right after a fresh seed
  // is expected and must not trip this; only a gap that persists is the real failure signature.
  absolute_time_t physicalImuGapSince = 0;
  static constexpr float maxPhysicalImuGapDeg = 20.0f;
  static constexpr int64_t physicalImuGapMaxUs = divergenceCheckIntervalUs * divergenceMaxIntervals;

  // Soft rate limit, checked every tick (not just after the fact like the guard above): never let
  // physicalAngle get more than this far ahead of where imuAngle's own progress says it should be.
  // The deadline-driven speed calc above only reacts to error vs targetAngle, so if imuAngle can't
  // keep up (real torque limit, filter lag) it just commands more speed instead of backing off -
  // this clamp is what actually stops physicalAngle from racing ahead, rather than freezing after
  // it already has. Kept comfortably under maxPhysicalImuGapDeg so the hard guard stays a rare backstop.
  static constexpr float maxLeadDeg = 10.0f;

  // With physicalAngle now rate-limited to follow imuAngle, a genuinely stuck joint (imuAngle not
  // advancing - real mechanical stall, not just lag) no longer produces a growing physical-imu gap
  // to catch - it would sit quietly forever instead. Catch that directly: imuAngle must move at
  // least minProgressDeg over each check window while error is still outside the dead zone.
  // Window is deliberately longer than the other guards' 1.8s: confirmed live (shoulderY, no
  // mechanical resistance by hand) that closing the last few degrees under the new maxLeadDeg
  // clamp is legitimately slower than the old deadline-chasing ramp - 1.8s falsely caught that
  // as "stuck". 5s gives real (if unhurried) convergence room while still catching an actual stall.
  absolute_time_t progressCheckTime = 0;
  float progressCheckImuAngle = NAN;
  static constexpr float minProgressDeg = 1.0f;
  static constexpr int64_t progressCheckIntervalUs = 5000000;

  void resetDivergence();

  int getWrapAndDivider(const uint freq, PWMParameters& params) const;
  uint16_t getSlices(const float targetPeriod, const float period, const uint32_t wrap) const;
  Periodic printer;

public:
  const float maxDegree;
  const float minDegree;
  // Software-enforced safety limits, separate from minDegree/maxDegree: those two calibrate the
  // degree-to-PWM-pulse-width scale (pulseStep) and must stay equal to the servo's true full
  // mechanical span, or the whole degree scale silently distorts. Pass safeRange to the constructor
  // instead to keep commands away from a physical stop found empirically - it clamps without
  // touching pulseStep. Defaults to the full degreeRange (no extra restriction) when omitted.
  const float safeMinDegree;
  const float safeMaxDegree;
  int setDegreeDirect(const float degree);
  int setFrequency(const uint freq);

  Servo(const uint pin, Range degreeRange, const float homePosition,
        const float freq = 50, const float lowPeriod = 0.0005f,
        const float highPeriod = 0.0025f, Range safeRange = Range(NAN, NAN));
  bool setTargetAngle(const float angle, uint16_t timeMS, float deadZone);
  float getTargetAngle() const { return targetAngle; }

  void setIMUAngle(float value);
  
  float getPhysicalAngle() const { return physicalAngle; }
  float getIMUAngle() const { return imuAngle; }

  // =========================================================
  // Config setters
  // =========================================================
  void setTimeMS(uint16_t timeMS);
  void setDeadZone(float dz);  
  void tick();

  void stop() {
    targetAngle = NAN;
  }

  void reset() {
    targetAngle = NAN;
    physicalAngle = NAN;
    // Must also clear here - setIMUAngle() low-pass filters against a stale imuAngle otherwise.
    imuAngle = NAN;
  }

  bool isPositioned() const;
  bool isDiverging() const { return diverging; }
};