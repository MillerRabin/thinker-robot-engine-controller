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

  // Past-deadline speed is gradient-based (proportional to absError), not flat - a flat 1deg/s felt
  // too slow recovering from a real external disturbance (someone rotating the whole arm), but
  // going back to the plain error/timeLeft formula reproduced the original sustained oscillation
  // (see tick()'s comment). stabilizationSpeed is the ceiling that gradient is clamped to - same
  // ceiling already proven safe for initial-seed-recovery scaling elsewhere.
  float stabilizationGainPerSec = 0.5f; // deg/s per deg of error, see setHoldResponse()
  float stabilizationSpeed = 18.0f; // deg/s ceiling

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

  // REMOVED (2026-11-03, the user's call): the last freeze guard - error grown by more than 2deg in
  // each of 6 consecutive 0.3 s checks. Like the stall check below, it read a tilting base as a
  // fault: on a fast platform tilt the error outgrows the hold speed. With it went the whole
  // diverging/isDiverging() freeze; a joint now always keeps correcting.


  // REMOVED (2026-08-24): a "lead clamp" + hard physicalAngle-vs-imuAngle gap guard used to live
  // here, meant to stop physicalAngle from racing ahead of imuAngle. Both versions tried (an
  // absolute-gap version, then an anchored-relative-progress version) assumed some fixed
  // correspondence between physicalAngle's scale and imuAngle's scale - offset-only, then
  // offset-plus-1:1-slope. BOTH assumptions were confirmed wrong for elbow, the second time by
  // causing a REAL, physically-observed dangerous oscillation live (elbow commanded to swing
  // between its 0 and 270deg Range limits during calibration, chasing a mathematically-predicted
  // physicalAngle that didn't correspond to reality - see project memory item 57 for the full
  // incident). Do not reintroduce a check that assumes a specific physicalAngle-vs-imuAngle
  // relationship without solid, full-range live-measured evidence it holds for every joint that
  // will use it - it doesn't for elbow, and there's no guarantee it holds for future joints either.

  // REMOVED (2026-11-03): a stall check that froze the joint when imuAngle had moved less than 2deg
  // over 5 s with the error still >= 2deg - built for windup while the data lagged ("physicalAngle
  // ran from ~91 to its 180 range limit over several seconds while imuAngle sat at ~90"). It assumed a
  // still base: a joint holding its angle while the platform or its parent tilts rocks around its
  // target, so the IMU ends where it started 5 s earlier while the command has moved 17deg - it froze
  // the elbow, then the shoulder, into recalibrations on a live platform tilt. Windup now ends at the
  // safe limit, where tick() holds.

  // Error size that counts as a real correction rather than noise or a bias - see the
  // fine-correction budget in tick().
  static constexpr float minProgressDeg = 2.0f;

  // Opt-in anti-windup limit (deg), NAN = disabled - see setMaxLead() and tick().
  float maxLeadDeg = NAN;

  // Fine-correction budget - see tick(). Reset only by a new target (setTargetAngle()/reset()), not
  // by resetCorrection(), which also runs on every dead-zone dip.
  float fineAnchorPhysical = NAN;
  float fineCorrectionBudgetDeg = 3.0f; // see setFineCorrectionBudget()
  // When absError first went >= minProgressDeg, 0 while below - the anchor is only released once
  // that lasts fineReleaseUs (see tick()).
  absolute_time_t coarseSince = 0;
  static constexpr int64_t fineReleaseUs = 1000000;
  // Opt-in, NAN = off: an error this large releases the anchor at once - see setFineReleaseError().
  float fineReleaseErrorDeg = NAN;

  // True while the PWM output carries a real commanded pulse (set by setDegreeDirect(), cleared by
  // release()) - the constructor's homePosition is bookkeeping only, nothing is output until then.
  bool outputActive = false;

  // Restarts the speed ramp and the fine-budget release timer - see tick().
  void resetCorrection();

  // Active damping, opt-in (see setDamping()): each tick the pulse leads physicalAngle by
  // -dampingGainSec * (linkRate - appliedSpeedDegPerSec), within +-dampingMaxDeg. physicalAngle
  // stays the command - the offset only goes to the output.
  float dampingGainSec = 0.0f;
  float dampingMaxDeg = 0.0f;
  float dampingDeadbandDegPerSec = 0.0f;
  float linkRate = NAN;       // deg/s, from setLinkRate(); consumed by tick()
  float dampingOffset = 0.0f; // what the last tick() added to the pulse
  void tickCommand();
  void applyDamping();
  // The PWM pulse for `degree`, clamped to the safe range - physicalAngle untouched.
  void writePulse(float degree);

  // Planned move (see planMove()): the command follows a minimum-jerk curve from planStart to
  // planEnd over planDurationUs, by time rather than by IMU error; tick() then falls back to its
  // slow IMU trim. planActive is written last by planMove() (CAN task) and read first by tick().
  volatile bool planActive = false;
  float planStart = NAN;
  float planEnd = NAN;
  absolute_time_t planStarted = 0;
  int64_t planDurationUs = 0;
  bool planTick(absolute_time_t now, int64_t dtUs);
  // Opt-in settle wait after a plan (see setPlanSettle()): the IMU trim holds off while the joint
  // is still moving toward the plan's end, up to planSettleMaxUs. 0 = trim right away.
  int64_t planSettleMaxUs = 0;
  bool planSettling = false;
  absolute_time_t planEndedAt = 0;
  float settleRefIMU = NAN;
  absolute_time_t settleRefTime = 0;
  static constexpr int64_t settleCheckUs = 100000;
  static constexpr float settleMotionDeg = 0.3f; // per settleCheckUs, i.e. 3deg/s
  bool settleTick(absolute_time_t now);
  // Backlash/sag compensation for planned moves (see planMove()): where a loaded joint settles
  // relative to its command depends on which way gravity loads it, so the link-minus-command
  // offset it last settled at is remembered per zone and reused as the next plan's end offset.
  // Zones: 0 below gravityNeutral, 1/2 within neutralBandDeg of it approached from below/above
  // (unloaded, the side depends on momentum), 3 above. NAN = nothing learned yet.
  float gravityNeutral = NAN;
  static constexpr float neutralBandDeg = 3.0f;
  static constexpr float learnToleranceDeg = 0.5f;
  float zoneOffset[4] = {NAN, NAN, NAN, NAN};
  int8_t planZone = -1; // zone the running planned target learns into, -1 = none or done
  int8_t zoneOf(float target, float from) const;

  // Diagnostic trace, opt-in (see enableTrace()): every tick() for traceCapacity ticks after a
  // target change of at least traceMinMoveDeg, then dumped to the log a few samples per tick so the
  // printing doesn't disturb the 5 ms loop while recording. Shows whether the command itself swings
  // with the IMU, and with what lag.
  struct TraceSample {
    uint32_t tUs;
    float target;
    float rawIMU;
    float imu;
    float physical;
    float aux; // see setTraceAux()
  };
  enum class TraceState : uint8_t { Idle, Recording, Dumping };
  static constexpr uint16_t traceCapacity = 800;
  static constexpr float traceMinMoveDeg = 1.0f;
  TraceSample *trace = nullptr;
  TraceState traceState = TraceState::Idle;
  // Set by setTargetAngle(), which runs on the CAN task - tick() starts the recording itself.
  volatile bool traceRequested = false;
  uint16_t traceCount = 0;
  uint16_t traceDumped = 0;
  absolute_time_t traceStart = 0;
  float traceFrom = NAN;
  float traceTo = NAN;
  uint16_t traceTimeMS = 0;
  float rawIMUAngle = NAN; // setIMUAngle()'s input before the low-pass filter, for the trace
  float traceAux = NAN;
  void traceTick(absolute_time_t now);

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
  // setTargetAngle() for a commanded move: the command travels the IMU distance to the target on a
  // smooth, time-based curve instead of chasing the IMU error - see tick() and planTick().
  bool planMove(const float angle, uint16_t timeMS, float deadZone);
  // For a joint slower than its plans: after a plan, wait (up to maxMS) for the joint to stop
  // before the IMU trim starts, so the trim doesn't push the command on while the joint catches up.
  void setPlanSettle(uint32_t maxMS) { planSettleMaxUs = (int64_t)maxMS * 1000; }
  // IMU-space angle where gravity doesn't load the joint (vertical) - enables the per-zone
  // backlash compensation of planMove(). Set after each calibration.
  void setGravityNeutral(float angle) { gravityNeutral = angle; }
  float getTargetAngle() const { return targetAngle; }

  void setIMUAngle(float value);
  
  float getPhysicalAngle() const { return physicalAngle; }
  float getIMUAngle() const { return imuAngle; }

  // =========================================================
  // Config setters
  // =========================================================
  void setTimeMS(uint16_t timeMS);
  void setDeadZone(float dz);
  // Only for a joint whose physicalAngle and imuAngle are verified to share one scale (offset ~0,
  // slope ~1) - elbow's didn't when a relationship-assuming check was last tried (see the removed
  // lead-clamp comment above). Unlike that check, this one only ever holds physicalAngle back.
  void setMaxLead(float deg) { maxLeadDeg = deg; }
  // A joint whose backlash is wider than the default 3deg needs a wider budget, or the command can't
  // cross the backlash once the error is small and the joint never settles (see tick()).
  void setFineCorrectionBudget(float deg) { fineCorrectionBudgetDeg = deg; }
  // For a joint holding its own angle against a parent's motion: the budget's 1 s release made it
  // ride along with the parent for that whole second (the wrist tilted 17.7deg on a 15deg elbow move,
  // 2026-11-03). An error this large is a real disturbance - beyond the noise and the sub-degree
  // bias the budget exists for - so it releases the anchor at once.
  void setFineReleaseError(float deg) { fineReleaseErrorDeg = deg; }
  // Holding gain and speed ceiling once past a move's deadline (see tick()). The defaults (0.5/s,
  // 18deg/s) date from 2026-08-26, when data lagged and faster recovery oscillated; a joint that
  // must follow a parent's motion needs more.
  void setHoldResponse(float gainPerSec, float maxSpeed) {
    stabilizationGainPerSec = gainPerSec;
    stabilizationSpeed = std::min(maxSpeed, maxAngularSpeed);
  }
  // An extra value recorded with every trace sample, e.g. an independent angle estimate to check
  // the IMU angle against. Set it before tick().
  void setTraceAux(float value) { traceAux = value; }
  // Starts a trace now if none is running - for a disturbance from outside, when the joint's own
  // target doesn't change (see enableTrace()).
  void requestTrace() {
    if (trace != nullptr && traceState == TraceState::Idle) {
      traceFrom = targetAngle;
      traceTo = targetAngle;
      traceTimeMS = 0;
      traceRequested = true;
    }
  }
  // Allocates the trace buffer (~19 KB) - see TraceSample. False if the heap can't spare it.
  bool enableTrace();
  bool isTraceIdle() const { return traceState == TraceState::Idle && !traceRequested; }
  // Active damping: the servo yields against the link's own swing. gainSec = degrees of pulse per
  // deg/s of the link moving faster than the command does, past deadbandDegPerSec; maxDeg bounds the
  // offset. 0 = off.
  void setDamping(float gainSec, float maxDeg, float deadbandDegPerSec) {
    dampingGainSec = gainSec;
    dampingMaxDeg = maxDeg;
    dampingDeadbandDegPerSec = deadbandDegPerSec;
  }
  // The link's angular rate about this joint's axis (deg/s, IMU-measured, positive the way a growing
  // command turns it), for this tick only: tick() consumes it, so a joint that stops feeding it - or
  // feeds NaN - gets no damping.
  void setLinkRate(float degPerSec) { linkRate = degPerSec; }
  float getDampingOffset() const { return dampingOffset; }
  void tick();

  void stop() {
    targetAngle = NAN;
  }

  // Stops control but keeps holding: the PWM output stays at the last commanded position, and
  // physicalAngle keeps describing it, so the next calibration can seed from it with no jump (see
  // isHolding()). For a lost parent while engines stay on - the previous version
  // also cut the output (release()), so every shoulder recalibration dropped a limp elbow and wrist
  // under gravity (confirmed live 2026-10-31: the wrist "lost vertical" in one such cascade).
  void reset() {
    planActive = false;
    planSettling = false;
    planZone = -1;
    targetAngle = NAN;
    fineAnchorPhysical = NAN;
    coarseSince = 0;
    // Must also clear here - setIMUAngle() low-pass filters against a stale imuAngle otherwise.
    imuAngle = NAN;
  }

  // reset() plus cutting the output - only for engines going off, see each module's updateStatusLed().
  void release() {
    reset();
    physicalAngle = NAN;
    outputActive = false;
    // Clearing the software state alone leaves the PWM peripheral outputting whatever duty cycle it
    // was last set to - RP2040 PWM slices hold their last level indefinitely with no software
    // refresh needed, and that keeps driving the servo even through an engines-off period (only the
    // servo's own power rail is cut, not RP2040 logic). Confirmed live (2026-08-26): the servo
    // mechanically snaps to that stale leftover signal the instant power returns - before any
    // calibration/seeding logic even runs - which looked like an uncontrolled jerk toward the old
    // target. Level 0 removes the pulses entirely so there's nothing to snap to; setDegreeDirect()
    // writes a real level again the moment a real, intentional position is next commanded.
    // Not pwm_set_enabled(false) (the first version of this fix): disabling a slice halts its
    // counter with the pin frozen high or low depending on exactly when it stopped (pico-sdk
    // pwm_set_enabled() docs) - at a ~50% duty pulse that's a coin flip on leaving a constant-high
    // line on the servo input. Level 0 is a clean constant low, and only touches this channel.
    pwm_set_chan_level(slice, channel, 0);
  }

  bool isPositioned() const;
  // Actively outputting physicalAngle - the servo is physically there (within backlash/sag).
  bool isHolding() const { return outputActive && !isnan(physicalAngle); }
};