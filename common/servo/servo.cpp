#include "servo.h"

Servo::Servo(const uint pin, Range degreeRange, const float homePosition,
             const float freq, const float lowPeriod, const float highPeriod)
    : pin(pin), minDegree(degreeRange.from), maxDegree(degreeRange.to),
      lowPeriod(lowPeriod), highPeriod(highPeriod), physicalAngle(homePosition),
      printer(pdMS_TO_TICKS(1000)) {
  gpio_set_function(pin, GPIO_FUNC_PWM);
  slice = pwm_gpio_to_slice_num(pin);
  channel = pwm_gpio_to_channel(pin);
  setFrequency(freq);
}

int Servo::getWrapAndDivider(const uint freq, PWMParameters &params) const {
  uint32_t clock = clock_get_hz(clk_sys);
  uint8_t tDiv = 1;
  uint32_t tWrap = 0;
  uint8_t tDivFrac = 0;

  for (; tDiv < UINT8_MAX; tDiv++) {
    tWrap = (clock / tDiv) / freq - 1;
    if (tWrap <= UINT16_MAX)
      break;
  }

  if (tDiv == UINT8_MAX)
    return SERVO_FREQUENCY_OUT_OF_RANGE;

  float realDivider = (float)clock / ((float)(tWrap + 1) * (float)freq);
  tDivFrac = (uint8_t)roundf((realDivider - tDiv) * 16.0f);
  if (tDivFrac > 15) {
    tDivFrac = 0;
    tDiv++;
  }

  params.wrap = tWrap;
  params.divInt = tDiv;
  params.divFrac = tDivFrac;
  return 0;
}

int Servo::setFrequency(const uint freq) {
  uint32_t clock = clock_get_hz(clk_sys);

  if (freq < 1 || freq > clock)
    return SERVO_FREQUENCY_OUT_OF_RANGE;

  PWMParameters params;

  int res = getWrapAndDivider(freq, params);
  if (res != 0) {
    return res;
  }

  float period = (params.divInt + params.divFrac / 16.0f) / (float)clock;
  lowSlices = getSlices(lowPeriod, period, params.wrap);
  highSlices = getSlices(highPeriod, period, params.wrap);

  if (highSlices <= lowSlices)
    return SERVO_WRONG_PERIODS;

  delta = highSlices - lowSlices;
  pulseStep = (float)delta / (maxDegree - minDegree);

  pwm_set_clkdiv_int_frac(slice, params.divInt, 0);
  pwm_set_wrap(slice, (uint16_t)params.wrap);
  pwm_set_enabled(slice, true);

  initialized = true;
  return 0;
}

uint16_t Servo::getSlices(const float targetPeriod, const float period,
                          const uint32_t wrap) const {
  if (targetPeriod <= 0.0f)
    return 0;

  uint32_t slices = (uint32_t)roundf(targetPeriod / period);

  if (slices > wrap)
    slices = wrap;

  return (uint16_t)slices;
}

int Servo::setDegreeDirect(const float degree) {
  if (!initialized) {
    return SERVO_NOT_INITIALIZED;
  }
  if (isnan(degree)) {
    return SERVO_DEGREE_IS_NAN;
  }
  if (degree < minDegree) {
    return SERVO_DEGREE_IS_BELOW_MINIMUM;
  }
  if (degree > maxDegree) {
    return SERVO_DEGREE_IS_ABOVE_MAXIMUM;
  }

  uint16_t slices =
      lowSlices + (uint16_t)roundf((degree - minDegree) * pulseStep);
  pwm_set_chan_level(slice, channel, slices);
  if (positionTime == 0) {
    positionTime = xTaskGetTickCount();
  }
  physicalAngle = degree;
  return SERVO_OK;
}

bool Servo::setTargetAngle(const float angle, uint16_t newTimeMS,
                           float newDeadZone) {
  if (isnan(angle))
    return false;
  if (angle < minDegree || angle > maxDegree)
    return false;

  targetAngle = angle;
  setDeadZone(newDeadZone);
  setTimeMS(newTimeMS);
  moveStarted = get_absolute_time();
  resetDivergence();
  return true;
}

void Servo::resetDivergence() {
  divergenceCheckError = NAN;
  divergingIntervals = 0;
  diverging = false;
  appliedSpeedDegPerSec = 0.0f;
  limitPinnedSince = 0;
}

void Servo::tick() {
  if (!pulseStepLogged) {
    pulseStepLogged = true;
    LogQueue::Log("[DIAG] Servo pin=%u pulseStep=%.3f slices/deg (min PWM step ~%.4f deg)\n",
                  pin, pulseStep, 1.0f / pulseStep);
  }

  absolute_time_t now = get_absolute_time();
  int64_t dtUs = absolute_time_diff_us(lastTickTime, now);
  lastTickTime = now;

  if ((dtUs <= 0) || isnan(imuAngle) || isnan(physicalAngle) || isnan(targetAngle))
    return;

  if (diverging)
    return;

  const float error = targetAngle - imuAngle;
  const float absError = fabsf(error);
  // Hold still within the dead zone, otherwise sensor noise keeps producing a nonzero error forever.
  if (absError <= deadZone) {
    resetDivergence();
    return;
  }

  if (isnan(divergenceCheckError)) {
    divergenceCheckError = absError;
    divergenceCheckTime = now;
  } else if (absolute_time_diff_us(divergenceCheckTime, now) >= divergenceCheckIntervalUs) {
    if (absError > divergenceCheckError + divergenceMarginDeg) {
      divergingIntervals++;
      if (divergingIntervals >= divergenceMaxIntervals) {
        diverging = true;
        LogQueue::Log("Servo diverging: error grew %.2f -> %.2f over %d checks, freezing\n",
                      divergenceCheckError, absError, divergingIntervals);
        return;
      }
    } else {
      divergingIntervals = 0;
    }
    divergenceCheckError = absError;
    divergenceCheckTime = now;
  }

  float dtSec = dtUs / 1000000.0f;
  dtSec = std::min(dtSec, 0.01f);

  const float dir = (error >= 0.0f) ? 1.0f : -1.0f;
  int64_t elapsedUs = absolute_time_diff_us(moveStarted, now);
  int64_t remainingUs = timeUs - elapsedUs;
  remainingUs = std::max<int64_t>(remainingUs, stabilizationTimeUs);

  float timeLeftSec = std::max(remainingUs / 1000000.0f, 0.001f);
  float desiredSpeedDegPerSec = absError / timeLeftSec;
  float limitedSpeedDegPerSec = fminf(desiredSpeedDegPerSec, maxAngularSpeed);

  // Ramp the applied speed toward the desired speed instead of jumping to it -
  // otherwise a single noisy/moving-target tick can swing the commanded speed
  // instantly, which is what made continuous tracking (elbow following a
  // moving shoulder) look jerky rather than smooth.
  float desiredSignedSpeed = limitedSpeedDegPerSec * dir;
  float maxSpeedDelta = maxAngularAccelerationCmd * dtSec;
  float speedDelta = std::clamp(desiredSignedSpeed - appliedSpeedDegPerSec, -maxSpeedDelta, maxSpeedDelta);
  appliedSpeedDegPerSec += speedDelta;

  float increment = appliedSpeedDegPerSec * dtSec;
  float nextPhysical = physicalAngle + increment;
  printer.interval([&]() {
    LogQueue::Log("Tick: Target: %.2f, IMU: %.2f, Physical: %.2f, Error: %.2f, DesiredSpeed: %.2f, AppliedSpeed: %.2f\n",
                  targetAngle, imuAngle, physicalAngle, error, desiredSpeedDegPerSec, appliedSpeedDegPerSec);
  });

  bool pinnedAtLimit = (nextPhysical <= minDegree && dir < 0.0f) || (nextPhysical >= maxDegree && dir > 0.0f);
  if (pinnedAtLimit) {
    if (limitPinnedSince == 0) {
      limitPinnedSince = now;
    } else if (absolute_time_diff_us(limitPinnedSince, now) >= limitPinnedMaxUs) {
      diverging = true;
      LogQueue::Log("Servo diverging: pinned at limit (%.2f) with error %.2f, freezing\n",
                    nextPhysical <= minDegree ? minDegree : maxDegree, absError);
      physicalAngle = std::clamp(nextPhysical, minDegree, maxDegree);
      setDegreeDirect(physicalAngle);
      return;
    }
  } else {
    limitPinnedSince = 0;
  }

  physicalAngle = std::clamp(nextPhysical, minDegree, maxDegree);

  setDegreeDirect(physicalAngle);
}

void Servo::setIMUAngle(float value) {
  // Propagate NaN rather than keeping a stale value - tick()'s NaN guard must see it and freeze.
  if (isnan(value)) {
    imuAngle = value;
    return;
  }

  // No [minDegree, maxDegree] clamp: imuAngle is feedback in the caller's units (can be negative),
  // not a PWM command. physicalAngle (the actual PWM output) is range-clamped separately in tick().
  // Low-pass filter instead of taking the raw reading directly - sensor noise otherwise feeds
  // straight into the speed calculation in tick(), showing up as jerks even with speed ramping.
  if (isnan(imuAngle)) {
    imuAngle = value;
  } else {
    imuAngle = imuFilterAlpha * value + (1.0f - imuFilterAlpha) * imuAngle;
  }

  if (isnan(physicalAngle)) {
    physicalAngle = imuAngle;
  }
}

void Servo::setTimeMS(uint16_t timeMS) {
  this->timeUs = (timeMS < 1 || timeMS > 10000) ? this->timeUs : timeMS * 1000;
}

void Servo::setDeadZone(float dz) {
  if (dz < 0.0f || dz > 5.0f) {
    return;
  }
  deadZone = dz;
}

bool Servo::isPositioned() const {
  if (isnan(targetAngle) || isnan(imuAngle)) {
    return false;
  }
  return fabsf(targetAngle - imuAngle) <= deadZone;
}