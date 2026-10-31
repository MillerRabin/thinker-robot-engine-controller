#include "servo.h"

Servo::Servo(const uint pin, Range degreeRange, const float homePosition,
             const float freq, const float lowPeriod, const float highPeriod, Range safeRange)
    : pin(pin), minDegree(degreeRange.from), maxDegree(degreeRange.to),
      safeMinDegree(isnan(safeRange.from) ? degreeRange.from : std::clamp(safeRange.from, degreeRange.from, degreeRange.to)),
      safeMaxDegree(isnan(safeRange.to) ? degreeRange.to : std::clamp(safeRange.to, degreeRange.from, degreeRange.to)),
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
  if (degree < safeMinDegree) {
    return SERVO_DEGREE_IS_BELOW_MINIMUM;
  }
  if (degree > safeMaxDegree) {
    return SERVO_DEGREE_IS_ABOVE_MAXIMUM;
  }

  uint16_t slices =
      lowSlices + (uint16_t)roundf((degree - minDegree) * pulseStep);
  pwm_set_chan_level(slice, channel, slices);
  outputActive = true;
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
  if (angle < safeMinDegree || angle > safeMaxDegree)
    return false;

  targetAngle = angle;
  setDeadZone(newDeadZone);
  setTimeMS(newTimeMS);
  moveStarted = get_absolute_time();
  resetDivergence();
  fineAnchorPhysical = NAN;
  return true;
}

void Servo::resetDivergence() {
  divergenceCheckError = NAN;
  divergingIntervals = 0;
  diverging = false;
  appliedSpeedDegPerSec = 0.0f;
  limitPinnedSince = 0;
  progressCheckTime = 0;
  progressCheckImuAngle = NAN;
  coarseSince = 0;
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

  // Real mechanical stall check - imuAngle itself must be making progress. Relationship-agnostic
  // by design (see the header comment) - doesn't compare against physicalAngle at all.
  // Skipped when absError is already below minProgressDeg: confirmed live (2026-08-25) that this
  // was tripping on completely successful, stable holding just outside the (much tighter) dead zone
  // - imuAngle correctly wasn't moving because there was nothing meaningful left to correct, not
  // because anything was stuck. Every one of those trips forced a full reset+recalibration on
  // shoulder, which cascaded into elbow (whose engineLoop depends on shoulder.isPositionOK()) also
  // resetting and re-seeding - a real, visible several-tens-of-degrees move each time, causing a
  // repeating "settles, then jerks" pattern neither joint's own numbers looked wrong enough to catch.
  if (absError < minProgressDeg) {
    progressCheckTime = 0;
    progressCheckImuAngle = NAN;
  } else if (isnan(progressCheckImuAngle)) {
    progressCheckImuAngle = imuAngle;
    progressCheckTime = now;
  } else if (absolute_time_diff_us(progressCheckTime, now) >= progressCheckIntervalUs) {
    if (fabsf(imuAngle - progressCheckImuAngle) < minProgressDeg) {
      diverging = true;
      LogQueue::Log("Servo diverging: imuAngle stuck near %.2f (target %.2f), freezing\n",
                    imuAngle, targetAngle);
      return;
    }
    progressCheckImuAngle = imuAngle;
    progressCheckTime = now;
  }

  float dtSec = dtUs / 1000000.0f;
  dtSec = std::min(dtSec, 0.01f);

  const float dir = (error >= 0.0f) ? 1.0f : -1.0f;
  int64_t elapsedUs = absolute_time_diff_us(moveStarted, now);
  int64_t remainingUs = timeUs - elapsedUs;
  remainingUs = std::max<int64_t>(remainingUs, stabilizationTimeUs);

  float timeLeftSec = std::max(remainingUs / 1000000.0f, 0.001f);
  float desiredSpeedDegPerSec = absError / timeLeftSec;
  // Past the nominal deadline, remainingUs is pinned at stabilizationTimeUs regardless of how large
  // absError still is - so desiredSpeedDegPerSec no longer reflects genuine urgency, just "how much
  // time is left in an artificial floor," which demands near-maxAngularSpeed for any uncorrected
  // error indefinitely rather than settling. Combined with real mechanical/sensor lag, that turned
  // into a sustained, undamped oscillation instead of convergence (confirmed live 2026-08-26:
  // shoulder swinging ~30-145deg for many cycles, never settling).
  //
  // Once past the deadline, switch to a gradient proportional to absError instead - a flat cap
  // (first tried) was safe but felt too slow recovering from a real external disturbance (someone
  // rotating the whole arm by hand); scaling with error size gives a faster response to a large
  // disturbance while a small residual still corrects gently. Clamped at stabilizationSpeed, well
  // under maxAngularSpeed - there's no genuine reason to match the original urgency once the
  // schedule itself has already been missed.
  bool pastDeadline = remainingUs <= (int64_t)stabilizationTimeUs;
  if (pastDeadline) {
    desiredSpeedDegPerSec = absError * stabilizationGainPerSec;
  }
  float speedCeiling = pastDeadline ? stabilizationSpeed : maxAngularSpeed;
  float limitedSpeedDegPerSec = fminf(desiredSpeedDegPerSec, speedCeiling);

  // Ramp toward the desired speed instead of jumping to it - a noisy/moving target otherwise looks jerky.
  float desiredSignedSpeed = limitedSpeedDegPerSec * dir;
  float maxSpeedDelta = maxAngularAccelerationCmd * dtSec;
  float speedDelta = std::clamp(desiredSignedSpeed - appliedSpeedDegPerSec, -maxSpeedDelta, maxSpeedDelta);
  appliedSpeedDegPerSec += speedDelta;

  float increment = appliedSpeedDegPerSec * dtSec;
  float nextPhysical = physicalAngle + increment;

  // Anti-windup (opt-in, see setMaxLead()): physicalAngle may advance at most maxLeadDeg ahead of
  // imuAngle in the direction of travel. Confirmed from live logs (2026-08-25..31): when the arm
  // can't keep up with the commanded speed (shoulder lifting from a droop moved 0-4deg/s while
  // commanded 10-27deg/s), physicalAngle ran 25-70deg ahead, kept integrating past the target, and
  // then dragged the arm through it - every 17-23deg growing-error trip had that shape. Only ever
  // holds physicalAngle back, never moves it toward imuAngle, so a wrong offset assumption can at
  // worst stop the advance (caught by the stall check), not chase a predicted position.
  bool leadLimited = false;
  if (!isnan(maxLeadDeg) && dir * increment > 0.0f) {
    float maxAdvance = imuAngle + dir * maxLeadDeg;
    if (dir * (nextPhysical - maxAdvance) > 0.0f) {
      nextPhysical = (dir * (maxAdvance - physicalAngle) > 0.0f) ? maxAdvance : physicalAngle;
      // Keep the ramp's state honest, or it resumes at the windup speed once the arm catches up.
      appliedSpeedDegPerSec = (nextPhysical - physicalAngle) / dtSec;
      leadLimited = true;
    }
  }

  // Fine-correction budget: below minProgressDeg, physicalAngle may move at most
  // fineCorrectionBudgetDeg from where it was when the error first got that small. Confirmed live
  // (2026-10-31): with the error parked just outside the dead zone (0.2-0.5deg - a bias, or a joint
  // that doesn't answer sub-degree commands), the gradient kept integrating ~0.15deg/s for minutes -
  // shoulder Z's command crept 133->127.7 and wrist Y's 132->129 with both IMU readings flat, and the
  // stall check never fires there because it skips errors under minProgressDeg. Left alone, that
  // windup releases as one sudden move. Only clamps within the band, so it can't create motion.
  // The anchor is released only after absError has stayed >= minProgressDeg for fineReleaseUs:
  // releasing it the moment the error crossed 2deg (first version) let shoulder Z's +/-2.5deg yaw
  // noise re-arm the budget over and over, ratcheting the command 135->177 in ~8deg steps (2026-10-31).
  if (absError < minProgressDeg) {
    coarseSince = 0;
    if (isnan(fineAnchorPhysical)) {
      fineAnchorPhysical = physicalAngle;
    }
  } else if (coarseSince == 0) {
    coarseSince = now;
  } else if (absolute_time_diff_us(coarseSince, now) >= fineReleaseUs) {
    fineAnchorPhysical = NAN;
  }
  bool fineLimited = false;
  if (!isnan(fineAnchorPhysical)) {
    float clamped = std::clamp(nextPhysical, fineAnchorPhysical - fineCorrectionBudgetDeg,
                               fineAnchorPhysical + fineCorrectionBudgetDeg);
    if (clamped != nextPhysical) {
      nextPhysical = clamped;
      appliedSpeedDegPerSec = (nextPhysical - physicalAngle) / dtSec;
      fineLimited = true;
    }
  }

  printer.interval([&]() {
    LogQueue::Log("Tick: Target: %.2f, IMU: %.2f, Physical: %.2f, Error: %.2f, DesiredSpeed: %.2f, AppliedSpeed: %.2f, LeadLimited: %d, FineLimited: %d\n",
                  targetAngle, imuAngle, physicalAngle, error, desiredSpeedDegPerSec, appliedSpeedDegPerSec, leadLimited, fineLimited);
  });

  bool pinnedAtLimit = (nextPhysical <= safeMinDegree && dir < 0.0f) || (nextPhysical >= safeMaxDegree && dir > 0.0f);
  if (pinnedAtLimit) {
    if (limitPinnedSince == 0) {
      limitPinnedSince = now;
    } else if (absolute_time_diff_us(limitPinnedSince, now) >= limitPinnedMaxUs) {
      diverging = true;
      LogQueue::Log("Servo diverging: pinned at limit (%.2f) with error %.2f, freezing\n",
                    nextPhysical <= safeMinDegree ? safeMinDegree : safeMaxDegree, absError);
      physicalAngle = std::clamp(nextPhysical, safeMinDegree, safeMaxDegree);
      setDegreeDirect(physicalAngle);
      return;
    }
  } else {
    limitPinnedSince = 0;
  }

  physicalAngle = std::clamp(nextPhysical, safeMinDegree, safeMaxDegree);

  setDegreeDirect(physicalAngle);
}

void Servo::setIMUAngle(float value) {
  // Propagate NaN rather than keeping a stale value - tick()'s NaN guard must see it and freeze.
  if (isnan(value)) {
    imuAngle = value;
    return;
  }

  // Low-pass filtered, not raw - sensor noise otherwise feeds straight into tick()'s speed calc.
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