#include "servo.h"
#include "../tuning/tuning.h"

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

  writePulse(degree);
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

  if (trace != nullptr && traceState == TraceState::Idle && !isnan(targetAngle) &&
      fabsf(angle - targetAngle) >= traceMinMoveDeg) {
    traceFrom = targetAngle;
    traceTo = angle;
    traceTimeMS = newTimeMS;
    traceRequested = true;
  }
  // A plain target (calibration, home, recovery) ends any planned move - planMove() re-arms it - and
  // any stream (streamTo() re-arms it).
  planActive = false;
  streamActive = false;
  streamWaiting = false;
  streamSettlePlan = false;
  gentleHold = false;
  holdSettled = false;
  holdWaking = false;
  planSettling = false;
  planZone = -1;
  targetAngle = angle;
  setDeadZone(newDeadZone);
  setTimeMS(newTimeMS);
  moveStarted = get_absolute_time();
  resetCorrection();
  fineAnchorPhysical = NAN;
  return true;
}

bool Servo::streamTo(const float angle, uint16_t newTimeMS, float newDeadZone) {
  if (isnan(angle) || angle < safeMinDegree || angle > safeMaxDegree) {
    return false;
  }
  if (!streamActive || isnan(imuAngle) || isnan(physicalAngle)) {
    if (!setTargetAngle(angle, newTimeMS, newDeadZone)) {
      return false;
    }
    if (isnan(imuAngle) || isnan(physicalAngle)) {
      return true; // nothing to stream from yet - a plain target
    }
    // The setpoint starts on the first waypoint; the gap from the link fades out (STREAM_ENTRY_SEC).
    streamSetpoint = angle;
    streamEntryOffset = imuAngle - angle;
    streamEntry = streamEntryOffset;
    streamEntrySec = std::max(entrySec, 1.5f * fabsf(streamEntryOffset) / STREAM_ENTRY_MAX_SPEED_DEG_PER_SEC);
    streamStartedAt = get_absolute_time();
    streamErrorLP = 0.0f;
    streamStartIMU = imuAngle;
    streamPeakSpeed = 0.0f;
    streamSpeedLP = 0.0f;
    streamEndWeight = 0.0f;
    streamEndWeightLP = 0.0f;
    streamShift = 0.0f;
    streamCorrection = 0.0f;
    streamCorrectionCapped = false;
    streamCorrectionPeak = 0.0f;
    streamErrorPeak = 0.0f;
    streamPausedSec = 0.0f;
    streamPausedFusionMax = 0.0f;
    streamLeadDir = 0;
    streamLeadFrom = angle;
    streamLeadTarget = 0.0f;
    streamLead = 0.0f;
    streamLeadMissing = NAN;
    streamLeadAhead = 0.0f;
    streamLeadSpeedLP = 0.0f;
    streamLeadScale = 0.0f;
    streamLeadScaleBroke = 1.0f;
    streamLeadScalePeak = 0.0f;
    streamActive = true;
    // Each stream's start is traced, whatever its first step: its waypoints move the target by far
    // less than traceMinMoveDeg each, so setTargetAngle() never would.
    if (trace != nullptr && traceState == TraceState::Idle) {
      traceFrom = imuAngle;
      traceTo = angle;
      traceTimeMS = newTimeMS;
      traceRequested = true;
    }
  } else {
    targetAngle = angle;
    setDeadZone(newDeadZone);
  }
  const float seconds = std::max<uint16_t>(newTimeMS, 20) / 1000.0f;
  streamTarget = angle;
  streamSpeed = std::clamp((angle - streamSetpoint) / seconds, -maxAngularSpeed, maxAngularSpeed);
  streamReachAt = delayed_by_us(get_absolute_time(), (int64_t)(seconds * 1000000.0f));
  return true;
}

bool Servo::streamTick(absolute_time_t now, int64_t dtUs) {
  if (!streamActive) {
    return false;
  }
  const float dtSec = std::min(dtUs / 1000000.0f, 0.01f);
  // What's left of the entry gap, on a smoothstep (no jump in speed at either end).
  float entrySpeed = 0.0f;
  if (streamEntryOffset != 0.0f) {
    const float tau = absolute_time_diff_us(streamStartedAt, now) / (streamEntrySec * 1000000.0f);
    if (tau >= 1.0f) {
      streamEntryOffset = 0.0f;
      streamEntry = 0.0f;
    } else {
      streamEntry = streamEntryOffset * (1.0f - tau * tau * (3.0f - 2.0f * tau));
      entrySpeed = -streamEntryOffset * 6.0f * tau * (1.0f - tau) / streamEntrySec;
    }
  }
  if (absolute_time_diff_us(now, streamReachAt) > 0) {
    streamSetpoint += streamSpeed * dtSec;
    streamLinkSpeed = streamSpeed + entrySpeed;
  } else {
    streamSetpoint = streamTarget;
    streamLinkSpeed = entrySpeed;
    if (absolute_time_diff_us(streamReachAt, now) > (int64_t)STREAM_END_MS * 1000) {
      // No next waypoint: hold the last one like any target (targetAngle is it) - once the link has
      // calmed (see streamWaitTick()). A new waypoint meanwhile starts a new stream.
      LogQueue::Log("Servo pin=%u stream end: travel %.1f, link %.2f past the target with shift %.2f, "
                    "correction %.2f (peak %.2f), error peak %.2f\n", pin, streamTarget - streamStartIMU,
                    imuAngle - streamTarget, streamEndShift, streamCorrection, streamCorrectionPeak,
                    streamErrorPeak);
      if (streamPausedSec > 0.0f) {
        LogQueue::Log("Servo pin=%u stream correction was paused %lu ms: the IMU up to %.1fdeg off its "
                      "accelerometer\n", pin, (unsigned long)(streamPausedSec * 1000.0f), streamPausedFusionMax);
      }
      if (streamLeadDir != 0) {
        const int side = streamLeadDir > 0 ? 1 : 0;
        // The part of the lead a slow setpoint held back at the break-away (its speed scale) was needed
        // all the same: what's learned is the whole lead, not what a slow move takes of it.
        const float missing = streamLeadMissing - std::max(streamLeadAhead - STREAM_LEAD_BREAK_DEG, 0.0f) -
                              streamLeadTarget * (1.0f - streamLeadScaleBroke);
        const bool learn = leadLearn && !isnan(streamLeadMissing) &&
                           fabsf(streamTarget - streamStartIMU) >= STREAM_LEAD_LEARN_MIN_TRAVEL_DEG;
        if (learn) {
          leadDeg[side] = std::clamp(leadDeg[side] + STREAM_LEAD_LEARN_RATE * missing, 0.0f, STREAM_LEAD_MAX_DEG);
        }
        // The speed scale at the break-away - or, the link never having broken away, the most it got to.
        LogQueue::Log("Servo pin=%u stream lead %.2f (x%.2f by speed%s): short %.2f at the break-away, ran %.2f "
                      "past -> %.2f%s\n", pin, streamLeadTarget,
                      isnan(streamLeadMissing) ? streamLeadScalePeak : streamLeadScaleBroke,
                      isnan(streamLeadMissing) ? " at most" : "", streamLeadMissing, streamLeadAhead, leadDeg[side],
                      learn ? "" : " (kept)");
      }
      streamActive = false;
      gentleHold = true;
      resetCorrection();
      streamWaiting = true;
      streamEndedAt = now;
      streamWaitStart = now;
      streamParentWaitLogged = false;
      streamBlockParentBusy = false;
      streamBlockStart = now;
      streamBlockMin = NAN;
      streamBlockMax = NAN;
      streamBlockSum = 0.0f;
      streamBlockCount = 0;
      streamBlocksDone = 0;
      return false;
    }
  }
  // Measured live (2026-11-11, servo traces of 4 s host moves, dither off in the hold): the wrist's
  // command-minus-link offset was 143.6deg while it moved and 139deg once it stopped - moving, the
  // motor drags the link through the joint's friction; braking, the claw's weight takes the link to
  // the other side of the backlash, ~4.5deg ahead (the user: "the arm overshot the point, then came
  // back"). The same every time, so the command ends short by what the link overran last time the
  // same way (streamEndShift, learned by the host), blended in as the setpoint slows from its peak.
  // The host learns it in joint angles: learned here from the absolute angle each module holds, the
  // wrist took the elbow's overrun for its own and its shift flipped sign move to move. The
  // correction aims at the shifted setpoint, and holds still from the moment the setpoint arrives:
  // pulling the command back then only drags the link back after it has rolled over - what's left is
  // the hold's.
  // (A first try, 2026-11-10, faded the correction out over the braking and learned with the dither
  // still on in the hold, which let the claw creep; the shift grew instead of settling.)
  // Low-passed: the host's waypoints come in 0.1deg steps, so a slow setpoint's speed jitters by
  // ~1.7deg/s from one waypoint to the next - too much against the shoulder's ~3deg/s peak.
  streamSpeedLP += (fabsf(streamLinkSpeed) - streamSpeedLP) * std::min(dtSec / STREAM_END_SPEED_TAU_SEC, 1.0f);
  const float speedNow = streamSpeedLP;
  // Only once the setpoint is well on its way: the first waypoint's step from where the link is to the
  // plan's start can be fast and short, and the slow start of the curve after it isn't braking.
  const float setpoint = streamSetpoint + streamEntry;
  // The break-away lead (see STREAM_LEAD_*): the command goes this much further the stream's way.
  float leadSpeed = 0.0f;
  if (leadEnabled && dtSec > 0.0f) {
    // The plan's own speed, not the entry's (see STREAM_LEAD_FULL_SPEED_DEG_PER_SEC).
    streamLeadSpeedLP += (fabsf(streamLinkSpeed - entrySpeed) - streamLeadSpeedLP) *
                         std::min(dtSec / STREAM_LEAD_SPEED_TAU_SEC, 1.0f);
    streamLeadScale = std::min(streamLeadSpeedLP / leadFullSpeed, 1.0f);
    if (streamLeadDir == 0 && fabsf(streamSetpoint - streamLeadFrom) >= STREAM_LEAD_ARM_DEG) {
      streamLeadDir = streamSetpoint > streamLeadFrom ? 1 : -1;
      streamLeadTarget = leadDeg[streamLeadDir > 0 ? 1 : 0];
      streamLeadAt = now;
    }
    if (streamLeadDir != 0) {
      streamLeadScalePeak = std::max(streamLeadScalePeak, streamLeadScale);
      const float tau = std::min(absolute_time_diff_us(streamLeadAt, now) / (STREAM_LEAD_FADE_SEC * 1000000.0f), 1.0f);
      // Out again as the setpoint brakes (the end shift's weight, last tick's): braking, the motor
      // needs less than it took to drive the link, and the lead left in the command is a run past. And
      // only as much as the setpoint's speed asks for: a slow, short correction isn't a move to lead.
      const float lead = streamLeadTarget * tau * tau * (3.0f - 2.0f * tau) * (1.0f - streamEndWeightLP) *
                         streamLeadScale;
      leadSpeed = streamLeadDir * (lead - streamLead) / dtSec;
      streamLead = lead;
      const float along = (imuAngle - streamStartIMU) * streamLeadDir;
      if (isnan(streamLeadMissing) && along >= STREAM_LEAD_BREAK_DEG) {
        // How far the plan had got ahead of a link that tracked from the start, and what the
        // correction had added meanwhile: both are lead it went without.
        streamLeadMissing = (streamSetpoint - streamLeadFrom) * streamLeadDir - STREAM_LEAD_BREAK_DEG +
                            streamCorrection * streamLeadDir;
        streamBrokeAt = now;
        streamLeadScaleBroke = streamLeadScale;
      }
      if (!isnan(streamLeadMissing) &&
          absolute_time_diff_us(streamBrokeAt, now) <= (int64_t)(STREAM_LEAD_AHEAD_SEC * 1000000.0f)) {
        streamLeadAhead = std::max(streamLeadAhead, (imuAngle - setpoint) * streamLeadDir);
      }
    }
  }
  if (fabsf(setpoint - streamStartIMU) >= STREAM_END_ARM_DEG) {
    streamPeakSpeed = std::max(streamPeakSpeed, speedNow);
  }
  if (streamPeakSpeed >= STREAM_END_MIN_PEAK_DEG_PER_SEC) {
    streamEndWeight = std::max(streamEndWeight, 1.0f - speedNow / streamPeakSpeed);
  }
  streamEndWeightLP += (streamEndWeight - streamEndWeightLP) * std::min(dtSec / STREAM_END_SMOOTH_SEC, 1.0f);
  const float shift = -streamEndShift * streamEndWeightLP;
  const float shiftSpeed = dtSec > 0.0f ? (shift - streamShift) / dtSec : 0.0f;
  streamShift = shift;
  // The correction follows the error low-passed: it takes up the slow sag under a changing load but
  // not the link's 2-4 Hz swing through its backlash, which it only pumped (see STREAM_GAIN).
  const float streamError = setpoint + shift - imuAngle;
  if (fabsf(streamError) > fabsf(streamErrorPeak)) {
    streamErrorPeak = streamError;
  }
  streamErrorLP += (streamError - streamErrorLP) * std::min(dtSec / streamErrorTauSec, 1.0f);
  float correctionSpeed = streamGain * streamErrorLP;
  // A joint the plan doesn't move holds its command (see setStreamCorrectWhenStill()), and so does
  // every joint once its setpoint has arrived (see above).
  if ((!streamCorrectWhenStill && fabsf(setpoint - streamStartIMU) < STREAM_STILL_DEG) ||
      absolute_time_diff_us(now, streamReachAt) <= 0) {
    correctionSpeed = 0.0f;
  }
  // Not while the IMU disagrees with its own accelerometer (see STREAM_FUSION_GATE_DEG).
  if (fusionErrorDeg > fusionGateDeg) {
    if (streamPausedSec == 0.0f) {
      LogQueue::Log("Servo pin=%u stream correction paused at %.2f: the IMU %.1fdeg off its accelerometer\n",
                    pin, streamCorrection, fusionErrorDeg);
    }
    correctionSpeed = 0.0f;
    streamPausedSec += std::max(dtSec, 0.001f);
    streamPausedFusionMax = std::max(streamPausedFusionMax, fusionErrorDeg);
  }
  // Bounded: measured live (2026-11-10), the shoulder's BNO came up turning about the wrong axis
  // (see LocalBNO::fusionDiverged()) and read a 5.7deg tilt as ~4; chasing it the correction drove the
  // command 15deg past the plan - the shoulder tilted that much further and the arm dipped - before
  // the fusion check reset the BNO. A real sag needed at most ~6deg (the shoulder at the work pose).
  if (dtSec > 0.0f && fabsf(streamCorrection + correctionSpeed * dtSec) > correctionMaxDeg) {
    const float bound = copysignf(correctionMaxDeg, streamCorrection + correctionSpeed * dtSec);
    correctionSpeed = (bound - streamCorrection) / dtSec;
    if (!streamCorrectionCapped) {
      streamCorrectionCapped = true;
      LogQueue::Log("Servo pin=%u stream correction capped at %.1f: IMU %.2f, setpoint %.2f\n", pin, bound,
                    imuAngle, setpoint);
    }
  }
  streamCorrection += correctionSpeed * dtSec;
  if (fabsf(streamCorrection) > fabsf(streamCorrectionPeak)) {
    streamCorrectionPeak = streamCorrection;
  }
  float speed = streamLinkSpeed + shiftSpeed - parentPlannedSpeed + correctionSpeed + leadSpeed;
  speed = std::clamp(speed, -maxAngularSpeed, maxAngularSpeed);
  appliedSpeedDegPerSec = speed;
  physicalAngle = std::clamp(physicalAngle + speed * dtSec, safeMinDegree, safeMaxDegree);
  setDegreeDirect(physicalAngle);
  return true;
}

bool Servo::streamWaitTick(absolute_time_t now, int64_t dtUs) {
  if (!streamWaiting) {
    return false;
  }
  // The command holds; it only takes out a parent's streamed move, as in the stream, and the parent's
  // settle (the link holds an absolute angle).
  const float dtSec = std::min(dtUs / 1000000.0f, 0.01f);
  const float follow = parentFollow ? parentSettleSpeed : 0.0f;
  const float speed = std::clamp(-(parentPlannedSpeed + follow), -maxAngularSpeed, maxAngularSpeed);
  appliedSpeedDegPerSec = speed;
  if (speed != 0.0f) {
    physicalAngle = std::clamp(physicalAngle + speed * dtSec, safeMinDegree, safeMaxDegree);
    setDegreeDirect(physicalAngle);
  }
  streamBlockMin = isnan(streamBlockMin) ? imuAngle : std::min(streamBlockMin, imuAngle);
  streamBlockMax = isnan(streamBlockMax) ? imuAngle : std::max(streamBlockMax, imuAngle);
  streamBlockSum += targetAngle - imuAngle;
  streamBlockCount++;
  streamBlockParentBusy = streamBlockParentBusy || (parentWait && ancestorsSettling);
  if (absolute_time_diff_us(streamBlockStart, now) < (int64_t)calmBlockMs * 1000) {
    return true;
  }
  // A parent still settling - this block or any part of it - moves this link with it: the calm wait
  // starts over once it's done (see setAncestorsSettling()).
  if (streamBlockParentBusy &&
      absolute_time_diff_us(streamEndedAt, now) < (int64_t)STREAM_PARENT_WAIT_MAX_MS * 1000) {
    if (!streamParentWaitLogged) {
      streamParentWaitLogged = true;
      LogQueue::Log("Servo pin=%u stream end: waiting for a parent to settle\n", pin);
    }
    streamWaitStart = now;
    streamBlocksDone = 0;
    streamBlockStart = now;
    streamBlockMin = NAN;
    streamBlockMax = NAN;
    streamBlockSum = 0.0f;
    streamBlockCount = 0;
    streamBlockParentBusy = false;
    return true;
  }
  const float mean = streamBlockSum / (float)std::max<uint32_t>(streamBlockCount, 1);
  const float span = streamBlockMax - streamBlockMin;
  streamBlockMeans[streamBlocksDone % STREAM_CALM_MEAN_BLOCKS] = mean;
  streamBlocksDone++;
  const unsigned long waitedMs = (unsigned long)(absolute_time_diff_us(streamWaitStart, now) / 1000);
  const unsigned long sinceEndMs = (unsigned long)(absolute_time_diff_us(streamEndedAt, now) / 1000);
  if (span <= calmSpanDeg) {
    LogQueue::Log("Servo pin=%u stream end: calm after %lu ms (%lu since the end, span %.2f), %.2fdeg off on average\n",
                  pin, waitedMs, sinceEndMs, span, mean);
    streamWaiting = false;
    streamSettle(now, mean);
    return true;
  }
  if (waitedMs >= calmWaitMaxMs) {
    const uint8_t n = std::min<uint8_t>(streamBlocksDone, STREAM_CALM_MEAN_BLOCKS);
    float sum = 0.0f;
    for (uint8_t i = 0; i < n; i++) {
      sum += streamBlockMeans[i];
    }
    LogQueue::Log("Servo pin=%u stream end: still swinging after %lu ms (span %.2f), %.2fdeg off on average\n",
                  pin, waitedMs, span, sum / n);
    streamWaiting = false;
    streamSettle(now, sum / n);
    return true;
  }
  streamBlockStart = now;
  streamBlockMin = NAN;
  streamBlockMax = NAN;
  streamBlockSum = 0.0f;
  streamBlockCount = 0;
  streamBlockParentBusy = false;
  return true;
}

void Servo::streamSettle(absolute_time_t now, float error) {
  // What's left, the command takes out in one minimum-jerk move (dithered, see ditherTick()).
  // Measured live (2026-11-11): left to the hold's slow correction it went in steps - the servo
  // sticks at ~1deg/s - and wrist Z jumped 1.3deg after seconds of creeping (the user: "make the
  // final approach smooth, without jerks").
  if (!isnan(error) && !isnan(holdWakeDeg) && fabsf(error) <= settleToleranceDeg) {
    // Close enough: a fraction of a degree costs the motor the whole backlash to put right, and
    // the link jumps when it gets there - measured live (2026-11-11): the shoulder 0.5deg behind,
    // its hold crept the command 1.9deg on for 3 s and the link then jumped 0.8deg, the whole arm
    // with it. The hold's hysteresis still wakes it for a real error.
    holdSettled = true;
    holdWaking = false;
    gentleHold = false;
    LogQueue::Log("Servo pin=%u stream end: %.2fdeg off, left\n", pin, error);
  } else if (!isnan(error) && fabsf(error) > deadZone) {
    planZone = -1;
    planStart = physicalAngle;
    planEnd = std::clamp(physicalAngle + error, safeMinDegree, safeMaxDegree);
    const int64_t minDurationUs =
        (int64_t)(1.875f * fabsf(planEnd - planStart) / settlePlanSpeed * 1000000.0f);
    planDurationUs = std::max((int64_t)STREAM_SETTLE_PLAN_MS * 1000, minDurationUs);
    planStarted = now;
    streamSettlePlan = true;
    planActive = true;
    LogQueue::Log("Servo pin=%u stream end: settling %.2fdeg in %lu ms\n", pin, error,
                  (unsigned long)(planDurationUs / 1000));
  }
}

float Servo::tune(uint8_t param, bool set, float value) {
  const bool apply = set && !isnan(value);
  auto clampTo = [value](float lo, float hi) { return std::min(std::max(value, lo), hi); };
  switch (param) {
  case TUNE_DAMPING_GAIN:
    // Not past the shoulder's 0.1 s: 0.08 on the elbow and the wrist with a faster rate swung the whole
    // arm violently (2026-11-12).
    if (apply) dampingGainSec = clampTo(0.0f, 0.1f);
    return dampingGainSec;
  case TUNE_DAMPING_MAX:
    if (apply) dampingMaxDeg = clampTo(0.0f, 3.0f);
    return dampingMaxDeg;
  case TUNE_DAMPING_DEADBAND:
    if (apply) dampingDeadbandDegPerSec = clampTo(0.0f, 30.0f);
    return dampingDeadbandDegPerSec;
  case TUNE_DITHER_DEG:
    if (apply) ditherAmpDeg = clampTo(0.0f, 1.0f);
    return ditherAmpDeg;
  case TUNE_DITHER_HALF_PERIOD_MS:
    if (apply) ditherHalfPeriodUs = (int64_t)(clampTo(5.0f, 100.0f) * 1000.0f);
    return ditherHalfPeriodUs / 1000.0f;
  case TUNE_STREAM_GAIN:
    if (apply) streamGain = clampTo(0.0f, 3.0f);
    return streamGain;
  case TUNE_STREAM_ERROR_TAU:
    if (apply) streamErrorTauSec = clampTo(0.05f, 2.0f);
    return streamErrorTauSec;
  case TUNE_SETTLE_TOLERANCE:
    if (apply) settleToleranceDeg = clampTo(0.0f, 5.0f);
    return settleToleranceDeg;
  case TUNE_SETTLE_SPEED:
    if (apply) settlePlanSpeed = clampTo(0.5f, 10.0f);
    return settlePlanSpeed;
  case TUNE_CALM_SPAN:
    if (apply) calmSpanDeg = clampTo(0.05f, 5.0f);
    return calmSpanDeg;
  case TUNE_CALM_BLOCK_MS:
    if (apply) calmBlockMs = (uint32_t)clampTo(100.0f, 2000.0f);
    return (float)calmBlockMs;
  case TUNE_CALM_WAIT_MAX_MS:
    if (apply) calmWaitMaxMs = (uint32_t)clampTo(0.0f, 10000.0f);
    return (float)calmWaitMaxMs;
  case TUNE_HOLD_WAKE_DEG:
    if (apply) holdWakeDeg = clampTo(0.1f, 10.0f);
    return holdWakeDeg;
  case TUNE_HOLD_WAKE_MS:
    if (apply) holdWakeUs = (int64_t)(clampTo(0.0f, 5000.0f) * 1000.0f);
    return holdWakeUs / 1000.0f;
  case TUNE_HOLD_WAKE_NOW_DEG:
    if (apply) holdWakeNowDeg = clampTo(0.5f, 20.0f);
    return holdWakeNowDeg;
  case TUNE_PARENT_WAIT:
    if (apply) parentWait = value >= 0.5f;
    return parentWait ? 1.0f : 0.0f;
  case TUNE_PARENT_FOLLOW:
    if (apply) parentFollow = value >= 0.5f;
    return parentFollow ? 1.0f : 0.0f;
  case TUNE_ENTRY_SEC:
    if (apply) entrySec = clampTo(0.06f, 2.0f);
    return entrySec;
  case TUNE_CORRECTION_MAX:
    if (apply) correctionMaxDeg = clampTo(0.0f, STREAM_CORRECTION_MAX_DEG);
    return correctionMaxDeg;
  case TUNE_FUSION_GATE:
    if (apply) fusionGateDeg = clampTo(2.0f, 30.0f);
    return fusionGateDeg;
  case TUNE_LEAD_SMALLER:
  case TUNE_LEAD_LARGER: {
    if (!leadEnabled) return NAN;
    float &lead = leadDeg[param == TUNE_LEAD_LARGER ? 1 : 0];
    // A stream under way keeps the lead it took (streamLeadTarget); this one is for the next.
    if (apply) lead = clampTo(0.0f, STREAM_LEAD_MAX_DEG);
    return lead;
  }
  case TUNE_LEAD_LEARN:
    if (!leadEnabled) return NAN;
    if (apply) leadLearn = value >= 0.5f;
    return leadLearn ? 1.0f : 0.0f;
  case TUNE_LEAD_FULL_SPEED:
    if (!leadEnabled) return NAN;
    if (apply) leadFullSpeed = clampTo(0.1f, 50.0f);
    return leadFullSpeed;
  default:
    return NAN;
  }
}

bool Servo::planMove(const float angle, uint16_t newTimeMS, float newDeadZone) {
  if (isnan(imuAngle) || isnan(physicalAngle) || isnan(targetAngle)) {
    planActive = false;
    return setTargetAngle(angle, newTimeMS, newDeadZone);
  }
  // Keep the command-vs-IMU offset the joint has now (backlash side, gravity sag) - mid-plan that's
  // the running plan's own end-vs-target offset, since the IMU lags the command while moving.
  // Read before setTargetAngle(), which clears planActive/planSettling.
  // Settling counts as mid-plan too: the joint is still on its way to planEnd.
  float offset = (planActive || planSettling) ? planEnd - targetAngle : physicalAngle - imuAngle;
  // A learned offset for the zone the target is in wins - the current one belongs to where the
  // joint is now, and loaded the other way it settles on the other side of its backlash.
  const int8_t zone = zoneOf(angle, imuAngle);
  if (zone >= 0 && !isnan(zoneOffset[zone])) {
    offset = -zoneOffset[zone];
  }
  planActive = false;
  if (!setTargetAngle(angle, newTimeMS, newDeadZone)) {
    return false;
  }
  planZone = zone;
  planStart = physicalAngle;
  planEnd = std::clamp(angle + offset, safeMinDegree, safeMaxDegree);
  // A minimum-jerk curve peaks at 1.875x its average speed - stretch it to stay under maxAngularSpeed.
  const int64_t minDurationUs = (int64_t)(1.875f * fabsf(planEnd - planStart) / maxAngularSpeed * 1000000.0f);
  planDurationUs = std::max(timeUs, minDurationUs);
  planStarted = get_absolute_time();
  planActive = true;
  return true;
}

// Measured live (2026-11-02, servo trace): the IMU-error speed law ramped the command to ~20deg/s
// at once, the link sat 160-350 ms in the gear backlash, then got yanked, overshot the command by
// 2-4.5deg - and that overshoot cut the command from ~20 to ~1deg/s almost instantly, leaving the
// link ringing in the backlash for seconds (elbow ~1.3-2 Hz). A minimum-jerk curve starts and stops
// with zero speed and acceleration, so the backlash is taken up slowly and nothing stops abruptly.
// Returns false once the plan is over (or there is none) - tick() then trims by IMU as before.
bool Servo::planTick(absolute_time_t now, int64_t dtUs) {
  if (!planActive) {
    return false;
  }
  const float dtSec = std::min(dtUs / 1000000.0f, 0.01f);
  const float tau = absolute_time_diff_us(planStarted, now) / (float)planDurationUs;
  if (tau >= 1.0f) {
    planActive = false;
    appliedSpeedDegPerSec = 0.0f;
    physicalAngle = planEnd;
    setDegreeDirect(physicalAngle);
    if (planSettleMaxUs > 0) {
      planSettling = true;
      planEndedAt = now;
      settleRefIMU = imuAngle;
      settleRefTime = now;
    }
    return true;
  }
  const float s = tau * tau * tau * (10.0f + tau * (-15.0f + 6.0f * tau));
  float nextPhysical = planStart + (planEnd - planStart) * s;
  // Same opt-in hold-back as tick()'s - and the curve's clock waits while it holds, so the command
  // resumes smoothly instead of jumping to where the curve has got to meanwhile.
  const float dir = (planEnd >= planStart) ? 1.0f : -1.0f;
  if (!isnan(maxLeadDeg) && dir * (nextPhysical - (imuAngle + dir * maxLeadDeg)) > 0.0f) {
    nextPhysical = physicalAngle;
    planStarted = delayed_by_us(planStarted, dtUs);
  }
  appliedSpeedDegPerSec = (nextPhysical - physicalAngle) / dtSec;
  physicalAngle = std::clamp(nextPhysical, safeMinDegree, safeMaxDegree);
  setDegreeDirect(physicalAngle);
  return true;
}

// Measured live (2026-11-02): the elbow servo dropped to 8-15deg/s, so a 1 s plan ended 10-15deg
// ahead of the joint; the trim saw that as a real error, drove the command another 5-6deg past the
// plan, and the joint overshot the target by 11-12deg and swung for seconds. Holding the command
// until the joint stops leaves the trim only the true residual. Returns true while holding.
bool Servo::settleTick(absolute_time_t now) {
  if (!planSettling) {
    return false;
  }
  if (absolute_time_diff_us(planEndedAt, now) >= planSettleMaxUs) {
    planSettling = false;
    return false;
  }
  if (absolute_time_diff_us(settleRefTime, now) >= settleCheckUs) {
    if (fabsf(imuAngle - settleRefIMU) < settleMotionDeg) {
      planSettling = false;
      return false;
    }
    settleRefIMU = imuAngle;
    settleRefTime = now;
  }
  return true;
}

int8_t Servo::zoneOf(float target, float from) const {
  if (isnan(gravityNeutral) || isnan(target) || isnan(from)) {
    return -1;
  }
  if (target < gravityNeutral - neutralBandDeg) {
    return 0;
  }
  if (target > gravityNeutral + neutralBandDeg) {
    return 3;
  }
  return (target >= from) ? 1 : 2;
}

void Servo::resetCorrection() {
  appliedSpeedDegPerSec = 0.0f;
  coarseSince = 0;
}

void Servo::writePulse(float degree) {
  degree = std::clamp(degree, safeMinDegree, safeMaxDegree);
  uint16_t slices =
      lowSlices + (uint16_t)roundf((degree - minDegree) * pulseStep);
  pwm_set_chan_level(slice, channel, slices);
}

void Servo::tick() {
  tickCommand();
  ditherTick();
  applyDamping();
}

void Servo::ditherTick() {
  const bool allowed = ditherAllowed;
  ditherAllowed = false;
  const absolute_time_t now = get_absolute_time();
  const float dtSec = std::clamp(absolute_time_diff_us(ditherTickAt, now) / 1000000.0f, 0.0f, 0.01f);
  ditherTickAt = now;
  if (ditherAmpDeg <= 0.0f || ditherHalfPeriodUs <= 0 || !allowed || !outputActive || !initialized ||
      isnan(physicalAngle)) {
    ditherLevel = 0.0f;
    ditherOffset = 0.0f;
    return;
  }
  // Moving: a stream's setpoint or a planned move's curve travels. Not a hold correcting, nor a plan
  // settling: measured live (2026-11-10), dithered through the slow correction after a host move the
  // wrist link crept 1deg further along gravity in 2 s while its command walked back 3.6deg - the
  // shaking let the claw's weight slide it across the backlash.
  // The command's own speed counts too: a joint the plan doesn't move can still have its command
  // turned by the stream's correction - wrist Z taking up the claw's twist (2026-11-12).
  const bool moving = (streamActive && (fabsf(streamLinkSpeed) > DITHER_MIN_SPEED_DEG_PER_SEC ||
                                        fabsf(appliedSpeedDegPerSec) > DITHER_MIN_SPEED_DEG_PER_SEC)) ||
                      (streamWaiting && fabsf(appliedSpeedDegPerSec) > DITHER_MIN_SPEED_DEG_PER_SEC) ||
                      planActive;
  if (moving) {
    ditherMotionAt = now;
  }
  const bool on = absolute_time_diff_us(ditherMotionAt, now) < (int64_t)DITHER_HOLD_MS * 1000;
  const float step = dtSec / DITHER_RAMP_SEC;
  ditherLevel = std::clamp(ditherLevel + (on ? step : -step), 0.0f, 1.0f);
  if (ditherLevel <= 0.0f) {
    ditherOffset = 0.0f;
    return;
  }
  if (absolute_time_diff_us(ditherFlipAt, now) >= 0) {
    ditherSign = -ditherSign;
    ditherFlipAt = delayed_by_us(now, ditherHalfPeriodUs);
  }
  ditherOffset = ditherSign * ditherAmpDeg * ditherLevel;
}

// After tickCommand(), whichever way it returned, so the offset follows the swing on every tick -
// in the dead zone and at rest too, where the command itself doesn't move.
void Servo::applyDamping() {
  const float rate = linkRate;
  linkRate = NAN;
  const bool live = outputActive && initialized && !isnan(physicalAngle);
  if (dampingGainSec <= 0.0f || !live || isnan(rate)) {
    dampingOffset = 0.0f;
  } else {
    // The rate the link should have: the command's during a commanded move (a plan, or a plain
    // target's own timeMS), otherwise none - the IMU-held joints hold an absolute angle, so a parent's
    // move, a platform tilt or a swing are all motion to resist (the hold's own correction speed is
    // small and isn't one). The deadband keeps a small limit cycle from closing the loop on itself -
    // see setDamping().
    const bool moving = planActive || absolute_time_diff_us(moveStarted, get_absolute_time()) < timeUs;
    // Streaming, the link should move at the setpoint's speed - the command's also cancels the parent's.
    const float expected = streamActive ? streamLinkSpeed : (moving ? appliedSpeedDegPerSec : 0.0f);
    const float error = rate - expected;
    const float beyond = std::max(fabsf(error) - dampingDeadbandDegPerSec, 0.0f);
    dampingOffset = std::clamp(-dampingGainSec * copysignf(beyond, error), -dampingMaxDeg, dampingMaxDeg);
  }
  // tickCommand() wrote the bare command when it moved it; put the offsets on top, and take them off
  // once they're gone.
  const float offset = live ? dampingOffset + ditherOffset : 0.0f;
  if (live && (offset != 0.0f || pulseOffset != 0.0f)) {
    writePulse(physicalAngle + offset);
  }
  pulseOffset = offset;
}

void Servo::tickCommand() {
  if (!pulseStepLogged) {
    pulseStepLogged = true;
    LogQueue::Log("[DIAG] Servo pin=%u pulseStep=%.3f slices/deg (min PWM step ~%.4f deg)\n",
                  pin, pulseStep, 1.0f / pulseStep);
  }

  absolute_time_t now = get_absolute_time();
  int64_t dtUs = absolute_time_diff_us(lastTickTime, now);
  lastTickTime = now;
  traceTick(now);

  if ((dtUs <= 0) || isnan(imuAngle) || isnan(physicalAngle) || isnan(targetAngle))
    return;

  if (streamTick(now, dtUs) || streamWaitTick(now, dtUs) || planTick(now, dtUs) || settleTick(now))
    return;

  const float error = targetAngle - imuAngle;
  const float absError = fabsf(error);
  // A stream's settle (see streamSettle()) has run and the link has stopped: what it left within
  // settleToleranceDeg stays, as at a stream's own end. Measured live (2026-11-21, elbow, two days of
  // logs): after a settle the gentle hold went on trimming a 0.5-0.6deg residual against a link stuck
  // in its gear's friction and crept the command 1-4deg on while the link moved under 0.1deg (never
  // after a stream left within tolerance); the next lift started with that much of its backlash taken
  // up, took its lead on top and broke away 2.6-5.8deg past its setpoint (6 lifts after a settle; the
  // 16 after none 0-1.5deg). Only where the link's stop is waited for (setPlanSettle()): right at a
  // plan's end the link can still be on its way.
  if (streamSettlePlan) {
    streamSettlePlan = false;
    if (planSettleMaxUs > 0 && !isnan(holdWakeDeg) && absError <= settleToleranceDeg) {
      holdSettled = true;
      holdWaking = false;
      gentleHold = false;
      LogQueue::Log("Servo pin=%u stream end: settled, %.2fdeg off, left\n", pin, error);
      resetCorrection();
      return;
    }
  }
  // Hold still within the dead zone, otherwise sensor noise keeps producing a nonzero error forever.
  // Only reached once a plan and its settle wait are over, so the joint has stopped - a swing
  // passing through the target can't be learned as its resting offset.
  if (planZone >= 0 && absError <= std::max(deadZone, learnToleranceDeg)) {
    zoneOffset[planZone] = imuAngle - physicalAngle;
    LogQueue::Log("Servo pin=%u learned zone %d offset %.2f (target %.2f)\n", pin, planZone,
                  zoneOffset[planZone], targetAngle);
    planZone = -1;
  }
  // Measured live (2026-11-06): after a knock on the table all three IMU-held joints hunted for ~20 s -
  // the error flipped sign at 0.3-1.5deg and the commands chased it by +-1-1.5deg, because the dead
  // zones (0.1-0.3deg) sit at the IMU's own post-knock jitter and the backlash makes every correction
  // jump past the target. See setHoldHysteresis().
  if (holdSettled && !isnan(holdWakeDeg)) {
    if (absError <= holdWakeDeg) {
      holdWaking = false;
      resetCorrection();
      return;
    }
    if (!holdWaking) {
      holdWaking = true;
      holdWakeSince = now;
    }
    // A steadily growing error - a parent's move - doesn't wait (it would be ~9deg off after 0.4 s
    // behind a 20deg/s shoulder); the knocks' tremor stayed under 1.6deg.
    if (absolute_time_diff_us(holdWakeSince, now) < holdWakeUs && !(absError > holdWakeNowDeg)) {
      resetCorrection();
      return;
    }
    holdSettled = false;
    holdWaking = false;
    LogQueue::Log("Servo pin=%u hold woke: error %.2f\n", pin, error);
  }
  if (absError <= deadZone) {
    holdSettled = true;
    gentleHold = false;
    resetCorrection();
    return;
  }
  // At a safe limit with the error still pushing outward, wait there and resume once the error turns
  // back. Every joint holds its own angle by its own IMU, so a parent's or the platform's tilt can put
  // its target past its range - the shoulder on a tilted platform (2026-11-03). A pinned-at-limit check
  // used to freeze the joint after 1.8 s and send it into a recalibration.
  if ((physicalAngle <= safeMinDegree && error < 0.0f) ||
      (physicalAngle >= safeMaxDegree && error > 0.0f)) {
    printer.interval([&]() {
      LogQueue::Log("Servo pin=%u held at limit %.2f, IMU %.2f, error %.2f\n", pin, physicalAngle,
                    imuAngle, error);
    });
    resetCorrection();
    return;
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
  if (gentleHold) {
    // After a stream the motor may have backlash to cross before it reaches the link - slowly, or it
    // yanks the link when it gets there (see STREAM_SETTLE_SPEED_DEG_PER_SEC).
    speedCeiling = std::min(speedCeiling, STREAM_SETTLE_SPEED_DEG_PER_SEC);
  }
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
  // worst stop the advance, not chase a predicted position. Only for a joint on a base that doesn't
  // tilt - see the shoulder's constructor for why it no longer uses this.
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
  // shoulder Z's command crept 133->127.7 and wrist Y's 132->129 with both IMU readings flat. Left
  // alone, that windup releases as one sudden move. Only clamps within the band, so it can't create
  // motion.
  // The anchor is released only after absError has stayed >= minProgressDeg for fineReleaseUs:
  // releasing it the moment the error crossed 2deg (first version) let shoulder Z's +/-2.5deg yaw
  // noise re-arm the budget over and over, ratcheting the command 135->177 in ~8deg steps (2026-10-31).
  if (absError < minProgressDeg) {
    coarseSince = 0;
    if (isnan(fineAnchorPhysical)) {
      fineAnchorPhysical = physicalAngle;
    }
  } else if (!isnan(fineReleaseErrorDeg) && absError >= fineReleaseErrorDeg) {
    // A real disturbance, not noise or a bias - see setFineReleaseError().
    fineAnchorPhysical = NAN;
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

  physicalAngle = std::clamp(nextPhysical, safeMinDegree, safeMaxDegree);

  setDegreeDirect(physicalAngle);
}

bool Servo::enableTrace() {
  if (trace == nullptr) {
    trace = static_cast<TraceSample *>(pvPortMalloc(sizeof(TraceSample) * traceCapacity));
  }
  return trace != nullptr;
}

// Records at the top of tick(), before its early returns, so dead-zone and frozen ticks show too.
// physicalAngle there is the command currently on the PWM output.
void Servo::traceTick(absolute_time_t now) {
  if (trace == nullptr) {
    return;
  }
  if (traceState == TraceState::Idle && traceRequested) {
    traceRequested = false;
    traceCount = 0;
    traceSkip = 0;
    traceStart = now;
    traceState = TraceState::Recording;
  }
  if (traceState == TraceState::Recording) {
    if (traceSkip++ % traceEvery != 0) {
      return;
    }
    // While streaming, the target column is the stream's setpoint (the waypoints show as its bends).
    trace[traceCount++] = {(uint32_t)absolute_time_diff_us(traceStart, now),
                           streamActive ? streamSetpoint + streamEntry : targetAngle, rawIMUAngle, imuAngle, physicalAngle,
                           traceAux};
    if (traceCount == traceCapacity) {
      traceDumped = 0;
      traceState = TraceState::Dumping;
      LogQueue::Log("TR%u start: target %.2f -> %.2f in %u ms; t ms, target, raw imu, imu, command, aux\n",
                    pin, traceFrom, traceTo, traceTimeMS);
    }
    return;
  }
  if (traceState == TraceState::Dumping) {
    char line[LOG_QUEUE_MSG_SIZE - 2];
    int len = snprintf(line, sizeof(line), "TR%u", pin);
    for (int i = 0; i < 2 && traceDumped < traceCount; i++, traceDumped++) {
      const TraceSample &s = trace[traceDumped];
      len += snprintf(line + len, sizeof(line) - len, " %.1f %.2f %.2f %.2f %.2f %.2f;", s.tUs / 1000.0f,
                      s.target, s.rawIMU, s.imu, s.physical, s.aux);
    }
    LogQueue::Log("%s\n", line);
    if (traceDumped >= traceCount) {
      LogQueue::Log("TR%u end\n", pin);
      traceState = TraceState::Idle;
    }
  }
}

void Servo::setIMUAngle(float value) {
  rawIMUAngle = value;
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