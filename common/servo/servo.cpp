#include "servo.h"

Servo::Servo(const uint pin, Range degreeRange, const float homePosition,
             const float freq, const float lowPeriod, const float highPeriod)
    : minDegree(degreeRange.from), maxDegree(degreeRange.to),
      lowPeriod(lowPeriod), highPeriod(highPeriod), homePosition(homePosition) {
  gpio_set_function(pin, GPIO_FUNC_PWM);
  slice = pwm_gpio_to_slice_num(pin);
  channel = pwm_gpio_to_channel(pin);
  setFrequency(freq);
}

uint Servo::setFrequency(const uint freq) {
  uint32_t clock = clock_get_hz(clk_sys);
  uint32_t clock_div = 1;

  if ((freq < 8) && (freq > clock)) {
    return -1;
  }

  for (clk_divider = 1; clk_divider < UINT8_MAX; clk_divider++) {
    clock_div = div_u32u32(clock, clk_divider);
    wrap = div_u32u32(clock_div, freq);

    if (div_u32u32(clock_div, UINT16_MAX) <= freq && wrap <= UINT16_MAX) {
      break;
    }
  }

  if (clk_divider == UINT8_MAX) {
    return -2;
  }

  period = 1.0f / ((float)clock_div / (float)wrap);
  lowSlices = getSlices(lowPeriod);
  highSlices = getSlices(highPeriod);
  delta = (highSlices - lowSlices);
  pulseStep = (float)delta / maxDegree;

  pwm_set_clkdiv_int_frac(slice, clk_divider, 0);
  pwm_set_wrap(slice, wrap);
  pwm_set_enabled(slice, true);

  return 1;
}

uint16_t Servo::getSlices(const float targetPeriod) {
  return (uint16_t)((targetPeriod / period) * wrap);
}

uint Servo::setDegreeDirect(const float degree) {
  if (isnan(degree)) {
    return SERVO_DEGREE_IS_NAN;
  }
  if (degree < minDegree) {
    return SERVO_DEGREE_IS_BELOW_MINIMUM;
  }
  if (degree > maxDegree) {
    return SERVO_DEGREE_IS_ABOVE_MAXIMUM;
  }

  const uint16_t slices = lowSlices + (uint16_t)(pulseStep * degree);
  pwm_set_chan_level(slice, channel, slices);
  return 0;
}

bool Servo::setTargetAngle(const float angle, uint16_t timeMS, float deadZone) {
  if (isnan(angle)) {
    return false;
  }

  if (angle < minDegree) {
    targetAngle = minDegree;
  } else if (angle > maxDegree) {
    targetAngle = maxDegree;
  } else {
    targetAngle = angle;
  }

  setTimeMS(timeMS);
  setDeadZone(deadZone);
  return true;
}

void Servo::tick() {
  if (isnan(targetAngle) || isnan(imuAngle) || isnan(physicalAngle)) {
    return;
  }

  constexpr float dt = ENGINE_TASK_LOOP_TIMEOUT / 1000.0f;

  const float error = targetAngle - imuAngle;

  // already on target and almost not moving
  if (fabs(error) <= deadZone && fabs(currentAngularSpeed) < 0.5f) {
    return;
  }

  float cmdSpeed =
      Kp * error - Kd * currentAngularSpeed - Ka * currentAcceleration;

  if (cmdSpeed > maxAngularSpeedCmd) {
    cmdSpeed = maxAngularSpeedCmd;
  }
  if (cmdSpeed < -maxAngularSpeedCmd) {
    cmdSpeed = -maxAngularSpeedCmd;
  }

  float stepDeg = cmdSpeed * dt;

  if (stepDeg > maxAngleStepPerTick) {
    stepDeg = maxAngleStepPerTick;
  }
  if (stepDeg < -maxAngleStepPerTick) {
    stepDeg = -maxAngleStepPerTick;
  }

  // do not overshoot target
  if (fabs(stepDeg) > fabs(error)) {
    stepDeg = error;
  }

  physicalAngle += stepDeg;

  if (physicalAngle > maxDegree) {
    physicalAngle = maxDegree;
  }
  if (physicalAngle < minDegree) {
    physicalAngle = minDegree;
  }

  setDegreeDirect(physicalAngle);
}

void Servo::setIMUAngle(float value) {
  imuAngle = value;

  if (isnan(physicalAngle)) {
    physicalAngle = imuAngle;
  }
}

void Servo::setTimeMS(uint16_t timeMS) {
  this->timeMS = (timeMS < 1 || timeMS > 10000) ? this->timeMS : timeMS;

  float diff = fabs(targetAngle - physicalAngle);
  float timeSec = this->timeMS / 1000.0f;

  if (timeSec > 0.0f) {
    maxAngularSpeedCmd = diff / timeSec;

    if (maxAngularSpeedCmd < 10.0f) {
      maxAngularSpeedCmd = 10.0f;
    }
    if (maxAngularSpeedCmd > 300.0f) {
      maxAngularSpeedCmd = 300.0f;
    }
  }
}

void Servo::setDeadZone(float dz) {
  if (dz < 0.0f || dz > 5.0f) {
    return;
  }
  deadZone = dz;
}