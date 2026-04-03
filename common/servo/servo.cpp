#include "servo.h"

Servo::Servo(const uint pin, Range degreeRange, const float homePosition,
             const float freq, const float lowPeriod, const float highPeriod)
    : pin(pin), minDegree(degreeRange.from), maxDegree(degreeRange.to),
      lowPeriod(lowPeriod), highPeriod(highPeriod),
      printer(pdMS_TO_TICKS(100)) {
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
  return true;
}

void Servo::tick() {
  absolute_time_t now = get_absolute_time();
  int64_t dtUs = absolute_time_diff_us(lastTickTime, now);
  lastTickTime = now;
  
  if ((dtUs <= 0) || isnan(imuAngle) || isnan(physicalAngle) || isnan(targetAngle))
    return;

  /*if (!isPositioned()) {
    moveStarted = now;
    setDegreeDirect(imuAngle);
    return;
  }*/


  float dtSec = dtUs / 1000000.0f;
  dtSec = std::min(dtSec, 0.01f);

  const float error = targetAngle - imuAngle;
  const float absError = fabsf(error);

  /*if (absError <= deadZone) {
    physicalAngle = imuAngle;
    return;
  }*/

  const float dir = (error >= 0.0f) ? 1.0f : -1.0f;
  const float currentSpeed = speedBuffer.speed();

  int64_t elapsedUs = absolute_time_diff_us(moveStarted, now);
  int64_t remainingUs = timeUs - elapsedUs;
  remainingUs = std::max<int64_t>(remainingUs, stabilizationTimeUs);

  float timeLeftSec = std::max(remainingUs / 1000000.0f, 0.001f);
  float desiredSpeedDegPerSec = absError / timeLeftSec;  
  float limitedSpeedDegPerSec = fminf(desiredSpeedDegPerSec, maxAngularSpeed);  
  float increment = limitedSpeedDegPerSec * dtSec * dir;
  float nextPhysical = physicalAngle + increment;
   
  physicalAngle = std::clamp(nextPhysical, minDegree, maxDegree);

  /*if (absError > 2.0f) {
    printer.run([this, currentSpeed, remainingUs, now, dtSec]() {
    printf("%llu, Remaining: %ld, Moving, dt: %.4f, Speed: %f, acceleration: %f, imuAngle: %f, "
           "cmd: %f\n",
           (unsigned long long)now, (long)remainingUs, dtSec, currentSpeed,
           speedBuffer.acceleration(), imuAngle, physicalAngle);
    });
  } else {
    printer.run([this, currentSpeed, remainingUs, now, dtSec]() {
      printf("%llu,  Remaining: %ld, Achieving, dt: %.4f, Speed: %f, "
             "acceleration: %f, "
             "imuAngle: %f, cmd: %f\n",
             (unsigned long long)now, (long)remainingUs, dtSec, currentSpeed,
             speedBuffer.acceleration(), imuAngle, physicalAngle);
    });
  }*/

  setDegreeDirect(physicalAngle);
}

void Servo::setIMUAngle(float value) {
  if (isnan(value)) {
    return;
  }

  speedBuffer.push(value);

  if (value < minDegree)
    value = minDegree;
  if (value > maxDegree)
    value = maxDegree;

  imuAngle = value;

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
  if (positionTime == 0) {
    return false;
  }
  auto now = xTaskGetTickCount();
  return (now - positionTime > positionInterval);
}