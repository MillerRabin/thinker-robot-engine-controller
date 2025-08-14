#include "servo.h"

Servo::Servo(
  const uint pin, 
  Range degreeRange,
  const float homePosition,
  const float freq, 
  const float lowPeriod, 
  const float highPeriod
) :
  minDegree(degreeRange.from),
  maxDegree(degreeRange.to),
  lowPeriod(lowPeriod),
  highPeriod(highPeriod),
  homePosition(homePosition) {     
    gpio_set_function(pin, GPIO_FUNC_PWM);  
    slice = pwm_gpio_to_slice_num(pin);
    channel = pwm_gpio_to_channel(pin);    
    setFrequency(freq);
}

uint Servo::setFrequency(const uint freq) {            
    uint32_t clock = clock_get_hz(clk_sys);
    uint32_t clock_div = 1;
    if((freq < 8) && (freq > clock))
      return -1;
    
    for(clk_divider = 1; clk_divider < UINT8_MAX; clk_divider++) {        
      clock_div = div_u32u32( clock, clk_divider );
      wrap = div_u32u32(clock_div, freq);
      if (div_u32u32(clock_div, UINT16_MAX) <= freq && wrap <= UINT16_MAX) {
        break;
      }
    }
    if (clk_divider == UINT8_MAX)
      return -2;

    period = 1.0 / (clock_div / wrap);
    lowSlices = getSlices(lowPeriod);
    highSlices = getSlices(highPeriod);
    delta = (highSlices - lowSlices);    
    step = (float)delta / maxDegree;
    pwm_set_clkdiv_int_frac(slice, clk_divider, 0);    
    pwm_set_wrap(slice, wrap);
    pwm_set_enabled(slice, true);    
    return 1;
}

uint16_t Servo::getSlices(const float targetPeriod) {
  return ( targetPeriod / period ) * wrap;
}

uint Servo::setDegreeDirect(const float degree) {
  if (isnan(degree))
      return SERVO_DEGREE_IS_NAN;
  if (degree < minDegree)
    return SERVO_DEGREE_IS_BELOW_MINIMUM;
  if (degree > maxDegree)
    return SERVO_DEGREE_IS_ABOVE_MAXIMUM;  
  const uint16_t slices = lowSlices + (step * degree);
  pwm_set_chan_level(slice, channel, slices);
  return 0;
}

bool Servo::setTargetAngle(const float angle, uint16_t timeMS, float deadZone) {
  if (isnan(angle)) 
    return false;

  if (angle < minDegree) {
    targetAngle = minDegree;
    return true;
  }
  
  if (angle > maxDegree) {
    targetAngle = maxDegree;
    return true;
  }
    
  targetAngle = angle;
  setTimeMS(timeMS);
  setDeadZone(deadZone);
  return true;
}

void Servo::tick() {
  if (isnan(targetAngle)) {
    return;
  }
  float ca = currentAngle;  
  if (!isCalibrating()) {    
    ca = imuAngle;
  }
    
  float path = targetAngle - ca;
  float dir = (path > 0) ? 1.0 : -1.0;
  float apath = fabs(path);
  if (!isCalibrating() && isStopped() && apath < deadZone) {
    return;
  }

  float increment = apath > angleStep ? angleStep : apath;
  increment = increment > SERVO_MAX_DEGREE_CHANGE ? SERVO_MAX_DEGREE_CHANGE :
              increment;
              
  float step = dir * increment;
  currentAngle += step;
  //printf("isCalibrating: %d, targetAngle %f, currentAngle: %f, imuAngle: %f, angleStep: %f, step: %f, path: %f, timeMS: %d\n", isCalibrating(), targetAngle, currentAngle, imuAngle, angleStep, step, path, timeMS);
  setDegreeDirect(currentAngle);
}

bool Servo::isStopped() {
  return fabs(currentAngle - targetAngle) <= SERVO_MIN_DEGREE_CHANGE;
}

bool Servo::atHomePosition() {  
  return (targetAngle == homePosition);
}

bool Servo::equalAngles(float a, float b) {
  return fabs(a - b) <= SERVO_MIN_DEGREE_CHANGE;
} 

bool Servo::isCalibrating() {
  return equalAngles(targetAngle, homePosition) &&
         !equalAngles(currentAngle, imuAngle) &&
         !equalAngles(targetAngle, imuAngle);
}

void Servo::setIMUAngle(float value) {  
  imuAngle = value;
}

void Servo::setTimeMS(uint16_t timeMS) {  
  if (timeMS < 1 || timeMS > 10000) {
    timeMS = 1000;
  }
  this->timeMS = timeMS;
  float iter = float(timeMS) / ENGINE_TASK_LOOP_TIMEOUT;
  float diff = fabs(targetAngle - currentAngle);  
  this->angleStep = diff / iter;  
}

void Servo::setDeadZone(float dz) {
  if (dz < 0.0f || dz > 5.0f) {
    return;
  }
  deadZone = dz;
}