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

bool Servo::setTargetAngle(const float angle) {
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
  return true;
}

void Servo::tick() {
  if (isnan(targetAngle)) {
    return;
  }
  float ca = currentAngle;  
  if (!isCalibrating()) {
    /*filteredImuAngle = (isnan(filteredImuAngle)) ? imuAngle : 
                                IMU_ANGLE_FILTER * imuAngle + (1.0f - IMU_ANGLE_FILTER) * filteredImuAngle;
    ca = filteredImuAngle;*/
    ca = imuAngle;
  }
    
  float path = targetAngle - ca;
  float dir = (path > 0) ? 1.0 : -1.0;
  float apath = fabs(path);
  if (!isCalibrating() && (apath < SERVO_DEAD_ZONE)) {
    return;
  }

  float increment = apath > SERVO_MAX_DEGREE_CHANGE ? SERVO_MAX_DEGREE_CHANGE : 
                    apath < SERVO_MIN_DEGREE_CHANGE ? SERVO_MIN_DEGREE_CHANGE :
                    apath;  
  float step = dir * increment;
  currentAngle += step;  
  setDegreeDirect(currentAngle);
}

bool Servo::isStopped() {
  return fabs(imuAngle - targetAngle) <= SERVO_MIN_DEGREE_CHANGE;
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