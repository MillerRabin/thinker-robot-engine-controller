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
  //printf("Set angle angle: %f, timeMS: %d, deadZone: %f\n", angle, timeMS, deadZone);
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

float Servo::getDir(float path) {  
  prevPhysicalAngle = isnan(prevPhysicalAngle) ? physicalAngle : prevPhysicalAngle;
  prevImuAngle = isnan(prevImuAngle) ? imuAngle : prevImuAngle;
  float physDelta = physicalAngle - prevPhysicalAngle;
  float imuDelta = imuAngle - prevImuAngle;
  imuDelta = (fabs(imuDelta) < IMU_DEGREE_CHANGE_IGNORE) ? 0 : imuDelta;
  //printf("isCalibrating: %d, path %f, physicalAngle: %f, prevPhysicalAngle %f, imuAngle: %f, prevImuAngle: %f, physDelta: %f, imuDelta: %f\n",
  //        isCalibrating(), path, physicalAngle, prevPhysicalAngle, imuAngle, prevImuAngle, physDelta, imuDelta);

  return  (path > 0) ? (physDelta >= 0) ? (imuDelta >= 0) ? 1.0f : 1.0f :
                                          (imuDelta >= 0) ? 1.0f : 1.0f :
          (path < 0) ? (physDelta >= 0) ? (imuDelta >= 0) ? -1.0f : -1.0f :
                                          (imuDelta >= 0) ? -1.0f : -1.0f :
          0;      
}

void Servo::tick() {
  if (isnan(targetAngle)) {
    return;
  }

  float ca = physicalAngle;
  bool calibrating = isCalibrating();
  if (calibrating) {
    wasCalibrated = true;
    ca = physicalAngle;
    prevImuAngle = imuAngle;
  }
  else {
    if (wasCalibrated) {
      wasCalibrated = false;
      physicalAngle = imuAngle;
      prevImuAngle = imuAngle;
    }
    ca = imuAngle;
  }

  float path = targetAngle - ca;
  float dir = getDir(path);
  float apath = fabs(path);
  if (!isCalibrating() && isStopped() && apath < deadZone) {
    return;
  }

  float increment = fmin(apath, angleStep);
  increment = fmin(increment, SERVO_MAX_DEGREE_CHANGE);
  //increment = fmax(increment, SERVO_MIN_DEGREE_CHANGE);

  float step = dir * increment;
  
  prevPhysicalAngle = physicalAngle;  
  physicalAngle += step;
  if (physicalAngle > maxDegree)
    physicalAngle = maxDegree;
  if (physicalAngle < minDegree)
    physicalAngle = minDegree;

  //printf("targetAngle: %f, physicalAngle: %f, imuAngle: %f, dir: %f, increment: %f\n", targetAngle, physicalAngle, imuAngle, dir, increment);
  setDegreeDirect(physicalAngle);
}

bool Servo::isStopped() {
  return fabs(physicalAngle - targetAngle) <= deadZone;
}

bool Servo::atHomePosition() {  
  return (targetAngle == homePosition);
}

bool Servo::equalAngles(float a, float b) {
  return fabs(a - b) <= SERVO_MIN_DEGREE_CHANGE;
} 

bool Servo::isCalibrating() {
  return equalAngles(targetAngle, homePosition);
}

void Servo::setIMUAngle(float value) {
  prevImuAngle = imuAngle;
  imuAngle = value;
}

void Servo::setTimeMS(uint16_t timeMS) {  
  this->timeMS = (timeMS < 1 || timeMS > 10000) ? this->timeMS : timeMS;
  float iter = float(timeMS) / ENGINE_TASK_LOOP_DIVIDER;
  float diff = fabs(targetAngle - physicalAngle);  
  this->angleStep = fabs(diff / iter);
}

void Servo::setDeadZone(float dz) {
  if (dz < 0.0f || dz > 5.0f) {
    return;
  }
  deadZone = dz;
}