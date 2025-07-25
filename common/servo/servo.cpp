#include "servo.h"

Servo::Servo(const uint pin, Range degreeRange, Range imuRange, ImuUseAngle useAngle, const float freq, const float lowPeriod, const float highPeriod) :
  minDegree(degreeRange.from),
  maxDegree(degreeRange.to),
  lowPeriod(lowPeriod),
  highPeriod(highPeriod),
  useAngle(useAngle),  
  imuMap(imuRange, degreeRange),
  euler(NAN, NAN, NAN) {     
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
  targetAngle = angle;
  return true;
}

float Servo::getImuAngle() {
  const float angle = euler.getAngle(useAngle);
  return imuMap.getDestValue(angle);
}

void Servo::tick() {  
  const float current = getImuAngle();
  const float diff = targetAngle - current;
  printf("current: %f, target: %f, diff: %f\n", current, targetAngle, diff);
  setDegreeDirect(targetAngle);
}