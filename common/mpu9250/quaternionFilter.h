#pragma once

#include <cstdint>
#include <iostream>
#include <cstring>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/stdlib.h"
#include <FreeRTOS.h>
#include <task.h>

#include "../config/config.h"

enum class QuatFilterSel
{
  NONE,
  MADGWICK,
  MAHONY,
};

class QuaternionFilter
{
  // for madgwick
  float GyroMeasError = PI * (40.0f / 180.0f);    // gyroscope measurement error in rads/s (start at 40 deg/s)
  float GyroMeasDrift = PI * (0.0f / 180.0f);     // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
  float beta = sqrt(3.0f / 4.0f) * GyroMeasError; // compute beta
  float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift; // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

  // for mahony
  float Kp = 30.0;
  float Ki = 0.0;

  QuatFilterSel filter_sel{QuatFilterSel::MADGWICK};
  double deltaT{0.};
  uint32_t newTime{0}, oldTime{0};

public:
  void select_filter(QuatFilterSel sel);

  void update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float *q);
  void no_filter(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float *q);

  // Madgwick Quaternion Update
  void madgwick(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float *q);

  // Mahony accelleration filter
  // Mahony scheme uses proportional and integral filtering on
  // the error between estimated reference vector (gravity) and measured one.
  // Madgwick's implementation of Mayhony's AHRS algorithm.
  // See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
  // Free parameters in the Mahony filter and fusion scheme,
  // Kp for proportional feedback, Ki for integral
  // float Kp = 30.0;
  // float Ki = 0.0;
  // with MPU-9250, angles start oscillating at Kp=40. Ki does not seem to help and is not required.
  // with MPU-6050, some instability observed at Kp=100 Now set to 30.
  void mahony(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float *q);  
};