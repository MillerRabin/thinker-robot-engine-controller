#pragma once

#include <cstdint>
#include <hardware/i2c.h>
#include <iostream>
#include <cstring>
#include <math.h>

#include "pico/stdlib.h"
#include <FreeRTOS.h>
#include <task.h>

#include "registerMap.h"
#include "quaternionFilter.h"

enum class ACCEL_FS_SEL {
  A2G,
  A4G,
  A8G,
  A16G
};

enum class GYRO_FS_SEL {
  G250DPS,
  G500DPS,
  G1000DPS,
  G2000DPS
};
enum class MAG_OUTPUT_BITS {
  M14BITS,
  M16BITS
};

enum class FIFO_SAMPLE_RATE : uint8_t {
  SMPL_1000HZ,
  SMPL_500HZ,
  SMPL_333HZ,
  SMPL_250HZ,
  SMPL_200HZ,
  SMPL_167HZ,
  SMPL_143HZ,
  SMPL_125HZ,
};

enum class GYRO_DLPF_CFG : uint8_t {
  DLPF_250HZ,
  DLPF_184HZ,
  DLPF_92HZ,
  DLPF_41HZ,
  DLPF_20HZ,
  DLPF_10HZ,
  DLPF_5HZ,
  DLPF_3600HZ,
};

enum class ACCEL_DLPF_CFG : uint8_t {
  DLPF_218HZ_0,
  DLPF_218HZ_1,
  DLPF_99HZ,
  DLPF_45HZ,
  DLPF_21HZ,
  DLPF_10HZ,
  DLPF_5HZ,
  DLPF_420HZ,
};

static constexpr uint8_t MPU9250_WHOAMI_DEFAULT_VALUE{0x71};
static constexpr uint8_t MPU9255_WHOAMI_DEFAULT_VALUE{0x73};
static constexpr uint8_t MPU6500_WHOAMI_DEFAULT_VALUE{0x70};

struct MPU9250Setting {
  ACCEL_FS_SEL accel_fs_sel{ACCEL_FS_SEL::A16G};
  GYRO_FS_SEL gyro_fs_sel{GYRO_FS_SEL::G2000DPS};
  MAG_OUTPUT_BITS mag_output_bits{MAG_OUTPUT_BITS::M16BITS};
  FIFO_SAMPLE_RATE fifo_sample_rate{FIFO_SAMPLE_RATE::SMPL_200HZ};
  uint8_t gyro_fchoice{0x03};
  GYRO_DLPF_CFG gyro_dlpf_cfg{GYRO_DLPF_CFG::DLPF_41HZ};
  uint8_t accel_fchoice{0x01};
  ACCEL_DLPF_CFG accel_dlpf_cfg{ACCEL_DLPF_CFG::DLPF_45HZ};
};

class MPU9250 {
  static constexpr uint8_t MPU9250_DEFAULT_ADDRESS{0x68}; // Device address when ADO = 0
  static constexpr uint8_t AK8963_ADDRESS{0x0C};          //  Address of magnetometer
  static constexpr uint8_t AK8963_WHOAMI_DEFAULT_VALUE{0x48};
  uint8_t mpu_i2c_addr{MPU9250_DEFAULT_ADDRESS};

  // settings
  MPU9250Setting setting;
  // TODO: this should be configured!!
  static constexpr uint8_t MAG_MODE{0x06}; // 0x02 for 8 Hz, 0x06 for 100 Hz continuous magnetometer data read
  float acc_resolution{0.f};               // scale resolutions per LSB for the sensors
  float gyro_resolution{0.f};              // scale resolutions per LSB for the sensors
  float mag_resolution{0.f};               // scale resolutions per LSB for the sensors

  // Calibration Parameters
  float acc_bias[3]{0., 0., 0.};  // acc calibration value in ACCEL_FS_SEL: 2g
  float gyro_bias[3]{0., 0., 0.}; // gyro calibration value in GYRO_FS_SEL: 250dps
  float mag_bias_factory[3]{0., 0., 0.};
  float mag_bias[3]{0., 0., 0.}; // mag calibration value in MAG_OUTPUT_BITS: 16BITS
  float mag_scale[3]{1., 1., 1.};
  float magnetic_declination = -7.51; // Japan, 24th June

  // Temperature
  int16_t temperature_count{0}; // temperature raw count output
  float temperature{0.f};       // Stores the real internal chip temperature in degrees Celsius

  // Self Test
  float self_test_result[6]{0.f}; // holds results of gyro and accelerometer self test
  bool self_test_impl();
  // IMU Data
  float a[3]{0.f, 0.f, 0.f};
  float g[3]{0.f, 0.f, 0.f};
  float m[3]{0.f, 0.f, 0.f};
  float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // vector to hold quaternion
  float rpy[3]{0.f, 0.f, 0.f};
  float lin_acc[3]{0.f, 0.f, 0.f}; // linear acceleration (acceleration with gravity component subtracted)
  QuaternionFilter quat_filter;
  size_t n_filter_iter{1};

  // Other settings
  bool has_connected{false};
  bool b_ahrs{true};
  bool b_verbose{false};

  // I2C
  i2c_inst_t *_i2cPort;
  uint8_t i2c_err_;

public:
  static constexpr uint16_t CALIB_GYRO_SENSITIVITY{131};    // LSB/degrees/sec
  static constexpr uint16_t CALIB_ACCEL_SENSITIVITY{16384}; // LSB/g

  bool setup(const uint8_t addr, const MPU9250Setting &mpu_setting = MPU9250Setting(), i2c_inst_t *_i2cPort = i2c_default);  
  void sleep(bool b);  
  void verbose(const bool b);  
  void ahrs(const bool b);
  void calibrateAccelGyro();  
  void calibrateMag();  
  bool isConnected();  
  bool isConnectedMPU9250();
  bool isConnectedAK8963();  
  bool isSleeping();  
  bool available();  
  bool update();  
  float getRoll() const { return rpy[0]; }
  float getPitch() const { return rpy[1]; }
  float getYaw() const { return rpy[2]; }

  float getEulerX() const { return rpy[0]; }
  float getEulerY() const { return -rpy[1]; }
  float getEulerZ() const { return -rpy[2]; }

  float getQuaternionX() const { return q[1]; }
  float getQuaternionY() const { return q[2]; }
  float getQuaternionZ() const { return q[3]; }
  float getQuaternionW() const { return q[0]; }

  float getAcc(const uint8_t i) const { return (i < 3) ? a[i] : 0.f; }
  float getGyro(const uint8_t i) const { return (i < 3) ? g[i] : 0.f; }
  float getMag(const uint8_t i) const { return (i < 3) ? m[i] : 0.f; }
  float getLinearAcc(const uint8_t i) const { return (i < 3) ? lin_acc[i] : 0.f; }

  float getAccX() const { return a[0]; }
  float getAccY() const { return a[1]; }
  float getAccZ() const { return a[2]; }
  float getGyroX() const { return g[0]; }
  float getGyroY() const { return g[1]; }
  float getGyroZ() const { return g[2]; }
  float getMagX() const { return m[0]; }
  float getMagY() const { return m[1]; }
  float getMagZ() const { return m[2]; }
  float getLinearAccX() const { return lin_acc[0]; }
  float getLinearAccY() const { return lin_acc[1]; }
  float getLinearAccZ() const { return lin_acc[2]; }

  float getAccBias(const uint8_t i) const { return (i < 3) ? acc_bias[i] : 0.f; }
  float getGyroBias(const uint8_t i) const { return (i < 3) ? gyro_bias[i] : 0.f; }
  float getMagBias(const uint8_t i) const { return (i < 3) ? mag_bias[i] : 0.f; }
  float getMagScale(const uint8_t i) const { return (i < 3) ? mag_scale[i] : 0.f; }

  float getAccBiasX() const { return acc_bias[0]; }
  float getAccBiasY() const { return acc_bias[1]; }
  float getAccBiasZ() const { return acc_bias[2]; }
  float getGyroBiasX() const { return gyro_bias[0]; }
  float getGyroBiasY() const { return gyro_bias[1]; }
  float getGyroBiasZ() const { return gyro_bias[2]; }
  float getMagBiasX() const { return mag_bias[0]; }
  float getMagBiasY() const { return mag_bias[1]; }
  float getMagBiasZ() const { return mag_bias[2]; }
  float getMagScaleX() const { return mag_scale[0]; }
  float getMagScaleY() const { return mag_scale[1]; }
  float getMagScaleZ() const { return mag_scale[2]; }

  float getTemperature() const { return temperature; }
  void setAccBias(const float x, const float y, const float z);  
  void setGyroBias(const float x, const float y, const float z);  
  void setMagBias(const float x, const float y, const float z);  
  void setMagScale(const float x, const float y, const float z);  
  void setMagneticDeclination(const float d) { magnetic_declination = d; }
  void selectFilter(QuatFilterSel sel);
  void setFilterIterations(const size_t n);  
  bool selftest();  
  void initMPU9250();  
  void initAK8963();  
  void update_rpy(float qw, float qx, float qy, float qz);
  void update_accel_gyro();  
  void read_accel_gyro(int16_t *destination);
  void update_mag();  
  bool read_mag(int16_t *destination);  
  int16_t read_temperature_data();  

  // Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
  // of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
  // ACCEL_FS_SEL: 2g (maximum sensitivity)
  // GYRO_FS_SEL: 250dps (maximum sensitivity)
  void calibrate_acc_gyro_impl();  
  void set_acc_gyro_to_calibration();  
  void collect_acc_gyro_data_to(float *a_bias, float *g_bias);  
  void write_accel_offset();  
  void write_gyro_offset();
  
  // mag calibration is executed in MAG_OUTPUT_BITS: 16BITS
  void calibrate_mag_impl();
  void collect_mag_data_to(float *m_bias, float *m_scale);  
  float get_acc_resolution(const ACCEL_FS_SEL accel_af_sel) const;
  float get_gyro_resolution(const GYRO_FS_SEL gyro_fs_sel) const;
  float get_mag_resolution(const MAG_OUTPUT_BITS mag_output_bits) const;
  void write_byte(uint8_t address, uint8_t subAddress, uint8_t data);  
  uint8_t read_byte(uint8_t address, uint8_t subAddress);  
  int read_bytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest); 
  void print_i2c_error();  
};