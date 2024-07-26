#include "mpu9250.h"

bool MPU9250::setup(const uint8_t addr, const MPU9250Setting &mpu_setting, i2c_inst_t *_i2cPort)
{
  // addr should be valid for MPU
  if ((addr < MPU9250_DEFAULT_ADDRESS) || (addr > MPU9250_DEFAULT_ADDRESS + 7))
  {
    printf("I2C address 0x%x is not valid for MPU. Please check your I2C address.\n", addr);
    return false;
  }
  mpu_i2c_addr = addr;
  setting = mpu_setting;
  this->_i2cPort = _i2cPort;

  if (isConnectedMPU9250())
  {
    initMPU9250();
    if (isConnectedAK8963())
      initAK8963();
    else
    {
      if (b_verbose)
        printf("Could not connect to AK8963\n");
      has_connected = false;
      return false;
    }
  }
  else
  {
    if (b_verbose)
      printf("Could not connect to MPU9250\n");
    has_connected = false;
    return false;
  }
  has_connected = true;
  return true;
}

void MPU9250::sleep(bool b)
{
  uint8_t c = read_byte(mpu_i2c_addr, PWR_MGMT_1); // read the value, change sleep bit to match b, write uint8_t back to register
  if (b)
  {
    c = c | 0x40; // sets the sleep bit
  }
  else
  {
    c = c & 0xBF; // mask 1011111 keeps all the previous bits
  }
  write_byte(mpu_i2c_addr, PWR_MGMT_1, c);
}

void MPU9250::verbose(const bool b)
{
  b_verbose = b;
}

void MPU9250::ahrs(const bool b)
{
  b_ahrs = b;
}

void MPU9250::calibrateAccelGyro()
{
  calibrate_acc_gyro_impl();
}

void MPU9250::calibrateMag()
{
  calibrate_mag_impl();
}

bool MPU9250::isConnected()
{
  has_connected = isConnectedMPU9250() && isConnectedAK8963();
  return has_connected;
}

bool MPU9250::isConnectedMPU9250()
{
  uint8_t c = read_byte(mpu_i2c_addr, WHO_AM_I_MPU9250);
  if (b_verbose) {
    printf("MPU9250 WHO AM I = 0x%x\n", c);    
  }
  bool b = (c == MPU9250_WHOAMI_DEFAULT_VALUE);
  b |= (c == MPU9255_WHOAMI_DEFAULT_VALUE);
  b |= (c == MPU6500_WHOAMI_DEFAULT_VALUE);
  return b;
}

bool MPU9250::isConnectedAK8963()
{
  uint8_t c = read_byte(AK8963_ADDRESS, AK8963_WHO_AM_I);
  if (b_verbose) {
    printf("AK8963 WHO AM I = 0x%x\n", c);    
  }
  return (c == AK8963_WHOAMI_DEFAULT_VALUE);
}

bool MPU9250::isSleeping() {
  uint8_t c = read_byte(mpu_i2c_addr, PWR_MGMT_1);
  return (c & 0x40) == 0x40;
}

bool MPU9250::available() {
  return has_connected && (read_byte(mpu_i2c_addr, INT_STATUS) & 0x01);
}

bool MPU9250::update() {
  if (!available())
    return false;

  update_accel_gyro();
  update_mag();

  // Madgwick function needs to be fed North, East, and Down direction like
  // (AN, AE, AD, GN, GE, GD, MN, ME, MD)
  // Accel and Gyro direction is Right-Hand, X-Forward, Z-Up
  // Magneto direction is Right-Hand, Y-Forward, Z-Down
  // So to adopt to the general Aircraft coordinate system (Right-Hand, X-Forward, Z-Down),
  // we need to feed (ax, -ay, -az, gx, -gy, -gz, my, -mx, mz)
  // but we pass (-ax, ay, az, gx, -gy, -gz, my, -mx, mz)
  // because gravity is by convention positive down, we need to ivnert the accel data

  // get quaternion based on aircraft coordinate (Right-Hand, X-Forward, Z-Down)
  // acc[mg], gyro[deg/s], mag [mG]
  // gyro will be convert from [deg/s] to [rad/s] inside of this function
  // quat_filter.update(-a[0], a[1], a[2], g[0] * DEG_TO_RAD, -g[1] * DEG_TO_RAD, -g[2] * DEG_TO_RAD, m[1], -m[0], m[2], q);

  float an = -a[0];
  float ae = +a[1];
  float ad = +a[2];
  float gn = +g[0] * DEG_TO_RAD;
  float ge = -g[1] * DEG_TO_RAD;
  float gd = -g[2] * DEG_TO_RAD;
  float mn = +m[1];
  float me = -m[0];
  float md = +m[2];

  for (size_t i = 0; i < n_filter_iter; ++i) {
    quat_filter.update(an, ae, ad, gn, ge, gd, mn, me, md, q);
  }

  if (!b_ahrs) {
    temperature_count = read_temperature_data();              // Read the adc values
    temperature = ((float)temperature_count) / 333.87 + 21.0; // Temperature in degrees Centigrade
  }
  else
  {
    update_rpy(q[0], q[1], q[2], q[3]);
  }
  return true;
}

void MPU9250::setAccBias(const float x, const float y, const float z)
{
  acc_bias[0] = x;
  acc_bias[1] = y;
  acc_bias[2] = z;
  write_accel_offset();
}
void MPU9250::setGyroBias(const float x, const float y, const float z)
{
  gyro_bias[0] = x;
  gyro_bias[1] = y;
  gyro_bias[2] = z;
  write_gyro_offset();
}

void MPU9250::setMagBias(const float x, const float y, const float z)
{
  mag_bias[0] = x;
  mag_bias[1] = y;
  mag_bias[2] = z;
}

void MPU9250::setMagScale(const float x, const float y, const float z)
{
  mag_scale[0] = x;
  mag_scale[1] = y;
  mag_scale[2] = z;
}

void MPU9250::selectFilter(QuatFilterSel sel)
{
  quat_filter.select_filter(sel);
}

void MPU9250::setFilterIterations(const size_t n)
{
  if (n > 0)
    n_filter_iter = n;
}

bool MPU9250::selftest()
{
  return self_test_impl();
}

void MPU9250::initMPU9250()
{
  acc_resolution = get_acc_resolution(setting.accel_fs_sel);
  gyro_resolution = get_gyro_resolution(setting.gyro_fs_sel);
  mag_resolution = get_mag_resolution(setting.mag_output_bits);

  // reset device
  write_byte(mpu_i2c_addr, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  vTaskDelay(pdMS_TO_TICKS(100));
  
  // wake up device
  write_byte(mpu_i2c_addr, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
  vTaskDelay(pdMS_TO_TICKS(100));                                 // Wait for all registers to reset

  // get stable time source
  write_byte(mpu_i2c_addr, PWR_MGMT_1, 0x01); // Auto select clock source to be PLL gyroscope reference if ready else
  vTaskDelay(pdMS_TO_TICKS(200));

  // Configure Gyro and Thermometer
  // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
  // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
  // be higher than 1 / 0.0059 = 170 Hz
  // GYRO_DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
  // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  uint8_t mpu_config = (uint8_t)setting.gyro_dlpf_cfg;
  write_byte(mpu_i2c_addr, MPU_CONFIG, mpu_config);

  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  uint8_t sample_rate = (uint8_t)setting.fifo_sample_rate;
  write_byte(mpu_i2c_addr, SMPLRT_DIV, sample_rate); // Use a 200 Hz rate; a rate consistent with the filter update rate
                                                     // determined inset in CONFIG above

  // Set gyroscope full scale range
  // Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c = read_byte(mpu_i2c_addr, GYRO_CONFIG); // get current GYRO_CONFIG register value
  c = c & ~0xE0;                                    // Clear self-test bits [7:5]
  c = c & ~0x03;                                    // Clear Fchoice bits [1:0]
  c = c & ~0x18;                                    // Clear GYRO_FS_SEL bits [4:3]
  c = c | (uint8_t(setting.gyro_fs_sel) << 3);      // Set full scale range for the gyro
  c = c | (uint8_t(~setting.gyro_fchoice) & 0x03);  // Set Fchoice for the gyro
  write_byte(mpu_i2c_addr, GYRO_CONFIG, c);         // Write new GYRO_CONFIG value to register

  // Set accelerometer full-scale range configuration
  c = read_byte(mpu_i2c_addr, ACCEL_CONFIG);    // get current ACCEL_CONFIG register value
  c = c & ~0xE0;                                // Clear self-test bits [7:5]
  c = c & ~0x18;                                // Clear ACCEL_FS_SEL bits [4:3]
  c = c | (uint8_t(setting.accel_fs_sel) << 3); // Set full scale range for the accelerometer
  write_byte(mpu_i2c_addr, ACCEL_CONFIG, c);    // Write new ACCEL_CONFIG register value

  // Set accelerometer sample rate configuration
  // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
  // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = read_byte(mpu_i2c_addr, ACCEL_CONFIG2);       // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F;                                    // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  c = c | (~(setting.accel_fchoice << 3) & 0x08);   // Set accel_fchoice_b to 1
  c = c | (uint8_t(setting.accel_dlpf_cfg) & 0x07); // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  write_byte(mpu_i2c_addr, ACCEL_CONFIG2, c);       // Write new ACCEL_CONFIG2 register value

  // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
  // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
  // can join the I2C bus and all can be controlled by the Arduino as master
  write_byte(mpu_i2c_addr, INT_PIN_CFG, 0x22);
  write_byte(mpu_i2c_addr, INT_ENABLE, 0x01); // Enable data ready (bit 0) interrupt
  vTaskDelay(pdMS_TO_TICKS(100));
}

void MPU9250::initAK8963()
{
  // First extract the factory calibration for each magnetometer axis
  uint8_t raw_data[3];                           // x/y/z gyro calibration data stored here
  write_byte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
  vTaskDelay(pdMS_TO_TICKS(10));
  write_byte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
  vTaskDelay(pdMS_TO_TICKS(10));
  read_bytes(AK8963_ADDRESS, AK8963_ASAX, 3, &raw_data[0]);     // Read the x-, y-, and z-axis calibration values
  mag_bias_factory[0] = (float)(raw_data[0] - 128) / 256. + 1.; // Return x-axis sensitivity adjustment values, etc.
  mag_bias_factory[1] = (float)(raw_data[1] - 128) / 256. + 1.;
  mag_bias_factory[2] = (float)(raw_data[2] - 128) / 256. + 1.;
  write_byte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
  vTaskDelay(pdMS_TO_TICKS(10));
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition MAG_MODE (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  write_byte(AK8963_ADDRESS, AK8963_CNTL, (uint8_t)setting.mag_output_bits << 4 | MAG_MODE); // Set magnetometer data resolution and sample ODR
  vTaskDelay(pdMS_TO_TICKS(10));

  if (b_verbose) {
    printf("Mag Factory Calibration Values: \n");
    printf("  X-Axis sensitivity offset value: %f\n", mag_bias_factory[0]);
    printf("  Y-Axis sensitivity offset value: %f\n", mag_bias_factory[1]);
    printf("  Z-Axis sensitivity offset value: %f'n", mag_bias_factory[2]);    
  }
}

void MPU9250::update_rpy(float qw, float qx, float qy, float qz)
{
  // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
  // In this coordinate system, the positive z-axis is down toward Earth.
  // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
  // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
  // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
  // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
  // applied in the correct order which for this configuration is yaw, pitch, and then roll.
  // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
  float a12, a22, a31, a32, a33; // rotation matrix coefficients for Euler angles and gravity components
  a12 = 2.0f * (qx * qy + qw * qz);
  a22 = qw * qw + qx * qx - qy * qy - qz * qz;
  a31 = 2.0f * (qw * qx + qy * qz);
  a32 = 2.0f * (qx * qz - qw * qy);
  a33 = qw * qw - qx * qx - qy * qy + qz * qz;
  rpy[0] = atan2f(a31, a33);
  rpy[1] = -asinf(a32);
  rpy[2] = atan2f(a12, a22);
  rpy[0] *= 180.0f / PI;
  rpy[1] *= 180.0f / PI;
  rpy[2] *= 180.0f / PI;
  rpy[2] += magnetic_declination;
  if (rpy[2] >= +180.f)
    rpy[2] -= 360.f;
  else if (rpy[2] < -180.f)
    rpy[2] += 360.f;

  lin_acc[0] = a[0] + a31;
  lin_acc[1] = a[1] + a32;
  lin_acc[2] = a[2] - a33;
}

void MPU9250::update_accel_gyro()
{
  int16_t raw_acc_gyro_data[7];       // used to read all 14 bytes at once from the MPU9250 accel/gyro
  read_accel_gyro(raw_acc_gyro_data); // INT cleared on any read

  // Now we'll calculate the accleration value into actual g's
  a[0] = (float)raw_acc_gyro_data[0] * acc_resolution; // get actual g value, this depends on scale being set
  a[1] = (float)raw_acc_gyro_data[1] * acc_resolution;
  a[2] = (float)raw_acc_gyro_data[2] * acc_resolution;

  temperature_count = raw_acc_gyro_data[3];                 // Read the adc values
  temperature = ((float)temperature_count) / 333.87 + 21.0; // Temperature in degrees Centigrade

  // Calculate the gyro value into actual degrees per second
  g[0] = (float)raw_acc_gyro_data[4] * gyro_resolution; // get actual gyro value, this depends on scale being set
  g[1] = (float)raw_acc_gyro_data[5] * gyro_resolution;
  g[2] = (float)raw_acc_gyro_data[6] * gyro_resolution;
}

void MPU9250::read_accel_gyro(int16_t *destination)
{
  uint8_t raw_data[14];                                                // x/y/z accel register data stored here
  read_bytes(mpu_i2c_addr, ACCEL_XOUT_H, 14, &raw_data[0]);            // Read the 14 raw data registers into data array
  destination[0] = ((int16_t)raw_data[0] << 8) | (int16_t)raw_data[1]; // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)raw_data[2] << 8) | (int16_t)raw_data[3];
  destination[2] = ((int16_t)raw_data[4] << 8) | (int16_t)raw_data[5];
  destination[3] = ((int16_t)raw_data[6] << 8) | (int16_t)raw_data[7];
  destination[4] = ((int16_t)raw_data[8] << 8) | (int16_t)raw_data[9];
  destination[5] = ((int16_t)raw_data[10] << 8) | (int16_t)raw_data[11];
  destination[6] = ((int16_t)raw_data[12] << 8) | (int16_t)raw_data[13];
}

void MPU9250::update_mag()
{
  int16_t mag_count[3] = {0, 0, 0}; // Stores the 16-bit signed magnetometer sensor output

  // Read the x/y/z adc values
  if (read_mag(mag_count))
  {
    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
    // mag_bias is calcurated in 16BITS
    float bias_to_current_bits = mag_resolution / get_mag_resolution(MAG_OUTPUT_BITS::M16BITS);
    m[0] = (float)(mag_count[0] * mag_resolution * mag_bias_factory[0] - mag_bias[0] * bias_to_current_bits) * mag_scale[0]; // get actual magnetometer value, this depends on scale being set
    m[1] = (float)(mag_count[1] * mag_resolution * mag_bias_factory[1] - mag_bias[1] * bias_to_current_bits) * mag_scale[1];
    m[2] = (float)(mag_count[2] * mag_resolution * mag_bias_factory[2] - mag_bias[2] * bias_to_current_bits) * mag_scale[2];
  }
}

bool MPU9250::read_mag(int16_t *destination)
{
  const uint8_t st1 = read_byte(AK8963_ADDRESS, AK8963_ST1);
  if (st1 & 0x01)
  {                                                             // wait for magnetometer data ready bit to be set
    uint8_t raw_data[7];                                        // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
    read_bytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &raw_data[0]); // Read the six raw data and ST2 registers sequentially into data array
    if (MAG_MODE == 0x02 || MAG_MODE == 0x04 || MAG_MODE == 0x06)
    {                        // continuous or external trigger read mode
      if ((st1 & 0x02) != 0) // check if data is not skipped
        return false;        // this should be after data reading to clear DRDY register
    }

    uint8_t c = raw_data[6]; // End data read by reading ST2 register
    if (!(c & 0x08))
    {                                                             // Check if magnetic sensor overflow set, if not then report data
      destination[0] = ((int16_t)raw_data[1] << 8) | raw_data[0]; // Turn the MSB and LSB into a signed 16-bit value
      destination[1] = ((int16_t)raw_data[3] << 8) | raw_data[2]; // Data stored as little Endian
      destination[2] = ((int16_t)raw_data[5] << 8) | raw_data[4];
      return true;
    }
  }
  return false;
}

int16_t MPU9250::read_temperature_data()
{
  uint8_t raw_data[2];                                   // x/y/z gyro register data stored here
  read_bytes(mpu_i2c_addr, TEMP_OUT_H, 2, &raw_data[0]); // Read the two raw data registers sequentially into data array
  return ((int16_t)raw_data[0] << 8) | raw_data[1];      // Turn the MSB and LSB into a 16-bit value
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
// ACCEL_FS_SEL: 2g (maximum sensitivity)
// GYRO_FS_SEL: 250dps (maximum sensitivity)
void MPU9250::calibrate_acc_gyro_impl()
{
  set_acc_gyro_to_calibration();
  collect_acc_gyro_data_to(acc_bias, gyro_bias);
  write_accel_offset();
  write_gyro_offset();
  vTaskDelay(pdMS_TO_TICKS(100));
  initMPU9250();
  vTaskDelay(pdMS_TO_TICKS(1000));
}

void MPU9250::set_acc_gyro_to_calibration()
{
  // reset device
  write_byte(mpu_i2c_addr, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  vTaskDelay(pdMS_TO_TICKS(100));

  // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
  // else use the internal oscillator, bits 2:0 = 001
  write_byte(mpu_i2c_addr, PWR_MGMT_1, 0x01);
  write_byte(mpu_i2c_addr, PWR_MGMT_2, 0x00);
  vTaskDelay(pdMS_TO_TICKS(200));

  // Configure device for bias calculation
  write_byte(mpu_i2c_addr, INT_ENABLE, 0x00);   // Disable all interrupts
  write_byte(mpu_i2c_addr, FIFO_EN, 0x00);      // Disable FIFO
  write_byte(mpu_i2c_addr, PWR_MGMT_1, 0x00);   // Turn on internal clock source
  write_byte(mpu_i2c_addr, I2C_MST_CTRL, 0x00); // Disable I2C master
  write_byte(mpu_i2c_addr, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  write_byte(mpu_i2c_addr, USER_CTRL, 0x0C);    // Reset FIFO and DMP
  vTaskDelay(pdMS_TO_TICKS(15));

  // Configure MPU6050 gyro and accelerometer for bias calculation
  write_byte(mpu_i2c_addr, MPU_CONFIG, 0x01);   // Set low-pass filter to 188 Hz
  write_byte(mpu_i2c_addr, SMPLRT_DIV, 0x00);   // Set sample rate to 1 kHz
  write_byte(mpu_i2c_addr, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  write_byte(mpu_i2c_addr, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

  // Configure FIFO to capture accelerometer and gyro data for bias calculation
  write_byte(mpu_i2c_addr, USER_CTRL, 0x40); // Enable FIFO
  write_byte(mpu_i2c_addr, FIFO_EN, 0x78);   // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  vTaskDelay(pdMS_TO_TICKS(40));
}

void MPU9250::collect_acc_gyro_data_to(float *a_bias, float *g_bias)
{
  // At end of sample accumulation, turn off FIFO sensor read
  uint8_t data[12];                                   // data array to hold accelerometer and gyro x, y, z, data
  write_byte(mpu_i2c_addr, FIFO_EN, 0x00);            // Disable gyro and accelerometer sensors for FIFO
  read_bytes(mpu_i2c_addr, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  uint16_t fifo_count = ((uint16_t)data[0] << 8) | data[1];
  uint16_t packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

  for (uint16_t ii = 0; ii < packet_count; ii++)
  {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    read_bytes(mpu_i2c_addr, FIFO_R_W, 12, &data[0]);             // read data for averaging
    accel_temp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]); // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
    accel_temp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);
    gyro_temp[0] = (int16_t)(((int16_t)data[6] << 8) | data[7]);
    gyro_temp[1] = (int16_t)(((int16_t)data[8] << 8) | data[9]);
    gyro_temp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);

    a_bias[0] += (float)accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    a_bias[1] += (float)accel_temp[1];
    a_bias[2] += (float)accel_temp[2];
    g_bias[0] += (float)gyro_temp[0];
    g_bias[1] += (float)gyro_temp[1];
    g_bias[2] += (float)gyro_temp[2];
  }
  a_bias[0] /= (float)packet_count; // Normalize sums to get average count biases
  a_bias[1] /= (float)packet_count;
  a_bias[2] /= (float)packet_count;
  g_bias[0] /= (float)packet_count;
  g_bias[1] /= (float)packet_count;
  g_bias[2] /= (float)packet_count;

  if (a_bias[2] > 0L)
  {
    a_bias[2] -= (float)CALIB_ACCEL_SENSITIVITY;
  } // Remove gravity from the z-axis accelerometer bias calculation
  else
  {
    a_bias[2] += (float)CALIB_ACCEL_SENSITIVITY;
  }
}

void MPU9250::write_accel_offset()
{
  // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
  // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
  // non-zero values. In addition, bit 0 of the lower uint8_t must be preserved since it is used for temperature
  // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
  // the accelerometer biases calculated above must be divided by 8.

  uint8_t read_data[2] = {0};
  int16_t acc_bias_reg[3] = {0, 0, 0};                     // A place to hold the factory accelerometer trim biases
  read_bytes(mpu_i2c_addr, XA_OFFSET_H, 2, &read_data[0]); // Read factory accelerometer trim values
  acc_bias_reg[0] = ((int16_t)read_data[0] << 8) | read_data[1];
  read_bytes(mpu_i2c_addr, YA_OFFSET_H, 2, &read_data[0]);
  acc_bias_reg[1] = ((int16_t)read_data[0] << 8) | read_data[1];
  read_bytes(mpu_i2c_addr, ZA_OFFSET_H, 2, &read_data[0]);
  acc_bias_reg[2] = ((int16_t)read_data[0] << 8) | read_data[1];

  int16_t mask_bit[3] = {1, 1, 1}; // Define array to hold mask bit for each accelerometer bias axis
  for (int i = 0; i < 3; i++)
  {
    if (acc_bias_reg[i] % 2)
    {
      mask_bit[i] = 0;
    }
    acc_bias_reg[i] -= (int16_t)acc_bias[i] >> 3; // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g
    if (mask_bit[i])
    {
      acc_bias_reg[i] = acc_bias_reg[i] & ~mask_bit[i]; // Preserve temperature compensation bit
    }
    else
    {
      acc_bias_reg[i] = acc_bias_reg[i] | 0x0001; // Preserve temperature compensation bit
    }
  }

  uint8_t write_data[6] = {0};
  write_data[0] = (acc_bias_reg[0] >> 8) & 0xFF;
  write_data[1] = (acc_bias_reg[0]) & 0xFF;
  write_data[2] = (acc_bias_reg[1] >> 8) & 0xFF;
  write_data[3] = (acc_bias_reg[1]) & 0xFF;
  write_data[4] = (acc_bias_reg[2] >> 8) & 0xFF;
  write_data[5] = (acc_bias_reg[2]) & 0xFF;

  // Push accelerometer biases to hardware registers
  write_byte(mpu_i2c_addr, XA_OFFSET_H, write_data[0]);
  write_byte(mpu_i2c_addr, XA_OFFSET_L, write_data[1]);
  write_byte(mpu_i2c_addr, YA_OFFSET_H, write_data[2]);
  write_byte(mpu_i2c_addr, YA_OFFSET_L, write_data[3]);
  write_byte(mpu_i2c_addr, ZA_OFFSET_H, write_data[4]);
  write_byte(mpu_i2c_addr, ZA_OFFSET_L, write_data[5]);
}

void MPU9250::write_gyro_offset()
{
  // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  uint8_t gyro_offset_data[6]{0};
  gyro_offset_data[0] = (-(int16_t)gyro_bias[0] / 4 >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  gyro_offset_data[1] = (-(int16_t)gyro_bias[0] / 4) & 0xFF;      // Biases are additive, so change sign on calculated average gyro biases
  gyro_offset_data[2] = (-(int16_t)gyro_bias[1] / 4 >> 8) & 0xFF;
  gyro_offset_data[3] = (-(int16_t)gyro_bias[1] / 4) & 0xFF;
  gyro_offset_data[4] = (-(int16_t)gyro_bias[2] / 4 >> 8) & 0xFF;
  gyro_offset_data[5] = (-(int16_t)gyro_bias[2] / 4) & 0xFF;

  // Push gyro biases to hardware registers
  write_byte(mpu_i2c_addr, XG_OFFSET_H, gyro_offset_data[0]);
  write_byte(mpu_i2c_addr, XG_OFFSET_L, gyro_offset_data[1]);
  write_byte(mpu_i2c_addr, YG_OFFSET_H, gyro_offset_data[2]);
  write_byte(mpu_i2c_addr, YG_OFFSET_L, gyro_offset_data[3]);
  write_byte(mpu_i2c_addr, ZG_OFFSET_H, gyro_offset_data[4]);
  write_byte(mpu_i2c_addr, ZG_OFFSET_L, gyro_offset_data[5]);
}

// mag calibration is executed in MAG_OUTPUT_BITS: 16BITS
void MPU9250::calibrate_mag_impl()
{
  // set MAG_OUTPUT_BITS to maximum to calibrate
  MAG_OUTPUT_BITS mag_output_bits_cache = setting.mag_output_bits;
  setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
  initAK8963();
  collect_mag_data_to(mag_bias, mag_scale);

  if (b_verbose) {
    printf("Mag Calibration done!");
    printf("AK8963 mag biases (mG) %f, %f, %f\n", mag_bias[0], mag_bias[1], mag_bias[2]);        
    printf("AK8963 mag scale (mG) %f, %f, %f\n", mag_scale[0], mag_scale[1], mag_scale[2]);    
  }

  // restore MAG_OUTPUT_BITS
  setting.mag_output_bits = mag_output_bits_cache;
  initAK8963();
}

void MPU9250::collect_mag_data_to(float *m_bias, float *m_scale)
{
  if (b_verbose)
    printf("Mag Calibration: Wave device in a figure eight until done!");
  vTaskDelay(pdMS_TO_TICKS(4000));

  // shoot for ~fifteen seconds of mag data
  uint16_t sample_count = 0;
  if (MAG_MODE == 0x02)
    sample_count = 128;      // at 8 Hz ODR, new mag data is available every 125 ms
  else if (MAG_MODE == 0x06) // in this library, fixed to 100Hz
    sample_count = 1500;     // at 100 Hz ODR, new mag data is available every 10 ms

  int32_t bias[3] = {0, 0, 0}, scale[3] = {0, 0, 0};
  int16_t mag_max[3] = {-32767, -32767, -32767};
  int16_t mag_min[3] = {32767, 32767, 32767};
  int16_t mag_temp[3] = {0, 0, 0};
  for (uint16_t ii = 0; ii < sample_count; ii++)
  {
    read_mag(mag_temp); // Read the mag data
    for (int jj = 0; jj < 3; jj++)
    {
      if (mag_temp[jj] > mag_max[jj])
        mag_max[jj] = mag_temp[jj];
      if (mag_temp[jj] < mag_min[jj])
        mag_min[jj] = mag_temp[jj];
    }
    if (MAG_MODE == 0x02)
      vTaskDelay(pdMS_TO_TICKS(135)); // at 8 Hz ODR, new mag data is available every 125 ms
    if (MAG_MODE == 0x06)
      vTaskDelay(pdMS_TO_TICKS(12));; // at 100 Hz ODR, new mag data is available every 10 ms
  }

  if (b_verbose) {
    printf("mag x min/max: %d/%d\n", mag_min[0], mag_max[0]);    
    printf("mag y min/max: %d/%d\n", mag_min[1], mag_max[1]);    
    printf("mag z min/max: %d/%d\n", mag_min[2], mag_max[2]);    
  }

  // Get hard iron correction
  bias[0] = (mag_max[0] + mag_min[0]) / 2; // get average x mag bias in counts
  bias[1] = (mag_max[1] + mag_min[1]) / 2; // get average y mag bias in counts
  bias[2] = (mag_max[2] + mag_min[2]) / 2; // get average z mag bias in counts

  float bias_resolution = get_mag_resolution(MAG_OUTPUT_BITS::M16BITS);
  m_bias[0] = (float)bias[0] * bias_resolution * mag_bias_factory[0]; // save mag biases in G for main program
  m_bias[1] = (float)bias[1] * bias_resolution * mag_bias_factory[1];
  m_bias[2] = (float)bias[2] * bias_resolution * mag_bias_factory[2];

  // Get soft iron correction estimate
  //*** multiplication by mag_bias_factory added in accordance with the following comment
  //*** https://github.com/kriswiner/MPU9250/issues/456#issue-836657973
  scale[0] = (float)(mag_max[0] - mag_min[0]) * mag_bias_factory[0] / 2; // get average x axis max chord length in counts
  scale[1] = (float)(mag_max[1] - mag_min[1]) * mag_bias_factory[1] / 2; // get average y axis max chord length in counts
  scale[2] = (float)(mag_max[2] - mag_min[2]) * mag_bias_factory[2] / 2; // get average z axis max chord length in counts

  float avg_rad = scale[0] + scale[1] + scale[2];
  avg_rad /= 3.0;

  m_scale[0] = avg_rad / ((float)scale[0]);
  m_scale[1] = avg_rad / ((float)scale[1]);
  m_scale[2] = avg_rad / ((float)scale[2]);
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
bool MPU9250::self_test_impl() { // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
  uint8_t raw_data[6] = {0, 0, 0, 0, 0, 0};
  int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
  float factoryTrim[6];
  uint8_t FS = 0;

  write_byte(mpu_i2c_addr, SMPLRT_DIV, 0x00);      // Set gyro sample rate to 1 kHz
  write_byte(mpu_i2c_addr, MPU_CONFIG, 0x02);      // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  write_byte(mpu_i2c_addr, GYRO_CONFIG, FS << 3);  // Set full scale range for the gyro to 250 dps
  write_byte(mpu_i2c_addr, ACCEL_CONFIG2, 0x02);   // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  write_byte(mpu_i2c_addr, ACCEL_CONFIG, FS << 3); // Set full scale range for the accelerometer to 2 g

  for (int ii = 0; ii < 200; ii++)
  { // get average current values of gyro and acclerometer

    read_bytes(mpu_i2c_addr, ACCEL_XOUT_H, 6, &raw_data[0]);         // Read the six raw data registers into data array
    aAvg[0] += (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]); // Turn the MSB and LSB into a signed 16-bit value
    aAvg[1] += (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]);
    aAvg[2] += (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]);

    read_bytes(mpu_i2c_addr, GYRO_XOUT_H, 6, &raw_data[0]);          // Read the six raw data registers sequentially into data array
    gAvg[0] += (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]); // Turn the MSB and LSB into a signed 16-bit value
    gAvg[1] += (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]);
    gAvg[2] += (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]);
  }

  for (int ii = 0; ii < 3; ii++)
  { // Get average of 200 values and store as average current readings
    aAvg[ii] /= 200;
    gAvg[ii] /= 200;
  }

  // Configure the accelerometer for self-test
  write_byte(mpu_i2c_addr, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
  write_byte(mpu_i2c_addr, GYRO_CONFIG, 0xE0);  // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
  vTaskDelay(pdMS_TO_TICKS(25));

  for (int ii = 0; ii < 200; ii++)
  { // get average self-test values of gyro and acclerometer

    read_bytes(mpu_i2c_addr, ACCEL_XOUT_H, 6, &raw_data[0]);           // Read the six raw data registers into data array
    aSTAvg[0] += (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]); // Turn the MSB and LSB into a signed 16-bit value
    aSTAvg[1] += (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]);
    aSTAvg[2] += (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]);

    read_bytes(mpu_i2c_addr, GYRO_XOUT_H, 6, &raw_data[0]);            // Read the six raw data registers sequentially into data array
    gSTAvg[0] += (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]); // Turn the MSB and LSB into a signed 16-bit value
    gSTAvg[1] += (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]);
    gSTAvg[2] += (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]);
  }

  for (int ii = 0; ii < 3; ii++)
  { // Get average of 200 values and store as average self-test readings
    aSTAvg[ii] /= 200;
    gSTAvg[ii] /= 200;
  }

  // Configure the gyro and accelerometer for normal operation
  write_byte(mpu_i2c_addr, ACCEL_CONFIG, 0x00);
  write_byte(mpu_i2c_addr, GYRO_CONFIG, 0x00);
  vTaskDelay(pdMS_TO_TICKS(25)); // Delay a while to let the device stabilize

  // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
  uint8_t self_test_data[6];
  self_test_data[0] = read_byte(mpu_i2c_addr, SELF_TEST_X_ACCEL); // X-axis accel self-test results
  self_test_data[1] = read_byte(mpu_i2c_addr, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
  self_test_data[2] = read_byte(mpu_i2c_addr, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
  self_test_data[3] = read_byte(mpu_i2c_addr, SELF_TEST_X_GYRO);  // X-axis gyro self-test results
  self_test_data[4] = read_byte(mpu_i2c_addr, SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
  self_test_data[5] = read_byte(mpu_i2c_addr, SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

  // Retrieve factory self-test value from self-test code reads
  factoryTrim[0] = (float)(2620 / 1 << FS) * (pow(1.01, ((float)self_test_data[0] - 1.0))); // FT[Xa] factory trim calculation
  factoryTrim[1] = (float)(2620 / 1 << FS) * (pow(1.01, ((float)self_test_data[1] - 1.0))); // FT[Ya] factory trim calculation
  factoryTrim[2] = (float)(2620 / 1 << FS) * (pow(1.01, ((float)self_test_data[2] - 1.0))); // FT[Za] factory trim calculation
  factoryTrim[3] = (float)(2620 / 1 << FS) * (pow(1.01, ((float)self_test_data[3] - 1.0))); // FT[Xg] factory trim calculation
  factoryTrim[4] = (float)(2620 / 1 << FS) * (pow(1.01, ((float)self_test_data[4] - 1.0))); // FT[Yg] factory trim calculation
  factoryTrim[5] = (float)(2620 / 1 << FS) * (pow(1.01, ((float)self_test_data[5] - 1.0))); // FT[Zg] factory trim calculation

  // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
  // To get percent, must multiply by 100
  for (int i = 0; i < 3; i++)
  {
    self_test_result[i] = 100.0 * ((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i] - 100.;         // Report percent differences
    self_test_result[i + 3] = 100.0 * ((float)(gSTAvg[i] - gAvg[i])) / factoryTrim[i + 3] - 100.; // Report percent differences
  }

  if (b_verbose) {
    printf("x-axis self test: acceleration trim within : %f % of factory value\n", self_test_result[0]);
    printf("y-axis self test: acceleration trim within : % of factory value\n", self_test_result[1]);
    printf("z-axis self test: acceleration trim within : % of factory value\n", self_test_result[2]);
    printf("x-axis self test: gyration trim within : % of factory value\n", self_test_result[3]);
    printf("y-axis self test: gyration trim within : % of factory value\n", self_test_result[4]);
    printf("z-axis self test: gyration trim within : % of factory value\n", self_test_result[5]);
  }

  bool b = true;
  for (uint8_t i = 0; i < 6; ++i) {
    b &= fabs(self_test_result[i]) <= 14.f;
  }
  return b;
}

float MPU9250::get_acc_resolution(const ACCEL_FS_SEL accel_af_sel) const {
  switch (accel_af_sel)
  {
  // Possible accelerometer scales (and their register bit settings) are:
  // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
  // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
  case ACCEL_FS_SEL::A2G:
    return 2.0 / 32768.0;
  case ACCEL_FS_SEL::A4G:
    return 4.0 / 32768.0;
  case ACCEL_FS_SEL::A8G:
    return 8.0 / 32768.0;
  case ACCEL_FS_SEL::A16G:
    return 16.0 / 32768.0;
  default:
    return 0.;
  }
}

float MPU9250::get_gyro_resolution(const GYRO_FS_SEL gyro_fs_sel) const
{
  switch (gyro_fs_sel)
  {
  // Possible gyro scales (and their register bit settings) are:
  // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
  // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
  case GYRO_FS_SEL::G250DPS:
    return 250.0 / 32768.0;
  case GYRO_FS_SEL::G500DPS:
    return 500.0 / 32768.0;
  case GYRO_FS_SEL::G1000DPS:
    return 1000.0 / 32768.0;
  case GYRO_FS_SEL::G2000DPS:
    return 2000.0 / 32768.0;
  default:
    return 0.;
  }
}

float MPU9250::get_mag_resolution(const MAG_OUTPUT_BITS mag_output_bits) const
{
  switch (mag_output_bits)
  {
  // Possible magnetometer scales (and their register bit settings) are:
  // 14 bit resolution (0) and 16 bit resolution (1)
  // Proper scale to return milliGauss
  case MAG_OUTPUT_BITS::M14BITS:
    return 10. * 4912. / 8190.0;
  case MAG_OUTPUT_BITS::M16BITS:
    return 10. * 4912. / 32760.0;
  default:
    return 0.;
  }
}

void MPU9250::write_byte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  int wr = i2c_write_blocking(_i2cPort, subAddress, &data, 1, false);    
  if (wr == PICO_ERROR_GENERIC)
    print_i2c_error();
}

uint8_t MPU9250::read_byte(uint8_t address, uint8_t subAddress)
{
  int wr = i2c_write_blocking(_i2cPort, address, &subAddress, 1, false);
  if (wr == PICO_ERROR_GENERIC) {
    print_i2c_error();
    return 0;
  }
  uint8_t pdata[1];
  int rr = i2c_read_blocking(_i2cPort, address, pdata, 1, false);
  if (rr == PICO_ERROR_GENERIC) {
    print_i2c_error();
    return 0;
  }
  return pdata[0];    
}

int MPU9250::read_bytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest)
{
  int wr = i2c_write_blocking(_i2cPort, address, &subAddress, 1, false);
  if (wr == PICO_ERROR_GENERIC) return 0;
  int rr = i2c_read_blocking(_i2cPort, address, dest, count, false);  
  if (rr == PICO_ERROR_GENERIC) {
    print_i2c_error();
    return 0;
  }
  return rr;
}

void MPU9250::print_i2c_error()
{
  printf("I2C ERROR CODE: %d\n", i2c_err_);
}