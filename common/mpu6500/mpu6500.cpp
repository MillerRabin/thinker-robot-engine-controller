#include "mpu6500.h"

bool MPU6500::setup(const uint8_t addr, const MPU6500Setting &mpu_setting, i2c_inst_t *_i2cPort)
{
  // addr should be valid for MPU
  if ((addr < MPU6500_DEFAULT_ADDRESS) || (addr > MPU6500_DEFAULT_ADDRESS + 7))
  {
    printf("I2C address 0x%x is not valid for MPU. Please check your I2C address.\n", addr);
    return false;
  }
  mpu_i2c_addr = addr;
  setting = mpu_setting;
  this->_i2cPort = _i2cPort;

  if (isConnectedMPU6500())
  {
    printf("MPU connected\n");
    initMPU6500();
    printf("MPU inited\n");
  }
  else
  {
    if (b_verbose)
      printf("Could not connect to MPU6500\n");
    has_connected = false;
    return false;
  }
  has_connected = true;
  return true;
}

void MPU6500::sleep(bool b)
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

void MPU6500::verbose(const bool b)
{
  b_verbose = b;
}

void MPU6500::ahrs(const bool b)
{
  b_ahrs = b;
}

void MPU6500::calibrateAccelGyro()
{
  calibrate_acc_gyro_impl();
}

bool MPU6500::isConnected()
{
  has_connected = isConnectedMPU6500();
  return has_connected;
}

bool MPU6500::isConnectedMPU6500()
{
  uint8_t c = read_byte(mpu_i2c_addr, WHO_AM_I_MPU9250);
  if (b_verbose)
  {
    printf("MPU6500 WHO AM I = 0x%x\n", c);
  }
  bool b = (c == MPU6500_WHOAMI_DEFAULT_VALUE);
  b |= (c == MPU9255_WHOAMI_DEFAULT_VALUE);
  b |= (c == MPU6500_WHOAMI_DEFAULT_VALUE);
  return b;
}

bool MPU6500::isSleeping()
{
  uint8_t c = read_byte(mpu_i2c_addr, PWR_MGMT_1);
  return (c & 0x40) == 0x40;
}

bool MPU6500::available()
{
  return has_connected && (read_byte(mpu_i2c_addr, INT_STATUS) & 0x01);
}

bool MPU6500::update()
{
  if (!available())
    return false;

  update_accel_gyro();

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
  float mn = 0;
  float me = 0;
  float md = 0;

  printf("an: %f, ae: %f, ad:%f, gn: %f, ge: %f, gd:%f\n", an, ae, ad, gn, ge, gd);

  for (size_t i = 0; i < n_filter_iter; ++i)
  {
    quat_filter.update(an, ae, ad, gn, ge, gd, mn, me, md, q);
  }

  if (!b_ahrs)
  {
    temperature_count = read_temperature_data();              // Read the adc values
    temperature = ((float)temperature_count) / 333.87 + 21.0; // Temperature in degrees Centigrade
  }
  else
  {
    update_rpy(q[0], q[1], q[2], q[3]);
  }
  return true;
}

void MPU6500::setAccBias(const float x, const float y, const float z)
{
  acc_bias[0] = x;
  acc_bias[1] = y;
  acc_bias[2] = z;
  write_accel_offset();
}
void MPU6500::setGyroBias(const float x, const float y, const float z)
{
  gyro_bias[0] = x;
  gyro_bias[1] = y;
  gyro_bias[2] = z;
  write_gyro_offset();
}

void MPU6500::selectFilter(QuatFilterSel sel)
{
  quat_filter.select_filter(sel);
}

void MPU6500::setFilterIterations(const size_t n)
{
  if (n > 0)
    n_filter_iter = n;
}

bool MPU6500::selftest()
{
  return self_test_impl();
}

void MPU6500::initMPU6500()
{
  acc_resolution = get_acc_resolution(setting.accel_fs_sel);
  gyro_resolution = get_gyro_resolution(setting.gyro_fs_sel);
  
  // reset device
  printf("reset device\n");
  write_byte(mpu_i2c_addr, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  printf("device reseted\n");
  vTaskDelay(pdMS_TO_TICKS(100));
  // wake up device
  printf("device wake up\n");
  write_byte(mpu_i2c_addr, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
  printf("device wake up done\n");
  vTaskDelay(pdMS_TO_TICKS(100)); // Wait for all registers to reset

  // get stable time source
  printf("Select clock source\n");
  write_byte(mpu_i2c_addr, PWR_MGMT_1, 0x01); // Auto select clock source to be PLL gyroscope reference if ready else
  vTaskDelay(pdMS_TO_TICKS(200));
  printf("Select clock source done\n");
  // Configure Gyro and Thermometer
  // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
  // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
  // be higher than 1 / 0.0059 = 170 Hz
  // GYRO_DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
  // With the MPU6500, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
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

void MPU6500::update_rpy(float qw, float qx, float qy, float qz)
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
  if (rpy[2] >= +180.f)
    rpy[2] -= 360.f;
  else if (rpy[2] < -180.f)
    rpy[2] += 360.f;

  lin_acc[0] = a[0] + a31;
  lin_acc[1] = a[1] + a32;
  lin_acc[2] = a[2] - a33;
}

void MPU6500::update_accel_gyro()
{
  int16_t raw_acc_gyro_data[7];       // used to read all 14 bytes at once from the MPU6500 accel/gyro
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

void MPU6500::read_accel_gyro(int16_t *destination)
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

int16_t MPU6500::read_temperature_data()
{
  uint8_t raw_data[2];                                   // x/y/z gyro register data stored here
  read_bytes(mpu_i2c_addr, TEMP_OUT_H, 2, &raw_data[0]); // Read the two raw data registers sequentially into data array
  return ((int16_t)raw_data[0] << 8) | raw_data[1];      // Turn the MSB and LSB into a 16-bit value
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
// ACCEL_FS_SEL: 2g (maximum sensitivity)
// GYRO_FS_SEL: 250dps (maximum sensitivity)
void MPU6500::calibrate_acc_gyro_impl()
{
  set_acc_gyro_to_calibration();
  collect_acc_gyro_data_to(acc_bias, gyro_bias);
  write_accel_offset();
  write_gyro_offset();
  vTaskDelay(pdMS_TO_TICKS(100));
  initMPU6500();
  vTaskDelay(pdMS_TO_TICKS(1000));
}

void MPU6500::set_acc_gyro_to_calibration()
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

void MPU6500::collect_acc_gyro_data_to(float *a_bias, float *g_bias)
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

void MPU6500::write_accel_offset()
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

void MPU6500::write_gyro_offset()
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

// Accelerometer and gyroscope self test; check calibration wrt factory settings
bool MPU6500::self_test_impl()
{ // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
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

  if (b_verbose)
  {
    printf("x-axis self test: acceleration trim within : %f % of factory value\n", self_test_result[0]);
    printf("y-axis self test: acceleration trim within : %f % of factory value\n", self_test_result[1]);
    printf("z-axis self test: acceleration trim within : %f % of factory value\n", self_test_result[2]);
    printf("x-axis self test: gyration trim within : %f % of factory value\n", self_test_result[3]);
    printf("y-axis self test: gyration trim within : %f % of factory value\n", self_test_result[4]);
    printf("z-axis self test: gyration trim within : %f % of factory value\n", self_test_result[5]);
  }

  bool b = true;
  for (uint8_t i = 0; i < 6; ++i)
  {
    b &= fabs(self_test_result[i]) <= 14.f;
  }
  return b;
}

float MPU6500::get_acc_resolution(const ACCEL_FS_SEL accel_af_sel) const
{
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

float MPU6500::get_gyro_resolution(const GYRO_FS_SEL gyro_fs_sel) const
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

void MPU6500::write_byte(uint8_t address, uint8_t subAddress, uint8_t data) {
  uint8_t pdata[2] = { subAddress, data };
  int wr = i2c_write_blocking(_i2cPort, subAddress, pdata, 2, false);
  if (wr == PICO_ERROR_GENERIC)
    print_i2c_error();
}

uint8_t MPU6500::read_byte(uint8_t address, uint8_t subAddress)
{
  int wr = i2c_write_blocking(_i2cPort, address, &subAddress, 1, true);
  if (wr == PICO_ERROR_GENERIC)
  {
    print_i2c_error();
    return 0;
  }
  uint8_t pdata[1];
  int rr = i2c_read_blocking(_i2cPort, address, pdata, 1, false);
  if (rr == PICO_ERROR_GENERIC)
  {
    print_i2c_error();
    return 0;
  }
  return pdata[0];
}

int MPU6500::read_bytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest)
{
  int wr = i2c_write_blocking(_i2cPort, address, &subAddress, 1, true);
  if (wr == PICO_ERROR_GENERIC)
    return 0;
  int rr = i2c_read_blocking(_i2cPort, address, dest, count, false);
  if (rr == PICO_ERROR_GENERIC)
  {
    print_i2c_error();
    return 0;
  }
  return rr;
}

void MPU6500::print_i2c_error()
{
  printf("I2C ERROR CODE: %d\n", i2c_err_);
}