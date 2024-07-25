#include "bmp280.h"

BMP280::BMP280() {
  temp_sensor = new BMP280Temperature(this);
  pressure_sensor = new BMP280Pressure(this);
}
  
BMP280::~BMP280() {
  delete temp_sensor;
  delete pressure_sensor;
}


/*!
 *  Initialises the sensor.
 *  @param addr
 *         The I2C address to use (default = 0x77)
 *  @param chipid
 *         The expected chip ID (used to validate connection).
 *  @return True if the init was successful, otherwise false.
 */
bool BMP280::begin(uint8_t addr, uint8_t chipid, i2c_inst_t *i2c) {
  _i2caddr = addr;
  _i2cPort = i2c;
  _sensorID = read8(BMP280_REGISTER_CHIPID);
  if (_sensorID != chipid)
    return false;

  readCoefficients();
  // write8(BMP280_REGISTER_CONTROL, 0x3F); /* needed? */
  setSampling();  
  vTaskDelay(pdMS_TO_TICKS(100));
  return true;
}

/*!
 * Sets the sampling config for the device.
 * @param mode
 *        The operating mode of the sensor.
 * @param tempSampling
 *        The sampling scheme for temp readings.
 * @param pressSampling
 *        The sampling scheme for pressure readings.
 * @param filter
 *        The filtering mode to apply (if any).
 * @param duration
 *        The sampling duration.
 */
void BMP280::setSampling(sensor_mode mode,
                         sensor_sampling tempSampling,
                         sensor_sampling pressSampling,
                         sensor_filter filter,
                         standby_duration duration)
{
  if (!_sensorID)
    return;
  _measReg.mode = mode;
  _measReg.osrs_t = tempSampling;
  _measReg.osrs_p = pressSampling;

  _configReg.filter = filter;
  _configReg.t_sb = duration;
  
  write8(BMP280_REGISTER_CONFIG, _configReg.get());
  write8(BMP280_REGISTER_CONTROL, _measReg.get());
}

/**************************************************************************/
/*!
    @brief  Writes an 8 bit value over I2C/SPI
*/
/**************************************************************************/
void BMP280::write8(uint8_t reg, uint8_t value)
{
  uint8_t pdata[2];    
  pdata[0] = reg;
  pdata[1] = value;
  i2c_write_blocking(_i2cPort, _i2caddr, pdata, 2, false);      
}

/*!
 *  @brief  Reads an 8 bit value over I2C/SPI
 *  @param  reg
 *          selected register
 *  @return value from selected register
 */
uint8_t BMP280::read8(uint8_t reg)
{  
  int wr = i2c_write_blocking(_i2cPort, _i2caddr, &reg, 1, false);
  if (wr == PICO_ERROR_GENERIC) return 0;
  uint8_t pdata[1];
  int rr = i2c_read_blocking(_i2cPort, _i2caddr, pdata, 1, false);
  if (rr == PICO_ERROR_GENERIC) return 0;
  return pdata[0];      
}

/*!
 *  @brief  Reads a 16 bit value over I2C/SPI
 */
uint16_t BMP280::read16(uint8_t reg)
{
  int wr = i2c_write_blocking(_i2cPort, _i2caddr, &reg, 1, false);
  if (wr == PICO_ERROR_GENERIC) return 0;
  uint8_t pdata[2];
  int rr = i2c_read_blocking(_i2cPort, _i2caddr, pdata, 2, false);
  if (rr == PICO_ERROR_GENERIC) return 0;
  return uint16_t(pdata[0]) << 8 | uint16_t(pdata[1]);
}

uint16_t BMP280::read16_LE(uint8_t reg)
{
  uint16_t temp = read16(reg);
  return (temp >> 8) | (temp << 8);
}

/*!
 *   @brief  Reads a signed 16 bit value over I2C/SPI
 */
int16_t BMP280::readS16(uint8_t reg) { return (int16_t)read16(reg); }

int16_t BMP280::readS16_LE(uint8_t reg)
{
  return (int16_t)read16_LE(reg);
}

/*!
 *  @brief  Reads a 24 bit value over I2C/SPI
 */
uint32_t BMP280::read24(uint8_t reg)
{
  int wr = i2c_write_blocking(_i2cPort, _i2caddr, &reg, 1, false);
  if (wr == PICO_ERROR_GENERIC) return 0;
  uint8_t pdata[3];
  int rr = i2c_read_blocking(_i2cPort, _i2caddr, pdata, 3, false);
  if (rr == PICO_ERROR_GENERIC) return 0;
  return uint32_t(pdata[0]) << 16 | uint32_t(pdata[1]) << 8 |
         uint32_t(pdata[2]);
}

/*!
 *  @brief  Reads the factory-set coefficients
 */
void BMP280::readCoefficients()
{
  _bmp280_calib.dig_T1 = read16_LE(BMP280_REGISTER_DIG_T1);
  _bmp280_calib.dig_T2 = readS16_LE(BMP280_REGISTER_DIG_T2);
  _bmp280_calib.dig_T3 = readS16_LE(BMP280_REGISTER_DIG_T3);

  _bmp280_calib.dig_P1 = read16_LE(BMP280_REGISTER_DIG_P1);
  _bmp280_calib.dig_P2 = readS16_LE(BMP280_REGISTER_DIG_P2);
  _bmp280_calib.dig_P3 = readS16_LE(BMP280_REGISTER_DIG_P3);
  _bmp280_calib.dig_P4 = readS16_LE(BMP280_REGISTER_DIG_P4);
  _bmp280_calib.dig_P5 = readS16_LE(BMP280_REGISTER_DIG_P5);
  _bmp280_calib.dig_P6 = readS16_LE(BMP280_REGISTER_DIG_P6);
  _bmp280_calib.dig_P7 = readS16_LE(BMP280_REGISTER_DIG_P7);
  _bmp280_calib.dig_P8 = readS16_LE(BMP280_REGISTER_DIG_P8);
  _bmp280_calib.dig_P9 = readS16_LE(BMP280_REGISTER_DIG_P9);
}

/*!
 * Reads the temperature from the device.
 * @return The temperature in degrees celsius.
 */
float BMP280::readTemperature()
{
  int32_t var1, var2;
  if (!_sensorID)
    return NAN; // begin() not called yet

  int32_t adc_T = read24(BMP280_REGISTER_TEMPDATA);
  adc_T >>= 4;

  var1 = ((((adc_T >> 3) - ((int32_t)_bmp280_calib.dig_T1 << 1))) *
          ((int32_t)_bmp280_calib.dig_T2)) >>
         11;

  var2 = (((((adc_T >> 4) - ((int32_t)_bmp280_calib.dig_T1)) *
            ((adc_T >> 4) - ((int32_t)_bmp280_calib.dig_T1))) >>
           12) *
          ((int32_t)_bmp280_calib.dig_T3)) >>
         14;

  t_fine = var1 + var2;

  float T = (t_fine * 5 + 128) >> 8;
  return T / 100;
}

/*!
 * Reads the barometric pressure from the device.
 * @return Barometric pressure in Pa.
 */
float BMP280::readPressure()
{
  int64_t var1, var2, p;
  if (!_sensorID)
    return NAN; // begin() not called yet

  // Must be done first to get the t_fine variable set up
  readTemperature();

  int32_t adc_P = read24(BMP280_REGISTER_PRESSUREDATA);
  adc_P >>= 4;

  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)_bmp280_calib.dig_P6;
  var2 = var2 + ((var1 * (int64_t)_bmp280_calib.dig_P5) << 17);
  var2 = var2 + (((int64_t)_bmp280_calib.dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)_bmp280_calib.dig_P3) >> 8) +
         ((var1 * (int64_t)_bmp280_calib.dig_P2) << 12);
  var1 =
      (((((int64_t)1) << 47) + var1)) * ((int64_t)_bmp280_calib.dig_P1) >> 33;

  if (var1 == 0)
  {
    return 0; // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)_bmp280_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)_bmp280_calib.dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t)_bmp280_calib.dig_P7) << 4);
  return (float)p / 256;
}

/*!
 * @brief Calculates the approximate altitude using barometric pressure and the
 * supplied sea level hPa as a reference.
 * @param seaLevelhPa
 *        The current hPa at sea level.
 * @return The approximate altitude above sea level in meters.
 */
float BMP280::readAltitude(float seaLevelhPa)
{
  float altitude;

  float pressure = readPressure(); // in Si units for Pascal
  pressure /= 100;

  altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));

  return altitude;
}

/*!
 * Calculates the pressure at sea level (QNH) from the specified altitude,
 * and atmospheric pressure (QFE).
 * @param  altitude      Altitude in m
 * @param  atmospheric   Atmospheric pressure in hPa
 * @return The approximate pressure in hPa
 */
float BMP280::seaLevelForAltitude(float altitude, float atmospheric)
{
  // Equation taken from BMP180 datasheet (page 17):
  // http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

  // Note that using the equation from wikipedia can give bad results
  // at high altitude.  See this thread for more information:
  // http://forums.adafruit.com/viewtopic.php?f=22&t=58064
  return atmospheric / pow(1.0 - (altitude / 44330.0), 5.255);
}

/*!
    @brief  calculates the boiling point  of water by a given pressure
    @param pressure pressure in hPa
    @return temperature in °C
*/

float BMP280::waterBoilingPoint(float pressure)
{
  // Magnusformular for calculation of the boiling point of water at a given
  // pressure
  return (234.175 * log(pressure / 6.1078)) /
         (17.08085 - log(pressure / 6.1078));
}

/*!
    @brief  Take a new measurement (only possible in forced mode)
    @return true if successful, otherwise false
 */
bool BMP280::takeForcedMeasurement()
{
  // If we are in forced mode, the BME sensor goes back to sleep after each
  // measurement and we need to set it to forced mode once at this point, so
  // it will take the next measurement and then return to sleep again.
  // In normal mode simply does new measurements periodically.
  if (_measReg.mode == MODE_FORCED)
  {
    // set to forced mode, i.e. "take next measurement"
    write8(BMP280_REGISTER_CONTROL, _measReg.get());
    // wait until measurement has been completed, otherwise we would read
    // the values from the last measurement
    while (read8(BMP280_REGISTER_STATUS) & 0x08 ) 
      vTaskDelay(pdMS_TO_TICKS(1));
    return true;
  }
  return false;
}

/*!
 *  @brief  Resets the chip via soft reset
 */
void BMP280::reset(void)
{
  write8(BMP280_REGISTER_SOFTRESET, MODE_SOFT_RESET_CODE);
}

/*!
 *   Returns Sensor ID for diagnostics
 *   @returns 0x61 for BME680, 0x60 for BME280, 0x56, 0x57, 0x58 for BMP280
 */
uint8_t BMP280::sensorID(void) { return _sensorID; };

/*!
    @brief  Gets the most recent sensor event from the hardware status register.
    @return Sensor status as a byte.
 */
uint8_t BMP280::getStatus(void)
{
  return read8(BMP280_REGISTER_STATUS);
}

/*!
    @brief  Gets an Adafruit Unified Sensor object for the temp sensor component
    @return Adafruit_Sensor pointer to temperature sensor
 */
Sensor* BMP280::getTemperatureSensor(void)
{
  return temp_sensor;
}

/*!
    @brief  Gets an Adafruit Unified Sensor object for the pressure sensor
   component
    @return Adafruit_Sensor pointer to pressure sensor
 */
Sensor* BMP280::getPressureSensor(void)
{
  return pressure_sensor;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data for the BMP280's temperature sensor
*/
/**************************************************************************/
void BMP280Temperature::getSensor(sensor_t *sensor)
{
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "BMP280", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  sensor->min_delay = 0;
  sensor->min_value = -40.0; /* Temperature range -40 ~ +85 C  */
  sensor->max_value = +85.0;
  sensor->resolution = 0.01; /*  0.01 C */
}

/**************************************************************************/
/*!
    @brief  Gets the temperature as a standard sensor event
    @param  event Sensor event object that will be populated
    @returns True
*/
/**************************************************************************/
bool BMP280Temperature::getEvent(sensors_event_t *event)
{
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  event->timestamp = xTaskGetTickCount() * (1 / configTICK_RATE_HZ) * 1000;
  event->temperature = _theBMP280->readTemperature();
  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data for the BMP280's pressure sensor
*/
/**************************************************************************/
void BMP280Pressure::getSensor(sensor_t *sensor)
{
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "BMP280", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_PRESSURE;
  sensor->min_delay = 0;
  sensor->min_value = 300.0; /* 300 ~ 1100 hPa  */
  sensor->max_value = 1100.0;
  sensor->resolution = 0.012; /* 0.12 hPa relative */
}

/**************************************************************************/
/*!
    @brief  Gets the pressure as a standard sensor event
    @param  event Sensor event object that will be populated
    @returns True
*/
/**************************************************************************/
bool BMP280Pressure::getEvent(sensors_event_t *event)
{
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type = SENSOR_TYPE_PRESSURE;
  event->timestamp = xTaskGetTickCount() * (1 / configTICK_RATE_HZ) * 1000;
  event->pressure = _theBMP280->readPressure() / 100; // convert Pa to hPa
  return true;
}