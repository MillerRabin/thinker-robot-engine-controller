#include "../../vl53l0x_def.h"
#include "../../vl53l0x_i2c_platform.h"
#include "pico/stdlib.h"
#include <string.h>

#define STATUS_OK              0x00
#define STATUS_FAIL            0x01

//#define I2C_DEBUG

int VL53L0X_write_multi(uint8_t deviceAddress, uint8_t index, uint8_t *pdata,
                        uint32_t count, i2c_inst_t *i2c)
{    
  int32_t status = VL53L0X_ERROR_NONE;
  uint8_t i2c_buff[count+1];
  i2c_buff[0] = index;
  memcpy(i2c_buff+1, pdata, count);
  if (i2c_write_blocking(i2c, deviceAddress, i2c_buff, count+1, false) == PICO_ERROR_GENERIC) {    
    status = STATUS_FAIL;
  }
  return status;
}

int VL53L0X_read_multi(uint8_t deviceAddress, uint8_t index, uint8_t *pdata,
                       uint32_t count, i2c_inst_t *i2c) {
    int32_t status = VL53L0X_ERROR_NONE;
 
    int i2c_ret = i2c_write_blocking(i2c, deviceAddress, &index, 1, true);
    if (i2c_ret == PICO_ERROR_GENERIC) {      
      return STATUS_FAIL;
    }
    i2c_ret = i2c_read_blocking(i2c, deviceAddress, pdata, count, false);
    if (i2c_ret == PICO_ERROR_GENERIC) {      
      return STATUS_FAIL;
    }
    return status;
}

int VL53L0X_write_byte(uint8_t deviceAddress, uint8_t index, uint8_t data,
                       i2c_inst_t *i2c) {
  return VL53L0X_write_multi(deviceAddress, index, &data, 1, i2c);
}

int VL53L0X_write_word(uint8_t deviceAddress, uint8_t index, uint16_t data,
                       i2c_inst_t *i2c) {
  uint8_t buff[2];
  buff[1] = data & 0xFF;
  buff[0] = data >> 8;
  return VL53L0X_write_multi(deviceAddress, index, buff, 2, i2c);
}

int VL53L0X_write_dword(uint8_t deviceAddress, uint8_t index, uint32_t data,
                        i2c_inst_t *i2c) {
  uint8_t buff[4];

  buff[3] = data & 0xFF;
  buff[2] = data >> 8;
  buff[1] = data >> 16;
  buff[0] = data >> 24;

  return VL53L0X_write_multi(deviceAddress, index, buff, 4, i2c);
}

int VL53L0X_read_byte(uint8_t deviceAddress, uint8_t index, uint8_t *data,
                      i2c_inst_t *i2c) {
  return VL53L0X_read_multi(deviceAddress, index, data, 1, i2c);
}

int VL53L0X_read_word(uint8_t deviceAddress, uint8_t index, uint16_t *data,
                      i2c_inst_t *i2c) {
  uint8_t buff[2];
  int r = VL53L0X_read_multi(deviceAddress, index, buff, 2, i2c);

  uint16_t tmp;
  tmp = buff[0];
  tmp <<= 8;
  tmp |= buff[1];
  *data = tmp;

  return r;
}

int VL53L0X_read_dword(uint8_t deviceAddress, uint8_t index, uint32_t *data,
                       i2c_inst_t *i2c) {
  uint8_t buff[4];
  int r = VL53L0X_read_multi(deviceAddress, index, buff, 4, i2c);

  uint32_t tmp;
  tmp = buff[0];
  tmp <<= 8;
  tmp |= buff[1];
  tmp <<= 8;
  tmp |= buff[2];
  tmp <<= 8;
  tmp |= buff[3];

  *data = tmp;

  return r;
}
