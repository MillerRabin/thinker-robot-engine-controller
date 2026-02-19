#pragma once

#include "../BNO080/BNO080.h"
#include <iostream>
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "../imuBase/imuBase.h"
#include "../armPart/armPart.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "pico/binary_info.h"

class LocalBNO : public IMUBase {
  private:    
    const uint sdaPin;
    const uint sclPin;
    const uint intPin;
    const uint rstPin;
    const uint sckPin;
    const uint misoPin;
    const uint mosiPin;
    const uint csPin;
    const bool useSPI;
    static void compassTask(void* instance);
    static void compassCallback(uint gpio, uint32_t events);    
    static uint32_t notificationIndex;
    static TaskHandle_t compassTaskHandle;
    bool beginI2C();
    bool beginSPI();
  protected:
    void initIMU();
    ArmPart *armPart;
    BNO080 imu;
    int16_t accelerometer_Q1;    
    bool updateGyroscopeData(uint16_t rawGyroX, uint16_t rawGyroY, uint16_t rawGyroZ);
    bool updateAccuracy(uint16_t quaternionRadianAccuracy, uint8_t quaternionAccuracy, uint8_t gyroscopeAccuracy, uint8_t accelerometerAccuracy);    
  public:
    LocalBNO(ArmPart* armPart, const uint sdaPin, const uint sclPin, const uint intPin, const uint rstPin);
    LocalBNO(ArmPart *armPart, const uint sckPin, const uint misoPin, const uint mosiPin, const uint csPin, const uint rstPin, const uint intPin);
    bool begin();
    void tare(uint8_t axisMask);
    void saveTare() {
      imu.saveTare();
    }
    void clearTare() {
      imu.clearTare();
    }
    uint writeSystemOrientationQuaternion(float w, float x, float y, float z);
    void hardReset();
    void readSystemOrientationQuaternion() {
      Quaternion quat;
      auto res = imu.readSystemOrientationQuaternion(quat);  
      if (res == 0) {
        printf("System Orientation Quaternion: w=%f, x=%f, y=%f, z=%f\n", quat.real, quat.i, quat.j, quat.k);
        return;
      }
      printf("Failed to read System Orientation Quaternion %d\n", res);
    }
};