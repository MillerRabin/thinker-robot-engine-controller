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
    static uint32_t notificationIndex;
    static TaskHandle_t compassTaskHandle;
    bool beginI2C();
    bool beginSPI();
    Quaternion rotate;
    // Fusion check state - see fusionDiverged().
    float fusionErrorDeg = 0.0f;       // angle between the quaternion's gravity and fusionAccel
    Vector3 fusionAccel{0.0f, 0.0f, 0.0f}; // low-passed accelerometer vector (corrected frame)
    bool fusionAccelValid = false;
    float fusionErrorMaxDeg = 0.0f;    // peak of fusionErrorDeg since the last "BNO reads" log line
    // The same angle with both vectors low-passed alike - see getFusionDriftDeg().
    Vector3 driftAccel{0.0f, 0.0f, 0.0f};   // unit vectors
    Vector3 driftGravity{0.0f, 0.0f, 0.0f};
    bool driftValid = false;
    float fusionDriftDeg = 0.0f;
    float fusionDriftMaxDeg = 0.0f;    // peak since the last "BNO reads" log line
    TickType_t fusionCheckFrom = 0;    // no check before this tick (the fusion settling after a reset)
    TickType_t fusionLastCheck = 0;
    TickType_t fusionBadSince = 0;     // 0 = fusionErrorDeg is under the limit
    TickType_t fusionLastReset = 0;
    int fusionResets = 0;              // resets within FUSION_RESET_WINDOW of each other
    volatile bool resetRequested = false; // see requestReset()
    bool fusionDiverged(const Accelerometer &acc, TickType_t now);
  protected:
    void initIMU();
    ArmPart *armPart;
    BNO080 imu;
    int16_t accelerometer_Q1;        
    SemaphoreHandle_t quaternionSemaphore;
    bool updateAccuracy(uint16_t quaternionRadianAccuracy, uint8_t quaternionAccuracy, uint8_t gyroscopeAccuracy, uint8_t accelerometerAccuracy);    
  public:
    // The angle between the quaternion's gravity and the (low-passed) accelerometer's, deg - see
    // fusionDiverged(). 0 until FUSION_SETTLE_TIME after a reset.
    float getFusionErrorDeg() const { return fusionErrorDeg; }
    // The same, with the quaternion's gravity through the same low-pass as the accelerometer's and a
    // slower one (FUSION_DRIFT_TAU_SEC): fusionErrorDeg compares the quaternion now with the
    // accelerometer ~0.1 s ago, so it grew with the joint's speed - measured live (2026-11-19, host
    // moves from the park pose) up to 2.3deg on the shoulder, 4.3 on the elbow and 12.5 on the wrist
    // (78deg in 4 s), plus the swing's own acceleration at the end of the arm. What stays here is the
    // quaternion turning away from gravity for good - a BNO integrating about the wrong axis. For the
    // stream correction's pause (Servo::setFusionError()). 0 until FUSION_SETTLE_TIME after a reset.
    float getFusionDriftDeg() const { return fusionDriftDeg; }
    // Reset the BNO from its own task at its next report, the way fusionDiverged() does (the part's
    // onIMUReset() follows).
    void requestReset() { resetRequested = true; }
    void setRotate(Quaternion rotate) {
      this->rotate = rotate;
    }
    // quaternion is stored already rotated; a raw-frame reading (the accelerometer) needs this too.
    Quaternion getRotate() const { return rotate; }
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
    bool isPositionOK();
    uint getRefreshInterval();
};