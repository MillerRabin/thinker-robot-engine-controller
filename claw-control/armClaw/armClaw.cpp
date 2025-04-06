#include "armClaw.h"

volatile QueueHandle_t ArmClaw::queue;
int counter = 0;

void ArmClawQueueParams::set(uint8_t data[]) {
  uint16_t clawX;
  uint16_t clawY;
  uint16_t clawZ;
  uint16_t clawGripper;
  
  memcpy(&clawX, &data[0], 2);
  memcpy(&clawY, &data[2], 2);
  memcpy(&clawZ, &data[4], 2);
  memcpy(&clawGripper, &data[6], 2);
}

void ArmClaw::engineTask(void *instance) {  
  ArmClaw* claw = (ArmClaw*)instance;
  while(true) {        
    //counter++;
    //printf("engine task %d\n", counter);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void ArmClaw::busReceiverTask(void *instance) {  
  ArmClaw* claw = (ArmClaw*)instance;
  while(true) {        
    ArmClawQueueParams params;    
    xQueueReceive(ArmClaw::queue, &params, portMAX_DELAY);        
    claw->clawX.setTargetAngle(params.clawX); 
    claw->clawY.setTargetAngle(params.clawY);
    claw->clawZ.setTargetAngle(params.clawZ);
  }
}

ArmClaw::ArmClaw(
  const uint8_t detectorsSdaPin, 
  const uint8_t detectorsSclPin, 
  const uint8_t engineXPin,
  const uint8_t engineYPin,
  const uint8_t engineZPin, 
  const uint8_t engineGripperPin,
  const uint8_t canRxPin,
  const uint8_t canTxPin,
  const uint8_t memsRxPin,
  const uint8_t memsTxPin,
  const uint8_t memsRstPin,
  const uint8_t memsIntPin,
  const uint8_t shortDetectorShutPin,
  const uint8_t longDetectorShutPin
) :
    ArmPart(canRxPin, canTxPin),    
    clawX(engineXPin, Range(0, 180), Range(-90, 90), IMU_USE_PITCH, 100),
    clawY(engineYPin, Range(0, 180), Range(-90, 90), IMU_USE_PITCH, 100),
    clawZ(engineZPin, Range(0, 180), Range(-90, 90), IMU_USE_PITCH, 100),
    clawGripper(engineGripperPin, Range(0, 180), Range(-90, 90), IMU_USE_PITCH, 100),
    position(this, memsRxPin, memsTxPin, memsRstPin, memsIntPin),
    rangeDetector(this, i2c1, longDetectorShutPin, shortDetectorShutPin) {            
    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(detectorsSdaPin, GPIO_FUNC_I2C);
    gpio_set_function(detectorsSclPin, GPIO_FUNC_I2C);
    gpio_pull_up(detectorsSdaPin);
    gpio_pull_up(detectorsSclPin);
    bi_decl(bi_2pins_with_func(DETECTORS_SDA_PIN, DETECTORS_SCL_PIN, GPIO_FUNC_I2C));    
    ArmClaw::queue = xQueueCreate(10, sizeof(ArmClawQueueParams));
    xTaskCreate(ArmClaw::busReceiverTask, "ArmClaw::busReceiverTask", 1024, this, 5, NULL);
    xTaskCreate(ArmClaw::engineTask, "ArmClaw::engineTask", 1024, this, 5, NULL);
}

int ArmClaw::updateQuaternion(BasePosition* position) {    
  return ArmPart::updateQuaternion(position->quaternion);
}

int ArmClaw::updateGyroscope(BasePosition* position) {
  return ArmPart::updateGyroscope(position->gyroscope);
}

int ArmClaw::updateAccelerometer(BasePosition* position) {
  return ArmPart::updateAccelerometer(position->accelerometer);
}

int ArmClaw::updateAccuracy(BasePosition* position) {
  return ArmPart::updateAccuracy(position->accuracy);
}

void ArmClaw::busReceiveCallback(can2040_msg frame) {
  if (frame.id == CAN_CLAW_SET_XYZG_DEGREE) {
    ArmClawQueueParams params;
    params.set(frame.data);    
    xQueueSend(ArmClaw::queue, &params, 0);
  }
    
  if (frame.id == CAN_CLAW_FIRMWARE_UPGRADE) {
    rebootInBootMode();
  }
}