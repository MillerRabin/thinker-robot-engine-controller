#include "armClaw.h"

volatile QueueHandle_t ArmClaw::queue;
bool ArmClaw::vl6180xEnabled = true;

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
    /*claw->clawX.tick();   
    claw->clawY.tick();
    claw->clawZ.tick(); 
    claw->clawGripper.tick();
    Euler rEuler = claw->platform.bno.quaternion.getEuler();    
    printf("Platform roll: %f, pitch: %f, yaw: %f\n", rEuler.getRollAngle(), rEuler.getPitchAngle(), rEuler.getYawAngle());
    Euler sEuler = claw->position.quaternion.getEuler();      
    printf("Wrist roll: %f, pitch: %f, yaw: %f\n", sEuler.getRollAngle(), sEuler.getPitchAngle(), sEuler.getYawAngle()); */
    scanI2cTask();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void ArmClaw::scanI2cTask()
{	    
  printf("\nScan started\n");
  i2c_init(i2c1, 400 * 1000);
  gpio_set_function(DETECTORS_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(DETECTORS_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(DETECTORS_SDA_PIN);
  gpio_pull_up(DETECTORS_SCL_PIN);

  
  bi_decl(bi_2pins_with_func(DETECTORS_SDA_PIN, DETECTORS_SCL_PIN, GPIO_FUNC_I2C));
  
  printf("Init detectors\n");
  gpio_init(VL_6180X_SHUT_PIN);
  gpio_set_dir(VL_6180X_SHUT_PIN, GPIO_OUT);
  gpio_init(VL_53LOX_SHUT_PIN);
  gpio_set_dir(VL_53LOX_SHUT_PIN, GPIO_OUT);

  
  gpio_put(VL_6180X_SHUT_PIN, vl6180xEnabled ? 1 : 0);
  
  if (vl6180xEnabled) {
    printf("VL6810X is enabled\n");
  } else {
    printf("VL6810X is disabled\n");
  }
  
  printf("\nI2C Bus Scan\n");
  printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

  vl6180xEnabled = !vl6180xEnabled;  

  for (int addr = 0; addr < (1 << 7); ++addr) {
    if (addr % 16 == 0) {
      printf("%02x ", addr);
    }
        
    int ret;
    uint8_t rxdata;
      if (reserved_addr(addr))
        ret = PICO_ERROR_GENERIC;
      else
        ret = i2c_read_blocking(i2c1, addr, &rxdata, 1, false);
        printf(ret < 0 ? "." : "@");
        printf(addr % 16 == 15 ? "\n" : "  ");
    }
}

bool ArmClaw::reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
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
  const uint detectorsSdaPin, 
  const uint detectorsSclPin, 
  const uint engineXPin,
  const uint engineYPin,
  const uint engineZPin, 
  const uint engineGripperPin,
  const uint canRxPin,
  const uint canTxPin,
  const uint memsRxPin,
  const uint memsTxPin,
  const uint memsRstPin,
  const uint memsIntPin
) :
    ArmPart(canRxPin, canTxPin),    
    clawX(engineXPin, Range(0, 180), Range(-90, 90), IMU_USE_PITCH, 100),
    clawY(engineYPin, Range(0, 180), Range(-90, 90), IMU_USE_PITCH, 100),
    clawZ(engineZPin, Range(0, 180), Range(-90, 90), IMU_USE_PITCH, 100),
    clawGripper(engineGripperPin, Range(0, 180), Range(-90, 90), IMU_USE_PITCH, 100),
    position(this, memsRxPin, memsTxPin, memsRstPin, memsIntPin)
  {            
    ArmClaw::queue = xQueueCreate(10, sizeof(ArmClawQueueParams));
    xTaskCreate(ArmClaw::busReceiverTask, "ArmClaw::busReceiverTask", 1024, this, 1, NULL);
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
}