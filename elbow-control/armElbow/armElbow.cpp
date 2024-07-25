#include "armElbow.h"

volatile QueueHandle_t ArmElbow::queue;
BMP280 ArmElbow::bmp;

void ArmElbow::engineTask(void *instance) {  
  ArmElbow* elbow = (ArmElbow*)instance;
  
  while(true) {                
    //I2CScan::scan(i2c_default);
    float alt = bmp.readAltitude();
    printf("Altitude is %f\n", alt);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void ArmElbow::busReceiverTask(void *instance) {  
  ArmElbow* elbow = (ArmElbow*)instance;
  while(true) {        
    ArmElbowQueueParams params;    
    xQueueReceive(ArmElbow::queue, &params, portMAX_DELAY);
    elbow->elbowY.setTargetAngle(params.elbowY);
  }
}

ArmElbow::ArmElbow(
  const uint memsSdaPin, 
  const uint memsSclPin, 
  const uint memsIntPin, 
  const uint memsRstPin,
  const uint engineYPin, 
  const uint canRxPin,
  const uint canTxPin) :
    ArmPart(canRxPin, canTxPin),    
    elbowY(engineYPin, Range(0, 180), Range(-90, 90), IMU_USE_PITCH, 100)    
  {            
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(memsSdaPin, GPIO_FUNC_I2C);
    gpio_set_function(memsSclPin, GPIO_FUNC_I2C);
    gpio_pull_up(memsSdaPin);
    gpio_pull_up(memsSclPin);

    Sensor *bmp_temp = bmp.getTemperatureSensor();
    Sensor *bmp_pressure = bmp.getPressureSensor();
      
    printf("BMP280 Sensor event test");

    unsigned status;
    status = bmp.begin(BMP280_ADDRESS_ALT);    
    if (!status) {
      printf("Could not find a valid BMP280 sensor, check wiring or try a different address!\n");
      printf("SensorID was: 0x%x\n",bmp.sensorID());
      printf("  ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
      printf("  ID of 0x56-0x58 represents a BMP 280,\n");
      printf("  ID of 0x60 represents a BME 280.\n");
      printf("  ID of 0x61 represents a BME 680.\n");
      return;
    }

    bmp.setSampling(BMP280::MODE_NORMAL,     /* Operating Mode. */
                    BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    BMP280::FILTER_X16,      /* Filtering. */
                    BMP280::STANDBY_MS_500); /* Standby time. */
    bmp_temp->printSensorDetails();    
    ArmElbow::queue = xQueueCreate(10, sizeof(ArmElbowQueueParams));
    xTaskCreate(ArmElbow::busReceiverTask, "ArmElbow::busReceiverTask", 1024, this, 1, NULL);
    xTaskCreate(ArmElbow::engineTask, "ArmElbow::engineTask", 1024, this, tskIDLE_PRIORITY, NULL);
}

int ArmElbow::updateQuaternion(BasePosition* position) {  
  Euler euler = position->quaternion.getEuler();  
  //printf("Euler get pitch angle: %f\n", euler.getPitchAngle());
  elbowY.euler = euler;  
  return ArmPart::updateQuaternion(position->quaternion);
}

int ArmElbow::updateGyroscope(BasePosition* position) {
  return ArmPart::updateGyroscope(position->gyroscope);
}

int ArmElbow::updateAccelerometer(BasePosition* position) {
  return ArmPart::updateAccelerometer(position->accelerometer);
}

int ArmElbow::updateAccuracy(BasePosition* position) {
  return ArmPart::updateAccuracy(position->accuracy);
}

void ArmElbow::busReceiveCallback(can2040_msg frame) {
  if (frame.id == CAN_ELBOW_SET_Y_DEGREE) {    
    ArmElbowQueueParams params;    
    memcpy(&params.elbowY, &frame.data32[0], 4);    
    xQueueSend(ArmElbow::queue, &params, 0);
  }
}