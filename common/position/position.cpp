#include "position.h"

uint8_t action = 0;
volatile QueueHandle_t queue = NULL;

void compass_callback(uint gpio, uint32_t events) {  
  xQueueSend(queue, &action, 0);
}

/*void Position::readMetadata() {
  rotationVector_Q1 = imu.getQ1(FRS_RECORDID_ROTATION_VECTOR);
  accelerometer_Q1 = imu.getQ1(FRS_RECORDID_ACCELEROMETER);
  printf("rotationVector_Q1: %d, accelerometer_Q1: %d\n", rotationVector_Q1, accelerometer_Q1);
  printf("For rotation vector\n");
  float range = imu.getRange(FRS_RECORDID_ROTATION_VECTOR);
  float resolution = imu.getResolution(FRS_RECORDID_ROTATION_VECTOR);
  printf("Range: %f, Resolution: %f\n", range, resolution);
  printf("Q1: %d, Q2: %d, Q3: %d\n", imu.getQ1(FRS_RECORDID_ROTATION_VECTOR), imu.getQ2(FRS_RECORDID_ROTATION_VECTOR), imu.getQ3(FRS_RECORDID_ROTATION_VECTOR));
      
  printf("\nFor accelerometer\n");
  float aRange = imu.getRange(FRS_RECORDID_ACCELEROMETER);
  float aResolution = imu.getResolution(FRS_RECORDID_ACCELEROMETER);
  printf("Range: %f, Resolution: %f\n", aRange, aResolution);
  
  int16_t aQ1 = imu.getQ1(FRS_RECORDID_ACCELEROMETER);
  int16_t aQ2 = imu.getQ2(FRS_RECORDID_ACCELEROMETER);
  int16_t aQ3 = imu.getQ3(FRS_RECORDID_ACCELEROMETER);
  printf("Q1: %d, Q2: %d, Q3: %d\n", aQ1, aQ2, aQ3);
    
  uint16_t accelerometer_power = imu.readFRSword(FRS_RECORDID_ACCELEROMETER, 3) & 0xFFFF; //Get word 3, lower 16 bits
  float accel_power = imu.qToFloat(accelerometer_power, 10); //Q point is 10 for power
  printf("\nAccelerometer power: %f (mA)\n", accel_power);
}*/


static void compassTask(void* instance) {  
  Position* position = (Position*)instance;  
  BNO080 imu = position->imu;
  
  imu.getReadings();
  float ax, ay, az, gx, gy, gz;
  uint8_t linAccuracy = 0;
  uint8_t gyroAccuracy = 0;
      
  while (true) {
    BaseType_t res = xQueueReceive(queue, &action, 2000 / portTICK_PERIOD_MS);
    uint16_t datatype = imu.getReadings();
    if (res != pdPASS) {      
      printf("Position sending result %d\n", res);
      continue;
    }
    
    if (datatype == SENSOR_REPORTID_ROTATION_VECTOR) {
      if (position->updateQuat(imu.rawQuatI, imu.rawQuatJ, imu.rawQuatK, imu.rawQuatReal, imu.rawQuatRadianAccuracy, imu.quatAccuracy)) {
        printf("qx: %d, qy: %d, qz: %d, qw: %d, quatRadianAccuracy: %d, quatAccuracy: %d\n", 
          imu.rawQuatI, imu.rawQuatJ, imu.rawQuatK, imu.rawQuatReal, imu.rawQuatRadianAccuracy, imu.quatAccuracy);
        int res = position->armPart->sendQuaternon(position->rawQuat);
        printf("Position sending result %d\n", res);
      }      
      //imu->getQuat(qx, qy, qz, qw, quatRadianAccuracy, quatAccuracy);    

      /*float roll = (imu.getRoll()) * 180.0 / PI; // Convert roll to degrees
      float pitch = (imu.getPitch()) * 180.0 / PI; // Convert pitch to degrees
      float yaw = (imu.getYaw()) * 180.0 / PI; // Convert yaw / heading to degrees
      printf("roll: %f, pitch: %f, yaw: %f\n", roll, pitch, yaw);*/
    }

    /*if (datatype == SENSOR_REPORTID_GYROSCOPE) {
      imu.getGyro(gx, gy, gz, gyroAccuracy);      
      //printf("gx: %f, gy: %f, gz: %f, gyroAccuracy: %d\n", gx, gy, gz, gyroAccuracy);
    }*/

    /*if (datatype == SENSOR_REPORTID_LINEAR_ACCELERATION) {
      imu.getLinAccel(ax, ay, az, linAccuracy);
      //printf("ax: %f, ay: %f, az: %f, linAccuracy: %d\n", ax, ay, az, linAccuracy);
    } */ 
  }  
}

bool Position::updateQuat(uint16_t rawQuatI, uint16_t rawQuatJ, uint16_t rawQuatK, uint16_t rawQuatReal, uint16_t rawQuatRadianAccuracy, uint8_t quatAccuracy) {
  uint16_t id = (rawQuatI > rawQuat.i) ? rawQuatI - rawQuat.i : rawQuat.i - rawQuatI;
  uint16_t jd = (rawQuatJ > rawQuat.j) ? rawQuatJ - rawQuat.j : rawQuat.j - rawQuatJ;
  uint16_t kd = (rawQuatK > rawQuat.k) ? rawQuatK - rawQuat.k : rawQuat.k - rawQuatK;
  uint16_t rd = (rawQuatReal > rawQuat.real) ? rawQuatReal - rawQuat.real : rawQuat.real - rawQuatReal;
  rawQuat.i = rawQuatI;
  rawQuat.j = rawQuatJ;
  rawQuat.k = rawQuatK;
  rawQuat.real = rawQuatReal;
  rawQuat.quatRadAcc = rawQuatRadianAccuracy;
  rawQuat.quatAcc = quatAccuracy;  
  return ((id > 1) || (jd > 1) || (kd > 1) || (rd > 1));
}

Position::Position(ArmPart* armPart, const uint sdaPin, const uint sclPin, const uint intPin, const uint rstPin) :
  sdaPin(sdaPin),
  sclPin(sclPin),
  intPin(intPin),
  rstPin(rstPin),
  armPart(armPart)
{   
  printf("Initialize position\n");
  i2c_init(i2c_default, 400 * 1000);
  gpio_set_function(sdaPin, GPIO_FUNC_I2C);
  gpio_set_function(sclPin, GPIO_FUNC_I2C);
  gpio_pull_up(sdaPin);
  gpio_pull_up(sclPin);
  gpio_init(rstPin);
  gpio_set_dir(rstPin, GPIO_OUT);
  gpio_put(rstPin, 1);

  gpio_init(intPin);
  gpio_set_dir(intPin, GPIO_IN);
  gpio_pull_up(intPin);
  gpio_set_irq_enabled_with_callback(intPin, GPIO_IRQ_EDGE_FALL, true, &compass_callback);

  queue = xQueueCreate(1, sizeof(uint8_t));
  xTaskCreate(compassTask, "compassTask", 1024, this, 5, NULL);
  
  bi_decl(bi_2pins_with_func(sdaPin, sclPin, GPIO_FUNC_I2C));
  
  imu.enableDebugging();
  imu.begin(BNO080_DEFAULT_ADDRESS, i2c_default, intPin);
  //readMetadata();
  imu.calibrateAll();
  imu.enableRotationVector(50);  
  imu.enableLinearAccelerometer(50);
  imu.enableGyro(50);
}