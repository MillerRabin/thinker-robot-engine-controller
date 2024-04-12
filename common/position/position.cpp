#include "position.h"

uint8_t action = 0;
volatile QueueHandle_t queue = NULL;

void Position::compassCallback(uint gpio, uint32_t events) {    
  xQueueSend(queue, &action, 0);
}

void Position::compassTask(void* instance) {
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
      if (position->updateQuaternionData(imu.rawQuatI, imu.rawQuatJ, imu.rawQuatK, imu.rawQuatReal)) {        
        /*printf("qx: %d, qy: %d, qz: %d, qw: %d\n", 
          position->quaternion.i, position->quaternion.j, position->quaternion.k, position->quaternion.real);*/
        if (position->armPart->sendQuaternion(position->quaternion) != 0) {
          printf("quat sendind error\n");
        }
      }

      if (position->updateAccuracy(imu.rawQuatRadianAccuracy, imu.quatAccuracy, imu.gyroAccuracy, imu.accelLinAccuracy)) {
        /*printf("Quaternion: rawQuatRadianAccuracy: %d, quatAccuracy: %d, gyroAccuracy: %d, lineAccelAccuracy: %d\n", 
          position->accuracy.quaternionRadAccuracy, 
          position->accuracy.quaternionAccuracy, 
          position->accuracy.gyroscopeAccuracy, 
          position->accuracy.accelerometerAccuracy
        );*/
        if (position->armPart->sendAccuracy(position->accuracy) != 0) {
          printf("Quaternion accuracy sendind error\n");
        };
      }
    
      //, imu.rawQuatRadianAccuracy, imu.quatAccuracy
      //imu->getQuat(qx, qy, qz, qw, quatRadianAccuracy, quatAccuracy);    

      float roll = (imu.getRoll()) * 180.0 / PI; // Convert roll to degrees
      float pitch = (imu.getPitch()) * 180.0 / PI; // Convert pitch to degrees
      float yaw = (imu.getYaw()) * 180.0 / PI; // Convert yaw / heading to degrees
      printf("roll: %f, pitch: %f, yaw: %f\n", roll, pitch, yaw);
    }

    if (datatype == SENSOR_REPORTID_GYROSCOPE) {
      if (position->updateGyroscopeData(imu.rawGyroX, imu.rawGyroY, imu.rawGyroZ)) {
       /*printf("gyroX: %d, gyroY: %d, gyroZ: %d\n", 
          position->gyroscope.x, position->gyroscope.y, position->gyroscope.z);*/          
        if (position->armPart->sendGyroscope(position->gyroscope) != 0) {
          printf("Gyro sending error\n");
        }
      }

      if (position->updateAccuracy(imu.rawQuatRadianAccuracy, imu.quatAccuracy, imu.gyroAccuracy, imu.accelLinAccuracy)) {
        /*printf("Gyroscope: rawQuatRadianAccuracy: %d, quatAccuracy: %d, gyroAccuracy: %d, lineAccelAccuracy: %d\n", 
          position->accuracy.quaternionRadAccuracy, 
          position->accuracy.quaternionAccuracy, 
          position->accuracy.gyroscopeAccuracy, 
          position->accuracy.accelerometerAccuracy
        );*/
        if (position->armPart->sendAccuracy(position->accuracy) != 0) {
          printf("Gyro Accuracy senging error\n");
        }
      }
    }

    if (datatype == SENSOR_REPORTID_LINEAR_ACCELERATION) {
      if (position->updateAccelerometerData(imu.rawLinAccelX, imu.rawLinAccelY, imu.rawLinAccelZ)) {
        /*printf("accX: %d, accY: %d, accZ: %d\n", 
          position->accelerometer.x, position->accelerometer.y, position->accelerometer.z);*/
        
        if (position->armPart->sendAccelerometer(position->accelerometer) != 0) {
          printf("Accelerometer sending error\n");
        }
      }

      if (position->updateAccuracy(imu.rawQuatRadianAccuracy, imu.quatAccuracy, imu.gyroAccuracy, imu.accelLinAccuracy)) {
        /*printf("Acceleration: rawQuatRadianAccuracy: %d, quatAccuracy: %d, gyroAccuracy: %d, lineAccelAccuracy: %d\n", 
          position->accuracy.quaternionRadAccuracy, 
          position->accuracy.quaternionAccuracy, 
          position->accuracy.gyroscopeAccuracy, 
          position->accuracy.accelerometerAccuracy
        );*/
        if (position->armPart->sendAccuracy(position->accuracy) != 0) {
          printf("Accelerometer Accuracy sending error");
        }
      }      
    }
  }  
}

bool Position::updateQuaternionData(uint16_t rawQuatI, uint16_t rawQuatJ, uint16_t rawQuatK, uint16_t rawQuatReal) {
  uint16_t id = (rawQuatI > quaternion.i) ? rawQuatI - quaternion.i : quaternion.i - rawQuatI;
  uint16_t jd = (rawQuatJ > quaternion.j) ? rawQuatJ - quaternion.j : quaternion.j - rawQuatJ;
  uint16_t kd = (rawQuatK > quaternion.k) ? rawQuatK - quaternion.k : quaternion.k - rawQuatK;
  uint16_t rd = (rawQuatReal > quaternion.real) ? rawQuatReal - quaternion.real : quaternion.real - rawQuatReal;
  quaternion.i = rawQuatI;
  quaternion.j = rawQuatJ;
  quaternion.k = rawQuatK;
  quaternion.real = rawQuatReal;
  return ((id > 1) || (jd > 1) || (kd > 1) || (rd > 1));
}

bool Position::updateAccelerometerData(uint16_t rawAccX, uint16_t rawAccY, uint16_t rawAccZ) {
  uint16_t xd = (rawAccX > accelerometer.x) ? rawAccX - accelerometer.x : accelerometer.x - rawAccX;
  uint16_t yd = (rawAccY > accelerometer.y) ? rawAccY - accelerometer.y : accelerometer.y - rawAccY;
  uint16_t zd = (rawAccZ > accelerometer.z) ? rawAccZ - accelerometer.z : accelerometer.z - rawAccZ;  
  accelerometer.x = rawAccX;
  accelerometer.y = rawAccY;
  accelerometer.z = rawAccZ;  
  return ((xd > 1) || (yd > 1) || (zd > 1));
}

bool Position::updateGyroscopeData(uint16_t rawGyroX, uint16_t rawGyroY, uint16_t rawGyroZ) {
  uint16_t xd = (rawGyroX > gyroscope.x) ? rawGyroX - gyroscope.x : gyroscope.x - rawGyroX;
  uint16_t yd = (rawGyroY > gyroscope.y) ? rawGyroY - gyroscope.y : gyroscope.y - rawGyroY;
  uint16_t zd = (rawGyroZ > gyroscope.z) ? rawGyroZ - gyroscope.z : gyroscope.z - rawGyroZ;  
  gyroscope.x = rawGyroX;
  gyroscope.y = rawGyroY;
  gyroscope.z = rawGyroZ;  
  return ((xd > 1) || (yd > 1) || (zd > 1));
}

bool Position::updateAccuracy(uint16_t quaternionRadianAccuracy, uint8_t quaternionAccuracy, uint8_t gyroscopeAccuracy, uint8_t accelerometerAccuracy) {
  bool rd = (accuracy.quaternionRadAccuracy != quaternionRadianAccuracy);
  bool qd = (accuracy.quaternionAccuracy != quaternionAccuracy);
  bool gd = (accuracy.gyroscopeAccuracy != gyroscopeAccuracy);
  bool ad = (accuracy.accelerometerAccuracy != accelerometerAccuracy);
  accuracy.accelerometerAccuracy = accelerometerAccuracy;
  accuracy.quaternionAccuracy = quaternionAccuracy;
  accuracy.quaternionRadAccuracy = quaternionRadianAccuracy;  
  accuracy.gyroscopeAccuracy = gyroscopeAccuracy;
  return ad || rd || qd || gd;
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
  gpio_set_irq_enabled_with_callback(intPin, GPIO_IRQ_EDGE_FALL, true, &Position::compassCallback);

  queue = xQueueCreate(1, sizeof(uint8_t));
  xTaskCreate(Position::compassTask, "Position::compassTask", 1024, this, 5, NULL);
  
  bi_decl(bi_2pins_with_func(sdaPin, sclPin, GPIO_FUNC_I2C));
    
  imu.begin(BNO080_DEFAULT_ADDRESS, i2c_default, intPin);  
  imu.calibrateAll();
  imu.enableRotationVector(50);  
  imu.enableLinearAccelerometer(50);
  imu.enableGyro(50);
}