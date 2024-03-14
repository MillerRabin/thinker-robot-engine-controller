#include "position.h"

uint8_t action = 0;
volatile QueueHandle_t queue = NULL;

void compass_callback(uint gpio, uint32_t events) {  
  xQueueSend(queue, &action, 0);
}

static void compassTask(void* instance) {  
  BNO080 *imu = (BNO080*)instance;
  imu->getReadings();
  while (true) {
    BaseType_t res = xQueueReceive(queue, &action, portMAX_DELAY);
    uint16_t datatype = imu->getReadings();
    if (datatype != SENSOR_REPORTID_ROTATION_VECTOR) 
      continue;    
    float roll = (imu->getRoll()) * 180.0 / PI;
    float pitch = (imu->getPitch()) * 180.0 / PI;
    float yaw = (imu->getYaw()) * 180.0 / PI;
    printf("%f, %f, %f\n", roll, pitch, yaw); 
  }  
}

Position::Position(const uint sdaPin, const uint sclPin, const uint intPin, const uint rstPin) :
  sdaPin(sdaPin),
  sclPin(sclPin),
  intPin(intPin),
  rstPin(rstPin)
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
  
  if (!imu.begin(BNO080_DEFAULT_ADDRESS, i2c_default, intPin)) {  
    printf("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide\n");
  }        
  imu.enableRotationVector(50);  
}