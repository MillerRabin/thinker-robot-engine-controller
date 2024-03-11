#include "main.h"
#include <RP2040.h>

extern "C" {
  #include "../common/canBus/src/can2040.h" 
}

#include "../common/BNO080/BNO080.h"
#include <math.h>
#include <hardware/pwm.h>

volatile QueueHandle_t queue = NULL;
#define ID_MASTER_STOP_CMD      0x0A0
#define ID_MASTER_START_CMD     0x0A1
#define ID_MASTER_PING          0x0A2
#define ID_SLAVE_STOP_RESP      0x0B0
#define ID_SLAVE_DATA           0x0B1
#define ID_SLAVE_PING_RESP      0x0B2

#ifndef PI
  #define PI					3.14159265358979f
#endif

#define SDA_PIN 0
#define SCL_PIN 1
#define INT_PIN 10
#define RST_PIN 2
#define SHOULDER_Z_PIN 8
#define SHOULDER_Y_PIN 3

BNO080 myIMU;

bool accReady = false;
bool quatReady = false;

static struct can2040 cbus;

can2040_msg busMsg;
can2040_msg respMsg = {
  .id = ID_SLAVE_PING_RESP,
  .dlc = 0,  
  .data = {0}
};

uint32_t busNotify;
bool busDataReady = false;

static void busTask(void *pvParameters)
{	  
  while (true) {
    if (!busDataReady) {
      vTaskDelay(10 / portTICK_PERIOD_MS);
      continue;
    }
    
    /*if (busMsg.id == ID_MASTER_PING) {
      printf("Ping received, Sending response\n");
      can2040_transmit(&cbus, &respMsg);      
    }*/
    
    printf("data received 0x%x, id: 0x%x\n", busNotify, busMsg.id);
    printf("busMsg 0x%x:%x\n", busMsg.data32[0], busMsg.data32[1]);
                
    busDataReady = false;    
  }  
}

static void engineTask(void *pvParameters)
{	  
  printf("PWM started\n");
  gpio_set_function(SHOULDER_Z_PIN, GPIO_FUNC_PWM);
  gpio_set_function(SHOULDER_Y_PIN, GPIO_FUNC_PWM);
  uint zSlice   = pwm_gpio_to_slice_num(SHOULDER_Z_PIN);
  uint zChannel = pwm_gpio_to_channel(SHOULDER_Z_PIN);  
  uint ySlice   = pwm_gpio_to_slice_num(SHOULDER_Y_PIN);
  uint yChannel = pwm_gpio_to_channel(SHOULDER_Y_PIN);
  
  pwm_set_clkdiv(ySlice, 255.0f);
  pwm_set_clkdiv(zSlice, 255.0f);
  
  pwm_set_wrap(ySlice, 9804);
  pwm_set_wrap(zSlice, 9804);
  
  pwm_set_enabled(ySlice, true);
  pwm_set_enabled(zSlice, true);
  
  
  printf("zSlice: %d, zChannel: %d, ySlice: %d, yChannel: %d\n", zSlice, zChannel, ySlice, yChannel);
  //pwm_config config = pwm_get_default_config();
  //printf("clkdiv: %d\n", config.csr);
  //pwm_config_set_clkdiv(&config, 255.f);
  //printf("clkdiv: %d\n", config.csr);
  
  while(true) {
    printf("Set ySlice to 490\n");
    pwm_set_chan_level(zSlice, zChannel, 490);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    printf("Set ySlice to 550\n");
    pwm_set_chan_level(zSlice, zChannel, 550);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    printf("Set ySlice to 600\n");
    pwm_set_chan_level(zSlice, zChannel, 600);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

/**
 * @brief Generate and print a debug message from a supplied string.
 *
 * @param msg: The base message to which `[DEBUG]` will be prefixed.
 */
void log_debug(const char* msg) {
  uint msg_length = 9 + strlen(msg);
  char* sprintf_buffer = (char*)malloc(msg_length);
  sprintf(sprintf_buffer, "[DEBUG] %s\n", msg);
  printf("%s", sprintf_buffer);
  free(sprintf_buffer);
}

static void can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg) {
  memcpy(&busMsg, msg, sizeof(can2040_msg));
  busNotify = notify;
  busDataReady = true;
}

static void PIOx_IRQHandler(void)
{
    can2040_pio_irq_handler(&cbus);
}

void canbus_setup(void)
{
    log_debug("Can bus setup started");
    uint32_t pio_num = 0;
    uint32_t sys_clock = 125000000, bitrate = 1000000;
    //uint32_t gpio_rx = 9, gpio_tx = 8;
    uint32_t gpio_rx = 4, gpio_tx = 5;
        
    // Setup canbus
    can2040_setup(&cbus, pio_num);
    can2040_callback_config(&cbus, can2040_cb);

    // Enable irqs
    irq_set_exclusive_handler(PIO0_IRQ_0_IRQn, PIOx_IRQHandler);
    NVIC_SetPriority(PIO0_IRQ_0_IRQn, 1);
    NVIC_EnableIRQ(PIO0_IRQ_0_IRQn);

    // Start canbus
    can2040_start(&cbus, sys_clock, bitrate, gpio_rx, gpio_tx);
    log_debug("Can bus setup finished");
}

uint8_t action = 0;

void compass_callback(uint gpio, uint32_t events) {  
  xQueueSend(queue, &action, 0);
}

static void compassTask(void *pvParameters)
{	  
  printf("\nCompass started\n");
  i2c_init(i2c_default, 400 * 1000);
  gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(SDA_PIN);
  gpio_pull_up(SCL_PIN);
  gpio_init(RST_PIN);
  gpio_set_dir(RST_PIN, GPIO_OUT);
  gpio_put(RST_PIN, 1);

  gpio_init(INT_PIN);
  gpio_set_dir(INT_PIN, GPIO_IN);
  gpio_pull_up(INT_PIN);  
  gpio_set_irq_enabled_with_callback(INT_PIN, GPIO_IRQ_EDGE_FALL, true, &compass_callback);
  
  bi_decl(bi_2pins_with_func(SDA_PIN, SCL_PIN, GPIO_FUNC_I2C));

  
  if (!myIMU.begin(BNO080_DEFAULT_ADDRESS, i2c_default, INT_PIN)) {  
    printf("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide\n");
  }
        
  myIMU.enableRotationVector(50);
  //myIMU.enableGameRotationVector(100);
  printf("Rotation game vector enabled\n");
  printf("Output in form roll, pitch, yaw, x, y, z\n");  
  myIMU.getReadings();
  while (true) {
    BaseType_t res = xQueueReceive(queue, &action, portMAX_DELAY);
    uint16_t datatype = myIMU.getReadings();
    if (datatype != SENSOR_REPORTID_ROTATION_VECTOR) 
      continue;    
    float roll = (myIMU.getRoll()) * 180.0 / PI; // Convert roll to degrees
    float pitch = (myIMU.getPitch()) * 180.0 / PI; // Convert pitch to degrees
    float yaw = (myIMU.getYaw()) * 180.0 / PI; // Convert yaw / heading to degrees
    printf("%f, %f, %f\n", roll, pitch, yaw); 
  }  
}

int main() {	
  stdio_init_all();
  myIMU.enableDebugging();
  sleep_ms(10000);
  log_debug("Started");  
  
  canbus_setup();
    	        
  //BaseType_t busStatus = xTaskCreate(busTask, "busTask", 1024, NULL, 5, NULL);
  //BaseType_t scanI2cTaskStatus = xTaskCreate(compassTask, "compassTask", 1024, NULL, 5, NULL);
  BaseType_t engineTaskStatus = xTaskCreate(engineTask, "engineTask", 1024, NULL, 5, NULL);  
  
  queue = xQueueCreate(1, sizeof(uint8_t));
    
  //if ((readTaskStatus == pdPASS) || (writeTaskStatus == pdPASS) || (scanI2cTaskStatus == pdPASS))
  vTaskStartScheduler();  
    
  // We should never get here, but just in case...
  log_debug("Never Here");
  while(true) {
    // NOP
  };
}