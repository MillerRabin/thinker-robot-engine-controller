#include "main.h"

extern "C" {
  #include "../common/canBus/src/can2040.h" 
}

#include "../common/BNO080/BNO080.h"
#include "../common/servo/servo.h"
#include "../common/position/position.h"

#include <math.h>
#include <hardware/pwm.h>

#define ID_MASTER_STOP_CMD      0x0A0
#define ID_MASTER_START_CMD     0x0A1
#define ID_MASTER_PING          0x0A2
#define ID_SLAVE_STOP_RESP      0x0B0
#define ID_SLAVE_DATA           0x0B1
#define ID_SLAVE_PING_RESP      0x0B2

#define SDA_PIN 0
#define SCL_PIN 1
#define INT_PIN 10
#define RST_PIN 2
#define SHOULDER_Z_PIN 8
#define SHOULDER_Y_PIN 3

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
  Servo shoulderZ = Servo(SHOULDER_Z_PIN, 270, 100);
  Servo shoulderY = Servo(SHOULDER_Y_PIN, 180, 100);
    
  while(true) {        
    shoulderZ.setDegree(135);    
    shoulderY.setDegree(90);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

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

int main() {	
  stdio_init_all();
  sleep_ms(10000);
  log_debug("Started");  
  
  canbus_setup();
  Position position(SDA_PIN, SCL_PIN, INT_PIN, RST_PIN);
    	        
  //BaseType_t busStatus = xTaskCreate(busTask, "busTask", 1024, NULL, 5, NULL);
  
  BaseType_t engineTaskStatus = xTaskCreate(engineTask, "engineTask", 1024, NULL, 5, NULL);  
        
  //if ((readTaskStatus == pdPASS) || (writeTaskStatus == pdPASS) || (scanI2cTaskStatus == pdPASS))
  vTaskStartScheduler();  
    
  // We should never get here, but just in case...
  log_debug("Never Here");
  while(true) {
    // NOP
  };
}