/**
 * RP2040 FreeRTOS Template - App #4
 *
 * @copyright 2023, Tony Smith (@smittytone)
 * @version   1.4.2
 * @licence   MIT
 *
 */
#include "main.h"
#include "wit_c_sdk.h"
#include "hardware/uart.h"
#include "hardware/i2c.h"
#include "VL53L0X/Pico_VL53L0X.h"

using std::string;
using std::stringstream;

static char event_str[128];

void gpio_event_string(char *buf, uint32_t events);

#define UART_ID uart0
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define XSHUT_PIN 2
#define INT_PIN 3

#define BUF_SIZE 1024

#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80

static volatile char s_cDataUpdate = 0;
const uint32_t c_uiBaud[10] = {0, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
volatile QueueHandle_t queue = NULL;
Pico_VL53L0X lox = Pico_VL53L0X();
static void AutoScanSensor(void);
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);
static void Usart1Init(uint32_t baud_rate);

static void readDetectorTask(void *pvParameters)
{
	unsigned char ucTemp;	  
	while(1)	
  {		    
    uart_read_blocking(UART_ID, &ucTemp, 1);    
		WitSerialDataIn(ucTemp);
	}
}

static void writeDetectorData(void *pvParameters)
{	
  float fAcc[3], fGyro[3], fAngle[3];
	int i;	
  AutoScanSensor();  
	while (1) {		
    if(s_cDataUpdate) {      
			float rawHeight = sReg[HeightL];
			float height = 4200 + rawHeight;
			bool updated = false;
			for(i = 0; i < 3; i++)
			{
				fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;
				fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;
				fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;
			}
			if(s_cDataUpdate & ACC_UPDATE)
			{
				//printf("acc:%.3f %.3f %.3f", fAcc[0], fAcc[1], fAcc[2]);
				updated = true;
				s_cDataUpdate &= ~ACC_UPDATE;
			}
			if(s_cDataUpdate & GYRO_UPDATE)
			{
				//printf("\tgyro:%.3f %.3f %.3f", fGyro[0], fGyro[1], fGyro[2]);
				updated = true;
				s_cDataUpdate &= ~GYRO_UPDATE;
			}
			if(s_cDataUpdate & ANGLE_UPDATE)
			{
				printf("\tangle:%.3f %.3f %.3f", fAngle[0], fAngle[1], fAngle[2]);
				updated = true;
				s_cDataUpdate &= ~ANGLE_UPDATE;
			}
			if(s_cDataUpdate & MAG_UPDATE)
			{
				//printf("\tmag:%d %d %d", sReg[HX], sReg[HY], sReg[HZ]);
				printf("\theight:%.3f", height);
				updated = true;
				s_cDataUpdate &= ~MAG_UPDATE;
			}
			if (updated) {
				printf("\r\n");				
			}
		}    
	}
}

bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

static void scanI2cTask(void *pvParameters)
{	  
  printf("\nScan started\n");
  i2c_init(i2c_default, 400 * 1000);
  gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
  gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

  bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

  printf("\nI2C Bus Scan\n");
  printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

  for (int addr = 0; addr < (1 << 7); ++addr) {
    if (addr % 16 == 0) {
      printf("%02x ", addr);
    }
        
    int ret;
    uint8_t rxdata;
      if (reserved_addr(addr))
        ret = PICO_ERROR_GENERIC;
      else
        ret = i2c_read_blocking(i2c_default, addr, &rxdata, 1, false);
        printf(ret < 0 ? "." : "@");
        printf(addr % 16 == 15 ? "\n" : "  ");
    }
}

static void startRangeDetector(void *pvParameters)
{	  
  i2c_init(i2c_default, 400 * 1000);
  gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
  gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
      
  bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
  
  if (!lox.begin(VL53L0X_I2C_ADDR, true, i2c_default)) {
    printf("Failed to boot VL53L0X\n");
    return;
  }  
  printf("VL53L0X API Continuous Ranging example\n\n");
  lox.startRangeContinuous();

  while (1) {
    printf("Reading range");
    if (lox.isRangeComplete()) {
      printf("Distance in mm: %d\n", lox.readRange());
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
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

void gpio_callback(uint gpio, uint32_t events) {
  printf("Data ready\n");
}

int main() {	
  stdio_init_all();
  sleep_ms(10000);
  log_debug("Started");
  Usart1Init(9600);    
  log_debug("UART Inited");
  WitInit(WIT_PROTOCOL_NORMAL, 0x50);
  log_debug("Init success");
	WitSerialWriteRegister(SensorUartSend);
	log_debug("Serial registered");
  WitRegisterCallBack(SensorDataUpdata);
  log_debug("Register Callback");
	WitDelayMsRegister(Delayms);

  gpio_init(XSHUT_PIN);
  gpio_set_dir(XSHUT_PIN, GPIO_OUT);
  gpio_put(XSHUT_PIN, 1);
  gpio_init(INT_PIN);
  gpio_set_dir(INT_PIN, GPIO_IN);  
  gpio_set_irq_enabled_with_callback(INT_PIN, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    
  BaseType_t readTaskStatus = xTaskCreate(readDetectorTask, "readDetectorTask", 128, NULL, 5, NULL);
  BaseType_t writeTaskStatus = xTaskCreate(writeDetectorData, "writeDetectorData", 1024, NULL, 5, NULL);
  //BaseType_t scanI2cTaskStatus = xTaskCreate(startRangeDetector, "startRangeDetector", 1024, NULL, 5, NULL);
  //BaseType_t scanI2cTaskStatus = xTaskCreate(scanI2cTask, "scanI2cTask", 1024, NULL, 5, NULL);
  
  queue = xQueueCreate(4, sizeof(uint8_t));
    
  //if ((readTaskStatus == pdPASS) || (writeTaskStatus == pdPASS) || (scanI2cTaskStatus == pdPASS))
  vTaskStartScheduler();  
    
  // We should never get here, but just in case...
  log_debug("Never Here");
  while(true) {
    // NOP
  };


}

static void Usart1Init(uint32_t baud_rate)
{
  uart_init(UART_ID, 9600);  
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
  uart_set_hw_flow(UART_ID, false, false);  
  uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
}

static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{  
  uart_write_blocking(UART_ID, p_data, uiSize);	  
}

static void Delayms(uint16_t usMs)
{
	vTaskDelay(usMs / portTICK_PERIOD_MS);
}

static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{	  
  for(int i = 0; i < uiRegNum; i++){
    switch(uiReg) {
      //case AX:
      //case AY:
      case AZ:
		    s_cDataUpdate |= ACC_UPDATE;
      break;
      //case GX:
      //case GY:
      case GZ:
				s_cDataUpdate |= GYRO_UPDATE;
      break;
      //case HX:
      //case HY:
      case HZ:
				s_cDataUpdate |= MAG_UPDATE;
      break;
      //case Roll:
      //case Pitch:
      case Yaw:
				s_cDataUpdate |= ANGLE_UPDATE;
      break;
      default:
				s_cDataUpdate |= READ_UPDATE;
			break;
    }
		uiReg++;
  }
}

static void AutoScanSensor(void)
{
	int i, iRetry;
  printf("AutoScan Sensor\r\n");
	for(i = 1; i < 10; i++)
	{
		uart_set_baudrate(UART_ID, c_uiBaud[i]);
		iRetry = 2;
		do
		{
			s_cDataUpdate = 0;
			WitReadReg(AX, 3);
			Delayms(100);
			if(s_cDataUpdate != 0)
			{
				printf("%lu baud find sensor\r\n\r\n", c_uiBaud[i]);				
				return ;
			}
			iRetry--;
		}while(iRetry);
	}
	printf("can not find sensor\r\n");
	printf("please check your connection\r\n");
}

