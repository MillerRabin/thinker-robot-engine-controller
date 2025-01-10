#include "WitMotion.h"

uint32_t WitMotion::c_uiBaud[10] = {0, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
volatile bool WitMotion::dataAvailable = false;

volatile uint16_t WitMotion::rawAccX = 0;
volatile uint16_t WitMotion::rawAccY = 0;
volatile uint16_t WitMotion::rawAccZ = 0;
volatile uint16_t WitMotion::rawGyrX = 0;
volatile uint16_t WitMotion::rawGyrY = 0;
volatile uint16_t WitMotion::rawGyrZ = 0;
volatile uint16_t WitMotion::rawRoll = 0;
volatile uint16_t WitMotion::rawPitch = 0;
volatile uint16_t WitMotion::rawYaw = 0;
volatile uint32_t WitMotion::rawHeight = 0;

static void Usart1Init(uint32_t baud_rate) {
  uart_init(MEMS_UART_ID, 9600);  
  gpio_set_function(MEMS_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(MEMS_RX_PIN, GPIO_FUNC_UART);
  uart_set_hw_flow(MEMS_UART_ID, false, false);  
  uart_set_format(MEMS_UART_ID, 8, 1, UART_PARITY_NONE);
}

void WitMotion::SensorUartSend(uint8_t *p_data, uint32_t uiSize) {  
  uart_write_blocking(MEMS_UART_ID, p_data, uiSize);	  
}

void WitMotion::readDetectorTask(void *pvParameters) {
	unsigned char ucTemp;	  
	while(1)	
  {		    
    uart_read_blocking(MEMS_UART_ID, &ucTemp, 1);    
		WitSerialDataIn(ucTemp);
	}
}

void WitMotion::SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{	  
  for(int i = 0; i < uiRegNum; i++){
    switch(uiReg) {
      //case AX:
      //case AY:
      case AZ:
		   rawAccX = sReg[AX];
       rawAccY = sReg[AY];
       rawAccZ = sReg[AZ];
       dataAvailable = true;
      break;
      //case GX:
      //case GY:
      case GZ:
       rawGyrX = sReg[GX];
       rawGyrY = sReg[GY];
       rawGyrZ = sReg[GZ];
       dataAvailable = true;
      break;
      //case Roll:
      //case Pitch:
      case Yaw:
				rawRoll = sReg[Roll];
        rawPitch = sReg[Pitch];
        rawYaw = sReg[Yaw];
        dataAvailable = true;
      break;
      //case PressureL
      case PressureH:
        rawHeight = (uint32_t)sReg[PressureH] << 16 | sReg[PressureL];
        dataAvailable = true;
      default:
				dataAvailable = true;
			break;
    }
		uiReg++;
  }
}

void WitMotion::Delayms(uint16_t usMs) {
	vTaskDelay(usMs / portTICK_PERIOD_MS);
}

void WitMotion::gpio_callback(uint gpio, uint32_t events) {
  printf("Data ready\n");
}

void WitMotion::AutoScanSensor(void) {
	int i, iRetry;
  printf("AutoScan Sensor\r\n");
	for(i = 1; i < 10; i++)
	{
		uart_set_baudrate(MEMS_UART_ID, c_uiBaud[i]);
		iRetry = 2;
		do
		{
			dataAvailable = false;
			WitReadReg(AX, 3);
			Delayms(100);
			if(dataAvailable)
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

WitMotion::WitMotion(const uint memsRxPin, const uint memsTxPin, const uint memsRstPin, const uint memsIntPin) :
  memsRxPin(memsRxPin),
  memsTxPin(memsTxPin) {
  Usart1Init(9600);    
  printf("UART Inited\n");
  WitInit(WIT_PROTOCOL_NORMAL, 0x50);
  printf("Init success\n");
	WitSerialWriteRegister(SensorUartSend);
	printf("Serial registered\n");
  WitRegisterCallBack(SensorDataUpdata);
  printf("Register Callback\n");
	WitDelayMsRegister(Delayms);

  gpio_init(memsRstPin);
  gpio_set_dir(memsRstPin, GPIO_OUT);
  gpio_put(memsRstPin, 1);
  gpio_init(memsIntPin);
  gpio_set_dir(memsIntPin, GPIO_IN);
  gpio_set_irq_enabled_with_callback(memsIntPin, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    
  BaseType_t readTaskStatus = xTaskCreate(readDetectorTask, "readDetectorTask", 128, NULL, 5, NULL);
}