#include "WitMotion.h"

volatile char WitMotion::s_cDataUpdate = 0;
uint32_t WitMotion::c_uiBaud[10] = {0, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};

static void Usart1Init(uint32_t baud_rate)
{
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

void WitMotion::writeDetectorData(void *pvParameters) {	
  float fAcc[3], fGyro[3], fAngle[3], q[4];
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
        q[0] = sReg[q0];
        q[1] = sReg[q1];
        q[2] = sReg[q2];
        q[3] = sReg[q3];
				printf("\tangle:%.3f %.3f %.3f\n", fAngle[0], fAngle[1], fAngle[2]);
        printf("\tQ:%.3f %.3f %.3f %.3f\n", q[0], q[1], q[2], q[3]);
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

void WitMotion::SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
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
  BaseType_t writeTaskStatus = xTaskCreate(writeDetectorData, "writeDetectorData", 1024, NULL, 5, NULL);
  //BaseType_t scanI2cTaskStatus = xTaskCreate(startRangeDetector, "startRangeDetector", 1024, NULL, 5, NULL);
  //BaseType_t scanI2cTaskStatus = xTaskCreate(scanI2cTask, "scanI2cTask", 1024, NULL, 5, NULL);
  queue = xQueueCreate(4, sizeof(uint8_t));
}