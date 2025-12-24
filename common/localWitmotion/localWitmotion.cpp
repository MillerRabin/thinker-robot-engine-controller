#include "localWitmotion.h"

uint32_t LocalWitmotion::c_uiBaud[10] = {0, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
LocalWitmotion *LocalWitmotion::instance = NULL;
volatile bool LocalWitmotion::dataAvailable = false;

volatile uint8_t LocalWitmotion::uartRxBuffer[512];
volatile uint16_t LocalWitmotion::uartHead = 0;
volatile uint16_t LocalWitmotion::uartTail = 0;
SemaphoreHandle_t LocalWitmotion::uartRxSemaphore;
SemaphoreHandle_t LocalWitmotion::dataReadySemaphore;


void LocalWitmotion::Usart1Init(uint32_t baud_rate) {
  uart_init(MEMS_UART_ID, baud_rate);
  gpio_set_function(MEMS_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(MEMS_RX_PIN, GPIO_FUNC_UART);
  uart_set_hw_flow(MEMS_UART_ID, false, false);
  uart_set_format(MEMS_UART_ID, 8, 1, UART_PARITY_NONE);
    
  /*uartRxSemaphore = xSemaphoreCreateBinary();
  irq_set_exclusive_handler(UART0_IRQ, LocalWitmotion::on_uart_rx);
  irq_set_enabled(UART0_IRQ, true);
  uart_set_irq_enables(MEMS_UART_ID, true, false);*/
}

void LocalWitmotion::on_uart_rx() {  
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  while (uart_is_readable(MEMS_UART_ID)) {
    uint8_t byte = uart_getc(MEMS_UART_ID);
    uint16_t nextHead = (uartHead + 1) % sizeof(uartRxBuffer);
    if (nextHead != uartTail) {
      uartRxBuffer[uartHead] = byte;
      uartHead = nextHead;
    }
  }
  //uartSendCounter++;
  xSemaphoreGiveFromISR(uartRxSemaphore, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void LocalWitmotion::SensorUartSend(uint8_t *p_data, uint32_t uiSize) {  
  uart_write_blocking(MEMS_UART_ID, p_data, uiSize);	  
}

void LocalWitmotion::readDetectorTask(void *pvParameters) {
  unsigned char ucTemp;    
  while (true) {
    uart_read_blocking(MEMS_UART_ID, &ucTemp, 1);
    WitSerialDataIn(ucTemp);
  }

    /*if (xSemaphoreTake(uartRxSemaphore, pdMS_TO_TICKS(IMU_WAIT_TIMEOUT)) == pdTRUE) {
      while (uartHead != uartTail) {
        uint8_t byte = uartRxBuffer[uartTail];
        uartTail = (uartTail + 1) % sizeof(uartRxBuffer);
        WitSerialDataIn(byte);
      }
    } else {
      printf("No data from IMU\n");
      vTaskDelay(pdMS_TO_TICKS(IMU_NO_DATA_TIMEOUT));
    }
  }*/
}

void LocalWitmotion::init(void *pvParameters) {
  uint32_t scanResult = AutoScanSensor();
  if (scanResult == 0) {
    printf("No sensor found\n");
    vTaskDelete(NULL);
    return;
  }

  printf("Sensor found at baud rate: %lu\n", scanResult);
  printf("Set content Mode to Quaternion\n");
  if (WitSetContent( RSW_ACC | RSW_GYRO | RSW_PRESS | RSW_Q) != WIT_HAL_OK) {
		printf("Set RSW Error\n");
    vTaskDelete(NULL);
    return;
  }
  printf("Set content Mode Success\n");
  vTaskDelete( NULL );
}

void LocalWitmotion::SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum) {
  dataAvailable = true;  
  for (int i = 0; i < uiRegNum; i++){
    switch (uiReg) {
      case AZ:
        if (instance == NULL) 
          break;
        
        instance->accelerometer.x = sReg[AX];
        instance->accelerometer.y = sReg[AY];
        instance->accelerometer.z = sReg[AZ];

        if (instance->armPart->updateAccelerometer(instance) != 0) {
          printf("Accelerometer sending error\n");
        }
        break;
      case GZ:
        if (instance == NULL)
          break;
        instance->gyroscope.x = sReg[GX];
        instance->gyroscope.y = sReg[GY];
        instance->gyroscope.z = sReg[GZ];
        if (instance->armPart->updateGyroscope(instance) != 0) {
          printf("Gyro sending error\n");
        }
        break;
      case Yaw:
        break;
      case HeightH:        
        instance->height = (int32_t)((sReg[HeightH] << 16) | sReg[HeightL]);
        instance->temperature = sReg[TEMP];

        if (instance->armPart->updateHeight(instance) != 0) {
          printf("Height sending error\n");
        }
        break;
      case q3:        
        if (instance == NULL)
          break;
        instance->quaternion.real = sReg[q0] / 32768.0f;
        instance->quaternion.i = sReg[q1] / 32768.0f;
        instance->quaternion.j = sReg[q2] / 32768.0f;
        instance->quaternion.k = sReg[q3] / 32768.0f;
        {
          float norm = sqrtf(
            instance->quaternion.real * instance->quaternion.real +
            instance->quaternion.i * instance->quaternion.i +
            instance->quaternion.j * instance->quaternion.j +
            instance->quaternion.k * instance->quaternion.k);
          if (norm > 0.0001f) {
            instance->quaternion.real = instance->quaternion.real / norm;
            instance->quaternion.i = instance->quaternion.i / norm;
            instance->quaternion.j = instance->quaternion.j / norm;
            instance->quaternion.k = instance->quaternion.k / norm;
          }
        }
        if (instance->armPart->updateQuaternion(instance) != 0) {
          printf("quat sending error\n");
        }        
        break;
      default:        
        break;
    }
    uiReg++;
  }
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(dataReadySemaphore, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void LocalWitmotion::Delayms(uint16_t usMs) {
	vTaskDelay(pdMS_TO_TICKS(usMs));
}

uint32_t LocalWitmotion::AutoScanSensor(void) {
	int i, iRetry;  
	for(i = 1; i < 10; i++) {
		uart_set_baudrate(MEMS_UART_ID, c_uiBaud[i]);
		iRetry = 2;
		do {
			dataAvailable = false;
			WitReadReg(AX, 3);
			Delayms(100);
			if (dataAvailable) {				
        return c_uiBaud[i];
      }
			iRetry--;
		} while(iRetry);
	}
	return 0;
}

LocalWitmotion::LocalWitmotion(ArmPart *armPart, const uint memsRxPin, const uint memsTxPin, const uint memsRstPin) : 
  armPart(armPart),
  memsRxPin(memsRxPin),
  memsTxPin(memsTxPin)
{
  LocalWitmotion::instance = this;
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
  dataReadySemaphore = xSemaphoreCreateBinary();
  BaseType_t readTaskStatus = xTaskCreate(readDetectorTask, "LocalWitmotion::readDetectorTask", 1024, NULL, 5, NULL);
  BaseType_t initTaskStatus = xTaskCreate(init, "LocalWitmotion::init", 1024, NULL, 5, NULL);
}