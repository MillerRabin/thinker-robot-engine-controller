#include "localWitmotion.h"

uint32_t LocalWitmotion::c_uiBaud[10] = {0, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
LocalWitmotion *LocalWitmotion::instance = NULL;
volatile bool LocalWitmotion::dataAvailable = false;

volatile uint8_t LocalWitmotion::uartRxBuffer[512];
volatile uint16_t LocalWitmotion::uartHead = 0;
volatile uint16_t LocalWitmotion::uartTail = 0;

TaskHandle_t LocalWitmotion::readTaskHandle = NULL;
TaskHandle_t LocalWitmotion::initTaskHandle = NULL;

void LocalWitmotion::Usart1Init(uint32_t baud_rate) {
  uart_init(MEMS_UART_ID, baud_rate);
  gpio_set_function(MEMS_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(MEMS_RX_PIN, GPIO_FUNC_UART);
  uart_set_hw_flow(MEMS_UART_ID, false, false);
  uart_set_format(MEMS_UART_ID, 8, 1, UART_PARITY_NONE);    
}

void LocalWitmotion::SensorUartSend(uint8_t *p_data, uint32_t uiSize) {  
  uart_write_blocking(MEMS_UART_ID, p_data, uiSize);	  
}

void LocalWitmotion::readDetectorTask(void* instance) {
  unsigned char ucTemp;    
  while (true) {    
    uart_read_blocking(MEMS_UART_ID, &ucTemp, 1);    
    WitSerialDataIn(ucTemp);
  }    
}

void LocalWitmotion::init(void* instance) {
  auto *imu = static_cast<LocalWitmotion*>(instance);
  // DIAG: uses LogQueue::Log (bounded, non-blocking), not printf() - a blocking printf() with no
  // USB CDC reader attached at boot can wedge this task forever (same root cause as an earlier,
  // already-fixed shoulder issue - see project memory item #2), which would silently prevent the
  // sensor from ever initializing on every subsequent boot too, since this task only used to run once.
  // DIAG: retries indefinitely on failure instead of a single attempt-then-delete - a one-shot
  // attempt made it impossible to observe (had to win a race attaching a log reader before the task
  // gave up and deleted itself) and meant a transient/intermittent connection issue at boot would
  // permanently disable the sensor for that whole power cycle. Remove the retry loop once claw's
  // IMU connection issue is confirmed fixed and this is no longer needed for diagnosis.
  uint32_t attempt = 0;
  while (true) {
    attempt++;
    LogQueue::Log("LocalWitmotion::init: attempt %lu, starting AutoScanSensor\n", attempt);
    uint32_t scanResult = AutoScanSensor();
    if (scanResult == 0) {
      LogQueue::Log("LocalWitmotion::init: attempt %lu, no sensor found, retrying in 3s\n", attempt);
      Delayms(3000);
      continue;
    }

    LogQueue::Log("LocalWitmotion::init: attempt %lu, sensor found at baud rate: %lu\n", attempt, scanResult);
    imu->setRefreshRate();
    imu->setBandwidth();
    imu->set6AxisMode();

    if (WitSetContent( RSW_ACC | RSW_GYRO | RSW_PRESS | RSW_Q) != WIT_HAL_OK) {
      LogQueue::Log("LocalWitmotion::init: attempt %lu, Set RSW Error, retrying in 3s\n", attempt);
      Delayms(3000);
      continue;
    }
    LogQueue::Log("LocalWitmotion::init: attempt %lu, Set content Mode Success\n", attempt);
    break;
  }
  vTaskDelete(NULL);
}

void LocalWitmotion::SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum) {
  dataAvailable = true;  
  for (int i = 0; i < uiRegNum; i++){
    switch (uiReg) {
      case AZ: {
        if (instance == NULL) 
          break;
        
        Accelerometer acc;
        acc.fromWitmotion(sReg[AX], sReg[AY], sReg[AZ]);        
        instance->accelerometer.store(acc);

        if (instance->armPart->updateAccelerometer(instance) != 0) {
          LogQueue::Log("Accelerometer sending error\n");
        }
        break;
      }
      case GZ:
        if (instance == NULL)
          break;
        instance->gyroscope.x = sReg[GX];
        instance->gyroscope.y = sReg[GY];
        instance->gyroscope.z = sReg[GZ];
        if (instance->armPart->updateGyroscope(instance) != 0) {
          LogQueue::Log("Gyro sending error\n");
        }
        break;
      case Yaw:
        break;
      case HeightH:        
        instance->height = (int32_t)((sReg[HeightH] << 16) | sReg[HeightL]);
        instance->temperature = sReg[TEMP];

        if (instance->armPart->updateHeight(instance) != 0) {
          LogQueue::Log("Height sending error\n");
        }
        break;
      case q3: {
        if (instance == NULL)
          break;
        Quaternion quat;
        quat.real = sReg[q0] / 32768.0f;
        quat.i = sReg[q1] / 32768.0f;
        quat.j = sReg[q2] / 32768.0f;
        quat.k = sReg[q3] / 32768.0f;
        
        float norm = sqrtf(quat.real * quat.real + quat.i * quat.i + quat.j * quat.j +quat.k * quat.k);
        if (norm > 0.0001f) {
          quat.real = quat.real / norm;
          quat.i = quat.i / norm;
          quat.j = quat.j / norm;
          quat.k = quat.k / norm;
        }        
        instance->quaternion.store(quat);
        // Not calibrated yet (no base) is the normal state until calibration finishes, not a send
        // error - logging it put one line per IMU sample into the claw's log (40+MB in a session),
        // burying everything else.
        int quatResult = instance->armPart->updateQuaternion(instance);
        if (quatResult != 0 && quatResult != ERROR_PART_IS_NOT_CALIBRATED) {
          LogQueue::Log("quat sending error\n");
        }        
        break;
      }
      default:        
        break;
    }
    uiReg++;
  }
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
			LogQueue::Log("LocalWitmotion::AutoScanSensor: baud=%lu try=%d dataAvailable=%d\n",
			              c_uiBaud[i], 2 - iRetry, dataAvailable);
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
  WitInit(WIT_PROTOCOL_NORMAL, 0x50);  
	WitSerialWriteRegister(SensorUartSend);	
  WitRegisterCallBack(SensorDataUpdata);
	WitDelayMsRegister(Delayms);

  gpio_init(memsRstPin);
  gpio_set_dir(memsRstPin, GPIO_OUT);
  gpio_put(memsRstPin, 1);
  LogQueue::Log("LocalWitmotion: constructed, rxPin=%u txPin=%u rstPin=%u, reset pin driven high\n",
                memsRxPin, memsTxPin, memsRstPin);

  if (LocalWitmotion::readTaskHandle == NULL) {
    if (xTaskCreateAffinitySet(LocalWitmotion::readDetectorTask,
                               "LocalWitmotion::readDetectorTask", 4096, this,
                               5, IMU_CORE,
                               &LocalWitmotion::readTaskHandle) != pdPASS) {
      LogQueue::Log("LocalWitmotion::readTaskHandle creation failed\n");
    } else {
      LogQueue::Log("LocalWitmotion::readTaskHandle created\n");
    }
  }

  if (LocalWitmotion::initTaskHandle == NULL) {
    if (xTaskCreateAffinitySet(LocalWitmotion::init, "LocalWitmotion::initTask", 1024, this, 5,IMU_CORE,
                               &LocalWitmotion::initTaskHandle) != pdPASS) {
      LogQueue::Log("LocalWitmotion::initTaskHandle creation failed\n");
    } else {
      LogQueue::Log("LocalWitmotion::initTaskHandle created\n");
    }
  }
}

void LocalWitmotion::tare() {
  WitWriteReg(CALSW, 0x03);
  Delayms(200);
  WitWriteReg(CALSW, 0x00);
  /*WitWriteReg(SAVE, 0x00);
  Delayms(200);*/
}

bool LocalWitmotion::isPositionOK() { return quaternion.isFresh(); }

void LocalWitmotion::set6AxisMode() {
  WitWriteReg(AXIS6, ALGRITHM6);
  Delayms(50);
  WitWriteReg(SAVE, 0x00);
  Delayms(100);
}

void LocalWitmotion::set9AxisMode() {
  WitWriteReg(AXIS6, ALGRITHM9);
  Delayms(50);
  WitWriteReg(SAVE, 0x00);
  Delayms(100);
}

void LocalWitmotion::setRefreshRate() {
  if (WitSetOutputRate(RRATE_200HZ) != WIT_HAL_OK) {
    LogQueue::Log("LocalWitmotion::setRefreshRate: Failed to set rate\n");
  } else {
    LogQueue::Log("LocalWitmotion::setRefreshRate: Output rate set to 200Hz. Saving settings\n");
    if (WitWriteReg(SAVE, 0x00) != WIT_HAL_OK) {
      LogQueue::Log("LocalWitmotion::setRefreshRate: Failed to save settings\n");
    } else {
      LogQueue::Log("LocalWitmotion::setRefreshRate: Settings saved\n");
    }
    Delayms(100);
  }
}

void LocalWitmotion::setBandwidth() {
  if (WitWriteReg(BANDWIDTH, BANDWIDTH_94HZ) != WIT_HAL_OK) {
    LogQueue::Log("LocalWitmotion::setBandwidth: Failed to set bandwidth\n");
  } else {
    LogQueue::Log("LocalWitmotion::setBandwidth: Bandwidth set to 94Hz\n");
    WitWriteReg(SAVE, 0x00);
    Delayms(100);
  }
}