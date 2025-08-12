#include "flashSettings.h"

TaskHandle_t FlashSettings::flashSaveTaskHandle = NULL;
uint32_t FlashSettings::flashDataOffset = 0;
uint32_t FlashSettings::flashDataSize = 0;
uint8_t FlashSettings::flashData[EFFECTIVE_FLASH_SIZE] = {0};

void __not_in_flash_func(flashSaveTaskFunc)(const uint8_t *data,uint32_t offset, uint32_t dataSize) {
  const uint8_t *flash_contents = (const uint8_t *)(XIP_BASE + FLASH_EEPROM_OFFSET);
  uint8_t sectorBuffer[FLASH_SECTOR_SIZE];

  memcpy(sectorBuffer, flash_contents, FLASH_SECTOR_SIZE);
  memcpy(sectorBuffer + offset, data, dataSize);

  uint32_t ints = save_and_disable_interrupts();
  flash_range_erase(FLASH_EEPROM_OFFSET, FLASH_SECTOR_SIZE);
  flash_range_program(FLASH_EEPROM_OFFSET, sectorBuffer, FLASH_SECTOR_SIZE);
  restore_interrupts(ints);
}

void FlashSettings::flashSaveTask(void *param) {
  (void)param;
  while (true) {
    uint32_t notificationValue = ulTaskNotifyTakeIndexed(FLASH_SETTINGS_NOTIFICATION_INDEX, pdTRUE, pdMS_TO_TICKS(FLASH_WAIT_TIMEOUT));
    if (notificationValue == 0) {
      continue;
    }

    flashSaveTaskFunc(FlashSettings::flashData, FlashSettings::flashDataOffset, FlashSettings::flashDataSize);
    vTaskDelay(pdMS_TO_TICKS(FLASH_WAIT_TIMEOUT));
  }
}

void FlashSettings::init() {
  if (FlashSettings::flashSaveTaskHandle != NULL) {  
    return;
  }

  BaseType_t res = xTaskCreate(FlashSettings::flashSaveTask, "FlashSave", 4096, NULL, tskIDLE_PRIORITY + 2, &FlashSettings::flashSaveTaskHandle);
  if (res != pdPASS) {
    printf("Failed to create flashSaveTask\n");
  }
}

void FlashSettings::save(FlashSettingsData &data) {
  if (data.getSize() > FLASH_BUFFER_SIZE) {
    printf("Data size exceeds FLASH_BUFFER_SIZE size\n");
    return;
  }

  data.updateChecksum();
  

  flashDataSize = data.getSize() + sizeof(uint32_t) + sizeof(uint64_t);
  flashDataOffset = data.getOffset();

  memset(flashData, 0xFF, sizeof(flashData));

  uint32_t tchecksum = data.getChecksum();
  memcpy(flashData, &tchecksum, sizeof(tchecksum));

  uint64_t now_us = time_us_64();
  memcpy(flashData + sizeof(uint32_t), &now_us, sizeof(now_us));
  
  memcpy(flashData + sizeof(uint32_t) + sizeof(uint64_t),
         data.getBuffer(),
         data.getSize());

  if (flashSaveTaskHandle != NULL) {
    xTaskNotifyGiveIndexed(flashSaveTaskHandle,
                           FLASH_SETTINGS_NOTIFICATION_INDEX);
  } else {
    printf("Flash save task handle is NULL, cannot save to EEPROM\n");
  }
}

bool FlashSettings::load(FlashSettingsData &data){
  if (data.getSize() > FLASH_BUFFER_SIZE)
  {
    printf("Data size exceeds FLASH_BUFFER_SIZE size\n");
    return false;
  }

  const uint8_t *flash_contents = (const uint8_t *)(XIP_BASE + FLASH_EEPROM_OFFSET);
  size_t baseOffset = data.getOffset();

  uint32_t storedChecksum;
  uint64_t storedTimestamp;
  
  memcpy(&storedChecksum, flash_contents + baseOffset, sizeof(storedChecksum));
  memcpy(&storedTimestamp, flash_contents + baseOffset + sizeof(uint32_t), sizeof(storedTimestamp));

  if (storedTimestamp == 0 || storedTimestamp == 0xFFFFFFFFFFFFFFFFULL) {
    printf("No valid data in EEPROM\n");
    return false;
  }
  
  memcpy(data.getBuffer(),
         flash_contents + baseOffset + sizeof(uint32_t) + sizeof(uint64_t),
         data.getSize());

  data.setChecksum(storedChecksum);
  data.setTimestamp(storedTimestamp);


  if (!data.validateChecksum()) {
    printf("Checksum mismatch for FlashSettingsData\n");
    return false;
  }
  
  return true;
}

void FlashSettingsData::updateChecksum() {
  const uint32_t *data32 = reinterpret_cast<const uint32_t *>(getBuffer());
  int words = size / sizeof(uint32_t);
  uint32_t sum = 0;
  for (int i = 0; i < words; i++)
  {
    sum ^= data32[i];
  }
  checksum = sum;
}

bool FlashSettingsData::validateChecksum() const {
  const uint32_t *data32 = reinterpret_cast<const uint32_t *>(getBuffer());
  int words = size / sizeof(uint32_t);
  uint32_t sum = 0;
  for (int i = 0; i < words; i++)
  {
    sum ^= data32[i];
  }  
  return (sum == checksum);
}