#pragma once

#include <RP2040.h>
#include <FreeRTOS.h>
#include <task.h>
#include <cstring>
#include <cstdio>
#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/sync.h"


#include "../config/config.h"

constexpr uint32_t EFFECTIVE_FLASH_SIZE = FLASH_BUFFER_SIZE + sizeof(uint64_t) + sizeof(uint32_t); //buffser + checksum + timestamp

class FlashSettingsData {
  protected:
    uint32_t size = 0;
    uint32_t checksum = 0;
    uint32_t offset = 0;
    uint64_t timestamp = 0;    
  public:
    FlashSettingsData(uint32_t sizeBytes, uint32_t flashOffset)
        : size(sizeBytes), offset(flashOffset) {}
    virtual ~FlashSettingsData() = default;
  
    virtual uint8_t *getBuffer() = 0;
    virtual const uint8_t *getBuffer() const = 0;

    uint32_t getSize() const { return size; }
    uint32_t getChecksum() const { return checksum; }
    uint32_t getOffset() const { return offset; }
    void setTimestamp(uint64_t ts) { timestamp = ts; }
    void setChecksum(uint32_t cs) { checksum = cs; }
    void updateChecksum();  
    bool validateChecksum()const;
};


class FlashSettings {
  private:    
    static void flashSaveTask(void *param);
    static TaskHandle_t flashSaveTaskHandle;    
  public:
    static void init();
    static uint8_t flashData[EFFECTIVE_FLASH_SIZE];
    static uint32_t flashDataOffset;
    static uint32_t flashDataSize;
    static void save(FlashSettingsData& data);
    static bool load(FlashSettingsData& data);
};