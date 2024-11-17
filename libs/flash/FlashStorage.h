#ifndef FLASH_STORAGE_H
#define FLASH_STORAGE_H

#include <string>
#include "pico/stdlib.h"

class FlashStorage {
public:
    FlashStorage();
    void initialize();
    void saveFloat(float value, size_t index);
    void saveUint32(uint32_t value, size_t index);
    float retrieveFloat(size_t index);
    uint32_t retrieveUint32(size_t index);
    void eraseFlash();

private:
    static const size_t FLASH_STORAGE_PAGE_SIZE = 256; // Bytes per page
    static const size_t BASE_ADDRESS = 0x100000;       // Start of flash storage

    void writeFlash(const void *data, size_t offset, size_t size);
    void readFlash(void *buffer, size_t offset, size_t size);
};

#endif // FLASH_STORAGE_H
