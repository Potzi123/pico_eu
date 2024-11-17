#include "FlashStorage.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include <cstring>

FlashStorage::FlashStorage() {}

void FlashStorage::initialize() {
    // Optional: Any initialization logic
}

void FlashStorage::writeFlash(const void *data, size_t offset, size_t size) {
    uint8_t buffer[FLASH_STORAGE_PAGE_SIZE];
    size_t page_start = (BASE_ADDRESS + offset) & ~(FLASH_STORAGE_PAGE_SIZE - 1);
    size_t page_offset = (BASE_ADDRESS + offset) % FLASH_STORAGE_PAGE_SIZE;

    const uint8_t *flash_data = (const uint8_t *)(XIP_BASE + page_start);
    memcpy(buffer, flash_data, FLASH_STORAGE_PAGE_SIZE);

    memcpy(buffer + page_offset, data, size);

    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(page_start, FLASH_STORAGE_PAGE_SIZE);
    flash_range_program(page_start, buffer, FLASH_STORAGE_PAGE_SIZE);
    restore_interrupts(ints);
}


void FlashStorage::readFlash(void *buffer, size_t offset, size_t size) {
    const uint8_t *flashData = (const uint8_t *)(XIP_BASE + BASE_ADDRESS + offset);
    memcpy(buffer, flashData, size);
}

void FlashStorage::saveFloat(float value, size_t index) {
    writeFlash(&value, index * sizeof(float), sizeof(float));
}

void FlashStorage::saveInt(int value, size_t index) {
    writeFlash(&value, index * sizeof(int), sizeof(int));
}

void FlashStorage::saveUint32(uint32_t value, size_t index) {
    writeFlash(&value, index * sizeof(uint32_t), sizeof(uint32_t));
}

void FlashStorage::saveString(const std::string &value, size_t index) {
    size_t length = value.length();
    writeFlash(&length, index * FLASH_STORAGE_PAGE_SIZE, sizeof(size_t));
    writeFlash(value.c_str(), index * FLASH_STORAGE_PAGE_SIZE + sizeof(size_t), length);
}

float FlashStorage::retrieveFloat(size_t index) {
    float value = 0;
    readFlash(&value, index * sizeof(float), sizeof(float));
    return value;
}

int FlashStorage::retrieveInt(size_t index) {
    int value = 0;
    readFlash(&value, index * sizeof(int), sizeof(int));
    return value;
}

uint32_t FlashStorage::retrieveUint32(size_t index) {
    uint32_t value = 0;
    readFlash(&value, index * sizeof(uint32_t), sizeof(uint32_t));
    return value;
}

std::string FlashStorage::retrieveString(size_t index) {
    size_t length = 0;
    readFlash(&length, index * FLASH_STORAGE_PAGE_SIZE, sizeof(size_t));
    char buffer[length + 1];
    readFlash(buffer, index * FLASH_STORAGE_PAGE_SIZE + sizeof(size_t), length);
    buffer[length] = '\0';
    return std::string(buffer);
}

void FlashStorage::eraseFlash() {
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(BASE_ADDRESS, STORAGE_SIZE);
    restore_interrupts(ints);
}
