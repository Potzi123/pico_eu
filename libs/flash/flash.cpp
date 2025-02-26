#include "flash.h"
#include <cstring>
#include <algorithm>
#include <cstdio>  // Add this include for printf

// Constants - Using the SDK's macros directly instead of redefining them
// (removes the redefinition warnings)
#define SENSOR_DATA_SIZE  sizeof(SensorData)
#define DATA_COUNT_SIZE   sizeof(uint32_t)

// Default flash target offset (1.8MB from beginning of flash)
#define DEFAULT_FLASH_TARGET_OFFSET (1792 * 1024)

Flash::Flash(uint32_t flash_offset) {
    if (flash_offset == 0) {
        _flash_offset = DEFAULT_FLASH_TARGET_OFFSET;
    } else {
        _flash_offset = flash_offset;
    }
    
    // Setup addresses
    _data_count_address = _flash_offset;
    _data_start_address = _flash_offset + FLASH_PAGE_SIZE; // Store count in its own page
    
    // Calculate max data count
    // Use 64KB (16 sectors) for storage by default
    uint32_t storage_size = 16 * FLASH_SECTOR_SIZE;
    _max_data_count = (storage_size - FLASH_PAGE_SIZE) / SENSOR_DATA_SIZE;
    
    _stored_data_count = 0;
}

Flash::~Flash() {
    // Nothing to clean up
}

bool Flash::init() {
    // Read stored count from flash
    const uint32_t* stored_count_ptr = (const uint32_t*)flashAddressToXIP(_data_count_address);
    uint32_t stored_count = *stored_count_ptr;
    
    // Validate count (simple check to see if flash is initialized)
    if (stored_count == 0xFFFFFFFF) { // Erased flash is all 1's
        _stored_data_count = 0;
        
        // Initialize by writing 0 as the count
        uint32_t count = 0;
        uint32_t interrupt_state = save_and_disable_interrupts();
        flash_range_program(_data_count_address, (const uint8_t*)&count, DATA_COUNT_SIZE);
        restore_interrupts(interrupt_state);
    } else if (stored_count > _max_data_count) {
        _stored_data_count = 0;
        return false; // Invalid count
    } else {
        _stored_data_count = stored_count;
    }
    
    return true;
}

bool Flash::saveSensorData(const SensorData& data) {
    // Check if there's space
    if (_stored_data_count >= _max_data_count) {
        return false;
    }
    
    // Calculate storage address for this data point
    uint32_t data_address = _data_start_address + (_stored_data_count * SENSOR_DATA_SIZE);
    
    // Important: align the address to FLASH_PAGE_SIZE
    data_address = (data_address / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE;
    
    // Debug the alignment
    printf("FLASH: Original address: 0x%08x, Aligned address: 0x%08x\n", 
           (unsigned int)(_data_start_address + (_stored_data_count * SENSOR_DATA_SIZE)),
           (unsigned int)data_address);
    
    // Allocate a full page buffer
    uint8_t* page_buffer = new uint8_t[FLASH_PAGE_SIZE];
    
    // Initialize buffer to 0xFF (erased flash value)
    memset(page_buffer, 0xFF, FLASH_PAGE_SIZE);
    
    // Calculate offset within the page
    uint32_t page_offset = (_stored_data_count * SENSOR_DATA_SIZE) % FLASH_PAGE_SIZE;
    
    // Serialize data into the page buffer at the correct offset
    serializeSensorData(data, page_buffer + page_offset);
    
    // Debug what we're writing
    printf("FLASH: Writing to page at 0x%08x, offset %u bytes\n", 
           (unsigned int)data_address, (unsigned int)page_offset);
    
    // Write the entire page to flash
    uint32_t interrupt_state = save_and_disable_interrupts();
    flash_range_program(data_address, page_buffer, FLASH_PAGE_SIZE);
    restore_interrupts(interrupt_state);
    
    delete[] page_buffer;
    
    // Update count in flash
    _stored_data_count++;
    uint32_t interrupt_state2 = save_and_disable_interrupts();
    flash_range_program(_data_count_address, (const uint8_t*)&_stored_data_count, DATA_COUNT_SIZE);
    restore_interrupts(interrupt_state2);
    
    // Debug: Print what we just wrote
    printf("FLASH DEBUG: Just wrote record %lu\n", _stored_data_count - 1);
    
    const uint8_t* debug_ptr = (const uint8_t*)flashAddressToXIP(data_address + page_offset);
    printf("FLASH DEBUG: Raw bytes at 0x%08x: ", (unsigned int)(data_address + page_offset));
    for (size_t i = 0; i < 16 && i < SENSOR_DATA_SIZE; i++) {
        printf("%02x ", debug_ptr[i]);
    }
    printf("\n");

    // Read back immediately for verification
    SensorData verify = deserializeSensorData(debug_ptr);
    printf("FLASH DEBUG: Verification read: Time=%u, Temp=%.2f, Hum=%.2f, CO2=%u, PM2.5=%u\n",
           verify.timestamp, verify.temp, verify.hum, verify.co2, verify.pm2_5);

    return true;
}

bool Flash::saveSensorDataBatch(const std::vector<SensorData>& data) {
    // Check if there's enough space
    if (_stored_data_count + data.size() > _max_data_count) {
        return false;
    }
    
    // Calculate how many pages we need
    size_t totalSize = data.size() * SENSOR_DATA_SIZE;
    size_t pages = (totalSize + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE;
    
    // Serialize all data into a buffer
    uint8_t* buffer = new uint8_t[pages * FLASH_PAGE_SIZE];
    memset(buffer, 0xFF, pages * FLASH_PAGE_SIZE); // Initialize with 0xFF (erased flash value)
    
    for (size_t i = 0; i < data.size(); i++) {
        serializeSensorData(data[i], buffer + (i * SENSOR_DATA_SIZE));
    }
    
    // Calculate storage address for this batch
    uint32_t data_address = _data_start_address + (_stored_data_count * SENSOR_DATA_SIZE);
    
    // Write data to flash
    uint32_t interrupt_state = save_and_disable_interrupts();
    flash_range_program(data_address, buffer, pages * FLASH_PAGE_SIZE);
    restore_interrupts(interrupt_state);
    
    delete[] buffer;
    
    // Update count in flash
    _stored_data_count += data.size();
    uint32_t interrupt_state2 = save_and_disable_interrupts();
    flash_range_program(_data_count_address, (const uint8_t*)&_stored_data_count, DATA_COUNT_SIZE);
    restore_interrupts(interrupt_state2);
    
    return true;
}

std::vector<SensorData> Flash::loadSensorData() {
    std::vector<SensorData> result;
    
    // Return empty vector if no data
    if (_stored_data_count == 0) {
        return result;
    }
    
    // Reserve space for all records
    result.reserve(_stored_data_count);
    
    // Read each record - with proper page alignment handling
    for (uint32_t i = 0; i < _stored_data_count; i++) {
        // Calculate the page that contains this record
        uint32_t page_address = _data_start_address + ((i * SENSOR_DATA_SIZE) / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE;
        
        // Calculate offset within the page
        uint32_t page_offset = (i * SENSOR_DATA_SIZE) % FLASH_PAGE_SIZE;
        
        // Get pointer to the data in flash memory
        const uint8_t* data_ptr = (const uint8_t*)flashAddressToXIP(page_address + page_offset);
        
        // Debug the address calculation
        printf("FLASH: Reading record %lu from address 0x%08x (page 0x%08x + offset %u)\n",
               i, (unsigned int)(page_address + page_offset), (unsigned int)page_address, 
               (unsigned int)page_offset);
        
        // Dump raw bytes for debug
        printf("FLASH DEBUG: Raw bytes for record %lu: ", i);
        for (size_t j = 0; j < 16 && j < SENSOR_DATA_SIZE; j++) {
            printf("%02x ", data_ptr[j]);
        }
        printf("\n");
        
        // Deserialize the data
        SensorData data = deserializeSensorData(data_ptr);
        result.push_back(data);
    }
    
    return result;
}

bool Flash::eraseStorage() {
    // Calculate how many sectors we need to erase
    uint32_t sectors_to_erase = 1; // At least one for the count
    
    if (_stored_data_count > 0) {
        uint32_t data_size = _stored_data_count * SENSOR_DATA_SIZE;
        uint32_t data_sectors = (data_size + FLASH_SECTOR_SIZE - 1) / FLASH_SECTOR_SIZE;
        sectors_to_erase += data_sectors;
    }
    
    // Erase sectors
    uint32_t interrupt_state = save_and_disable_interrupts();
    flash_range_erase(_flash_offset, sectors_to_erase * FLASH_SECTOR_SIZE);
    restore_interrupts(interrupt_state);
    
    // Reset count
    _stored_data_count = 0;
    uint32_t count = 0;
    uint32_t interrupt_state2 = save_and_disable_interrupts();
    flash_range_program(_data_count_address, (const uint8_t*)&count, DATA_COUNT_SIZE);
    restore_interrupts(interrupt_state2);
    
    return true;
}

uint32_t Flash::getStoredCount() {
    return _stored_data_count;
}

bool Flash::isStorageFull() {
    return _stored_data_count >= _max_data_count;
}

void Flash::serializeSensorData(const SensorData& data, uint8_t* buffer) {
    // Create a packed struct for serialization
    SerializedSensorData serialized;
    
    // Copy data fields
    serialized.temp = data.temp;
    serialized.hum = data.hum;
    serialized.pres = data.pres;
    serialized.gasRes = data.gasRes;
    serialized.pm2_5 = data.pm2_5;
    serialized.pm5 = data.pm5;
    serialized.pm10 = data.pm10;
    serialized.padding = 0; // Zero out padding
    serialized.co2 = data.co2;
    serialized.latitude = data.latitude;
    serialized.longitude = data.longitude;
    serialized.timestamp = data.timestamp;
    
    // Copy the entire struct to the buffer
    memcpy(buffer, &serialized, sizeof(SerializedSensorData));
}

SensorData Flash::deserializeSensorData(const uint8_t* buffer) {
    SensorData data;
    SerializedSensorData serialized;
    
    // Copy from buffer to packed struct
    memcpy(&serialized, buffer, sizeof(SerializedSensorData));
    
    // Extract fields
    data.temp = serialized.temp;
    data.hum = serialized.hum;
    data.pres = serialized.pres;
    data.gasRes = serialized.gasRes;
    data.pm2_5 = serialized.pm2_5;
    data.pm5 = serialized.pm5;
    data.pm10 = serialized.pm10;
    data.co2 = serialized.co2;
    data.latitude = serialized.latitude;
    data.longitude = serialized.longitude;
    data.timestamp = serialized.timestamp;
    
    return data;
}

void Flash::dumpRawFlashContents(size_t max_records) {
    printf("Raw flash contents (first %zu records):\n", max_records);
    
    // Dump count
    const uint32_t* count_ptr = (const uint32_t*)flashAddressToXIP(_data_count_address);
    printf("Count value at 0x%08x: %lu\n", (unsigned int)_data_count_address, *count_ptr);
    
    // Limit to maximum records or stored count, whichever is smaller
    size_t records_to_dump = (_stored_data_count < max_records) ? _stored_data_count : max_records;
    
    for (size_t i = 0; i < records_to_dump; i++) {
        uint32_t data_address = _data_start_address + (i * SENSOR_DATA_SIZE);
        const uint8_t* data_ptr = (const uint8_t*)flashAddressToXIP(data_address);
        
        printf("Record %zu at 0x%08x: ", i, (unsigned int)data_address);
        // Dump first 16 bytes in hex
        for (size_t j = 0; j < 16 && j < SENSOR_DATA_SIZE; j++) {
            printf("%02x ", data_ptr[j]);
        }
        printf("\n");
    }
}

bool Flash::resetStorage() {
    printf("FLASH: Completely erasing flash storage area...\n");
    
    // Calculate total sectors to erase (16 sectors = 64KB)
    uint32_t sectors_to_erase = 16;
    
    // Erase all sectors
    uint32_t interrupt_state = save_and_disable_interrupts();
    flash_range_erase(_flash_offset, sectors_to_erase * FLASH_SECTOR_SIZE);
    restore_interrupts(interrupt_state);
    
    // Reset count
    _stored_data_count = 0;
    uint32_t count = 0;
    uint32_t interrupt_state2 = save_and_disable_interrupts();
    flash_range_program(_data_count_address, (const uint8_t*)&count, DATA_COUNT_SIZE);
    restore_interrupts(interrupt_state2);
    
    printf("FLASH: Storage reset complete\n");
    return true;
}
