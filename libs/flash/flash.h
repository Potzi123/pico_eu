#ifndef FLASH_H
#define FLASH_H

#include <vector>
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

// Add this definition at the top of the file, after the struct definitions
#define SENSOR_DATA_SIZE  sizeof(SensorData)

// SensorData struct matches the one in pico_eu.cpp
struct SensorData {
    float temp = 0.0;
    float hum = 0.0;
    float pres = 0.0;
    float gasRes = 0.0;
    uint16_t pm2_5 = 0;
    uint16_t pm5 = 0;
    uint16_t pm10 = 0;
    uint32_t co2 = 0;
    uint32_t latitude = 0;
    uint32_t longitude = 0;
    uint32_t timestamp = 0;
};

// Add this struct to ensure aligned, packed serialization
#pragma pack(push, 1)
struct SerializedSensorData {
    float temp;
    float hum;
    float pres;
    float gasRes;
    uint16_t pm2_5;
    uint16_t pm5;
    uint16_t pm10;
    uint16_t padding;  // Ensure proper alignment
    uint32_t co2;
    uint32_t latitude;
    uint32_t longitude;
    uint32_t timestamp;
};
#pragma pack(pop)

class Flash {
public:
    Flash(uint32_t flash_offset = 0);
    ~Flash();

    // Initialize flash storage
    bool init();
    
    // Save a sensor data point to flash
    bool saveSensorData(const SensorData& data);
    
    // Save a vector of sensor data points to flash
    bool saveSensorDataBatch(const std::vector<SensorData>& data);
    
    // Load all sensor data from flash
    std::vector<SensorData> loadSensorData();
    
    // Erase flash sector (to clear stored data)
    bool eraseStorage();
    
    // Get count of stored records
    uint32_t getStoredCount();
    
    // Get maximum data count
    uint32_t getMaxDataCount() const { return _max_data_count; }
    
    // Check if storage is full
    bool isStorageFull();

    // Debug function to dump raw flash contents
    void dumpRawFlashContents(size_t max_records = 5);

    // Function to erase all data and reset storage
    bool resetStorage();

    bool readSensorDataRecord(uint32_t addr, SensorData &data);

private:
    uint32_t _flash_offset;                // Where to start storing data in flash
    uint32_t _data_count_address;          // Where to store the count of records
    uint32_t _data_start_address;          // Where the actual data starts
    uint32_t _max_data_count;              // Maximum number of records that can be stored
    uint32_t _stored_data_count;           // Current count of stored records
    
    // Converts between flash address and XIP mapped address
    inline uint32_t flashAddressToXIP(uint32_t flash_addr) {
        return XIP_BASE + flash_addr;
    }
    
    // Serialize sensor data to a byte array
    void serializeSensorData(const SensorData& data, uint8_t* buffer);
    
    // Deserialize sensor data from a byte array
    SensorData deserializeSensorData(const uint8_t* buffer);

    uint32_t calculateNextWriteAddress() {
        // Calculate storage address for the next data point
        return _data_start_address + (_stored_data_count * SENSOR_DATA_SIZE);
    }
};

#endif // FLASH_H
