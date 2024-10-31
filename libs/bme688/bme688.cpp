#include "bme688.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <cstdio>
#include <cstring> // For memcpy
#include <math.h> // For altitude factor calculation

// Constructor to initialize I2C for BME688
BME688::BME688(i2c_inst_t *i2c_port, uint8_t addr, uint sda_pin, uint scl_pin)
    : i2c_port(i2c_port), addr(addr), sda_pin(sda_pin), scl_pin(scl_pin) {
}

// Delays for sensor-specific timing in microseconds
void delay_us_wrapper(unsigned long us, void *intf_ptr) {
    sleep_us(us);
}

// Initializes the BME688 sensor
bool BME688::begin() {
    dev.intf_ptr = this;
    dev.intf = BME68X_I2C_INTF;
    dev.read = [](uint8_t reg, uint8_t *data, unsigned long len, void *intf_ptr) {
        return static_cast<BME688 *>(intf_ptr)->i2c_read(reg, data, len);
    };
    dev.write = [](uint8_t reg, const uint8_t *data, unsigned long len, void *intf_ptr) {
        return static_cast<BME688 *>(intf_ptr)->i2c_write(reg, data, len);
    };
    dev.delay_us = delay_us_wrapper;

    int8_t init_result = bme68x_init(&dev);
    if (init_result != BME68X_OK) {
        printf("BME688 initialization failed with error code: %d\n", init_result);
        return false;
    }

    printf("BME688 initialized successfully\n");
    return true;
}

// Reads data from the BME688 sensor
bool BME688::readData(float &temperature, float &humidity, float &pressure, float &gas_resistance) {
    struct bme68x_data data = {0}; 
    uint8_t n_fields = 0;

    // Set the sensor to FORCED_MODE to start a new measurement
    bme68x_set_op_mode(BME68X_FORCED_MODE, &dev);

    // Delay to allow data to be ready
    sleep_ms(100);

    // Retrieve data from the sensor
    int8_t result = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &dev);

    // Check for read errors and data availability
    if (result != BME68X_OK || n_fields == 0) {
        printf("Failed to read BME688 data, error code: %d, fields: %d\n", result, n_fields);
        return false;
    }

    // Validate raw data ranges to filter out invalid or outlier readings
    if (data.temperature < -4000 || data.temperature > 85000 ||
        data.humidity < 0 || data.humidity > 100000 ||
        data.pressure < 30000 || data.pressure > 110000 ||
        data.gas_resistance <= 0) {
        printf("Sensor data out of expected range. Skipping invalid read.\n");
        return false;
    }

    // Apply altitude compensation
    float alt_fac = pow(1.0 - 520 / 44330.0, 5.255);
    temperature = data.temperature / 100.0f;        // Convert temperature to °C
    humidity = data.humidity / 1000.0f;             // Convert humidity to %RH
    pressure = (data.pressure / 100.0f) / alt_fac;  // Convert pressure to hPa with altitude factor
    gas_resistance = static_cast<float>(data.gas_resistance);

    // Debugging: Print raw data values
    printf("Raw Temperature: %d, Raw Humidity: %d, Raw Pressure: %d, Raw Gas Resistance: %u\n",
           data.temperature, data.humidity, data.pressure, data.gas_resistance);

    // Print the formatted data
    #ifdef WITH_UNITS
    printf("Time(ms): %lu, Temp(°C): %.1f, Press(hPa): %.0f, Hum(%%): %.0f, Gas(ohm): %lu\n",
           platform_get_timestamp(), temperature, pressure, humidity, data.gas_resistance);
    #else
    printf("Temp(°C): %.1f\nPress(hPa): %.0f\nHum(%%): %.0f\nGas(ohm): %lu\n\n",
            temperature, pressure, humidity, data.gas_resistance);
    #endif

    return true;
}

// I2C write function for BME688
int8_t BME688::i2c_write(uint8_t reg, const uint8_t *data, unsigned long len) {
    uint8_t buf[len + 1];
    buf[0] = reg;
    memcpy(&buf[1], data, len);

    // Write data to the I2C bus
    int result = i2c_write_blocking(i2c_port, addr, buf, len + 1, false);
    if (result == PICO_ERROR_GENERIC) {
        printf("I2C write error at register 0x%X\n", reg);
        return -1;
    }
    return 0;
}

// I2C read function for BME688
int8_t BME688::i2c_read(uint8_t reg, uint8_t *data, unsigned long len) {
    // Send register address to read from
    int result = i2c_write_blocking(i2c_port, addr, &reg, 1, true);
    if (result == PICO_ERROR_GENERIC) {
        printf("I2C write error while setting register address 0x%X\n", reg);
        return -1;
    }

    // Read data from the I2C bus
    result = i2c_read_blocking(i2c_port, addr, data, len, false);
    if (result == PICO_ERROR_GENERIC) {
        printf("I2C read error at register 0x%X\n", reg);
        return -1;
    }
    return 0;
}
