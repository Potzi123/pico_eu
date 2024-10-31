// bme688.h

#ifndef BME688_H
#define BME688_H

#include "hardware/i2c.h"
#include "libs/bme688/api/BME68x_SensorAPI/bme68x.h"

class BME688 {
public:
    BME688(i2c_inst_t *i2c_port, uint8_t addr, uint sda_pin, uint scl_pin);
    bool begin();
    bool readData(float &temperature, float &humidity, float &pressure, float &gas_resistance);

private:
    i2c_inst_t *i2c_port;
    uint8_t addr;
    uint sda_pin;
    uint scl_pin;
    struct bme68x_dev dev;

    int8_t i2c_write(uint8_t reg, const uint8_t *data, unsigned long len);
    int8_t i2c_read(uint8_t reg, uint8_t *data, unsigned long len);
};

#endif
