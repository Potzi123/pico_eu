#include "pas_co2.h" 
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// Constructor to initialize address
Pas_co2::Pas_co2(uint8_t address, i2c_inst_t* i2c_instance) 
    : i2c_address(address), i2c(i2c_instance), result(0) {}

int Pas_co2::init() {
    uint8_t buffer[2];

    // Set sensor to idle mode
    buffer[0] = MEAS_CFG;
    buffer[1] = 0x00;
    i2c_write_blocking(i2c, i2c_address, buffer, 2, false);

    // Set measurement rate high and low bytes (10s interval)
    buffer[0] = MEAS_RATE_H;
    buffer[1] = 0x00;
    i2c_write_blocking(i2c, i2c_address, buffer, 2, false);
    
    buffer[0] = MEAS_RATE_L;
    buffer[1] = 0x01;
    i2c_write_blocking(i2c, i2c_address, buffer, 2, false);

    // Set continuous measurement mode
    buffer[0] = MEAS_CFG;
    buffer[1] = 0x02;
    i2c_write_blocking(i2c, i2c_address, buffer, 2, false);

    return 0;
}

void Pas_co2::read() {
    // Check if new data is available
    i2c_write_blocking(i2c, i2c_address, &MEAS_STS, 1, true);
    i2c_read_blocking(i2c, i2c_address, &data_rdy, 1, false);

    if (data_rdy & COMP_BIT) {
        // Read CO2 PPM high and low bytes
        i2c_write_blocking(i2c, i2c_address, &CO2PPM_H, 1, true);
        i2c_read_blocking(i2c, i2c_address, &msb, 1, false);

        i2c_write_blocking(i2c, i2c_address, &CO2PPM_L, 1, true);
        i2c_read_blocking(i2c, i2c_address, &lsb, 1, false);

        // Combine high and low bytes to calculate the result
        result = (msb << 8) | lsb;
    }
}
