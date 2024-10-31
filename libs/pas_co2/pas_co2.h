#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/i2c.h"
#include "tusb.h"

class Pas_co2
{
public:
    // I2C device-address PAS-CO2
    uint8_t PAS_CO2_ADDR = 0x28;

    // PAS-CO2 registers
    uint8_t MEAS_RATE_H = 0x02;
    uint8_t MEAS_RATE_L = 0x03;
    uint8_t MEAS_CFG = 0x04;
    uint8_t CO2PPM_H = 0x05;
    uint8_t CO2PPM_L = 0x06;
    uint8_t MEAS_STS = 0x07;  //Bit 4 in this register indicates if unread data in CO2PPM_H & CO2PPM_L registers is available

    uint8_t COMP_BIT = 0b00010000;
    uint8_t lsb;
    uint8_t msb;
    uint8_t data_rdy;
    uint16_t result;

public:
    uint16_t getResult()
    {
        return this->result;
    }

public:
    void setResult(uint16_t result)
    {
        this->result = result;
    }

public:
    int pas_co2_init();
    void pas_co2_read();
    
};


