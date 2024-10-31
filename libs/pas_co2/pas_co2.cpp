#include "pas_co2.h" 
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/i2c.h"

int Pas_co2::pas_co2_init()
{
 //Write values to Sensor registers
 uint8_t buffer[2];

 //IDLE MODE
 buffer[0] = MEAS_CFG;
 buffer[1] = 0x00;
 i2c_write_blocking(PAS_CO2_PORT, PAS_CO2_ADDR, buffer, 2, false);

 //Measurement rate to 10s
 buffer[0] = MEAS_RATE_H;
 buffer[1] = 0x00;
 i2c_write_blocking(i2c_default, PAS_CO2_ADDR, buffer, 2, false);

 buffer[0] = MEAS_RATE_L;
 buffer[1] = 0x01;
 i2c_write_blocking(i2c_default, PAS_CO2_ADDR, buffer, 2, false);

 //Configuring continous mode
 buffer[0] = MEAS_CFG;
 buffer[1] = 0x02;
 i2c_write_blocking(i2c_default, PAS_CO2_ADDR, buffer, 2, false);

 return 0;
}

void Pas_co2::pas_co2_read(){

    i2c_write_blocking(PAS_CO2_PORT, PAS_CO2_ADDR, &MEAS_STS, 2, true);
    i2c_read_blocking(PAS_CO2_PORT, PAS_CO2_ADDR, &data_rdy, 1, false); 
    if(data_rdy & COMP_BIT)  //check if unread data is available
     { 
      //reading CO2-ppm values
      i2c_write_blocking(PAS_CO2_PORT, PAS_CO2_ADDR, &CO2PPM_H, 2, true);
      i2c_read_blocking(PAS_CO2_PORT, PAS_CO2_ADDR, &msb, 1, false);    
      i2c_write_blocking(PAS_CO2_PORT, PAS_CO2_ADDR, &CO2PPM_L, 2, true);
      i2c_read_blocking(PAS_CO2_PORT, PAS_CO2_ADDR, &lsb, 1, false);    
      //calculating the value
     
      Pas_co2::result = (msb << 8) | lsb;
    } 
}