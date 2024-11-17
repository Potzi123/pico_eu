#ifndef GPS_H
#define GPS_H

#include "pico/stdlib.h"
#include "hardware/uart.h"

#define GPS_BUFFER_SIZE 256

void init_gps(uart_inst_t *uart, uint baud_rate, uint tx_pin, uint rx_pin);
bool read_gps_sentence(uart_inst_t *uart, char *buffer, size_t buffer_size);
void process_gps_data(const char *nmea_sentence, float &latitude, float &longitude, bool &valid);

#endif  // GPS_H
