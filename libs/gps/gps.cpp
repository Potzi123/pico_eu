#include "gps.h"
#include <cstring>
#include <cstdlib>

#define GPS_TX_PIN 0
#define GPS_RX_PIN 1

void init_gps(uart_inst_t *uart, uint baud_rate, uint tx_pin, uint rx_pin) {
    uart_init(uart, baud_rate);
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);
    uart_set_format(uart, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart, true);
}

// Read a GPS NMEA sentence
bool read_gps_sentence(uart_inst_t *uart, char *buffer, size_t buffer_size) {
    size_t index = 0;

    while (true) {
        if (uart_is_readable(uart)) {
            char c = uart_getc(uart);
            if (c == '$') {
                buffer[index++] = c;
                break;
            }
        }
    }

    while (index < buffer_size - 1) {
        if (uart_is_readable(uart)) {
            char c = uart_getc(uart);
            buffer[index++] = c;

            if (c == '\n') {
                buffer[index] = '\0';
                return true;
            }
        }
    }


    buffer[buffer_size - 1] = '\0';
    return false;
}

void process_gps_data(const char *nmea_sentence, float &latitude, float &longitude, bool &valid) {

    if (strstr(nmea_sentence, "$GPGGA") || strstr(nmea_sentence, "$GPRMC")) {

        char *token = strtok((char *)nmea_sentence, ",");
        int field_count = 0;

        while (token) {
            field_count++;

            if (field_count == 3) {
                latitude = atof(token) / 100.0; // Zu Dezimalzahl konverten
            }

            if (field_count == 5) {
                longitude = atof(token) / 100.0;
            }

            if (field_count == 2 && token[0] == 'A') {
                valid = true;
            }

            token = strtok(NULL, ",");
        }
    } else {
        valid = false;
    }
}
