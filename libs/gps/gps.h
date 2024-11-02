// gps.h

#ifndef MY_PROJECT_MYGPS_H
#define MY_PROJECT_MYGPS_H

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include <string>
#include "pico/time.h"

#ifdef ERROR_GPS_LOG
#define ERROR_GPS(fmt, ...) printf("ERROR-GPS: " fmt "\n", ##__VA_ARGS__)
#else
#define ERROR_GPS(fmt, ...)
#endif

#ifdef DEBUG_GPS_LOG
#define DEBUG_GPS(fmt, ...) printf("DEBUG-GPS: " fmt "\n", ##__VA_ARGS__)
#else
#define DEBUG_GPS(fmt, ...)
#endif

const std::string GNTXT = "$GNTXT";
const std::string GNGLL = "$GNGLL";

class myGPS {
private:
    uart_inst_t *uart_id;
    int baud_rate;
    int tx_pin;
    int rx_pin;
    double latitude = 0;
    char nsIndicator = ' ';
    double longitude = 0;
    char ewIndicator = ' ';
    std::string time = "00:00:00";
    std::string buffer;

public:
    myGPS(uart_inst_t *, int, int, int);
    void init();
    int readLine(std::string &);
    int readLine(std::string &, double &, char &, double &, char &, std::string &);
    std::string to_string(double, char, double, char, std::string &);

    // Getter methods for accessing private member variables
    double getLatitude() const { return latitude; }
    char getNsIndicator() const { return nsIndicator; }
    double getLongitude() const { return longitude; }
    char getEwIndicator() const { return ewIndicator; }
    std::string getTime() const { return time; }
};

#endif //MY_PROJECT_MYGPS_H
