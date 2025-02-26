//
// Created by Benedikt Walter on 18.01.24.
//

#ifndef MY_PROJECT_MYGPS_H
#define MY_PROJECT_MYGPS_H


#include "hardware/uart.h"
#include <string>
#include <sstream>
#include <vector>
#include <algorithm>

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



#define UART0_UART_ID uart0
#define UART0_BAUD_RATE 9600
#define UART0_TX_PIN 0
#define UART0_RX_PIN 1

const std::string GNTXT = "$GNTXT";
const std::string GNGLL = "$GNGLL";

const std::string AUTHREQ = "$AUTHREQ";
const std::string AUTHRES = "$AUTHRES";
const std::string DATASEND = "$DATASEND";
const std::string DATAACKN = "$DATAACKN";

class myGPS {
private:
    uart_inst_t *uart_id;
    int baud_rate;
    int tx_pin;
    int rx_pin;
    double latitude = 0;
    char nsIndicator = 'C';
    double longitude = 0;
    char ewIndicator = 'C';
    std::string time = "00:00:00";
    std::string buffer;
public:
    myGPS(uart_inst_t *, int, int, int);
    void init();
    int readLine(std::string &);
    int readLine(std::string &, double &, char &, double &, char &, std::string &);
    std::string to_string(double, char, double, char, std::string &);

};



#endif //MY_PROJECT_MYGPS_H
