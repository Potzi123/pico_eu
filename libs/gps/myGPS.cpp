//
// Created by Benedikt Walter on 18.01.24.
//

#include "stdlib.h"
#include "libs/gps/myGPS.h"
#include <hardware/gpio.h>


myGPS::myGPS(uart_inst_t *uart_id, int baud_rate, int tx_pin, int rx_pin) {
    this->uart_id = uart_id;
    this->baud_rate = baud_rate;
    this->tx_pin = tx_pin;
    this->rx_pin = rx_pin;
    this->init();
}

void myGPS::init() {
    uart_init(this->uart_id, this->baud_rate);
    gpio_set_function(this->tx_pin, GPIO_FUNC_UART);
    gpio_set_function(this->rx_pin, GPIO_FUNC_UART);
}

/** /@return 0 on sucess \n 1 on not sucess
 */
int myGPS::readLine(std::string &line) {
    if(!uart_is_readable(this->uart_id)) {
        return 1;
    }

    do {
        this->buffer = "";
        while(this->buffer.back() != '\n') {
            this->buffer += uart_getc(this->uart_id);
            //printf("%s", this->buffer.c_str());
        }
        if(buffer.empty()) {
            //printf("Buffer empty");
            return 1;
        }
        printf("%s", this->buffer.c_str());
    } while(this->buffer.substr(0, GNGLL.length()) != GNGLL);

    line = this->buffer;

    std::istringstream iss(this->buffer);
    std::string token;

    // Skip the first token "$GNGLL,"
    std::getline(iss, token, ',');

    // Latitude
    std::getline(iss, token, ',');
    if (!token.empty())
        this->latitude = std::stod(token.substr(0, 2)) + std::stod(token.substr(2)) / 60.0;
    else
        this->latitude = 0;

    // N/S indicator
    std::getline(iss, token, ',');
    if (!token.empty())
        this->nsIndicator = token[0];
    else
        this->nsIndicator = 'C';

    // Longitude
    std::getline(iss, token, ',');
    if (!token.empty())
        this->longitude = std::stod(token.substr(0, 3)) + std::stod(token.substr(3)) / 60.0;
    else
        this->longitude = 0;

    // E/W indicator
    std::getline(iss, token, ',');
    if (!token.empty())
        this->ewIndicator = token[0];
    else
        this->ewIndicator = 'C';

    // Time
    std::getline(iss, token, ',');
    if (!token.empty() && token.size() >= 6)
        this->time = token.substr(0, 2) + ":" + token.substr(2, 2) + ":" + token.substr(4, 2);
    else
        this->time = "00:00:00";

    printf("Long: %f%c\n", this->longitude, this->nsIndicator);
    printf("Lat: %f%c\n", this->latitude, this->ewIndicator);
    printf("Time: %s\n\n", this->time.c_str());

    return 0;
}

int myGPS::readLine(std::string &buffer, double &latitude, char &nsIndicator, double &longitude, char &ewIndicator, std::string &time) {
    if(this->readLine(buffer)) {
        return 1;
    }
    latitude = this->latitude;
    nsIndicator = this->nsIndicator;
    longitude = this->longitude;
    ewIndicator = this->ewIndicator;
    time = this->time;
    return 0;
}

std::string myGPS::to_string(double latitude, char nsIndicator, double longitude, char ewIndicator, std::string &time) {
    std::string flash_data;
    for(auto i : std::to_string(latitude)) {
        flash_data.push_back(i);
    }
    flash_data.push_back('|');
    flash_data.push_back(nsIndicator);
    flash_data.push_back('|');
    for(auto i : std::to_string(longitude)) {
        flash_data.push_back(i);
    }
    flash_data.push_back('|');
    flash_data.push_back(ewIndicator);
    flash_data.push_back('|');
    for(auto i : time) {
        flash_data.push_back(i);
    }
    flash_data.push_back('^');
    return flash_data;
}
