//
// Created by Benedikt Walter on 20.02.24.
//

#ifndef MY_PROJECT_MYADC_H
#define MY_PROJECT_MYADC_H

#include <stdio.h>

#ifdef ERROR_ADC_LOG
#define ERROR_ADC(fmt, ...) printf("ERROR-ADC: " fmt "\n", ##__VA_ARGS__)
#else
#define ERROR_ADC(fmt, ...)
#endif

#ifdef DEBUG_ADC_LOG
#define DEBUG_ADC(fmt, ...) printf("DEBUG-ADC: " fmt "\n", ##__VA_ARGS__)
#else
#define DEBUG_ADC(fmt, ...)
#endif

class myADC {
private:
    const float minVoltage = 3.0f;   // Battery level 0%
    const float maxVoltage = 4.2f;   // Battery level 100%
    const float conversionFactor = (3.3f / 4096) * 2;  // Adjusted for a 2:1 voltage divider
    int gpioPin;

public:
    explicit myADC(int pin);
    void init();
    float readVoltage();
    float calculateBatteryLevel();
};


#endif //MY_PROJECT_MYADC_H
