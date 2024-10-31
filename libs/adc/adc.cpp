#include "adc.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include <stdio.h>
#include <string.h>

myADC::myADC(int pin) : gpioPin(pin) {}

void myADC::init() {
    if (gpioPin < 26 || gpioPin > 28) {
        printf("Invalid GPIO pin: %d. ADC pins are 26, 27, or 28.", gpioPin);
        return;
    }

    adc_init();
    adc_gpio_init(gpioPin);
    adc_select_input(gpioPin - 26);
    printf("Initialized ADC on GPIO pin %d", gpioPin);
}

float myADC::readVoltage() {
    uint16_t result = adc_read();
    float voltage = result * conversionFactor;
    //printf("Read voltage: %.3fV", voltage);
    return voltage;
}

float myADC::calculateBatteryLevel() {
    float voltage = readVoltage();
    if (voltage < minVoltage) {
        printf("Voltage below min threshold: %.3fV", voltage);
        return 0.0f;  // Battery level is 0% if voltage is below minVoltage
    }
    if (voltage > maxVoltage) {
        printf("Voltage above max threshold: %.3fV", voltage);
        return 100.0f;  // Battery level is 100% if voltage is above maxVoltage
    }

    // Calculate the battery level as a percentage
    float scaledValue = ((voltage - minVoltage) / (maxVoltage - minVoltage)) * 100.0f;
    //printf("Voltage: %.3fV, Battery level: %.2f%%", voltage, scaledValue);
    sleep_ms(100);
    return scaledValue;
}
