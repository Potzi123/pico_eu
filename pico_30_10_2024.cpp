#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/interp.h"
#include "hardware/timer.h"
#include "hardware/watchdog.h"
#include "hardware/clocks.h"
#include "pico/cyw43_arch.h"
#include "hardware/uart.h"

#include "password.h"
#include "libs/hm3301/hm3301.h"
#include "libs/bme688/bme688.h"
#include "libs/pas_co2/pas_co2.h"


// I2C configuration and sensor addresses
#define I2C_PORT i2c0
#define I2C_SDA 4
#define I2C_SCL 5
#define HM3301_ADDRESS 0x40
#define BME688_ADDRESS 0x76
#define PAS_CO2_ADDRESS 0x28

void i2c_init() {
    i2c_init(I2C_PORT, 400000);  // Initialize I2C at 400kHz
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
}


int main() {
    stdio_init_all();

    while (!stdio_usb_connected()) {
        sleep_ms(100);  // Poll every 100ms
    }

    i2c_init();
    printf("I2C initialized\n");

    // Initialize the HM3301 sensor
    HM3301 hm3301_sensor(I2C_PORT, HM3301_ADDRESS, I2C_SDA, I2C_SCL);
    if (!hm3301_sensor.begin()) {
        printf("Failed to initialize HM3301 sensor\n");
        return 1;
    }
    printf("HM3301 sensor initialized\n");


    BME688 bme688_sensor(I2C_PORT, BME688_ADDRESS, I2C_SDA, I2C_SCL);
    if (!bme688_sensor.begin()) {
        printf("Failed to initialize BME688 sensor\n");
        return 1;
    }
    printf("BME688 sensor initialized\n");

    Pas_co2 pas_co2_sensor(PAS_CO2_ADDRESS, I2C_PORT);
    if (pas_co2_sensor.init() != 0) {
        printf("Sensor initialization failed.\n");
        return 1;
    }
    printf("PAS_CO2 sensor initialized\n");

    // Main loop
    while (true) {
        uint16_t pm1_0, pm2_5, pm10;
        float temperature, humidity, pressure, gas_resistance;

        // Read data from HM3301
        if (hm3301_sensor.read(pm1_0, pm2_5, pm10)) {
            printf("HM3301 - PM1.0: %u µg/m3, PM2.5: %u µg/m3, PM10: %u µg/m3\n", pm1_0, pm2_5, pm10);
        } else {
            printf("Failed to read HM3301 data\n");
        }

        // Read data from BME688
         if (bme688_sensor.readData(temperature, humidity, pressure, gas_resistance)) {
            printf("BME688 - Temperature: %.2f°C, Humidity: %.2f%%, Pressure: %.2f hPa, Gas Resistance: %.2f ohms\n",
                   temperature, humidity, pressure, gas_resistance);
        } else {
            printf("Failed to read BME688 data\n");
        }

        pas_co2_sensor.read();
        printf("CO2 concentration: %u ppm\n", pas_co2_sensor.getResult());

        // Delay between readings
        sleep_ms(1000);
    }
}
