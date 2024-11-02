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
#include "libs/adc/adc.h"
#include "libs/gps/gps.h"


// I2C configuration and sensor addresses
#define I2C_PORT i2c0
#define I2C_SDA 4
#define I2C_SCL 5
#define HM3301_ADDRESS 0x40
#define BME688_ADDRESS 0x76
#define PAS_CO2_ADDRESS 0x28

#define UART0_UART_ID uart0
#define UART0_BAUD_RATE 9600
#define UART0_TX_PIN 0
#define UART0_RX_PIN 1

#define ADC 26

void i2c_init() {
    i2c_init(I2C_PORT, 400000);  // Initialize I2C at 400kHz
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
}

void setup_uart() {
    // Initialize UART for GPS communication
    uart_init(UART0_UART_ID, UART0_BAUD_RATE);
    gpio_set_function(UART0_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART0_RX_PIN, GPIO_FUNC_UART);
}


int main() {



    stdio_init_all();

    while (!stdio_usb_connected()) {
        sleep_ms(100);  // Poll every 100ms
    }

    i2c_init();
    printf("I2C initialized\n");


    myADC batteryADC(ADC);
    batteryADC.init();
    printf("ADC initialized\n");


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


    // Initialize GPS
    std::string buffer;
    double latitude, longitude;
    char nsIndicator, ewIndicator;
    std::string time;


    setup_uart();
    myGPS gps(UART0_UART_ID, UART0_BAUD_RATE, UART0_TX_PIN, UART0_RX_PIN);
    gps.init();
    printf("GPS initialized\n");


    // Main loop
    while (true) {
        
        //HM3301
        uint16_t pm1_0, pm2_5, pm10;
        
        //BME688
        float temperature, humidity, pressure, gas_resistance;


        float batteryLevel = batteryADC.calculateBatteryLevel();
        printf("Battery level: %.2f%%\n", batteryLevel);

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


        // Attempt to read a line of GPS data
        if (gps.readLine(buffer, latitude, nsIndicator, longitude, ewIndicator, time) == 0) {
            // Display the parsed GPS data
            printf("GPS Data: Latitude = %.6f%c, Longitude = %.6f%c, Time = %s\n", 
                   latitude, nsIndicator, longitude, ewIndicator, time.c_str());
        } else {
            // Print error if GPS data could not be read
            printf("Failed to read GPS data or no data available.\n");
        }



        // Delay between readings
        sleep_ms(1000);
    }
}
