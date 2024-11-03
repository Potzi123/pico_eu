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
#include "libs/wifi/wifi.h"
#include "libs/eInk/GUI/GUI_Paint.h"
#include "libs/eInk/EPD_1in54_V2/EPD_1in54_V2.h"
#include "libs/eInk/Fonts/fonts.h"


// I2C configuration and sensor addresses
#define I2C_PORT i2c0
#define I2C_SDA 4
#define I2C_SCL 5
#define HM3301_ADDRESS 0x40
#define BME688_ADDRESS 0x76
#define PAS_CO2_ADDRESS 0x28

#define ADC 26

myWIFI wifi;
myADC batteryADC(ADC);
HM3301 hm3301_sensor(I2C_PORT, HM3301_ADDRESS, I2C_SDA, I2C_SCL);
BME688 bme688_sensor(I2C_PORT, BME688_ADDRESS, I2C_SDA, I2C_SCL);
Pas_co2 pas_co2_sensor(PAS_CO2_ADDRESS, I2C_PORT);

// Create new image cache
    UBYTE *ImageBuffer;
    UWORD Imagesize = ((EPD_1IN54_V2_WIDTH % 8 == 0) ? (EPD_1IN54_V2_WIDTH / 8) : (EPD_1IN54_V2_WIDTH / 8 + 1)) * EPD_1IN54_V2_HEIGHT;

void eInk(){
    // Init EPD //
    Init_Device();
    printf("init done\n");
    EPD_1IN54_V2_Init();
    printf("EPD initialized\n");
    EPD_1IN54_V2_Clear();
    printf("EPD cleared\n");
    if ((ImageBuffer = (UBYTE *)malloc(Imagesize)) == NULL)
    {
        printf("Failed to apply for memory...\n");
    }
    Paint_NewImage(ImageBuffer, EPD_1IN54_V2_WIDTH, EPD_1IN54_V2_HEIGHT, 270, WHITE);
    printf("Paint_NewImage done\n");
    Paint_SelectImage(ImageBuffer);
    Paint_Clear(WHITE);
}

int wifi_init() {
    // Initialize WiFi
    if (wifi.init() != 0) {
        printf("WiFi initialization failed.\n");
        return 1;
    }
    printf("WiFi initialized successfully.\n");

    // Scan and connect to WiFi
    if (wifi.scanAndConnect() != 0) {
        printf("WiFi scan and connection attempt failed.\n");
        return 1;
    }
    printf("Successfully connected to WiFi.\n");
    return 0;
}

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

    eInk();

    Paint_DrawString_EN(10, 5, "Hello World", &Font24, BLACK, WHITE); //Write "Hello World" in the Buffer
    Paint_DrawNum(10, 25, 03112024, &Font20, BLACK, WHITE); // Write 03112024 in the Buffer
    EPD_1IN54_V2_Display(ImageBuffer); //Display the Buffer on e-Paper Display
    EPD_1IN54_V2_Sleep(); //enter deep sleep

    sleep_ms(2000);

    if(wifi_init() != 0) {
        printf("WiFi initialization failed.\n");
        return 1;
    }
    printf("WiFi initialized successfully.\n");



    i2c_init();
    printf("I2C initialized\n");


    batteryADC.init();
    printf("ADC initialized\n");


    // Initialize the HM3301 sensor
    if (!hm3301_sensor.begin()) {
        printf("Failed to initialize HM3301 sensor\n");
        return 1;
    }
    printf("HM3301 sensor initialized\n");


    if (!bme688_sensor.begin()) {
        printf("Failed to initialize BME688 sensor\n");
        return 1;
    }
    printf("BME688 sensor initialized\n");

    if (pas_co2_sensor.init() != 0) {
        printf("Sensor initialization failed.\n");
        return 1;
    }
    printf("PAS_CO2 sensor initialized\n");

    // Main loop
    while (true) {
        uint16_t pm1_0, pm2_5, pm10;
        float temperature, humidity, pressure, gas_resistance;

        wifi.poll();
        if (wifi.getConnected() == CYW43_LINK_UP) {
            printf("WiFi is connected.\n");
        } else {
            printf("WiFi is not connected.\n");
        }


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

        // Delay between readings
        sleep_ms(1000);
    }

    free(ImageBuffer);
    return 0;
}
