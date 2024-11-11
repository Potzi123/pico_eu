#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/irq.h"
#include "pico/cyw43_arch.h"

#include "password.h"
#include "libs/hm3301/hm3301.h"
#include "libs/bme688/bme688.h"
#include "libs/pas_co2/pas_co2.h"
#include "libs/adc/adc.h"
#include "libs/wifi/wifi.h"
#include "libs/eInk/GUI/GUI_Paint.h"
#include "libs/eInk/EPD_1in54_V2/EPD_1in54_V2.h"
#include "libs/eInk/Fonts/fonts.h"

// Flash and display constants
#define I2C_PORT i2c0
#define I2C_SDA 4
#define I2C_SCL 5
#define HM3301_ADDRESS 0x40
#define BME688_ADDRESS 0x76
#define PAS_CO2_ADDRESS 0x28
#define ADC 26
#define FLASH_TARGET_OFFSET (1792 * 1024)  // 1.8MB offset in 2MB flash

// GPIO for button control
#define TASTER_COUNT 2
#define BUTTON_NEXT_PAGE 20
#define BUTTON_REFRESH_DISPLAY 21
#define NOT_PRESSED 0
#define SHORT_PRESSED 1
#define LONG_PRESSED 2

#define SHORT_PRESSED_TIME 1000
#define LONG_PRESSED_TIME 2000

// Variables for sensors, display, and data
#define PAGE_COUNT 4

bool fist_time = true;

int refreshInterval = 30000;  // in milliseconds
int refreshIntervals[] = {5000, 10000, 30000, 60000, 120000};  // 5se 10 sec, 30 sec, 1 min , 2 min
int currentIntervalIndex = 1;  // Default to 30 seconds

const uint8_t *flash_target_contents = (const uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);
UBYTE *ImageBuffer;
UWORD Imagesize = ((EPD_1IN54_V2_WIDTH % 8 == 0) ? (EPD_1IN54_V2_WIDTH / 8) : (EPD_1IN54_V2_WIDTH / 8 + 1)) * EPD_1IN54_V2_HEIGHT;

//GPIO
volatile uint64_t tast_lasttime[TASTER_COUNT] = {0, 0};;
volatile int tast_pressed[TASTER_COUNT] = {NOT_PRESSED, NOT_PRESSED};
volatile uint8_t tast[TASTER_COUNT] = {BUTTON_NEXT_PAGE, BUTTON_REFRESH_DISPLAY};

myWIFI wifi;
myADC batteryADC(ADC, 10);
HM3301 hm3301_sensor(I2C_PORT, HM3301_ADDRESS, I2C_SDA, I2C_SCL);
BME688 bme688_sensor(I2C_PORT, BME688_ADDRESS, I2C_SDA, I2C_SCL);
Pas_co2 pas_co2_sensor(PAS_CO2_ADDRESS, I2C_PORT);

// Variables for page navigation and timing
volatile int current_page = 0;
volatile bool refresh_display = false;
absolute_time_t last_refresh_time;

// Variables for sensor data storage
float temp = 0.0;
float hum = 0.0;
float pres = 0.0;
float gasRes = 0.0;
uint16_t pm1_0 = 0;
uint16_t pm2_5 = 0;
uint16_t pm10 = 0;
uint32_t co2 = 0;

float batteryLevel = 0;

// Function to initialize the eInk display
void eInk_init() {
    Init_Device();
    EPD_1IN54_V2_Init();
    EPD_1IN54_V2_Clear();
    printf("eInk display initialized and cleared.\n");
}

// Function to reset ImageBuffer and clear display
void resetImageBuffer() {
    if (ImageBuffer) {
        free(ImageBuffer);  // Free the previous buffer memory
    }
    ImageBuffer = (UBYTE *)malloc(Imagesize);  // Allocate new buffer memory
    if (ImageBuffer == NULL) {
        printf("Failed to allocate memory for eInk display buffer.\n");
        return;
    } else {
        printf("Memory allocated for eInk display buffer.\n");
    }
    Paint_NewImage(ImageBuffer, EPD_1IN54_V2_WIDTH, EPD_1IN54_V2_HEIGHT, 270, WHITE);
    Paint_SelectImage(ImageBuffer);
    Paint_Clear(WHITE);
    printf("ImageBuffer reset and cleared.\n");
}

// Initialize the I2C bus
void i2c_init() {
    i2c_init(I2C_PORT, 400000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    printf("I2C bus initialized on SDA: %d, SCL: %d\n", I2C_SDA, I2C_SCL);
}

// Display initial "Hello :)" message on the eInk display
void displayHello() {
    resetImageBuffer();
    Paint_DrawString_EN(10, 5, "Hello :)", &Font24, BLACK, WHITE);
    EPD_1IN54_V2_Display(ImageBuffer);
    printf("Displayed 'Hello :)' on eInk display.\n");
    sleep_ms(1000);
}

// Check if all sensors are initialized properly
void checkSensors() {
    if (hm3301_sensor.begin()) {
        printf("HM3301 sensor initialized successfully.\n");
    } else {
        printf("Failed to initialize HM3301 sensor.\n");
    }

    if (bme688_sensor.begin()) {
        printf("BME688 sensor initialized successfully.\n");
    } else {
        printf("Failed to initialize BME688 sensor.\n");
    }

    if (pas_co2_sensor.init() == 0) {
        printf("PAS_CO2 sensor initialized successfully.\n");
    } else {
        printf("Failed to initialize PAS_CO2 sensor.\n");
    }
}

// Display battery level and sensor values
void displayStatus(float batteryLevel) {
    resetImageBuffer();
    Paint_DrawNum(10, 5, batteryLevel, &Font24, BLACK, WHITE);
    EPD_1IN54_V2_Display(ImageBuffer);
    printf("Displayed battery level and sensor values on eInk display.\n");
}

// Save data to flash
void saveToFlash(uint16_t pm1_0, uint16_t pm2_5, uint16_t pm10, float temperature, float humidity, float pressure, float gas_resistance, float batteryLevel, uint32_t co2) {
    uint32_t flash_data[9] = { pm1_0, pm2_5, pm10, (uint32_t)temperature, (uint32_t)humidity, (uint32_t)pressure, (uint32_t)gas_resistance, (uint32_t)batteryLevel, co2 };
    uint32_t ints = save_and_disable_interrupts(); // Disable interrupts
    flash_range_program(FLASH_TARGET_OFFSET, (uint8_t*)flash_data, sizeof(flash_data));
    restore_interrupts(ints); // Re-enable interrupts
    printf("Saved data to flash memory.\n");
}

void drawBatteryIcon(int x, int y) {
    // Draw battery outline
    Paint_DrawRectangle(x, y, x + 30, y + 15, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    // Draw battery tip
    Paint_DrawRectangle(x + 30, y + 4, x + 32, y + 11, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);

    // Determine the number of filled indicators based on the battery level
    int numIndicators = 0;
    if (batteryLevel >= 77) {
        numIndicators = 4;
    } else if (batteryLevel >= 52) {
        numIndicators = 3;
    } else if (batteryLevel >= 27) {
        numIndicators = 2;
    } else if (batteryLevel >= 5) {
        numIndicators = 1;
    } else {
        numIndicators = 0;
    }

    // Draw filled indicators
    for (int i = 0; i < numIndicators; i++) {
        Paint_DrawRectangle(x + 2 + (i * 6), y + 2, x + 6 + (i * 6), y + 13, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    }

}




// Updated `displayPage` function
void displayPage(int page) {
    resetImageBuffer();
    char buffer[50];

    // Draw battery icon in the top right corner
    drawBatteryIcon(150, 5);

    if (page == 0) {
        // Page 1: BME688 Sensor Data
        Paint_DrawString_EN(10, 5, "BME688", &Font20, BLACK, WHITE);

        // Temp
        sprintf(buffer, "Temp: %.2f C", temp);
        Paint_DrawString_EN(10, 30, buffer, &Font20, BLACK, WHITE);

        // Hum
        sprintf(buffer, "Hum: %.2f %%", hum);
        Paint_DrawString_EN(10, 55, buffer, &Font20, BLACK, WHITE);

        printf("Displayed Page 1: BME688 Data.\n");

    } else if (page == 1) {
        // Page 2: HM3301 Sensor Data
        Paint_DrawString_EN(10, 5, "HM3301", &Font20, BLACK, WHITE);

        // Units header
        Paint_DrawString_EN(10, 30, "Units: ug/m3", &Font20, BLACK, WHITE);

        // PM1.0
        sprintf(buffer, "PM1.0: %u", pm1_0);
        Paint_DrawString_EN(10, 55, buffer, &Font20, BLACK, WHITE);

        // PM2.5
        sprintf(buffer, "PM2.5: %u", pm2_5);
        Paint_DrawString_EN(10, 80, buffer, &Font20, BLACK, WHITE);

        // PM10
        sprintf(buffer, "PM10: %u", pm10);
        Paint_DrawString_EN(10, 105, buffer, &Font20, BLACK, WHITE);

        printf("Displayed Page 2: HM3301 Data.\n");

    } else if (page == 2) {
        // Page 3: PAS CO2 Sensor Data
        Paint_DrawString_EN(10, 5, "PAS CO2", &Font20, BLACK, WHITE);

        // CO2
        sprintf(buffer, "CO2: %u", co2);
        Paint_DrawString_EN(10, 30, buffer, &Font20, BLACK, WHITE);

        // Unit for CO2
        Paint_DrawString_EN(140, 30, "ppm", &Font20, BLACK, WHITE);

        printf("Displayed Page 3: PAS CO2 Data.\n");

    } else if (page == 3) {
        // Page 4: Settings Page
        Paint_DrawString_EN(10, 5, "Settings", &Font20, BLACK, WHITE);
        
        // Display current refresh intervall
        sprintf(buffer, "Refresh: ");
        Paint_DrawString_EN(10, 30, buffer, &Font24, BLACK, WHITE);
        sprintf(buffer, "%d s", refreshInterval/1000);
        Paint_DrawString_EN(10, 60, buffer, &Font24, BLACK, WHITE);
        
        Paint_DrawString_EN(10, 90, "Press button to", &Font16, BLACK, WHITE);
        Paint_DrawString_EN(10, 105, "change intervall", &Font16, BLACK, WHITE);

        printf("Displayed Page 4: Settings.\n");
    }

    EPD_1IN54_V2_Display(ImageBuffer);
}






// GPIO interrupt callback
void gpio_callback(uint gpio, uint32_t events) {
    uint8_t gpio_pin = -1;
    for (int i = 0; i < TASTER_COUNT; i++) {
        if (tast[i] == gpio) {
            gpio_pin = i;
            break;
        }
    }

    if (gpio_pin == -1) return;  // Invalid GPIO

    if (events & GPIO_IRQ_EDGE_FALL) {
        tast_lasttime[gpio_pin] = time_us_64();
    } else if (events & GPIO_IRQ_EDGE_RISE) {
        uint64_t currentTime = time_us_64();
        if ((currentTime - tast_lasttime[gpio_pin]) <= 10000) {
            tast_lasttime[gpio_pin] = currentTime;
            return;
        }
        int pressed_time = (int)((currentTime - tast_lasttime[gpio_pin]) / 1000);
        if (pressed_time < SHORT_PRESSED_TIME) {
            tast_pressed[gpio_pin] = SHORT_PRESSED;
        } else if (pressed_time < LONG_PRESSED_TIME) {
            tast_pressed[gpio_pin] = LONG_PRESSED;
        }
    }
}

void initButtons() {
    gpio_set_dir(BUTTON_NEXT_PAGE, GPIO_IN);
    gpio_pull_up(BUTTON_NEXT_PAGE);
    gpio_set_irq_enabled_with_callback(BUTTON_NEXT_PAGE, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

    gpio_set_dir(BUTTON_REFRESH_DISPLAY, GPIO_IN);
    gpio_pull_up(BUTTON_REFRESH_DISPLAY);
    gpio_set_irq_enabled_with_callback(BUTTON_REFRESH_DISPLAY, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
}

void nextPage() {
    // Increment the current page index and wrap around if needed
    current_page = (current_page + 1) % PAGE_COUNT;  // Assume 3 pages

    // Print a message for debugging/logging purposes
    printf("Switched to page %d.\n", current_page);
}

void refreshDisplay_Settings_Button(int page) {
    if (current_page == 3) {  // If on the settings page
        // Cycle through the predefined refresh intervals
        currentIntervalIndex = (currentIntervalIndex + 1) % (sizeof(refreshIntervals) / sizeof(refreshIntervals[0]));
        refreshInterval = refreshIntervals[currentIntervalIndex];
        printf("Updated refresh interval to %d ms.\n", refreshInterval);
    } else {
        refresh_display = true;
    }
}




int main() {
    stdio_init_all();

    eInk_init();
    i2c_init();  // Initialize I2C before checking sensors
    checkSensors();
    batteryADC.init();
    initButtons();
    displayHello();



    last_refresh_time = get_absolute_time();

    while (true) {

        // Handle button presses
        if (tast_pressed[0] == SHORT_PRESSED) {
            nextPage();
            refresh_display = true;
            tast_pressed[0] = NOT_PRESSED;
        }

        if (tast_pressed[1] == SHORT_PRESSED) {
            refreshDisplay_Settings_Button(current_page);
            refresh_display = true;
            tast_pressed[1] = NOT_PRESSED;
        }
        
        // Refresh the display either on button press or after the defined interval
        if (refresh_display || absolute_time_diff_us(last_refresh_time, get_absolute_time()) >= refreshInterval * 1000) {
            gpio_set_irq_enabled_with_callback(BUTTON_REFRESH_DISPLAY, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, false, &gpio_callback);
            displayPage(current_page);  // Pass batteryLevel as an argument
            last_refresh_time = get_absolute_time();
            refresh_display = false;
            gpio_set_irq_enabled_with_callback(BUTTON_REFRESH_DISPLAY, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
          
        }

        batteryLevel = batteryADC.calculateBatteryLevel();

        // Read sensor data and store in flash at each interval

        if (bme688_sensor.readData(temp, hum, pres, gasRes)) {
            printf("Updated BME688 data - Temp: %.2f, Hum: %.2f, Pres: %.2f, Gas: %.2f\n", temp, hum, pres, gasRes);
        }

        if (hm3301_sensor.read(pm1_0, pm2_5, pm10)) {
            printf("Updated HM3301 data - PM1.0: %u, PM2.5: %u, PM10: %u\n", pm1_0, pm2_5, pm10);
        }

        pas_co2_sensor.read();
        co2 = pas_co2_sensor.getResult();
        printf("PAS_CO2 sensor data - CO2: %u ppm\n", co2);

        printf("> temp: %.2f, hum: %.2f, pres: %.2f, gas: %.2f, co2: %u, pm1_0: %u, pm2_5: %u, pm10: %u\n", temp, hum, pres, gasRes, co2, pm1_0, pm2_5, pm10);

        if(fist_time == true){
            fist_time = false;
            displayPage(current_page);  // Pass batteryLevel as an argument
            last_refresh_time = get_absolute_time();
            refresh_display = false;
        }

        //saveToFlash(pm1_0, pm2_5, pm10, temperature, humidity, pressure, gas_resistance, batteryLevel, co2);

        sleep_ms(50);  // Short sleep to reduce CPU load in the main loop
    }

    free(ImageBuffer);
    return 0;
}
