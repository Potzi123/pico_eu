#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <vector>
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
#include "libs/gps/myGPS.h"
#include "libs/flash/flash.h"
#include <cstdio>



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
#define BUTTON_NEXT_PAGE 18
#define BUTTON_REFRESH_DISPLAY 19
#define NOT_PRESSED 0
#define SHORT_PRESSED 1
#define LONG_PRESSED 2

#define SHORT_PRESSED_TIME 1000
#define LONG_PRESSED_TIME 2000

// Variables for sensors, display, and data
#define PAGE_COUNT 4

#define TLS_CLIENT_SERVER        "www.gm4s.eu"
/*#define TLS_CLIENT_HTTP_REQUEST  "GET /api/test HTTP/1.1\r\n" \
                                 "Host: " TLS_CLIENT_SERVER "\r\n" \
                                 "Connection: close\r\n" \
                                 "\r\n"*/
#define TLS_CLIENT_HTTP_REQUEST  "POST /api/addMarker HTTP/1.1\r\n" \
                                 "Host: " TLS_CLIENT_SERVER "\r\n" \
                                 "Content-Type: application/json\r\n" \
                                 "Content-Length: 203\r\n" \
                                 "Connection: close\r\n" \
                                 "\r\n" \
                                 "{\"token\":\"86ea63a5-4ea6-4bd1-88f0-bb370970dd16\"," \
                                 "\"measured_at\":\"2024-10-23T12:28:02.379+00:00\"," \
                                 "\"lat\":48.2072620612573,\"long\":15.61750700948781,\"co2\":3,\"hum\":4," \
                                 "\"temp\":5,\"part_2_5\":6,\"part_5\":7,\"part_10\":8}"
#define TLS_CLIENT_TIMEOUT_SECS  6000

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
SensorData sensor_data_obj = {0,0,0,0,0,0,0,0,0,0,0};
std::vector<SensorData> sensor_data;

float batteryLevel = 0;

extern bool run_tls_client_test(const uint8_t *cert, size_t cert_len, const char *server, const char *request, int timeout);

// After other variable declarations, add:
Flash flash_storage;  // Create flash storage object
absolute_time_t last_flash_write_time;  // For timing flash writes
bool flash_initialized = false;

// Function to initialize the eInk display
void eInk_init() {
    Init_Device();
    EPD_1IN54_V2_Init();
    EPD_1IN54_V2_Clear();
    // printf("eInk display initialized and cleared.\n");
}

// Function to reset ImageBuffer and clear display
void resetImageBuffer() {
    if (ImageBuffer) {
        free(ImageBuffer);  // Free the previous buffer memory
    }
    ImageBuffer = (UBYTE *)malloc(Imagesize);  // Allocate new buffer memory
    if (ImageBuffer == NULL) {
        printf("ERROR: Failed to allocate memory for eInk display buffer.\n");
        return;
    }
    Paint_NewImage(ImageBuffer, EPD_1IN54_V2_WIDTH, EPD_1IN54_V2_HEIGHT, 270, WHITE);
    Paint_SelectImage(ImageBuffer);
    Paint_Clear(WHITE);
}

// Initialize the I2C bus
void i2c_init() {
    i2c_init(I2C_PORT, 400000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
}

// Display initial "Hello :)" message on the eInk display
void displayHello() {
    resetImageBuffer();
    Paint_DrawString_EN(10, 5, "Hello :)", &Font24, BLACK, WHITE);
    EPD_1IN54_V2_Display(ImageBuffer);
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

void drawWiFiConnectedIcon(int x, int y) {
    // Set the top position for the icon
    int topY = y + 10;  // Adjust `10` to bring it further down if necessary

    // Outer arc
    for (float angle = 0; angle <= M_PI; angle += 0.1) {
        int x1 = x + (int)(10 * cos(angle));  // Radius 10
        int y1 = topY - (int)(10 * sin(angle));  // Subtract from topY
        int x2 = x + (int)(10 * cos(angle + 0.1));
        int y2 = topY - (int)(10 * sin(angle + 0.1));
        Paint_DrawLine(x1, y1, x2, y2, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    }

    // Middle arc
    for (float angle = 0; angle <= M_PI; angle += 0.1) {
        int x1 = x + (int)(7 * cos(angle));  // Radius 7
        int y1 = topY - (int)(7 * sin(angle));  // Subtract from topY
        int x2 = x + (int)(7 * cos(angle + 0.1));
        int y2 = topY - (int)(7 * sin(angle + 0.1));
        Paint_DrawLine(x1, y1, x2, y2, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    }

    // Inner arc
    for (float angle = 0; angle <= M_PI; angle += 0.1) {
        int x1 = x + (int)(4 * cos(angle));  // Radius 4
        int y1 = topY - (int)(4 * sin(angle));  // Subtract from topY
        int x2 = x + (int)(4 * cos(angle + 0.1));
        int y2 = topY - (int)(4 * sin(angle + 0.1));
        Paint_DrawLine(x1, y1, x2, y2, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    }

    // Dot for signal
    Paint_DrawPoint(x, topY + 5, BLACK, DOT_PIXEL_2X2, DOT_STYLE_DFT);  // Adjust dot position below arcs
}





void drawWiFiDisconnectedIcon(int x, int y) {
    // Set the top position for the icon
    int topY = y + 10;  // Adjust `10` to bring it further down if necessary

    // Outer arc
    for (float angle = 0; angle <= M_PI; angle += 0.1) {
        int x1 = x + (int)(10 * cos(angle));  // Radius 10
        int y1 = topY - (int)(10 * sin(angle));  // Subtract from topY
        int x2 = x + (int)(10 * cos(angle + 0.1));
        int y2 = topY - (int)(10 * sin(angle + 0.1));
        Paint_DrawLine(x1, y1, x2, y2, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    }

    // Middle arc
    for (float angle = 0; angle <= M_PI; angle += 0.1) {
        int x1 = x + (int)(7 * cos(angle));  // Radius 7
        int y1 = topY - (int)(7 * sin(angle));  // Subtract from topY
        int x2 = x + (int)(7 * cos(angle + 0.1));
        int y2 = topY - (int)(7 * sin(angle + 0.1));
        Paint_DrawLine(x1, y1, x2, y2, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    }

    // Inner arc
    for (float angle = 0; angle <= M_PI; angle += 0.1) {
        int x1 = x + (int)(4 * cos(angle));  // Radius 4
        int y1 = topY - (int)(4 * sin(angle));  // Subtract from topY
        int x2 = x + (int)(4 * cos(angle + 0.1));
        int y2 = topY - (int)(4 * sin(angle + 0.1));
        Paint_DrawLine(x1, y1, x2, y2, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    }

    // Line through the symbol (diagonal slash for "disconnected")
    Paint_DrawLine(x - 10, topY + 5, x + 10, topY - 5, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
}







// Updated `displayPage` function
void displayPage(int page) {
    resetImageBuffer();
    char buffer[50];

    // Draw battery icon in the top right corner
    drawBatteryIcon(150, 5);

    // Draw WiFi symbol to the left of the battery icon
    if (wifi.getConnected() == 3) {
        // Connected symbol
        drawWiFiConnectedIcon(130, 5);  // Adjust the position as needed
    } else {
        // Disconnected symbol
        drawWiFiDisconnectedIcon(130, 5);  // Adjust the position as needed
    }

    if (page == 0) {
        // Page 1: BME688 Sensor Data
        Paint_DrawString_EN(10, 5, "BME688", &Font20, BLACK, WHITE);

        // Temp
        sprintf(buffer, "Temp: %.2f C", sensor_data_obj.temp);
        Paint_DrawString_EN(10, 30, buffer, &Font20, BLACK, WHITE);

        // Hum
        sprintf(buffer, "Hum: %.2f %%", sensor_data_obj.hum);
        Paint_DrawString_EN(10, 55, buffer, &Font20, BLACK, WHITE);

        printf("Displayed Page 1: BME688 Data.\n");

    } else if (page == 1) {
        // Page 2: HM3301 Sensor Data
        Paint_DrawString_EN(10, 5, "HM3301", &Font20, BLACK, WHITE);

        // Units header
        Paint_DrawString_EN(10, 30, "Units: ug/m3", &Font20, BLACK, WHITE);

        // PM1.0
        sprintf(buffer, "PM1.0: %u", sensor_data_obj.pm2_5);
        Paint_DrawString_EN(10, 55, buffer, &Font20, BLACK, WHITE);

        // PM2.5
        sprintf(buffer, "PM2.5: %u", sensor_data_obj.pm5);
        Paint_DrawString_EN(10, 80, buffer, &Font20, BLACK, WHITE);

        // PM10
        sprintf(buffer, "PM10: %u", sensor_data_obj.pm10);
        Paint_DrawString_EN(10, 105, buffer, &Font20, BLACK, WHITE);

        printf("Displayed Page 2: HM3301 Data.\n");

    } else if (page == 2) {
        // Page 3: PAS CO2 Sensor Data
        Paint_DrawString_EN(10, 5, "PAS CO2", &Font20, BLACK, WHITE);

        // CO2
        sprintf(buffer, "CO2: %u", sensor_data_obj.co2);
        Paint_DrawString_EN(10, 30, buffer, &Font20, BLACK, WHITE);

        // Unit for CO2
        Paint_DrawString_EN(140, 30, "ppm", &Font20, BLACK, WHITE);

        printf("Displayed Page 3: PAS CO2 Data.\n");

    } else if (page == 3) {
        // Page 4: Settings Page
        Paint_DrawString_EN(10, 5, "Settings", &Font20, BLACK, WHITE);
        
        // Display current refresh interval
        sprintf(buffer, "Refresh: ");
        Paint_DrawString_EN(10, 30, buffer, &Font24, BLACK, WHITE);
        sprintf(buffer, "%d s", refreshInterval / 1000);
        Paint_DrawString_EN(10, 60, buffer, &Font24, BLACK, WHITE);
        
        Paint_DrawString_EN(10, 90, "Press button to", &Font16, BLACK, WHITE);
        Paint_DrawString_EN(10, 105, "change interval", &Font16, BLACK, WHITE);

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

    myGPS gps(UART0_UART_ID, UART0_BAUD_RATE, UART0_TX_PIN, UART0_RX_PIN);

    std::string buffer;
    double latitude = 0;
    char nsIndicator = ' ';
    double longitude = 0;
    char ewIndicator = ' ';
    std::string time;

    wifi.init();
    wifi.scanAndConnect();

    // Initialize flash storage
    flash_initialized = flash_storage.init();
    if (flash_initialized) {
        printf("Flash storage initialized successfully. Stored records: %lu\n", 
               flash_storage.getStoredCount());
    } else {
        printf("Failed to initialize flash storage.\n");
    }

    // Reset flash storage for testing (remove this in production)
    if (flash_initialized) {
        printf("Resetting flash storage for testing...\n");
        flash_storage.resetStorage();
        printf("Flash storage reset. Stored records: %lu\n", flash_storage.getStoredCount());
    }

    // Read existing data from flash
    if (flash_initialized) {
        std::vector<SensorData> loaded_data = flash_storage.loadSensorData();
        printf("Loaded %zu records from flash storage.\n", loaded_data.size());
        
        // Display the last 5 records or fewer if less are available
        size_t start_idx = loaded_data.size() > 5 ? loaded_data.size() - 5 : 0;
        for (size_t i = start_idx; i < loaded_data.size(); i++) {
            printf("Record %zu: Temp=%.2f, Hum=%.2f, CO2=%u, PM2.5=%u\n", 
                   i, loaded_data[i].temp, loaded_data[i].hum, 
                   loaded_data[i].co2, loaded_data[i].pm2_5);
        }
    }
    
    last_flash_write_time = get_absolute_time();
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
        

        if (tast_pressed[0] == LONG_PRESSED) {
            tast_pressed[0] = NOT_PRESSED;
            wifi.scanAndConnect();
            refresh_display = true;
        }

        if (tast_pressed[1] == LONG_PRESSED) {
            if(wifi.getConnected() == 3) {
                
            }
        }
            



        // Refresh the display either on button press or after the defined interval
        if (refresh_display || absolute_time_diff_us(last_refresh_time, get_absolute_time()) >= refreshInterval * 1000) {
            gpio_set_irq_enabled_with_callback(BUTTON_REFRESH_DISPLAY, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, false, &gpio_callback);
            gpio_set_irq_enabled_with_callback(BUTTON_NEXT_PAGE, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, false, &gpio_callback);

            // Take measurements from all sensors
            if (bme688_sensor.readData(sensor_data_obj.temp, sensor_data_obj.hum, sensor_data_obj.pres, sensor_data_obj.gasRes)) {
                // Data is now in sensor_data_obj - no need for any extra copying
                // printf("Updated BME688 data - Temp: %.2f, Hum: %.2f, Pres: %.2f, Gas: %.2f\n", sensor_data_obj.temp, sensor_data_obj.hum, sensor_data_obj.pres, sensor_data_obj.gasRes);
            }

            if (hm3301_sensor.read(sensor_data_obj.pm2_5, sensor_data_obj.pm5, sensor_data_obj.pm10)) {
                // Data is now in sensor_data_obj - no need for any extra copying
                // printf("Updated HM3301 data - PM1.0: %u, PM2.5: %u, PM10: %u\n", sensor_data_obj.pm2_5, sensor_data_obj.pm5, sensor_data_obj.pm10);
            }

            pas_co2_sensor.read();
            sensor_data_obj.co2 = pas_co2_sensor.getResult();
            
            // Get current timestamp
            sensor_data_obj.timestamp = to_ms_since_boot(get_absolute_time());

            // Add a debug print to verify values before saving
            printf("DEBUG: About to save - Time=%u, Temp=%.2f, Hum=%.2f, CO2=%u, PM2.5=%u, PM10=%u\n",
                   sensor_data_obj.timestamp, sensor_data_obj.temp, sensor_data_obj.hum, 
                   sensor_data_obj.co2, sensor_data_obj.pm2_5, sensor_data_obj.pm10);

            // Save data to flash each time we measure
            if (flash_initialized) {
                bool saved = flash_storage.saveSensorData(sensor_data_obj);
                if (saved) {
                    printf("FLASH: Saved sensor data at %u ms. Total records: %lu\n", 
                           sensor_data_obj.timestamp, flash_storage.getStoredCount());
                           
                    // After saving, display all data in flash
                    std::vector<SensorData> loaded_data = flash_storage.loadSensorData();
                    printf("FLASH: All %zu records from flash:\n", loaded_data.size());
                    
                    for (size_t i = 0; i < loaded_data.size(); i++) {
                        printf("FLASH: Record %zu: Time=%u, Temp=%.2f, Hum=%.2f, CO2=%u, PM2.5=%u, PM10=%u\n",
                               i, loaded_data[i].timestamp, loaded_data[i].temp, 
                               loaded_data[i].hum, loaded_data[i].co2, 
                               loaded_data[i].pm2_5, loaded_data[i].pm10);
                    }
                    
                    if (flash_storage.getStoredCount() > 0.9 * flash_storage.getMaxDataCount()) {
                        printf("FLASH WARNING: Storage almost full (%lu/%lu records).\n", 
                               flash_storage.getStoredCount(), flash_storage.getMaxDataCount());
                    }
                } else {
                    printf("FLASH ERROR: Failed to save data. Storage might be full.\n");
                }
            }

            displayPage(current_page);
            last_refresh_time = get_absolute_time();
            refresh_display = false;
            batteryLevel = batteryADC.calculateBatteryLevel();

            sensor_data.push_back(sensor_data_obj);

            gpio_set_irq_enabled_with_callback(BUTTON_REFRESH_DISPLAY, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
            gpio_set_irq_enabled_with_callback(BUTTON_NEXT_PAGE, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
        }

        batteryLevel = batteryADC.calculateBatteryLevel();

        //gps.readLine(buffer, longitude, nsIndicator, latitude, ewIndicator, time);

        //printf("GPS: longitude: %f, latitude: %f, time: %s\n", buffer.c_str(), longitude, latitude, time.c_str());

        

        sleep_ms(50);  // Short sleep to reduce CPU load in the main loop

        if(fist_time == true){
                    fist_time = false;
                    displayPage(current_page);  // Pass batteryLevel as an argument
                    last_refresh_time = get_absolute_time();
                    refresh_display = false;
        }

        //printf("wifi status : %d\n", wifi.getConnected());
        wifi.poll();
            
    }

    free(ImageBuffer);
    return 0;
}
