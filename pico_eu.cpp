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
#include "hardware/watchdog.h"
#include "pico/cyw43_arch.h"

// Comment this line to disable the watchdog timer
// #define USE_WATCHDOG

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

// Add this with other defines at the top of the file
#define ENABLE_GPS_DEBUG 0  // Set to 1 to enable verbose GPS debugging

// Time offset for system time (seconds since Jan 1, 1970)
// We'll set this to a reasonable value to avoid 1970 timestamps
static time_t time_offset = 0;

// Custom time function to override weak time() from SDK
extern "C" time_t time(time_t* t) {
    time_t current = to_ms_since_boot(get_absolute_time()) / 1000 + time_offset;
    if (t) *t = current;
    return current;
}

// Function to set system time based on offset from 1970
void set_system_time(time_t new_time) {
    time_offset = new_time - (to_ms_since_boot(get_absolute_time()) / 1000);
    // Print the updated system time
    time_t current = time(NULL);
    struct tm *timeinfo = gmtime(&current);
    printf("System time set to: %04d-%02d-%02d %02d:%02d:%02d UTC\n",
           timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday,
           timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
}

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
#define TASTER_COUNT 2  // Changed back to 2 buttons
#define BUTTON_NEXT_PAGE 18
#define BUTTON_REFRESH_DISPLAY 19
#define NOT_PRESSED 0
#define SHORT_PRESSED 1
#define LONG_PRESSED 2

#define SHORT_PRESSED_TIME 1000
#define LONG_PRESSED_TIME 2000

// Variables for sensors, display, and data
#define PAGE_COUNT 5

#define TLS_CLIENT_SERVER        "www.gm4s.eu"
/*#define TLS_CLIENT_HTTP_REQUEST  "GET /api/test HTTP/1.1\r\n" \
                                 "Host: " TLS_CLIENT_SERVER "\r\n" \
                                 "Connection: close\r\n" \
                                 "\r\n"*/
#define TLS_CLIENT_HTTP_REQUEST  "POST /api/addMarkers HTTP/1.1\r\n" \
                                 "Host: " TLS_CLIENT_SERVER "\r\n" \
                                 "Content-Type: application/json\r\n" \
                                 "Content-Length: 320\r\n" \
                                 "Connection: close\r\n" \
                                 "\r\n" \
                                 "{\"token\":\"86ea63a5-4ea6-4bd1-88f0-bb370970dd16\"," \
                                 "\"measurements\":[" \
                                 "{\"measured_at\":\"2024-11-08 12:12:12.121+00\"," \
                                 "\"lat\":48.20662016908546,\"long\":15.617513602109687,\"co2\":1656,\"hum\":32.8," \
                                 "\"temp\":27.79,\"part_2_5\":2,\"part_5\":3,\"part_10\":55555555}" \
                                 "]}"
#define TLS_CLIENT_TIMEOUT_SECS  6000

bool fist_time = true;

int refreshInterval = 5000;  // in milliseconds
int refreshIntervals[] = {5000, 10000, 30000, 60000, 120000};  // 5se 10 sec, 30 sec, 1 min , 2 min
int currentIntervalIndex = 1;  // Default to 30 seconds

const uint8_t *flash_target_contents = (const uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);
UBYTE *ImageBuffer;
UWORD Imagesize = ((EPD_1IN54_V2_WIDTH % 8 == 0) ? (EPD_1IN54_V2_WIDTH / 8) : (EPD_1IN54_V2_WIDTH / 8 + 1)) * EPD_1IN54_V2_HEIGHT;

//GPIO
volatile uint64_t tast_lasttime[TASTER_COUNT] = {0, 0};  // Changed back to 2 elements
volatile int tast_pressed[TASTER_COUNT] = {NOT_PRESSED, NOT_PRESSED};  // Changed back to 2 elements
volatile uint8_t tast[TASTER_COUNT] = {BUTTON_NEXT_PAGE, BUTTON_REFRESH_DISPLAY};  // Changed back to 2 elements

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

// Modify the external function declaration to match the expected signature exactly
extern "C" bool run_tls_client_test(unsigned char const* cert, unsigned int cert_len, char const* server, char const* request, int timeout);

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

// Add this function after the drawWiFiDisconnectedIcon function
void displayGPSStatus(absolute_time_t gps_start_time, int fix_status, int satellites_visible) {
    resetImageBuffer();
    char buffer[50];
    
    // Draw battery icon in the top right corner
    drawBatteryIcon(150, 5);

    // Draw WiFi symbol to the left of the battery icon
    if (wifi.getConnected() == 3) {
        // Connected symbol
        drawWiFiConnectedIcon(130, 5);
    } else {
        // Disconnected symbol
        drawWiFiDisconnectedIcon(130, 5);
    }
    
    // GPS Status Page Title
    Paint_DrawString_EN(10, 5, "GPS Status", &Font20, BLACK, WHITE);
    
    // Show time since GPS start
    uint32_t seconds_since_start = to_ms_since_boot(get_absolute_time()) / 1000 - to_ms_since_boot(gps_start_time) / 1000;
    sprintf(buffer, "Time: %d min %d sec", seconds_since_start / 60, seconds_since_start % 60);
    Paint_DrawString_EN(10, 30, buffer, &Font16, BLACK, WHITE);
    
    // Show fix status
    if (fix_status == 0) {
        Paint_DrawString_EN(10, 50, "Fix: VALID", &Font16, BLACK, WHITE);
    } else {
        Paint_DrawString_EN(10, 50, "Fix: waiting...", &Font16, BLACK, WHITE);
    }
    
    // Show satellites visible
    sprintf(buffer, "Satellites: %d", satellites_visible);
    Paint_DrawString_EN(10, 70, buffer, &Font16, BLACK, WHITE);
    
    // Instructions
    Paint_DrawString_EN(10, 90, "For best results:", &Font12, BLACK, WHITE);
    Paint_DrawString_EN(10, 105, "- Go outdoors", &Font12, BLACK, WHITE);
    Paint_DrawString_EN(10, 120, "- Clear sky view", &Font12, BLACK, WHITE);
    Paint_DrawString_EN(10, 135, "- Wait 5+ minutes", &Font12, BLACK, WHITE);
    
    EPD_1IN54_V2_Display(ImageBuffer);
}

// Modify the displayPage function to include the new GPS status page
void displayPage(int page, absolute_time_t gps_start_time, int fix_status, int satellites_visible) {
    if (page == 4) {  // GPS Status Page
        displayGPSStatus(gps_start_time, fix_status, satellites_visible);
        return;
    }
    
    resetImageBuffer();
    char buffer[50];

    // Draw battery icon in the top right corner
    drawBatteryIcon(150, 5);

    // Draw WiFi symbol to the left of the battery icon
    if (wifi.getConnected() == 3) {
        // Connected symbol
        drawWiFiConnectedIcon(130, 5);
    } else {
        // Disconnected symbol
        drawWiFiDisconnectedIcon(130, 5);
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

// Add this function before the main function
void displayUploadStatus(const char* message) {
    // Reset the display buffer
    resetImageBuffer();
    
    // Display the upload status message
    Paint_DrawString_EN(10, 5, "Data Upload", &Font24, BLACK, WHITE);
    Paint_DrawString_EN(10, 40, message, &Font16, BLACK, WHITE);
    
    // Draw WiFi symbol
    if (wifi.getConnected() == CYW43_LINK_UP) {
        drawWiFiConnectedIcon(130, 5);
    } else {
        drawWiFiDisconnectedIcon(130, 5);
    }
    
    // Draw battery level
    drawBatteryIcon(150, 5);
    
    // Update the display
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

// Add this function to format the data as JSON
void prepareDataForTransmission(const SensorData& data, char* json_buffer, size_t buffer_size) {
    snprintf(json_buffer, buffer_size,
             "{\"token\":\"86ea63a5-4ea6-4bd1-88f0-bb370970dd16\","
             "\"measured_at\":\"%u\","  // Use timestamp
             "\"lat\":%.7f,\"long\":%.7f,\"co2\":%u,\"hum\":%.2f,"
             "\"temp\":%.2f,\"part_2_5\":%u,\"part_5\":%u,\"part_10\":%u}",
             data.timestamp,
             data.latitude / 10000000.0,   // Convert back to float
             data.longitude / 10000000.0,  // Convert back to float
             data.co2,
             data.hum,
             data.temp,
             data.pm2_5,
             data.pm5,
             data.pm10);
}

// Format multiple sensor data records as a JSON array for transmission
void prepareBatchDataForTransmission(const std::vector<SensorData>& data_vec, char* json_buffer, size_t buffer_size, myGPS& gps) {
    // Start with the opening part of the JSON
    int written = snprintf(json_buffer, buffer_size,
                         "{\"token\":\"86ea63a5-4ea6-4bd1-88f0-bb370970dd16\","
                         "\"measurements\":[");
    
    size_t remaining = buffer_size - written;
    char* current_pos = json_buffer + written;
    
    // Add each measurement as an object in the array
    for (size_t i = 0; i < data_vec.size(); i++) {
        const SensorData& data = data_vec[i];
        
        // Get the current time from GPS if available, otherwise use system time
        time_t raw_time = time(NULL); // Use current time instead of data.timestamp
        struct tm *timeinfo = gmtime(&raw_time);
        
        // Get the GPS time string
        std::string gps_line_save;
        double lon_save = 0, lat_save = 0;
        char ns_save = ' ', ew_save = ' ';
        std::string time_str_save;
        
        char timestamp[64];
        
        // Try to get time from GPS for current time reference
        gps.readLine(gps_line_save, lon_save, ew_save, lat_save, ns_save, time_str_save);
        
        if (!time_str_save.empty() && time_str_save != "00:00:00") {
            // We have a valid GPS time - parse it
            int hours = 0, minutes = 0, seconds = 0;
            if (sscanf(time_str_save.c_str(), "%d:%d:%d", &hours, &minutes, &seconds) == 3) {
                // Format the complete ISO timestamp using the current system date and GPS time for hours/minutes/seconds
                snprintf(timestamp, sizeof(timestamp), 
                     "%04d-%02d-%02d %02d:%02d:%02d+00:00",
                     timeinfo->tm_year + 1900, 
                     timeinfo->tm_mon + 1, 
                     timeinfo->tm_mday, 
                     hours, minutes, seconds);
                printf("Record %zu: Using GPS time for timestamp: %s\n", i, timestamp);
            } else {
                // Fallback if parsing fails
                snprintf(timestamp, sizeof(timestamp), "%04d-%02d-%02d %02d:%02d:%02d+00:00",
                     timeinfo->tm_year + 1900, 
                     timeinfo->tm_mon + 1, 
                     timeinfo->tm_mday,
                     timeinfo->tm_hour, 
                     timeinfo->tm_min, 
                     timeinfo->tm_sec);
                printf("Record %zu: Using system time (GPS parse failed): %s\n", i, timestamp);
            }
        } else {
            // Fallback to system time if GPS time is not available
            snprintf(timestamp, sizeof(timestamp), "%04d-%02d-%02d %02d:%02d:%02d+00:00",
                 timeinfo->tm_year + 1900, 
                 timeinfo->tm_mon + 1, 
                 timeinfo->tm_mday,
                 timeinfo->tm_hour, 
                 timeinfo->tm_min, 
                 timeinfo->tm_sec);
            printf("Record %zu: Using system time (no GPS time): %s\n", i, timestamp);
        }
        
        int record_len = snprintf(current_pos, remaining,
                    "%s{"  // Add comma except for the first item
                    "\"measured_at\":\"%s\","
                    "\"lat\":%.7f,\"long\":%.7f,"
                    "\"co2\":%u,\"hum\":%.2f,"
                    "\"temp\":%.2f,\"part_2_5\":%u,\"part_5\":%u,\"part_10\":%u"
                    "}",
                    (i > 0) ? "," : "",  // Add comma for all but first item
                    timestamp,
                    data.latitude / 10000000.0,   // Convert back to float
                    data.longitude / 10000000.0,  // Convert back to float
                    data.co2,
                    data.hum,
                    data.temp,
                    data.pm2_5,
                    data.pm5,
                    data.pm10);
        
        current_pos += record_len;
        remaining -= record_len;
        
        // Ensure we don't overflow the buffer
        if (remaining <= 20) {  // Leave some room for closing the JSON
            break;
        }
    }
    
    // Close the JSON array and object
    snprintf(current_pos, remaining, "]}");
}

int main() {
    stdio_init_all();

    printf("Program starting...\n");
    
    // Set system time to a reasonable default (Jan 1, 2024)
    // Since the Pi Pico has no RTC, we need to set a default time
    // This will be used until we get actual time from GPS
    time_t default_time = 1706745600; // 2024-01-31 00:00:00 UTC
    set_system_time(default_time);
    
#ifdef USE_WATCHDOG
    // Initialize and start the watchdog timer (20 second timeout)
    watchdog_enable(20000, 1);
    printf("Watchdog enabled with 20 second timeout\n");
#else
    printf("Watchdog disabled\n");
#endif

    printf("Initializing I2C...\n");
    i2c_init();  // Initialize I2C before checking sensors
    printf("I2C initialized, checking sensors...\n");
    checkSensors();
    printf("Sensors checked, initializing ADC...\n");
    batteryADC.init();
    
    // Add delay to ensure all sensors are stable
    printf("Waiting for sensors to stabilize...\n");
    sleep_ms(2000);
    printf("Continuing initialization\n");
    
    printf("Initializing eInk display...\n");
    eInk_init();
    displayHello();
    
    printf("Initializing buttons...\n");
    initButtons();
    
    printf("Initializing flash storage...\n");
    flash_initialized = flash_storage.init();
    if (flash_initialized) {
        printf("Flash storage initialized successfully\n");
    } else {
        printf("Flash storage initialization failed\n");
    }
    
    printf("Initializing GPS module...\n");
    myGPS gps(uart0, 9600, 0, 1);
    printf("GPS module initialized\n");

    // Enable time messages explicitly
    printf("Requesting GPS module to enable time messages...\n");
    gps.enableTimeMessages();

    // First try a cold start to reset the module and clear all navigation data
    printf("Performing GPS cold start to reset the module...\n");
    bool cold_start_success = gps.sendColdStartCommand();
    if (cold_start_success) {
        printf("GPS cold start completed successfully\n");
    } else {
        printf("GPS cold start may not have been recognized by the module\n");
        // Try a hot start as fallback
        printf("Attempting hot start instead...\n");
        gps.sendHotStartCommand();
    }

    // Set GPS start time after initialization
    absolute_time_t gps_start_time = get_absolute_time();
    printf("Starting GPS acquisition...\n");
    
    // Initialize variables for main loop
    bool has_fix = false;
    int fix_status = 2; // Start with invalid fix
    int satellites_visible = 0;
    
    last_refresh_time = get_absolute_time();
    printf("Main loop starting now\n");

    // Initialize timing variables for sensor data saving
    uint32_t current_time_ms = 0;
    uint32_t last_save_time_ms = 0;
    uint32_t save_interval_ms = 60000; // Save data every minute
    
    // GPS error recovery variables
    int consecutive_gps_errors = 0;
    uint32_t last_gps_recovery_time_ms = 0;
    uint32_t gps_recovery_interval_ms = 60000; // Try recovery every minute if needed
    
    // Main loop
    while (true) {
        // Check for and handle WiFi status messages
        wifi.poll();

        // Update current time
        current_time_ms = to_ms_since_boot(get_absolute_time());

        // Handle button presses IMMEDIATELY at the beginning of each loop
        if (tast_pressed[0] == SHORT_PRESSED) {
            nextPage();
            tast_pressed[0] = NOT_PRESSED;
        } else if (tast_pressed[1] == SHORT_PRESSED) {
            refreshDisplay_Settings_Button(current_page);
            tast_pressed[1] = NOT_PRESSED;
        } else if (tast_pressed[1] == LONG_PRESSED) {
            tast_pressed[1] = NOT_PRESSED; // Reset button state
            
            printf("\n\n-------- UPLOAD PROCESS STARTED --------\n");
            printf("Long press detected on Button 1 (GPIO 19) - Initiating data upload...\n");
            
            displayUploadStatus("Initializing...");
            
            // Check if there is data to upload
            std::vector<SensorData> data_vec = flash_storage.loadSensorData();
            printf("[UPLOAD] Found %d records to upload\n", data_vec.size());
            displayUploadStatus("Reading data...");
            
            if (data_vec.empty()) {
                printf("[UPLOAD] No data to upload, aborting\n");
                displayUploadStatus("No data found");
                sleep_ms(3000);
                refresh_display = true;
                continue;
            }
            
            // Check if WiFi is already connected
            wifi.poll();
            if (wifi.getConnected() != CYW43_LINK_UP) {
                printf("[UPLOAD] WiFi not connected, attempting to connect...\n");
                displayUploadStatus("Connecting WiFi...");
                
                bool connected = false;
                const int MAX_WIFI_ATTEMPTS = 3;
                
                // Try multiple times to connect to WiFi
                for (int attempt = 1; attempt <= MAX_WIFI_ATTEMPTS; attempt++) {
                    printf("[UPLOAD] WiFi connection attempt %d/%d\n", attempt, MAX_WIFI_ATTEMPTS);
                    
                    displayUploadStatus("WiFi connecting...");
                    
                    // Before attempting scan and connect, ensure WiFi is in a clean state
                    if (attempt > 1) {
                        printf("[UPLOAD] Performing emergency WiFi reset before retry\n");
                        wifi.emergencyReset();
                    }
                    
                    // Use scan and connect to find and try any available networks
                    int connect_result = wifi.scanAndConnect();
                    
                    // Additional polling to ensure WiFi operations can complete
                    for (int i = 0; i < 20; i++) {
                        cyw43_arch_poll();
                        wifi.poll();
                        
                        // Check if connected after each poll
                        if (wifi.getConnected() == CYW43_LINK_UP) {
                            connected = true;
                            printf("[UPLOAD] Connection became active after polling\n");
                            break;
                        }
                        sleep_ms(100);
                    }
                    
                    // Check if we're connected now
                    if (wifi.getConnected() == CYW43_LINK_UP) {
                        connected = true;
                        printf("[UPLOAD] Successfully connected to WiFi on attempt %d\n", attempt);
                        break;
                    } else if (connect_result == 0) {
                        // scanAndConnect reported success but link is still down
                        printf("[UPLOAD] scanAndConnect reported success but link is still down, waiting...\n");
                        
                        // Give extra time to establish connection
                        for (int i = 0; i < 30; i++) {  // Increased wait time
                            cyw43_arch_poll();
                            wifi.poll();
                            
                            // Check if connected after polling
                            if (wifi.getConnected() == CYW43_LINK_UP) {
                                connected = true;
                                printf("[UPLOAD] Connection became active after additional polling\n");
                                break;
                            }
                            sleep_ms(200);  // Longer sleep between polls
                            
                            // Update status every few seconds
                            if (i % 10 == 0) {
                                char status[64];
                                sprintf(status, "Connecting...%ds", (i/5)+1);
                                displayUploadStatus(status);
                            }
                        }
                        
                        if (connected) break;
                    }
                    
                    // Connection failed, try again if not last attempt
                    printf("[UPLOAD] WiFi connection attempt %d failed\n", attempt);
                    
                    if (attempt < MAX_WIFI_ATTEMPTS) {
                        printf("[UPLOAD] Waiting before next attempt...\n");
                        char retry_msg[64];
                        sprintf(retry_msg, "Retry in %ds...", 3);
                        displayUploadStatus(retry_msg);
                        sleep_ms(3000);
                    }
                }
                
                // If we still couldn't connect after all attempts
                if (!connected) {
                    printf("[UPLOAD] Failed to connect to WiFi after %d attempts\n", MAX_WIFI_ATTEMPTS);
                    displayUploadStatus("WiFi failed");
                    sleep_ms(3000);
                    refresh_display = true;
                    continue;  // Skip the rest of the loop and try again
                }
            } else {
                printf("[UPLOAD] WiFi already connected\n");
            }
            
            // If we get here, WiFi is connected
            displayUploadStatus("WiFi connected");
            sleep_ms(1000);
            
            // Rest of the upload process continues here...
            displayUploadStatus("Uploading data...");
            printf("[UPLOAD] Loaded %d records for upload\n", data_vec.size());
            
            // Track successful and failed uploads
            int successful_uploads = 0;
            int failed_uploads = 0;
            int consecutive_failures = 0;
            const int MAX_CONSECUTIVE_FAILURES = 3;
            
            // Prepare the entire batch of data for transmission
            char json_buffer[4096];  // Increased buffer size for batch data
            prepareBatchDataForTransmission(data_vec, json_buffer, sizeof(json_buffer), gps);
            
            // Debug output for the payload
            printf("[UPLOAD] Uploading batch of %d records\n", (int)data_vec.size());
            printf("[UPLOAD] JSON Payload (full):\n%s\n", json_buffer);
            
            // Create HTTP request with the batch data
            char request[4096 + 256];  // Add extra space for headers
            sprintf(request, 
                    "POST /api/addMarkers HTTP/1.1\r\n"
                    "Host: www.gm4s.eu\r\n"
                    "Content-Type: application/json\r\n"
                    "Content-Length: %d\r\n"
                    "Connection: close\r\n"
                    "\r\n"
                    "%s", 
                    strlen(json_buffer), json_buffer);
            
            displayUploadStatus("Sending batch...");
            printf("[UPLOAD] Sending batch HTTP request to www.gm4s.eu...\n");
            
            // Check if WiFi is still connected
            wifi.poll();
            if (wifi.getConnected() != CYW43_LINK_UP) {
                printf("[UPLOAD] WiFi connection lost, attempting to reconnect...\n");
                displayUploadStatus("WiFi reconnecting");
                
                // Try to reconnect to WiFi
                bool reconnected = false;
                for (int retry = 0; retry < 2; retry++) {
                    if (retry > 0) {
                        wifi.emergencyReset();
                    }
                    
                    if (wifi.scanAndConnect() == 0) {
                        // Wait for connection to be established
                        for (int wait = 0; wait < 20; wait++) {
                            cyw43_arch_poll();
                            wifi.poll();
                            if (wifi.getConnected() == CYW43_LINK_UP) {
                                reconnected = true;
                                break;
                            }
                            sleep_ms(100);
                        }
                        
                        if (reconnected) {
                            printf("[UPLOAD] Successfully reconnected to WiFi\n");
                            displayUploadStatus("WiFi reconnected");
                            sleep_ms(1000);
                            break;
                        }
                    }
                }
                
                if (!reconnected) {
                    printf("[UPLOAD] Failed to reconnect to WiFi, aborting upload\n");
                    displayUploadStatus("WiFi lost");
                    sleep_ms(3000);
                    refresh_display = true;
                    continue;
                }
            }
            
            // Upload using TLS with retry
            bool result = false;
            int upload_attempts = 0;
            const int MAX_UPLOAD_ATTEMPTS = 3;
            
            while (!result && upload_attempts < MAX_UPLOAD_ATTEMPTS) {
                upload_attempts++;
                
                if (upload_attempts > 1) {
                    char retry_msg[64];
                    sprintf(retry_msg, "Retry batch %d/%d", upload_attempts, MAX_UPLOAD_ATTEMPTS);
                    displayUploadStatus(retry_msg);
                    sleep_ms(1000);  // Wait before retry
                }
                
                result = run_tls_client_test(NULL, 0, "www.gm4s.eu", request, 20000);  // Increased timeout for batch upload
                
                if (!result && upload_attempts < MAX_UPLOAD_ATTEMPTS) {
                    printf("[UPLOAD] Batch upload attempt %d failed, retrying...\n", upload_attempts);
                    
                    // Check WiFi connection before retry
                    wifi.poll();
                    if (wifi.getConnected() != CYW43_LINK_UP) {
                        printf("[UPLOAD] WiFi connection lost during upload attempt, trying to reconnect...\n");
                        wifi.emergencyReset();
                        if (wifi.scanAndConnect() != 0) {
                            printf("[UPLOAD] Failed to reconnect WiFi, aborting upload\n");
                            break;
                        }
                    }
                    
                    sleep_ms(2000);  // Longer delay before retry
                }
            }
            
            if (result) {
                printf("[UPLOAD] Batch upload successful\n");
                successful_uploads = data_vec.size();
                failed_uploads = 0;
                
                printf("[UPLOAD] All uploads successful, clearing flash storage\n");
                displayUploadStatus("Clearing data...");
                flash_storage.eraseStorage();
                printf("[UPLOAD] Flash storage cleared\n");
                
                displayUploadStatus("Upload complete");
                sleep_ms(3000);
            } else {
                printf("[UPLOAD] Batch upload failed after %d attempts\n", MAX_UPLOAD_ATTEMPTS);
                failed_uploads = data_vec.size();
                successful_uploads = 0;
                
                // Show a summary of the upload
                char summary[64];
                sprintf(summary, "Upload failed");
                displayUploadStatus(summary);
                sleep_ms(3000);
            }
            
            refresh_display = true; // Update the display
            printf("-------- UPLOAD PROCESS COMPLETED --------\n\n");
        }

        // Check if it's time to measure sensor data based on refreshInterval
        bool should_read_sensors = false;
        if (absolute_time_diff_us(last_refresh_time, get_absolute_time()) / 1000 >= refreshInterval) {
            should_read_sensors = true;
        }

        // Always try to get GPS data regardless of refresh interval
        // GPS often needs more frequent readings to get a good fix
        std::string gps_line;
        double latitude = 0, longitude = 0;
        char ns_indicator = ' ', ew_indicator = ' ';
        std::string time_str;
        int gps_status = gps.readLine(gps_line, longitude, ew_indicator, latitude, ns_indicator, time_str);
        
        // Update GPS status for display
        fix_status = gps_status;
        satellites_visible = gps.getVisibleSatellites();

        if (gps_status == 0) {
            if (ENABLE_GPS_DEBUG) {
                printf("Valid GPS fix obtained\n");
            }
            // Valid GPS data, update the sensor data object
            sensor_data_obj.latitude = static_cast<int32_t>(latitude * 10000000);
            sensor_data_obj.longitude = static_cast<int32_t>(longitude * 10000000);
            consecutive_gps_errors = 0;  // Reset error counter on success
            
            // If we have a valid time string from GPS, use it to update the system time
            if (!time_str.empty() && time_str != "00:00:00") {
                int hours = 0, minutes = 0, seconds = 0;
                if (sscanf(time_str.c_str(), "%d:%d:%d", &hours, &minutes, &seconds) == 3) {
                    // Get current system time to keep the date part
                    time_t now = time(NULL);
                    struct tm *timeinfo = gmtime(&now);
                    
                    // Update only the time part, keeping the date
                    timeinfo->tm_hour = hours;
                    timeinfo->tm_min = minutes;
                    timeinfo->tm_sec = seconds;
                    
                    // Convert back to time_t and update system time
                    time_t new_time = mktime(timeinfo);
                    set_system_time(new_time);
                    
                    if (ENABLE_GPS_DEBUG) {
                        printf("Updated system time with GPS time: %s\n", time_str.c_str());
                    }
                }
            }
        } else if (gps_status == 2) {
            if (ENABLE_GPS_DEBUG) {
                printf("Invalid GPS fix, coordinates not reliable\n");
            }
            // Invalid fix - set coordinates to 0
            sensor_data_obj.latitude = 0;
            sensor_data_obj.longitude = 0;
            consecutive_gps_errors++;
        } else {
            if (ENABLE_GPS_DEBUG) {
                printf("GPS read error, using default values\n");
            }
            // Error reading GPS - set coordinates to 0
            sensor_data_obj.latitude = 0;
            sensor_data_obj.longitude = 0;
            consecutive_gps_errors++;
        }

        // Try GPS recovery if we've had multiple consecutive errors
        if (consecutive_gps_errors > 10 && 
            (current_time_ms - last_gps_recovery_time_ms >= gps_recovery_interval_ms)) {
            printf("Multiple GPS errors detected, attempting recovery...\n");
            gps.sendHotStartCommand();
            last_gps_recovery_time_ms = current_time_ms;
        }
        
        // Only read other sensor data at refresh interval timing
        if (should_read_sensors) {
            // Read other sensor data
            if (ENABLE_GPS_DEBUG) {
                printf("Reading sensor data...\n");
            }
            
            float temperature, humidity, pressure, gas_resistance;
            if (bme688_sensor.readData(temperature, humidity, pressure, gas_resistance)) {
                sensor_data_obj.temp = temperature;
                sensor_data_obj.hum = humidity;
            }
            
            pas_co2_sensor.read();
            sensor_data_obj.co2 = pas_co2_sensor.getResult();
            
            // Read particle matter sensor
            uint16_t pm1_0, pm2_5, pm10;
            if (hm3301_sensor.read(pm1_0, pm2_5, pm10)) {
                sensor_data_obj.pm2_5 = pm2_5;
                sensor_data_obj.pm5 = pm2_5; // Assuming pm5 = pm2_5 for now
                sensor_data_obj.pm10 = pm10;
            }
            
            // Get battery level
            batteryLevel = batteryADC.readVoltage();
            printf("Battery level: %.2f%%\n", batteryLevel);
            
            // We'll set the timestamp when saving to flash, no need to update it here
            // sensor_data_obj.timestamp = to_ms_since_boot(get_absolute_time());
        }
        
        // Check if we should save the data to flash
        // Now also sync this with the refresh interval
        if (should_read_sensors && (current_time_ms - last_save_time_ms >= save_interval_ms)) {
            if (!flash_storage.isStorageFull()) {
                // Check if we have valid GPS coordinates
                bool valid_gps = false;
                
                // Consider GPS coordinates valid if they're non-zero
                // This assumes latitude and longitude are stored in sensor_data_obj
                if (sensor_data_obj.latitude != 0 || sensor_data_obj.longitude != 0) {
                    valid_gps = true;
                }
                
                if (valid_gps) {
                    // We have valid GPS coordinates, proceed with saving
                    bool save_success = false;
                    try {
                        // Get the current time from GPS if available, otherwise use system time
                        time_t raw_time = time(NULL); // Use current time instead of data.timestamp
                        struct tm *timeinfo = gmtime(&raw_time);
                        
                        // Get the GPS time string
                        std::string gps_line_save;
                        double lon_save = 0, lat_save = 0;
                        char ns_save = ' ', ew_save = ' ';
                        std::string time_str_save;
                        
                        char timestamp[64];
                        
                        // Try to get time from GPS for current time reference
                        gps.readLine(gps_line_save, lon_save, ew_save, lat_save, ns_save, time_str_save);
                        
                        if (!time_str_save.empty() && time_str_save != "00:00:00") {
                            // We have a valid GPS time - parse it
                            int hours = 0, minutes = 0, seconds = 0;
                            if (sscanf(time_str_save.c_str(), "%d:%d:%d", &hours, &minutes, &seconds) == 3) {
                                // Simply use the system date with GPS time
                                // This maintains day consistency while using accurate GPS time
                                timeinfo->tm_hour = hours;
                                timeinfo->tm_min = minutes;
                                timeinfo->tm_sec = seconds;
                                
                                // Convert back to unix timestamp - we're already using gmtime so this is in UTC
                                raw_time = mktime(timeinfo);
                                printf("Using GPS time for timestamp: %s\n", time_str_save.c_str());
                            }
                        } else {
                            printf("No valid GPS time found, using system time\n");
                        }
                        
                        // Update timestamp for this reading (in seconds)
                        sensor_data_obj.timestamp = (uint32_t)raw_time;
                        save_success = flash_storage.saveSensorData(sensor_data_obj);
                        
                        if (save_success) {
                            printf("Saved data to flash. Total records: %lu\n", 
                                  flash_storage.getStoredCount());
                            last_save_time_ms = current_time_ms;
                        }
                    } catch (const std::exception& e) {
                        printf("Flash write error: %s\n", e.what());
                    }
                } else {
                    // No valid GPS coordinates, display a message to the user
                    printf("Not saving to flash: No valid GPS coordinates\n");
                    
                    // Display a message on the screen
                    resetImageBuffer();
                    Paint_Clear(WHITE);
                    
                    // Draw a message telling the user to go outside for better GPS signal
                    Paint_DrawString_EN(5, 10, "GPS FIX REQUIRED", &Font16, BLACK, WHITE);
                    Paint_DrawString_EN(5, 40, "Please go outside", &Font12, BLACK, WHITE);
                    Paint_DrawString_EN(5, 60, "for better GPS signal", &Font12, BLACK, WHITE);
                    
                    // Show satellite count
                    char sat_buf[32];
                    sprintf(sat_buf, "Satellites: %d", satellites_visible);
                    Paint_DrawString_EN(5, 90, sat_buf, &Font12, BLACK, WHITE);
                    
                    // Show battery level
                    drawBatteryIcon(150, 10);
                    
                    // Update the display
                    EPD_1IN54_V2_Display(ImageBuffer);
                    
                    // Wait a moment before returning to normal display
                    sleep_ms(3000);
                    refresh_display = true;
                    
                    // Still update the last save time to prevent constant messages
                    last_save_time_ms = current_time_ms;
                }
            }
        }
        
        // Update display if needed
        if (refresh_display || (should_read_sensors && absolute_time_diff_us(last_refresh_time, get_absolute_time()) / 1000 >= refreshInterval)) {
            printf("Refreshing display with current data (page %d)\n", current_page);
            displayPage(current_page, gps_start_time, fix_status, satellites_visible);
            last_refresh_time = get_absolute_time();
            refresh_display = false;
        }
        
        // Very short sleep to check for button presses without losing responsiveness
        // Decreased from 1000ms to 100ms to make button presses more responsive
        sleep_ms(100);
        
#ifdef USE_WATCHDOG
        // Reset the watchdog timer at the end of each loop iteration
        watchdog_update();
#endif
    }

    free(ImageBuffer);
    return 0;
}
