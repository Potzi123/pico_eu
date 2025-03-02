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
#include "hardware/clocks.h"
#include "hardware/rtc.h"
#include "hardware/structs/scb.h"

// Comment this line to disable the watchdog timer
// #define USE_WATCHDOG

// Set to 1 to enable fake GPS data (for indoor testing)
#define USE_FAKE_GPS 1

// Set to 1 to disable flash operations (for memory/power debugging)
// #define DISABLE_FLASH 0

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

#define SHORT_PRESSED_TIME 250  // Reduced from 500ms to 250ms for quicker response
#define LONG_PRESSED_TIME 1000  // Reduced from 2000ms to 1000ms for quicker response

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

// Add bike mode constant to make it clear this is a bike-specific configuration
#define BIKE_MODE 1

bool fist_time = true;

int refreshInterval = 5000;  // in milliseconds
// Update interval options to match user's preferences for bike usage
int refreshIntervals[] = {5000, 10000, 15000, 30000, 60000};  // 5s, 10s, 15s, 30s, 1min
int currentIntervalIndex = 0;  // Default to 5 seconds for bike usage

// Add a variable for data collection interval that follows the refresh interval
int dataCollectionInterval = refreshInterval;  
// Change the data collection multiplier to 1 so data collection matches display refresh
// This is ideal for bike usage to capture frequent environmental changes
const int DATA_COLLECTION_MULTIPLIER = 1;  // Collect data at same rate as display refresh

const uint8_t *flash_target_contents = (const uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);
UBYTE *ImageBuffer;
UWORD Imagesize = ((EPD_1IN54_V2_WIDTH % 8 == 0) ? (EPD_1IN54_V2_WIDTH / 8) : (EPD_1IN54_V2_WIDTH / 8 + 1)) * EPD_1IN54_V2_HEIGHT;

//GPIO
volatile uint64_t tast_lasttime[TASTER_COUNT] = {0, 0};  // Changed back to 2 elements
volatile int tast_pressed[TASTER_COUNT] = {NOT_PRESSED, NOT_PRESSED};  // Changed back to 2 elements
volatile uint8_t tast[TASTER_COUNT] = {BUTTON_NEXT_PAGE, BUTTON_REFRESH_DISPLAY};  // Changed back to 2 elements

// Add this with other globals
volatile bool button_state_changed = false;  // Flag to indicate a button state has changed
volatile bool fast_refresh_enabled = false;  // Changed to false by default
bool base_image_set = false;  // Tracks if we've set the base image for partial refresh
int refresh_counter = 0;     // Counter to track when to do a full refresh

myWIFI wifi;
myADC batteryADC(ADC, 10);
HM3301 hm3301_sensor(I2C_PORT, HM3301_ADDRESS, I2C_SDA, I2C_SCL);
BME688 bme688_sensor(I2C_PORT, BME688_ADDRESS, I2C_SDA, I2C_SCL);
Pas_co2 pas_co2_sensor(PAS_CO2_ADDRESS, I2C_PORT);

// Variables for page navigation and timing
volatile int current_page = 0;
volatile bool refresh_display = false;
absolute_time_t last_refresh_time;

// GPS variables
absolute_time_t gps_start_time;  // GPS start time for displaying time since first fix
int fix_status = 2;              // GPS fix status (0=valid, 2=invalid)
int satellites_visible = 0;      // Number of satellites currently visible

// Variables for sensor data storage
SensorData sensor_data_obj = {0,0,0,0,0,0,0,0,0,0,0};
std::vector<SensorData> sensor_data;

// In-memory buffer for high-frequency data collection
std::vector<SensorData> data_buffer;  // Stores readings between flash writes
bool buffer_modified = false;         // Track if buffer has unwritten changes

float batteryLevel = 0;

// Modify the external function declaration to match the expected signature exactly
extern "C" bool run_tls_client_test(unsigned char const* cert, unsigned int cert_len, char const* server, char const* request, int timeout);

// After other variable declarations, add:
Flash flash_storage;  // Create flash storage object
absolute_time_t last_flash_write_time;  // For timing flash writes
bool flash_initialized = false;

bool setupComplete = false;
bool initialDataCollected = false;  // Track if first data has been collected
bool initialDataSaved = false;      // Track if first save has occurred
bool initializationComplete = false; // Overall initialization state

// Forward declaration
void displayUploadStatus(const char* msg);

// Forward declaration
void displayPage(int page, absolute_time_t gps_start_time, int fix_status, int satellites_visible, bool is_fake_gps);

// Forward declaration for forceDisplayRefresh
void forceDisplayRefresh();

// Simple version of displayPage that just clears the screen - used by displayYesNo
void displayPage(int page) {
    // Create a dummy placeholder for the missing parameters
    absolute_time_t dummy_time = get_absolute_time();
    
    // Call the full version with default values for the other parameters
    displayPage(page, dummy_time, 0, 0, false);
}

// Display a yes/no question with one option highlighted
void displayYesNo(const char* message, bool highlight_yes) {
    // Create a simple formatted text screen
    // First clear the screen
    displayPage(0);  // Assuming displayPage(0) clears the screen
    
    // Display the message
    displayUploadStatus(message);
    sleep_ms(500);
    
    // Display a simple yes/no choice
    char buffer[64];
    sprintf(buffer, "Yes: %s  No: %s", 
            highlight_yes ? "[Selected]" : "", 
            !highlight_yes ? "[Selected]" : "");
    displayUploadStatus(buffer);
}

// Simpler function to show a yes/no prompt using existing display functions
bool showYesNoPrompt(const char* title, const char* question) {
    // First show a status message with the title
    displayUploadStatus(title);
    sleep_ms(1000);
    
    // Then display the yes/no prompt
    displayYesNo(question, true);  // Default to highlighting 'Yes'
    
    // Wait for user input
    bool selection_made = false;
    bool result = true;  // Default to 'Yes'
    bool highlight_yes = true;
    
    // Add a timeout to avoid waiting forever
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    uint32_t timeout_ms = 20000;  // 20 seconds timeout
    
    while (!selection_made) {
        // Check if we've timed out
        if (to_ms_since_boot(get_absolute_time()) - start_time > timeout_ms) {
            printf("Selection timed out, defaulting to 'Yes'\n");
            result = true;
            selection_made = true;
            break;
        }
        
        // Poll for GPIO events and check button states
        cyw43_arch_poll();
        
        // Process button presses
        if (tast_pressed[0] == SHORT_PRESSED) {
            // Button 1 toggles the highlight
            tast_pressed[0] = NOT_PRESSED;
            highlight_yes = !highlight_yes;
            displayYesNo(question, highlight_yes);
            printf("Toggled selection to: %s\n", highlight_yes ? "Yes" : "No");
        } else if (tast_pressed[1] == SHORT_PRESSED) {
            // Button 2 confirms the selection
            tast_pressed[1] = NOT_PRESSED;
            result = highlight_yes;
            selection_made = true;
            printf("Confirmed selection: %s\n", result ? "Yes" : "No");
        }
        
        // Small delay to avoid busy waiting
        sleep_ms(50);
    }
    
    // Show confirmation
    displayUploadStatus(result ? "Yes selected" : "No selected");
    sleep_ms(1000);
    
    return result;
}

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
void displayGPSStatus(absolute_time_t gps_start_time, int fix_status, int satellites_visible, bool is_fake_gps) {
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
    
    // Add Bike Mode indicator in the top left
    Paint_DrawString_EN(10, 5, "Bike Mode", &Font12, BLACK, WHITE);
    
    // GPS Status Page Title
    Paint_DrawString_EN(10, 25, "GPS Status", &Font20, BLACK, WHITE);
    
    // Show if using fake GPS data
    if (is_fake_gps) {
        Paint_DrawString_EN(10, 50, "Mode: SIMULATED", &Font16, BLACK, WHITE);
    } else {
        // Show time since GPS start
        uint32_t seconds_since_start = to_ms_since_boot(get_absolute_time()) / 1000 - to_ms_since_boot(gps_start_time) / 1000;
        sprintf(buffer, "Time: %d min %d sec", seconds_since_start / 60, seconds_since_start % 60);
        Paint_DrawString_EN(10, 50, buffer, &Font16, BLACK, WHITE);
    }
    
    // Show fix status
    if (fix_status == 0) {
        Paint_DrawString_EN(10, 70, "Fix: VALID", &Font16, BLACK, WHITE);
    } else {
        Paint_DrawString_EN(10, 70, "Fix: waiting...", &Font16, BLACK, WHITE);
    }
    
    // Show satellites visible
    sprintf(buffer, "Satellites: %d", satellites_visible);
    Paint_DrawString_EN(10, 90, buffer, &Font16, BLACK, WHITE);
    
    // Instructions - adjust depending on GPS mode
    if (is_fake_gps) {
        Paint_DrawString_EN(10, 110, "Indoor Testing Mode", &Font12, BLACK, WHITE);
        Paint_DrawString_EN(10, 125, "Using simulated GPS", &Font12, BLACK, WHITE);
        Paint_DrawString_EN(10, 140, "coordinates for", &Font12, BLACK, WHITE);
        Paint_DrawString_EN(10, 155, "development testing", &Font12, BLACK, WHITE);
    } else {
        Paint_DrawString_EN(10, 110, "For best results:", &Font12, BLACK, WHITE);
        Paint_DrawString_EN(10, 125, "- Go outdoors", &Font12, BLACK, WHITE);
        Paint_DrawString_EN(10, 140, "- Clear sky view", &Font12, BLACK, WHITE);
        Paint_DrawString_EN(10, 155, "- Wait 5+ minutes", &Font12, BLACK, WHITE);
    }
    
    // If fast refresh is enabled, use partial refresh with periodic full refresh
    if (fast_refresh_enabled) {
        refresh_counter++;
        
        // Every 10 refreshes, do a full refresh to clear any artifacts
        if (refresh_counter >= 10 || !base_image_set) {
            printf("Performing full refresh to clear artifacts (counter=%d)\n", refresh_counter);
            EPD_1IN54_V2_Display(ImageBuffer);
            base_image_set = false;
            refresh_counter = 0;
            sleep_ms(100);  // Give a little more time for full refresh
        } else if (!base_image_set) {
            // First set the base image
            printf("Setting base image for partial refresh\n");
            EPD_1IN54_V2_DisplayPartBaseImage(ImageBuffer);
            base_image_set = true;
        } else {
            // Then use partial refresh for updates
            printf("Using partial refresh (update %d/10)\n", refresh_counter);
            EPD_1IN54_V2_DisplayPart(ImageBuffer);
        }
    } else {
        // Use standard full refresh always
        EPD_1IN54_V2_Display(ImageBuffer);
    }
}

// Modify the displayPage function to include periodic full refresh
void displayPage(int page, absolute_time_t gps_start_time, int fix_status, int satellites_visible, bool is_fake_gps) {
    // GPS Status Page is a special case, handle separately
    if (page == 4) {  
        displayGPSStatus(gps_start_time, fix_status, satellites_visible, is_fake_gps);
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

    // Add Bike Mode indicator in the top left of each page
    Paint_DrawString_EN(10, 5, "Bike Mode", &Font12, BLACK, WHITE);

    if (page == 0) {
        // Page 1: BME688 Sensor Data
        Paint_DrawString_EN(10, 25, "BME688", &Font20, BLACK, WHITE);

        // Temp
        sprintf(buffer, "Temp: %.2f C", sensor_data_obj.temp);
        Paint_DrawString_EN(10, 50, buffer, &Font20, BLACK, WHITE);

        // Hum
        sprintf(buffer, "Hum: %.2f %%", sensor_data_obj.hum);
        Paint_DrawString_EN(10, 75, buffer, &Font20, BLACK, WHITE);

        printf("Displayed Page 1: BME688 Data.\n");

    } else if (page == 1) {
        // Page 2: HM3301 Sensor Data
        Paint_DrawString_EN(10, 25, "HM3301", &Font20, BLACK, WHITE);

        // Units header
        Paint_DrawString_EN(10, 50, "Units: ug/m3", &Font20, BLACK, WHITE);

        // PM1.0
        sprintf(buffer, "PM1.0: %u", sensor_data_obj.pm2_5);
        Paint_DrawString_EN(10, 75, buffer, &Font20, BLACK, WHITE);

        // PM2.5
        sprintf(buffer, "PM2.5: %u", sensor_data_obj.pm5);
        Paint_DrawString_EN(10, 100, buffer, &Font20, BLACK, WHITE);

        // PM10
        sprintf(buffer, "PM10: %u", sensor_data_obj.pm10);
        Paint_DrawString_EN(10, 125, buffer, &Font20, BLACK, WHITE);

        printf("Displayed Page 2: HM3301 Data.\n");

    } else if (page == 2) {
        // Page 3: PAS CO2 Sensor Data
        Paint_DrawString_EN(10, 25, "PAS CO2", &Font20, BLACK, WHITE);

        // CO2
        sprintf(buffer, "CO2: %u", sensor_data_obj.co2);
        Paint_DrawString_EN(10, 50, buffer, &Font20, BLACK, WHITE);

        // Unit for CO2
        Paint_DrawString_EN(140, 50, "ppm", &Font20, BLACK, WHITE);

        printf("Displayed Page 3: PAS CO2 Data.\n");

    } else if (page == 3) {
        // Page 4: Settings Page
        Paint_DrawString_EN(10, 25, "Settings", &Font20, BLACK, WHITE);
        
        // Display current refresh interval
        Paint_DrawString_EN(10, 50, "Display refresh:", &Font16, BLACK, WHITE);
        sprintf(buffer, "%d sec", refreshInterval / 1000);
        Paint_DrawString_EN(10, 70, buffer, &Font16, BLACK, WHITE);
        
        // Display current data collection interval
        Paint_DrawString_EN(10, 95, "Data collect:", &Font16, BLACK, WHITE);
        sprintf(buffer, "%d sec", dataCollectionInterval / 1000);
        Paint_DrawString_EN(10, 115, buffer, &Font16, BLACK, WHITE);
        
        // Add power management option
        Paint_DrawString_EN(10, 135, "Long press down:", &Font12, BLACK, WHITE);
        Paint_DrawString_EN(10, 150, "Sleep mode", &Font12, BLACK, WHITE);
        
        // Add fast refresh mode status
        Paint_DrawString_EN(10, 170, "Fast refresh:", &Font12, BLACK, WHITE);
        Paint_DrawString_EN(90, 170, fast_refresh_enabled ? "ON" : "OFF", &Font12, BLACK, WHITE);

        printf("Displayed Page 4: Settings.\n");
    }

    // If fast refresh is enabled, use partial refresh with periodic full refresh
    if (fast_refresh_enabled) {
        refresh_counter++;
        
        // Every 10 refreshes, do a full refresh to clear any artifacts
        if (refresh_counter >= 10 || !base_image_set) {
            printf("Performing full refresh to clear artifacts (counter=%d)\n", refresh_counter);
            EPD_1IN54_V2_Display(ImageBuffer);
            base_image_set = false;
            refresh_counter = 0;
            sleep_ms(100);  // Give a little more time for full refresh
        } else if (!base_image_set) {
            // First set the base image
            printf("Setting base image for partial refresh\n");
            EPD_1IN54_V2_DisplayPartBaseImage(ImageBuffer);
            base_image_set = true;
        } else {
            // Then use partial refresh for updates
            printf("Using partial refresh (update %d/10)\n", refresh_counter);
            EPD_1IN54_V2_DisplayPart(ImageBuffer);
        }
    } else {
        // Use standard full refresh always
        EPD_1IN54_V2_Display(ImageBuffer);
    }
}

// Modify displayUploadStatus to ensure reliable display
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
    
    // For upload status, always use full refresh for reliability
    EPD_1IN54_V2_Display(ImageBuffer);
    
    // After important messages, reset the fast refresh state
    if (fast_refresh_enabled) {
        base_image_set = false;
        refresh_counter = 0;
    }
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
        // Button pressed down
        tast_lasttime[gpio_pin] = time_us_64();
        printf("Button %d pressed down\n", gpio_pin);  // Debug output
    } else if (events & GPIO_IRQ_EDGE_RISE) {
        // Button released
        uint64_t currentTime = time_us_64();
        // Reduced debounce time from 10000us to 5000us
        if ((currentTime - tast_lasttime[gpio_pin]) <= 5000) {
            // Too short to be a real press (debounce)
            tast_lasttime[gpio_pin] = currentTime;
            return;
        }
        
        int pressed_time = (int)((currentTime - tast_lasttime[gpio_pin]) / 1000);
        printf("Button %d released after %d ms\n", gpio_pin, pressed_time);  // Debug output
        
        if (pressed_time < SHORT_PRESSED_TIME) {
            tast_pressed[gpio_pin] = SHORT_PRESSED;
            button_state_changed = true;  // Set flag for immediate handling
        } else if (pressed_time < LONG_PRESSED_TIME) {
            tast_pressed[gpio_pin] = SHORT_PRESSED;  // Still treat as short press
            button_state_changed = true;  // Set flag for immediate handling
        } else {
            tast_pressed[gpio_pin] = LONG_PRESSED;  // Long press
            button_state_changed = true;  // Set flag for immediate handling
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

// Rewrite forceDisplayRefresh function with no parameters
void forceDisplayRefresh() {
    // Mark display for refresh
    refresh_display = true;
    
    // Force immediate refresh instead of waiting for the next cycle
    printf("Forcing immediate display refresh for page %d\n", current_page);
    
    // Call displayPage with global variables
    displayPage(current_page, gps_start_time, fix_status, satellites_visible, USE_FAKE_GPS);
    
    printf("Display refreshed for page %d\n", current_page);
}

// Update the nextPage function to use the updated forceDisplayRefresh
void nextPage() {
    current_page = (current_page + 1) % PAGE_COUNT;
    printf("Switching to page %d\n", current_page);

    // Call updated forceDisplayRefresh
    forceDisplayRefresh();
}

// Also update refreshDisplay_Settings_Button to use the new forceDisplayRefresh signature
void refreshDisplay_Settings_Button(int page) {
    if (current_page == 3) {  // If on the settings page
        // Cycle through the predefined refresh intervals
        currentIntervalIndex = (currentIntervalIndex + 1) % (sizeof(refreshIntervals) / sizeof(refreshIntervals[0]));
        refreshInterval = refreshIntervals[currentIntervalIndex];
        
        // Update the data collection interval based on the new refresh interval
        dataCollectionInterval = refreshInterval * DATA_COLLECTION_MULTIPLIER;
        
        printf("Updated refresh interval to %d ms, data collection interval to %d ms (bike mode).\n", 
              refreshInterval, dataCollectionInterval);
        
        // Also toggle fast refresh mode on double press
        static uint32_t last_press_time = 0;
        uint32_t now = to_ms_since_boot(get_absolute_time());
        
        if (now - last_press_time < 500) {  // Double press within 500ms
            // Toggle fast refresh mode
            fast_refresh_enabled = !fast_refresh_enabled;
            printf("Fast refresh mode %s\n", fast_refresh_enabled ? "enabled" : "disabled");
            
            if (fast_refresh_enabled) {
                // Need to reset the base image flag when turning on fast refresh
                base_image_set = false;
            }
        }
        
        last_press_time = now;
        
        // Call parameter-less forceDisplayRefresh
        forceDisplayRefresh();
    } else {
        // When not on settings page, just force a refresh without changing settings
        forceDisplayRefresh();
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
    
    // Get the current time once for all records to prevent repeated GPS queries
    time_t current_time = time(NULL);
    struct tm *timeinfo = gmtime(&current_time);
    
    // Format a single timestamp to use if GPS time isn't available
    char default_timestamp[64];
    snprintf(default_timestamp, sizeof(default_timestamp), 
             "%04d-%02d-%02d %02d:%02d:%02d+00:00",
             timeinfo->tm_year + 1900, 
             timeinfo->tm_mon + 1, 
             timeinfo->tm_mday,
             timeinfo->tm_hour, 
             timeinfo->tm_min, 
             timeinfo->tm_sec);
    
    printf("[UPLOAD] Using default timestamp if needed: %s\n", default_timestamp);
    
    // Get GPS position and date information once before processing records
    std::string gps_line_save;
    double lon_save = 0, lat_save = 0;
    char ns_save = ' ', ew_save = ' ';
    std::string time_str_save;
    std::string date_str_save;
    
    // Try to get time from GPS (only once)
    bool valid_gps_data = (gps.readLine(gps_line_save, lon_save, ew_save, lat_save, ns_save, time_str_save, date_str_save) == 0);
    
    if (valid_gps_data) {
        printf("[UPLOAD] Using valid GPS data: Lat=%f%c, Long=%f%c, Time=%s, Date=%s\n", 
               lat_save, ns_save, lon_save, ew_save, time_str_save.c_str(), date_str_save.c_str());
    } else {
        printf("[UPLOAD] No valid GPS data available, using default values\n");
    }
    
    // Add each measurement as an object in the array
    int success_count = 0;
    for (size_t i = 0; i < data_vec.size(); i++) {
        const SensorData& data = data_vec[i];
        char timestamp[64];
        
        // Use the record's original timestamp to create a formatted date/time string
        time_t record_time = data.timestamp;
        struct tm *record_timeinfo = gmtime(&record_time);
        
        // Format with the record's timestamp
        snprintf(timestamp, sizeof(timestamp), 
                 "%04d-%02d-%02d %02d:%02d:%02d+00:00",
                 record_timeinfo->tm_year + 1900, 
                 record_timeinfo->tm_mon + 1, 
                 record_timeinfo->tm_mday,
                 record_timeinfo->tm_hour, 
                 record_timeinfo->tm_min, 
                 record_timeinfo->tm_sec);
        
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
        
        // Only increment position if we successfully wrote the record
        if (record_len > 0 && record_len < remaining) {
            current_pos += record_len;
            remaining -= record_len;
            success_count++;
            
            // Log periodically to show progress
            if (success_count % 10 == 0) {
                printf("[UPLOAD] Processed %d/%d records\n", success_count, (int)data_vec.size());
            }
        } else {
            printf("[UPLOAD] Warning: Could not add record %d (buffer limit reached)\n", (int)i);
            break;
        }
        
        // Ensure we don't overflow the buffer
        if (remaining <= 20) {  // Leave some room for closing the JSON
            printf("[UPLOAD] Buffer limit reached after %d/%d records\n", success_count, (int)data_vec.size());
            break;
        }
    }
    
    // Close the JSON array and object
    snprintf(current_pos, remaining, "]}");
    
    printf("[UPLOAD] Successfully prepared %d/%d records for transmission\n", success_count, (int)data_vec.size());
}

// Save the data buffer before sleeping
void saveBufferBeforeSleep() {
    if (buffer_modified && !data_buffer.empty()) {
        printf("Saving buffer data before sleep (%d entries)\n", data_buffer.size());
        
        // Save each buffered entry to flash
        for (const auto& buffered_data : data_buffer) {
            if (!flash_storage.saveSensorData(buffered_data)) {
                printf("ERROR: Failed to save data before sleep\n");
                break;
            }
        }
        
        printf("Buffer saved. Total records: %lu\n", flash_storage.getStoredCount());
        data_buffer.clear();
        buffer_modified = false;
    }
}

// Enter low power sleep mode
void enterSleepMode() {
    // First save any buffered data to flash
    saveBufferBeforeSleep();
    
    // Display sleep notification
    resetImageBuffer();
    Paint_Clear(WHITE);
    Paint_DrawString_EN(20, 50, "Sleeping...", &Font24, BLACK, WHITE);
    Paint_DrawString_EN(10, 90, "Press button to wake", &Font16, BLACK, WHITE);
    EPD_1IN54_V2_Display(ImageBuffer);
    
    // Disable all peripherals that consume power
    printf("Entering sleep mode...\n");
    sleep_ms(500); // Ensure the message is printed
    
    // Configure wake button with pull-up
    gpio_set_dir(BUTTON_REFRESH_DISPLAY, GPIO_IN);
    gpio_pull_up(BUTTON_REFRESH_DISPLAY);
    
    // Use a very simple sleep mode approach
    printf("Device sleeping. Press button to wake up...\n");
    
    // Enter a low-power wait loop until button is pressed
    bool waiting_for_wakeup = true;
    while (waiting_for_wakeup) {
        // Check if button is pressed (active low with pull-up)
        if (gpio_get(BUTTON_REFRESH_DISPLAY) == 0) {
            // Button pressed - wait for debounce
            sleep_ms(50);
            if (gpio_get(BUTTON_REFRESH_DISPLAY) == 0) {
                waiting_for_wakeup = false;
            }
        }
        
        // Sleep for a short period to reduce power consumption
        sleep_ms(100);
    }
    
    // Code execution will resume here on wake-up
    printf("Waking up from sleep mode...\n");
    
    // Restore button configuration
    gpio_set_irq_enabled_with_callback(BUTTON_REFRESH_DISPLAY, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    
    // Display wake-up confirmation
    resetImageBuffer();
    Paint_Clear(WHITE);
    Paint_DrawString_EN(20, 50, "Waking up...", &Font24, BLACK, WHITE);
    EPD_1IN54_V2_Display(ImageBuffer);
    sleep_ms(1000);
    
    // Force refresh
    refresh_display = true;
}

// Modify the setFastRefreshMode function for better reliability
void setFastRefreshMode(bool enable) {
    // If we're not changing the mode, return early
    if (fast_refresh_enabled == enable) {
        return;
    }

    printf("Fast refresh mode %s\n", enable ? "enabled" : "disabled");
    
    // Always do a full refresh when changing modes
    resetImageBuffer();
    
    // If enabling, tell the user about potential artifacts
    if (enable) {
        // Draw a warning message
        Paint_DrawString_EN(10, 5, "Fast Refresh ON", &Font16, BLACK, WHITE);
        Paint_DrawString_EN(10, 30, "May cause artifacts", &Font12, BLACK, WHITE);
        Paint_DrawString_EN(10, 50, "Full refresh every", &Font12, BLACK, WHITE);
        Paint_DrawString_EN(10, 70, "10 updates", &Font12, BLACK, WHITE);
    } else {
        // Draw standard mode message
        Paint_DrawString_EN(10, 5, "Standard Refresh", &Font16, BLACK, WHITE);
        Paint_DrawString_EN(10, 30, "For best quality", &Font12, BLACK, WHITE);
    }
    
    // Do a full refresh to ensure clean state
    EPD_1IN54_V2_Display(ImageBuffer);
    sleep_ms(300);  // Give time for the display to complete
    
    // Now set the mode
    fast_refresh_enabled = enable;
    
    // Reset base image state
    base_image_set = false;
    refresh_counter = 0;
    
    // Force a regular refresh to update the current screen
    refresh_display = true;
}

// Add these after variable declarations, before entering the main loop
volatile uint32_t btn1_events = 0;  // Add missing variable for button events
#define SAVE_INTERVAL_MS 180000  // Add missing constant for save interval (3 minutes instead of 60 seconds)

// Add this new function for uploading data with retry logic
bool uploadDataWithRetry(const char* json_data, int max_retries, int retry_delay_ms) {
    printf("Starting data upload with max %d retries, delay %d ms\n", max_retries, retry_delay_ms);
    displayUploadStatus("Connecting...");
    
    bool upload_successful = false;
    int retry_count = 0;
    
    // Check input parameters
    if (json_data == NULL || max_retries < 0 || retry_delay_ms < 0) {
        printf("ERROR: Invalid parameters for upload\n");
        displayUploadStatus("Upload error");
        return false;
    }
    
    // Print the full JSON data being uploaded for debugging
    printf("UPLOAD JSON DATA (FULL):\n%s\n", json_data);
    
    // Prepare the HTTP request with the JSON data
    char request_buffer[4096]; // Buffer for the HTTP request
    snprintf(request_buffer, sizeof(request_buffer), 
             "POST /api/addMarkers HTTP/1.1\r\n"
             "Host: " TLS_CLIENT_SERVER "\r\n"
             "Content-Type: application/json\r\n"
             "Content-Length: %d\r\n"
             "Connection: close\r\n"
             "\r\n"
             "%s",
             (int)strlen(json_data), json_data);
    
    printf("Prepared HTTP request with %d bytes of JSON data\n", (int)strlen(json_data));
    
    // Attempt upload with retries
    while (retry_count <= max_retries && !upload_successful) {
        if (retry_count > 0) {
            // Display retry status
            char retry_msg[64];
            snprintf(retry_msg, sizeof(retry_msg), "Retry %d of %d...", retry_count, max_retries);
            displayUploadStatus(retry_msg);
            
            // Wait before retry to allow network to recover
            printf("Waiting %d ms before retry %d...\n", retry_delay_ms, retry_count);
            sleep_ms(retry_delay_ms);
        }
        
        // Show status
        displayUploadStatus("Uploading data...");
        
        // Attempt the upload
        printf("Upload attempt %d of %d...\n", retry_count + 1, max_retries + 1);
        
        // Set a much longer timeout for large data uploads (3 minutes)
        int upload_timeout_ms = 180000;  // Increased from 60000 to 180000 (3 minutes)
        bool result = run_tls_client_test(NULL, 0, TLS_CLIENT_SERVER, request_buffer, upload_timeout_ms);
        
        if (result) {
            // Success
            upload_successful = true;
            printf("DATA UPLOAD SUCCESSFUL on attempt %d\n", retry_count + 1);
            displayUploadStatus("Upload success!");
        } else {
            // Failed, prepare for retry
            printf("DATA UPLOAD FAILED on attempt %d\n", retry_count + 1);
            
            if (retry_count < max_retries) {
                printf("Will retry in %d ms...\n", retry_delay_ms);
                // Increase retry delay for each attempt (exponential backoff)
                retry_delay_ms = retry_delay_ms * 1.5;
            } else {
                printf("MAX RETRIES REACHED. Upload failed.\n");
                displayUploadStatus("Upload failed");
            }
        }
        
        retry_count++;
    }
    
    return upload_successful;
}

// Add a function to handle uploading sensor data from flash storage
bool uploadSensorData(Flash& flash, myGPS& gps) {
    if (flash.getStoredCount() == 0) {
        printf("No data to upload\n");
        displayUploadStatus("No data to upload");
        return true; // Nothing to upload is considered success
    }
    
    printf("Preparing to upload %lu records from flash\n", flash.getStoredCount());
    displayUploadStatus("Preparing data...");
    
    // Load records from flash
    std::vector<SensorData> records = flash.loadSensorData();
    printf("Loaded %lu records from flash\n", records.size());
    
    if (records.empty()) {
        printf("No valid records found in flash\n");
        displayUploadStatus("No valid data");
        return false;
    }
    
    // Buffer for JSON data
    char json_buffer[16384]; // Larger buffer for many records
    
    // Prepare JSON batch for transmission
    prepareBatchDataForTransmission(records, json_buffer, sizeof(json_buffer), gps);
    
    // Attempt upload with 3 retries, 2 second initial delay
    bool result = uploadDataWithRetry(json_buffer, 3, 2000);
    
    // If upload was successful, clear the flash storage
    if (result) {
        printf("Upload successful, data preserved for future use\n");
        displayUploadStatus("Upload successful");
        sleep_ms(2000);
        displayUploadStatus("Data preserved");
        sleep_ms(1000);
    }
    
    return result;
}

// Add this new function to display the initialization page
void displayInitializationPage(const char* status) {
    resetImageBuffer();
    
    // Page title
    Paint_DrawString_EN(10, 5, "Initializing...", &Font20, BLACK, WHITE);
    
    // Show battery and WiFi status icons just like regular pages
    drawBatteryIcon(150, 5);
    if (wifi.getConnected() == 3) {
        drawWiFiConnectedIcon(130, 5);
    } else {
        drawWiFiDisconnectedIcon(130, 5);
    }
    
    // Show current status message
    Paint_DrawString_EN(10, 40, status, &Font16, BLACK, WHITE);
    
    // Draw status indicators
    Paint_DrawString_EN(10, 70, "Data collection:", &Font12, BLACK, WHITE);
    Paint_DrawString_EN(120, 70, initialDataCollected ? "OK" : "Waiting", &Font12, BLACK, WHITE);
    
    Paint_DrawString_EN(10, 90, "Flash storage:", &Font12, BLACK, WHITE);
    Paint_DrawString_EN(120, 90, initialDataSaved ? "OK" : "Waiting", &Font12, BLACK, WHITE);
    
    // Show sensors status
    Paint_DrawString_EN(10, 120, "Sensors:", &Font12, BLACK, WHITE);
    if (setupComplete) {
        Paint_DrawString_EN(120, 120, "OK", &Font12, BLACK, WHITE);
    } else {
        Paint_DrawString_EN(120, 120, "Initializing", &Font12, BLACK, WHITE);
    }
    
    // Show guidance
    if (initialDataCollected && initialDataSaved) {
        Paint_DrawString_EN(10, 150, "Setup complete!", &Font16, BLACK, WHITE);
        Paint_DrawString_EN(10, 175, "Press any button to begin", &Font12, BLACK, WHITE);
    } else {
        Paint_DrawString_EN(10, 150, "Please wait...", &Font16, BLACK, WHITE);
    }
    
    // Update the display
    EPD_1IN54_V2_Display(ImageBuffer);
}

// Add this new function to upload data in smaller chunks
bool uploadSensorDataChunked(Flash& flash, myGPS& gps) {
    if (flash.getStoredCount() == 0) {
        printf("No data to upload\n");
        displayUploadStatus("No data to upload");
        return true; // Nothing to upload is considered success
    }
    
    printf("Preparing to upload %lu records from flash\n", flash.getStoredCount());
    displayUploadStatus("Preparing data...");
    
    // Load records from flash
    std::vector<SensorData> records = flash.loadSensorData();
    printf("Loaded %lu records from flash\n", records.size());
    
    if (records.empty()) {
        printf("No valid records found in flash\n");
        displayUploadStatus("No valid data");
        return false;
    }
    
    // Define chunk size - 5 records per chunk to avoid overwhelming the TLS client
    const size_t CHUNK_SIZE = 10;
    size_t total_chunks = (records.size() + CHUNK_SIZE - 1) / CHUNK_SIZE; // Ceiling division
    size_t total_records = records.size();
    size_t successful_uploads = 0;
    
    // Buffer for JSON data - smaller for chunked uploads
    char json_buffer[8192]; // Smaller buffer for chunked data
    
    displayUploadStatus("Starting chunked upload");
    printf("Uploading %lu records in %lu chunks (%lu records per chunk)\n", 
           total_records, total_chunks, CHUNK_SIZE);
    
    // Process each chunk
    for (size_t chunk = 0; chunk < total_chunks; chunk++) {
        // Calculate chunk boundaries
        size_t start_idx = chunk * CHUNK_SIZE;
        size_t end_idx = std::min(start_idx + CHUNK_SIZE, total_records);
        size_t chunk_size = end_idx - start_idx;
        
        // Create a smaller vector with just this chunk's records
        std::vector<SensorData> chunk_records(records.begin() + start_idx, records.begin() + end_idx);
        
        // Display current chunk status
        char status_msg[64];
        sprintf(status_msg, "Uploading chunk %lu/%lu", chunk + 1, total_chunks);
        displayUploadStatus(status_msg);
        
        printf("Processing chunk %lu/%lu (records %lu-%lu)\n", 
               chunk + 1, total_chunks, start_idx + 1, end_idx);
        
        // Clear buffer before reuse
        memset(json_buffer, 0, sizeof(json_buffer));
        
        // Prepare JSON for just this chunk
        prepareBatchDataForTransmission(chunk_records, json_buffer, sizeof(json_buffer), gps);
        
        // Show size of the current chunk's JSON
        printf("Chunk %lu JSON size: %lu bytes\n", chunk + 1, strlen(json_buffer));
        
        // Attempt upload with 2 retries, 2 second initial delay
        bool result = uploadDataWithRetry(json_buffer, 2, 2000);
        
        if (result) {
            printf("Chunk %lu/%lu uploaded successfully\n", chunk + 1, total_chunks);
            successful_uploads++;
        } else {
            printf("Chunk %lu/%lu upload failed\n", chunk + 1, total_chunks);
        }
        
        // Add delay between chunks to allow system to recover
        sleep_ms(1000);
    }
    
    // Show final results
    printf("Upload complete: %lu/%lu chunks successful (%lu/%lu records)\n", 
           successful_uploads, total_chunks, 
           successful_uploads * CHUNK_SIZE < total_records ? successful_uploads * CHUNK_SIZE : total_records, 
           total_records);
    
    // Only consider upload successful if all chunks were uploaded
    bool all_chunks_successful = (successful_uploads == total_chunks);
    
    // If upload was successful, clear the flash storage
    if (all_chunks_successful) {
        displayUploadStatus("All data uploaded!");
        sleep_ms(2000);
        displayUploadStatus("Data preserved");
        sleep_ms(1000);
        printf("Upload complete: Data preserved for future uploads\n");
    } else if (successful_uploads > 0) {
        char result_msg[64];
        sprintf(result_msg, "%lu/%lu chunks uploaded", successful_uploads, total_chunks);
        displayUploadStatus(result_msg);
        sleep_ms(2000);
    } else {
        displayUploadStatus("Upload failed");
        sleep_ms(2000);
    }
    
    return all_chunks_successful;
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
#if defined(DISABLE_FLASH) && DISABLE_FLASH == 1
    flash_storage.setFlashEnabled(false);
    printf("Flash operations DISABLED by configuration\n");
#endif
    flash_initialized = flash_storage.init();
    if (flash_initialized) {
        printf("Flash storage initialized successfully\n");
        printf("Flash storage can hold up to %lu records\n", flash_storage.getMaxDataCount());
        printf("Currently %lu records stored\n", flash_storage.getStoredCount());
    } else {
        printf("Flash storage initialization failed\n");
    }
    
    printf("Initializing GPS module...\n");
    myGPS gps(uart0, 9600, 0, 1);
    printf("GPS module initialized\n");

#if USE_FAKE_GPS
    // Enable fake GPS data for indoor testing
    printf("NOTICE: Fake GPS data enabled for indoor testing\n");
    gps.enableFakeGPS(true);
    // You can set custom coordinates if needed
    // gps.setFakeCoordinates(48.20662016908546, 15.617513602109687);
#endif

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
    gps_start_time = get_absolute_time();
    printf("Starting GPS acquisition...\n");
    
    // Initialize variables for main loop
    bool has_fix = false;
    fix_status = 2; // Start with invalid fix
    satellites_visible = 0;
    
    last_refresh_time = get_absolute_time();
    printf("Main loop starting now\n");

    // Update with new initialization flags
    initialDataCollected = false;
    initialDataSaved = false;
    initializationComplete = false;
    
    // Display the initial initialization screen before entering the main loop
    displayInitializationPage("Starting up...");
    sleep_ms(1000);

    // Set up storage and data collection timing variables
    // Reduce collection interval to match refresh interval (already handled by dataCollectionInterval)
    // Reduce flash save interval for more frequent saves while biking (every 30s instead of 3min)
    const uint32_t FLASH_SAVE_INTERVAL_MS = 30000;    // Save to flash every 30 seconds for bike mode
    uint32_t current_time_ms = to_ms_since_boot(get_absolute_time());
    uint32_t last_data_collection_ms = current_time_ms;
    uint32_t last_flash_save_ms = current_time_ms;

    // Add a shorter save interval for initialization - keep this the same
    const uint32_t INIT_FLASH_SAVE_INTERVAL_MS = 60000; // 1 minute during initialization
    
    printf("BIKE MODE: Display refresh=%d ms, data collection=%d ms, flash save=%d ms\n",
           refreshInterval, dataCollectionInterval, (unsigned int)FLASH_SAVE_INTERVAL_MS);

    // Reduce buffer size for bike mode to ensure more frequent saves
    // and prevent large data loss in case of power issues during the ride
    const int MAX_BUFFER_SIZE = 10; // Reduced from 25 for more frequent saves during biking
    // Smaller buffer size during initialization - keep this the same
    const int INIT_MAX_BUFFER_SIZE = 2;  // Save after just 2 readings during initialization
    
    // GPS error recovery variables
    int consecutive_gps_errors = 0;
    uint32_t last_gps_recovery_time_ms = 0;
    uint32_t gps_recovery_interval_ms = 60000; // Try recovery every minute if needed
    
    // Wait a moment for systems to stabilize
    sleep_ms(1000);
    
    // After all initialization, but before the main loop, add:
    
    // Check if there's data in flash when starting up
    if (!setupComplete) {
        // Check if there's existing data in flash storage
        uint32_t stored_count = flash_storage.getStoredCount();
        if (stored_count > 0) {
            printf("Found %lu existing records in flash storage\n", stored_count);
            
            // Temporarily disable fast refresh for startup prompt to ensure it displays properly
            bool was_fast_refresh_enabled = fast_refresh_enabled;
            if (fast_refresh_enabled) {
                printf("Temporarily disabling fast refresh for startup prompt\n");
                fast_refresh_enabled = false;
                base_image_set = false;  // Reset base image state
            }
            
            // Ask user if they want to continue with existing data
            displayUploadStatus("Continue with");
            sleep_ms(500);
            displayUploadStatus("existing data?");
            
            printf("STARTUP: Beginning timeout sequence\n");
            
            // ULTRA RELIABLE TIMEOUT IMPLEMENTATION
            // No dependencies on anything except sleep_ms()
            bool keep_data = true; // Default to keeping data
            
            // Print a message every second to show we're alive
            for (int i = 0; i < 10; i++) {
                printf("STARTUP: Waiting for button press - %d seconds elapsed, %d remaining\n", 
                       i, 10-i);
                
                // Check buttons 20 times over 1 second (50ms intervals)
                for (int j = 0; j < 20; j++) {
                    // Direct GPIO read of buttons (no interrupt dependence)
                    if (gpio_get(BUTTON_NEXT_PAGE) == 0) {  // Button is pressed (active low)
                        printf("STARTUP: Button 0 (Next Page) pressed directly\n");
                        keep_data = true;
                        goto timeout_complete;
                    }
                    
                    if (gpio_get(BUTTON_REFRESH_DISPLAY) == 0) {  // Button is pressed (active low)
                        printf("STARTUP: Button 1 (Refresh Display) pressed directly\n");
                        keep_data = false;
                        goto timeout_complete;
                    }
                    
                    // Very short sleep between checks
                    sleep_ms(50);
                }
            }
            
timeout_complete:
            // Show what decision was made
            printf("STARTUP: Timeout complete - %s existing data\n", 
                   keep_data ? "keeping" : "erasing");
            
            if (!keep_data) {
                // User chose to start fresh
                printf("Erasing flash storage\n");
                displayUploadStatus("Erasing old data");
                
                // Clear the storage
                flash_storage.eraseStorage();
                
                printf("Flash storage erased\n");
                displayUploadStatus("Starting fresh");
                sleep_ms(2000);
            } else {
                printf("Continuing with existing data (%lu records)\n", stored_count);
                displayUploadStatus("Continuing ride");
                sleep_ms(2000);
            }
            
            // Restore fast refresh mode if it was enabled before
            if (was_fast_refresh_enabled) {
                printf("Re-enabling fast refresh mode after startup prompt\n");
                fast_refresh_enabled = true;
                base_image_set = false;  // We'll need to set a new base image
            }
        }
        
        setupComplete = true;
    }
    
    // Additional setup: Enable fast refresh mode for better responsiveness
    printf("Fast refresh disabled by default for display reliability\n");
    printf("Enable from Settings page if desired (double press button)\n");
    // Do not call setFastRefreshMode here
    
    // Main loop
    while (true) {
        // Handle any pending button input
        volatile uint32_t events = btn1_events;
        btn1_events = 0;
        
        if (events & GPIO_IRQ_EDGE_FALL) {
            // Button press processing...
        }
        
        // Process GPS data on each iteration
        std::string gps_data;
        int gps_result = gps.readLine(gps_data);
        
        // Don't wait too long for GPS data before proceeding with other tasks
        // This prevents hanging the main loop on GPS data
        static uint32_t last_task_time = 0;
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        
        // Process other tasks at least every 100ms regardless of GPS activity
        if (current_time - last_task_time > 100) {
            // Check if it's time to collect data
            if (current_time - last_data_collection_ms >= dataCollectionInterval) {
                printf("TIMING: Data collection triggered (elapsed: %u ms, interval: %u ms)\n",
                       (unsigned int)(current_time - last_data_collection_ms),
                       (unsigned int)dataCollectionInterval);
                
                // Read latest data from sensors and GPS
                batteryLevel = batteryADC.calculateBatteryLevel();
                if (batteryLevel == 0) {
                    batteryLevel = 50;  // Use default value if reading fails
                }
                
                // Read real sensor values from BME688
                float temp, hum, pres, gas;
                if (bme688_sensor.readData(temp, hum, pres, gas)) {
                    sensor_data_obj.temp = temp;
                    sensor_data_obj.hum = hum;
                    sensor_data_obj.pres = pres;
                    sensor_data_obj.gasRes = gas;
                } else {
                    printf("Failed to read from BME688 sensor\n");
                }
                
                // Read real values from HM3301 particulate matter sensor
                uint16_t pm1_0, pm2_5, pm10;
                if (hm3301_sensor.read(pm1_0, pm2_5, pm10)) {
                    sensor_data_obj.pm2_5 = pm2_5;
                    sensor_data_obj.pm5 = pm1_0;  // Using PM1.0 for PM5 since there's no direct PM5 reading
                    sensor_data_obj.pm10 = pm10;
                    } else {
                    printf("Failed to read from HM3301 sensor\n");
                }
                
                // Read real CO2 values from PAS CO2 sensor
                pas_co2_sensor.read();
                sensor_data_obj.co2 = pas_co2_sensor.getResult();
                
                // Get GPS data
                std::string gps_line;
                double longitude, latitude;
                char ew, ns;
                std::string time_str, date_str;
                
                int gps_result = gps.readLine(gps_line, longitude, ew, latitude, ns, time_str, date_str);
                // Set GPS data in the sensor data object
                sensor_data_obj.longitude = (int32_t)(longitude * 10000000); // Store as fixed-point
                sensor_data_obj.latitude = (int32_t)(latitude * 10000000);   // Store as fixed-point
                
                // Set timestamp from system time
                sensor_data_obj.timestamp = time(NULL);
                
                // Set fake GPS flag based on gps settings
                sensor_data_obj.is_fake_gps = (USE_FAKE_GPS == 1);
                
                // Add to in-memory buffer
                data_buffer.push_back(sensor_data_obj);
                buffer_modified = true;
                printf("Added data record #%lu to buffer (now %lu records in buffer)\n",
                       flash_storage.getStoredCount() + data_buffer.size(), data_buffer.size());
                
                // Set the initial data collected flag
                if (!initialDataCollected) {
                    initialDataCollected = true;
                    printf("INIT: First data collection complete\n");
                    // Update initialization page if we're still in init mode
                    if (!initializationComplete) {
                        displayInitializationPage("First data collected");
                        
                        // Force a quicker save after first collection if we have multiple records
                        if (data_buffer.size() >= 2) {
                            printf("INIT: Already have multiple records, triggering immediate save\n");
                            // Reset the last save time to force a save on next iteration
                            last_flash_save_ms = current_time - INIT_FLASH_SAVE_INTERVAL_MS;
                        }
                    }
                }
                
                // Update last collection time
                last_data_collection_ms = current_time;
                
                // Force refresh of display with updated sensor data
                            refresh_display = true;
            }
            
            // Check if it's time to save to flash or if buffer is getting too full
            if (((current_time - last_flash_save_ms >= 
                  (initializationComplete ? FLASH_SAVE_INTERVAL_MS : INIT_FLASH_SAVE_INTERVAL_MS)) || 
                 (data_buffer.size() >= 
                  (initializationComplete ? MAX_BUFFER_SIZE : INIT_MAX_BUFFER_SIZE))) && 
                data_buffer.size() > 0) {
                
                printf("TIMING: Flash save triggered (elapsed: %u ms, interval: %u ms, buffer size: %lu)\n",
                       (unsigned int)(current_time - last_flash_save_ms),
                       (unsigned int)(initializationComplete ? FLASH_SAVE_INTERVAL_MS : INIT_FLASH_SAVE_INTERVAL_MS),
                       data_buffer.size());
                
                // Add forced save if we've collected first data but haven't saved yet
                if (initialDataCollected && !initialDataSaved && data_buffer.size() > 0) {
                    printf("INIT: Forcing first data save for initialization\n");
                    // Save is being handled below, so we don't need additional code here
                }
                
                // Save entire buffer to flash
                size_t saved_count = 0;
                for (const auto& data : data_buffer) {
                    if (flash_storage.saveSensorData(data)) {
                        saved_count++;
                    } else {
                        printf("ERROR: Failed to save record to flash (stored count: %lu)\n", 
                               flash_storage.getStoredCount());
                        
                        if (flash_storage.isStorageFull()) {
                            printf("Flash storage is full - cannot save more records\n");
                            
                            // Show a warning on the display
                            displayUploadStatus("Storage FULL!");
                            sleep_ms(2000);
                            displayUploadStatus("Upload required");
                            sleep_ms(2000);
                            
                                    break;
                                }
                    }
                }
                
                printf("Saved %lu/%lu records to flash. Total stored: %lu\n",
                       saved_count, data_buffer.size(), flash_storage.getStoredCount());
                
                // Clear buffer after successful save
                if (saved_count > 0) {
                    data_buffer.clear();
                    buffer_modified = false;
                    
                    // Set the initial data saved flag
                    if (!initialDataSaved) {
                        initialDataSaved = true;
                        printf("INIT: First data save complete\n");
                        // Update initialization page if we're still in init mode
                        if (!initializationComplete) {
                            displayInitializationPage("First save complete");
                            sleep_ms(500);
                            displayInitializationPage("Press any button");
                        }
                    }
                }
                
                // Update last save time
                last_flash_save_ms = current_time;
            }
            
            // Check if initialization is complete
            if (!initializationComplete && initialDataCollected && initialDataSaved) {
                // Check if any button is pressed to exit initialization
                if (button_state_changed || 
                    (tast_pressed[0] != NOT_PRESSED) || 
                    (tast_pressed[1] != NOT_PRESSED)) {
                    
                    // Transition from initialization to normal operation
                    printf("INIT: Initialization complete, transitioning to normal operation\n");
                    initializationComplete = true;
                    
                    // Clear button states
                    tast_pressed[0] = NOT_PRESSED;
                    tast_pressed[1] = NOT_PRESSED;
                    button_state_changed = false;
                    
                    // Force a refresh of the BME688 page (page 0)
                    current_page = 0;  // Page 0 is the BME688 page
                    printf("INIT: Setting display to BME688 page (page 0)\n");
                    refresh_display = true;
                }
            }
            
            // Add a timeout to force initialization completion after 30 seconds
            static uint32_t init_start_time = 0;
            if (!initializationComplete) {
                if (init_start_time == 0) {
                    init_start_time = current_time;
                } else if (current_time - init_start_time > 30000) {  // 30 second timeout
                    printf("INIT: Forcing initialization complete after timeout\n");
                    initializationComplete = true;
                    
                    // Force a refresh of the BME688 page (page 0)
                    current_page = 0;  // Page 0 is the BME688 page
                    printf("INIT: Setting display to BME688 page (page 0)\n");
                    refresh_display = true;
                }
            }
            
            // Process button state changes immediately when the flag is set and we're in normal operation
            if (initializationComplete && button_state_changed) {
                // Process button events here...
                if (tast_pressed[0] == SHORT_PRESSED) {
                    // Next page button
                    tast_pressed[0] = NOT_PRESSED;
                    // nextPage already calls forceDisplayRefresh
                    nextPage();
                    printf("Changed to page %d and refreshed display\n", current_page);
                } else if (tast_pressed[0] == LONG_PRESSED) {
                    // Long press on next page button - trigger data upload
                    tast_pressed[0] = NOT_PRESSED;
                    printf("Long press detected on button 0 - starting data upload\n");
                    
                    // Handle upload functionality
                    // Check if WiFi is connected
                    if (wifi.getConnected() != 3) {
                        printf("WiFi not connected, attempting to connect...\n");
                        displayUploadStatus("Connecting WiFi...");
                        
                        // Try to connect to WiFi
                        if (wifi.scanAndConnect() != 0) {
                            printf("Failed to connect to WiFi\n");
                            displayUploadStatus("WiFi connection failed");
                            sleep_ms(2000);
        } else {
                            printf("WiFi connected successfully\n");
                            displayUploadStatus("WiFi connected");
                            sleep_ms(1000);
                        }
                    }
                    
                    // Only proceed with upload if WiFi is connected
                    if (wifi.getConnected() == 3) {
                        // First flush any data from the buffer to flash
                        if (data_buffer.size() > 0) {
                            printf("Flushing %d records from buffer to flash before upload\n", data_buffer.size());
                            displayUploadStatus("Saving buffer...");
                            
                            for (const auto& data : data_buffer) {
                                flash_storage.saveSensorData(data);
                            }
                            
                            printf("Buffer saved to flash\n");
                            data_buffer.clear();
                        }
                        
                        // Now attempt the upload
                        uploadSensorDataChunked(flash_storage, gps);
                    } else {
                        displayUploadStatus("No WiFi, can't upload");
                        sleep_ms(2000);
                    }
                }
                
                if (tast_pressed[1] == SHORT_PRESSED) {
                    // Refresh/settings button
                    tast_pressed[1] = NOT_PRESSED;
                    // This now calls forceDisplayRefresh internally
                    refreshDisplay_Settings_Button(current_page);
                    printf("Settings updated and display refreshed\n");
                } else if (tast_pressed[1] == LONG_PRESSED) {
                    // Long press on refresh button - enter sleep mode
                    tast_pressed[1] = NOT_PRESSED;
                    printf("Long press detected on button 1 - entering sleep mode\n");
                    enterSleepMode();
                }
                
                button_state_changed = false;
            }
            
            // Update last task processing time
            last_task_time = current_time;
            
            // Check if display needs to be refreshed due to page change or data update
            static uint32_t last_display_refresh_time = 0;
            if (refresh_display || (current_time - last_display_refresh_time >= refreshInterval)) {
                if (initializationComplete) {
                    // Normal operation - display the current page
                    printf("Refreshing display for page %d (refresh flag: %s, timed: %s)\n", 
                          current_page,
                          refresh_display ? "true" : "false",
                          (current_time - last_display_refresh_time >= refreshInterval) ? "true" : "false");
                    
                    // Update the display based on current page
                    displayPage(current_page, gps_start_time, fix_status, satellites_visible, USE_FAKE_GPS);
                    
                    // Reset the refresh flag and update the refresh time
                    refresh_display = false;
                    last_display_refresh_time = current_time;
                    printf("Display refreshed for page %d\n", current_page);
            } else {
                    // We're still in initialization mode - update the status
                    // But only update periodically to avoid too much flicker
                    if (current_time - last_display_refresh_time >= 5000) { // Update every 5 seconds at most
                        displayInitializationPage("Please wait...");
                        refresh_display = false;
                        last_display_refresh_time = current_time;
                    }
                }
            }
            
            // Additional optional debug output
            static uint32_t last_debug_print_time = 0;
            if (current_time - last_debug_print_time > 10000) {  // Every 10 seconds
                printf("DEBUG: Current page: %d, Refresh flag: %s, Time since last refresh: %u ms\n", 
                       current_page,
                       refresh_display ? "true" : "false",
                       (unsigned int)(current_time - last_display_refresh_time));
                last_debug_print_time = current_time;
            }
        }
        
        // Watchdog feed to prevent resets
#ifdef USE_WATCHDOG
        watchdog_update();
#endif
    }

    free(ImageBuffer);
    return 0;
}
