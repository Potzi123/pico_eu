//
// Created by Benedikt Walter on 18.01.24.
//

#include "pico/stdlib.h"
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

    // Add timeout mechanism
    absolute_time_t timeout_time = make_timeout_time_ms(2000); // 2 second timeout
    
    bool valid_sentence_found = false;
    std::string sentence_type;
    
    // Keep trying until we find a valid sentence or timeout
    while (!valid_sentence_found && !absolute_time_diff_us(get_absolute_time(), timeout_time) <= 0) {
        this->buffer = "";
        // Start with an empty buffer
        while(this->buffer.empty() || this->buffer.back() != '\n') {
            if(uart_is_readable(this->uart_id)) {
                this->buffer += uart_getc(this->uart_id);
            } else {
                // Check for timeout
                if (absolute_time_diff_us(get_absolute_time(), timeout_time) <= 0) {
                    // GPS read timeout (not usually an error, just no data right now)
                    return 1;
                }
                sleep_ms(1); // Small delay to prevent busy-waiting
            }
            // If buffer gets too large without finding \n, reset it
            if(this->buffer.length() > 200) {
                // Buffer overflow, reset
                this->buffer = "";
                break;
            }
        }
        
        if(buffer.empty()) {
            continue;
        }
        
        // Only print raw GPS NMEA sentences if specifically debugging GPS
#if defined(DEBUG_GPS_LOG) && DEBUG_GPS_LOG
        printf("%s", this->buffer.c_str());
#endif
        
        // Check for either GLL or RMC sentences
        if (this->buffer.find(GNGLL) == 0) {
            valid_sentence_found = true;
            sentence_type = "GLL";
        } else if (this->buffer.find(GNRMC) == 0) {
            valid_sentence_found = true;
            sentence_type = "RMC";
        } else if (this->buffer.find("$GPRMC") == 0) {
            valid_sentence_found = true;
            sentence_type = "RMC";
        } else if (this->buffer.find("$GPGLL") == 0) {
            valid_sentence_found = true;
            sentence_type = "GLL";
        }
    }
    
    // If we didn't find a valid sentence before timeout
    if (!valid_sentence_found) {
#if defined(DEBUG_GPS_LOG) && DEBUG_GPS_LOG
        printf("No valid GLL or RMC sentence found before timeout\n");
#endif
        return 1;
    }

    line = this->buffer;
    std::istringstream iss(this->buffer);
    std::string token;

    // Skip the first token (sentence identifier)
    std::getline(iss, token, ',');
    
    if (sentence_type == "GLL") {
        // Parse GLL sentence
        // Format: $GNGLL,ddmm.mmmm,N,dddmm.mmmm,E,hhmmss.sss,A,*xx
        
        // Latitude
        std::getline(iss, token, ',');
        if (!token.empty()) {
            try {
                // Added try-catch block to handle invalid conversion
                if (token.length() >= 3) { // Make sure token has enough characters for parsing
                    this->latitude = std::stod(token.substr(0, 2)) + std::stod(token.substr(2)) / 60.0;
                } else {
                    this->latitude = 0;
                    printf("Warning: Latitude token too short: '%s'\n", token.c_str());
                }
#if defined(DEBUG_GPS_LOG) && DEBUG_GPS_LOG
                printf("Parsed latitude: %f\n", this->latitude);
#endif
            } catch (const std::exception& e) {
                this->latitude = 0;
                printf("Error parsing latitude token: '%s' - %s\n", token.c_str(), e.what());
            }
        } else {
            this->latitude = 0;
#if defined(DEBUG_GPS_LOG) && DEBUG_GPS_LOG
            printf("Empty latitude value\n");
#endif
        }

        // N/S indicator
        std::getline(iss, token, ',');
        if (!token.empty()) {
            this->nsIndicator = token[0];
        } else {
            this->nsIndicator = 'C'; // 'C' for "Can't determine"
        }

        // Longitude
        std::getline(iss, token, ',');
        if (!token.empty()) {
            try {
                // Added try-catch block to handle invalid conversion
                if (token.length() >= 4) { // Make sure token has enough characters for parsing
                    this->longitude = std::stod(token.substr(0, 3)) + std::stod(token.substr(3)) / 60.0;
                } else {
                    this->longitude = 0;
                    printf("Warning: Longitude token too short: '%s'\n", token.c_str());
                }
#if defined(DEBUG_GPS_LOG) && DEBUG_GPS_LOG
                printf("Parsed longitude: %f\n", this->longitude);
#endif
            } catch (const std::exception& e) {
                this->longitude = 0;
                printf("Error parsing longitude token: '%s' - %s\n", token.c_str(), e.what());
            }
        } else {
            this->longitude = 0;
#if defined(DEBUG_GPS_LOG) && DEBUG_GPS_LOG
            printf("Empty longitude value\n");
#endif
        }

        // E/W indicator
        std::getline(iss, token, ',');
        if (!token.empty()) {
            this->ewIndicator = token[0];
        } else {
            this->ewIndicator = 'C';
        }

        // Time
        std::getline(iss, token, ',');
#if defined(DEBUG_GPS_LOG) && DEBUG_GPS_LOG
        printf("Raw time token: '%s'\n", token.c_str()); // Add raw token debug
#endif
        if (!token.empty() && token.size() >= 6) {
            this->time = token.substr(0, 2) + ":" + token.substr(2, 2) + ":" + token.substr(4, 2);
#if defined(DEBUG_GPS_LOG) && DEBUG_GPS_LOG
            printf("Parsed timestamp: %s\n", this->time.c_str());
#endif
        } else {
            this->time = "00:00:00";
#if defined(DEBUG_GPS_LOG) && DEBUG_GPS_LOG
            printf("Empty or invalid time field in GPS data\n");
#endif
        }

        // Fix validity
        std::getline(iss, token, ',');
        bool valid_fix = (token == "A");
        
#if defined(DEBUG_GPS_LOG) && DEBUG_GPS_LOG
        printf("GPS Fix Status: %s\n", valid_fix ? "VALID" : "INVALID");
#endif
        
        return valid_fix ? 0 : 2;  // Return 0 for valid fix, 2 for invalid
    }
    else if (sentence_type == "RMC") {
        // Parse RMC sentence
        // Format: $GNRMC,hhmmss.sss,A,ddmm.mmmm,N,dddmm.mmmm,E,speed,course,ddmmyy,,,*xx
        
        // Time is the first field in RMC
        std::getline(iss, token, ',');
#if defined(DEBUG_GPS_LOG) && DEBUG_GPS_LOG
        printf("Raw time token (RMC): '%s'\n", token.c_str());
#endif
        if (!token.empty() && token.size() >= 6) {
            this->time = token.substr(0, 2) + ":" + token.substr(2, 2) + ":" + token.substr(4, 2);
#if defined(DEBUG_GPS_LOG) && DEBUG_GPS_LOG
            printf("Parsed timestamp (RMC): %s\n", this->time.c_str());
#endif
        } else {
            this->time = "00:00:00";
#if defined(DEBUG_GPS_LOG) && DEBUG_GPS_LOG
            printf("Empty or invalid time field in RMC data\n");
#endif
        }
        
        // Fix validity 
        std::getline(iss, token, ',');
        bool valid_fix = (token == "A");
        
        // Latitude
        std::getline(iss, token, ',');
        if (!token.empty()) {
            try {
                // Added try-catch block to handle invalid conversion
                if (token.length() >= 3) { // Make sure token has enough characters for parsing
                    this->latitude = std::stod(token.substr(0, 2)) + std::stod(token.substr(2)) / 60.0;
                } else {
                    this->latitude = 0;
                    printf("Warning: Latitude token too short: '%s'\n", token.c_str());
                }
#if defined(DEBUG_GPS_LOG) && DEBUG_GPS_LOG
                printf("Parsed latitude (RMC): %f\n", this->latitude);
#endif
            } catch (const std::exception& e) {
                this->latitude = 0;
                printf("Error parsing latitude token: '%s' - %s\n", token.c_str(), e.what());
            }
        } else {
            this->latitude = 0;
#if defined(DEBUG_GPS_LOG) && DEBUG_GPS_LOG
            printf("Empty latitude value in RMC\n");
#endif
        }
        
        // N/S indicator
        std::getline(iss, token, ',');
        if (!token.empty()) {
            this->nsIndicator = token[0];
        } else {
            this->nsIndicator = 'C';
        }
        
        // Longitude
        std::getline(iss, token, ',');
        if (!token.empty()) {
            try {
                // Added try-catch block to handle invalid conversion
                if (token.length() >= 4) { // Make sure token has enough characters for parsing
                    this->longitude = std::stod(token.substr(0, 3)) + std::stod(token.substr(3)) / 60.0;
                } else {
                    this->longitude = 0;
                    printf("Warning: Longitude token too short: '%s'\n", token.c_str());
                }
#if defined(DEBUG_GPS_LOG) && DEBUG_GPS_LOG
                printf("Parsed longitude (RMC): %f\n", this->longitude);
#endif
            } catch (const std::exception& e) {
                this->longitude = 0;
                printf("Error parsing longitude token: '%s' - %s\n", token.c_str(), e.what());
            }
        } else {
            this->longitude = 0;
#if defined(DEBUG_GPS_LOG) && DEBUG_GPS_LOG
            printf("Empty longitude value in RMC\n");
#endif
        }
        
        // E/W indicator
        std::getline(iss, token, ',');
        if (!token.empty()) {
            this->ewIndicator = token[0];
        } else {
            this->ewIndicator = 'C';
        }
        
#if defined(DEBUG_GPS_LOG) && DEBUG_GPS_LOG
        printf("GPS Fix Status (RMC): %s\n", valid_fix ? "VALID" : "INVALID");
        printf("Long: %f%c\n", this->longitude, this->ewIndicator);
        printf("Lat: %f%c\n", this->latitude, this->nsIndicator);
        printf("Time: %s\n\n", this->time.c_str());
#endif
        
        return valid_fix ? 0 : 2;  // Return 0 for valid fix, 2 for invalid
    }
    
    // If we get here, something went wrong with parsing
    return 1;
}

int myGPS::readLine(std::string &buffer, double &longitude, char &ewIndicator, double &latitude, char &nsIndicator, std::string &time) {
    int result = this->readLine(buffer);
    
    // Always copy the current values, even if invalid
    longitude = this->longitude;
    ewIndicator = this->ewIndicator;
    latitude = this->latitude;
    nsIndicator = this->nsIndicator;
    time = this->time;
    
    return result;
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

int myGPS::testConnection() {
    printf("Testing GPS connection...\n");
    
    // First, check if UART is configured properly
    if (!uart_is_enabled(this->uart_id)) {
        printf("UART not enabled. Initializing UART...\n");
        this->init();
        sleep_ms(100); // Give it time to initialize
    }
    
    // Attempt to read data (any data) from UART for 3 seconds
    printf("Checking if GPS module is sending any data...\n");
    absolute_time_t timeout = make_timeout_time_ms(3000);
    bool any_data_received = false;
    bool valid_nmea_seen = false;
    bool frame_errors_seen = false;
    int chars_received = 0;
    std::string sample_data;
    
    while (!absolute_time_diff_us(get_absolute_time(), timeout) <= 0) {
        if (uart_is_readable(this->uart_id)) {
            char c = uart_getc(this->uart_id);
            chars_received++;
            
            // Keep a small sample of the data for debugging
            if (sample_data.length() < 100) {
                sample_data += c;
            }
            
            // Look for NMEA sentence start character '$'
            if (c == '$') {
                any_data_received = true;
                
                // Try to read a complete NMEA sentence
                std::string nmea_sentence;
                nmea_sentence += c;
                
                absolute_time_t sentence_timeout = make_timeout_time_ms(500);
                bool complete_sentence = false;
                
                // Read until newline or timeout
                while (!complete_sentence && !absolute_time_diff_us(get_absolute_time(), sentence_timeout) <= 0) {
                    if (uart_is_readable(this->uart_id)) {
                        char nc = uart_getc(this->uart_id);
                        chars_received++;
                        nmea_sentence += nc;
                        
                        // Look for end of NMEA sentence
                        if (nc == '\n' || nc == '\r') {
                            complete_sentence = true;
                        }
                    } else {
                        sleep_ms(1);
                    }
                    
                    // Check for excessively long sentence (likely corrupt data)
                    if (nmea_sentence.length() > 100) {
                        break;
                    }
                }
                
                // Check if we found a valid NMEA sentence
                if (complete_sentence && nmea_sentence.length() > 6) {
                    // Check if it contains "More than 100 frame errors"
                    if (nmea_sentence.find("frame errors") != std::string::npos) {
                        frame_errors_seen = true;
                        printf("Frame errors detected in GPS data\n");
                    } else {
                        valid_nmea_seen = true;
                        printf("Valid NMEA sentence received: %s", nmea_sentence.c_str());
                    }
                }
            }
            
            any_data_received = true;
        } else {
            sleep_ms(10);
        }
        
        // Exit early if we've already seen valid NMEA data
        if (valid_nmea_seen && chars_received > 100) {
            break;
        }
    }
    
    printf("GPS test complete. Received %d characters.\n", chars_received);
    
    if (!any_data_received) {
        printf("No data received from GPS module. Check connections and power.\n");
        return 2; // UART communication error
    }
    
    if (frame_errors_seen) {
        printf("Frame errors detected. Likely baud rate mismatch or noisy connection.\n");
        printf("Try different baud rates: 4800, 9600, 38400, etc.\n");
        printf("Current baud rate: %d\n", this->baud_rate);
        return 3; // Frame errors
    }
    
    if (!valid_nmea_seen) {
        if (chars_received > 0 && chars_received < 10) {
            printf("Minimal data received. Likely incorrect baud rate.\n");
            printf("Sample data received: %s\n", sample_data.c_str());
            return 4; // Likely baud rate issue
        } else {
            printf("Data received but no valid NMEA sentences. GPS module may be starting up.\n");
            printf("Sample data received: %s\n", sample_data.c_str());
            return 1; // No valid NMEA data
        }
    }
    
    printf("GPS connection test successful. Valid NMEA data received.\n");
    return 0; // Good connection with valid data
}

int myGPS::getVisibleSatellites() {
    // Attempt to read from the GPS module for up to 2 seconds
    absolute_time_t timeout = make_timeout_time_ms(2000);
    int satellite_count = 0;
    bool found_gsv_message = false;
    
    printf("Checking for visible satellites...\n");
    
    while (!absolute_time_diff_us(get_absolute_time(), timeout) <= 0 && !found_gsv_message) {
        if (uart_is_readable(this->uart_id)) {
            std::string gsv_message = "";
            char c = uart_getc(this->uart_id);
            
            // Look for the start of a GSV message
            if (c == '$') {
                gsv_message += c;
                
                // Read more characters to check if this is a GSV message
                for (int i = 0; i < 5; i++) {
                    if (uart_is_readable(this->uart_id)) {
                        c = uart_getc(this->uart_id);
                        gsv_message += c;
                    } else {
                        sleep_ms(1);
                    }
                }
                
                // If we've found a GSV message (satellites in view)
                if (gsv_message == "$GPGSV" || gsv_message == "$GNGSV" || gsv_message == "$GLGSV") {
                    found_gsv_message = true;
                    
                    // Continue reading the rest of the message
                    bool reading_message = true;
                    while (reading_message) {
                        if (uart_is_readable(this->uart_id)) {
                            c = uart_getc(this->uart_id);
                            gsv_message += c;
                            
                            // End of message
                            if (c == '\n' || c == '\r') {
                                reading_message = false;
                            }
                        } else {
                            sleep_ms(1);
                        }
                        
                        // Safety check for message length
                        if (gsv_message.length() > 100) {
                            reading_message = false;
                        }
                    }
                    
                    // Parse the GSV message to get satellite count
                    std::istringstream iss(gsv_message);
                    std::string token;
                    
                    // GSV message format: $GPGSV,3,1,11,...
                    // Third field is the total number of satellites in view
                    
                    // Skip the header
                    std::getline(iss, token, ',');
                    
                    // Skip the total message count
                    std::getline(iss, token, ',');
                    
                    // Skip the message number
                    std::getline(iss, token, ',');
                    
                    // Get the number of satellites in view
                    std::getline(iss, token, ',');
                    if (!token.empty()) {
                        try {
                            satellite_count = std::stoi(token);
                            printf("Satellites in view: %d\n", satellite_count);
                        } catch (const std::exception& e) {
                            printf("Error parsing satellite count: %s\n", e.what());
                        }
                    }
                }
            }
        } else {
            sleep_ms(10);
        }
    }
    
    return satellite_count;
}

// Add a helper method to assist with getting a position fix
bool myGPS::waitForFix(int timeout_seconds) {
    printf("Waiting for GPS fix (timeout: %d seconds)...\n", timeout_seconds);
    
    absolute_time_t timeout = make_timeout_time_ms(timeout_seconds * 1000);
    std::string buffer;
    bool got_fix = false;
    
    // Reset receiver if we haven't received any valid data
    if (!uart_is_readable(this->uart_id)) {
        printf("No data from GPS, reinitializing...\n");
        this->init();
        sleep_ms(200);
    }
    
    while (!absolute_time_diff_us(get_absolute_time(), timeout) <= 0 && !got_fix) {
        // Check for readable data
        if (uart_is_readable(this->uart_id)) {
            try {
                // Try to read a full line with fix information
                std::string tmp_buffer;
                double longitude = 0, latitude = 0;
                char ewIndicator = ' ', nsIndicator = ' ';
                std::string time;
                
                int status = this->readLine(tmp_buffer, longitude, ewIndicator, latitude, nsIndicator, time);
                
                // If we got a valid fix, break out of the loop
                if (status == 0) {
                    buffer = tmp_buffer;
                    got_fix = true;
                    printf("Got GPS fix! Lat: %f%c, Long: %f%c\n", 
                           latitude, nsIndicator, longitude, ewIndicator);
                } else {
                    // If we're still waiting, check every 5 seconds how many satellites we can see
                    static uint32_t last_sat_check = 0;
                    uint32_t now = to_ms_since_boot(get_absolute_time());
                    
                    if (now - last_sat_check > 5000) {
                        int sats = this->getVisibleSatellites();
                        printf("Waiting for fix... Satellites in view: %d\n", sats);
                        last_sat_check = now;
                        
                        // Calculate remaining time
                        int remaining = (timeout_seconds * 1000 - (now - to_ms_since_boot(timeout) + timeout_seconds * 1000)) / 1000;
                        printf("Timeout in %d seconds\n", remaining > 0 ? remaining : 0);
                    }
                    
                    // Small delay to not flood the terminal
                    sleep_ms(200);
                }
            } catch (const std::exception& e) {
                printf("Exception while waiting for fix: %s\n", e.what());
                sleep_ms(100);
            }
        } else {
            sleep_ms(100);
        }
    }
    
    if (!got_fix) {
        printf("Timeout waiting for GPS fix\n");
    }
    
    return got_fix;
}

// Send hot start command to the GPS module
bool myGPS::sendHotStartCommand() {
    printf("Sending GPS hot start command...\n");
    
    // Ensure UART is initialized
    if (!uart_is_enabled(this->uart_id)) {
        this->init();
        sleep_ms(100);
    }
    
    // For NMEA GPS modules, the hot start command varies by manufacturer
    // This is the most common format, compatible with many GPS modules
    // $PMTK101*32<CR><LF> - Hot start (MTK chipset)
    const char *hot_start_cmd = "$PMTK101*32\r\n";
    
    // Send the command
    for (const char *p = hot_start_cmd; *p != '\0'; p++) {
        uart_putc(this->uart_id, *p);
    }
    
    // Add a small delay to let the command take effect
    sleep_ms(500);
    
    // Also send specific commands to enable time output
    // $PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28 - Enable RMC and GLL sentences
    const char *enable_time_cmd = "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n";
    
    for (const char *p = enable_time_cmd; *p != '\0'; p++) {
        uart_putc(this->uart_id, *p);
    }
    
    sleep_ms(200);
    
    // Also try u-blox format (used in many GPS modules)
    // Warm start for u-blox
    const char *ublox_warm_start = "$PUBX,40,GLL,0,1,0,0,0,0*5D\r\n"; // Enable GLL messages
    for (const char *p = ublox_warm_start; *p != '\0'; p++) {
        uart_putc(this->uart_id, *p);
    }
    
    sleep_ms(200);
    
    // Also enable RMC messages which contain time info
    const char *ublox_enable_rmc = "$PUBX,40,RMC,0,1,0,0,0,0*47\r\n";
    for (const char *p = ublox_enable_rmc; *p != '\0'; p++) {
        uart_putc(this->uart_id, *p);
    }
    
    sleep_ms(500);
    
    // Check if we're receiving data after the restart
    int attempts = 10;
    bool data_received = false;
    
    while (attempts > 0 && !data_received) {
        sleep_ms(100);
        if (uart_is_readable(this->uart_id)) {
            data_received = true;
        }
        attempts--;
    }
    
    if (data_received) {
        printf("GPS module responded after hot start command\n");
        
        // Flush any pending data
        while (uart_is_readable(this->uart_id)) {
            uart_getc(this->uart_id);
        }
        
        return true;
    } else {
        printf("No response from GPS module after hot start command\n");
        return false;
    }
}

// Send warm start command to the GPS module
bool myGPS::sendWarmStartCommand() {
    printf("Sending GPS warm start command...\n");
    
    // Ensure UART is initialized
    if (!uart_is_enabled(this->uart_id)) {
        this->init();
        sleep_ms(100);
    }
    
    // For NMEA GPS modules, the warm start command varies by manufacturer
    // This is the most common format for MTK chipsets
    // $PMTK102*31<CR><LF> - Warm start
    const char *warm_start_cmd = "$PMTK102*31\r\n";
    
    // Send the command
    for (const char *p = warm_start_cmd; *p != '\0'; p++) {
        uart_putc(this->uart_id, *p);
    }
    
    // Add a small delay to let the command take effect
    sleep_ms(500);
    
    // Check if we're receiving data after the restart
    int attempts = 10;
    bool data_received = false;
    
    while (attempts > 0 && !data_received) {
        sleep_ms(100);
        if (uart_is_readable(this->uart_id)) {
            data_received = true;
        }
        attempts--;
    }
    
    if (data_received) {
        printf("GPS module responded after warm start command\n");
        
        // Flush any pending data
        while (uart_is_readable(this->uart_id)) {
            uart_getc(this->uart_id);
        }
        
        return true;
    } else {
        printf("No response from GPS module after warm start command\n");
        return false;
    }
}

// Send a command to enable time messages from the GPS module
bool myGPS::enableTimeMessages() {
    printf("Sending command to enable GPS time messages...\n");
    
    // Ensure UART is initialized
    if (!uart_is_enabled(this->uart_id)) {
        this->init();
        sleep_ms(100);
    }
    
    // Try multiple command formats to cover different GPS module types
    
    // 1. For MTK chipsets - Enable GLL messages which contain time
    const char *mtk_enable_gll = "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n";
    
    // 2. For u-blox modules - Enable GLL messages
    const char *ublox_enable_gll = "$PUBX,40,GLL,0,1,0,0,0,0*5D\r\n";
    
    // 3. For SiRF modules - Enable NMEA output
    const char *sirf_enable_nmea = "$PSRF100,1,9600,8,1,0*0C\r\n";
    
    // Send all commands with delays between them
    printf("Sending MTK command...\n");
    for (const char *p = mtk_enable_gll; *p != '\0'; p++) {
        uart_putc(this->uart_id, *p);
    }
    sleep_ms(200);
    
    printf("Sending u-blox command...\n");
    for (const char *p = ublox_enable_gll; *p != '\0'; p++) {
        uart_putc(this->uart_id, *p);
    }
    sleep_ms(200);
    
    printf("Sending SiRF command...\n");
    for (const char *p = sirf_enable_nmea; *p != '\0'; p++) {
        uart_putc(this->uart_id, *p);
    }
    sleep_ms(500);
    
    // Flush any pending data
    while (uart_is_readable(this->uart_id)) {
        uart_getc(this->uart_id);
    }
    
    printf("Time message commands sent, waiting for response...\n");
    
    // Check if we're getting any data back
    absolute_time_t timeout = make_timeout_time_ms(3000);
    bool got_response = false;
    std::string received_data;
    
    while (!absolute_time_diff_us(get_absolute_time(), timeout) <= 0 && received_data.length() < 100) {
        if (uart_is_readable(this->uart_id)) {
            char c = uart_getc(this->uart_id);
            received_data += c;
            got_response = true;
            
            // If we've received a full line, break
            if (c == '\n' && received_data.find('$') != std::string::npos) {
                break;
            }
        } else {
            sleep_ms(10);
        }
    }
    
    if (got_response) {
        printf("Received response from GPS module: %s\n", received_data.c_str());
        return true;
    } else {
        printf("No response from GPS module after time message commands\n");
        return false;
    }
}

// Send cold start command to the GPS module
bool myGPS::sendColdStartCommand() {
    printf("Sending GPS cold start command...\n");
    
    // Ensure UART is initialized
    if (!uart_is_enabled(this->uart_id)) {
        this->init();
        sleep_ms(100);
    }
    
    // For NMEA GPS modules, the cold start command varies by manufacturer
    // This is the most common format for MTK chipsets
    // $PMTK103*30<CR><LF> - Cold start
    const char *cold_start_cmd = "$PMTK103*30\r\n";
    
    // Send the command
    for (const char *p = cold_start_cmd; *p != '\0'; p++) {
        uart_putc(this->uart_id, *p);
    }
    
    // Add a longer delay for cold start since it's a full reset
    sleep_ms(1000);
    
    // Also try u-blox format for cold start
    // $PUBX,40,GLL,0,1,0,0,0,0*5D\r\n - Enable GLL messages after reset
    const char *ublox_cold_start = "$PUBX,104*37\r\n";
    
    // Send the command
    for (const char *p = ublox_cold_start; *p != '\0'; p++) {
        uart_putc(this->uart_id, *p);
    }
    
    sleep_ms(1000);
    
    // Configure NMEA sentences after cold start
    const char *enable_sentences = "$PMTK314,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n";
    
    // Send the command to enable specific NMEA sentences
    for (const char *p = enable_sentences; *p != '\0'; p++) {
        uart_putc(this->uart_id, *p);
    }
    
    sleep_ms(500);
    
    // Check if we're receiving data after the cold start
    int attempts = 20; // More attempts for cold start
    bool data_received = false;
    
    printf("Waiting for GPS to restart after cold start...\n");
    
    while (attempts > 0 && !data_received) {
        sleep_ms(200);
        if (uart_is_readable(this->uart_id)) {
            data_received = true;
        }
        attempts--;
    }
    
    if (data_received) {
        printf("GPS module responded after cold start command\n");
        
        // Flush any pending data
        while (uart_is_readable(this->uart_id)) {
            uart_getc(this->uart_id);
        }
        
        // Send enable time messages command after cold start
        enableTimeMessages();
        
        return true;
    } else {
        printf("No response from GPS module after cold start command\n");
        return false;
    }
}
