//
// Created by Benedikt Walter on 19.01.24.
//

#include "wifi.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include <string>
#include <vector>
#include <algorithm>
#include "password.h"


std::vector<std::string> scannedSSID;

int myWIFI::getConnected() {
    return this->connected;
}

void myWIFI::poll() {
    this->connected = cyw43_tcpip_link_status(&cyw43_state, 0);
}

int myWIFI::init() {
    if(cyw43_arch_init_with_country(CYW43_COUNTRY_AUSTRIA)) {
        printf("Failed to initialise");
        return 1;
    }
    printf("WiFi initialised");
    return 0;
}

int myWIFI::scanAndConnect() {
    printf("[WIFI DEBUG] Starting scanAndConnect method\n");
    
    // Check if already connected
    if (this->connected == CYW43_LINK_UP) {
        printf("[WIFI DEBUG] Already connected to WiFi\n");
        return 0;
    }
    
    // Reset connection state
    this->trying_to_connect = false;
    
    // Initialize WiFi if it's not already initialized
    if (cyw43_is_initialized(&cyw43_state) == 0) {
        printf("[WIFI DEBUG] CYW43 not initialized, initializing now\n");
        if (cyw43_arch_init_with_country(CYW43_COUNTRY_AUSTRIA)) {
            printf("[WIFI ERROR] Failed to initialize WiFi\n");
            return 1;
        }
    }
    
    cyw43_arch_enable_sta_mode();
    printf("[WIFI DEBUG] STA Mode active\n");
    
    // Clear previous scan results
    scannedSSID.clear();
    printf("[WIFI DEBUG] Cleared previous scan results\n");
    
    cyw43_wifi_scan_options_t scan_options = {0};
    int scan_start_result;
    
    // Retry scan if it fails to start
    int scan_start_attempts = 0;
    const int MAX_SCAN_START_ATTEMPTS = 3;
    
    while (scan_start_attempts < MAX_SCAN_START_ATTEMPTS) {
        printf("[WIFI DEBUG] Starting WiFi scan (attempt %d/%d)...\n", 
               scan_start_attempts + 1, MAX_SCAN_START_ATTEMPTS);
        
        scan_start_result = cyw43_wifi_scan(&cyw43_state, &scan_options, nullptr, scan_result);
        
        if (scan_start_result == 0) {
            printf("[WIFI DEBUG] WiFi scan initiated successfully\n");
            break;
        }
        
        printf("[WIFI ERROR] Could not start WiFi Scan! Error: %d\n", scan_start_result);
        scan_start_attempts++;
        
        if (scan_start_attempts < MAX_SCAN_START_ATTEMPTS) {
            printf("[WIFI DEBUG] Retrying scan in 1 second...\n");
            sleep_ms(1000);
        }
    }
    
    if (scan_start_result != 0) {
        printf("[WIFI ERROR] Failed to start WiFi scan after %d attempts\n", MAX_SCAN_START_ATTEMPTS);
        return 1;
    }
    
    // Wait for scan to complete with timeout
    printf("[WIFI DEBUG] Waiting for scan to complete...\n");
    absolute_time_t scan_timeout = make_timeout_time_ms(10000); // 10 second timeout
    bool scan_completed = false;
    
    while (!scan_completed) {
        // Check for timeout
        if (absolute_time_diff_us(get_absolute_time(), scan_timeout) <= 0) {
            printf("[WIFI ERROR] Scan timeout after 10 seconds\n");
            
            // If we have some networks, continue anyway
            if (!scannedSSID.empty()) {
                printf("[WIFI DEBUG] Proceeding with %d networks found before timeout\n", (int)scannedSSID.size());
                scan_completed = true;
            } else {
                printf("[WIFI ERROR] No networks found before timeout\n");
                
                // Try to reset the WiFi scan state
                printf("[WIFI DEBUG] Attempting to reset WiFi scan state\n");
                
                // First check if scan is still active and wait a short time if it is
                if (cyw43_wifi_scan_active(&cyw43_state)) {
                    printf("[WIFI DEBUG] Scan still active, waiting 500ms\n");
                    sleep_ms(500);
                }
                
                return 1;
            }
        }
        
        // Check if scan is complete
        if (!cyw43_wifi_scan_active(&cyw43_state)) {
            printf("[WIFI DEBUG] Scan completed normally\n");
            scan_completed = true;
        }
        
        // Make sure cyw43 background tasks run
        cyw43_arch_poll();
        sleep_ms(100); // Small delay to prevent busy waiting
    }
    
    // Process scan results
    printf("[WIFI DEBUG] Processing scan results...\n");
    std::sort(scannedSSID.begin(), scannedSSID.end());
    scannedSSID.erase(std::unique(scannedSSID.begin(), scannedSSID.end()), scannedSSID.end());
    scannedSSID.erase(std::remove(scannedSSID.begin(), scannedSSID.end(), ""), scannedSSID.end());
    
    printf("[WIFI DEBUG] -------- WiFi Scan Results --------\n");
    printf("[WIFI DEBUG] Found %d networks\n", (int)scannedSSID.size());
    for(const auto &ssid : scannedSSID) {
        printf("[WIFI DEBUG] Network: %s\n", ssid.c_str());
    }
    printf("[WIFI DEBUG] ------------------------------------\n");
    
    if (scannedSSID.empty()) {
        printf("[WIFI ERROR] No networks found in scan\n");
        return 1;
    }
    
    int attempts = 0;
    bool connected = false;
    
    // Find matching networks from our known list
    printf("[WIFI DEBUG] Looking for known networks in scan results...\n");
    for (size_t i = 0; i < SSID.size(); i++) {
        printf("[WIFI DEBUG] Checking for known network: %s\n", SSID.at(i).c_str());
        
        for(const auto &ssid : scannedSSID) {
            if (ssid == SSID.at(i)) {
                printf("[WIFI DEBUG] Found matching network: %s\n", SSID.at(i).c_str());
                attempts++;
                
                printf("[WIFI DEBUG] Attempting to connect to %s...\n", SSID.at(i).c_str());
                int connect_result = this->connect(SSID.at(i), PASS.at(i));
                
                if (connect_result == 0) {
                    // Connection successful
                    printf("[WIFI DEBUG] Successfully connected to %s\n", SSID.at(i).c_str());
                    connected = true;
                    break;
                } else {
                    printf("[WIFI ERROR] Failed to connect to %s (error %d)\n", SSID.at(i).c_str(), connect_result);
                }
            }
        }
        
        // If we're connected, no need to try more networks
        if (connected) {
            break;
        }
    }
    
    if (attempts == 0) {
        printf("[WIFI ERROR] No known WiFi networks found in scan results\n");
        return 1;
    }
    
    if (!connected) {
        printf("[WIFI ERROR] Failed to connect to any of the %d available networks\n", attempts);
        return 1;
    }
    
    printf("[WIFI DEBUG] Successfully connected to WiFi\n");
    return 0;
}

int myWIFI::connect(std::string ssid, std::string pass) {
    printf("[WIFI DEBUG] Starting connect method for SSID: %s\n", ssid.c_str());
    
    // Check if we're already trying to connect
    if(this->trying_to_connect) {
        printf("[WIFI ERROR] Connection attempt already in progress\n");
        // Try to reset the state to allow a new connection attempt
        printf("[WIFI DEBUG] Forcing reset of connection state\n");
        this->trying_to_connect = false;
    }
    
    // Check for empty SSID or password
    if (ssid.empty() || pass.empty()) {
        printf("[WIFI ERROR] SSID or password is empty\n");
        this->connected = CYW43_LINK_DOWN;
        return 1;
    }
    
    this->trying_to_connect = true;
    
    // Enable station mode
    printf("[WIFI DEBUG] Enabling STA mode...\n");
    cyw43_arch_enable_sta_mode();
    printf("[WIFI DEBUG] Attempting connection to %s...\n", ssid.c_str());
    
    // Set timeout to 8 seconds instead of 10 - shorter is better to avoid hanging
    int connect_timeout = 8000;
    printf("[WIFI DEBUG] Connection timeout set to %d ms\n", connect_timeout);
    
    // Make sure background tasks run
    cyw43_arch_poll();
    
    // Save the start time to implement our own timeout
    absolute_time_t connect_start_time = get_absolute_time();
    
    // Try to connect
    int connect_result = cyw43_arch_wifi_connect_timeout_ms(ssid.c_str(), pass.c_str(), CYW43_AUTH_WPA2_AES_PSK, connect_timeout);
    
    // Check if the connection took too long
    int elapsed_ms = to_ms_since_boot(get_absolute_time()) - to_ms_since_boot(connect_start_time);
    if (elapsed_ms > (connect_timeout + 2000)) {  // If it took more than 2 seconds over the timeout
        printf("[WIFI WARNING] Connection attempt took %d ms, which is more than expected timeout of %d ms\n", 
               elapsed_ms, connect_timeout);
    }
    
    // Check connection result
    if (connect_result) {
        this->connected = CYW43_LINK_DOWN;
        this->trying_to_connect = false;
        
        // Provide specific error messages
        if (connect_result == -1) {
            printf("[WIFI ERROR] Connection failed: Timeout after %d ms\n", connect_timeout);
        } else if (connect_result == -2) {
            printf("[WIFI ERROR] Connection failed: Authentication error (check password)\n");
        } else if (connect_result == -3) {
            printf("[WIFI ERROR] Connection failed: Network not found\n");
        } else {
            printf("[WIFI ERROR] Connection failed with error code: %d\n", connect_result);
        }
        
        return 1;
    }
    
    // Update and check connection status
    printf("[WIFI DEBUG] Connection attempt completed, checking status...\n");
    this->connected = cyw43_tcpip_link_status(&cyw43_state, 0);
    this->trying_to_connect = false;
    
    if (this->connected == CYW43_LINK_UP) {
        printf("[WIFI DEBUG] -------- Connection Successful --------\n");
        printf("[WIFI DEBUG] Connected to: %s\n", ssid.c_str());
        printf("[WIFI DEBUG] IP address: %s\n", ip4addr_ntoa(netif_ip_addr4(netif_default)));
        printf("[WIFI DEBUG] Subnet mask: %s\n", ip4addr_ntoa(netif_ip_netmask4(netif_default)));
        printf("[WIFI DEBUG] Gateway: %s\n", ip4addr_ntoa(netif_ip_gw4(netif_default)));
        printf("[WIFI DEBUG] Hostname: %s\n", netif_get_hostname(netif_default));
        printf("[WIFI DEBUG] ---------------------------------------\n");
        return 0;
    } else {
        printf("[WIFI ERROR] Connection attempt completed but status is not connected (status=%d)\n", this->connected);
        return 1;
    }
}

// Add emergency reset method
void myWIFI::emergencyReset() {
    printf("[WIFI DEBUG] Performing emergency WiFi reset\n");
    
    // Reset the connection status flags
    this->trying_to_connect = false;
    this->connected = CYW43_LINK_DOWN;
    
    // Try to restart WiFi
    cyw43_arch_deinit();
    sleep_ms(1000);
    
    if (cyw43_arch_init_with_country(CYW43_COUNTRY_AUSTRIA)) {
        printf("[WIFI ERROR] Failed to reinitialize WiFi during emergency reset\n");
        return;
    }
    
    printf("[WIFI DEBUG] Emergency reset completed\n");
}

// Add disconnect method
void myWIFI::disconnect() {
    if (this->connected == CYW43_LINK_UP) {
        printf("[WIFI DEBUG] Disconnecting from WiFi\n");
        cyw43_wifi_leave(&cyw43_state, 0);
        this->connected = CYW43_LINK_DOWN;
        printf("[WIFI DEBUG] Disconnected from WiFi\n");
    } else {
        printf("[WIFI DEBUG] Not connected, no need to disconnect\n");
    }
}

static int scan_result(void *env, const cyw43_ev_scan_result_t *result) {
    if(result) {
        char buffer[33];
        std::sprintf(buffer, "%-32s", reinterpret_cast<const char *>(result->ssid));
        std::string ssidString(buffer);
        while (!ssidString.empty() && std::isspace(ssidString.back())) {
            ssidString.pop_back();
        }
        scannedSSID.push_back(ssidString);
    }

    /*if (result) {
        printf("ssid: %-32s rssi: %4d chan: %3d mac: %02x:%02x:%02x:%02x:%02x:%02x sec: %u\n",
               result->ssid, result->rssi, result->channel,
               result->bssid[0], result->bssid[1], result->bssid[2], result->bssid[3], result->bssid[4], result->bssid[5],
               result->auth_mode);
    }*/
    return 0;
}
