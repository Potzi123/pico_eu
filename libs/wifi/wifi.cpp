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
    cyw43_arch_enable_sta_mode();
    printf("STA Mode active");
    cyw43_wifi_scan_options_t scan_options = {0};
    if(cyw43_wifi_scan(&cyw43_state, &scan_options, nullptr, scan_result)) {
        printf("Could not start WiFi Scan!");
        return 1;
    }
    printf("Started WiFi Scan sucessfully!");
    while(cyw43_wifi_scan_active(&cyw43_state));
    std::sort(scannedSSID.begin(), scannedSSID.end());
    scannedSSID.erase(std::unique(scannedSSID.begin(), scannedSSID.end()), scannedSSID.end());
    scannedSSID.erase(std::remove(scannedSSID.begin(), scannedSSID.end(), ""), scannedSSID.end());
    printf("--------BEGIN WiFi Scan Result--------");
    for(const auto &ssid : scannedSSID) {
        printf("Found: %s", ssid.c_str());
    }
    printf("--------END WiFi Scan Result--------");
    for (int i = 0; i < SSID.size(); i++) {
        for(const auto &ssid : scannedSSID) {
            if (ssid == SSID.at(i)) {
                printf("Trying to connect to %s with %s...", SSID.at(i).c_str(), PASS.at(i).c_str());
                if (this->connect(SSID.at(i), PASS.at(i))) {
                    printf("Failed to connect");
                    return 1;
                }
                break;
            }
        }
    }
    return 0;
}

int myWIFI::connect(std::string ssid, std::string pass) {
    if(!this->trying_to_connect) {
        this->trying_to_connect = true;
        if (ssid.empty() || pass.empty()) {
            this->connected = CYW43_LINK_DOWN;
            this->trying_to_connect = false;
            printf("SSID and/or Password not specified!");
            return 1;
        }
        else {
            cyw43_arch_enable_sta_mode();
            if (cyw43_arch_wifi_connect_timeout_ms(ssid.c_str(), pass.c_str(), CYW43_AUTH_WPA2_AES_PSK, 6000)) {
                this->connected = CYW43_LINK_DOWN;
                this->trying_to_connect = false;
                printf("Failed to connect");
                return 1;
            }
        }


        this->connected = cyw43_tcpip_link_status(&cyw43_state, 0);
        this->trying_to_connect = false;
        printf("--------BEGIN WiFi Connect Result--------");
        printf("Connected to:\t%s", ssid.c_str());
        printf("IP address:\t\t%s", ip4addr_ntoa(netif_ip_addr4(netif_default)));
        printf("Mask:\t\t\t%s", ip4addr_ntoa(netif_ip_netmask4(netif_default)));
        printf("Gateway:\t\t%s", ip4addr_ntoa(netif_ip_gw4(netif_default)));
        printf("Host Name:\t%s", netif_get_hostname(netif_default));
        printf("--------END WiFi Connect Result--------");
    }
    return 0;
}

/*void myWIFI::disconnect() {
    cyw43_arch_deinit();
    printf("WiFi deinitialised");
}*/

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
