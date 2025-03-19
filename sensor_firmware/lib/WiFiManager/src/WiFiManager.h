#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include "NetworkHub.h"

#define WIFI_MANAGER_MAX_SSID_LENGTH 32    // Max SSID length
#define WIFI_MANAGER_MAX_PASS_LENGTH 64    // Max password length
#define WIFI_MANAGER_DEFAULT_AP_PASS "wifimanager"  // Default AP password
#define WIFI_MANAGER_DEFAULT_TIMEOUT 10000 // 10s timeout in ms

class WiFiManager {
public:
    WiFiManager();

    // Connection Management
    bool connect(const char* ssid, const char* password, unsigned long timeout = WIFI_MANAGER_DEFAULT_TIMEOUT);
    bool connectFromHub(NetworkHub& hub);  // Use NetworkHub credentials
    bool disconnect();
    bool isConnected() const;

    // Access Point Mode
    bool startAP(const char* apName, const char* apPassword = WIFI_MANAGER_DEFAULT_AP_PASS, uint8_t channel = 1);
    bool stopAP();

    // Status and Info
    const char* getLastError() const;
    const char* getLocalIP() const;  // Returns IP as string

private:
    char lastError[64];
    bool apActive;

    bool waitForConnection(unsigned long timeout);
    void setLastError(const char* error);
};

#endif