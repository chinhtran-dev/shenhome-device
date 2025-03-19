#include "WiFiManager.h"

WiFiManager::WiFiManager() : apActive(false) {
    strcpy(lastError, "No error");
    WiFi.mode(WIFI_OFF);  // Start with WiFi off
}

bool WiFiManager::connect(const char* ssid, const char* password, unsigned long timeout) {
    if (!ssid || !password || strlen(ssid) == 0 || strlen(ssid) > WIFI_MANAGER_MAX_SSID_LENGTH || 
        strlen(password) > WIFI_MANAGER_MAX_PASS_LENGTH) {
        setLastError("Invalid SSID or password");
        return false;
    }

    if (apActive) {
        stopAP();
    }

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    if (waitForConnection(timeout)) {
        strcpy(lastError, "No error");
        return true;
    }

    WiFi.disconnect();
    setLastError("Connection timeout");
    return false;
}

bool WiFiManager::connectFromHub(NetworkHub& hub) {
    uint8_t wifiCount = hub.getWiFiNetworkCount();
    if (wifiCount == 0) {
        setLastError("No WiFi networks in hub");
        return false;
    }

    char ssid[WIFI_MANAGER_MAX_SSID_LENGTH];
    char pass[WIFI_MANAGER_MAX_PASS_LENGTH];
    for (uint8_t i = 0; i < wifiCount; i++) {
        if (!hub.getWiFiNetwork(i, ssid, sizeof(ssid), pass, sizeof(pass))) {
            continue;
        }

        if (connect(ssid, pass)) {
            return true;
        }
    }

    setLastError("Failed to connect to any network from hub");
    return false;
}

bool WiFiManager::disconnect() {
    if (!isConnected()) {
        strcpy(lastError, "Not connected");
        return false;
    }

    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    strcpy(lastError, "No error");
    return true;
}

bool WiFiManager::isConnected() const {
    return WiFi.status() == WL_CONNECTED && !apActive;
}

bool WiFiManager::startAP(const char* apName, const char* apPassword, uint8_t channel) {
    if (!apName || strlen(apName) == 0 || strlen(apName) > WIFI_MANAGER_MAX_SSID_LENGTH) {
        setLastError("Invalid AP name");
        return false;
    }

    if (isConnected()) {
        disconnect();
    }

    WiFi.mode(WIFI_AP);
    bool success = WiFi.softAP(apName, apPassword, channel);
    if (!success) {
        setLastError("Failed to start AP");
        WiFi.mode(WIFI_OFF);
        return false;
    }

    apActive = true;
    strcpy(lastError, "No error");
    return true;
}

bool WiFiManager::stopAP() {
    if (!apActive) {
        strcpy(lastError, "AP not active");
        return false;
    }

    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    apActive = false;
    strcpy(lastError, "No error");
    return true;
}

const char* WiFiManager::getLastError() const {
    return lastError;
}

const char* WiFiManager::getLocalIP() const {
    if (isConnected()) {
        return WiFi.localIP().toString().c_str();
    } else if (apActive) {
        return WiFi.softAPIP().toString().c_str();
    }
    return "0.0.0.0";
}

bool WiFiManager::waitForConnection(unsigned long timeout) {
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - start >= timeout) {
            return false;
        }
        delay(100);
    }
    return true;
}

void WiFiManager::setLastError(const char* error) {
    strncpy(lastError, error, sizeof(lastError));
    lastError[sizeof(lastError) - 1] = '\0';
}