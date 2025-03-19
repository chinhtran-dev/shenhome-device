#ifndef NETWORK_HUB_H
#define NETWORK_HUB_H

#include <Arduino.h>
#include <ArduinoJson.h>

#define NETWORK_HUB_MAX_WIFI_NETWORKS 5  // Max WiFi networks
#define NETWORK_HUB_MAX_SSID_LENGTH 32   // WiFi SSID max length
#define NETWORK_HUB_MAX_PASS_LENGTH 32   // Reduced from 64 (WPA2-PSK min 8, max 63)
#define NETWORK_HUB_MAX_BROKER_LENGTH 64 // MQTT broker max length
#define NETWORK_HUB_JSON_BUFFER_SIZE 256 // JSON buffer size

class NetworkHub
{
private:
    struct WiFiInfo
    {
        char ssid[NETWORK_HUB_MAX_SSID_LENGTH];     // 32 bytes
        char password[NETWORK_HUB_MAX_PASS_LENGTH]; // 32 bytes (optimized)

        // Constructor to ensure null termination
        WiFiInfo()
        {
            ssid[0] = '\0';
            password[0] = '\0';
        }

        void toJson(JsonObject &json) const
        {
            json["ssid"] = ssid;
            json["password"] = password;
        }

        bool fromJson(const JsonObject &json)
        {
            const char *s = json["ssid"];
            const char *p = json["password"];
            if (!s || !p || strlen(s) == 0 || strlen(p) >= NETWORK_HUB_MAX_PASS_LENGTH)
            {
                return false; // Reject empty SSID or oversized password
            }
            strncpy(ssid, s, NETWORK_HUB_MAX_SSID_LENGTH);
            ssid[NETWORK_HUB_MAX_SSID_LENGTH - 1] = '\0';
            strncpy(password, p, NETWORK_HUB_MAX_PASS_LENGTH);
            password[NETWORK_HUB_MAX_PASS_LENGTH - 1] = '\0';
            return true;
        }
    } __attribute__((packed));

    WiFiInfo wifiInfos[NETWORK_HUB_MAX_WIFI_NETWORKS];
    uint8_t wifiCount;
    char mqttBroker[NETWORK_HUB_MAX_BROKER_LENGTH];
    uint16_t mqttPort;
    mutable char lastError[64];

    void setLastError(const char* error) const;

public:
    NetworkHub();

    // WiFi Configuration
    bool addWiFiNetwork(const char *ssid, const char *password);
    uint8_t getWiFiNetworkCount() const;
    bool getWiFiNetwork(uint8_t index, char *ssid, size_t ssidLen, char *password, size_t passLen) const;

    // MQTT Configuration
    bool setMQTTBroker(const char *broker, uint16_t port);
    const char *getMQTTBroker() const;
    uint16_t getMQTTPort() const;

    // Serialization
    String toJson() const;
    bool fromJson(const char *jsonStr);

    // EEPROM
    bool saveToEEPROM(int address) const;
    bool loadFromEEPROM(int address);

    // Error Handling
    const char *getLastError() const;
};

#endif