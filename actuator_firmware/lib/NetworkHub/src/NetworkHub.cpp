#include "NetworkHub.h"
#include <EEPROM.h>

NetworkHub::NetworkHub() : wifiCount(0), mqttPort(0) {
    for (uint8_t i = 0; i < NETWORK_HUB_MAX_WIFI_NETWORKS; i++) {
        wifiInfos[i] = WiFiInfo();  // Explicitly call constructor
    }
    mqttBroker[0] = '\0';
    strcpy(lastError, "No error");
}

bool NetworkHub::addWiFiNetwork(const char* ssid, const char* password) {
    if (wifiCount >= NETWORK_HUB_MAX_WIFI_NETWORKS) {
        setLastError("Max WiFi networks reached");
        return false;
    }
    if (!ssid || !password || strlen(ssid) == 0 || strlen(password) >= NETWORK_HUB_MAX_PASS_LENGTH) {
        setLastError("Invalid WiFi credentials");
        return false;
    }
    WiFiInfo& info = wifiInfos[wifiCount++];
    strncpy(info.ssid, ssid, NETWORK_HUB_MAX_SSID_LENGTH);
    info.ssid[NETWORK_HUB_MAX_SSID_LENGTH - 1] = '\0';
    strncpy(info.password, password, NETWORK_HUB_MAX_PASS_LENGTH);
    info.password[NETWORK_HUB_MAX_PASS_LENGTH - 1] = '\0';
    setLastError("No error");
    return true;
}

uint8_t NetworkHub::getWiFiNetworkCount() const {
    return wifiCount;
}

bool NetworkHub::getWiFiNetwork(uint8_t index, char* ssid, size_t ssidLen, char* password, size_t passLen) const {
    if (index >= wifiCount) {
        setLastError("Invalid network index");
        return false;
    }
    const WiFiInfo& info = wifiInfos[index];
    strncpy(ssid, info.ssid, ssidLen);
    ssid[ssidLen - 1] = '\0';
    strncpy(password, info.password, passLen);
    password[passLen - 1] = '\0';
    setLastError("No error");
    return true;
}

bool NetworkHub::setMQTTBroker(const char* broker, uint16_t port) {
    if (!broker || strlen(broker) == 0 || strlen(broker) >= NETWORK_HUB_MAX_BROKER_LENGTH) {
        setLastError("Invalid MQTT broker");
        return false;
    }
    strncpy(mqttBroker, broker, NETWORK_HUB_MAX_BROKER_LENGTH);
    mqttBroker[NETWORK_HUB_MAX_BROKER_LENGTH - 1] = '\0';
    mqttPort = port;
    setLastError("No error");
    return true;
}

const char* NetworkHub::getMQTTBroker() const {
    return mqttBroker;
}

uint16_t NetworkHub::getMQTTPort() const {
    return mqttPort;
}

String NetworkHub::toJson() const {
    JsonDocument doc;
    JsonArray wifiArray = doc["wifiNetworks"].to<JsonArray>();
    for (uint8_t i = 0; i < wifiCount; i++) {
        JsonObject wifiObj = wifiArray.add<JsonObject>();
        wifiInfos[i].toJson(wifiObj);
    }
    doc["mqttBroker"] = mqttBroker;
    doc["mqttPort"] = mqttPort;
    String jsonStr;
    serializeJson(doc, jsonStr);
    return jsonStr;
}

bool NetworkHub::fromJson(const char* jsonStr) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, jsonStr);
    if (error) {
        setLastError(error.c_str());
        return false;
    }

    // Reset current configuration
    wifiCount = 0;
    JsonArray wifiArray = doc["wifiNetworks"];
    for (JsonVariant v : wifiArray) {
        if (wifiCount >= NETWORK_HUB_MAX_WIFI_NETWORKS) {
            setLastError("Too many WiFi networks in JSON");
            return false;
        }
        if (!wifiInfos[wifiCount++].fromJson(v.as<JsonObject>())) {
            setLastError("Invalid WiFi info in JSON");
            return false;
        }
    }

    const char* broker = doc["mqttBroker"];
    if (!broker || !setMQTTBroker(broker, doc["mqttPort"])) {
        setLastError("Invalid MQTT info in JSON");
        return false;
    }
    setLastError("No error");
    return true;
}

bool NetworkHub::saveToEEPROM(int address) const {
    // Calculate total size needed
    int totalSize = sizeof(wifiCount) + wifiCount * sizeof(WiFiInfo) + sizeof(mqttBroker) + sizeof(mqttPort);
    if (address < 0 || static_cast<size_t>(address + totalSize) > EEPROM.length()) {
        setLastError("EEPROM overflow");
        return false;
    }

    EEPROM.begin(512);  // Typical ESP8266 EEPROM size, adjust if needed
    int pos = address;
    EEPROM.put(pos, wifiCount);
    pos += sizeof(wifiCount);
    for (uint8_t i = 0; i < wifiCount; i++) {
        EEPROM.put(pos, wifiInfos[i]);
        pos += sizeof(WiFiInfo);
    }
    EEPROM.put(pos, mqttBroker);
    pos += sizeof(mqttBroker);
    EEPROM.put(pos, mqttPort);
    EEPROM.commit();
    EEPROM.end();
    setLastError("No error");
    return true;
}

bool NetworkHub::loadFromEEPROM(int address) {
    if (address < 0 || address + sizeof(wifiCount) > EEPROM.length()) {
        setLastError("Invalid EEPROM address");
        return false;
    }

    EEPROM.begin(512);
    uint8_t count = 0;
    EEPROM.get(address, count);
    if (count > NETWORK_HUB_MAX_WIFI_NETWORKS) {
        setLastError("Invalid WiFi count in EEPROM");
        EEPROM.end();
        return false;
    }

    int totalSize = sizeof(wifiCount) + count * sizeof(WiFiInfo) + sizeof(mqttBroker) + sizeof(mqttPort);
    if (static_cast<size_t>(address + totalSize) > EEPROM.length()) {
        setLastError("EEPROM data too large");
        EEPROM.end();
        return false;
    }

    wifiCount = count;
    int pos = address + sizeof(wifiCount);
    for (uint8_t i = 0; i < wifiCount; i++) {
        EEPROM.get(pos, wifiInfos[i]);
        pos += sizeof(WiFiInfo);
    }
    EEPROM.get(pos, mqttBroker);
    pos += sizeof(mqttBroker);
    EEPROM.get(pos, mqttPort);
    EEPROM.end();
    setLastError("No error");
    return true;
}

void NetworkHub::setLastError(const char* error) const {
    strncpy(this->lastError, error, sizeof(lastError));
    this->lastError[sizeof(lastError) - 1] = '\0';
}

const char* NetworkHub::getLastError() const {
    return lastError;
}