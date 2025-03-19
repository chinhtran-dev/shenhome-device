#include "MqttClient.h"
#include <string.h>

// Constructor
MQTTClient::MQTTClient(WiFiClient* espClient) 
    : client(*espClient), handlerCount(0), mqttUser(nullptr), mqttPass(nullptr) {
    for (uint8_t i = 0; i < MQTT_MAX_TOPICS; i++) {
        topicHandlers[i].active = false;
        topicHandlers[i].jsonCallback = nullptr;
        topicHandlers[i].rawCallback = nullptr;
        memset(topicHandlers[i].topic, 0, sizeof(topicHandlers[i].topic));
    }
    strcpy(lastError, "No error");
    client.setCallback([this](char* topic, byte* payload, unsigned int length) {
        this->messageReceived(topic, payload, length);
    });
}

// Destructor
MQTTClient::~MQTTClient() {
    client.disconnect();
    mqttUser = nullptr;
    mqttPass = nullptr;
}

bool MQTTClient::setup(const char* host, 
                      uint16_t port, 
                      const char* user, 
                      const char* pass) {
    client.setServer(host, port);
    mqttUser = const_cast<char*>(user);  // Store for reconnect
    mqttPass = const_cast<char*>(pass);
    
    if (!reconnect()) {
        strcpy(lastError, "Initial connection failed");
        return false;
    }

    return true;
}

void MQTTClient::loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();
}

bool MQTTClient::onJsonMessage(const char* topic, JsonCallback callback) {
    if (handlerCount >= MQTT_MAX_TOPICS) {
        strcpy(lastError, "Max topics reached");
        return false;
    }
    
    // Find or add handler
    for (uint8_t i = 0; i < MQTT_MAX_TOPICS; i++) {
        if (!topicHandlers[i].active) {
            strncpy(topicHandlers[i].topic, topic, sizeof(topicHandlers[i].topic));
            topicHandlers[i].topic[sizeof(topicHandlers[i].topic) - 1] = '\0';            
            topicHandlers[i].jsonCallback = callback;
            topicHandlers[i].rawCallback = nullptr;
            topicHandlers[i].active = true;
            handlerCount++;

            Serial.println(String(topicHandlers[i].topic));
            if (client.connected()) {
                if(client.subscribe(topic)) {
                    Serial.print("Subscribing to topic: ");
                    Serial.println(topic);
                };
            }
            return true;
        }
    }
    strcpy(lastError, "No free handler slots");  // Shouldn't happen due to count check
    return false;
}

bool MQTTClient::onRawMessage(const char* topic, RawCallback callback) {
    if (handlerCount >= MQTT_MAX_TOPICS) {
        strcpy(lastError, "Max topics reached");
        return false;
    }
    
    for (uint8_t i = 0; i < MQTT_MAX_TOPICS; i++) {
        if (!topicHandlers[i].active) {
            strncpy(topicHandlers[i].topic, topic, sizeof(topicHandlers[i].topic));
            topicHandlers[i].rawCallback = callback;
            topicHandlers[i].jsonCallback = nullptr;
            topicHandlers[i].active = true;
            handlerCount++;
            if (client.connected()) {
                client.subscribe(topic);
            }
            return true;
        }
    }
    strcpy(lastError, "No free handler slots");
    return false;
}

bool MQTTClient::publishMessage(const char* topic, 
                               const char* payload, 
                               bool retained) {
    if (!client.connected()) {
        strcpy(lastError, "Not connected");
        return false;
    }
    
    bool success = client.publish(topic, payload, retained);
    if (!success) {
        strcpy(lastError, "Publish failed");
    }
    return success;
}

bool MQTTClient::isConnected() {
    return client.connected();
}

const char* MQTTClient::getLastError() const {
    return lastError;
}

bool MQTTClient::reconnect() {
    if (client.connected()) {
        return true;
    }

    // Attempt to connect with optional credentials
    const char* clientId = "ESP8266Client-";  // Could be randomized or parameterized
    bool connected = (mqttUser && mqttPass) 
        ? client.connect(clientId, mqttUser, mqttPass)
        : client.connect(clientId);

    if (!connected) {
        strcpy(lastError, "Reconnection failed");
        return false;
    }

    // Resubscribe to all active topics
    for (uint8_t i = 0; i < MQTT_MAX_TOPICS; i++) {
        if (topicHandlers[i].active) {
            client.subscribe(topicHandlers[i].topic);
        }
    }
    strcpy(lastError, "No error");
    return true;
}

void MQTTClient::messageReceived(char* topic, byte* payload, unsigned int length) {
    // Null-terminate payload for safety (PubSubClient doesn't guarantee it)
    byte safePayload[length + 1];
    memcpy(safePayload, payload, length);
    safePayload[length] = '\0';

    // Find matching handler
    for (uint8_t i = 0; i < MQTT_MAX_TOPICS; i++) {
        Serial.println("Checking topic: " + String(topicHandlers[i].topic));
        Serial.println("Checking against: " + String(topic));
        if (strcmp(topicHandlers[i].topic, topic) == 0) {
            Serial.println("Found matching handler");
            // Call raw callback if set
            if (topicHandlers[i].rawCallback) {
                topicHandlers[i].rawCallback(topic, safePayload, length);
            }
            // Call JSON callback if set
            if (topicHandlers[i].jsonCallback) {
                Serial.println("Calling JSON callback");
                DeserializationError error = deserializeJson(jsonDoc, safePayload);
                if (!error) {
                    topicHandlers[i].jsonCallback(jsonDoc);
                } else {
                    snprintf(lastError, sizeof(lastError), "JSON parse error: %s", error.c_str());
                }
            }
            break;  // Assume one handler per topic
        }
    }
}