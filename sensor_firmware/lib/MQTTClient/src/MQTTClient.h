#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WiFiClient.h>
#include <functional>

#define MQTT_MAX_TOPICS 10  // Configurable maximum number of topic subscriptions
#define MQTT_DEFAULT_BUFFER_SIZE 256  // Default JSON buffer size

typedef std::function<void(const JsonDocument&)> JsonCallback;
typedef std::function<void(const char* topic, const byte* payload, unsigned int length)> RawCallback;

class MQTTClient {
private:
    PubSubClient client;
    struct TopicHandler {
        char topic[64];  // Fixed-size topic string
        JsonCallback jsonCallback;
        RawCallback rawCallback;
        bool active;
    };
    TopicHandler topicHandlers[MQTT_MAX_TOPICS];
    uint8_t handlerCount;
    JsonDocument jsonDoc;
    char* mqttUser;  // Optional username
    char* mqttPass;  // Optional password
    bool reconnect();
    void messageReceived(char* topic, byte* payload, unsigned int length);

public:
    MQTTClient(WiFiClient* espClient);  // Changed to reference
    ~MQTTClient();  // Destructor to clean up

    bool setup(const char* host, 
               uint16_t port, 
               const char* user = nullptr, 
               const char* pass = nullptr);
    
    void loop();
    
    bool onJsonMessage(const char* topic, JsonCallback callback);
    bool onRawMessage(const char* topic, RawCallback callback);
    
    bool publishMessage(const char* topic, 
                       const char* payload, 
                       bool retained = false);
    
    bool isConnected();
    const char* getLastError() const;

private:
    char lastError[64];  // Error message buffer
};

#endif