#ifndef MQTT_PAYLOAD_H
#define MQTT_PAYLOAD_H

#include <Arduino.h>
#include <ArduinoJson.h>

class MQTTPayload {
private:
  String mac;
  String type;
  JsonObject data;

public:
  MQTTPayload(const String& macAddress, const String& deviceType, JsonDocument& doc);
  String toJsonString() const;
  static MQTTPayload deserializeEntity(const JsonDocument& doc);

  String getMac() const { return mac; }
  String getType() const { return type; }
  JsonObject getData() const { return data; }
};

#endif