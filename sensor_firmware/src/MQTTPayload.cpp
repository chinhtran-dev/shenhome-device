#include "MQTTPayload.h"

MQTTPayload::MQTTPayload(const String &macAddress, const String &deviceType, JsonDocument &doc)
    : mac(macAddress), type(deviceType), data(doc["data"].to<JsonObject>())
{
}

String MQTTPayload::toJsonString() const
{
    JsonDocument doc;
    doc["mac"] = mac;
    doc["type"] = type;
    if (!data.isNull())
    {
        doc["data"] = data;
    }
    String payload;
    serializeJson(doc, payload);
    return payload;
}