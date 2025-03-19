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

MQTTPayload MQTTPayload::deserializeEntity(const JsonDocument &doc)
{
    String mac = "";
    String type = "";
    JsonDocument data;

    if (doc["mac"].is<String>())
    {
        mac = doc["mac"].as<String>();
    }

    if (doc["type"].is<String>())
    {
        type = doc["type"].as<String>();
    }

    if (doc["data"].is<String>() && !doc["data"].isNull())
    {
        deserializeJson(data, doc["data"].as<String>());
    }

    return MQTTPayload(mac, type, data);
}