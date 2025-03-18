#include <Arduino.h>
#include <ArduinoJson.h>
#include "main.h"

NetworkHub hub;
WiFiManager wifi;
UdpCom udp;
WiFiClient espClient;
MQTTClient mqttClient(espClient);

bool pairingMode = false;
char macAddress[18];
unsigned long lastHeartbeatTime = 0;
unsigned long lastBlinkTime = 0;

void setup()
{
  Serial.begin(115200);

  // Initialize LED pin
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // Turn off LED

  // Get MAC address for pairing mode
  snprintf(macAddress, sizeof(macAddress), "%02X:%02X:%02X:%02X:%02X:%02X",
           WiFi.macAddress()[0], WiFi.macAddress()[1], WiFi.macAddress()[2],
           WiFi.macAddress()[3], WiFi.macAddress()[4], WiFi.macAddress()[5]);

  // Load credentials and attempt connections
  loadCredentialsFromEEPROM();
  connectWiFi();
  setupMQTT();

  // If not connected, start UDP pairing mode
  if (!wifi.isConnected() || !mqttClient.isConnected())
  {
    startUDPPairingMode();
  }
}

void loop()
{
  // Maintain MQTT connection
  if (wifi.isConnected() && !pairingMode)
  {
    mqttClient.loop();

    if (millis() - lastHeartbeatTime > HEARTBEAT_INTERVAL)
    {
      sendHeartbeat();
      lastHeartbeatTime = millis();
    }
  }

  // Handle UDP pairing mode
  if (pairingMode)
  {
    udp.receivePackets();
    blinkLED();
  } else {
    digitalWrite(LED_BUILTIN, HIGH); // Turn off LED
  }

  delay(10);
}

void handleUDPPacket(const char *data, size_t length, IPAddress remoteIP, uint16_t remotePort)
{
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, data, length);
  if (error)
  {
    Serial.println("Failed to parse UDP packet");
    return;
  }

  const char *ssid = doc["ssid"];
  const char *pass = doc["password"];
  const char *ip = doc["ip"];

  if (!ssid || !pass || !ip)
  {
    Serial.println("Invalid UDP packet");
    return;
  }

  // Store credentials in NetworkHub
  hub.addWiFiNetwork(ssid, pass);
  hub.setMQTTBroker(ip, MQTT_PORT);

  if (wifi.connectFromHub(hub))
  {
    Serial.println("WiFi connected via UDP pairing: " + String(wifi.getLocalIP()));

    // Send MAC address back to sender
    String response = "{\"mac\":\"" + String(macAddress) + "\"}";
    udp.sendPacket(remoteIP.toString().c_str(), remotePort, response.c_str(), response.length());

    // Save credentials to EEPROM
    hub.saveToEEPROM(EEPROM_ADDRESS);
    pairingMode = false; // Exit pairing mode
  }
  else
  {
    Serial.println("WiFi connection failed: " + String(wifi.getLastError()));
  }
}

void handleCommandMessage(const JsonDocument &doc)
{
  Serial.println("Received JSON message:");
  serializeJsonPretty(doc, Serial);
  Serial.println();
}

void loadCredentialsFromEEPROM()
{
  if (hub.loadFromEEPROM(EEPROM_ADDRESS))
  {
    Serial.println("Credentials loaded from EEPROM");
  }
  else
  {
    Serial.println("Failed to load credentials: " + String(hub.getLastError()));
  }
}

void connectWiFi()
{
  if (wifi.connectFromHub(hub))
  {
    Serial.println("WiFi connected: " + String(wifi.getLocalIP()));
  }
  else
  {
    Serial.println("WiFi connection failed: " + String(wifi.getLastError()));
  }
}

void setupMQTT()
{
  if (wifi.isConnected())
  {
    if (mqttClient.setup(hub.getMQTTBroker(), hub.getMQTTPort()))
    {
      Serial.println("MQTT setup successful for " + String(hub.getMQTTBroker()));
      // Register callbacks
      String commandTopic = "device/" + String(macAddress) + "/command";
      mqttClient.onJsonMessage(commandTopic.c_str(), handleCommandMessage);
    }
    else
    {
      Serial.println("MQTT setup failed: " + String(mqttClient.getLastError()));
    }
  }
}

void startUDPPairingMode()
{
  Serial.println("Starting UDP pairing mode...");
  pairingMode = true;

  // Start UDP server
  if (wifi.startAP("ESP8266-Pairing", "pairing123"))
  {
    Serial.println("AP started: " + String(wifi.getLocalIP()));
    if (udp.begin())
    {
      Serial.println("UDP server started on port " + String(udp.getLocalPort()));
      udp.setPacketCallback(handleUDPPacket);
    }
    else
    {
      Serial.println("UDP failed: " + String(udp.getLastError()));
    }
  }
  else
  {
    Serial.println("AP failed: " + String(wifi.getLastError()));
  }
}

void sendHeartbeat()
{
  if (mqttClient.isConnected())
  {
    JsonDocument doc;
    doc["mac"] = macAddress;
    doc["type"] = "Actuator";
    JsonObject data = doc["data"].add<JsonObject>();
    data["status"] = ENABLED;

    String payload;
    serializeJson(doc, payload);
    if (mqttClient.publishMessage("heartbeat", payload.c_str(), true)) {
      Serial.println("Heartbeat sent: " + payload);
    } else {
      Serial.println("Failed to send heartbeat: " + String(mqttClient.getLastError()));
    }
  }
}

void blinkLED()
{
  if (millis() - lastBlinkTime > BLINK_INTERVAL)
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    lastBlinkTime = millis();
  }
}