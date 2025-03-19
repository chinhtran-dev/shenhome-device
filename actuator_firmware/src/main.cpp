#include <Arduino.h>
#include <ArduinoJson.h>
#include "main.h"

#define SSID_TEST "Emyeucogiao"
#define PASSWORD_TEST "hoicoemdi1227"
#define MQTT_BROKER "192.168.2.124"
#define MQTT_PORT 1883

NetworkHub hub;
WiFiManager wifi;
UdpCom udp;
WiFiClient espClient;
MQTTClient mqttClient(&espClient);

bool pairingMode = false;
char macAddress[18];
unsigned long lastHeartbeatTime = 0;
unsigned long lastBlinkTime = 0;
unsigned long buttonPressTime = 0;
bool buttonPressed = false;
unsigned long lastToggleTime = 0;
bool lastToggleState = HIGH;
bool ledState = false;
unsigned long udpTimeout = 0;
unsigned long lastBroadcastTime = 0;

void setup()
{
  Serial.begin(115200);

  // Initialize LED pin
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // Turn off LED

  // Initialize reset button
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);

  // Initialize toggle button
  pinMode(TOGGLE_BUTTON_PIN, INPUT_PULLUP);

  // Get MAC address for pairing mode
  snprintf(macAddress, sizeof(macAddress), "%02x:%02x:%02x:%02x:%02x:%02x",
           WiFi.macAddress()[0], WiFi.macAddress()[1], WiFi.macAddress()[2],
           WiFi.macAddress()[3], WiFi.macAddress()[4], WiFi.macAddress()[5]);

  // Load credentials and attempt connections
  loadCredentialsFromEEPROM();
  connectWiFi();
  setupMQTT();

  // if (wifi.connect(SSID_TEST, PASSWORD_TEST)) {
  //   Serial.println("WiFi connected: " + String(wifi.getLocalIP()));
  //   mqttClient.setup(MQTT_BROKER, MQTT_PORT);
  //   String commandTopic = "device/" + String(macAddress) + "/command";
  //   mqttClient.onJsonMessage(commandTopic.c_str(), handleCommandMessage);
  //   Serial.println("MQTT connected: " + String(mqttClient.isConnected()));
  // }

  // If not connected, start UDP pairing mode
  if (!wifi.isConnected() || !mqttClient.isConnected())
  {
    udp.setPacketCallback(handleUDPPacket);
    startPairingMode();
  }
}

void loop()
{
  // Maintain MQTT connection
  if (wifi.isConnected() && !pairingMode)
  {
    mqttClient.loop();

    if (String(mqttClient.getLastError()) != "No error") {
      Serial.println("MQTT error " + String(mqttClient.getLastError()));
    }

    if (millis() - lastHeartbeatTime > HEARTBEAT_INTERVAL)
    {
      sendHeartbeat();
      lastHeartbeatTime = millis();
    }
    digitalWrite(LED_BUILTIN, ledState ? LOW : HIGH); // Turn on/off LED
  }

  // Handle UDP pairing mode
  if (pairingMode)
  {
    if (millis() - lastBroadcastTime > BROADCAST_INTERVAL)
    {
      sendUdpBroadcast();
      lastBroadcastTime = millis();
    }
    if (udpTimeout && millis() < udpTimeout)
    {
      udp.receivePackets();
    }
    else if (udpTimeout)
    {
      Serial.println("UDP timeout, retrying...");
      udpTimeout = millis() + UDP_TIMEOUT_MS;
    }
    blinkLED();
  }
  else
  {
    digitalWrite(LED_BUILTIN, HIGH); // Turn off LED
  }

  checkResetButton();

  checkToggleButton();

  delay(10);
}

void startPairingMode()
{
  Serial.println("Starting pairing mode...");
  pairingMode = true;

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (!WiFi.beginSmartConfig())
  {
    Serial.println("Failed to start SmartConfig");
    return;
  }

  Serial.println("Waiting for SmartConfig data...");
  while (!WiFi.smartConfigDone())
  {
    delay(500);
    blinkLED();
  }

  Serial.println("\nSmartConfig received!");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20)
  {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("\nWiFi connected: " + String(wifi.getLocalIP()));
    hub.addWiFiNetwork(WiFi.SSID().c_str(), WiFi.psk().c_str());
    hub.saveToEEPROM(EEPROM_ADDRESS);
    if (udp.begin())
    {
      sendUdpBroadcast();
      lastBroadcastTime = millis();
      udpTimeout = millis() + UDP_TIMEOUT_MS;
    }
    else
    {
      Serial.println("Failed to start UDP: " + String(udp.getLastError()));
      startPairingMode();
    }
  }
  else
  {
    Serial.println("\nWiFi connection failed, restarting...");
    startPairingMode();
  }
}

void sendUdpBroadcast()
{
  JsonDocument doc;
  doc["macAddress"] = macAddress;
  doc["ip"] = WiFi.localIP().toString();
  String payload;
  serializeJson(doc, payload);

  if (udp.sendPacket("255.255.255.255", UDP_DEFAULT_PORT, payload.c_str(), payload.length()))
  {
    Serial.println("Sent UDP broadcast: " + payload);
  }
  else
  {
    Serial.println("Failed to send UDP broadcast: " + String(udp.getLastError()));
  }
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

  const char *broker = doc["broker"];
  if (!broker)
  {
    Serial.println("Invalid UDP packet");
    return;
  }

  hub.setMQTTBroker(broker, MQTT_PORT);

  if (setupMQTT() && mqttClient.isConnected())
  {
    sendMacAddressViaMQTT();
    stopPairingMode();
    hub.saveToEEPROM(EEPROM_ADDRESS);
  }
  else
  {
    Serial.println("MQTT setup failed, retrying...");
    udpTimeout = millis() + UDP_TIMEOUT_MS;
  }
}

void stopPairingMode()
{
  if (pairingMode)
  {
    udp.stop(); // Stop UDP server
    pairingMode = false;
    Serial.println("Pairing mode stopped");
  }
}

void sendMacAddressViaMQTT()
{
  if (mqttClient.isConnected())
  {
    JsonDocument doc;
    MQTTPayload payload(macAddress, "Actuator", doc);

    String json = payload.toJsonString();

    if (mqttClient.publishMessage("gateway/register", json.c_str(), true))
    {
      Serial.println("MAC address sent: " + json);
    }
    else
    {
      Serial.println("Failed to send MAC address: " + String(mqttClient.getLastError()));
    }
  }
}

void sendHeartbeat()
{
  if (mqttClient.isConnected())
  {
    JsonDocument doc;
    JsonObject data = doc["data"].add<JsonObject>();
    data["status"] = ENABLED;

    MQTTPayload payload(macAddress, "Actuator", doc);
    String json = payload.toJsonString();
    if (mqttClient.publishMessage("gateway/heartbeat", json.c_str(), true))
    {
      Serial.println("Heartbeat sent: " + json);
    }
    else
    {
      Serial.println("Failed to send heartbeat: " + String(mqttClient.getLastError()));
    }
  }
}

void handleCommandMessage(const JsonDocument &doc)
{
  Serial.println("Received JSON message:");
  serializeJsonPretty(doc, Serial);
  String command = doc["status"];

  if (command == ENABLED)
  {
    ledState = true;
  }
  else if (command == DISABLED)
  {
    ledState = false;
  }
  else
  {
    Serial.println("Invalid command: " + command);
  }
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

bool setupMQTT()
{
  if (wifi.isConnected() && mqttClient.setup(hub.getMQTTBroker(), hub.getMQTTPort()))
  {
    Serial.println("MQTT setup successful for " + String(hub.getMQTTBroker()));
    // Register callbacks
    String commandTopic = "device/" + String(macAddress) + "/command";
    mqttClient.onJsonMessage(commandTopic.c_str(), handleCommandMessage);

    Serial.println("MQTT setup failed: " + String(mqttClient.getLastError()));
    return true;
  }
  Serial.println("MQTT setup failed: " + String(mqttClient.getLastError()));
  return false;
}

void blinkLED()
{
  if (millis() - lastBlinkTime > BLINK_INTERVAL)
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    lastBlinkTime = millis();
  }
}

void checkResetButton()
{
  bool currentState = digitalRead(RESET_BUTTON_PIN);

  if (currentState == LOW && !buttonPressed)
  {
    // Button pressed (LOW due to pull-up)
    buttonPressed = true;
    buttonPressTime = millis();
    Serial.println("Reset button pressed");
  }
  else if (currentState == HIGH && buttonPressed)
  {
    // Button released
    buttonPressed = false;
    Serial.println("Reset button released");
  }

  if (buttonPressed && (millis() - buttonPressTime >= RESET_HOLD_TIME))
  {
    Serial.println("Reset triggered: Clearing EEPROM and restarting...");
    clearEEPROMAndRestart();
  }
}

void clearEEPROMAndRestart()
{
  EEPROM.begin(512); // Match size from NetworkHub
  for (int i = 0; i < 512; i++)
  {
    EEPROM.write(i, 0); // Clear EEPROM
  }
  EEPROM.commit();
  EEPROM.end();
  Serial.println("EEPROM cleared");
  ESP.restart(); // Restart the ESP8266
}

void checkToggleButton()
{
  bool currentState = digitalRead(TOGGLE_BUTTON_PIN);

  // Check for button press with debounce
  if (currentState == LOW && lastToggleState == HIGH && (millis() - lastToggleTime > DEBOUNCE_TIME))
  {
    // Toggle LED state
    ledState = !ledState;
    Serial.print("Toggle button pressed, LED state: ");
    Serial.println(ledState ? "ON" : "OFF");

    // Send MQTT message
    if (mqttClient.isConnected())
    {
      JsonDocument doc;
      JsonObject data = doc["data"].to<JsonObject>();
      data["status"] = ledState ? ENABLED : DISABLED;

      MQTTPayload payload(macAddress, "Actuator", doc);

      String json = payload.toJsonString();
      if (mqttClient.publishMessage("gateway/data", json.c_str()))
      {
        Serial.println("Published to gateway/data: " + json);
      }
      else
      {
        Serial.println("Failed to publish: " + String(mqttClient.getLastError()));
      }
    }
    else
    {
      Serial.println("MQTT not connected, message not sent");
    }

    lastToggleTime = millis();
  }

  lastToggleState = currentState;
}
