#include "main.h"

#define SSID_TEST "Emyeucogiao"
#define PASSWORD_TEST "hoicoemdi1227"
#define MQTT_BROKER "192.168.2.124"

NetworkHub hub;
WiFiManager wifi;
UdpCom udp;
WiFiClient espClient;
MQTTClient mqttClient(&espClient);

bool pairingMode = false;
char macAddress[18];
unsigned long lastSensorTime = 0;
unsigned long lastHeartbeatTime = 0;
unsigned long lastBlinkTime = 0;
unsigned long buttonPressTime = 0;
bool buttonPressed = false;
unsigned long udpTimeout = 0;
unsigned long lastBroadcastTime = 0;

void setup() {
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // LED off
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);

  snprintf(macAddress, sizeof(macAddress), "%02x:%02x:%02x:%02x:%02x:%02x",
           WiFi.macAddress()[0], WiFi.macAddress()[1], WiFi.macAddress()[2],
           WiFi.macAddress()[3], WiFi.macAddress()[4], WiFi.macAddress()[5]);

  loadCredentialsFromEEPROM();
  connectWiFi();
  setupMQTT();

  if (!wifi.isConnected() || !mqttClient.isConnected()) {
    udp.setPacketCallback(handleUDPPacket);
    startPairingMode();
  }
}

void loop() {
  if (wifi.isConnected() && !pairingMode) {
    mqttClient.loop();

    if (String(mqttClient.getLastError()) != "No error") {
      Serial.println("MQTT error: " + String(mqttClient.getLastError()));
    }

    if (millis() - lastSensorTime > SENSOR_INTERVAL) {
      sendSensorData();
      lastSensorTime = millis();
    }

    if (millis() - lastHeartbeatTime > HEARTBEAT_INTERVAL) {
      sendHeartbeat();
      lastHeartbeatTime = millis();
    }
  }

  if (pairingMode) {
    if (millis() - lastBroadcastTime > BROADCAST_INTERVAL) {
      sendUdpBroadcast();
      lastBroadcastTime = millis();
    }
    if (udpTimeout && millis() < udpTimeout) {
      udp.receivePackets();
    } else if (udpTimeout) {
      udpTimeout = millis() + UDP_TIMEOUT_MS;
    }
    blinkLED();
  }

  checkResetButton();
  delay(10);
}

void startPairingMode() {
  pairingMode = true;
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (!WiFi.beginSmartConfig()) {
    Serial.println("Failed to start SmartConfig");
    return;
  }

  while (!WiFi.smartConfigDone()) {
    delay(500);
    blinkLED();
  }

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    hub.addWiFiNetwork(WiFi.SSID().c_str(), WiFi.psk().c_str());
    hub.saveToEEPROM(EEPROM_ADDRESS);
    if (udp.begin()) {
      sendUdpBroadcast();
      lastBroadcastTime = millis();
      udpTimeout = millis() + UDP_TIMEOUT_MS;
    } else {
      Serial.println("Failed to start UDP: " + String(udp.getLastError()));
      startPairingMode();
    }
  } else {
    Serial.println("WiFi connection failed, restarting pairing mode...");
    startPairingMode();
  }
}

void sendUdpBroadcast() {
  JsonDocument doc;
  doc["macAddress"] = macAddress;
  doc["ip"] = WiFi.localIP().toString();
  String payload;
  serializeJson(doc, payload);

  if (!udp.sendPacket(BROADCAST_IP, UDP_PAIRING_PORT, payload.c_str(), payload.length())) {
    Serial.println("Failed to send UDP broadcast: " + String(udp.getLastError()));
  }
}

void handleUDPPacket(const char *data, size_t length, IPAddress remoteIP, uint16_t remotePort) {
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, data, length);
  if (error) {
    Serial.println("Failed to parse UDP packet");
    return;
  }

  const char *broker = doc["broker"];
  if (!broker) {
    Serial.println("Invalid UDP packet");
    return;
  }

  hub.setMQTTBroker(broker, MQTT_PORT);

  if (setupMQTT() && mqttClient.isConnected()) {
    sendMacAddressViaMQTT();
    stopPairingMode();
    hub.saveToEEPROM(EEPROM_ADDRESS);
  } else {
    udpTimeout = millis() + UDP_TIMEOUT_MS;
  }
}

void stopPairingMode() {
  if (pairingMode) {
    udp.stop();
    pairingMode = false;
  }
}

void sendMacAddressViaMQTT() {
  if (mqttClient.isConnected()) {
    JsonDocument doc;
    MQTTPayload payload(macAddress, "Sensor", doc);
    String json = payload.toJsonString();

    if (!mqttClient.publishMessage("gateway/register", json.c_str(), true)) {
      Serial.println("Failed to send MAC address: " + String(mqttClient.getLastError()));
    }
  }
}

void sendHeartbeat() {
  if (mqttClient.isConnected()) {
    JsonDocument doc;
    JsonObject data = doc["data"].to<JsonObject>();
    data["status"] = ENABLED;
    MQTTPayload payload(macAddress, "Sensor", doc);
    String json = payload.toJsonString();

    if (!mqttClient.publishMessage("gateway/heartbeat", json.c_str(), true)) {
      Serial.println("Failed to send heartbeat: " + String(mqttClient.getLastError()));
    }
  }
}

void sendSensorData() {
  if (mqttClient.isConnected()) {
    JsonDocument doc;
    JsonObject data = doc["data"].to<JsonObject>();
    data["light"] = analogRead(A0);  //Read data sensor
    MQTTPayload payload(macAddress, "Sensor", doc);
    String json = payload.toJsonString();

    if (!mqttClient.publishMessage("gateway/data", json.c_str(), true)) {
      Serial.println("Failed to send sensor data: " + String(mqttClient.getLastError()));
    }
  }
}

void loadCredentialsFromEEPROM() {
  if (!hub.loadFromEEPROM(EEPROM_ADDRESS)) {
    Serial.println("Failed to load credentials: " + String(hub.getLastError()));
  }
}

void connectWiFi() {
  if (!wifi.connectFromHub(hub)) {
    Serial.println("WiFi connection failed: " + String(wifi.getLastError()));
  }
}

bool setupMQTT() {
  if (wifi.isConnected() && mqttClient.setup(hub.getMQTTBroker(), hub.getMQTTPort())) {
    return true;
  }
  Serial.println("MQTT setup failed: " + String(mqttClient.getLastError()));
  return false;
}

void blinkLED() {
  if (millis() - lastBlinkTime > BLINK_INTERVAL) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    lastBlinkTime = millis();
  }
}

void checkResetButton() {
  bool currentState = digitalRead(RESET_BUTTON_PIN);

  if (currentState == LOW && !buttonPressed) {
    buttonPressed = true;
    buttonPressTime = millis();
  } else if (currentState == HIGH && buttonPressed) {
    buttonPressed = false;
  }

  if (buttonPressed && (millis() - buttonPressTime >= RESET_HOLD_TIME)) {
    Serial.println("Reset triggered: Clearing EEPROM and restarting...");
    clearEEPROMAndRestart();
  }
}

void clearEEPROMAndRestart() {
  EEPROM.begin(512);
  for (int i = 0; i < 512; i++) {
    EEPROM.write(i, 0);
  }
  EEPROM.commit();
  EEPROM.end();
  ESP.restart();
}