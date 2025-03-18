#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <WiFiManager.h>
#include <NetworkHub.h>
#include <MQTTClient.h>
#include "Config.h"
#include <UdpCom.h>

void handleUDPPacket(const char* data, size_t length, IPAddress remoteIP, uint16_t remotePort);
void handleJsonMessage(const JsonDocument& doc);
void handleRawMessage(const char* topic, const byte* payload, unsigned int length);
void loadCredentialsFromEEPROM();
void connectWiFi();
void setupMQTT();
void startUDPPairingMode();

#endif