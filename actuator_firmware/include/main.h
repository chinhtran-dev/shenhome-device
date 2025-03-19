#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <WiFiManager.h>
#include <NetworkHub.h>
#include <MQTTClient.h>
#include "Config.h"
#include <UdpCom.h>
#include <EEPROM.h>
#include "MQTTPayload.h"

void startPairingMode();
void sendUdpBroadcast();
void handleUDPPacket(const char* data, size_t length, IPAddress remoteIP, uint16_t remotePort);
void stopPairingMode();
void sendMacAddressViaMQTT();
void sendHeartbeat();
void loadCredentialsFromEEPROM();
void connectWiFi();
bool setupMQTT();
void blinkLED();
void checkResetButton();
void clearEEPROMAndRestart();
void checkToggleButton();

#endif