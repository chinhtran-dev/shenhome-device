#ifndef UDP_H
#define UDP_H

#include <Arduino.h>
#include <WiFiUdp.h>

#define UDP_MAX_PACKET_SIZE 512
#define UDP_DEFAULT_PORT 4210

class UdpCom
{
private:
    WiFiUDP udp;
    uint16_t port;
    
    char packetBuffer[UDP_MAX_PACKET_SIZE];
    char lastError[64];
    void (*onPacketCallback)(const char *data, size_t length, IPAddress remoteIP, uint16_t remotePort);

    void setLastError(const char *error);

public:
    UdpCom(uint16_t port = UDP_DEFAULT_PORT);
    ~UdpCom();

    bool begin();
    void stop();

    bool sendPacket(const char *ip, uint16_t port, const char *data, size_t length);
    void receivePackets();

    void setPacketCallback(void (*callback)(const char *data, size_t length, IPAddress remoteIP, uint16_t remotePort));

    const char *getLastError() const;

    const uint16_t getLocalPort() const;
};

#endif