#include "UdpCom.h"

UdpCom::UdpCom(uint16_t port) : port(port), onPacketCallback(nullptr) {
    packetBuffer[0] = '\0';
}

UdpCom::~UdpCom() {
    udp.stop();
}

bool UdpCom::begin() {
    if (!udp.begin(port)) {
        setLastError("UDP begin failed");
        return false;
    }

    setLastError("");
    return true;
}

bool UdpCom::sendPacket(const char *ip, uint16_t port, const char *data, size_t length) {
    if (length >= UDP_MAX_PACKET_SIZE) {
        setLastError("Packet size exceeds maximum");
        return false;
    }

    udp.beginPacket(ip, port);
    size_t written = udp.write((const uint8_t*)data, length);
    if (written != length || !udp.endPacket()) {
        setLastError("Failed to send UDP packet");
        return false;
    }

    strcpy(lastError, "No error");
    return true;
}

void UdpCom::receivePackets() {
    int packetSize = udp.parsePacket();
    if (packetSize > 0) {
        if (packetSize > UDP_MAX_PACKET_SIZE - 1) {
            packetSize = UDP_MAX_PACKET_SIZE - 1;  // Truncate if too large
        }

        int len = udp.read(packetBuffer, packetSize);
        if (len > 0) {
            packetBuffer[len] = '\0';  // Null-terminate
            if (onPacketCallback) {
                onPacketCallback(packetBuffer, len);
            }
        } else {
            setLastError("Failed to read packet");
        }
    }
}

void UdpCom::setPacketCallback(void (*callback)(const char *data, size_t length)) {
    onPacketCallback = callback;
}

const char* UdpCom::getLastError() const {
    return lastError;
}

void UdpCom::setLastError(const char* error) {
    strncpy(lastError, error, sizeof(lastError));
    lastError[sizeof(lastError) - 1] = '\0';
}