# IoT Device Firmware

This repository contains firmware for IoT devices built using the ESP8266 platform and PlatformIO. The project supports two types of devices: **[Actuators](./actuator_firmware/README)** (e.g., devices that receive commands to control outputs like LEDs) and **[Sensors](./sensor_firmware/README)** (e.g., devices that periodically send data like light levels). Both firmwares leverage a common set of libraries for WiFi connectivity, MQTT communication, and device pairing.

## Features
- **WiFi Management**: Uses `WiFiManager` for easy network configuration via SmartConfig.
- **MQTT Communication**: Connects to an MQTT broker for data exchange using `MQTTClient`.
- **UDP Pairing**: Implements a UDP-based pairing mechanism with `UdpCom` for initial setup.
- **EEPROM Storage**: Stores WiFi and MQTT credentials using `NetworkHub`.
- **Reset Mechanism**: Supports factory reset via a physical button.

## Project Structure
```
shenhome-device/
├── actuator_firmware/    # Firmware for actuator devices
│   ├── src/              # Source code
│   └── platformio.ini    # PlatformIO configuration
├── sensor_firmware/      # Firmware for sensor devices
│   ├── src/              # Source code
│   └── platformio.ini    # PlatformIO configuration
└── README.md             # This file
```

## Prerequisites
- **PlatformIO**: Install via VSCode extension or CLI:
  ```bash
  pip install platformio
  ```
- **ESP8266 Board**: Compatible hardware (e.g., NodeMCU).
- **MQTT Broker**: A running MQTT broker (e.g., Mosquitto on `192.168.2.124:1883`).
- **Dependencies**: Managed via `platformio.ini` (see individual project READMEs).

## Common Libraries
- `WiFiManager`: Handles WiFi setup and SmartConfig pairing.
- `MQTTClient`: Manages MQTT connections and message handling.
- `NetworkHub`: Stores and retrieves credentials from EEPROM.
- `UdpCom`: Facilitates UDP-based device discovery and pairing.

## Getting Started
1. Clone this repository:
   ```bash
   git clone https://github.com/chinhtran-dev/shenhome-device.git
   ```
2. Open the desired project folder (`actuator_firmware` or `sensor_firmware`) in PlatformIO IDE.
3. Configure hardware connections (pins defined in `Config.h`).
4. Build and upload firmware:
   ```bash
   pio run -t upload
   ```
5. Refer to the specific README for each firmware for detailed setup and usage.

## Sub-Projects
- **[Actuator Firmware](./actuator_firmware/README)**: For devices that receive and act on MQTT commands.
- **[Sensor Firmware](./sensor_firmware/README)**: For devices that periodically send sensor data via MQTT.

For more details, refer to the respective project directories.