# Sensor Firmware

This firmware is designed for ESP8266-based sensor devices that periodically send data (e.g., light levels) via MQTT. It supports WiFi configuration, MQTT communication, and a reset mechanism.

## Features
- Sends light sensor data to `gateway/data` every 30 seconds.
- Publishes heartbeats to `gateway/heartbeat` every 60 seconds.
- Supports UDP pairing for initial setup.
- Factory reset via a physical button.

## Hardware Requirements
- **ESP8266 Board**: e.g., NodeMCU.
- **Light Sensor**: Connected to A0 (ADC pin).
- **Reset Button**: Connected to GPIO14 (D5).

## Pin Configuration
Defined in `Config.h`:
- `RESET_BUTTON_PIN`: GPIO14

## MQTT Topics
- **Publish**:
  - `gateway/register`: Device registration with MAC address.
  - `gateway/heartbeat`: Periodic status (`Enabled` every 60s).
  - `gateway/data`: Light sensor data every 30s.
    - Payload: `{"mac":"xx:xx:xx:xx:xx:xx","type":"LightSensor","data":{"light":<value>}}`

## Setup
1. Open `sensor_firmware` in PlatformIO IDE.
2. Connect hardware as per pin configuration.
3. Build and upload:
   ```bash
   pio run -t upload
4. On first boot, enter pairing mode:
    - Device creates a WiFi AP (SmartConfig).
    - Use a mobile app to send WiFi credentials.
5. Device connects to MQTT broker (192.168.2.124:1883 by default).

## Usage
- Monitor Data: Subscribe to gateway/data:
    ```bash
    mosquitto_sub -h 192.168.2.124 -p 1883 -t "gateway/data"
- Reset: Hold reset button for 3 seconds to clear EEPROM and restart.

## Dependencies
# Defined in platformio.ini:
- WiFiManager (custom, see ./lib)
- MQTTClient (custom, see ./lib)
- NetworkHub (custom, see ./lib/)
- UdpCom (custom, see ./lib/)

## Troubleshooting
- Check Serial Monitor (115200 baud) for error logs.
- Ensure MQTT broker is running and accessible.
- Verify sensor connection to A0
