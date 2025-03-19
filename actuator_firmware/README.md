# Actuator Firmware

This firmware is designed for ESP8266-based actuator devices that receive commands via MQTT to control outputs (e.g., LEDs, relays). It supports WiFi configuration, MQTT communication, and a reset mechanism.

## Features
- Receives MQTT commands on `device/<MAC>/command` to control outputs.
- Publishes status updates to `gateway/data` when toggled manually.
- Sends periodic heartbeats to `gateway/heartbeat`.
- Supports UDP pairing for initial setup.
- Factory reset via a physical button.

## Hardware Requirements
- **ESP8266 Board**: e.g., NodeMCU.
- **LED**: Connected to `LED_BUILTIN`.
- **Reset Button**: Connected to GPIO14 (D5).
- **Toggle Button**: Connected to GPIO12 (D6).

## Pin Configuration
Defined in `Config.h`:
- `RESET_BUTTON_PIN`: GPIO14
- `TOGGLE_BUTTON_PIN`: GPIO12

## MQTT Topics
- **Subscribe**: `device/<MAC>/command`
  - Payload: `{"status":"Enabled"}` or `{"status":"Disabled"}`
- **Publish**:
  - `gateway/register`: Device registration with MAC address.
  - `gateway/heartbeat`: Periodic status (`Enabled` every 60s).
  - `gateway/data`: Status updates on toggle.

## Setup
1. Open `actuator_firmware` in PlatformIO IDE.
2. Connect hardware as per pin configuration.
3. Build and upload:
   ```bash
   pio run -t upload
4. On first boot, enter pairing mode:
    - Device creates a WiFi AP (SmartConfig).
    - Use a mobile app to send WiFi credentials.
5. Device connects to MQTT broker (192.168.2.124:1883 by default).

## Usage
- Toggle LED: Press the toggle button to change state and send updates.
- Send Commands: Publish to device/<MAC>/command
    ``` bash
    mosquitto_pub -h 192.168.2.124 -p 1883 -t "device/xx:xx:xx:xx:xx:xx/command" -m '{"status":"Enabled"}'
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
