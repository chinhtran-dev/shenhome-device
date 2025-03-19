#ifndef CONFIG_H
#define CONFIG_H

#define EEPROM_ADDRESS 0
#define UDP_PAIRING_PORT 4210
#define MQTT_PORT 1883
#define MQTT_CLIENT_ID "lightsensor"
#define ENABLED "Enabled"
#define DISABLED "Disabled"
#define SENSOR_INTERVAL 30000  // Gửi dữ liệu cảm biến mỗi 30 giây
#define HEARTBEAT_INTERVAL 60000  // Heartbeat mỗi 60 giây
#define BLINK_INTERVAL 500
#define RESET_BUTTON_PIN 14
#define RESET_HOLD_TIME 3000
#define UDP_TIMEOUT_MS 60000
#define BROADCAST_INTERVAL 1000
#define BROADCAST_IP "255.255.255.255"

#endif