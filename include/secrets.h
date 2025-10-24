```cpp
#pragma once

// Wi-Fi
#define WIFI_SSID_1      "JosPar (paku)"
#define WIFI_PASSWORD_1    "V11vuskaBaabuska"
#define WIFI_SSID_2      "JosPar_5G"
#define WIFI_PASSWORD_2    "V11vuskaBaabuska"
static const char* WIFI_SSIDS[] = { "JosPar (paku)", "JosPar_5G", "iPhone (Jossu)" };
static const char* WIFI_PASSWORDS[] = { "V11vuskaBaabuska", "V11vuskaBaabuska", "1234567890" };
static const size_t WIFI_COUNT = sizeof(WIFI_SSIDS) / sizeof(WIFI_SSIDS[0]);

// MQTT
#define MQTT_HOST        "192.168.1.10"
#define MQTT_PORT        1883
#define MQTT_USERNAME    "mqtt_user"
#define MQTT_PASSWORD    "mqtt_pass"

// Identity
#define DEVICE_ID        "edge-1"      // short, URL-safe
#define MQTT_BASE_TOPIC  "paku"        // topic namespace root
