**Path:** `docs/edge/config.md`
```markdown
# EDGE configuration

Create a header for secrets (not committed to git).

## 1) Wi-Fi & MQTT secrets
Create `include/secrets.h` with:

```cpp
#pragma once

// Wi-Fi
static const char* WIFI_SSIDS[] = { "HomeWiFi", "PhoneHotspot", "Van" };
static const char* WIFI_PASSWORDS[] = { "home-pass", "phone-pass", "van-pass" };
static const size_t WIFI_COUNT = sizeof(WIFI_SSIDS) / sizeof(WIFI_SSIDS[0]);

// MQTT
#define MQTT_HOST        "192.168.1.10"
#define MQTT_PORT        1883
#define MQTT_USERNAME    "mqtt_user"
#define MQTT_PASSWORD    "mqtt_pass"

// Identity
#define DEVICE_ID        "edge-1"      // short, URL-safe
#define MQTT_BASE_TOPIC  "paku"        // topic namespace root
