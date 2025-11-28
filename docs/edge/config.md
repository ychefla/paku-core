# EDGE configuration

Create a header for secrets (not committed to git).

## 1) Wi-Fi & MQTT secrets

Copy the template file to create your secrets file:
```bash
cp paku_core/include/secrets.h.template paku_core/include/secrets.h
```

Edit `paku_core/include/secrets.h` with your credentials:

```cpp
#pragma once

// Wi-Fi Networks (in order of priority)
#define WIFI_SSID_HOME                "your-home-wifi-ssid"
#define WIFI_PASSWORD_HOME            "your-home-wifi-password"
#define WIFI_SSID_IPHONE              "your-iphone-hotspot-ssid"
#define WIFI_PASSWORD_IPHONE          "your-iphone-hotspot-password"
#define WIFI_SSID_PAKU                "your-paku-wifi-ssid"
#define WIFI_PASSWORD_PAKU            "your-paku-wifi-password"

// MQTT Broker
#define MQTT_SERVER                   "your-mqtt-server-hostname"
#define MQTT_PORT                     1883
```

The firmware will try each WiFi network in the following order until a connection is established:
1. HOME
2. IPHONE
3. PAKU

## 2) Pin Configuration

Pin assignments are defined in `src/pin_config.h`. These are configured for the LilyGo T-Display S3 board and should not need modification unless you're using different hardware.
