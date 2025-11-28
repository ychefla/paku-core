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

// paku-iot HTTP Endpoint (optional)
#define PAKU_IOT_ENABLED              0
#define PAKU_IOT_HOST                 "iot.paku.example.com"
#define PAKU_IOT_PORT                 443
#define PAKU_IOT_API_KEY              "your-api-key"
#define PAKU_IOT_USE_TLS              true
```

The firmware will try each WiFi network in the following order until a connection is established:
1. HOME
2. IPHONE
3. PAKU

## 2) paku-iot Integration

The EDGE device can send telemetry data to paku-iot using HTTP/HTTPS in addition to MQTT. This is useful for direct integration with paku-iot services without requiring an MQTT broker.

To enable paku-iot HTTP transport:

1. Set `PAKU_IOT_ENABLED` to `1` in `secrets.h`
2. Configure the paku-iot endpoint:
   - `PAKU_IOT_HOST`: The hostname of your paku-iot server
   - `PAKU_IOT_PORT`: The port number (443 for HTTPS, 80 for HTTP)
   - `PAKU_IOT_API_KEY`: Your API key for authentication
   - `PAKU_IOT_USE_TLS`: Set to `true` for HTTPS, `false` for HTTP

See [paku-iot-integration.md](paku-iot-integration.md) for detailed protocol documentation.

## 3) Pin Configuration

Pin assignments are defined in `src/pin_config.h`. These are configured for the LilyGo T-Display S3 board and should not need modification unless you're using different hardware.
