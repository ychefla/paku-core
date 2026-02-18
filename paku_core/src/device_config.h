/**
 * @file device_config.h
 * @brief Device-specific configuration and feature detection
 * 
 * Device selection is automatic — each PlatformIO environment passes
 * the appropriate -DDEVICE_xxx build flag.  If no flag is set (e.g. a
 * plain "pio run"), DEVICE_LILYGO_T_DISPLAY_S3 is used as the default.
 * 
 * Feature flags (FAN_IR_ENABLED, MILIGHT_ENABLED, HEATER_ENABLED, …)
 * are also injected via build_flags in platformio.ini and translated
 * to the HAS_xxx macros consumed by the rest of the codebase.
 * 
 * This file IS committed to version control.  secrets.h is not.
 */
#pragma once

// =============================================================================
// TARGET DEVICE SELECTION
// =============================================================================
// Uncomment ONE line below to select your target hardware.
// Default: LilyGo T-Display S3 (ESP32-S3 with display)
//
// Available devices:
//   DEVICE_LILYGO_T_DISPLAY_S3    - ESP32-S3, LilyGo T-Display S3 with ST7789V TFT display
//   DEVICE_ESP32_CH340C_30PIN     - ESP32 (not S3), Generic 30-pin board, no display
//   DEVICE_ESP8266_WIRED_SENSORS  - ESP8266, NodeMCU or generic board with DS18B20 sensor
//
// If a device is defined via build flags (-D), use that.
// Otherwise fall back to the manual selection below.
#if !defined(DEVICE_LILYGO_T_DISPLAY_S3) && \
    !defined(DEVICE_ESP32_CH340C_30PIN) && \
    !defined(DEVICE_ESP8266_WIRED_SENSORS)
#define DEVICE_LILYGO_T_DISPLAY_S3
// #define DEVICE_ESP32_CH340C_30PIN
// #define DEVICE_ESP8266_WIRED_SENSORS
#endif

// =============================================================================
// Device Configuration: LilyGo T-Display S3 (ESP32-S3)
// =============================================================================
#if defined(DEVICE_LILYGO_T_DISPLAY_S3)
    #define DEVICE_NAME "LilyGo T-Display S3"
    #define DEVICE_MODEL "lilygo-t-display-s3"
    #define MCU_ESP32_S3 1      // MCU type indicator for future MCU-specific code
    #define HAS_DISPLAY 1
    #define HAS_TOUCH 1
    #define HAS_PSRAM 1
    #define HAS_LED 1           // Has onboard LED for status indication
    #define HAS_BLE 1           // Supports BLE for Ruuvi tags and other BLE sensors
    #define HAS_WIRED_SENSORS 0 // No wired sensors by default (can be added)
    // LilyGo T-Display S3 LED configuration:
    // - Some versions have RGB LED on GPIO43 or GPIO44
    // - If your board has LED on a different pin, change PIN_LED_BUILTIN below
    // - GPIO38 is the LCD backlight pin - DO NOT use for LED
    #define PIN_LED_BUILTIN 43  // Default RGB LED pin (check your board version)
    #define LED_ON HIGH         // LED is active HIGH
    #define HAS_RGB_LED 1       // Addressable RGB LED (WS2812 style)
    // Note: For RGB LED control, use a NeoPixel or FastLED library
    
    // MiLight/MIBO light controller support (controlled by build flag)
    #ifdef MILIGHT_ENABLED
    #define HAS_MILIGHT 1
    #else
    #define HAS_MILIGHT 0
    #endif

// =============================================================================
// Device Configuration: ESP32 CH340C 30PIN (ESP32, not S3)
// =============================================================================
#elif defined(DEVICE_ESP32_CH340C_30PIN)
    #define DEVICE_NAME "ESP32 CH340C 30PIN"
    #define DEVICE_MODEL "esp32-ch340c-30pin"
    #define MCU_ESP32 1         // MCU type indicator for future MCU-specific code
    #define HAS_DISPLAY 0
    #define HAS_TOUCH 0
    #define HAS_PSRAM 0
    #define HAS_LED 1           // Has onboard LED for status indication
    #define HAS_BLE 1           // Supports BLE for Ruuvi tags and other BLE sensors
    #define HAS_WIRED_SENSORS 0 // No wired sensors by default (can be added)
    // GPIO2 is used for the onboard LED (D2) on most ESP32 dev boards
    // Flow sensor has been moved to GPIO4 to avoid conflict
    #define PIN_LED_BUILTIN 2   // LED on GPIO2 (D2)
    #define LED_ON HIGH         // LED is active HIGH

// =============================================================================
// Device Configuration: ESP8266 Wired Sensors (ESP8266, no display)
// =============================================================================
#elif defined(DEVICE_ESP8266_WIRED_SENSORS)
    #define DEVICE_NAME "ESP8266 DS18B20 Sensor"
    #define DEVICE_MODEL "esp8266-wired-sensors"
    #define MCU_ESP8266 1       // MCU type indicator for ESP8266-specific code
    #define HAS_DISPLAY 0
    #define HAS_TOUCH 0
    #define HAS_PSRAM 0
    #define HAS_LED 1           // Has onboard LED for status indication
    #define HAS_BLE 0           // ESP8266 does not support BLE
    #define HAS_WIRED_SENSORS 1 // Has DS18B20 digital temperature sensor
    #define HAS_DS18B20 1       // Has DS18B20 1-Wire temperature sensor on GPIO5
    // GPIO2 (D4 on NodeMCU) is the onboard LED
    // Note: ESP8266 LED is typically active LOW
    #define PIN_LED_BUILTIN 2   // LED on GPIO2 (D4 on NodeMCU)
    #define LED_ON LOW          // LED is active LOW on ESP8266
    // DS18B20 temperature sensor pin
    #define PIN_DS18B20 5       // GPIO5 (D1 on NodeMCU) for 1-Wire DS18B20

// =============================================================================
// Default / Error
// =============================================================================
#else
    // Default to LilyGo T-Display S3 for backward compatibility
    #warning "No device defined in device_config.h, defaulting to DEVICE_LILYGO_T_DISPLAY_S3"
    #define DEVICE_NAME "LilyGo T-Display S3"
    #define MCU_ESP32_S3 1      // MCU type indicator for future MCU-specific code
    #define HAS_DISPLAY 1
    #define HAS_TOUCH 1
    #define HAS_PSRAM 1
    #define HAS_LED 1
    #define HAS_BLE 1
    #define HAS_WIRED_SENSORS 0
    #define PIN_LED_BUILTIN 43  // Default RGB LED pin (check your board version)
    #define LED_ON HIGH
    #define HAS_RGB_LED 1
#endif

// =============================================================================
// Feature Defaults
// =============================================================================
// Set default values for optional features if not defined
#ifndef HAS_BLE
#define HAS_BLE 0
#endif

#ifndef HAS_WIRED_SENSORS
#define HAS_WIRED_SENSORS 0
#endif

#ifndef HAS_MILIGHT
#define HAS_MILIGHT 0
#endif

// MaxxFan IR control (enabled via FAN_IR_ENABLED build flag)
#ifdef FAN_IR_ENABLED
#define HAS_FAN_IR 1
#else
#define HAS_FAN_IR 0
#endif

// =============================================================================
// LED Status Indicator Patterns (for HAS_LED devices)
// =============================================================================
// Suggested LED patterns for status indication:
//
// For simple LEDs (ESP32 CH340C):
// --------------------------------
// BOOT/STARTUP:
//   - 3 quick blinks: Device starting up
//
// WIFI STATUS:
//   - Fast blink (100ms): Connecting to WiFi
//   - Solid ON (1s) then OFF: WiFi connected successfully
//   - 2 slow blinks: WiFi connection failed, retrying
//
// MQTT STATUS:
//   - Slow blink (500ms): Connecting to MQTT broker
//   - Solid ON (500ms) then OFF: MQTT connected successfully
//   - 3 rapid blinks: MQTT connection failed
//
// NORMAL OPERATION:
//   - Brief flash every 5s: Heartbeat (system running normally)
//   - Double blink: Data sent successfully
//
// ERROR CONDITIONS:
//   - Continuous fast blink: Critical error
//   - SOS pattern (... --- ...): Fatal error, needs reset
//
// SLEEP/LOW POWER:
//   - LED OFF: Device in deep sleep
//   - Very dim or single flash before sleep: Entering sleep mode
//
// For RGB LEDs (LilyGo T-Display S3):
// ------------------------------------
// Use colors to indicate different states:
//   - BLUE: WiFi connecting
//   - GREEN: WiFi connected, system OK
//   - CYAN: MQTT connecting
//   - WHITE pulse: Heartbeat (normal operation)
//   - YELLOW: Warning (e.g., sensor issue)
//   - RED: Error condition
//   - PURPLE: OTA update in progress
//   - OFF: Deep sleep
//
// Note: RGB LED control requires NeoPixel or FastLED library
//
