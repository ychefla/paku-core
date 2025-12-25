/**
 * @file logging.h
 * @brief Flexible logging system with per-feature debug control
 * 
 * This logging system supports:
 * - Global enable/disable (LOG_ENABLED)
 * - Standard informational messages (LOG_INFO)
 * - Per-feature debug messages (LOG_DEBUG)
 * - Multiple debug categories active simultaneously
 * 
 * Usage:
 *   LOG_INFO("System", "WiFi connected, IP: %s", WiFi.localIP().toString().c_str());
 *   LOG_DEBUG_WIFI("SSID: %s, RSSI: %d", ssid, rssi);
 *   LOG_DEBUG_MQTT("Publishing to topic: %s", topic);
 */

#pragma once

#include <Arduino.h>

// ============================================================================
// Global Logging Control
// ============================================================================

// Master logging switch - set to 0 to disable ALL logging
#ifndef LOG_ENABLED
  #define LOG_ENABLED 1
#endif

// Standard info logging (status, connections, key events)
#ifndef LOG_INFO_ENABLED
  #define LOG_INFO_ENABLED 1
#endif

// ============================================================================
// Debug Logging Categories
// ============================================================================
// Enable specific debug categories by defining them as 1 in secrets.h or here
// Multiple categories can be enabled simultaneously

#ifndef LOG_DEBUG_WIFI
  #define LOG_DEBUG_WIFI 0          // WiFi connection, scanning, credentials
#endif

#ifndef LOG_DEBUG_MQTT
  #define LOG_DEBUG_MQTT 0          // MQTT connection, publish, subscribe
#endif

#ifndef LOG_DEBUG_BLE
  #define LOG_DEBUG_BLE 0           // BLE scanning, device discovery
#endif

#ifndef LOG_DEBUG_SENSORS_RUUVI
  #define LOG_DEBUG_SENSORS_RUUVI 0 // RuuviTag data parsing, registration
#endif

#ifndef LOG_DEBUG_SENSORS_MOKO
  #define LOG_DEBUG_SENSORS_MOKO 0  // MoKo sensor data parsing, registration
#endif

#ifndef LOG_DEBUG_SENSORS_WIRED
  #define LOG_DEBUG_SENSORS_WIRED 0 // DS18B20 and other wired sensors
#endif

#ifndef LOG_DEBUG_SENSORS_FLOW
  #define LOG_DEBUG_SENSORS_FLOW 0  // Flow meter sensors
#endif

#ifndef LOG_DEBUG_POWER
  #define LOG_DEBUG_POWER 0         // Battery, sleep modes, power management
#endif

#ifndef LOG_DEBUG_OTA
  #define LOG_DEBUG_OTA 0           // OTA updates
#endif

#ifndef LOG_DEBUG_CONFIG
  #define LOG_DEBUG_CONFIG 0        // Configuration changes
#endif

#ifndef LOG_DEBUG_TIMING
  #define LOG_DEBUG_TIMING 0        // Interval timing, state machine
#endif

#ifndef LOG_DEBUG_BUFFER
  #define LOG_DEBUG_BUFFER 0        // Sensor snapshot buffering
#endif

// ============================================================================
// Logging Macros
// ============================================================================

#if LOG_ENABLED

  // Standard informational logging
  #if LOG_INFO_ENABLED
    #define LOG_INFO(category, fmt, ...) \
      Serial.printf("[INFO] [%s] " fmt "\n", category, ##__VA_ARGS__)
  #else
    #define LOG_INFO(category, fmt, ...)
  #endif

  // Per-category debug logging
  #if LOG_DEBUG_WIFI
    #define LOG_DEBUG_WIFI(fmt, ...) \
      Serial.printf("[DEBUG] [WiFi] " fmt "\n", ##__VA_ARGS__)
  #else
    #define LOG_DEBUG_WIFI(fmt, ...)
  #endif

  #if LOG_DEBUG_MQTT
    #define LOG_DEBUG_MQTT(fmt, ...) \
      Serial.printf("[DEBUG] [MQTT] " fmt "\n", ##__VA_ARGS__)
  #else
    #define LOG_DEBUG_MQTT(fmt, ...)
  #endif

  #if LOG_DEBUG_BLE
    #define LOG_DEBUG_BLE(fmt, ...) \
      Serial.printf("[DEBUG] [BLE] " fmt "\n", ##__VA_ARGS__)
  #else
    #define LOG_DEBUG_BLE(fmt, ...)
  #endif

  #if LOG_DEBUG_SENSORS_RUUVI
    #define LOG_DEBUG_RUUVI(fmt, ...) \
      Serial.printf("[DEBUG] [Ruuvi] " fmt "\n", ##__VA_ARGS__)
  #else
    #define LOG_DEBUG_RUUVI(fmt, ...)
  #endif

  #if LOG_DEBUG_SENSORS_MOKO
    #define LOG_DEBUG_MOKO(fmt, ...) \
      Serial.printf("[DEBUG] [MoKo] " fmt "\n", ##__VA_ARGS__)
  #else
    #define LOG_DEBUG_MOKO(fmt, ...)
  #endif

  #if LOG_DEBUG_SENSORS_WIRED
    #define LOG_DEBUG_WIRED(fmt, ...) \
      Serial.printf("[DEBUG] [Wired] " fmt "\n", ##__VA_ARGS__)
  #else
    #define LOG_DEBUG_WIRED(fmt, ...)
  #endif

  #if LOG_DEBUG_SENSORS_FLOW
    #define LOG_DEBUG_FLOW(fmt, ...) \
      Serial.printf("[DEBUG] [Flow] " fmt "\n", ##__VA_ARGS__)
  #else
    #define LOG_DEBUG_FLOW(fmt, ...)
  #endif

  #if LOG_DEBUG_POWER
    #define LOG_DEBUG_POWER(fmt, ...) \
      Serial.printf("[DEBUG] [Power] " fmt "\n", ##__VA_ARGS__)
  #else
    #define LOG_DEBUG_POWER(fmt, ...)
  #endif

  #if LOG_DEBUG_OTA
    #define LOG_DEBUG_OTA(fmt, ...) \
      Serial.printf("[DEBUG] [OTA] " fmt "\n", ##__VA_ARGS__)
  #else
    #define LOG_DEBUG_OTA(fmt, ...)
  #endif

  #if LOG_DEBUG_CONFIG
    #define LOG_DEBUG_CONFIG(fmt, ...) \
      Serial.printf("[DEBUG] [Config] " fmt "\n", ##__VA_ARGS__)
  #else
    #define LOG_DEBUG_CONFIG(fmt, ...)
  #endif

  #if LOG_DEBUG_TIMING
    #define LOG_DEBUG_TIMING(fmt, ...) \
      Serial.printf("[DEBUG] [Timing] " fmt "\n", ##__VA_ARGS__)
  #else
    #define LOG_DEBUG_TIMING(fmt, ...)
  #endif

  #if LOG_DEBUG_BUFFER
    #define LOG_DEBUG_BUFFER(fmt, ...) \
      Serial.printf("[DEBUG] [Buffer] " fmt "\n", ##__VA_ARGS__)
  #else
    #define LOG_DEBUG_BUFFER(fmt, ...)
  #endif

  // Generic debug macro (always enabled if LOG_ENABLED)
  #define LOG_DEBUG(category, fmt, ...) \
    Serial.printf("[DEBUG] [%s] " fmt "\n", category, ##__VA_ARGS__)

#else
  // All logging disabled
  #define LOG_INFO(category, fmt, ...)
  #define LOG_DEBUG_WIFI(fmt, ...)
  #define LOG_DEBUG_MQTT(fmt, ...)
  #define LOG_DEBUG_BLE(fmt, ...)
  #define LOG_DEBUG_RUUVI(fmt, ...)
  #define LOG_DEBUG_MOKO(fmt, ...)
  #define LOG_DEBUG_WIRED(fmt, ...)
  #define LOG_DEBUG_FLOW(fmt, ...)
  #define LOG_DEBUG_POWER(fmt, ...)
  #define LOG_DEBUG_OTA(fmt, ...)
  #define LOG_DEBUG_CONFIG(fmt, ...)
  #define LOG_DEBUG_TIMING(fmt, ...)
  #define LOG_DEBUG_BUFFER(fmt, ...)
  #define LOG_DEBUG(category, fmt, ...)
#endif

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * @brief Print current logging configuration
 */
inline void printLoggingConfig() {
  #if LOG_ENABLED
    Serial.println("\n=== Logging Configuration ===");
    Serial.printf("Master: %s\n", "ENABLED");
    Serial.printf("Info:   %s\n", LOG_INFO_ENABLED ? "ON" : "OFF");
    Serial.println("\nDebug Categories:");
    
    #if LOG_DEBUG_WIFI
      Serial.println("  WiFi:        ON");
    #else
      Serial.println("  WiFi:        OFF");
    #endif
    
    #if LOG_DEBUG_MQTT
      Serial.println("  MQTT:        ON");
    #else
      Serial.println("  MQTT:        OFF");
    #endif
    
    #if LOG_DEBUG_BLE
      Serial.println("  BLE:         ON");
    #else
      Serial.println("  BLE:         OFF");
    #endif
    
    #if LOG_DEBUG_SENSORS_RUUVI
      Serial.println("  Ruuvi:       ON");
    #else
      Serial.println("  Ruuvi:       OFF");
    #endif
    
    #if LOG_DEBUG_SENSORS_MOKO
      Serial.println("  MoKo:        ON");
    #else
      Serial.println("  MoKo:        OFF");
    #endif
    
    #if LOG_DEBUG_SENSORS_WIRED
      Serial.println("  Wired:       ON");
    #else
      Serial.println("  Wired:       OFF");
    #endif
    
    #if LOG_DEBUG_SENSORS_FLOW
      Serial.println("  Flow:        ON");
    #else
      Serial.println("  Flow:        OFF");
    #endif
    
    #if LOG_DEBUG_POWER
      Serial.println("  Power:       ON");
    #else
      Serial.println("  Power:       OFF");
    #endif
    
    #if LOG_DEBUG_OTA
      Serial.println("  OTA:         ON");
    #else
      Serial.println("  OTA:         OFF");
    #endif
    
    #if LOG_DEBUG_CONFIG
      Serial.println("  Config:      ON");
    #else
      Serial.println("  Config:      OFF");
    #endif
    
    #if LOG_DEBUG_TIMING
      Serial.println("  Timing:      ON");
    #else
      Serial.println("  Timing:      OFF");
    #endif
    
    #if LOG_DEBUG_BUFFER
      Serial.println("  Buffer:      ON");
    #else
      Serial.println("  Buffer:      OFF");
    #endif
    
    Serial.println("=============================\n");
  #else
    // Logging disabled - no output
  #endif
}
