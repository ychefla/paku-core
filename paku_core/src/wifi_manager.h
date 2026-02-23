#pragma once

#ifdef ESP8266
#include <EEPROM.h>
#else
#include <Preferences.h>
#endif

#include <Arduino.h>

#define MAX_NVS_WIFI_NETWORKS 5
#define MAX_WIFI_SSID_LEN 32
#define MAX_WIFI_PASSWORD_LEN 64

/**
 * @brief WiFi credentials stored in NVS
 */
struct WiFiCredential {
  char ssid[MAX_WIFI_SSID_LEN + 1];
  char password[MAX_WIFI_PASSWORD_LEN + 1];
  bool valid;
};

/**
 * @brief Manages WiFi credentials with NVS persistence
 * 
 * Supports dynamic WiFi configuration via MQTT with persistent storage.
 * Credentials survive OTA updates and power cycles.
 * 
 * Priority order:
 * 1. NVS stored networks (configured via MQTT)
 * 2. Firmware default networks (from secrets.h)
 */
class WiFiManager {
private:
#ifndef ESP8266
  Preferences preferences;
#endif
  WiFiCredential nvsNetworks[MAX_NVS_WIFI_NETWORKS];
  int nvsNetworkCount;
  
  void loadFromNVS();
  void saveToNVS();
  String getKey(int index, const char* suffix);

public:
  WiFiManager();
  
  /**
   * @brief Initialize WiFi manager and load stored credentials
   */
  void begin();
  
  /**
   * @brief Add a WiFi network to NVS storage
   * @param ssid Network SSID
   * @param password Network password
   * @return true if added successfully, false if storage is full
   */
  bool addNetwork(const char* ssid, const char* password);
  
  /**
   * @brief Remove a WiFi network from NVS storage
   * @param ssid Network SSID to remove
   * @return true if removed, false if not found
   */
  bool removeNetwork(const char* ssid);
  
  /**
   * @brief Clear all stored WiFi networks
   */
  void clearAllNetworks();
  
  /**
   * @brief Get count of stored WiFi networks
   * @return Number of networks in NVS
   */
  int getStoredNetworkCount() const { return nvsNetworkCount; }
  
  /**
   * @brief Get stored network by index
   * @param index Network index (0 to MAX_NVS_WIFI_NETWORKS-1)
   * @return WiFiCredential structure (check valid flag)
   */
  WiFiCredential getNetwork(int index) const;
  
  /**
   * @brief List all stored networks (SSIDs only, not passwords)
   * @return JSON array string of SSIDs
   */
  String listNetworks() const;
};
