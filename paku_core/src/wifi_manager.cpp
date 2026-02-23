#include "wifi_manager.h"
#include <ArduinoJson.h>

// Fallback logging if logging.h not available
#ifndef LOG_INFO
#define LOG_INFO(tag, fmt, ...) Serial.printf("[INFO][%s] " fmt "\n", tag, ##__VA_ARGS__)
#endif
#ifndef LOG_DEBUG
#define LOG_DEBUG(tag, fmt, ...) Serial.printf("[DEBUG][%s] " fmt "\n", tag, ##__VA_ARGS__)
#endif
#ifndef LOG_ERROR
#define LOG_ERROR(tag, fmt, ...) Serial.printf("[ERROR][%s] " fmt "\n", tag, ##__VA_ARGS__)
#endif
#ifndef LOG_WARNING
#define LOG_WARNING(tag, fmt, ...) Serial.printf("[WARNING][%s] " fmt "\n", tag, ##__VA_ARGS__)
#endif

#ifndef LOG_ERROR
#define LOG_ERROR(tag, format, ...) Serial.printf("[ERROR][%s] " format "\n", tag, ##__VA_ARGS__)
#endif
#ifndef LOG_WARNING
#define LOG_WARNING(tag, format, ...) Serial.printf("[WARNING][%s] " format "\n", tag, ##__VA_ARGS__)
#endif
#ifndef LOG_INFO
#define LOG_INFO(tag, format, ...) Serial.printf("[INFO][%s] " format "\n", tag, ##__VA_ARGS__)
#endif
#ifndef LOG_DEBUG
#define LOG_DEBUG(tag, format, ...) Serial.printf("[DEBUG][%s] " format "\n", tag, ##__VA_ARGS__)
#endif

WiFiManager::WiFiManager() : nvsNetworkCount(0) {
  for (int i = 0; i < MAX_NVS_WIFI_NETWORKS; i++) {
    nvsNetworks[i].valid = false;
    nvsNetworks[i].ssid[0] = '\0';
    nvsNetworks[i].password[0] = '\0';
  }
}

void WiFiManager::begin() {
#ifndef ESP8266
  preferences.begin("wifi", false);  // Read-write mode
  loadFromNVS();
  LOG_INFO("WiFiManager", "Loaded %d networks from NVS", nvsNetworkCount);
#else
  LOG_INFO("WiFiManager", "NVS not supported on ESP8266");
  nvsNetworkCount = 0;
#endif
}

void WiFiManager::loadFromNVS() {
#ifndef ESP8266
  nvsNetworkCount = 0;
  
  for (int i = 0; i < MAX_NVS_WIFI_NETWORKS; i++) {
    String ssidKey = getKey(i, "ssid");
    String passKey = getKey(i, "pass");
    
    if (preferences.isKey(ssidKey.c_str())) {
      String ssid = preferences.getString(ssidKey.c_str(), "");
      String password = preferences.getString(passKey.c_str(), "");
      
      if (ssid.length() > 0) {
        strncpy(nvsNetworks[i].ssid, ssid.c_str(), MAX_WIFI_SSID_LEN);
        nvsNetworks[i].ssid[MAX_WIFI_SSID_LEN] = '\0';
        
        strncpy(nvsNetworks[i].password, password.c_str(), MAX_WIFI_PASSWORD_LEN);
        nvsNetworks[i].password[MAX_WIFI_PASSWORD_LEN] = '\0';
        
        nvsNetworks[i].valid = true;
        nvsNetworkCount++;
        
        LOG_DEBUG("WiFiManager", "Loaded network %d: %s", i, nvsNetworks[i].ssid);
      }
    }
  }
#endif
}

void WiFiManager::saveToNVS() {
#ifndef ESP8266
  // Clear all existing keys first
  preferences.clear();
  
  // Save valid networks
  int savedCount = 0;
  for (int i = 0; i < MAX_NVS_WIFI_NETWORKS; i++) {
    if (nvsNetworks[i].valid && strlen(nvsNetworks[i].ssid) > 0) {
      String ssidKey = getKey(savedCount, "ssid");
      String passKey = getKey(savedCount, "pass");
      
      preferences.putString(ssidKey.c_str(), nvsNetworks[i].ssid);
      preferences.putString(passKey.c_str(), nvsNetworks[i].password);
      
      LOG_DEBUG("WiFiManager", "Saved network %d: %s", savedCount, nvsNetworks[i].ssid);
      savedCount++;
    }
  }
  
  nvsNetworkCount = savedCount;
  LOG_INFO("WiFiManager", "Saved %d networks to NVS", nvsNetworkCount);
#endif
}

String WiFiManager::getKey(int index, const char* suffix) {
  return String("wifi") + String(index) + "_" + String(suffix);
}

bool WiFiManager::addNetwork(const char* ssid, const char* password) {
  if (!ssid || strlen(ssid) == 0) {
    LOG_ERROR("WiFiManager", "Invalid SSID");
    return false;
  }
  
  // Check if network already exists
  for (int i = 0; i < MAX_NVS_WIFI_NETWORKS; i++) {
    if (nvsNetworks[i].valid && strcmp(nvsNetworks[i].ssid, ssid) == 0) {
      // Update existing network
      strncpy(nvsNetworks[i].password, password ? password : "", MAX_WIFI_PASSWORD_LEN);
      nvsNetworks[i].password[MAX_WIFI_PASSWORD_LEN] = '\0';
      saveToNVS();
      LOG_INFO("WiFiManager", "Updated network: %s", ssid);
      return true;
    }
  }
  
  // Find empty slot
  for (int i = 0; i < MAX_NVS_WIFI_NETWORKS; i++) {
    if (!nvsNetworks[i].valid) {
      strncpy(nvsNetworks[i].ssid, ssid, MAX_WIFI_SSID_LEN);
      nvsNetworks[i].ssid[MAX_WIFI_SSID_LEN] = '\0';
      
      strncpy(nvsNetworks[i].password, password ? password : "", MAX_WIFI_PASSWORD_LEN);
      nvsNetworks[i].password[MAX_WIFI_PASSWORD_LEN] = '\0';
      
      nvsNetworks[i].valid = true;
      nvsNetworkCount++;
      saveToNVS();
      
      LOG_INFO("WiFiManager", "Added network: %s", ssid);
      return true;
    }
  }
  
  LOG_ERROR("WiFiManager", "Storage full, cannot add network: %s", ssid);
  return false;
}

bool WiFiManager::removeNetwork(const char* ssid) {
  for (int i = 0; i < MAX_NVS_WIFI_NETWORKS; i++) {
    if (nvsNetworks[i].valid && strcmp(nvsNetworks[i].ssid, ssid) == 0) {
      nvsNetworks[i].valid = false;
      nvsNetworks[i].ssid[0] = '\0';
      nvsNetworks[i].password[0] = '\0';
      nvsNetworkCount--;
      saveToNVS();
      
      LOG_INFO("WiFiManager", "Removed network: %s", ssid);
      return true;
    }
  }
  
  LOG_WARNING("WiFiManager", "Network not found: %s", ssid);
  return false;
}

void WiFiManager::clearAllNetworks() {
  for (int i = 0; i < MAX_NVS_WIFI_NETWORKS; i++) {
    nvsNetworks[i].valid = false;
    nvsNetworks[i].ssid[0] = '\0';
    nvsNetworks[i].password[0] = '\0';
  }
  nvsNetworkCount = 0;
  
#ifndef ESP8266
  preferences.clear();
#endif
  
  LOG_INFO("WiFiManager", "Cleared all stored networks");
}

WiFiCredential WiFiManager::getNetwork(int index) const {
  if (index >= 0 && index < MAX_NVS_WIFI_NETWORKS) {
    return nvsNetworks[index];
  }
  
  WiFiCredential empty;
  empty.valid = false;
  empty.ssid[0] = '\0';
  empty.password[0] = '\0';
  return empty;
}

String WiFiManager::listNetworks() const {
  JsonDocument doc;
  JsonArray networks = doc["networks"].to<JsonArray>();
  
  for (int i = 0; i < MAX_NVS_WIFI_NETWORKS; i++) {
    if (nvsNetworks[i].valid && strlen(nvsNetworks[i].ssid) > 0) {
      networks.add(nvsNetworks[i].ssid);
    }
  }
  
  doc["count"] = nvsNetworkCount;
  
  String output;
  serializeJson(doc, output);
  return output;
}
