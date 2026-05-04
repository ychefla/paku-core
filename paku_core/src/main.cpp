#ifdef ESP8266
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#include <WiFiClientSecure.h>
#endif

#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>  // ESP32 native time functions

// Preferences for persistent storage
#ifdef ESP8266
#include <EEPROM.h>
#else
#include <Preferences.h>
#endif

#include "Arduino.h"
#include "logging.h"            // Flexible logging system
#include "device_config.h"      // Device selection and feature detection
#include "pin_config.h"
#include "timing_config.h"      // Timing and power management configuration
#include "PakuIotClient.h"
#include "sensor_placeholders.h"
#include "OtaClient.h"
#include "wifi_manager.h"       // WiFi credentials manager with NVS persistence
#include "mqtt_manager.h"       // MQTT broker failover (local-preferred, cloud-synced)
#ifndef ESP8266
#include <ESPmDNS.h>            // mDNS for resolving homeassistant.local
#endif
#include <string>

// Firmware version
#ifndef FIRMWARE_VERSION
#define FIRMWARE_VERSION "1.4.2"
#endif

// BLE support (ESP32 only)
#if HAS_BLE
#include "BLEDevice.h"
#include "ruuvi.h"
#include "ruuvi_scanner.h"
#include "moko.h"
#include "moko_scanner.h"
#include "frezzer.h"
#include "frezzer_controller.h"
#ifndef ESP8266
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#endif // ESP8266
#endif // HAS_BLE

// Analog sensor support (ESP8266 and ESP32)
#if HAS_WIRED_SENSORS
#include "wired_sensors.h"
#endif // HAS_WIRED_SENSORS

#ifdef HEATER_ENABLED
#include "heater_addon.h"
#endif

#if HAS_FAN_IR
#include "maxxfan_ir.h"
#endif

#if HAS_MILIGHT
#include "milight_client.h"
#endif

// Display-related includes and definitions (only when display is available)
#if HAS_DISPLAY
#include "TFT_eSPI.h" /* Please use the TFT library provided in the library. */
#include "img_logo.h"
#include "display_ui.h"  // Multi-screen UI with button controls

/* The product now has two screens, and the initialization code needs a small change in the new version. The LCD_MODULE_CMD_1 is used to define the
 * switch macro. */
#define LCD_MODULE_CMD_1

TFT_eSPI tft = TFT_eSPI();
#define TFT_UPDATE_WAIT 1000
unsigned long targetTime = 0; // Used for testing draw times

#if defined(LCD_MODULE_CMD_1)
typedef struct {
    uint8_t cmd;
    uint8_t data[14];
    uint8_t len;
} lcd_cmd_t;

lcd_cmd_t lcd_st7789v[] = {
    {0x11, {0}, 0 | 0x80},
    {0x3A, {0X05}, 1},
    {0xB2, {0X0B, 0X0B, 0X00, 0X33, 0X33}, 5},
    {0xB7, {0X75}, 1},
    {0xBB, {0X28}, 1},
    {0xC0, {0X2C}, 1},
    {0xC2, {0X01}, 1},
    {0xC3, {0X1F}, 1},
    {0xC6, {0X13}, 1},
    {0xD0, {0XA7}, 1},
    {0xD0, {0XA4, 0XA1}, 2},
    {0xD6, {0XA1}, 1},
    {0xE0, {0XF0, 0X05, 0X0A, 0X06, 0X06, 0X03, 0X2B, 0X32, 0X43, 0X36, 0X11, 0X10, 0X2B, 0X32}, 14},
    {0xE1, {0XF0, 0X08, 0X0C, 0X0B, 0X09, 0X24, 0X2B, 0X22, 0X43, 0X38, 0X15, 0X16, 0X2F, 0X37}, 14},
};
#endif
#endif // HAS_DISPLAY

// Waveshare RGB LCD touchscreen GUI (LVGL-based, 800×480)
#if HAS_RGB_LCD
#include "paku_gui.h"
#include "waveshare_hal.h"
#endif

// BLE settings (ESP32 only)
#if HAS_BLE
bool scanBT_enabled = true;
// BLE scan interval in milliseconds between scan cycles
#define BLE_SCAN_INTERVAL_MS 10000

// Frezzer compressor fridge configuration
// Define FREZZER_COUNT > 0 and FREZZER_MACS/FREZZER_LOCATIONS in secrets.h
#ifndef FREZZER_COUNT
#define FREZZER_COUNT 0
#endif
#endif // HAS_BLE

// Analog sensor settings (ESP8266 and ESP32)
#if HAS_WIRED_SENSORS
WiredSensors wiredSensors;
bool wiredSensorsEnabled = true;
unsigned long lastWiredSensorRead = 0;
#define WIRED_SENSOR_INTERVAL_MS 10000  // Read every 10 seconds (was 60000)
#endif // HAS_WIRED_SENSORS

// Maximum number of telemetry readings per HTTP batch
#define MAX_TELEMETRY_READINGS 20

// Maximum number of MQTT payloads that can be queued
#define MAX_MQTT_PAYLOADS 30

// Ruuvi tag configuration - placeholder MAC addresses for known tags
// These should be configured in secrets.h for production deployments
#ifndef RUUVI_TAG_COUNT
#define RUUVI_TAG_COUNT 0
#endif
// Example: In secrets.h, define known Ruuvi tags like:
// #define RUUVI_TAG_COUNT 4
// static const char* RUUVI_TAG_MACS[] = {"AA:BB:CC:DD:EE:01", "AA:BB:CC:DD:EE:02", "AA:BB:CC:DD:EE:03", "AA:BB:CC:DD:EE:04"};
// static const char* RUUVI_TAG_LOCATIONS[] = {"cabin", "kitchen", "lounge", "dryer"};

// Device ID buffer (derived from MAC address)
char deviceId[20] = "";  // Not static - needs external linkage for display_ui.cpp

// Enable placeholder sensor data generation for testing
// Set to false by default - only enable for local testing
bool generatePlaceholderData = false;

// WiFi settings (using arrays from secrets.h)
String wifi_status = "";


// MQTT settings — cloud (MQTT_SERVER) and local HA (MQTT_LOCAL) broker configs
// Cloud broker defaults (used as fallback when local HA is unreachable)
#ifndef MQTT_SERVER
  #define MQTT_SERVER ""
#endif
#ifndef MQTT_PORT
  #define MQTT_PORT 8883
#endif
#ifndef MQTT_USER
  #define MQTT_USER ""
#endif
#ifndef MQTT_PASSWORD
  #define MQTT_PASSWORD ""
#endif
#ifndef MQTT_USE_TLS
  #define MQTT_USE_TLS 0
#endif

// Local HA broker defaults (tried first — fast, works offline)
#ifndef MQTT_LOCAL
  #define MQTT_LOCAL "homeassistant.local"
#endif
#ifndef MQTT_LOCAL_PORT
  #define MQTT_LOCAL_PORT 1883
#endif
#ifndef MQTT_LOCAL_USER
  #define MQTT_LOCAL_USER ""
#endif
#ifndef MQTT_LOCAL_PASSWORD
  #define MQTT_LOCAL_PASSWORD ""
#endif

// CA cert for TLS brokers (optional — cloud broker may use publicly trusted CA)
#ifndef MQTT_CA_CERT
  #define MQTT_CA_CERT nullptr
#endif

// PubSubClient — transport (WiFiClient vs WiFiClientSecure) is managed
// by MqttManager internally, switching per-broker as needed.
PubSubClient client;
// ESP32 native time functions will be used instead of NTPClient for automatic DST support

// MQTT broker failover manager
MqttManager mqttMgr;

// WiFi credentials manager with NVS persistence
WiFiManager wifiManager;

// paku-iot HTTP client (enabled via PAKU_IOT_ENABLED in secrets.h)
PakuIotClient pakuIotClient;
#ifndef PAKU_IOT_ENABLED
#define PAKU_IOT_ENABLED 0
#endif
unsigned long lastTime_pakuIot = 0;
unsigned long pakuIotInterval = 60000;  // 1 minute interval for HTTP transport

// OTA update client
OtaClient otaClient;
bool otaUpdatePending = false;
String pendingOtaUrl = "";
String pendingOtaChecksum = "";
String pendingOtaVersion = "";

// ============================================================================
// Timing Configuration and State Management
// ============================================================================

// Device configuration structure (replaces individual timing variables)
DeviceConfig deviceConfig;

// Current operational state
DeviceState currentState = STATE_INIT;

// ============================================================================
// Phase 2: State Machine and Sensor Buffer
// ============================================================================

// System state machine
enum SystemState {
  SYS_STATE_IDLE,
  SYS_STATE_COLLECT_SENSORS,
  SYS_STATE_CONNECT_NETWORK,
  SYS_STATE_TRANSMIT,
  SYS_STATE_DISCONNECT
};

SystemState currentSystemState = SYS_STATE_IDLE;
unsigned long stateEnteredAt = 0;
unsigned long lastSensorCollection = 0;
unsigned long lastNetworkConnect = 0;

// Sensor reading buffer
struct SensorReading {
  char timestamp[32];
  char sensor_id[32];
  char metric[32];
  float value;
  bool transmitted;
};

#define MAX_BUFFERED_READINGS 50
SensorReading sensorBuffer[MAX_BUFFERED_READINGS];
int bufferCount = 0;

// Preferences for config persistence
#ifndef ESP8266
Preferences preferences;
#endif

// ============================================================================
// End Phase 2 Additions
// ============================================================================

// Legacy timing variables (maintained for backward compatibility during Phase 1)
unsigned long lastTime_sensor = 0;
unsigned long lastTime_mqtt = 0;
unsigned long mqttFastInterval = 10000;  // 10 second interval in ms
unsigned long mqttSlowInterval = 3600000;  // 1 hour interval in ms
unsigned long sensorFastInterval = 5000;  // 5 second interval in ms
unsigned long sensorSlowInterval = 60000;  // 1 minute interval in ms
unsigned long mqttInterval;
unsigned long sensorInterval;

// ============================================================================
// Flow Sensor (DISABLED - Future Development)
// ============================================================================
// Flow sensor functionality has been moved to lib/paku_lib/src/flow_sensor.h
// Awaiting hardware integration and calibration before production use.
// TODO: Re-enable when hardware is installed and calibrated

#define FLOW_SENSOR_ENABLED false  // Set to true when ready for production

// Default heater status to 1 (on) for testing - use fast mode by default
// Controlled via MQTT paku/control topic: {"heater": 1} on, {"heater": 0} off
int heaterStatus = 1;

// Tracks whether optional add-ons were actually initialised (depends on runtime peripheral config).
// Set during setup(); checked in loop() and MQTT handlers to guard add-on calls.
#ifdef HEATER_ENABLED
bool heaterAddonActive = false;
#endif
#if HAS_FAN_IR
bool fanIrActive = false;
#endif
#if HAS_MILIGHT
bool milightActive = false;
#endif

// GUI board sensor-slot table: maps incoming sensor_id strings to GUI series indices.
// First sensor seen → series 0 (Indoor), second → series 1 (Outdoor), etc.
#if HAS_RGB_LCD
struct GuiSensorSlot {
    char id[36];
    float lastTemp;
    float lastHum;
};
static GuiSensorSlot guiSensorSlots[4] = {};
static uint8_t guiSensorSlotCount = 0;

static uint8_t getOrAssignGuiSlot(const char* sensorId) {
    for (uint8_t i = 0; i < guiSensorSlotCount; i++) {
        if (strcmp(guiSensorSlots[i].id, sensorId) == 0) return i;
    }
    if (guiSensorSlotCount < 4) {
        strncpy(guiSensorSlots[guiSensorSlotCount].id, sensorId, 35);
        guiSensorSlots[guiSensorSlotCount].id[35] = '\0';
        guiSensorSlots[guiSensorSlotCount].lastTemp = NAN;
        guiSensorSlots[guiSensorSlotCount].lastHum  = NAN;
        return guiSensorSlotCount++;
    }
    return 255;
}
#endif

// ============================================================================
// MQTT Payload Buffer
// ============================================================================

struct Payload {
  String topic;
  String data;
};

Payload payloads[MAX_MQTT_PAYLOADS];
int payloadIndex = 0;

// Sensor Snapshot Buffer (for consolidated multi-metric payloads from BLE, wired, etc.)
#define MAX_SENSOR_SNAPSHOTS 10
struct SensorSnapshot {
  String topic;
  String payload;
  bool transmitted;
};

SensorSnapshot sensorSnapshots[MAX_SENSOR_SNAPSHOTS];
int sensorSnapshotCount = 0;

// Edge device friendly name lookup
const char* getEdgeDeviceFriendlyName() {
#ifdef EDGE_DEVICE_COUNT
  uint8_t mac[6];
  WiFi.macAddress(mac);
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  
  for (size_t i = 0; i < EDGE_DEVICE_COUNT; i++) {
    if (strcmp(macStr, EDGE_DEVICE_MACS[i]) == 0) {
      return EDGE_DEVICE_NAMES[i];
    }
  }
#endif
  return nullptr;  // No friendly name configured
}

// ISR function declaration moved up
void IRAM_ATTR countRisingEdges();
void updateIntervals();
void connectMQTT();
void createPayload(String topic, float value, String timestamp);
void connect_wifi();
void sendToMQTT();
void sendToPakuIot();
void initPakuIot();
void processData();
void processRuuviData();
void publishDeviceStatus();
void publishDeviceConfig();
void onMqttConnect(PubSubClient& client, MqttBroker broker);

// Phase 2: New function declarations
void printConfig(const char* context);
void saveConfig();
void loadConfig();
void addSensorReading(const char* sensor_id, const char* metric, float value, const char* timestamp);
void clearTransmittedReadings();
void collectSensorData();
void transmitBufferedData();
void connectNetwork();
void disconnectNetwork();
void handleSystemState();

#if HAS_BLE
void initRuuviTags();
void createRuuviPayloads(const char* timestamp);
void initMoKoSensors();
void createMoKoPayloads(const char* timestamp);
void initFrezzerDevices();
void createFrezzerPayloads(const char* timestamp);
void handleFrezzerMqttCommand(const char* topic, const char* payload);
void scanBT(void* parameter);
void saveRuuviWhitelist();
void loadRuuviWhitelist();
#endif // HAS_BLE
#if HAS_WIRED_SENSORS
void createWiredSensorPayloads(const char* timestamp);
#endif // HAS_WIRED_SENSORS
#if HAS_BLE
void createPlaceholderPayloads(const char* timestamp);
#endif // HAS_BLE
void initDeviceId();
void goToSleep();
void updateDisplay();
void initOta();
void processOtaUpdate();
void handleMqttMessage(char* topic, byte* payload, unsigned int length);
void otaProgressCallback(const OtaProgress& progress);

// LED status indicator functions
#if HAS_LED
void ledInit();
void ledOn();
void ledOff();
void ledBlink(int count, int onTime, int offTime);
void ledStartup();
void ledWifiConnecting();
void ledWifiConnected();
void ledMqttConnecting();
void ledMqttConnected();
void ledHeartbeat();
void ledError();
void ledUpdate();

// LED state tracking — fully non-blocking via millis()
static unsigned long lastHeartbeatTime = 0;
static const unsigned long HEARTBEAT_INTERVAL = 5000; // 5 seconds

// Non-blocking LED animation state machine
struct LedAnimation {
  bool     active;
  int      totalBlinks;   // total blinks requested
  int      currentBlink;  // blink index (0..totalBlinks-1)
  unsigned long onTime;   // ms LED stays ON per blink
  unsigned long offTime;  // ms LED stays OFF between blinks
  bool     ledIsOn;       // current physical state
  unsigned long phaseStart; // when current on/off phase started
};
static LedAnimation ledAnim = {false, 0, 0, 0, 0, false, 0};

/**
 * @brief Initialize the LED pin for status indication
 */
void ledInit() {
  pinMode(PIN_LED_BUILTIN, OUTPUT);
  digitalWrite(PIN_LED_BUILTIN, !LED_ON); // Start with LED off
}

/**
 * @brief Turn the LED on (immediate)
 */
void ledOn() {
  digitalWrite(PIN_LED_BUILTIN, LED_ON);
}

/**
 * @brief Turn the LED off (immediate)
 */
void ledOff() {
  digitalWrite(PIN_LED_BUILTIN, !LED_ON);
}

/**
 * @brief Start a non-blocking LED blink animation
 * @param count Number of blinks
 * @param onTime Duration LED is on in milliseconds per blink
 * @param offTime Duration LED is off in milliseconds between blinks
 * 
 * The animation is driven by ledUpdate() which must be called from loop().
 */
void ledBlinkAsync(int count, int onTime, int offTime) {
  ledAnim.active       = true;
  ledAnim.totalBlinks  = count;
  ledAnim.currentBlink = 0;
  ledAnim.onTime       = onTime;
  ledAnim.offTime      = offTime;
  ledAnim.ledIsOn      = true;
  ledAnim.phaseStart   = millis();
  ledOn();  // Start first ON phase immediately
}

/**
 * @brief Advance the non-blocking LED animation state machine
 * 
 * Call this every loop() iteration. Returns immediately if no animation
 * is active. Transitions through ON→OFF→ON→… phases using millis().
 */
void ledUpdate() {
  if (!ledAnim.active) return;
  unsigned long now = millis();

  if (ledAnim.ledIsOn) {
    // Currently ON — wait for onTime to elapse
    if (now - ledAnim.phaseStart >= ledAnim.onTime) {
      ledOff();
      ledAnim.ledIsOn = false;
      ledAnim.phaseStart = now;
      ledAnim.currentBlink++;
    }
  } else {
    // Currently OFF — wait for offTime, then start next blink or finish
    if (now - ledAnim.phaseStart >= ledAnim.offTime) {
      if (ledAnim.currentBlink >= ledAnim.totalBlinks) {
        // Animation complete
        ledAnim.active = false;
        return;
      }
      // Start next ON phase
      ledOn();
      ledAnim.ledIsOn = true;
      ledAnim.phaseStart = now;
    }
  }
}

/**
 * @brief LED pattern for device startup (3 quick blinks, blocking — only in setup)
 */
void ledStartup() {
  // Blocking is acceptable here — runs once before loop()
  for (int i = 0; i < 3; i++) {
    ledOn();  delay(100);  ledOff();
    if (i < 2) delay(100);
  }
}

/**
 * @brief Start non-blocking WiFi-connecting LED animation (single fast blink)
 */
void ledWifiConnecting() {
  if (!ledAnim.active) {
    ledBlinkAsync(1, 100, 100);
  }
}

/**
 * @brief Start non-blocking WiFi-connected LED animation (solid ON 500ms)
 */
void ledWifiConnected() {
  ledBlinkAsync(1, 500, 0);
}

/**
 * @brief Start non-blocking MQTT-connecting LED animation (slow blink)
 */
void ledMqttConnecting() {
  if (!ledAnim.active) {
    ledBlinkAsync(1, 150, 150);
  }
}

/**
 * @brief Start non-blocking MQTT-connected LED animation (double blink)
 */
void ledMqttConnected() {
  ledBlinkAsync(2, 100, 100);
}

/**
 * @brief Non-blocking LED heartbeat — brief flash every 5 seconds
 * Called regularly from the main loop.
 */
void ledHeartbeat() {
  unsigned long currentTime = millis();
  if (currentTime - lastHeartbeatTime >= HEARTBEAT_INTERVAL) {
    ledBlinkAsync(1, 200, 0);
    lastHeartbeatTime = currentTime;
  }
}

/**
 * @brief Start non-blocking LED error animation (5 rapid blinks)
 */
void ledError() {
  ledBlinkAsync(5, 100, 100);
}
#endif // HAS_LED

/**
 * @brief Get ISO 8601 formatted timestamp string
 * 
 * Creates a timestamp in format: YYYY-MM-DDTHH:MM:SS±HH:MM
 * Uses ESP32 native time functions with automatic DST support
 * 
 * @return String containing ISO 8601 formatted timestamp
 */
String getISO8601Timestamp() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    // Return empty string to signal invalid timestamp (caller should not publish)
    return "";  // Fallback - empty indicates time not available
  }
  
  // Get both local and UTC time to calculate offset
  time_t now = time(nullptr);
  struct tm localTime;
  struct tm utcTime;
  localtime_r(&now, &localTime);
  gmtime_r(&now, &utcTime);
  
  // Calculate offset in seconds (local - UTC)
  time_t localSec = localTime.tm_hour * 3600 + localTime.tm_min * 60 + localTime.tm_sec;
  time_t utcSec = utcTime.tm_hour * 3600 + utcTime.tm_min * 60 + utcTime.tm_sec;
  long tzOffset = localSec - utcSec;
  
  // Handle day boundary crossing
  if (localTime.tm_mday != utcTime.tm_mday) {
    if (localTime.tm_mday > utcTime.tm_mday || (localTime.tm_mday == 1 && utcTime.tm_mday > 1)) {
      tzOffset += 86400;  // Add a day
    } else {
      tzOffset -= 86400;  // Subtract a day
    }
  }
  
  char tzSign = tzOffset >= 0 ? '+' : '-';
  int tzHours = abs(tzOffset) / 3600;
  int tzMinutes = (abs(tzOffset) % 3600) / 60;
  
  // Format as ISO 8601 with timezone: YYYY-MM-DDTHH:MM:SS±HH:MM
  char isoTimestamp[32];
  snprintf(isoTimestamp, sizeof(isoTimestamp), 
           "%04d-%02d-%02dT%02d:%02d:%02d%c%02d:%02d",
           timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
           timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec,
           tzSign, tzHours, tzMinutes);
  
  return String(isoTimestamp);
}

// ============================================================================
// GUI Action Callbacks — invoked from the LVGL task, safe to publish MQTT.
// Registered in setup() after gui_init().
// ============================================================================
#if HAS_RGB_LCD

static void on_gui_light_changed(uint8_t zone, bool on, uint8_t brightness, uint16_t colorTempK) {
#if HAS_MILIGHT
    if (!client.connected()) return;
    String topic = String("paku/edge/") + deviceId + "/cmd/light/" + (zone + 1);
    JsonDocument doc;
    doc["state"]      = on ? "ON" : "OFF";
    doc["brightness"] = brightness;
    // Convert Kelvin to Mired (153-500); clamp to valid HA range
    uint16_t mired = (colorTempK > 0) ? (uint16_t)(1000000UL / colorTempK) : 370;
    if (mired < 153) mired = 153;
    if (mired > 500) mired = 500;
    doc["color_temp"] = mired;
    String payload;
    serializeJson(doc, payload);
    client.publish(topic.c_str(), payload.c_str());
    LOG_INFO("GUI", "Light z%u %s bri=%u ct=%u", zone + 1, on ? "ON" : "OFF", brightness, mired);
#endif // HAS_MILIGHT
}

static void on_gui_fan_changed(bool power, uint8_t speed, bool dirIn, bool lidOpen) {
#if HAS_FAN_IR
    if (!client.connected()) return;
    String topic = String("paku/edge/") + deviceId + "/cmd/fan";
    JsonDocument doc;
    doc["power"]     = power;
    doc["speed"]     = speed;
    doc["direction"] = dirIn ? "intake" : "exhaust";
    doc["lid"]       = lidOpen ? "open" : "closed";
    String payload;
    serializeJson(doc, payload);
    client.publish(topic.c_str(), payload.c_str());
    LOG_INFO("GUI", "Fan on=%d spd=%u dir=%s lid=%s", power, speed,
             dirIn ? "in" : "out", lidOpen ? "open" : "closed");
#endif // HAS_FAN_IR
}

static void on_gui_heater_changed(bool on, HeaterMode mode, uint8_t powerLevel, uint8_t targetTempC) {
#ifdef HEATER_ENABLED
    if (!client.connected()) return;
    String topic = String("paku/heater/") + deviceId + "/cmd";
    JsonDocument doc;
    if (mode == HEATER_MODE_VENT) {
        doc["cmd"]   = "vent";
        doc["power"] = (powerLevel <= 9) ? powerLevel : 5;
    } else if (on) {
        doc["cmd"] = "start";
        if (mode == HEATER_MODE_POWER) {
            doc["mode"]  = "power";
            doc["power"] = (powerLevel <= 9) ? powerLevel : 5;
        } else {
            doc["mode"]        = "thermostat";
            doc["target_temp"] = targetTempC;
        }
    } else {
        doc["cmd"] = "stop";
    }
    String payload;
    serializeJson(doc, payload);
    client.publish(topic.c_str(), payload.c_str());
    LOG_INFO("GUI", "Heater %s mode=%s pwr=%u temp=%u",
             on ? "ON" : "OFF",
             mode == HEATER_MODE_POWER ? "power" : (mode == HEATER_MODE_VENT ? "vent" : "thermostat"),
             powerLevel, targetTempC);
#endif // HEATER_ENABLED
}

static void on_gui_backlight_changed(uint8_t percent) {
    waveshare_hal_set_backlight(percent);
}

static void on_gui_restart() {
    LOG_INFO("GUI", "Restarting device via GUI request");
    delay(200);
    ESP.restart();
}

// NOTE: Lighting and climate presets are now managed by the GUI preset system
// (gui_presets.h/cpp) with NVS persistence.  The home tab applies presets
// directly via the registered _cb_light / _cb_heater / _cb_fan callbacks.

#endif // HAS_RGB_LCD

// ============================================================================
/**
 * @brief Initializes the system setup.
 * 
 * This function performs the following setup tasks:
 * - Initializes serial communication at 115200 baud rate.
 * - Prints a message indicating the start of the setup process.
 * - Sets up the WiFi connection in station mode and connects to the network.
 * - Sets up the MQTT connection by initializing the time client and setting the MQTT server and port.
 * - Configures the sensor by setting the pin mode and attaching an interrupt to count rising edges.
 * - Initializes intervals based on the heater status.
 * - Prints a message indicating the completion of the setup process.
 */
void setup() {
    Serial.begin(115200);
    delay(100);  // Allow serial to initialize
    
    // Display logging configuration
    printLoggingConfig();
    
    LOG_INFO("System", "Starting setup...");
    LOG_INFO("System", "Device: %s", DEVICE_NAME);
    LOG_INFO("System", "Firmware: %s", FIRMWARE_VERSION);
    
#if HAS_LED
    // Initialize LED for status indication
    ledInit();
    ledStartup();  // 3 quick blinks to indicate startup
#endif

#if HAS_DISPLAY
    pinMode(PIN_POWER_ON, OUTPUT);
    digitalWrite(PIN_POWER_ON, HIGH);
    tft.begin();

#if defined(LCD_MODULE_CMD_1)
    for (uint8_t i = 0; i < (sizeof(lcd_st7789v) / sizeof(lcd_cmd_t)); i++) {
        tft.writecommand(lcd_st7789v[i].cmd);
        for (int j = 0; j < (lcd_st7789v[i].len & 0x7f); j++) {
            tft.writedata(lcd_st7789v[i].data[j]);
        }

        if (lcd_st7789v[i].len & 0x80) {
            delay(120);
        }
    }
#endif

    tft.setRotation(3);
    tft.setSwapBytes(true);
    tft.pushImage(0, 0, 320, 170, (uint16_t *)img_logo);
    delay(2000);

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5,0,0)
    ledcSetup(0, 2000, 8);
    ledcAttachPin(PIN_LCD_BL, 0);
    ledcWrite(0, 100);   // 40% brightness (reduced from 255 to lower heat)
#else
    ledcAttach(PIN_LCD_BL, 200, 8);
    ledcWrite(PIN_LCD_BL, 100);  // 40% brightness (reduced from 255 to lower heat)
#endif

    // Initialize the display UI system with button controls
    Serial.println("Initializing Display UI...");
    displayUI.begin(&tft);
    
    // Reduce CPU frequency to save power and reduce heat
    // 240 MHz default is massive overkill for sensor polling + MQTT
    // 80 MHz is the minimum that keeps WiFi + BLE working on ESP32-S3
    setCpuFrequencyMhz(80);
    Serial.print("CPU frequency set to: ");
    Serial.print(getCpuFrequencyMhz());
    Serial.println(" MHz (power saving mode)");
    
#endif // HAS_DISPLAY

#if HAS_RGB_LCD
    // Initialize Waveshare RGB LCD touchscreen GUI (LVGL)
    Serial.println("Initializing Waveshare GUI...");
    gui_init();
    gui_set_firmware_version(FIRMWARE_VERSION);
    // Register GUI action callbacks (GUI → MQTT / hardware)
    gui_on_light_changed(on_gui_light_changed);
    gui_on_fan_changed(on_gui_fan_changed);
    gui_on_heater_changed(on_gui_heater_changed);
    gui_on_backlight_changed(on_gui_backlight_changed);
    gui_on_restart(on_gui_restart);
    // Preset callbacks removed — home tab now applies presets directly
    // via the individual light/heater/fan callbacks registered above.
    Serial.println("Waveshare GUI initialized");

    // Show busy overlay while the rest of setup runs
    gui_show_busy("Starting up\xE2\x80\xA6");
    gui_update();  // Force render so overlay is visible during setup
#endif // HAS_RGB_LCD

    Serial.println("Setup Wifi Connection...");
    WiFi.mode(WIFI_STA);
#ifndef ESP8266
    // Enable WiFi modem sleep — radio powers down between DTIM beacons
    // Saves ~80 mA when idle, with negligible latency impact for MQTT
    WiFi.setSleep(true);
    // Reduce TX power from default 20 dBm to 8 dBm (~6 mW vs ~100 mW)
    // Sufficient for typical indoor range to the access point
    WiFi.setTxPower(WIFI_POWER_8_5dBm);
    Serial.println("WiFi power saving: modem sleep ON, TX power 8.5 dBm");
#endif
#ifdef ESP8266
    // ESP8266-specific WiFi settings for improved stability
    WiFi.persistent(false);  // Don't write WiFi settings to flash
    WiFi.setAutoConnect(false);  // Disable auto-connect on boot
    WiFi.disconnect(true);  // Clear any stored WiFi credentials
    delay(100);
#endif
    
    // Initialize WiFi manager with NVS persistence
    Serial.println("Initializing WiFi Manager...");
    wifiManager.begin();
    
    //connect_wifi();

    // Initialize device ID from MAC address
    initDeviceId();

    // Initialize device configuration BEFORE setting up time (needs timezone)
    Serial.println("Loading device configuration...");
    
    // Load persisted config if available, otherwise use defaults
    loadConfig();

    Serial.println("Setup MQTT Connection...");
    
    // Configure broker configs for failover manager
    // Primary = local HA (MQTT_LOCAL), Fallback = cloud (MQTT_SERVER)
    static const MqttBrokerConfig primaryBroker = {
        MQTT_LOCAL,                               // host (homeassistant.local)
        static_cast<uint16_t>(MQTT_LOCAL_PORT),   // port (1883)
        MQTT_LOCAL_USER,                          // user
        MQTT_LOCAL_PASSWORD,                      // password
        (MQTT_LOCAL_PORT == 8883),                // useTls
#if MQTT_LOCAL_PORT == 8883 && !defined(ESP8266)
        MQTT_CA_CERT                              // caCert
#else
        nullptr                                   // caCert (no TLS)
#endif
    };
    
    static const MqttBrokerConfig fallbackBroker = {
        MQTT_SERVER,                              // host (cloud)
        static_cast<uint16_t>(MQTT_PORT),         // port (8883)
        MQTT_USER,                                // user
        MQTT_PASSWORD,                            // password
        (MQTT_USE_TLS != 0),                      // useTls
#if MQTT_USE_TLS && !defined(ESP8266)
        MQTT_CA_CERT                              // caCert
#else
        nullptr                                   // caCert
#endif
    };

    // Log broker configuration
    Serial.printf("  Primary broker: %s:%d\n", primaryBroker.host, primaryBroker.port);
    if (strlen(MQTT_SERVER) > 0) {
        Serial.printf("  Fallback broker: %s:%d\n", fallbackBroker.host, fallbackBroker.port);
    } else {
        Serial.println("  Fallback broker: not configured");
    }

    client.setCallback(handleMqttMessage);  // Set MQTT message callback
    client.setBufferSize(1024);  // Increase from default 256 bytes to handle larger config messages
    
    // Initialize MQTT broker failover manager.
    // When HA integration is disabled, skip local broker discovery by using
    // the cloud broker as both primary and fallback.
    if (deviceConfig.ha.enabled) {
        mqttMgr.begin(client, deviceId, primaryBroker, fallbackBroker, onMqttConnect);
    } else {
        Serial.println("  HA integration disabled — connecting cloud-only");
        mqttMgr.begin(client, deviceId, fallbackBroker, fallbackBroker, onMqttConnect);
    }

#ifndef ESP8266
    // Initialize mDNS for local broker resolution (homeassistant.local)
    if (!MDNS.begin("paku-edge")) {
        Serial.println("  mDNS responder failed to start");
    } else {
        Serial.println("  mDNS responder started (paku-edge.local)");
    }
#endif

    // Configure ESP32 native time with timezone support (auto DST)
    // NTP will sync in the background — no blocking wait.
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    setenv("TZ", deviceConfig.timing.timezone, 1);
    tzset();
    Serial.print("Timezone configured: ");
    Serial.println(deviceConfig.timing.timezone);
    Serial.println("NTP sync will complete in background (non-blocking)");

    // Initialize paku-iot HTTP client if enabled
    initPakuIot();

    // Initialize OTA update client
    Serial.println("Setup OTA Update Client...");
    initOta();

    // Initialize RuuviTag scanner
#if HAS_BLE
    // Initialize RuuviTag scanner (ESP32 only)
    Serial.println("Setup RuuviTag Scanner...");
    initRuuviTags();
    
    // Initialize MoKo sensor scanner (ESP32 only)
    Serial.println("Setup MoKo Sensor Scanner...");
    initMoKoSensors();

    // Initialize Frezzer controller
    Serial.println("Setup Frezzer Controller...");
    initFrezzerDevices();
#endif // HAS_BLE

#if HAS_WIRED_SENSORS
    // Initialize DS18B20 sensor (ESP8266 and ESP32)
    Serial.println("Setup DS18B20 Temperature Sensor...");
    if (wiredSensors.begin(0, 0)) {  // Parameters unused for DS18B20
        Serial.print("Sensor type: ");
        Serial.println(wiredSensors.getSensorType());
    } else {
        Serial.println("Warning: No DS18B20 sensors detected");
        wiredSensorsEnabled = false;
    }
#endif // HAS_WIRED_SENSORS

    // Flow sensor disabled - awaiting hardware integration and calibration
    // TODO: Enable flow sensor when ready (see lib/paku_lib/src/flow_sensor.h)
    Serial.println("Flow Sensor: DISABLED (future development)");
    
#if HAS_MILIGHT
    if (deviceConfig.peripherals.milight) {
        Serial.println("Setup MiLight Light Controller...");
        if (milight_init(PIN_NRF24_CE, PIN_NRF24_CSN, PIN_NRF24_SCK, PIN_NRF24_MOSI, PIN_NRF24_MISO)) {
            Serial.println("  MiLight controller initialized");
            milightActive = true;
        } else {
            Serial.println("  Warning: MiLight initialization failed");
        }
    } else {
        Serial.println("MiLight: disabled by peripheral config");
    }
#endif // HAS_MILIGHT
    
    // Apply scenario based on heater status (Phase 1: maintain existing behavior)
    if (heaterStatus == 1) {
        deviceConfig.applyScenario("heater_active");
    } else {
        deviceConfig.applyScenario("default");
    }
    
    // Initialize intervals based on heater status (legacy behavior for Phase 1)
    updateIntervals();
    
    // Set initial state for continuous operation (no sleep in Phase 1)
    currentState = STATE_CONTINUOUS;
    
    // Phase 2: Initialize state machine
    currentSystemState = SYS_STATE_IDLE;
    stateEnteredAt = millis();
    lastSensorCollection = 0;
    lastNetworkConnect = 0;
    
    Serial.printf("Wake interval: %d seconds\n", deviceConfig.timing.wake_interval_s);
    Serial.printf("BLE scan duration: %d seconds\n", deviceConfig.sensors.ble.scan_duration_s);
    Serial.printf("Deep sleep: %s\n", deviceConfig.power.deep_sleep_enabled ? "enabled" : "disabled");
    Serial.println("Phase 2: State machine initialized");
    
#if HAS_BLE
    // Create a task for scanning Bluetooth devices (ESP32 only)
    xTaskCreate(
      scanBT,          // Function to be called
      "scanBT",        // Name of the task
      10000,           // Stack size (bytes)
      NULL,            // Parameter to pass
      1,               // Task priority
      NULL             // Task handle
    );
#endif // HAS_BLE

#ifdef HEATER_ENABLED
    if (deviceConfig.peripherals.heater) {
        heater_addon_setup();
        heaterAddonActive = true;
        Serial.println("Heater add-on initialized");
    } else {
        Serial.println("Heater: disabled by peripheral config");
    }
#endif

#if HAS_FAN_IR
    if (deviceConfig.peripherals.fan_ir) {
        maxxfan_ir_init(PIN_IR_LED);
        fanIrActive = true;
        Serial.println("MaxxFan IR initialized");
    } else {
        Serial.println("MaxxFan IR: disabled by peripheral config");
    }
#endif

    Serial.println("Setup complete.");

#if HAS_RGB_LCD
    gui_hide_busy();
#endif

  }

/**
 * @brief Main loop function that handles WiFi and MQTT connections, sensor data processing, 
 *        and payload creation for various metrics.
 * 
 * This function performs the following tasks:
 * - Checks and maintains WiFi connection.
 * - Checks and maintains MQTT connection.
 * - Updates the time client.
 * - Processes sensor data from BLE and wired sensors.
 * - Creates payloads for temperature, humidity, and other metrics.
 * - Sends payloads to MQTT broker at specified intervals.
 * 
 * The function uses the following global variables:
 * - WiFi: WiFi connection object.
 * - client: MQTT client object.
 * - timeClient: NTP time client object.
 * - lastTime_sensor: Timestamp of the last sensor data processing.
 * - sensorInterval: Interval for sensor data processing.
 * - lastTime_mqtt: Timestamp of the last MQTT data sending.
 * - mqttInterval: Interval for sending data to MQTT broker.
 * - payloads: Array of payload objects to be sent to MQTT broker.
 * - payloadIndex: Index for the payload array.
 * 
 * The function uses the following external functions:
 * - connect_wifi(): Connects to WiFi.
 * - connectMQTT(): Connects to MQTT broker.
 * - updateIntervals(): Updates intervals based on heater status.
 * - createPayload(): Creates a payload for a given topic, value, and timestamp.
 */
/**
 * @brief Main loop function that handles WiFi and MQTT connections, updates time, 
 *        processes sensor data, and sends data to MQTT.
 * 
 * This function performs the following tasks:
 * - Checks and maintains WiFi connection.
 * - Checks and maintains MQTT connection.
 * - Calls the loop function of the MQTT client.
 * - Updates the time using the time client.
 * - Updates the heater status (placeholder for actual implementation).
 * - Updates intervals based on the heater status.
 * - Processes sensor data.
 * - Sends data to the MQTT broker.
 */
/**
 * @brief Main loop function - Phase 2 with State Machine
 * 
 * Manages system state transitions:
 * - IDLE: Wait for sensor collection interval
 * - COLLECT_SENSORS: Read sensors (WiFi OFF)
 * - CONNECT_NETWORK: Turn on WiFi and connect MQTT
 * - TRANSMIT: Send buffered data
 * - DISCONNECT: Turn off WiFi
 */
void loop() {
#if HAS_DISPLAY
  // CRITICAL: Update display UI FIRST to ensure button responsiveness
  // Must be before any potentially blocking operations
  displayUI.update();
#endif

#if HAS_RGB_LCD
  // CRITICAL: Run LVGL timer handler FIRST for touch responsiveness
  gui_update();

  // Push periodic status data to the GUI (~1 Hz)
  {
    static unsigned long lastGuiStatusPush = 0;
    if (millis() - lastGuiStatusPush >= 1000) {
      lastGuiStatusPush = millis();
      gui_set_wifi_status(WiFi.status() == WL_CONNECTED, WiFi.RSSI());
      gui_set_mqtt_status(client.connected());
#if HAS_BLE
      gui_set_ble_status(scanBT_enabled);

      // Push latest Ruuvi sensor readings to the GUI graphs
      {
        const RuuviTag* freshTags[MAX_RUUVI_TAGS];
        uint8_t freshCount = getFreshTags(freshTags, MAX_RUUVI_TAGS, millis());
        for (uint8_t i = 0; i < freshCount && i < 4; i++) {
          const RuuviTag* tag = freshTags[i];
          if (!tag->hasData || !tag->lastData.valid) continue;
          // Map tag index to GUI series (0-3 for temp, 0-2 for humidity)
          gui_push_temperature(i, tag->lastData.temperature);
          if (i < 3) gui_push_humidity(i, tag->lastData.humidity);
          // Push key readings to the header bar
          // Series 0 = Indoor, 1 = Outdoor  (matches sensor graph convention)
          if (i == 0) gui_set_header_indoor(tag->lastData.temperature, tag->lastData.humidity);
          if (i == 1) gui_set_header_outdoor(tag->lastData.temperature, tag->lastData.humidity);
        }
      }

      // Push fridge temperature to header
      {
        const FrezzerDevice* fDevices[MAX_FREZZER_DEVICES];
        uint8_t fCount = getFreshFrezzerDevices(fDevices, MAX_FREZZER_DEVICES, millis());
        if (fCount > 0 && fDevices[0]->hasData && fDevices[0]->lastData.valid) {
          gui_set_header_fridge(fDevices[0]->lastData.currentTemp);
        }
      }
#endif
#ifdef HEATER_ENABLED
      if (heaterAddonActive) {
        JsonDocument hDoc;
        heater_addon_telemetry(hDoc);
        const char* stateStr = hDoc["heater"]["state"] | "unknown";
        int hState = 0;  // 0=OFF, 1=STARTING, 2=ON, 3=STOPPING
        if (strcmp(stateStr, "Starting") == 0 || strcmp(stateStr, "Warming") == 0)
          hState = 1;
        else if (strcmp(stateStr, "Running") == 0)
          hState = 2;
        else if (strcmp(stateStr, "Shutting Down") == 0 || strcmp(stateStr, "Cooling") == 0)
          hState = 3;
        float coolant = hDoc["heater"]["coolantTemp"] | 0.0f;
        gui_set_heater_data(hState, -1, coolant);
      }
#endif
      // Push time string from NTP (if available)
      struct tm timeinfo;
      if (getLocalTime(&timeinfo, 0)) {
        char timeBuf[6];
        strftime(timeBuf, sizeof(timeBuf), "%H:%M", &timeinfo);
        gui_set_time(timeBuf);
      }
    }
  }
#endif

#ifdef HEATER_ENABLED
  if (heaterAddonActive) heater_addon_loop();
#endif

  // Phase 2: State machine-based operation
  handleSystemState();
  
#if HAS_LED
  // Drive non-blocking LED animations (must be called every iteration)
  ledUpdate();
  // LED heartbeat - brief flash every 5 seconds during normal operation
  ledHeartbeat();
#endif
  
  // Small delay to prevent busy-spinning the CPU.
  // Without this the loop runs thousands of times/sec, keeping the CPU
  // at 100% and generating significant heat.  10 ms still gives ~100 Hz
  // responsiveness for buttons/display while dropping CPU load to ~2%.
  delay(10);
}

void goToSleep() {
#if HAS_DISPLAY
    // Only enable sleep for devices with display (ESP32-S3)
    // ESP8266 doesn't wake from deep sleep without D0->RST connection
    // Check if the device has been awake for 30 seconds
    static unsigned long awakeStartTime = millis();
    if (millis() - awakeStartTime >= 30000) {
        Serial.println("Going to sleep for 15 seconds...");
        tft.fillScreen(TFT_BLACK);
        tft.setCursor(0, 0);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setTextSize(2);
        tft.println("Going to sleep for 15 seconds...");
        delay(2000);
        
        // Configure deep sleep to wake up after 15 seconds
#ifdef ESP8266
        ESP.deepSleep(15 * 1000000);  // ESP8266 deep sleep (microseconds)
#else
        esp_sleep_enable_timer_wakeup(15 * 1000000);
        esp_deep_sleep_start();
#endif
    } else {
        delay(1000);
    }
#else
    // No display = no sleep (e.g., ESP8266 headless, ESP32 headless)
    // Just keep running and reading sensors
    delay(100);
#endif
}

void updateDisplay() {
#if HAS_DISPLAY
  // DEPRECATED: Legacy display function replaced by DisplayUI class
  // This function is kept for backward compatibility but does nothing
  // The new displayUI.update() in loop() handles all display operations
  // If you need to force a display refresh, use: displayUI.refresh()
#endif // HAS_DISPLAY
}

/**
 * @brief Processes flow data and updates various sensor readings.
 * 
 * This function is called periodically to process flow data from a sensor.
 * It calculates the flow rate, required temperature delta, and creates payloads
 * for various sensor readings including humidity, temperature, flow, heating power,
 * battery voltage, and heater status.
 * 
 * The function performs the following steps:
 * - Checks if the sensor interval has elapsed.
 * - Detaches the interrupt to process the flow data safely.
 * - In test mode, generates a random count value.
 * - Calculates the frequency of pulses and the flow rate.
 * - Calculates the required temperature delta for the heater.
 * - Resets the count and updates the last sensor time.
 * - Reattaches the interrupt for the sensor.
 * - Creates payloads for humidity, temperature, flow, heating power, battery voltage, and heater status.
 * 
 * @note The function assumes the presence of global variables and functions such as `millis()`, 
 * `Serial.print()`, `detachInterrupt()`, `attachInterrupt()`, `random()`, `getISO8601Timestamp()`, 
 * and `createPayload()`.
 */
void processData() {
  unsigned long currentTime = millis();
  if (currentTime - lastTime_sensor >= sensorInterval) {
    Serial.print(".");
 
    // Flow sensor processing disabled - moved to flow_sensor.cpp
    // TODO: Re-enable when flow sensor hardware is ready
    String timestamp = getISO8601Timestamp();
 
    // Create payloads for all data
    
#if HAS_BLE
    // 1. RuuviTag sensor data (temperature, humidity, pressure from BLE sensors)
    createRuuviPayloads(timestamp.c_str());
    
    // 2. MoKo sensor data (temperature, humidity, pressure, accelerometer from BLE sensors)
    createMoKoPayloads(timestamp.c_str());

    // 3. Frezzer fridge data (temperature, status from BLE compressor fridges)
    createFrezzerPayloads(timestamp.c_str());
#endif // HAS_BLE
    
#if HAS_WIRED_SENSORS
    // 2. DS18B20 sensor data (ESP8266 and ESP32)
    createWiredSensorPayloads(timestamp.c_str());
#endif // HAS_WIRED_SENSORS
    
#if HAS_BLE
    // 3. Placeholder data for sensors not yet implemented (disabled by default)
    createPlaceholderPayloads(timestamp.c_str());
#endif // HAS_BLE
    
    // Flow sensor data disabled - future development
    // TODO: Re-enable flow sensor payload generation when hardware ready
    // See lib/paku_lib/src/flow_sensor.h for flow sensor module
 }
}

/**
 * @brief Sends payload data to the MQTT broker if the specified interval has elapsed.
 *
 * This function checks if the current time has surpassed the last time data was sent to the MQTT broker
 * by the defined interval. If so, it iterates through the payloads and publishes each one to the MQTT broker.
 * After sending the data, it resets the payload index and updates the last time data was sent.
 *
 * @note This function assumes that `currentTime`, `lastTime_mqtt`, `mqttInterval`, `payloadIndex`, 
 *       `payloads`, and `client` are defined and accessible in the scope where this function is used.
 */
void sendToMQTT() {
  unsigned long currentTime = millis();

  if (currentTime - lastTime_mqtt >= mqttInterval) {
    Serial.print("[DEBUG] sendToMQTT - payloadIndex before transmission: ");
    Serial.println(payloadIndex);
    
    if (payloadIndex == 0) {
      Serial.println("[DEBUG] No payloads to send to MQTT!");
    }
    
    for (int i = 0; i < payloadIndex; i++) {
      Serial.print("[DEBUG] Publishing payload ");
      Serial.print(i);
      Serial.print(" to topic: ");
      Serial.println(payloads[i].topic);
      Serial.print("[DEBUG] Data: ");
      Serial.println(payloads[i].data);
      bool result = client.publish((char*) payloads[i].topic.c_str(), (char*) payloads[i].data.c_str());
      Serial.print("[DEBUG] Publish result: ");
      Serial.println(result ? "SUCCESS" : "FAILED");
    }
    
    payloadIndex = 0;
    lastTime_mqtt = currentTime;
  }
}

/**
 * @brief Initializes the paku-iot HTTP client.
 * 
 * This function configures and initializes the PakuIotClient for sending
 * telemetry data to paku-iot via HTTP/HTTPS. The client is only initialized
 * if PAKU_IOT_ENABLED is set to 1 in secrets.h.
 * 
 * Configuration is read from secrets.h:
 * - PAKU_IOT_HOST: Server hostname
 * - PAKU_IOT_PORT: Server port
 * - PAKU_IOT_API_KEY: API key for authentication
 * - PAKU_IOT_USE_TLS: Whether to use HTTPS
 */
void initPakuIot() {
#if PAKU_IOT_ENABLED
  Serial.println("Initializing paku-iot client...");
  
  // Generate device ID from MAC address
  uint8_t mac[6];
  WiFi.macAddress(mac);
  static char deviceId[20];
#ifdef ESP8266
  snprintf(deviceId, sizeof(deviceId), "ESP8266-%02X%02X%02X%02X",
           mac[2], mac[3], mac[4], mac[5]);
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
  snprintf(deviceId, sizeof(deviceId), "ESP32-S3-%02X%02X%02X%02X",
           mac[2], mac[3], mac[4], mac[5]);
#elif defined(CONFIG_IDF_TARGET_ESP32)
  snprintf(deviceId, sizeof(deviceId), "ESP32-%02X%02X%02X%02X",
           mac[2], mac[3], mac[4], mac[5]);
#else
  snprintf(deviceId, sizeof(deviceId), "UNKNOWN-%02X%02X%02X%02X",
           mac[2], mac[3], mac[4], mac[5]);
#endif
  
  PakuIotConfig config;
  config.host = PAKU_IOT_HOST;
  config.port = PAKU_IOT_PORT;
  config.apiKey = PAKU_IOT_API_KEY;
  config.useTls = PAKU_IOT_USE_TLS;
  config.deviceId = deviceId;
  config.timeoutMs = 10000;
  config.maxRetries = 3;
  config.retryDelayMs = 1000;
  
  PakuIotResult result = pakuIotClient.begin(config);
  if (result == PakuIotResult::SUCCESS) {
    Serial.println("paku-iot client initialized successfully");
    Serial.print("Device ID: ");
    Serial.println(deviceId);
  } else {
    Serial.print("Failed to initialize paku-iot client: ");
    Serial.println(PakuIotClient::getErrorMessage(result));
  }
#else
  Serial.println("paku-iot client disabled (set PAKU_IOT_ENABLED=1 in secrets.h to enable)");
#endif
}

/**
 * @brief Sends telemetry data to paku-iot via HTTP.
 * 
 * This function sends the current sensor readings to paku-iot using the
 * HTTP transport. It is called periodically based on pakuIotInterval.
 * 
 * The function also processes any queued messages that failed to send
 * in previous attempts.
 * 
 * @note This function only operates if PAKU_IOT_ENABLED is set to 1.
 */
void sendToPakuIot() {
#if PAKU_IOT_ENABLED
  unsigned long currentTime = millis();
  
  // Process any queued messages first
  if (pakuIotClient.isReady()) {
    size_t sent = pakuIotClient.processQueue();
    if (sent > 0) {
      Serial.print("Sent ");
      Serial.print(sent);
      Serial.println(" queued messages to paku-iot");
    }
  }
  
  // Send current data at interval
  if (currentTime - lastTime_pakuIot >= pakuIotInterval) {
    Serial.println("Sending to paku-iot...");
    
    // Store timestamp in a static buffer to ensure pointer validity
    static char timestampBuf[32];
    String timestamp = getISO8601Timestamp();
    
    // Skip publishing if time is not available (empty string indicates NTP not synced)
    if (timestamp.length() == 0) {
      Serial.println("paku-iot: Skipping - time not synced yet");
      lastTime_pakuIot = currentTime;
      return;
    }
    
    strncpy(timestampBuf, timestamp.c_str(), sizeof(timestampBuf) - 1);
    timestampBuf[sizeof(timestampBuf) - 1] = '\0';
    
    // Create batch of readings - size matches MAX_TELEMETRY_READINGS constant
    TelemetryReading readings[MAX_TELEMETRY_READINGS];
    size_t readingCount = 0;
    
    // Add RuuviTag readings
    // Note: Static buffers are safe here as sendToPakuIot is called from main loop
    // and the BLE scan task does not call this function
    const RuuviTag* freshTags[MAX_RUUVI_TAGS];
    uint8_t freshCount = getFreshTags(freshTags, MAX_RUUVI_TAGS, millis());
    
    // Limit Ruuvi readings to leave room for other metrics
    const size_t maxRuuviReadings = (MAX_TELEMETRY_READINGS - 4) / 2;  // 2 readings per tag, reserve 4 for flow/status
    for (uint8_t i = 0; i < freshCount && i < maxRuuviReadings; i++) {
      const RuuviTag* tag = freshTags[i];
      if (!tag->hasData || !tag->lastData.valid) continue;
      
      // Static buffers for metric names (must persist during sendBatch call)
      // Safe because this function is only called from main loop
      static char tempMetrics[MAX_RUUVI_TAGS][64];
      static char humidMetrics[MAX_RUUVI_TAGS][64];
      
      snprintf(tempMetrics[i], sizeof(tempMetrics[i]), "temperature/ruuvi/%s", tag->location);
      snprintf(humidMetrics[i], sizeof(humidMetrics[i]), "humidity/ruuvi/%s", tag->location);
      
      readings[readingCount].metric = tempMetrics[i];
      readings[readingCount].value = tag->lastData.temperature;
      readings[readingCount].unit = "celsius";
      readings[readingCount].timestamp = timestampBuf;
      readingCount++;
      
      readings[readingCount].metric = humidMetrics[i];
      readings[readingCount].value = tag->lastData.humidity;
      readings[readingCount].unit = "percent";
      readings[readingCount].timestamp = timestampBuf;
      readingCount++;
    }
    
    // Flow sensor disabled - future development
    // TODO: Re-enable flow readings when hardware is ready
    // if (FLOW_SENSOR_ENABLED && flowRate > 0) {
    //   readings[readingCount].metric = "flow/coolant";
    //   readings[readingCount].value = flowRate;
    //   readings[readingCount].unit = "l_per_min";
    //   readings[readingCount].timestamp = timestampBuf;
    //   readingCount++;
    // }
    //
    // if (FLOW_SENSOR_ENABLED && requiredDeltaT > 0 && requiredDeltaT < 1000) {
    //   readings[readingCount].metric = "temperature/heating/required_dt";
    //   readings[readingCount].value = requiredDeltaT;
    //   readings[readingCount].unit = "celsius";
    //   readings[readingCount].timestamp = timestampBuf;
    //   readingCount++;
    // }
    
    // Add heater status
    readings[readingCount].metric = "status/heater";
    readings[readingCount].value = (float)heaterStatus;
    readings[readingCount].unit = nullptr;
    readings[readingCount].timestamp = timestampBuf;
    readingCount++;
    
    if (readingCount > 0) {
      PakuIotResult result = pakuIotClient.sendBatch(readings, readingCount);
      if (result == PakuIotResult::SUCCESS) {
        Serial.println("paku-iot: Data sent successfully");
      } else {
        Serial.print("paku-iot: Failed to send - ");
        Serial.println(PakuIotClient::getErrorMessage(result));
        Serial.print("Queued messages: ");
        Serial.println(pakuIotClient.getQueueSize());
      }
    }
    
    lastTime_pakuIot = currentTime;
  }
#endif
}

/**
 * @brief Establishes a WiFi connection by scanning for available networks first.
 * 
 * Strategy:
 * 1. Scan for available networks
 * 2. Try to connect to any matching network (NVS or firmware) that's in range
 * 3. If all available networks fail, rescan and retry (max 2 scan attempts)
 * 
 * @note Networks from NVS and firmware defaults are treated equally - no priority order.
 *       The function will connect to whichever available network succeeds first.
 * 
 * @warning Ensure that the firmware networks in secrets.h are properly defined.
 */
void connect_wifi() {
  
  delay(10);
  Serial.println();

#ifdef ESP8266
  // ESP8266-specific WiFi initialization for better connectivity
  WiFi.persistent(false);  // Don't write to flash on every connection
  WiFi.setAutoConnect(false);  // Disable auto-connect
  WiFi.setAutoReconnect(true);  // Enable auto-reconnect after connection
#endif

  // Build list of all configured networks (NVS + firmware)
  struct NetworkConfig {
    String ssid;
    String password;
    String source;  // "NVS" or "firmware"
  };
  
  NetworkConfig configuredNetworks[MAX_NVS_WIFI_NETWORKS + WIFI_COUNT];
  int configuredCount = 0;
  
  // Add NVS networks
  int nvsCount = wifiManager.getStoredNetworkCount();
  for (int i = 0; i < nvsCount && configuredCount < (MAX_NVS_WIFI_NETWORKS + WIFI_COUNT); i++) {
    WiFiCredential cred = wifiManager.getNetwork(i);
    if (cred.valid) {
      configuredNetworks[configuredCount].ssid = String(cred.ssid);
      configuredNetworks[configuredCount].password = String(cred.password);
      configuredNetworks[configuredCount].source = "NVS";
      configuredCount++;
    }
  }
  
  // Add firmware networks
  for (int i = 0; i < WIFI_COUNT && configuredCount < (MAX_NVS_WIFI_NETWORKS + WIFI_COUNT); i++) {
    configuredNetworks[configuredCount].ssid = String(WIFI_SSIDS[i]);
    configuredNetworks[configuredCount].password = String(WIFI_PASSWORDS[i]);
    configuredNetworks[configuredCount].source = "firmware";
    configuredCount++;
  }
  
  Serial.printf("Total configured networks: %d\n", configuredCount);
  
  // Try up to 2 scan cycles
  for (int scanCycle = 0; scanCycle < 2 && WiFi.status() != WL_CONNECTED; scanCycle++) {
    if (scanCycle > 0) {
      Serial.println("Rescanning for networks...");
      delay(1000);  // Wait before rescanning
    }
    
    // Scan for available networks
    Serial.println("Scanning for WiFi networks...");
    wifi_status = "Scanning WiFi...";
#if HAS_DISPLAY
    displayUI.refresh();
#endif
    
    int networksFound = WiFi.scanNetworks();
    Serial.printf("Found %d networks\n", networksFound);
    
    if (networksFound == 0) {
      Serial.println("No networks found in range");
      continue;
    }
    
    // Log all found networks
    Serial.println("Available networks:");
    for (int j = 0; j < networksFound; j++) {
      Serial.printf("  %d: %s (RSSI: %d, Ch: %d, Enc: %d)\n", 
                   j, WiFi.SSID(j).c_str(), WiFi.RSSI(j), WiFi.channel(j), WiFi.encryptionType(j));
    }
    
    // Log configured networks
    Serial.println("Configured networks:");
    for (int i = 0; i < configuredCount; i++) {
      Serial.printf("  %d: %s (%s)\n", i, configuredNetworks[i].ssid.c_str(), configuredNetworks[i].source.c_str());
    }
    
    // Try to connect to any available configured network
    for (int i = 0; i < configuredCount && WiFi.status() != WL_CONNECTED; i++) {
      // Check if this network is in range
      bool networkAvailable = false;
      for (int j = 0; j < networksFound; j++) {
        if (WiFi.SSID(j) == configuredNetworks[i].ssid) {
          networkAvailable = true;
          Serial.printf("Found configured network: %s (RSSI: %d)\n", 
                       configuredNetworks[i].ssid.c_str(), WiFi.RSSI(j));
          break;
        }
      }
      
      if (!networkAvailable) {
        continue;  // Skip networks not in range
      }
      
      // Try to connect
      Serial.printf("Connecting to %s (%s)...\n", 
                   configuredNetworks[i].ssid.c_str(), 
                   configuredNetworks[i].source.c_str());
      wifi_status = "Connecting to ";
      wifi_status += configuredNetworks[i].ssid;
      
      // Disconnect and clear any previous connection state
      WiFi.disconnect();
      delay(100);
      
      WiFi.begin(configuredNetworks[i].ssid.c_str(), configuredNetworks[i].password.c_str());
      int attempts = 0;

      while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        wifi_status += ".";
        attempts++;
#if HAS_LED
        ledWifiConnecting();
#endif
#if HAS_DISPLAY
        displayUI.update();
        displayUI.refresh();
#endif
      }

      if (WiFi.status() == WL_CONNECTED) {
        LOG_INFO("WiFi", "Connected to %s (%s)", 
                configuredNetworks[i].ssid.c_str(), 
                configuredNetworks[i].source.c_str());
        LOG_INFO("WiFi", "IP address: %s", WiFi.localIP().toString().c_str());
        LOG_DEBUG_WIFI("RSSI: %d dBm", WiFi.RSSI());
        
        wifi_status = "WiFi connected to ";
        wifi_status += configuredNetworks[i].ssid;
        wifi_status += " (";
        wifi_status += WiFi.localIP().toString();
        wifi_status += ")";
#if HAS_LED
        ledWifiConnected();
#endif
#if HAS_DISPLAY
        displayUI.refresh();
#endif
#if HAS_RGB_LCD
        gui_set_wifi_status(true, WiFi.RSSI());
#endif
        return;
      } else {
        Serial.printf("Failed to connect to %s\n", configuredNetworks[i].ssid.c_str());
#if HAS_LED
        ledError();
#endif
      }
    }
  }
  
  // All connection attempts failed
  Serial.println("Failed to connect to any WiFi network after scanning");
  wifi_status = "WiFi connection failed";
#if HAS_DISPLAY
  displayUI.refresh();
#endif
}

/**
 * @brief Callback invoked by MqttManager after successful broker connection.
 *
 * Subscribes to all required MQTT topics and publishes initial status.
 * Called for both local (RPi) and cloud (fallback) broker connections.
 *
 * @param mqttClient  Reference to the connected PubSubClient
 * @param broker      Which broker was connected (PRIMARY_LOCAL or FALLBACK_CLOUD)
 */
void onMqttConnect(PubSubClient& mqttClient, MqttBroker broker) {
    LOG_INFO("MQTT", "Connected to %s broker — subscribing to topics",
             (broker == MqttBroker::PRIMARY_LOCAL) ? "LOCAL" : "CLOUD");

    // Subscribe to legacy control topic (backward compatibility)
    mqttClient.subscribe("paku/control");
    
    // Subscribe to new edge device control topic (schema-compliant)
    String edgeControlTopic = String("paku/edge/") + deviceId + "/control";
    mqttClient.subscribe(edgeControlTopic.c_str());
    
    // Phase 2: Subscribe to config/set topic
    String edgeConfigTopic = String("paku/edge/") + deviceId + "/config/set";
    mqttClient.subscribe(edgeConfigTopic.c_str());
    
    // Subscribe to OTA command topic
    String otaTopic = String("paku/edge/") + deviceId + "/cmd/ota";
    mqttClient.subscribe(otaTopic.c_str());
    
    // Subscribe to WiFi management command topic
    String wifiCmdTopic = String("paku/devices/") + deviceId + "/cmd/wifi";
    mqttClient.subscribe(wifiCmdTopic.c_str());

#if HAS_BLE
    // Subscribe to Ruuvi whitelist management command topic
    String ruuviCmdTopic = String("paku/devices/") + deviceId + "/cmd/ruuvi";
    mqttClient.subscribe(ruuviCmdTopic.c_str());
#endif

#ifdef HEATER_ENABLED
    String heaterCmdTopic = String("paku/heater/") + deviceId + "/cmd";
    mqttClient.subscribe(heaterCmdTopic.c_str());
#endif

#if HAS_FAN_IR
    String fanCmdTopic = String("paku/edge/") + deviceId + "/cmd/fan";
    mqttClient.subscribe(fanCmdTopic.c_str());
#endif

#if HAS_MILIGHT
    for (int ch = 1; ch <= 4; ch++) {
      String lightCmdTopic = String("paku/edge/") + deviceId + "/cmd/light/" + ch;
      mqttClient.subscribe(lightCmdTopic.c_str());
      Serial.printf("  Subscribed to %s\n", lightCmdTopic.c_str());
    }
    {
      String lightAllTopic = String("paku/edge/") + deviceId + "/cmd/light/all";
      mqttClient.subscribe(lightAllTopic.c_str());
      Serial.printf("  Subscribed to %s\n", lightAllTopic.c_str());
    }
#endif

#if HAS_RGB_LCD
    // Subscribe to EcoFlow power data for the Power tab
    {
      const char* ecoflowTopic = "paku/ecoflow/+/power";
      mqttClient.subscribe(ecoflowTopic);
      Serial.printf("  Subscribed to %s\n", ecoflowTopic);
    }
    // Subscribe to sensor data from data-acquisition boards for GUI display.
    // Waveshare boards have HAS_BLE=0 so they receive sensor readings via MQTT
    // rather than by scanning BLE directly.
    mqttClient.subscribe("paku/sensors/+/data");
    mqttClient.subscribe("paku/heater/+/data");
    Serial.println("  Subscribed to paku/sensors/+/data (GUI sensor feed)");
    Serial.println("  Subscribed to paku/heater/+/data (GUI heater feed)");
#endif

    // Publish device status (announce we're online)
    publishDeviceStatus();

#if HAS_RGB_LCD
    gui_set_mqtt_status(true);
#endif
}

/**
 * @brief Attempts to establish a connection to the MQTT broker.
 * 
 * Uses MqttManager for automatic broker failover between local RPi and cloud.
 * This function continuously calls mqttMgr.maintain() until connected.
 * 
 * @note This function blocks execution until a connection is established.
 * @note Prefer the non-blocking Phase 2 state machine (PHASE_MQTT_TRY) for
 *       new code paths.
 */
void connectMQTT() {
  while (!client.connected()) {
#if HAS_LED
    ledMqttConnecting();  // Slow blink while connecting
#endif

    if (mqttMgr.maintain()) {
      // Connected — onMqttConnect callback already handled subscriptions
#if HAS_LED
      ledMqttConnected();  // Double blink to indicate success
#endif
      // Process any retained config/set messages
      Serial.println("Processing retained messages...");
      for (int i = 0; i < 10; i++) {
        client.loop();
        delay(10);
      }
      
      // Report current config (includes any updates from config/set)
      publishDeviceConfig();
    } else {
      Serial.printf("MQTT connect attempt failed (broker: %s), retrying in 5s...\n",
                     mqttMgr.activeHost());
#if HAS_LED
      ledError();  // Error blink for failed connection
#endif
      delay(5000);
    }
  }
}

/**
 * @brief Initializes the device ID from MAC address
 * 
 * Generates a unique device identifier using the last 4 bytes of the
 * ESP32/ESP8266 MAC address in the format "ESP32-AABBCCDD" or "ESP8266-AABBCCDD".
 */
void initDeviceId() {
  uint8_t mac[6];
  WiFi.macAddress(mac);
#ifdef ESP8266
  snprintf(deviceId, sizeof(deviceId), "ESP8266-%02X%02X%02X%02X",
           mac[2], mac[3], mac[4], mac[5]);
#else
  // ESP32 or ESP32-S3
  snprintf(deviceId, sizeof(deviceId), "ESP32-%02X%02X%02X%02X",
           mac[2], mac[3], mac[4], mac[5]);
#endif
  Serial.print("Device ID: ");
  Serial.println(deviceId);
}

#if HAS_BLE
/**
 * @brief Initializes RuuviTag scanner and registers known tags
 * 
 * If RUUVI_TAG_COUNT is defined in secrets.h with known MAC addresses,
 * those tags will be registered. Otherwise, tags will be auto-discovered
 * during BLE scans.
 */
void initRuuviTags() {
  initRuuviScanner();
  
  // Load whitelist from NVS first (takes precedence over secrets.h)
  loadRuuviWhitelist();
  
  // If NVS had entries, whitelist mode is already set.
  // Only fall back to secrets.h if NVS was empty.
  if (getRegisteredTagCount() == 0) {
#if RUUVI_TAG_COUNT > 0
    Serial.println("Registering RuuviTags from secrets.h...");
    for (int i = 0; i < RUUVI_TAG_COUNT; i++) {
      if (registerRuuviTag(RUUVI_TAG_MACS[i], RUUVI_TAG_LOCATIONS[i])) {
        Serial.print("  Registered: ");
        Serial.print(RUUVI_TAG_MACS[i]);
        Serial.print(" -> ");
        Serial.println(RUUVI_TAG_LOCATIONS[i]);
      }
    }
    // Pre-configured tags act as whitelist
    setWhitelistMode(RuuviWhitelistMode::WHITELIST);
    Serial.println("Whitelist mode enabled (from secrets.h)");
#else
    Serial.println("No pre-registered RuuviTags (auto-discovery enabled)");
    setWhitelistMode(RuuviWhitelistMode::AUTO_DISCOVER);
#endif
  }
  
  Serial.print("RuuviTag scanner initialized. Registered tags: ");
  Serial.print(getRegisteredTagCount());
  Serial.print(", mode: ");
  Serial.println(getWhitelistMode() == RuuviWhitelistMode::WHITELIST ? "WHITELIST" : "AUTO_DISCOVER");
}

/**
 * @brief Save Ruuvi tag whitelist to NVS
 * 
 * Persists all registered tags and the whitelist mode to ESP32 Preferences.
 * Tags are stored as indexed key-value pairs: "rtag0_mac", "rtag0_loc", etc.
 */
void saveRuuviWhitelist() {
#ifndef ESP8266
  preferences.begin("ruuvi", false);
  
  uint8_t count = getRegisteredTagCount();
  preferences.putUChar("tag_count", count);
  preferences.putUChar("wl_mode", static_cast<uint8_t>(getWhitelistMode()));
  
  for (uint8_t i = 0; i < count; i++) {
    const RuuviTag* tag = getRegisteredTag(i);
    if (tag) {
      char macKey[12], locKey[12];
      snprintf(macKey, sizeof(macKey), "tag%d_mac", i);
      snprintf(locKey, sizeof(locKey), "tag%d_loc", i);
      preferences.putString(macKey, tag->macString);
      preferences.putString(locKey, tag->location);
    }
  }
  
  // Clear any leftover entries from a previously larger list
  for (uint8_t i = count; i < MAX_RUUVI_TAGS; i++) {
    char macKey[12], locKey[12];
    snprintf(macKey, sizeof(macKey), "tag%d_mac", i);
    snprintf(locKey, sizeof(locKey), "tag%d_loc", i);
    preferences.remove(macKey);
    preferences.remove(locKey);
  }
  
  preferences.end();
  Serial.print("Ruuvi whitelist saved to NVS (");
  Serial.print(count);
  Serial.println(" tags)");
#endif
}

/**
 * @brief Load Ruuvi tag whitelist from NVS
 * 
 * Restores registered tags and whitelist mode from ESP32 Preferences.
 * Called during initRuuviTags() before secrets.h fallback.
 */
void loadRuuviWhitelist() {
#ifndef ESP8266
  preferences.begin("ruuvi", true);  // read-only
  
  if (!preferences.isKey("tag_count")) {
    preferences.end();
    Serial.println("No Ruuvi whitelist in NVS");
    return;
  }
  
  uint8_t count = preferences.getUChar("tag_count", 0);
  uint8_t mode = preferences.getUChar("wl_mode", 0);
  
  Serial.print("Loading Ruuvi whitelist from NVS: ");
  Serial.print(count);
  Serial.println(" tags");
  
  for (uint8_t i = 0; i < count && i < MAX_RUUVI_TAGS; i++) {
    char macKey[12], locKey[12];
    snprintf(macKey, sizeof(macKey), "tag%d_mac", i);
    snprintf(locKey, sizeof(locKey), "tag%d_loc", i);
    
    String mac = preferences.getString(macKey, "");
    String loc = preferences.getString(locKey, "");
    
    if (mac.length() > 0) {
      if (registerRuuviTag(mac.c_str(), loc.c_str())) {
        Serial.print("  Loaded: ");
        Serial.print(mac);
        Serial.print(" -> ");
        Serial.println(loc);
      }
    }
  }
  
  setWhitelistMode(static_cast<RuuviWhitelistMode>(mode));
  preferences.end();
#endif
}

/**
 * @brief Creates MQTT payloads from RuuviTag data
 * 
 * Iterates through all registered tags with fresh data and creates
 * temperature, humidity, and pressure payloads using the architecture-compliant
 * topic structure: paku/devices/{device_id}/telemetry/{type}/{location}
 * 
 * @param timestamp Current timestamp string
 */
void createRuuviPayloads(const char* timestamp) {
  const RuuviTag* freshTags[MAX_RUUVI_TAGS];
  uint8_t freshCount = getFreshTags(freshTags, MAX_RUUVI_TAGS, millis());
  
  for (uint8_t i = 0; i < freshCount; i++) {
    const RuuviTag* tag = freshTags[i];
    if (!tag->hasData || !tag->lastData.valid) continue;
    
    // Build consolidated payload with all metrics for this device
    JsonDocument doc;
    doc["timestamp"] = String(timestamp);
    
    // Construct device_id as ruuvi_<location>
    String deviceId = String("ruuvi_") + tag->location;
    doc["device_id"] = deviceId;
    doc["location"] = tag->location;
    
    // Create nested metrics object
    JsonObject metrics = doc["metrics"].to<JsonObject>();
    metrics["temperature_c"] = tag->lastData.temperature;
    metrics["humidity_percent"] = tag->lastData.humidity;
    
    if (tag->lastData.pressure > 0) {
      metrics["pressure_hpa"] = tag->lastData.pressure / 100.0f;
    }
    
    if (tag->lastData.batteryVoltage > 0) {
      metrics["battery_mv"] = tag->lastData.batteryVoltage;
    }
    
    // Serialize and buffer as snapshot
    String payload;
    serializeJson(doc, payload);
    String topic = String("paku/sensors/") + deviceId + "/data";
    
    // Add to snapshot buffer
    if (sensorSnapshotCount < MAX_SENSOR_SNAPSHOTS) {
      sensorSnapshots[sensorSnapshotCount].topic = topic;
      sensorSnapshots[sensorSnapshotCount].payload = payload;
      sensorSnapshots[sensorSnapshotCount].transmitted = false;
      sensorSnapshotCount++;
    }
    
    Serial.print("Buffered RuuviTag [");
    Serial.print(tag->location);
    Serial.print("]: T=");
    Serial.print(tag->lastData.temperature);
    Serial.print("°C, H=");
    Serial.print(tag->lastData.humidity);
    Serial.print("% (buffer: ");
    Serial.print(sensorSnapshotCount);
    Serial.println(")");
  }
}

/**
 * @brief Initializes MoKo sensor scanner and registers known sensors
 * 
 * If MOKO_SENSOR_COUNT is defined in secrets.h with known MAC addresses,
 * those sensors will be registered. Otherwise, sensors will be auto-discovered
 * during BLE scans.
 */
void initMoKoSensors() {
  initMoKoScanner();
  
#if defined(MOKO_SENSOR_COUNT) && MOKO_SENSOR_COUNT > 0
  Serial.println("Registering known MoKo sensors...");
  for (int i = 0; i < MOKO_SENSOR_COUNT; i++) {
    if (registerMoKoSensor(MOKO_SENSOR_MACS[i], MOKO_SENSOR_LOCATIONS[i])) {
      Serial.print("  Registered: ");
      Serial.print(MOKO_SENSOR_MACS[i]);
      Serial.print(" -> ");
      Serial.println(MOKO_SENSOR_LOCATIONS[i]);
    }
  }
#else
  Serial.println("No pre-registered MoKo sensors (auto-discovery enabled)");
#endif
  
  Serial.print("MoKo sensor scanner initialized. Registered sensors: ");
  Serial.println(getRegisteredSensorCount());
}

/**
 * @brief Creates MQTT payloads from MoKo sensor data
 * 
 * Iterates through all registered sensors with fresh data and creates
 * temperature, humidity, pressure, and accelerometer payloads using the 
 * architecture-compliant topic structure: paku/sensors/{device_id}/data
 * 
 * @param timestamp Current timestamp string
 */
void createMoKoPayloads(const char* timestamp) {
  const MoKoSensor* freshSensors[MAX_MOKO_SENSORS];
  uint8_t freshCount = getFreshSensors(freshSensors, MAX_MOKO_SENSORS, millis());
  
  for (uint8_t i = 0; i < freshCount; i++) {
    const MoKoSensor* sensor = freshSensors[i];
    if (!sensor->hasData || !sensor->lastData.valid) continue;
    
    // Build consolidated payload with all metrics for this device
    JsonDocument doc;
    doc["timestamp"] = String(timestamp);
    
    // Construct device_id as moko_<location>
    String deviceId = String("moko_") + sensor->location;
    doc["device_id"] = deviceId;
    doc["location"] = sensor->location;
    
    // Nest all metrics under "metrics" object
    JsonObject metrics = doc["metrics"].to<JsonObject>();
    metrics["temperature_c"] = sensor->lastData.temperature;
    metrics["humidity_percent"] = sensor->lastData.humidity;
    
    if (sensor->lastData.pressure > 0) {
      metrics["pressure_hpa"] = sensor->lastData.pressure / 100.0f;
    }
    
    if (sensor->lastData.batteryVoltage > 0) {
      metrics["battery_mv"] = sensor->lastData.batteryVoltage * 1000.0f;
    }
    
    if (sensor->lastData.batteryPercent > 0) {
      metrics["battery_percent"] = sensor->lastData.batteryPercent;
    }
    
    // Add accelerometer data (if non-zero)
    if (sensor->lastData.accelerationX != 0 || sensor->lastData.accelerationY != 0 || sensor->lastData.accelerationZ != 0) {
      metrics["accel_x"] = sensor->lastData.accelerationX;
      metrics["accel_y"] = sensor->lastData.accelerationY;
      metrics["accel_z"] = sensor->lastData.accelerationZ;
    }
    
    // Serialize and buffer as snapshot
    String payload;
    serializeJson(doc, payload);
    String topic = String("paku/sensors/") + deviceId + "/data";
    
    // Add to snapshot buffer
    if (sensorSnapshotCount < MAX_SENSOR_SNAPSHOTS) {
      sensorSnapshots[sensorSnapshotCount].topic = topic;
      sensorSnapshots[sensorSnapshotCount].payload = payload;
      sensorSnapshots[sensorSnapshotCount].transmitted = false;
      sensorSnapshotCount++;
    }
    
    // Determine model name
    const char* modelName = "Unknown";
    if (sensor->model == 1) modelName = "H2";
    else if (sensor->model == 2) modelName = "H3";
    else if (sensor->model == 3) modelName = "H4";
    
    Serial.print("Buffered MoKo [");
    Serial.print(sensor->location);
    Serial.print("] ");
    Serial.print(modelName);
    Serial.print(": T=");
    Serial.print(sensor->lastData.temperature);
    Serial.print("°C, H=");
    Serial.print(sensor->lastData.humidity);
    Serial.print("% (buffer: ");
    Serial.print(sensorSnapshotCount);
    Serial.println(")");
  }
}
#endif // HAS_BLE

#if HAS_WIRED_SENSORS
// Device ID suffix for wired sensors
#define WIRED_SENSOR_SUFFIX "_wired"

/**
 * @brief Creates MQTT payloads from DS18B20 sensor data
 * 
 * Reads temperature from DS18B20 digital sensor and creates payloads 
 * using architecture-compliant topic structure.
 * 
 * @param timestamp Current timestamp string
 */
void createWiredSensorPayloads(const char* timestamp) {
  if (!wiredSensorsEnabled) return;
  
  unsigned long currentTime = millis();
  if (currentTime - lastWiredSensorRead < WIRED_SENSOR_INTERVAL_MS) {
    return;  // Not time to read yet
  }
  
  lastWiredSensorRead = currentTime;
  
  WiredSensorData data = wiredSensors.readSensors();
  
  if (!data.valid) {
    Serial.println("Warning: DS18B20 sensor reading invalid");
    return;
  }
  
  // Create consolidated payload with all metrics
  JsonDocument doc;
  doc["timestamp"] = String(timestamp);
  doc["device_id"] = String(deviceId) + WIRED_SENSOR_SUFFIX;
  doc["location"] = "wired_sensor";
  doc["sensor_type"] = wiredSensors.getSensorType();
  
  JsonObject metrics = doc["metrics"].to<JsonObject>();
  
  // Add temperature from DS18B20
  if (wiredSensors.isAvailable()) {
    metrics["temperature_c"] = data.temperature;
  }
  
  String payload;
  serializeJson(doc, payload);
  
  String topic = String("paku/sensors/") + deviceId + WIRED_SENSOR_SUFFIX + "/data";
  
  // Add to sensor snapshot buffer (used for all consolidated sensor transmissions)
  if (sensorSnapshotCount < MAX_SENSOR_SNAPSHOTS) {
    sensorSnapshots[sensorSnapshotCount].topic = topic;
    sensorSnapshots[sensorSnapshotCount].payload = payload;
    sensorSnapshots[sensorSnapshotCount].transmitted = false;
    Serial.print("Buffered DS18B20 [wired_sensor]: T=");
    Serial.print(data.temperature, 2);
    Serial.print("°C (buffer: ");
    Serial.print(sensorSnapshotCount + 1);
    Serial.println(")");
    sensorSnapshotCount++;
  } else {
    Serial.println("Warning: Snapshot buffer full, dropping DS18B20 reading");
  }
}
#endif // HAS_WIRED_SENSORS

#if HAS_BLE
/**
 * @brief Creates placeholder payloads for sensors not yet implemented
 * 
 * Generates placeholder data for future sensor integration using 
 * architecture-compliant topic structure.
 * This ensures the data pipeline is tested even without hardware.
 * 
 * @param timestamp Current timestamp string
 */
void createPlaceholderPayloads(const char* timestamp) {
  if (!generatePlaceholderData) return;
  
  // Generate placeholder data for locations without Ruuvi tags
  const char* placeholderLocations[] = {"cabin", "dryer", "kitchen", "lounge"};
  
  for (int i = 0; i < 4; i++) {
    // Check if we have a Ruuvi tag for this location
    bool hasRuuviData = false;
    
    const RuuviTag* freshTags[MAX_RUUVI_TAGS];
    uint8_t freshCount = getFreshTags(freshTags, MAX_RUUVI_TAGS, millis());
    for (uint8_t j = 0; j < freshCount; j++) {
      if (strcmp(freshTags[j]->location, placeholderLocations[i]) == 0) {
        hasRuuviData = true;
        break;
      }
    }
    
    // Only create placeholders if no real Ruuvi data exists for this location
    if (!hasRuuviData) {
      // Generate placeholder values (SENSOR_NOT_AVAILABLE indicates no hardware)
      String humidTopic = String("paku/devices/") + deviceId + "/telemetry/humidity/" + placeholderLocations[i];
      createPayload(humidTopic, SENSOR_NOT_AVAILABLE, timestamp);
      
      String tempTopic = String("paku/devices/") + deviceId + "/telemetry/temperature/" + placeholderLocations[i];
      createPayload(tempTopic, SENSOR_NOT_AVAILABLE, timestamp);
    }
  }
  
  // Additional placeholder sensors for heating system
  createPayload(String("paku/devices/") + deviceId + "/telemetry/temperature/floor", SENSOR_NOT_AVAILABLE, timestamp);
  createPayload(String("paku/devices/") + deviceId + "/telemetry/temperature/heater_in", SENSOR_NOT_AVAILABLE, timestamp);
  createPayload(String("paku/devices/") + deviceId + "/telemetry/temperature/heater_out", SENSOR_NOT_AVAILABLE, timestamp);
  
  // Power readings placeholders
  createPayload(String("paku/devices/") + deviceId + "/telemetry/power/heat", SENSOR_NOT_AVAILABLE, timestamp);
  createPayload(String("paku/devices/") + deviceId + "/telemetry/power/cool", SENSOR_NOT_AVAILABLE, timestamp);
  
  // Battery voltage placeholders
  createPayload(String("paku/devices/") + deviceId + "/telemetry/voltage/car", SENSOR_NOT_AVAILABLE, timestamp);
  createPayload(String("paku/devices/") + deviceId + "/telemetry/voltage/leisure", SENSOR_NOT_AVAILABLE, timestamp);
  
  // Status placeholders
  createPayload(String("paku/devices/") + deviceId + "/telemetry/status/heater", SENSOR_NOT_AVAILABLE, timestamp);
  createPayload(String("paku/devices/") + deviceId + "/telemetry/status/heater_timer", SENSOR_NOT_AVAILABLE, timestamp);
  createPayload(String("paku/devices/") + deviceId + "/telemetry/status/pump", SENSOR_NOT_AVAILABLE, timestamp);
}

// Function to scan for Bluetooth devices and parse RuuviTag data
void scanBT(void* parameter) {
  // Initialize BLE once
  BLEDevice::init("");
  
  while (scanBT_enabled) {
    Serial.println("Starting Bluetooth scan...");

    // Create a BLE scan object
    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setActiveScan(false); // Passive scan — lower power, sufficient for RuuviTag broadcasts
    pBLEScan->setInterval(200);     // 200 × 0.625 ms = 125 ms cycle
    pBLEScan->setWindow(100);       // 100 × 0.625 ms = 62.5 ms listen window (~50% duty)

    // Start scanning for 5 seconds
    BLEScanResults foundDevices = pBLEScan->start(5, false);

    // Print the number of devices found
    Serial.print("Devices found: ");
    Serial.println(foundDevices.getCount());

    // Iterate through the found devices and check for RuuviTags and MoKo sensors
    for (int i = 0; i < foundDevices.getCount(); i++) {
      BLEAdvertisedDevice device = foundDevices.getDevice(i);
      
      // Get device name if available
      std::string deviceName = device.haveName() ? device.getName() : "";
      
      // Check if this is a Frezzer device by name (Alpicool/Frezzer BLE)
      if (!deviceName.empty() && isFrezzerDevice(deviceName.c_str())) {
        int rssi = device.getRSSI();
        uint8_t macBytes[6];
        esp_bd_addr_t* nativeAddr = device.getAddress().getNative();
        for (int j = 0; j < 6; j++) macBytes[j] = (*nativeAddr)[j];
        if (updateFrezzerFromScan(macBytes, deviceName.c_str(), rssi, millis())) {
          Serial.print("Frezzer device updated: ");
          Serial.println(device.getAddress().toString().c_str());
        }
      }
      
      // Check if device has manufacturer data
      if (device.haveManufacturerData()) {
        std::string mfData = device.getManufacturerData();
        
        // Check minimum length for data
        if (mfData.length() >= 2) {
          // Get manufacturer ID (little-endian)
          uint16_t manufacturerId = (uint8_t)mfData[0] | ((uint8_t)mfData[1] << 8);
          
          // Get MAC address bytes
          uint8_t macBytes[6];
          esp_bd_addr_t* nativeAddr = device.getAddress().getNative();
          for (int j = 0; j < 6; j++) {
            macBytes[j] = (*nativeAddr)[j];
          }
          
          // Check if this is Ruuvi data
          if (isRuuviManufacturer(manufacturerId)) {
            Serial.print("RuuviTag found: ");
            Serial.println(device.getAddress().toString().c_str());
            
            // Extract Ruuvi payload (skip 2-byte manufacturer ID)
            if (mfData.length() > 2) {
              const uint8_t* ruuviPayload = (const uint8_t*)mfData.c_str() + 2;
              size_t ruuviLength = mfData.length() - 2;
              
              // Update tag data
              if (updateRuuviTagData(macBytes, ruuviPayload, ruuviLength, millis())) {
                Serial.println("  -> RuuviTag data updated");
              } else {
                Serial.println("  -> Failed to parse RuuviTag data");
              }
            }
          }
          // MOKO sensor support disabled - advertisement format requires additional documentation
          // TODO: Re-enable when complete protocol specification is available
          /*
          else if (isMoKoManufacturer(manufacturerId) || isMoKoDeviceName(deviceName.c_str())) {
            // MoKo sensor processing disabled
          }
          */
        }
      }
    }

    // Clear the scan results
    pBLEScan->clearResults();

    // Delay before the next scan using configured interval
    vTaskDelay(BLE_SCAN_INTERVAL_MS / portTICK_PERIOD_MS);
  }

  // Delete the task if scanBT_enabled is set to false
  vTaskDelete(NULL);
}
#endif // HAS_BLE

/**
 * @brief Creates a payload with the given topic, value, and timestamp, and stores it in the payloads array.
 * 
 * This function constructs a JSON string containing the value and timestamp, and assigns it to the data field
 * of the payload at the current payloadIndex. It also assigns the provided topic to the topic field of the payload.
 * The payloadIndex is then incremented to point to the next available slot in the payloads array.
 * 
 * @param topic The topic associated with the payload.
 * @param value The value to be included in the payload.
 * @param timestamp The timestamp to be included in the payload.
 * 
 * @note The function ensures that the payloadIndex does not exceed MAX_MQTT_PAYLOADS.
 */
void createPayload(String topic, float value, String timestamp) {
  if (payloadIndex < MAX_MQTT_PAYLOADS) { // Ensure we don't exceed array size
    String data = "{\"value\": " + String(value) + ", \"timestamp\": \"" + timestamp + "\"}";
    payloads[payloadIndex].topic = topic;
    payloads[payloadIndex].data = data;
    payloadIndex++;
  }
}

/**
 * @brief Updates the intervals for MQTT and sensor readings based on the heater status.
 *
 * This function sets the intervals for MQTT and sensor readings to either fast or slow
 * intervals depending on whether the heater is on or off. If the heater is on (heaterStatus == 1),
 * the intervals are set to fast intervals. Otherwise, they are set to slow intervals.
 */
void updateIntervals() {
  if (heaterStatus == 1) {
    // Heater is on - use fast intervals for the entire duration
    mqttInterval = mqttFastInterval;
    sensorInterval = sensorFastInterval;
  } else {
    // Heater is off - use slow intervals
    mqttInterval = mqttSlowInterval;
    sensorInterval = sensorSlowInterval;
  }
}

/**
 * @brief Publish device status to MQTT
 * 
 * Publishes operational status to paku/edge/{deviceId}/status topic
 * following the documented MQTT schema.
 */
void publishDeviceStatus() {
  if (!client.connected()) {
    Serial.println("publishDeviceStatus: Client not connected!");
    return;
  }
  
  Serial.println("publishDeviceStatus: Starting...");
  
  String statusTopic = String("paku/edge/") + deviceId + "/status";
  JsonDocument doc;
  
  // Get MAC address
  uint8_t mac[6];
  WiFi.macAddress(mac);
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  
  doc["online"] = true;
  doc["device_id"] = deviceId;
  doc["mac_address"] = macStr;
  
  // Add friendly name if configured
  const char* friendlyName = getEdgeDeviceFriendlyName();
  if (friendlyName != nullptr) {
    doc["friendly_name"] = friendlyName;
  }
  
  doc["last_seen"] = getISO8601Timestamp();
  doc["signal_strength_dbm"] = WiFi.RSSI();
  doc["uptime_seconds"] = millis() / 1000;
  doc["firmware_version"] = FIRMWARE_VERSION;
  #ifdef DEVICE_MODEL
  doc["device_model"] = DEVICE_MODEL;
  #endif
  doc["state"] = (currentState == STATE_CONTINUOUS ? "continuous" : "unknown");
  doc["heater_status"] = heaterStatus;
  
  // MQTT broker failover status
  doc["mqtt_broker"] = mqttMgr.activeBrokerName();
  doc["mqtt_host"] = mqttMgr.activeHost();
  
  // Add active scenario info
  if (deviceConfig.timing.wake_interval_s == 10) {
    doc["active_scenario"] = "heater_active";
  } else if (deviceConfig.timing.wake_interval_s == 300) {
    doc["active_scenario"] = "power_save";
  } else {
    doc["active_scenario"] = "default";
  }
  
  String output;
  serializeJson(doc, output);
  
  // Publish with QoS 1 and retain flag
  client.publish(statusTopic.c_str(), output.c_str(), true);
  Serial.print("Published device status to: ");
  Serial.println(statusTopic);
}

/**
 * @brief Publish device configuration to MQTT
 * 
 * Publishes current configuration to paku/edge/{deviceId}/config/report topic
 * following the documented MQTT schema.
 */
void publishDeviceConfig() {
  if (!client.connected()) {
    Serial.println("publishDeviceConfig: Client not connected!");
    return;
  }
  
  Serial.println("publishDeviceConfig: Starting...");
  printConfig("Publishing to MQTT");
  
  String configTopic = String("paku/edge/") + deviceId + "/config/report";
  JsonDocument doc;

  doc["version"] = FIRMWARE_VERSION;
  
  // Timing configuration
  doc["timing"]["wake_interval_s"] = deviceConfig.timing.wake_interval_s;
  doc["timing"]["connection_duration_max_s"] = deviceConfig.timing.connection_duration_max_s;
  doc["timing"]["wifi_connect_timeout_s"] = deviceConfig.timing.wifi_connect_timeout_s;
  doc["timing"]["mqtt_connect_timeout_s"] = deviceConfig.timing.mqtt_connect_timeout_s;
  doc["timing"]["timezone"] = deviceConfig.timing.timezone;
  
  // Sensor configuration
  doc["sensors"]["ble"]["enabled"] = deviceConfig.sensors.ble.enabled;
  doc["sensors"]["ble"]["scan_duration_s"] = deviceConfig.sensors.ble.scan_duration_s;
  doc["sensors"]["ble"]["scan_active"] = deviceConfig.sensors.ble.scan_active;
  
  doc["sensors"]["wired"]["enabled"] = deviceConfig.sensors.wired.enabled;
  doc["sensors"]["wired"]["sample_count"] = deviceConfig.sensors.wired.sample_count;
  doc["sensors"]["wired"]["sample_interval_ms"] = deviceConfig.sensors.wired.sample_interval_ms;
  
  doc["sensors"]["flow"]["enabled"] = deviceConfig.sensors.flow.enabled;
  doc["sensors"]["flow"]["measurement_duration_s"] = deviceConfig.sensors.flow.measurement_duration_s;
  
  // Power configuration
  doc["power"]["deep_sleep_enabled"] = deviceConfig.power.deep_sleep_enabled;
  doc["power"]["light_sleep_during_wait"] = deviceConfig.power.light_sleep_during_wait;
  doc["power"]["battery_monitor_enabled"] = deviceConfig.power.battery_monitor_enabled;

  // Peripheral manifest
  doc["peripherals"]["ble_ruuvi"]     = deviceConfig.peripherals.ble_ruuvi;
  doc["peripherals"]["ble_moko"]      = deviceConfig.peripherals.ble_moko;
  doc["peripherals"]["ble_frezzer"]   = deviceConfig.peripherals.ble_frezzer;
  doc["peripherals"]["wired_ds18b20"] = deviceConfig.peripherals.wired_ds18b20;
  doc["peripherals"]["heater"]        = deviceConfig.peripherals.heater;
  doc["peripherals"]["fan_ir"]        = deviceConfig.peripherals.fan_ir;
  doc["peripherals"]["milight"]       = deviceConfig.peripherals.milight;

  // HA integration
  doc["ha"]["enabled"] = deviceConfig.ha.enabled;

#if HAS_BLE
  // Ruuvi tag whitelist
  doc["ruuvi"]["mode"] = (getWhitelistMode() == RuuviWhitelistMode::WHITELIST)
                          ? "whitelist" : "auto_discover";
  JsonArray ruuviTags = doc["ruuvi"]["tags"].to<JsonArray>();
  uint8_t ruuviCount = getRegisteredTagCount();
  for (uint8_t i = 0; i < ruuviCount; i++) {
    const RuuviTag* tag = getRegisteredTag(i);
    if (tag) {
      JsonObject t = ruuviTags.add<JsonObject>();
      t["mac"] = tag->macString;
      t["location"] = tag->location;
      t["registered"] = tag->registered;
    }
  }
  doc["ruuvi"]["count"] = ruuviCount;
#endif
  
  String output;
  serializeJson(doc, output);
  
  // Publish with QoS 1 and retain flag
  Serial.print("publishDeviceConfig: Serialized JSON length: ");
  Serial.println(output.length());
  
  // Publish with QoS 1 and retain flag
  bool published = client.publish(configTopic.c_str(), output.c_str(), true);
  Serial.print("Published config (");
  Serial.print(output.length());
  Serial.print(" bytes), result: ");
  Serial.println(published ? "SUCCESS" : "FAILED");
  Serial.print("Published device config to: ");
  Serial.println(configTopic);
}

// ============================================================================
// Phase 2: Config Persistence, Sensor Buffer, and Network Management
// ============================================================================

/**
 * @brief Print current device configuration for debugging
 * 
 * @param context Description of when/why config is being printed
 */
void printConfig(const char* context) {
  Serial.println("========================================");
  Serial.print("CONFIG [");
  Serial.print(context);
  Serial.println("]");
  Serial.println("========================================");
  
  Serial.println("Timing:");
  Serial.print("  wake_interval_s: ");
  Serial.println(deviceConfig.timing.wake_interval_s);
  Serial.print("  connection_duration_max_s: ");
  Serial.println(deviceConfig.timing.connection_duration_max_s);
  Serial.print("  wifi_connect_timeout_s: ");
  Serial.println(deviceConfig.timing.wifi_connect_timeout_s);
  Serial.print("  mqtt_connect_timeout_s: ");
  Serial.println(deviceConfig.timing.mqtt_connect_timeout_s);
  Serial.print("  timezone: ");
  Serial.println(deviceConfig.timing.timezone);
  
  Serial.println("Sensors:");
  Serial.print("  ble.enabled: ");
  Serial.println(deviceConfig.sensors.ble.enabled ? "true" : "false");
  Serial.print("  ble.scan_duration_s: ");
  Serial.println(deviceConfig.sensors.ble.scan_duration_s);
  Serial.print("  ble.scan_active: ");
  Serial.println(deviceConfig.sensors.ble.scan_active ? "true" : "false");
  Serial.print("  wired.enabled: ");
  Serial.println(deviceConfig.sensors.wired.enabled ? "true" : "false");
  Serial.print("  wired.sample_count: ");
  Serial.println(deviceConfig.sensors.wired.sample_count);
  Serial.print("  wired.sample_interval_ms: ");
  Serial.println(deviceConfig.sensors.wired.sample_interval_ms);
  Serial.print("  flow.enabled: ");
  Serial.println(deviceConfig.sensors.flow.enabled ? "true" : "false");
  Serial.print("  flow.measurement_duration_s: ");
  Serial.println(deviceConfig.sensors.flow.measurement_duration_s);
  
  Serial.println("Power:");
  Serial.print("  deep_sleep_enabled: ");
  Serial.println(deviceConfig.power.deep_sleep_enabled ? "true" : "false");
  Serial.print("  light_sleep_during_wait: ");
  Serial.println(deviceConfig.power.light_sleep_during_wait ? "true" : "false");
  Serial.print("  battery_monitor_enabled: ");
  Serial.println(deviceConfig.power.battery_monitor_enabled ? "true" : "false");

  Serial.println("Peripherals:");
  Serial.printf("  ble_ruuvi: %s  ble_moko: %s  ble_frezzer: %s\n",
                deviceConfig.peripherals.ble_ruuvi   ? "on" : "off",
                deviceConfig.peripherals.ble_moko    ? "on" : "off",
                deviceConfig.peripherals.ble_frezzer ? "on" : "off");
  Serial.printf("  wired_ds18b20: %s  heater: %s  fan_ir: %s  milight: %s\n",
                deviceConfig.peripherals.wired_ds18b20 ? "on" : "off",
                deviceConfig.peripherals.heater        ? "on" : "off",
                deviceConfig.peripherals.fan_ir        ? "on" : "off",
                deviceConfig.peripherals.milight       ? "on" : "off");

  Serial.println("HA:");
  Serial.printf("  enabled: %s\n", deviceConfig.ha.enabled ? "true" : "false");

  Serial.println("========================================");
}

/**
 * @brief Save device configuration to persistent storage
 * 
 * Saves the current deviceConfig to NVS (ESP32) or EEPROM (ESP8266).
 */
void saveConfig() {
#ifdef ESP8266
  // ESP8266: Use EEPROM
  EEPROM.begin(512);
  EEPROM.put(0, deviceConfig);
  EEPROM.commit();
  EEPROM.end();
  Serial.println("Config saved to EEPROM");
  printConfig("Saved to EEPROM");
#else
  // ESP32: Use Preferences
  preferences.begin("paku", false);
  preferences.putUInt("wake_int", deviceConfig.timing.wake_interval_s);
  preferences.putUInt("conn_dur", deviceConfig.timing.connection_duration_max_s);
  preferences.putUInt("wifi_to", deviceConfig.timing.wifi_connect_timeout_s);
  preferences.putUInt("mqtt_to", deviceConfig.timing.mqtt_connect_timeout_s);
  preferences.putString("timezone", deviceConfig.timing.timezone);
  
  preferences.putBool("ble_en", deviceConfig.sensors.ble.enabled);
  preferences.putUInt("ble_dur", deviceConfig.sensors.ble.scan_duration_s);
  preferences.putBool("ble_act", deviceConfig.sensors.ble.scan_active);
  
  preferences.putBool("wired_en", deviceConfig.sensors.wired.enabled);
  preferences.putUInt("wired_cnt", deviceConfig.sensors.wired.sample_count);
  preferences.putUInt("wired_int", deviceConfig.sensors.wired.sample_interval_ms);
  
  preferences.putBool("flow_en", deviceConfig.sensors.flow.enabled);
  preferences.putUInt("flow_dur", deviceConfig.sensors.flow.measurement_duration_s);
  
  preferences.putBool("deep_sl", deviceConfig.power.deep_sleep_enabled);
  preferences.putBool("light_sl", deviceConfig.power.light_sleep_during_wait);
  preferences.putBool("batt_mon", deviceConfig.power.battery_monitor_enabled);

  preferences.putBool("p_ruuvi", deviceConfig.peripherals.ble_ruuvi);
  preferences.putBool("p_moko",  deviceConfig.peripherals.ble_moko);
  preferences.putBool("p_frez",  deviceConfig.peripherals.ble_frezzer);
  preferences.putBool("p_ds18",  deviceConfig.peripherals.wired_ds18b20);
  preferences.putBool("p_heat",  deviceConfig.peripherals.heater);
  preferences.putBool("p_fan",   deviceConfig.peripherals.fan_ir);
  preferences.putBool("p_mil",   deviceConfig.peripherals.milight);

  preferences.putBool("ha_en",   deviceConfig.ha.enabled);

  preferences.end();
  Serial.println("Config saved to Preferences");
  printConfig("Saved to NVS");
#endif
}

/**
 * @brief Load device configuration from persistent storage
 * 
 * Loads deviceConfig from NVS (ESP32) or EEPROM (ESP8266).
 * If no valid config found, uses defaults.
 */
void loadConfig() {
#ifdef ESP8266
  // ESP8266: Use EEPROM
  EEPROM.begin(512);
  DeviceConfig loadedConfig;
  EEPROM.get(0, loadedConfig);
  EEPROM.end();
  
  // Basic validation - check if wake_interval is reasonable
  if (loadedConfig.timing.wake_interval_s > 0 && loadedConfig.timing.wake_interval_s < 86400) {
    deviceConfig = loadedConfig;
    Serial.println("Config loaded from EEPROM");
    printConfig("Loaded from EEPROM");
  } else {
    Serial.println("No valid config in EEPROM, loading defaults");
    deviceConfig.loadDefaults();
    printConfig("Defaults loaded (first boot)");
    Serial.println("Saving default config to EEPROM...");
    saveConfig();  // Save defaults so next boot doesn't re-initialize
  }
#else
  // ESP32: Use Preferences
  preferences.begin("paku", true); // read-only
  
  if (preferences.isKey("wake_int")) {
    // Load existing config from NVS
    deviceConfig.timing.wake_interval_s = preferences.getUInt("wake_int", 60);
    deviceConfig.timing.connection_duration_max_s = preferences.getUInt("conn_dur", 30);
    deviceConfig.timing.wifi_connect_timeout_s = preferences.getUInt("wifi_to", 10);
    deviceConfig.timing.mqtt_connect_timeout_s = preferences.getUInt("mqtt_to", 5);
    String tz = preferences.getString("timezone", "EET-2EEST,M3.5.0/3,M10.5.0/4");
    strncpy(deviceConfig.timing.timezone, tz.c_str(), sizeof(deviceConfig.timing.timezone) - 1);
    deviceConfig.timing.timezone[sizeof(deviceConfig.timing.timezone) - 1] = '\0';
    
    deviceConfig.sensors.ble.enabled = preferences.getBool("ble_en", true);
    deviceConfig.sensors.ble.scan_duration_s = preferences.getUInt("ble_dur", 10);
    deviceConfig.sensors.ble.scan_active = preferences.getBool("ble_act", true);
    
    deviceConfig.sensors.wired.enabled = preferences.getBool("wired_en", true);
    deviceConfig.sensors.wired.sample_count = preferences.getUInt("wired_cnt", 3);
    deviceConfig.sensors.wired.sample_interval_ms = preferences.getUInt("wired_int", 100);
    
    deviceConfig.sensors.flow.enabled = preferences.getBool("flow_en", false);
    deviceConfig.sensors.flow.measurement_duration_s = preferences.getUInt("flow_dur", 5);
    
    deviceConfig.power.deep_sleep_enabled = preferences.getBool("deep_sl", false);
    deviceConfig.power.light_sleep_during_wait = preferences.getBool("light_sl", true);
    deviceConfig.power.battery_monitor_enabled = preferences.getBool("batt_mon", false);

    deviceConfig.peripherals.ble_ruuvi    = preferences.getBool("p_ruuvi", true);
    deviceConfig.peripherals.ble_moko     = preferences.getBool("p_moko",  true);
    deviceConfig.peripherals.ble_frezzer  = preferences.getBool("p_frez",  true);
    deviceConfig.peripherals.wired_ds18b20 = preferences.getBool("p_ds18", true);
    deviceConfig.peripherals.heater       = preferences.getBool("p_heat",  true);
    deviceConfig.peripherals.fan_ir       = preferences.getBool("p_fan",   true);
    deviceConfig.peripherals.milight      = preferences.getBool("p_mil",   true);

    deviceConfig.ha.enabled = preferences.getBool("ha_en", true);

    Serial.println("Config loaded from Preferences");
    printConfig("Loaded from NVS");
  } else {
    // No saved config - this is first boot or NVS was erased
    Serial.println("No saved config found, loading defaults");
    deviceConfig.loadDefaults();
    printConfig("Defaults loaded (first boot)");
    Serial.println("Saving default config to Preferences...");
    saveConfig();  // Save defaults so next boot doesn't re-initialize
  }
  
  preferences.end();
#endif
}

/**
 * @brief Add a sensor reading to the buffer
 * 
 * @param sensor_id Sensor identifier
 * @param metric Metric name (temperature, humidity, etc.)
 * @param value Sensor value
 * @param timestamp ISO8601 timestamp
 */
void addSensorReading(const char* sensor_id, const char* metric, float value, const char* timestamp) {
  if (bufferCount >= MAX_BUFFERED_READINGS) {
    Serial.println("Warning: Sensor buffer full, dropping oldest reading");
    // Shift buffer to make room
    for (int i = 0; i < MAX_BUFFERED_READINGS - 1; i++) {
      sensorBuffer[i] = sensorBuffer[i + 1];
    }
    bufferCount = MAX_BUFFERED_READINGS - 1;
  }
  
  SensorReading* reading = &sensorBuffer[bufferCount];
  strncpy(reading->timestamp, timestamp, sizeof(reading->timestamp) - 1);
  strncpy(reading->sensor_id, sensor_id, sizeof(reading->sensor_id) - 1);
  strncpy(reading->metric, metric, sizeof(reading->metric) - 1);
  reading->value = value;
  reading->transmitted = false;
  
  bufferCount++;
  
  Serial.print("Buffered reading: ");
  Serial.print(sensor_id);
  Serial.print(" ");
  Serial.print(metric);
  Serial.print(" = ");
  Serial.print(value);
  Serial.print(" (buffer: ");
  Serial.print(bufferCount);
  Serial.println(")");
}

/**
 * @brief Clear transmitted readings from buffer
 * 
 * Removes readings that have been successfully transmitted,
 * compacting the buffer.
 */
void clearTransmittedReadings() {
  int writeIndex = 0;
  for (int readIndex = 0; readIndex < bufferCount; readIndex++) {
    if (!sensorBuffer[readIndex].transmitted) {
      if (writeIndex != readIndex) {
        sensorBuffer[writeIndex] = sensorBuffer[readIndex];
      }
      writeIndex++;
    }
  }
  int cleared = bufferCount - writeIndex;
  bufferCount = writeIndex;
  
  if (cleared > 0) {
    Serial.print("Cleared ");
    Serial.print(cleared);
    Serial.println(" transmitted readings from buffer");
  }
}

/**
 * @brief Collect sensor data without network connection
 * 
 * Reads all enabled sensors and stores readings in buffer.
 * WiFi should be OFF during this operation.
 */
void collectSensorData() {
  Serial.println("=== Collecting Sensor Data ===");
  String timestampStr = getISO8601Timestamp();
  const char* timestamp = timestampStr.c_str();
  
#if HAS_BLE
  // BLE Sensor Collection
  if (deviceConfig.sensors.ble.enabled) {
    Serial.println("Scanning BLE sensors...");
    if (deviceConfig.peripherals.ble_ruuvi)   createRuuviPayloads(timestamp);
    if (deviceConfig.peripherals.ble_moko)    createMoKoPayloads(timestamp);
  }
#endif

#if HAS_WIRED_SENSORS
  // Wired Sensor Collection
  if (deviceConfig.sensors.wired.enabled) {
    Serial.println("Reading wired sensors...");
    createWiredSensorPayloads(timestamp);
  }
#endif

  // Flow Sensor Collection - DISABLED (future development)
  // TODO: Re-enable when flow sensor hardware is installed and calibrated
  // if (deviceConfig.sensors.flow.enabled && FLOW_SENSOR_ENABLED) {
  //   Serial.println("Measuring flow...");
  //   float flow = processFlowData(millis() - lastSensorCollection);
  //   addSensorReading(deviceId, "flow_rate", flow, timestamp);
  //   addSensorReading(deviceId, "required_dt", getRequiredDeltaT(), timestamp);
  // }
  
  lastSensorCollection = millis();
  Serial.print("Sensor collection complete. Buffer size: ");
  Serial.println(bufferCount);
}

/**
 * @brief Transmit buffered sensor data
 * 
 * Sends all untransmitted readings from buffer to MQTT.
 * Marks readings as transmitted on success.
 */
void transmitBufferedData() {
  if (!client.connected()) {
    Serial.println("Cannot transmit: MQTT not connected");
    return;
  }
  
  Serial.print("Transmitting ");
  Serial.print(bufferCount);
  Serial.println(" buffered readings...");
  
  int transmitted = 0;
  for (int i = 0; i < bufferCount; i++) {
    SensorReading* reading = &sensorBuffer[i];
    
    if (reading->transmitted) {
      continue; // Skip already transmitted
    }
    
    // Create MQTT topic according to schema
    String topic = String("paku/sensors/") + reading->sensor_id + "/data";
    
    // Create JSON payload
    JsonDocument doc;
    doc["timestamp"] = reading->timestamp;
    doc["sensor_id"] = reading->sensor_id;
    doc[reading->metric] = reading->value;
    
    String payload;
    serializeJson(doc, payload);
    
    // Publish
    if (client.publish(topic.c_str(), payload.c_str())) {
      reading->transmitted = true;
      transmitted++;
      Serial.print("  TX: ");
      Serial.println(topic);
    } else {
      Serial.print("  FAILED: ");
      Serial.println(topic);
    }
  }
  
  Serial.print("Transmitted ");
  Serial.print(transmitted);
  Serial.print(" / ");
  Serial.println(bufferCount);
  
  // Clear transmitted readings
  clearTransmittedReadings();
}

/**
 * @brief Transmit buffered sensor snapshots
 * 
 * Sends all untransmitted sensor snapshots (consolidated payloads from BLE, wired, etc.) to MQTT.
 */
void transmitSensorSnapshots() {
  if (!client.connected()) {
    return; // Silently skip if not connected
  }
  
  if (sensorSnapshotCount == 0) {
    return; // Nothing to transmit
  }
  
  Serial.print("Transmitting ");
  Serial.print(sensorSnapshotCount);
  Serial.println(" sensor snapshots...");
  
  int transmitted = 0;
  for (int i = 0; i < sensorSnapshotCount; i++) {
    SensorSnapshot* snapshot = &sensorSnapshots[i];
    
    if (snapshot->transmitted) {
      continue; // Skip already transmitted
    }
    
    // Publish
    if (client.publish(snapshot->topic.c_str(), snapshot->payload.c_str())) {
      snapshot->transmitted = true;
      transmitted++;
      Serial.print("  TX: ");
      Serial.println(snapshot->topic);
    } else {
      Serial.print("  FAILED: ");
      Serial.println(snapshot->topic);
    }
  }
  
  Serial.print("Transmitted ");
  Serial.print(transmitted);
  Serial.print(" / ");
  Serial.println(sensorSnapshotCount);
  
  // Clear transmitted snapshots
  int writeIndex = 0;
  for (int readIndex = 0; readIndex < sensorSnapshotCount; readIndex++) {
    if (!sensorSnapshots[readIndex].transmitted) {
      if (writeIndex != readIndex) {
        sensorSnapshots[writeIndex] = sensorSnapshots[readIndex];
      }
      writeIndex++;
    }
  }
  int cleared = sensorSnapshotCount - writeIndex;
  sensorSnapshotCount = writeIndex;
  
  if (cleared > 0) {
    Serial.print("Cleared ");
    Serial.print(cleared);
    Serial.println(" transmitted sensor snapshots from buffer");
  }
}

/**
 * @brief Connect to WiFi and MQTT
 * 
 * Turns on WiFi, connects to AP, and establishes MQTT connection.
 */
void connectNetwork() {
  Serial.println("=== Connecting to Network ===");
  
  // Connect WiFi
  connect_wifi();
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi connection failed");
    return;
  }
  
  // Connect MQTT
  connectMQTT();
  
  if (client.connected()) {
    Serial.println("Network connection established");
    lastNetworkConnect = millis();
  } else {
    Serial.println("MQTT connection failed");
  }
}

/**
 * @brief Disconnect from network and turn off WiFi
 * 
 * Cleanly disconnects MQTT and turns off WiFi radio for power savings.
 */
void disconnectNetwork() {
  Serial.println("=== Disconnecting Network ===");
  
  // Disconnect MQTT
  if (client.connected()) {
    client.disconnect();
    Serial.println("MQTT disconnected");
  }
  
  // Disconnect WiFi
  WiFi.disconnect(true); // true = turn off WiFi radio
  WiFi.mode(WIFI_OFF);
  
  Serial.println("WiFi turned OFF");
}

// ============================================================================
// End Phase 2 Functions
// ============================================================================

/**
 * @brief State machine handler for Phase 2
 * 
 * Manages transitions between states based on timing and conditions.
 * Replaces the continuous loop pattern with clear state-based flow.
 */
void handleSystemState() {
  unsigned long now = millis();
  unsigned long stateElapsed = now - stateEnteredAt;
  
  switch (currentSystemState) {
    
    case SYS_STATE_IDLE: {
      // Keep processing MQTT messages while connected (local broker stay-alive)
      if (client.connected()) {
        client.loop();
      }

      // Wait for sensor collection interval
      if (now - lastSensorCollection >= (deviceConfig.timing.wake_interval_s * 1000)) {
        currentSystemState = SYS_STATE_COLLECT_SENSORS;
        stateEnteredAt = now;
        Serial.println("STATE: IDLE -> COLLECT_SENSORS");
      }
      break;
    }
      
    case SYS_STATE_COLLECT_SENSORS: {
      // ================================================================
      // Non-blocking sensor collection with async DS18B20 support.
      //
      // Sub-phases:
      //   COLLECT_BLE       → read cached BLE data (instant)
      //   COLLECT_WIRED_REQ → request async DS18B20 conversion
      //   COLLECT_WIRED_WAIT→ poll isConversionReady() each iteration
      //   COLLECT_DONE      → finalize and decide next state
      // ================================================================

      // Keep MQTT alive during sensor collection (local broker stay-alive)
      if (client.connected()) {
        client.loop();
      }

      enum CollectPhase : uint8_t {
        COLLECT_BLE,
        COLLECT_WIRED_REQ,
        COLLECT_WIRED_WAIT,
        COLLECT_DONE
      };

      static CollectPhase collectPhase = COLLECT_BLE;
      static unsigned long collectPhaseStart = 0;
      static String collectTimestamp;

      switch (collectPhase) {

        case COLLECT_BLE: {
          Serial.println("=== Collecting Sensor Data (non-blocking) ===");
          collectTimestamp = getISO8601Timestamp();
#if HAS_BLE
          if (deviceConfig.sensors.ble.enabled) {
            Serial.println("Reading cached BLE sensors...");
            createRuuviPayloads(collectTimestamp.c_str());
            createMoKoPayloads(collectTimestamp.c_str());
          }
#endif
          collectPhase = COLLECT_WIRED_REQ;
          collectPhaseStart = now;
          break;
        }

        case COLLECT_WIRED_REQ: {
#if HAS_WIRED_SENSORS
          if (deviceConfig.sensors.wired.enabled && wiredSensorsEnabled) {
            unsigned long currentTime = millis();
            if (currentTime - lastWiredSensorRead >= WIRED_SENSOR_INTERVAL_MS) {
              if (wiredSensors.requestConversion()) {
                Serial.println("DS18B20 async conversion requested");
                collectPhase = COLLECT_WIRED_WAIT;
                collectPhaseStart = now;
                break;
              }
            }
          }
#endif
          // No wired sensor to wait for — skip straight to done
          collectPhase = COLLECT_DONE;
          break;
        }

        case COLLECT_WIRED_WAIT: {
#if HAS_WIRED_SENSORS
          if (wiredSensors.isConversionReady()) {
            lastWiredSensorRead = millis();
            WiredSensorData data = wiredSensors.readConversion();
            if (data.valid) {
              // Build the payload inline (same as createWiredSensorPayloads)
              JsonDocument doc;
              doc["timestamp"] = collectTimestamp;
              doc["device_id"] = String(deviceId) + WIRED_SENSOR_SUFFIX;
              doc["location"] = "wired_sensor";
              doc["sensor_type"] = wiredSensors.getSensorType();
              JsonObject metrics = doc["metrics"].to<JsonObject>();
              metrics["temperature_c"] = data.temperature;

              String payload;
              serializeJson(doc, payload);
              String topic = String("paku/sensors/") + deviceId + WIRED_SENSOR_SUFFIX + "/data";

              if (sensorSnapshotCount < MAX_SENSOR_SNAPSHOTS) {
                sensorSnapshots[sensorSnapshotCount].topic = topic;
                sensorSnapshots[sensorSnapshotCount].payload = payload;
                sensorSnapshots[sensorSnapshotCount].transmitted = false;
                Serial.print("Buffered DS18B20 [wired_sensor]: T=");
                Serial.print(data.temperature, 2);
                Serial.print("°C (buffer: ");
                Serial.print(sensorSnapshotCount + 1);
                Serial.println(")");
                sensorSnapshotCount++;
              }
            } else {
              Serial.println("Warning: DS18B20 async reading invalid");
            }
            collectPhase = COLLECT_DONE;
          } else if (stateElapsed > 2000) {
            // Safety timeout — DS18B20 12-bit takes ~750ms max
            Serial.println("Warning: DS18B20 conversion timeout, skipping");
            collectPhase = COLLECT_DONE;
          }
          // else: stay in COLLECT_WIRED_WAIT, return to loop
#endif
          break;
        }

        case COLLECT_DONE: {
          lastSensorCollection = millis();
          Serial.print("Sensor collection complete. Buffer size: ");
          Serial.println(bufferCount);
          collectPhase = COLLECT_BLE;  // reset for next cycle

          // Decide if we should transmit now
          bool shouldTransmit = (bufferCount >= (MAX_BUFFERED_READINGS * 0.8)) ||
                                (now - lastNetworkConnect >= (deviceConfig.timing.wake_interval_s * 1000));

          if (shouldTransmit) {
            // If already connected (local stay-alive), skip network setup
            if (client.connected()) {
              currentSystemState = SYS_STATE_TRANSMIT;
              stateEnteredAt = now;
              Serial.println("STATE: COLLECT_SENSORS -> TRANSMIT (already connected)");
            } else {
              currentSystemState = SYS_STATE_CONNECT_NETWORK;
              stateEnteredAt = now;
              Serial.println("STATE: COLLECT_SENSORS -> CONNECT_NETWORK");
            }
          } else {
            currentSystemState = SYS_STATE_IDLE;
            stateEnteredAt = now;
            Serial.println("STATE: COLLECT_SENSORS -> IDLE (no transmission needed)");
          }
          break;
        }
      }
      break;
    }
      
    case SYS_STATE_CONNECT_NETWORK: {
      // ================================================================
      // Fully non-blocking WiFi + MQTT connection with async scan.
      //
      // Sub-phases tracked by static enum:
      //   PHASE_SCAN_START  → kick off async WiFi.scanNetworks(true)
      //   PHASE_SCAN_WAIT   → poll WiFi.scanComplete() each iteration
      //   PHASE_WIFI_WAIT   → WiFi.begin() was called, poll status()
      //   PHASE_MQTT_TRY    → single-attempt MQTT connect
      //   PHASE_MQTT_SETTLE → run client.loop() for a few iterations to
      //                       receive retained messages (no delay())
      // ================================================================

      enum ConnectPhase : uint8_t {
        PHASE_SCAN_START,
        PHASE_SCAN_WAIT,
        PHASE_WIFI_WAIT,
        PHASE_MQTT_TRY,
        PHASE_MQTT_SETTLE
      };

      static ConnectPhase connectPhase = PHASE_SCAN_START;
      static unsigned long phaseStartMs = 0;
      static int           settleLoops  = 0;

      switch (connectPhase) {

        case PHASE_SCAN_START: {
          WiFi.disconnect();
          WiFi.mode(WIFI_STA);
          // Start ASYNC scan (first param true = async)
          WiFi.scanNetworks(true);
          connectPhase = PHASE_SCAN_WAIT;
          phaseStartMs = now;
          Serial.println("Async WiFi scan started...");
#if HAS_RGB_LCD
          gui_show_busy("Scanning WiFi\xE2\x80\xA6");
#endif
          break;
        }

        case PHASE_SCAN_WAIT: {
          int found = WiFi.scanComplete();
          if (found == WIFI_SCAN_RUNNING) {
            // Still scanning — check for timeout (5 s)
            if (now - phaseStartMs > 5000) {
              Serial.println("WiFi scan timeout");
              WiFi.scanDelete();
              connectPhase = PHASE_SCAN_START;
              currentSystemState = SYS_STATE_DISCONNECT;
              stateEnteredAt = now;
            }
            break;  // yield to loop()
          }
          if (found <= 0) {
            Serial.println("No networks found — skipping connect");
            WiFi.scanDelete();
            connectPhase = PHASE_SCAN_START;
            currentSystemState = SYS_STATE_DISCONNECT;
            stateEnteredAt = now;
            break;
          }

          // Pick first configured SSID that is in range
          extern WiFiManager wifiManager;
          bool begun = false;
          int nvsCount = wifiManager.getStoredNetworkCount();

          for (int n = 0; n < nvsCount + WIFI_COUNT && !begun; n++) {
            String ssid, pass;
            if (n < nvsCount) {
              WiFiCredential cred = wifiManager.getNetwork(n);
              if (!cred.valid) continue;
              ssid = String(cred.ssid);
              pass = String(cred.password);
            } else {
              int fi = n - nvsCount;
              ssid = String(WIFI_SSIDS[fi]);
              pass = String(WIFI_PASSWORDS[fi]);
            }
            for (int j = 0; j < found; j++) {
              if (WiFi.SSID(j) == ssid) {
                Serial.printf("Trying %s...\n", ssid.c_str());
                WiFi.begin(ssid.c_str(), pass.c_str());
                begun = true;
                break;
              }
            }
          }
          WiFi.scanDelete();

          if (!begun) {
            Serial.println("No configured network in range");
            connectPhase = PHASE_SCAN_START;
            currentSystemState = SYS_STATE_DISCONNECT;
            stateEnteredAt = now;
            break;
          }

          connectPhase = PHASE_WIFI_WAIT;
          phaseStartMs = now;
#if HAS_RGB_LCD
          gui_show_busy("Connecting WiFi\xE2\x80\xA6");
#endif
          break;
        }

        case PHASE_WIFI_WAIT: {
          if (WiFi.status() == WL_CONNECTED) {
            Serial.printf("WiFi connected: %s\n", WiFi.localIP().toString().c_str());
            wifi_status = "WiFi connected (" + WiFi.localIP().toString() + ")";
#if HAS_LED
            ledWifiConnected();
#endif
            connectPhase = PHASE_MQTT_TRY;
            phaseStartMs = now;
          } else if (now - phaseStartMs >= (deviceConfig.timing.wifi_connect_timeout_s * 1000UL)) {
            Serial.println("WiFi connect timeout");
            connectPhase = PHASE_SCAN_START;
            currentSystemState = SYS_STATE_DISCONNECT;
            stateEnteredAt = now;
          }
#if HAS_LED
          else {
            ledWifiConnecting();
          }
#endif
          break;
        }

        case PHASE_MQTT_TRY: {
          // Single-attempt MQTT connect via failover manager
          // (probes local broker, falls back to cloud automatically)
#if HAS_RGB_LCD
          gui_show_busy("Connecting MQTT\xE2\x80\xA6");
          gui_update();  // Render before potentially blocking connect
#endif
          if (mqttMgr.tryConnectOnce()) {
            Serial.printf("MQTT connected to %s broker\n", mqttMgr.activeBrokerName());
#if HAS_LED
            ledMqttConnected();
#endif
            // onMqttConnect callback already handled topic subscriptions
            // Transition to settling phase — run client.loop() a few times
            // across successive loop() iterations to receive retained messages.
            connectPhase = PHASE_MQTT_SETTLE;
            settleLoops  = 0;
            phaseStartMs = now;
          } else if (mqttMgr.didAttempt()) {
            // A real connection attempt was made and failed
            Serial.printf("MQTT attempt failed on %s broker (rc=%d)\n",
                          mqttMgr.activeBrokerName(), client.state());
#if HAS_LED
            ledMqttConnecting();
#endif
          }
          // else: throttled — no-op, wait for next real attempt

          // Overall MQTT timeout
          if (!client.connected() &&
              stateElapsed >= (deviceConfig.timing.mqtt_connect_timeout_s * 1000UL)) {
            Serial.println("MQTT connect timeout — will retry next cycle");
            connectPhase = PHASE_SCAN_START;
            currentSystemState = SYS_STATE_DISCONNECT;
            stateEnteredAt = now;
          }
          break;
        }

        case PHASE_MQTT_SETTLE: {
          // Run client.loop() once per main-loop iteration (no delay!)
          // to drain retained messages from the broker.
          client.loop();
          settleLoops++;

          if (settleLoops >= 10) {
            // Done settling — publish status & config, move to TRANSMIT
            publishDeviceStatus();
            publishDeviceConfig();
            connectPhase = PHASE_SCAN_START;  // reset for next time
            currentSystemState = SYS_STATE_TRANSMIT;
            stateEnteredAt = now;
            Serial.println("STATE: CONNECT_NETWORK -> TRANSMIT");
#if HAS_RGB_LCD
            gui_hide_busy();
#endif
          }
          break;
        }
      }
      break;
    }
      
    case SYS_STATE_TRANSMIT: {
      // Process MQTT messages
      client.loop();
      
      // Transmit buffered data
      static bool transmissionStarted = false;
      if (!transmissionStarted) {
        // Transmit all buffered data
        transmitSensorSnapshots();
        transmitBufferedData();
        publishDeviceStatus();
        transmissionStarted = true;
      }
      
      // Check if we should disconnect
      bool transmissionComplete = (bufferCount == 0);
      bool connectionTimeout = (stateElapsed >= (deviceConfig.timing.connection_duration_max_s * 1000));
      
      if (transmissionComplete && connectionTimeout) {
        transmissionStarted = false;
        
        // When on local broker (same network as HA), stay connected
        // for real-time command responsiveness. Only disconnect on cloud.
        if (mqttMgr.isOnFallback()) {
          currentSystemState = SYS_STATE_DISCONNECT;
          stateEnteredAt = now;
          Serial.println("STATE: TRANSMIT -> DISCONNECT (cloud broker)");
        } else {
          currentSystemState = SYS_STATE_IDLE;
          stateEnteredAt = now;
          Serial.println("STATE: TRANSMIT -> IDLE (staying connected to local broker)");
        }
      }
      
      // Handle pending OTA updates
      if (otaUpdatePending) {
        processOtaUpdate();
      }
      break;
    }
      
    case SYS_STATE_DISCONNECT: {
#if HAS_RGB_LCD
      gui_hide_busy();
#endif
      // Disconnect and turn off WiFi
      disconnectNetwork();
      
      currentSystemState = SYS_STATE_IDLE;
      stateEnteredAt = now;
      Serial.println("STATE: DISCONNECT -> IDLE");
      break;
    }
  }
}

// ============================================================================
// End Phase 2 State Machine
// ============================================================================

// =============================================================================
// Frezzer PRO BLE Fridge Integration
// =============================================================================

#if HAS_BLE
/**
 * @brief Initializes Frezzer controller and registers known devices
 *
 * If FREZZER_COUNT is defined in secrets.h with known MAC addresses,
 * those devices will be registered. Otherwise, devices will be auto-discovered
 * during BLE scans.
 */
void initFrezzerDevices() {
  initFrezzerController();

#if FREZZER_COUNT > 0
  Serial.println("Registering known Frezzer devices...");
  for (int i = 0; i < FREZZER_COUNT; i++) {
    if (registerFrezzerDevice(FREZZER_MACS[i], FREZZER_LOCATIONS[i])) {
      Serial.print("  Registered: ");
      Serial.print(FREZZER_MACS[i]);
      Serial.print(" -> ");
      Serial.println(FREZZER_LOCATIONS[i]);
    }
  }
#else
  Serial.println("No pre-registered Frezzer devices (auto-discovery enabled)");
#endif

  Serial.print("Frezzer controller initialized. Registered devices: ");
  Serial.println(getFrezzerDeviceCount());
}

/**
 * @brief Creates MQTT payloads from Frezzer device data
 *
 * @param timestamp Current timestamp string
 */
void createFrezzerPayloads(const char* timestamp) {
  const FrezzerDevice* freshDevices[MAX_FREZZER_DEVICES];
  uint8_t freshCount = getFreshFrezzerDevices(freshDevices, MAX_FREZZER_DEVICES, millis());

  for (uint8_t i = 0; i < freshCount; i++) {
    const FrezzerDevice* device = freshDevices[i];
    if (!device->hasData || !device->lastData.valid) continue;

    JsonDocument doc;
    doc["timestamp"] = String(timestamp);
    doc["device_id"] = String("frezzer_") + device->location;
    doc["location"] = device->location;
    doc["mac"] = device->macString;
    if (strlen(device->deviceName) > 0) doc["device_name"] = device->deviceName;

    JsonObject metrics = doc["metrics"].to<JsonObject>();
    metrics["current_temp_c"] = device->lastData.currentTemp;
    metrics["target_temp_c"] = device->lastData.targetTemp;
    metrics["battery_voltage"] = device->lastData.batteryVoltage;
    metrics["bat_percent"] = device->lastData.batPercent;
    metrics["bat_saver_mode"] = device->lastData.batSaverMode;
    metrics["mode"] = frezzerModeToString(device->lastData.mode);
    metrics["compressor"] = frezzerCompressorStateToString(device->lastData.compressor);
    metrics["locked"] = device->lastData.locked;
    metrics["error"] = frezzerErrorToString(device->lastData.error);
    metrics["connected"] = device->connected;

    String payload;
    serializeJson(doc, payload);
    String topic = String("paku/fridge/") + device->location + "/data";

    if (payloadIndex < MAX_MQTT_PAYLOADS) {
      payloads[payloadIndex].topic = topic;
      payloads[payloadIndex].data = payload;
      payloadIndex++;
    }

    Serial.print("Frezzer [");
    Serial.print(device->location);
    Serial.print("]: T=");
    Serial.print(device->lastData.currentTemp);
    Serial.print("\u00b0C, Target=");
    Serial.print(device->lastData.targetTemp);
    Serial.print("\u00b0C, Mode=");
    Serial.println(frezzerModeToString(device->lastData.mode));
  }
}

/**
 * @brief Handles MQTT commands for Frezzer devices
 *
 * @param topic MQTT topic (paku/fridge/{location}/cmd)
 * @param payload JSON command payload
 */
void handleFrezzerMqttCommand(const char* topic, const char* payload) {
  String topicStr = String(topic);
  if (!topicStr.startsWith("paku/fridge/") || !topicStr.endsWith("/cmd")) return;

  int startIdx = 12;
  int endIdx = topicStr.lastIndexOf("/cmd");
  if (endIdx <= startIdx) return;
  String location = topicStr.substring(startIdx, endIdx);

  const FrezzerDevice* device = nullptr;
  for (uint8_t i = 0; i < getFrezzerDeviceCount(); i++) {
    const FrezzerDevice* d = getFrezzerDevice(i);
    if (d != nullptr && String(d->location) == location) { device = d; break; }
  }
  if (device == nullptr) {
    Serial.print("Frezzer command: device not found for location ");
    Serial.println(location);
    return;
  }

  JsonDocument cmdDoc;
  DeserializationError error = deserializeJson(cmdDoc, payload);
  if (error) {
    Serial.print("Frezzer command: JSON parse error - ");
    Serial.println(error.c_str());
    return;
  }

  const char* command = cmdDoc["command"];
  if (command == nullptr) { Serial.println("Frezzer command: missing 'command' field"); return; }

  FrezzerDevice* mutableDevice = const_cast<FrezzerDevice*>(device);
  FrezzerResult result = FrezzerResult::UNKNOWN_ERROR;

  if (strcmp(command, "set_temp") == 0) {
    float temp = cmdDoc["value"].as<float>();
    result = setFrezzerTargetTemp(mutableDevice, temp);
    Serial.print("Frezzer set_temp "); Serial.print(temp);
  } else if (strcmp(command, "set_mode") == 0) {
    const char* modeStr = cmdDoc["value"];
    FrezzerMode mode = FrezzerMode::UNKNOWN;
    if (strcmp(modeStr, "off") == 0)           mode = FrezzerMode::OFF;
    else if (strcmp(modeStr, "eco") == 0)      mode = FrezzerMode::ECO;
    else if (strcmp(modeStr, "max_cool") == 0) mode = FrezzerMode::MAX_COOL;
    // "fridge" and "freezer" are temperature targets, not modes — use set_temp
    result = setFrezzerMode(mutableDevice, mode);
    Serial.print("Frezzer set_mode "); Serial.print(modeStr);
  } else if (strcmp(command, "power") == 0) {
    const char* powerStr = cmdDoc["value"];
    if (strcmp(powerStr, "on") == 0)       result = turnFrezzerOn(mutableDevice);
    else if (strcmp(powerStr, "off") == 0) result = turnFrezzerOff(mutableDevice);
    Serial.print("Frezzer power "); Serial.print(powerStr);
  } else {
    Serial.print("Frezzer command: unknown command "); Serial.println(command); return;
  }

  Serial.print(" -> ");
  Serial.println(frezzerResultToString(result));
}
#endif // HAS_BLE

// TFT Pin check (only for devices with display)
#if HAS_DISPLAY
#if PIN_LCD_WR  != TFT_WR || \
    PIN_LCD_RD  != TFT_RD || \
    PIN_LCD_CS    != TFT_CS   || \
    PIN_LCD_DC    != TFT_DC   || \
    PIN_LCD_RES   != TFT_RST  || \
    PIN_LCD_D0   != TFT_D0  || \
    PIN_LCD_D1   != TFT_D1  || \
    PIN_LCD_D2   != TFT_D2  || \
    PIN_LCD_D3   != TFT_D3  || \
    PIN_LCD_D4   != TFT_D4  || \
    PIN_LCD_D5   != TFT_D5  || \
    PIN_LCD_D6   != TFT_D6  || \
    PIN_LCD_D7   != TFT_D7  || \
    PIN_LCD_BL   != TFT_BL  || \
    TFT_BACKLIGHT_ON   != HIGH  || \
    170   != TFT_WIDTH  || \
    320   != TFT_HEIGHT
#error  "Error! Please make sure <User_Setups/Setup206_LilyGo_T_Display_S3.h> is selected in <TFT_eSPI/User_Setup_Select.h>"
#endif
#endif // HAS_DISPLAY

/**
 * @brief Initialize the OTA update client
 * 
 * Initializes the OTA client and validates the current firmware.
 * Call this during setup().
 */
void initOta() {
  OtaResult result = otaClient.begin();
  if (result != OtaResult::SUCCESS) {
    Serial.print("OTA: Failed to initialize: ");
    Serial.println(otaClient.getLastError());
    return;
  }

  // Get current firmware info
  char version[32];
  char partition[32];
  if (otaClient.getCurrentFirmwareInfo(version, sizeof(version), partition, sizeof(partition))) {
    Serial.print("OTA: Running firmware version: ");
    Serial.print(version);
    Serial.print(" on partition: ");
    Serial.println(partition);
  }

  // Mark current firmware as valid (prevents rollback after successful boot)
  // This should be called after verifying the device is working correctly
  result = otaClient.validateCurrentFirmware();
  if (result == OtaResult::SUCCESS) {
    Serial.println("OTA: Current firmware validated");
  }

  Serial.println("OTA: Client initialized successfully");
}

/**
 * @brief Process pending OTA updates
 * 
 * Call this in the main loop to process OTA updates when triggered via MQTT.
 */
void processOtaUpdate() {
  if (!otaUpdatePending) {
    return;
  }

  Serial.println("OTA: Processing pending update...");

#if HAS_DISPLAY
  // Show OTA start screen on display
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(3);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(0, 0);
  tft.println(" OTA Update");
  tft.drawLine(0, 28, 320, 28, TFT_CYAN);

  // Version — truncate to fit on one line (max ~18 chars at textSize 2)
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 40);
  {
    String ver = pendingOtaVersion;
    if (ver.length() > 18) ver = ver.substring(0, 18) + "..";
    tft.printf("Ver: %s", ver.c_str());
  }

  // "Downloading" label (left) and percentage placeholder (right)
  tft.setCursor(10, 65);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.print("Downloading");
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(260, 65);
  tft.print("  0%");

  // Draw empty progress bar
  tft.drawRect(10, 95, 300, 25, TFT_WHITE);
#endif

  // Configure OTA update
  OtaConfig config;
  config.firmwareUrl = pendingOtaUrl.c_str();
  config.expectedChecksum = pendingOtaChecksum.c_str();
  config.targetVersion = pendingOtaVersion.c_str();
  config.verifySignature = false;  // Signature verification not implemented yet
  config.allowDowngrade = false;
  config.timeoutMs = 300000;  // 5 minutes
  config.bufferSize = 4096;
  config.resumeSupported = false;

  // Start the update (blocking operation)
  OtaResult result = otaClient.startUpdate(config, otaProgressCallback);

  // Report result via MQTT
  String resultTopic = String("paku/edge/") + deviceId + "/ota/result";
  JsonDocument resultDoc;
  resultDoc["timestamp"] = getISO8601Timestamp();
  resultDoc["current_version"] = FIRMWARE_VERSION;
  resultDoc["target_version"] = pendingOtaVersion;
  resultDoc["success"] = (result == OtaResult::SUCCESS);
  resultDoc["result_code"] = (int)result;
  resultDoc["message"] = OtaClient::resultToString(result);
  
  String resultPayload;
  serializeJson(resultDoc, resultPayload);
  client.publish(resultTopic.c_str(), resultPayload.c_str());

  // Clear pending flag
  otaUpdatePending = false;
  pendingOtaUrl = "";
  pendingOtaChecksum = "";
  pendingOtaVersion = "";

  if (result == OtaResult::SUCCESS) {
    Serial.println("OTA: Update successful! Rebooting in 3 seconds...");
#if HAS_DISPLAY
    tft.fillRect(10, 65, 300, 25, TFT_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(10, 70);
    tft.println("Update OK! Rebooting...");
#endif
    unsigned long rebootTime = millis();
    while (millis() - rebootTime < 3000) {
      client.loop();
      delay(100);
    }
    ESP.restart();
  } else {
    Serial.print("OTA: Update failed: ");
    Serial.println(otaClient.getLastError());
#if HAS_DISPLAY
    tft.fillRect(10, 65, 300, 95, TFT_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.setCursor(10, 70);
    tft.println("Update FAILED!");
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.setCursor(10, 95);
    tft.println(otaClient.getLastError());
#endif
  }
}

/**
 * @brief MQTT message callback handler
 * 
 * Handles incoming MQTT messages, including OTA update commands and heater control.
 * 
 * Supported topics:
 * - paku/control: Legacy control commands (e.g., {"heater": 1})
 * - paku/edge/{deviceId}/control: New schema-compliant control (e.g., {"scenario": "heater_active"})
 * - paku/edge/{deviceId}/cmd/ota: OTA update commands
 * 
 * @param topic MQTT topic
 * @param payload Message payload
 * @param length Payload length
 */
void handleMqttMessage(char* topic, byte* payload, unsigned int length) {
  Serial.print("MQTT message received on topic: ");
  Serial.println(topic);

  // Convert payload to string
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.print("Message: ");
  Serial.println(message);

  // Check for new edge device control topic (Phase 1: not yet fully implemented)
  String edgeControlTopic = String("paku/edge/") + deviceId + "/control";
  if (String(topic) == edgeControlTopic) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, message);
    
    if (!error && !doc["scenario"].isNull()) {
      const char* scenario = doc["scenario"];
      Serial.print("Switching to scenario: ");
      Serial.println(scenario);
      
      deviceConfig.applyScenario(scenario);
      
      // Update heater status based on scenario for backward compatibility
      if (strcmp(scenario, "heater_active") == 0) {
        heaterStatus = 1;
      } else {
        heaterStatus = 0;
      }
      
      updateIntervals();
      
      // Publish updated status and config
      publishDeviceStatus();
      publishDeviceConfig();
    }
    return;
  }

  // Check if this is a legacy control command
  if (String(topic) == "paku/control") {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, message);
    
    if (!error && !doc["heater"].isNull()) {
      int newHeaterStatus = doc["heater"];
      if (newHeaterStatus == 0 || newHeaterStatus == 1) {
        heaterStatus = newHeaterStatus;
        Serial.print("Heater status updated to: ");
        Serial.println(heaterStatus);
        
        // Apply corresponding scenario
        if (heaterStatus == 1) {
          deviceConfig.applyScenario("heater_active");
        } else {
          deviceConfig.applyScenario("default");
        }
        
        updateIntervals();
        
        // Publish updated status and config
        publishDeviceStatus();
        publishDeviceConfig();
      }
    }
    return;
  }

#if HAS_RGB_LCD
  // Handle sensor telemetry from data-acquisition boards — update GUI display.
  // Topics: paku/sensors/{sensor_id}/data  and  paku/heater/{device_id}/data
  if (strncmp(topic, "paku/sensors/", 13) == 0 && strstr(topic, "/data") != nullptr) {
    JsonDocument sDoc;
    if (!deserializeJson(sDoc, message)) {
      const char* sensorId = sDoc["sensor_id"] | topic;  // fall back to topic as key
      uint8_t slot = getOrAssignGuiSlot(sensorId);
      if (slot < 4) {
        if (!sDoc["temperature"].isNull()) {
          float t = sDoc["temperature"];
          guiSensorSlots[slot].lastTemp = t;
          gui_push_temperature(slot, t);
        }
        if (!sDoc["humidity"].isNull()) {
          float h = sDoc["humidity"];
          guiSensorSlots[slot].lastHum = h;
          if (slot < 3) gui_push_humidity(slot, h);
        }
        // Update header bar for first two slots (Indoor / Outdoor)
        float t = guiSensorSlots[slot].lastTemp;
        float h = guiSensorSlots[slot].lastHum;
        if (!isnan(t) && !isnan(h)) {
          if (slot == 0) gui_set_header_indoor(t, h);
          if (slot == 1) gui_set_header_outdoor(t, h);
        }
      }
    }
    return;
  }

  if (strncmp(topic, "paku/heater/", 12) == 0 && strstr(topic, "/data") != nullptr) {
    JsonDocument hDoc;
    if (!deserializeJson(hDoc, message)) {
      const char* stateStr = hDoc["metrics"]["heater_state"] | "unknown";
      int hState = 0;
      if (strcmp(stateStr, "Starting") == 0 || strcmp(stateStr, "Warming") == 0)  hState = 1;
      else if (strcmp(stateStr, "Running") == 0)                                   hState = 2;
      else if (strcmp(stateStr, "Shutting Down") == 0 || strcmp(stateStr, "Cooling") == 0) hState = 3;
      float coolant = hDoc["metrics"]["coolant_temp_c"] | 0.0f;
      gui_set_heater_data(hState, -1, coolant);
    }
    return;
  }
#endif  // HAS_RGB_LCD

  // Phase 2: Check for configuration command topic (config/set)
  String edgeConfigTopic = String("paku/edge/") + deviceId + "/config/set";
  if (String(topic) == edgeConfigTopic) {
    Serial.println("========================================");
    Serial.println("MQTT CONFIG/SET MESSAGE RECEIVED");
    Serial.print("Message: ");
    Serial.println(message);
    Serial.println("========================================");
    printConfig("Current config before processing MQTT");
    
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, message);
    
    if (error) {
      Serial.print("Config parse error: ");
      Serial.println(error.c_str());
      return;
    }
    
    // Track if any config actually changed to avoid processing our own published config
    bool configChanged = false;
    
    // Update timing configuration if present
    if (!doc["timing"].isNull()) {
      if (!doc["timing"]["wake_interval_s"].isNull()) {
        uint32_t newValue = doc["timing"]["wake_interval_s"];
        if (newValue != deviceConfig.timing.wake_interval_s) {
          Serial.print("Config change: wake_interval_s ");
          Serial.print(deviceConfig.timing.wake_interval_s);
          Serial.print(" -> ");
          Serial.println(newValue);
          deviceConfig.timing.wake_interval_s = newValue;
          configChanged = true;
        }
      }
      if (!doc["timing"]["connection_duration_max_s"].isNull()) {
        uint32_t newValue = doc["timing"]["connection_duration_max_s"];
        if (newValue != deviceConfig.timing.connection_duration_max_s) {
          Serial.print("Config change: connection_duration_max_s ");
          Serial.print(deviceConfig.timing.connection_duration_max_s);
          Serial.print(" -> ");
          Serial.println(newValue);
          deviceConfig.timing.connection_duration_max_s = newValue;
          configChanged = true;
        }
      }
      if (!doc["timing"]["wifi_connect_timeout_s"].isNull()) {
        uint32_t newValue = doc["timing"]["wifi_connect_timeout_s"];
        if (newValue != deviceConfig.timing.wifi_connect_timeout_s) {
          Serial.print("Config change: wifi_connect_timeout_s ");
          Serial.print(deviceConfig.timing.wifi_connect_timeout_s);
          Serial.print(" -> ");
          Serial.println(newValue);
          deviceConfig.timing.wifi_connect_timeout_s = newValue;
          configChanged = true;
        }
      }
      if (!doc["timing"]["mqtt_connect_timeout_s"].isNull()) {
        uint32_t newValue = doc["timing"]["mqtt_connect_timeout_s"];
        if (newValue != deviceConfig.timing.mqtt_connect_timeout_s) {
          Serial.print("Config change: mqtt_connect_timeout_s ");
          Serial.print(deviceConfig.timing.mqtt_connect_timeout_s);
          Serial.print(" -> ");
          Serial.println(newValue);
          deviceConfig.timing.mqtt_connect_timeout_s = newValue;
          configChanged = true;
        }
      }
      if (!doc["timing"]["timezone"].isNull()) {
        const char* newValue = doc["timing"]["timezone"];
        if (strcmp(newValue, deviceConfig.timing.timezone) != 0) {
          strncpy(deviceConfig.timing.timezone, newValue, sizeof(deviceConfig.timing.timezone) - 1);
          deviceConfig.timing.timezone[sizeof(deviceConfig.timing.timezone) - 1] = '\0';
          // Apply new timezone immediately
          setenv("TZ", deviceConfig.timing.timezone, 1);
          tzset();
          configChanged = true;
          Serial.print("Timezone updated to: ");
          Serial.println(deviceConfig.timing.timezone);
        }
      }
    }
    
    // Update sensor configuration if present
    if (!doc["sensors"].isNull()) {
      if (!doc["sensors"]["ble"].isNull()) {
        if (!doc["sensors"]["ble"]["enabled"].isNull()) {
          bool newValue = doc["sensors"]["ble"]["enabled"];
          if (newValue != deviceConfig.sensors.ble.enabled) {
            Serial.print("Config change: sensors.ble.enabled ");
            Serial.print(deviceConfig.sensors.ble.enabled ? "true" : "false");
            Serial.print(" -> ");
            Serial.println(newValue ? "true" : "false");
            deviceConfig.sensors.ble.enabled = newValue;
            configChanged = true;
          }
        }
        if (!doc["sensors"]["ble"]["scan_duration_s"].isNull()) {
          uint32_t newValue = doc["sensors"]["ble"]["scan_duration_s"];
          if (newValue != deviceConfig.sensors.ble.scan_duration_s) {
            Serial.print("Config change: sensors.ble.scan_duration_s ");
            Serial.print(deviceConfig.sensors.ble.scan_duration_s);
            Serial.print(" -> ");
            Serial.println(newValue);
            deviceConfig.sensors.ble.scan_duration_s = newValue;
            configChanged = true;
          }
        }
        if (!doc["sensors"]["ble"]["scan_active"].isNull()) {
          bool newValue = doc["sensors"]["ble"]["scan_active"];
          if (newValue != deviceConfig.sensors.ble.scan_active) {
            Serial.print("Config change: sensors.ble.scan_active ");
            Serial.print(deviceConfig.sensors.ble.scan_active ? "true" : "false");
            Serial.print(" -> ");
            Serial.println(newValue ? "true" : "false");
            deviceConfig.sensors.ble.scan_active = newValue;
            configChanged = true;
          }
        }
      }
      
      if (!doc["sensors"]["wired"].isNull()) {
        if (!doc["sensors"]["wired"]["enabled"].isNull()) {
          bool newValue = doc["sensors"]["wired"]["enabled"];
          if (newValue != deviceConfig.sensors.wired.enabled) {
            Serial.print("Config change: sensors.wired.enabled ");
            Serial.print(deviceConfig.sensors.wired.enabled ? "true" : "false");
            Serial.print(" -> ");
            Serial.println(newValue ? "true" : "false");
            deviceConfig.sensors.wired.enabled = newValue;
            configChanged = true;
          }
        }
        if (!doc["sensors"]["wired"]["sample_count"].isNull()) {
          uint8_t newValue = doc["sensors"]["wired"]["sample_count"];
          if (newValue != deviceConfig.sensors.wired.sample_count) {
            Serial.print("Config change: sensors.wired.sample_count ");
            Serial.print(deviceConfig.sensors.wired.sample_count);
            Serial.print(" -> ");
            Serial.println(newValue);
            deviceConfig.sensors.wired.sample_count = newValue;
            configChanged = true;
          }
        }
        if (!doc["sensors"]["wired"]["sample_interval_ms"].isNull()) {
          uint16_t newValue = doc["sensors"]["wired"]["sample_interval_ms"];
          if (newValue != deviceConfig.sensors.wired.sample_interval_ms) {
            Serial.print("Config change: sensors.wired.sample_interval_ms ");
            Serial.print(deviceConfig.sensors.wired.sample_interval_ms);
            Serial.print(" -> ");
            Serial.println(newValue);
            deviceConfig.sensors.wired.sample_interval_ms = newValue;
            configChanged = true;
          }
        }
      }
      
      if (!doc["sensors"]["flow"].isNull()) {
        if (!doc["sensors"]["flow"]["enabled"].isNull()) {
          bool newValue = doc["sensors"]["flow"]["enabled"];
          if (newValue != deviceConfig.sensors.flow.enabled) {
            Serial.print("Config change: sensors.flow.enabled ");
            Serial.print(deviceConfig.sensors.flow.enabled ? "true" : "false");
            Serial.print(" -> ");
            Serial.println(newValue ? "true" : "false");
            deviceConfig.sensors.flow.enabled = newValue;
            configChanged = true;
          }
        }
        if (!doc["sensors"]["flow"]["measurement_duration_s"].isNull()) {
          uint32_t newValue = doc["sensors"]["flow"]["measurement_duration_s"];
          if (newValue != deviceConfig.sensors.flow.measurement_duration_s) {
            Serial.print("Config change: sensors.flow.measurement_duration_s ");
            Serial.print(deviceConfig.sensors.flow.measurement_duration_s);
            Serial.print(" -> ");
            Serial.println(newValue);
            deviceConfig.sensors.flow.measurement_duration_s = newValue;
            configChanged = true;
          }
        }
      }
    }
    
    // Update power configuration if present
    if (!doc["power"].isNull()) {
      if (!doc["power"]["deep_sleep_enabled"].isNull()) {
        bool newValue = doc["power"]["deep_sleep_enabled"];
        if (newValue != deviceConfig.power.deep_sleep_enabled) {
          Serial.print("Config change: power.deep_sleep_enabled ");
          Serial.print(deviceConfig.power.deep_sleep_enabled ? "true" : "false");
          Serial.print(" -> ");
          Serial.println(newValue ? "true" : "false");
          deviceConfig.power.deep_sleep_enabled = newValue;
          configChanged = true;
        }
      }
      if (!doc["power"]["light_sleep_during_wait"].isNull()) {
        bool newValue = doc["power"]["light_sleep_during_wait"];
        if (newValue != deviceConfig.power.light_sleep_during_wait) {
          Serial.print("Config change: power.light_sleep_during_wait ");
          Serial.print(deviceConfig.power.light_sleep_during_wait ? "true" : "false");
          Serial.print(" -> ");
          Serial.println(newValue ? "true" : "false");
          deviceConfig.power.light_sleep_during_wait = newValue;
          configChanged = true;
        }
      }
      if (!doc["power"]["battery_monitor_enabled"].isNull()) {
        bool newValue = doc["power"]["battery_monitor_enabled"];
        if (newValue != deviceConfig.power.battery_monitor_enabled) {
          Serial.print("Config change: power.battery_monitor_enabled ");
          Serial.print(deviceConfig.power.battery_monitor_enabled ? "true" : "false");
          Serial.print(" -> ");
          Serial.println(newValue ? "true" : "false");
          deviceConfig.power.battery_monitor_enabled = newValue;
          configChanged = true;
        }
      }
    }
    
    // Update peripheral manifest if present
    if (!doc["peripherals"].isNull()) {
      auto parsePeriphBool = [&](const char* key, bool& field) {
        if (!doc["peripherals"][key].isNull()) {
          bool newValue = doc["peripherals"][key];
          if (newValue != field) {
            Serial.printf("Config change: peripherals.%s %s -> %s\n",
                          key, field ? "true" : "false", newValue ? "true" : "false");
            field = newValue;
            configChanged = true;
          }
        }
      };
      parsePeriphBool("ble_ruuvi",    deviceConfig.peripherals.ble_ruuvi);
      parsePeriphBool("ble_moko",     deviceConfig.peripherals.ble_moko);
      parsePeriphBool("ble_frezzer",  deviceConfig.peripherals.ble_frezzer);
      parsePeriphBool("wired_ds18b20",deviceConfig.peripherals.wired_ds18b20);
      parsePeriphBool("heater",       deviceConfig.peripherals.heater);
      parsePeriphBool("fan_ir",       deviceConfig.peripherals.fan_ir);
      parsePeriphBool("milight",      deviceConfig.peripherals.milight);
    }

    // Update HA integration flag if present (takes effect after reboot)
    if (!doc["ha"].isNull() && !doc["ha"]["enabled"].isNull()) {
      bool newValue = doc["ha"]["enabled"];
      if (newValue != deviceConfig.ha.enabled) {
        Serial.printf("Config change: ha.enabled %s -> %s (takes effect after reboot)\n",
                      deviceConfig.ha.enabled ? "true" : "false", newValue ? "true" : "false");
        deviceConfig.ha.enabled = newValue;
        configChanged = true;
      }
    }

    // Only save and republish if config actually changed
    if (configChanged) {
      Serial.println("========================================");
      Serial.println("CONFIG CHANGED - Saving and republishing");
      Serial.println("========================================");
      
      // Save updated configuration to persistent storage
      saveConfig();
      
      // Publish updated config back as confirmation
      publishDeviceConfig();
      publishDeviceStatus();
      
      Serial.println("Configuration updated and saved");
    } else {
      Serial.println("========================================");
      Serial.println("Config unchanged (ignoring duplicate/own message)");
      Serial.println("========================================");
    }
    return;
  }

  // Phase 2: Check for WiFi management commands (paku/devices/{deviceId}/cmd/wifi)
  String wifiCmdTopic = String("paku/devices/") + deviceId + "/cmd/wifi";
  if (String(topic) == wifiCmdTopic) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, message);
    
    if (error) {
      Serial.print("WiFi CMD: Parse error: ");
      Serial.println(error.c_str());
      return;
    }
    
    const char* action = doc["action"];
    String responseTopic = String("paku/devices/") + deviceId + "/wifi/status";
    JsonDocument responseDoc;
    responseDoc["timestamp"] = getISO8601Timestamp();
    
    if (!action) {
      responseDoc["success"] = false;
      responseDoc["error"] = "Missing action";
    } else if (strcmp(action, "add") == 0) {
      const char* ssid = doc["ssid"];
      const char* password = doc["password"];
      
      if (!ssid || strlen(ssid) == 0) {
        responseDoc["success"] = false;
        responseDoc["error"] = "Missing or empty SSID";
      } else {
        bool success = wifiManager.addNetwork(ssid, password ? password : "");
        responseDoc["success"] = success;
        responseDoc["action"] = "add";
        responseDoc["ssid"] = ssid;
        if (success) {
          LOG_INFO("WiFi", "Added network via MQTT: %s", ssid);
        } else {
          responseDoc["error"] = "Failed to add network (storage full)";
        }
      }
    } else if (strcmp(action, "remove") == 0) {
      const char* ssid = doc["ssid"];
      
      if (!ssid || strlen(ssid) == 0) {
        responseDoc["success"] = false;
        responseDoc["error"] = "Missing or empty SSID";
      } else {
        bool success = wifiManager.removeNetwork(ssid);
        responseDoc["success"] = success;
        responseDoc["action"] = "remove";
        responseDoc["ssid"] = ssid;
        if (success) {
          LOG_INFO("WiFi", "Removed network via MQTT: %s", ssid);
        } else {
          responseDoc["error"] = "Network not found";
        }
      }
    } else if (strcmp(action, "list") == 0) {
      String networks = wifiManager.listNetworks();
      responseDoc["success"] = true;
      responseDoc["action"] = "list";
      
      // Parse the listNetworks JSON and merge it
      JsonDocument listDoc;
      deserializeJson(listDoc, networks);
      responseDoc["networks"] = listDoc["networks"];
      responseDoc["count"] = listDoc["count"];
      
      LOG_INFO("WiFi", "Listed stored networks via MQTT");
    } else if (strcmp(action, "clear") == 0) {
      wifiManager.clearAllNetworks();
      responseDoc["success"] = true;
      responseDoc["action"] = "clear";
      LOG_INFO("WiFi", "Cleared all stored networks via MQTT");
    } else {
      responseDoc["success"] = false;
      responseDoc["error"] = "Unknown action";
    }
    
    String responsePayload;
    serializeJson(responseDoc, responsePayload);
    client.publish(responseTopic.c_str(), responsePayload.c_str());
    return;
  }

#if HAS_BLE
  // Phase 2: Check for Ruuvi whitelist commands (paku/devices/{deviceId}/cmd/ruuvi)
  String ruuviCmdTopic = String("paku/devices/") + deviceId + "/cmd/ruuvi";
  if (String(topic) == ruuviCmdTopic) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, message);
    
    if (error) {
      Serial.print("Ruuvi CMD: Parse error: ");
      Serial.println(error.c_str());
      return;
    }
    
    const char* action = doc["action"];
    String responseTopic = String("paku/devices/") + deviceId + "/ruuvi/status";
    JsonDocument responseDoc;
    responseDoc["timestamp"] = getISO8601Timestamp();
    
    if (!action) {
      responseDoc["success"] = false;
      responseDoc["error"] = "Missing action";
    } else if (strcmp(action, "add") == 0) {
      // Add a tag to the whitelist
      // Payload: {"action": "add", "mac": "AA:BB:CC:DD:EE:FF", "location": "cabin"}
      const char* mac = doc["mac"];
      const char* location = doc["location"];
      
      if (!mac || strlen(mac) == 0) {
        responseDoc["success"] = false;
        responseDoc["error"] = "Missing or empty MAC address";
      } else if (!location || strlen(location) == 0) {
        responseDoc["success"] = false;
        responseDoc["error"] = "Missing or empty location";
      } else {
        bool success = registerRuuviTag(mac, location);
        if (success) {
          // Switch to whitelist mode when first tag is explicitly added
          setWhitelistMode(RuuviWhitelistMode::WHITELIST);
          saveRuuviWhitelist();
          responseDoc["success"] = true;
          responseDoc["action"] = "add";
          responseDoc["mac"] = mac;
          responseDoc["location"] = location;
          LOG_INFO("Ruuvi", "Added tag via MQTT: %s -> %s", mac, location);
        } else {
          responseDoc["success"] = false;
          // Determine reason
          if (findRegisteredTagByMac(mac) != nullptr) {
            responseDoc["error"] = "Tag already registered";
          } else if (getRegisteredTagCount() >= MAX_RUUVI_TAGS) {
            responseDoc["error"] = "Whitelist full (max 8 tags)";
          } else {
            responseDoc["error"] = "Invalid MAC address format";
          }
        }
      }
    } else if (strcmp(action, "remove") == 0) {
      // Remove a tag from the whitelist
      // Payload: {"action": "remove", "mac": "AA:BB:CC:DD:EE:FF"}
      const char* mac = doc["mac"];
      
      if (!mac || strlen(mac) == 0) {
        responseDoc["success"] = false;
        responseDoc["error"] = "Missing or empty MAC address";
      } else {
        bool success = removeRuuviTag(mac);
        responseDoc["success"] = success;
        responseDoc["action"] = "remove";
        responseDoc["mac"] = mac;
        if (success) {
          // If whitelist is now empty, switch back to auto-discover
          if (getRegisteredTagCount() == 0) {
            setWhitelistMode(RuuviWhitelistMode::AUTO_DISCOVER);
            Serial.println("Ruuvi whitelist empty — switching to auto-discover mode");
          }
          saveRuuviWhitelist();
          LOG_INFO("Ruuvi", "Removed tag via MQTT: %s", mac);
        } else {
          responseDoc["error"] = "Tag not found";
        }
      }
    } else if (strcmp(action, "list") == 0) {
      // List all registered tags
      responseDoc["success"] = true;
      responseDoc["action"] = "list";
      responseDoc["mode"] = (getWhitelistMode() == RuuviWhitelistMode::WHITELIST) 
                            ? "whitelist" : "auto_discover";
      
      JsonArray tagsArray = responseDoc["tags"].to<JsonArray>();
      uint8_t count = getRegisteredTagCount();
      for (uint8_t i = 0; i < count; i++) {
        const RuuviTag* tag = getRegisteredTag(i);
        if (tag) {
          JsonObject tagObj = tagsArray.add<JsonObject>();
          tagObj["mac"] = tag->macString;
          tagObj["location"] = tag->location;
          tagObj["registered"] = tag->registered;
          tagObj["has_data"] = tag->hasData;
          if (tag->hasData) {
            tagObj["temperature"] = tag->lastData.temperature;
            tagObj["humidity"] = tag->lastData.humidity;
            tagObj["fresh"] = isTagDataFresh(tag, millis());
          }
        }
      }
      responseDoc["count"] = count;
      LOG_INFO("Ruuvi", "Listed %d tags via MQTT", count);
    } else if (strcmp(action, "clear") == 0) {
      // Clear all tags and switch to auto-discover
      clearRegisteredTags();
      setWhitelistMode(RuuviWhitelistMode::AUTO_DISCOVER);
      saveRuuviWhitelist();
      responseDoc["success"] = true;
      responseDoc["action"] = "clear";
      LOG_INFO("Ruuvi", "Cleared all tags via MQTT — auto-discover enabled");
    } else {
      responseDoc["success"] = false;
      responseDoc["error"] = "Unknown action. Use: add, remove, list, clear";
    }
    
    String ruuviResponsePayload;
    serializeJson(responseDoc, ruuviResponsePayload);
    client.publish(responseTopic.c_str(), ruuviResponsePayload.c_str());
    return;
  }
#endif // HAS_BLE

#ifdef HEATER_ENABLED
  // Route heater commands to the HeaterAddon
  String heaterCmdTopic = String("paku/heater/") + deviceId + "/cmd";
  if (String(topic) == heaterCmdTopic) {
    LOG_INFO("MQTT", "Heater command received: %s", message.c_str());
    if (heaterAddonActive) heater_addon_command(message.c_str(), message.length());

#if HAS_RGB_LCD
    // Parse command to update Waveshare GUI immediately
    {
      JsonDocument cmdDoc;
      if (!deserializeJson(cmdDoc, message)) {
        const char* cmd = cmdDoc["cmd"] | "";
        if (strcmp(cmd, "start") == 0) {
          const char* mode = cmdDoc["mode"] | "power";
          float target = cmdDoc["target_temp"] | 21;
          gui_set_heater_data(1, target, 0);  // 1=STARTING
        } else if (strcmp(cmd, "stop") == 0) {
          gui_set_heater_data(3, -1, 0);  // 3=STOPPING
        }
      }
    }
#endif

    return;
  }
#endif // HEATER_ENABLED

#if HAS_FAN_IR
  // Route fan commands to the MaxxFan IR controller
  String fanCmdTopic = String("paku/edge/") + deviceId + "/cmd/fan";
  if (String(topic) == fanCmdTopic) {
    if (!fanIrActive) return;
    LOG_INFO("MQTT", "Fan command received: %s", message.c_str());

    // Parse JSON command
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, message);

    if (error) {
      Serial.print("Fan CMD: Parse error: ");
      Serial.println(error.c_str());
      return;
    }
    
    // Get current state and update with command fields
    MaxxFanState& state = maxxfan_ir_get_state();
    
    // Update only provided fields
    if (!doc["power"].isNull()) {
      state.fan_on = doc["power"].as<bool>();
    }
    if (!doc["speed"].isNull()) {
      state.speed = doc["speed"].as<uint8_t>();
      if (state.speed > 100) state.speed = 100;
    }
    if (!doc["direction"].isNull()) {
      const char* dir = doc["direction"];
      if (strcmp(dir, "exhaust") == 0) {
        state.exhaust = true;
      } else if (strcmp(dir, "intake") == 0) {
        state.exhaust = false;
      }
    }
    if (!doc["lid"].isNull()) {
      const char* lid = doc["lid"];
      if (strcmp(lid, "open") == 0) {
        state.lid_open = true;
      } else if (strcmp(lid, "closed") == 0) {
        state.lid_open = false;
      }
    }
    if (!doc["mode"].isNull()) {
      const char* mode = doc["mode"];
      if (strcmp(mode, "auto") == 0) {
        state.auto_mode = true;
      } else if (strcmp(mode, "manual") == 0) {
        state.auto_mode = false;
      }
    }
    if (!doc["auto_temp_f"].isNull()) {
      state.auto_temp_f = doc["auto_temp_f"].as<uint8_t>();
    }
    
    // Send IR command
    maxxfan_ir_send(state);

#if HAS_RGB_LCD
    // Sync Waveshare GUI to match the new fan state
    // Pass speed=0 when fan is off so GUI _fanPower stays false
    gui_set_fan_data(state.fan_on ? state.speed : 0, !state.exhaust, state.lid_open);
#endif
    
    // Publish updated state
    String statusTopic = String("paku/edge/") + deviceId + "/status/fan";
    JsonDocument statusDoc;
    statusDoc["power"] = state.fan_on;
    statusDoc["speed"] = state.speed;
    statusDoc["direction"] = state.exhaust ? "exhaust" : "intake";
    statusDoc["lid"] = state.lid_open ? "open" : "closed";
    statusDoc["mode"] = state.auto_mode ? "auto" : "manual";
    statusDoc["auto_temp_f"] = state.auto_temp_f;
    statusDoc["timestamp"] = getISO8601Timestamp();
    
    String statusPayload;
    serializeJson(statusDoc, statusPayload);
    client.publish(statusTopic.c_str(), statusPayload.c_str());
    
    return;
  }
#endif // HAS_FAN_IR

#if HAS_MILIGHT
  // Handle MiLight/MIBO light commands — per-zone topics: cmd/light/{1-4}
  //                                     — broadcast topic: cmd/light/all
  String lightCmdPrefix = String("paku/edge/") + deviceId + "/cmd/light/";
  if (milightActive && String(topic).startsWith(lightCmdPrefix)) {
    String suffix = String(topic).substring(lightCmdPrefix.length());

    // ── Broadcast: cmd/light/all ─────────────────────────────────────────
    if (suffix == "all") {
      JsonDocument doc;
      DeserializationError error = deserializeJson(doc, message);
      if (error) {
        Serial.print("[MILIGHT] ALL parse error: ");
        Serial.println(error.c_str());
        return;
      }

      // Parse desired values from the single JSON payload
      bool want_on  = false;
      bool want_off = false;
      if (doc["state"].is<const char*>()) {
        const char* s = doc["state"];
        if (strcasecmp(s, "on") == 0)       want_on  = true;
        else if (strcasecmp(s, "off") == 0)  want_off = true;
      }
      int  new_brightness = doc["brightness"] | -1;   // -1 = not set
      int  new_color_temp = doc["color_temp"]  | -1;

      for (uint8_t ch = 1; ch <= 4; ch++) {
        MiLightState state = milight_get_state(ch);
        bool changed = false;

        if (want_on)  { state.on = true;  changed = true; }
        if (want_off) { state.on = false; changed = true; }
        if (new_brightness >= 0 && new_brightness <= 100) {
          state.brightness = new_brightness; changed = true;
        }
        if (new_color_temp >= 153 && new_color_temp <= 500) {
          state.color_temp = new_color_temp; changed = true;
        }

        if (changed && milight_send_state(state)) {
          Serial.printf("[MILIGHT] ALL zone %d → on:%d bright:%d temp:%d\n",
                        ch, state.on, state.brightness, state.color_temp);
#if HAS_RGB_LCD
          uint16_t colorTempK = (state.color_temp > 0)
              ? (uint16_t)(1000000UL / state.color_temp) : 4000;
          gui_set_light_zone(ch - 1, state.on, state.brightness, colorTempK);
#endif
          // Publish per-zone status so HA sensors pick up each zone
          String statusTopic = String("paku/edge/") + deviceId + "/status/light/" + ch;
          JsonDocument statusDoc;
          statusDoc["state"]      = state.on ? "ON" : "OFF";
          statusDoc["brightness"] = state.brightness;
          statusDoc["color_mode"] = "color_temp";
          statusDoc["color_temp"] = state.color_temp;
          statusDoc["channel"]    = ch;
          statusDoc["protocol"]   = milight_protocol_to_string(state.protocol);
          statusDoc["device_id"]  = state.device_id;
          statusDoc["timestamp"]  = getISO8601Timestamp();
          String statusPayload;
          serializeJson(statusDoc, statusPayload);
          client.publish(statusTopic.c_str(), statusPayload.c_str());
        }
      }
      return;
    }

    // ── Per-zone: cmd/light/{1-4} ────────────────────────────────────────
    // Extract channel from topic suffix
    uint8_t channel = suffix.toInt();
    if (channel < 1 || channel > 4) {
      Serial.printf("[MILIGHT] Invalid channel in topic: %s\n", topic);
      return;
    }

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, message);
    
    if (error) {
      Serial.print("[MILIGHT] CMD Parse error: ");
      Serial.println(error.c_str());
      return;
    }
    
    // Get current state
    MiLightState state = milight_get_state(channel);
    bool state_changed = false;
    
    // Check for special commands
    const char* cmd = doc["cmd"] | (const char*)nullptr;
    if (cmd) {
      if (strcmp(cmd, "pair") == 0) {
        const char* protocol_str = doc["protocol"] | "cct";
        MiLightProtocol protocol = milight_protocol_from_string(protocol_str);
        uint16_t device_id = doc["device_id"] | 0;
        
        if (milight_pair(channel, protocol, device_id)) {
          Serial.printf("[MILIGHT] Paired channel %d with protocol %s\n", channel, protocol_str);
          state.protocol = protocol;
          if (device_id != 0) {
            state.device_id = device_id;
          }
          state_changed = true;
        }
      } else if (strcmp(cmd, "unpair") == 0) {
        const char* protocol_str = doc["protocol"] | "cct";
        MiLightProtocol protocol = milight_protocol_from_string(protocol_str);
        
        if (milight_unpair(channel, protocol)) {
          Serial.printf("[MILIGHT] Unpaired channel %d\n", channel);
        }
      } else if (strcmp(cmd, "set_protocol") == 0) {
        const char* protocol_str = doc["protocol"];
        if (protocol_str) {
          state.protocol = milight_protocol_from_string(protocol_str);
          state_changed = true;
          Serial.printf("[MILIGHT] Protocol set to %s\n", protocol_str);
        }
      }
    }
    
    // Handle state commands (HA json schema: {"state":"ON"/"OFF"})
    if (doc["state"].is<const char*>()) {
      const char* state_str = doc["state"];
      if (strcasecmp(state_str, "on") == 0) {
        state.on = true;
        state_changed = true;
      } else if (strcasecmp(state_str, "off") == 0) {
        state.on = false;
        state_changed = true;
      } else if (strcasecmp(state_str, "toggle") == 0) {
        state.on = !state.on;
        state_changed = true;
      }
    }
    
    if (doc["brightness"].is<int>()) {
      uint8_t brightness = doc["brightness"];
      if (brightness <= 100) {
        state.brightness = brightness;
        state_changed = true;
      }
    }
    
    if (doc["color_temp"].is<int>()) {
      uint16_t color_temp = doc["color_temp"];
      if (color_temp >= 153 && color_temp <= 500) {
        state.color_temp = color_temp;
        state_changed = true;
      }
    }
    
    // Send state if changed
    if (state_changed) {
      if (milight_send_state(state)) {
        Serial.printf("[MILIGHT] Zone %d updated - on:%d bright:%d temp:%d\n",
                     channel, state.on, state.brightness, state.color_temp);

#if HAS_RGB_LCD
        // Sync Waveshare GUI — convert mireds (153-500) to Kelvin for GUI sliders
        uint16_t colorTempK = (state.color_temp > 0) ? (uint16_t)(1000000UL / state.color_temp) : 4000;
        gui_set_light_zone(channel - 1, state.on, state.brightness, colorTempK);
#endif
        
        // Publish status to per-zone topic
        String statusTopic = String("paku/edge/") + deviceId + "/status/light/" + channel;
        JsonDocument statusDoc;
        statusDoc["state"] = state.on ? "ON" : "OFF";
        statusDoc["brightness"] = state.brightness;
        statusDoc["color_mode"] = "color_temp";
        statusDoc["color_temp"] = state.color_temp;
        statusDoc["channel"] = channel;
        statusDoc["protocol"] = milight_protocol_to_string(state.protocol);
        statusDoc["device_id"] = state.device_id;
        statusDoc["timestamp"] = getISO8601Timestamp();
        
        String statusPayload;
        serializeJson(statusDoc, statusPayload);
        client.publish(statusTopic.c_str(), statusPayload.c_str());
      } else {
        Serial.println("[MILIGHT] ERROR: Failed to send state");
      }
    }
    
    return;
  }
#endif // HAS_MILIGHT

#if HAS_RGB_LCD
  // Handle EcoFlow power data — topic: paku/ecoflow/+/power
  if (String(topic).startsWith("paku/ecoflow/") && String(topic).endsWith("/power")) {
    JsonDocument doc;
    if (!deserializeJson(doc, message)) {
      float solarW   = doc["solar_w"]  | 0.0f;
      float acInW    = doc["ac_in_w"]  | 0.0f;
      float acOutW   = doc["ac_out_w"] | 0.0f;
      float dcOutW   = doc["dc_out_w"] | 0.0f;
      int   soc      = doc["soc"]      | 0;
      gui_set_power_data(solarW, acInW, acOutW, dcOutW);
      gui_set_header_battery(soc);
      LOG_INFO("ECOFLOW", "Power: solar=%.0fW acIn=%.0fW acOut=%.0fW dc=%.0fW soc=%d%%",
               solarW, acInW, acOutW, dcOutW, soc);
    }
    return;
  }
#endif

  // Check if this is an OTA command
  String otaTopic = String("paku/edge/") + deviceId + "/cmd/ota";
  if (String(topic) == otaTopic) {
    // Parse OTA command JSON
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, message);
    
    if (error) {
      Serial.print("OTA: Failed to parse JSON: ");
      Serial.println(error.c_str());
      return;
    }

    // Extract OTA parameters
    const char* url = doc["url"];
    const char* checksum = doc["checksum"];
    const char* version = doc["version"];

    if (!url || strlen(url) == 0) {
      Serial.println("OTA: Invalid command - missing URL");
      return;
    }

    // Store OTA parameters for processing
    pendingOtaUrl = String(url);
    pendingOtaChecksum = checksum ? String(checksum) : "";
    pendingOtaVersion = version ? String(version) : "unknown";
    otaUpdatePending = true;

    Serial.print("OTA: Update scheduled - URL: ");
    Serial.println(pendingOtaUrl);
    Serial.print("OTA: Target version: ");
    Serial.println(pendingOtaVersion);

    // Send acknowledgment
    String ackTopic = String("paku/edge/") + deviceId + "/ota/status";
    JsonDocument ackDoc;
    ackDoc["timestamp"] = getISO8601Timestamp();
    ackDoc["status"] = "accepted";
    ackDoc["current_version"] = FIRMWARE_VERSION;
    ackDoc["target_version"] = pendingOtaVersion;
    
    String ackPayload;
    serializeJson(ackDoc, ackPayload);
    client.publish(ackTopic.c_str(), ackPayload.c_str());
  }
}

/**
 * @brief OTA progress callback
 * 
 * Called periodically during OTA update to report progress.
 * Publishes progress updates via MQTT.
 * 
 * @param progress Current OTA progress information
 */
void otaProgressCallback(const OtaProgress& progress) {
  Serial.print("OTA Progress: ");
  Serial.print(progress.progressPercent);
  Serial.print("% - ");
  Serial.println(OtaClient::stateToString(progress.state));

#if HAS_DISPLAY
  // Update percentage text (right-aligned on the "Downloading" line)
  tft.fillRect(248, 65, 72, 18, TFT_BLACK);  // Clear previous percentage
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  // Right-align: 3 chars max ("100%") → start at x=260 for <= 99%, x=248 for 100%
  if (progress.progressPercent >= 100) {
    tft.setCursor(248, 65);
  } else {
    tft.setCursor(260, 65);
  }
  tft.printf("%d%%", progress.progressPercent);

  // Fill progress bar
  int barWidth = (int)(progress.progressPercent * 296.0 / 100.0);
  tft.fillRect(12, 97, barWidth, 21, TFT_GREEN);

  // Show downloaded/total and elapsed time below the bar
  tft.fillRect(10, 125, 300, 20, TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setCursor(10, 128);
  if (progress.totalBytes > 0) {
    tft.printf("%dKB / %dKB  %ds",
              progress.downloadedBytes / 1024,
              progress.totalBytes / 1024,
              progress.elapsedMs / 1000);
  }
#endif

  // Publish progress to MQTT
  String progressTopic = String("paku/edge/") + deviceId + "/ota/progress";
  JsonDocument progressDoc;
  progressDoc["timestamp"] = getISO8601Timestamp();
  progressDoc["state"] = OtaClient::stateToString(progress.state);
  progressDoc["percent"] = progress.progressPercent;
  progressDoc["downloaded"] = progress.downloadedBytes;
  progressDoc["total"] = progress.totalBytes;
  progressDoc["elapsed_ms"] = progress.elapsedMs;
  
  String progressPayload;
  serializeJson(progressDoc, progressPayload);
  client.publish(progressTopic.c_str(), progressPayload.c_str());
}

// NOTE: ESP-IDF 5.0+ and Arduino ESP32 3.0+ are now supported.
// The LEDC API differences are handled at lines 143-150.