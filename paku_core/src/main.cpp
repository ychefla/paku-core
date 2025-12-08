#ifdef ESP8266
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#endif

#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

#include "Arduino.h"
#include "device_config.h"  // Device selection and feature detection
#include "pin_config.h"
#include "PakuIotClient.h"
#include "sensor_placeholders.h"
#include "OtaClient.h"
#include <string>

// BLE support (ESP32 only)
#if HAS_BLE
#include "BLEDevice.h"
#include "ruuvi.h"
#include "ruuvi_scanner.h"
#ifndef ESP8266
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#endif // ESP8266
#endif // HAS_BLE

// Wired sensor support (ESP8266 and ESP32)
#if HAS_WIRED_SENSORS
#include "wired_sensors.h"
#endif // HAS_WIRED_SENSORS

// Display-related includes and definitions (only when display is available)
#if HAS_DISPLAY
#include "TFT_eSPI.h" /* Please use the TFT library provided in the library. */
#include "img_logo.h"

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

// BLE settings (ESP32 only)
#if HAS_BLE
bool scanBT_enabled = true;
// BLE scan interval in milliseconds between scan cycles
#define BLE_SCAN_INTERVAL_MS 10000
#endif // HAS_BLE

// Wired sensor settings (ESP8266 and ESP32)
#if HAS_WIRED_SENSORS
WiredSensors wiredSensors;
bool wiredSensorsEnabled = true;
unsigned long lastWiredSensorRead = 0;
#define WIRED_SENSOR_INTERVAL_MS 60000  // Read every 60 seconds
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
static char deviceId[20] = "";

// Enable placeholder sensor data generation for testing
// Set to false by default - only enable for local testing
bool generatePlaceholderData = false;

// WiFi settings (using arrays from secrets.h)
String wifi_status = "";


// MQTT settings
const char* mqtt_server = MQTT_SERVER;
const int mqtt_port = MQTT_PORT;

WiFiClient espClient;
PubSubClient client(espClient);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60000);  // GMT, update interval 1 minute

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

// Flow sensor pin - moved from GPIO2 to GPIO4 to avoid conflict with LED
// Note: GPIO2 is used for the onboard LED on many ESP32 dev boards
#define PIN_FLOW_SENSOR 4

unsigned long lastTime_sensor = 0;
unsigned long lastTime_mqtt = 0;
unsigned long mqttFastInterval = 10000;  // 10 second interval in ms
unsigned long mqttSlowInterval = 3600000;  // 1 hour interval in ms
unsigned long sensorFastInterval = 5000;  // 5 second interval in ms
unsigned long sensorSlowInterval = 60000;  // 1 minute interval in ms
unsigned long mqttInterval;
unsigned long sensorInterval;

volatile unsigned int count = 0;
float flowRate;
float calibrationFactor = 6.6;
float requiredDeltaT;
const float heaterPower = 5000;
bool testMode = true;  // Set to true to simulate flow data

// Default heater status to 1 (on)
int heaterStatus = 1;
struct Payload {
  String topic;
  String data;
};

Payload payloads[MAX_MQTT_PAYLOADS];
int payloadIndex = 0;

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
#if HAS_BLE
void initRuuviTags();
void createRuuviPayloads(const char* timestamp);
void scanBT(void* parameter);
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

// LED state tracking
static unsigned long lastHeartbeatTime = 0;
static const unsigned long HEARTBEAT_INTERVAL = 5000; // 5 seconds

/**
 * @brief Initialize the LED pin for status indication
 */
void ledInit() {
  pinMode(PIN_LED_BUILTIN, OUTPUT);
  digitalWrite(PIN_LED_BUILTIN, !LED_ON); // Start with LED off
}

/**
 * @brief Turn the LED on
 */
void ledOn() {
  digitalWrite(PIN_LED_BUILTIN, LED_ON);
}

/**
 * @brief Turn the LED off
 */
void ledOff() {
  digitalWrite(PIN_LED_BUILTIN, !LED_ON);
}

/**
 * @brief Blink the LED a specified number of times
 * @param count Number of blinks
 * @param onTime Duration LED is on in milliseconds
 * @param offTime Duration LED is off in milliseconds
 */
void ledBlink(int count, int onTime, int offTime) {
  for (int i = 0; i < count; i++) {
    ledOn();
    delay(onTime);
    ledOff();
    if (i < count - 1) {
      delay(offTime);
    }
  }
}

/**
 * @brief LED pattern for device startup (3 quick blinks)
 */
void ledStartup() {
  ledBlink(3, 100, 100);
}

/**
 * @brief LED pattern while connecting to WiFi (single fast blink)
 * Call this repeatedly during WiFi connection attempts
 * Note: Uses longer flash for better visibility
 */
void ledWifiConnecting() {
  ledOn();
  delay(100);
  ledOff();
  delay(100);
}

/**
 * @brief LED pattern when WiFi connected (solid ON briefly)
 */
void ledWifiConnected() {
  ledOn();
  delay(500);
  ledOff();
}

/**
 * @brief LED pattern while connecting to MQTT (slow blink)
 * Call this during MQTT connection attempts
 * Note: Uses longer flash for better visibility
 */
void ledMqttConnecting() {
  ledOn();
  delay(150);
  ledOff();
  delay(150);
}

/**
 * @brief LED pattern when MQTT connected (double blink)
 */
void ledMqttConnected() {
  ledBlink(2, 100, 100);
}

/**
 * @brief LED heartbeat pattern (brief flash)
 * Call this periodically to indicate normal operation
 */
void ledHeartbeat() {
  unsigned long currentTime = millis();
  if (currentTime - lastHeartbeatTime >= HEARTBEAT_INTERVAL) {
    ledBlink(1, 200, 0);  // 200ms flash for better visibility
    lastHeartbeatTime = currentTime;
  }
}

/**
 * @brief LED error pattern (5 rapid blinks)
 * Call this to indicate an error condition
 */
void ledError() {
  ledBlink(5, 100, 100);
}
#endif // HAS_LED

void IRAM_ATTR countRisingEdges() {
  count++;
}

/**
 * @brief Get ISO 8601 formatted timestamp string
 * 
 * Creates a timestamp in format: YYYY-MM-DDTHH:MM:SSZ
 * Uses NTP time to get accurate UTC time
 * 
 * @return String containing ISO 8601 formatted timestamp
 */
String getISO8601Timestamp() {
  unsigned long epochTime = timeClient.getEpochTime();
  
  // Calculate date components from epoch
  int year = 1970;
  int month = 1;
  int day = 1;
  
  // Simplified date calculation (good enough for this use case)
  unsigned long days = epochTime / 86400;
  unsigned long seconds = epochTime % 86400;
  
  int hours = seconds / 3600;
  seconds %= 3600;
  int minutes = seconds / 60;
  int secs = seconds % 60;
  
  // Calculate year (accounting for leap years)
  while (true) {
    int daysInYear = ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)) ? 366 : 365;
    if (days >= daysInYear) {
      days -= daysInYear;
      year++;
    } else {
      break;
    }
  }
  
  // Calculate month and day
  int daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  if ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)) {
    daysInMonth[1] = 29; // Leap year
  }
  
  for (int m = 0; m < 12; m++) {
    if (days >= daysInMonth[m]) {
      days -= daysInMonth[m];
      month++;
    } else {
      break;
    }
  }
  day += days;
  
  // Format as ISO 8601: YYYY-MM-DDTHH:MM:SSZ
  char isoTimestamp[32];
  snprintf(isoTimestamp, sizeof(isoTimestamp), 
           "%04d-%02d-%02dT%02d:%02d:%02dZ",
           year, month, day, hours, minutes, secs);
  
  return String(isoTimestamp);
}

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
    Serial.println("Starting setup...");
    Serial.print("Device: ");
    Serial.println(DEVICE_NAME);
    
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
    ledcWrite(0, 255);
#else
    ledcAttach(PIN_LCD_BL, 200, 8);
    ledcWrite(PIN_LCD_BL, 255);
#endif
#endif // HAS_DISPLAY

    Serial.println("Setup Wifi Connection...");
    WiFi.mode(WIFI_STA);
    //connect_wifi();

    // Initialize device ID from MAC address
    initDeviceId();

    Serial.println("Setup MQTT Connection...");
    timeClient.begin();
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(handleMqttMessage);  // Set MQTT message callback

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
#endif // HAS_BLE

#if HAS_WIRED_SENSORS
    // Initialize wired sensors (ESP8266 and ESP32)
    Serial.println("Setup Wired Sensors (I2C)...");
    if (wiredSensors.begin(PIN_I2C_SDA, PIN_I2C_SCL)) {
        Serial.print("Wired sensor type: ");
        Serial.println(wiredSensors.getSensorType());
    } else {
        Serial.println("Warning: No wired sensors detected");
        wiredSensorsEnabled = false;
    }
#endif // HAS_WIRED_SENSORS

    Serial.println("Setup Flow Sensor on GPIO4...");
    pinMode(PIN_FLOW_SENSOR, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_FLOW_SENSOR), countRisingEdges, RISING);
    
    // Initialize intervals based on heater status
    updateIntervals();
    
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

    Serial.println("Setup complete.");

  }

/**
 * @brief Main loop function that handles WiFi and MQTT connections, sensor data processing, 
 *        and payload creation for various metrics.
 * 
 * This function performs the following tasks:
 * - Checks and maintains WiFi connection.
 * - Checks and maintains MQTT connection.
 * - Updates the time client.
 * - Processes flow data and calculates flow rate and required temperature delta.
 * - Creates payloads for humidity, temperature, flow, heating power, battery voltage, and heater status.
 * - Sends payloads to MQTT broker at specified intervals.
 * 
 * The function uses the following global variables:
 * - WiFi: WiFi connection object.
 * - client: MQTT client object.
 * - timeClient: NTP time client object.
 * - lastTime_sensor: Timestamp of the last sensor data processing.
 * - sensorInterval: Interval for sensor data processing.
 * - count: Pulse count from the flow sensor.
 * - testMode: Flag to enable test mode with random pulse counts.
 * - calibrationFactor: Calibration factor for flow rate calculation.
 * - heaterPower: Power of the heater.
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
 * - countRisingEdges(): Interrupt service routine for counting rising edges of flow sensor pulses.
 */
/**
 * @brief Main loop function that handles WiFi and MQTT connections, updates time, 
 *        processes flow data, and sends data to MQTT.
 * 
 * This function performs the following tasks:
 * - Checks and maintains WiFi connection.
 * - Checks and maintains MQTT connection.
 * - Calls the loop function of the MQTT client.
 * - Updates the time using the time client.
 * - Updates the heater status (placeholder for actual implementation).
 * - Updates intervals based on the heater status.
 * - Processes flow data.
 * - Sends data to the MQTT broker.
 */
void loop() {
  static unsigned long loopCount = 0;
  loopCount++;
  
  // Update heater status here
  // heaterStatus = ...;  // Retrieve the actual heater status
  // Update intervals based on heater status
  updateIntervals();
  processData();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, calling connect_wifi...");
    connect_wifi();
  }
  if (!client.connected()) {
    Serial.println("MQTT not connected, calling connectMQTT...");
    connectMQTT();
  }

#if HAS_LED
  // LED heartbeat - brief flash every 5 seconds during normal operation
  ledHeartbeat();
#endif

  updateDisplay(); 
  client.loop();
  timeClient.update();
  sendToMQTT();
  sendToPakuIot();
  
  // Process pending OTA updates (only when triggered via MQTT)
  if (otaUpdatePending) {
    processOtaUpdate();
  }
  
  goToSleep();

}

void goToSleep() {
    // Check if the device has been awake for 30 seconds
  static unsigned long awakeStartTime = millis();
  if (millis() - awakeStartTime >= 30000) {
    Serial.println("Going to sleep for 15 seconds...");
#if HAS_DISPLAY
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(0, 0);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(2);
    tft.println("Going to sleep for 15 seconds...");
    delay(2000);
#endif
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
}

void updateDisplay() {
#if HAS_DISPLAY
  unsigned long currentTime = millis();
  if (currentTime - lastTime_sensor >= TFT_UPDATE_WAIT) {
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(0, 0);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(2);
    tft.println("Paku-Core");
    tft.setTextSize(1);
    tft.println(wifi_status);
    if (!client.connected()){
      tft.println("MQTT disconnected");
    }else{
      tft.println("MQTT connected");
    }
    
    //tft.println("Time: " + timeClient.getFormattedTime());
    //tft.println("Flow: " + String(flowRate, 1) + " L/min | dT: " + String(requiredDeltaT, 1) + "C");
    tft.println("");
    
    // Display RuuviTag data
    tft.setTextSize(2);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.println("Ruuvi Tags");
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    
    const RuuviTag* freshTags[MAX_RUUVI_TAGS];
    uint8_t freshCount = getFreshTags(freshTags, MAX_RUUVI_TAGS, millis());
    
    if (freshCount == 0) {
      tft.setTextColor(TFT_YELLOW, TFT_BLACK);
      tft.println("No data available");
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
    } else {
      for (uint8_t i = 0; i < freshCount && i < 4; i++) {  // Show max 4 tags to fit screen
        const RuuviTag* tag = freshTags[i];
        if (tag->hasData && tag->lastData.valid) {
          tft.print(tag->location);
          tft.print(": ");
          tft.print(String(tag->lastData.temperature, 1));
          tft.print("C ");
          tft.print(String(tag->lastData.humidity, 0));
          tft.println("%");
        }
      }
    }
  }
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
 
    // process flow data
    detachInterrupt(digitalPinToInterrupt(PIN_FLOW_SENSOR));
    
    if (testMode) {
      count = random(198, 462);
    }

    // Calculate frequency in Hz (pulses per second)
    float frequency = count / ((currentTime - lastTime_sensor) / 1000.0); 
    flowRate = (frequency / calibrationFactor) * 60.0;
    requiredDeltaT = heaterPower / (3.5 * flowRate);
    String timestamp = getISO8601Timestamp();

    count = 0;
    lastTime_sensor = currentTime;
    attachInterrupt(digitalPinToInterrupt(PIN_FLOW_SENSOR), countRisingEdges, RISING);
 
    // Create payloads for all data
    
#if HAS_BLE
    // 1. RuuviTag sensor data (temperature, humidity, pressure from BLE sensors)
    createRuuviPayloads(timestamp.c_str());
#endif // HAS_BLE
    
#if HAS_WIRED_SENSORS
    // 2. Wired sensor data (BME280, etc. - ESP8266 and ESP32)
    createWiredSensorPayloads(timestamp.c_str());
#endif // HAS_WIRED_SENSORS
    
#if HAS_BLE
    // 3. Placeholder data for sensors not yet implemented (disabled by default)
    createPlaceholderPayloads(timestamp.c_str());
#endif // HAS_BLE
    
    // 4. Flow sensor data - consolidated payload with all metrics
    JsonDocument flowDoc;
    flowDoc["timestamp"] = timestamp;
    flowDoc["device_id"] = "coolant";
    flowDoc["location"] = "coolant_line";
    
    JsonObject flowMetrics = flowDoc["metrics"].to<JsonObject>();
    flowMetrics["flow_rate_lpm"] = flowRate;
    flowMetrics["frequency_hz"] = frequency;
    flowMetrics["required_dt_c"] = requiredDeltaT;
    
    String flowPayload;
    serializeJson(flowDoc, flowPayload);
    
    if (payloadIndex < MAX_MQTT_PAYLOADS) {
      payloads[payloadIndex].topic = "paku/flow/coolant/data";
      payloads[payloadIndex].data = flowPayload;
      payloadIndex++;
    }
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
    Serial.println("send to MQTT");
    
    for (int i = 0; i < payloadIndex; i++) {
      client.publish((char*) payloads[i].topic.c_str(), (char*) payloads[i].data.c_str());
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
  snprintf(deviceId, sizeof(deviceId), "paku-%02X%02X%02X%02X", 
           mac[2], mac[3], mac[4], mac[5]);
  
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
    
    // Only send actual sensor values (not placeholder -1000 values)
    if (flowRate > 0) {
      readings[readingCount].metric = "flow/coolant";
      readings[readingCount].value = flowRate;
      readings[readingCount].unit = "l_per_min";
      readings[readingCount].timestamp = timestampBuf;
      readingCount++;
    }
    
    if (requiredDeltaT > 0 && requiredDeltaT < 1000) {
      readings[readingCount].metric = "temperature/heating/required_dt";
      readings[readingCount].value = requiredDeltaT;
      readings[readingCount].unit = "celsius";
      readings[readingCount].timestamp = timestampBuf;
      readingCount++;
    }
    
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
 * @brief Attempts to connect to a WiFi network from a list of SSIDs and passwords.
 * 
 * This function iterates through a predefined list of WiFi SSIDs and passwords,
 * attempting to connect to each one until a successful connection is made or all
 * options are exhausted. It prints the connection status to the Serial monitor.
 * 
 * @note The function will delay for 10 milliseconds at the start and will print
 *       connection attempts and results to the Serial monitor.
 * 
 * @details The function will try to connect to each SSID in the list. For each SSID,
 *          it will attempt to connect up to 20 times, with a 500-millisecond delay
 *          between each attempt. If a connection is established, it prints the IP
 *          address assigned to the device. If the connection fails, it moves on to
 *          the next SSID in the list.
 * 
 * @warning Ensure that the `wifi_ssid` and `wifi_password` arrays are properly defined
 *          and contain valid SSIDs and passwords.
 */
void connect_wifi() {
  
  delay(10);
  Serial.println();

  while (WiFi.status() != WL_CONNECTED) {
    for (int i = 0; i < WIFI_COUNT; i++) {
      Serial.print("Connecting to ");
      Serial.print(WIFI_SSIDS[i]);
      wifi_status = "Wifi connecting to ";
      wifi_status += WIFI_SSIDS[i];
      WiFi.begin(WIFI_SSIDS[i], WIFI_PASSWORDS[i]);
      int attempts = 0;

      while (WiFi.status() != WL_CONNECTED && attempts < 10) {
        delay(500);
        //Serial.print(".");
        wifi_status += ".";
        attempts++;
#if HAS_LED
        ledWifiConnecting();  // Fast blink while connecting
#endif
        updateDisplay();
      }

      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("");
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
        wifi_status = "WiFi connected to ";
        wifi_status += WIFI_SSIDS[i];
        wifi_status += " (";
        wifi_status += WiFi.localIP().toString();
        wifi_status += ")";
#if HAS_LED
        ledWifiConnected();  // Solid ON briefly to indicate success
#endif
        updateDisplay();
        return;
      } else {
        Serial.println("");
        Serial.print("Failed to connect to ");
        Serial.println(WIFI_SSIDS[i]);
#if HAS_LED
        ledError();  // Error blink for failed connection
#endif
      }
    }
  }
}

/**
 * @brief Attempts to establish a connection to the MQTT broker.
 * 
 * This function continuously tries to connect to the MQTT broker until a connection is established.
 * If the connection is successful, it subscribes to the "paku/control" topic.
 * If the connection fails, it prints the failure reason and retries after a 5-second delay.
 * 
 * @note This function blocks execution until a connection is established.
 */
void connectMQTT() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
#if HAS_LED
    ledMqttConnecting();  // Slow blink while connecting
#endif
    //tft.fillScreen(TFT_BLACK);
    //tft.setCursor(0, 0);
    //tft.setTextColor(TFT_WHITE, TFT_BLACK);
    //tft.setTextSize(2);
    //tft.println("Attempting MQTT connection...");

    if (client.connect("ESP32Client")) {  // Use a unique client ID for ESP32
      Serial.println("MQTT connected and subscribed to control topics");
#if HAS_LED
      ledMqttConnected();  // Double blink to indicate success
#endif
      //tft.println("MQTT connected and subscribed to 'paku/control'");
      client.subscribe("paku/control");
      
      // Subscribe to device-specific OTA command topic
      String otaTopic = String("paku/devices/") + deviceId + "/cmd/ota";
      client.subscribe(otaTopic.c_str());
      Serial.print("Subscribed to OTA topic: ");
      Serial.println(otaTopic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
#if HAS_LED
      ledError();  // Error blink for failed connection
#endif
      //tft.print("failed, rc=");
      //tft.print(client.state());
      //tft.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

/**
 * @brief Initializes the device ID from MAC address
 * 
 * Generates a unique device identifier using the last 4 bytes of the
 * ESP32's MAC address in the format "paku-AABBCCDD".
 */
void initDeviceId() {
  uint8_t mac[6];
  WiFi.macAddress(mac);
  snprintf(deviceId, sizeof(deviceId), "paku-%02X%02X%02X%02X", 
           mac[2], mac[3], mac[4], mac[5]);
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
  
#if RUUVI_TAG_COUNT > 0
  Serial.println("Registering known RuuviTags...");
  for (int i = 0; i < RUUVI_TAG_COUNT; i++) {
    if (registerRuuviTag(RUUVI_TAG_MACS[i], RUUVI_TAG_LOCATIONS[i])) {
      Serial.print("  Registered: ");
      Serial.print(RUUVI_TAG_MACS[i]);
      Serial.print(" -> ");
      Serial.println(RUUVI_TAG_LOCATIONS[i]);
    }
  }
#else
  Serial.println("No pre-registered RuuviTags (auto-discovery enabled)");
#endif
  
  Serial.print("RuuviTag scanner initialized. Registered tags: ");
  Serial.println(getRegisteredTagCount());
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
    doc["device_id"] = String("ruuvi_") + tag->location;
    doc["location"] = tag->location;
    doc["mac"] = tag->macString;
    
    JsonObject metrics = doc["metrics"].to<JsonObject>();
    metrics["temperature_c"] = tag->lastData.temperature;
    metrics["humidity_percent"] = tag->lastData.humidity;
    
    if (tag->lastData.pressure > 0) {
      metrics["pressure_hpa"] = tag->lastData.pressure / 100.0f;
    }
    
    if (tag->lastData.batteryVoltage > 0) {
      metrics["battery_mv"] = tag->lastData.batteryVoltage;
    }
    
    // Serialize and add to payload queue
    String payload;
    serializeJson(doc, payload);
    
    // Construct device_id as ruuvi_<location>
    String deviceId = String("ruuvi_") + tag->location;
    String topic = String("paku/sensors/") + deviceId + "/data";
    
    if (payloadIndex < MAX_MQTT_PAYLOADS) {
      payloads[payloadIndex].topic = topic;
      payloads[payloadIndex].data = payload;
      payloadIndex++;
    }
    
    Serial.print("RuuviTag [");
    Serial.print(tag->location);
    Serial.print("]: T=");
    Serial.print(tag->lastData.temperature);
    Serial.print("°C, H=");
    Serial.print(tag->lastData.humidity);
    Serial.println("%");
  }
}
#endif // HAS_BLE

#if HAS_WIRED_SENSORS
// Device ID suffix for wired sensors
#define WIRED_SENSOR_SUFFIX "_wired"

/**
 * @brief Creates MQTT payloads from wired sensor data (BME280, etc.)
 * 
 * Reads temperature, humidity, and pressure from wired I2C sensors
 * and creates payloads using architecture-compliant topic structure.
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
    Serial.println("Warning: Wired sensor reading invalid");
    return;
  }
  
  // Create consolidated payload with all metrics
  JsonDocument doc;
  doc["timestamp"] = String(timestamp);
  doc["device_id"] = String(deviceId) + WIRED_SENSOR_SUFFIX;
  doc["location"] = "wired_sensor";
  doc["sensor_type"] = wiredSensors.getSensorType();
  
  JsonObject metrics = doc["metrics"].to<JsonObject>();
  metrics["temperature_c"] = data.temperature;
  metrics["humidity_percent"] = data.humidity;
  metrics["pressure_hpa"] = data.pressure;
  
  String payload;
  serializeJson(doc, payload);
  
  String topic = String("paku/sensors/") + deviceId + WIRED_SENSOR_SUFFIX + "/data";
  
  if (payloadIndex < MAX_MQTT_PAYLOADS) {
    payloads[payloadIndex].topic = topic;
    payloads[payloadIndex].data = payload;
    payloadIndex++;
  }
  
  Serial.print("Wired Sensor: T=");
  Serial.print(data.temperature);
  Serial.print("°C, H=");
  Serial.print(data.humidity);
  Serial.print("%, P=");
  Serial.print(data.pressure);
  Serial.println(" hPa");
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
    pBLEScan->setActiveScan(true);  // Set active scan to get more data
    pBLEScan->setInterval(100);     // Set scan interval
    pBLEScan->setWindow(99);        // Set scan window

    // Start scanning for 5 seconds
    BLEScanResults foundDevices = pBLEScan->start(5, false);

    // Print the number of devices found
    Serial.print("Devices found: ");
    Serial.println(foundDevices.getCount());

    // Iterate through the found devices and check for RuuviTags
    for (int i = 0; i < foundDevices.getCount(); i++) {
      BLEAdvertisedDevice device = foundDevices.getDevice(i);
      
      // Check if device has manufacturer data
      if (device.haveManufacturerData()) {
        std::string mfData = device.getManufacturerData();
        
        // Check minimum length for Ruuvi data
        if (mfData.length() >= 2) {
          // Get manufacturer ID (little-endian)
          uint16_t manufacturerId = (uint8_t)mfData[0] | ((uint8_t)mfData[1] << 8);
          
          // Check if this is Ruuvi data
          if (isRuuviManufacturer(manufacturerId)) {
            Serial.print("RuuviTag found: ");
            Serial.println(device.getAddress().toString().c_str());
            
            // Get MAC address bytes
            uint8_t macBytes[6];
            esp_bd_addr_t* nativeAddr = device.getAddress().getNative();
            for (int j = 0; j < 6; j++) {
              macBytes[j] = (*nativeAddr)[j];
            }
            
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
  static unsigned long heaterOnStartTime = 0;

  if (heaterStatus == 1) {
    if (heaterOnStartTime == 0) {
      heaterOnStartTime = millis();
    }

    if (millis() - heaterOnStartTime >= 3600000) { // 1 hour in milliseconds
      mqttInterval = mqttSlowInterval;
      sensorInterval = sensorSlowInterval;
    } else {
      mqttInterval = mqttFastInterval;
      sensorInterval = sensorFastInterval;
    }
  } else {
    heaterOnStartTime = 0;
    mqttInterval = mqttSlowInterval;
    sensorInterval = sensorSlowInterval;
  }
}

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
  String resultTopic = String("paku/devices/") + deviceId + "/ota/result";
  JsonDocument resultDoc;
  resultDoc["timestamp"] = getISO8601Timestamp();
  resultDoc["version"] = pendingOtaVersion;
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
    // Short delay to allow MQTT message to be sent, then reboot
    unsigned long rebootTime = millis();
    while (millis() - rebootTime < 3000) {
      client.loop();  // Allow MQTT to process messages
      delay(100);
    }
    ESP.restart();
  } else {
    Serial.print("OTA: Update failed: ");
    Serial.println(otaClient.getLastError());
  }
}

/**
 * @brief MQTT message callback handler
 * 
 * Handles incoming MQTT messages, including OTA update commands.
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

  // Check if this is an OTA command
  String otaTopic = String("paku/devices/") + deviceId + "/cmd/ota";
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
    String ackTopic = String("paku/devices/") + deviceId + "/ota/status";
    JsonDocument ackDoc;
    ackDoc["timestamp"] = getISO8601Timestamp();
    ackDoc["status"] = "accepted";
    ackDoc["version"] = pendingOtaVersion;
    
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

  // Publish progress to MQTT
  String progressTopic = String("paku/devices/") + deviceId + "/ota/progress";
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