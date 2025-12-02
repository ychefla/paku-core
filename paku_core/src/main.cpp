#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include "BLEDevice.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "Arduino.h"
#include "TFT_eSPI.h" /* Please use the TFT library provided in the library. */
#include "img_logo.h"
#include "pin_config.h"
#include "PakuIotClient.h"
#include "ruuvi.h"
#include "ruuvi_scanner.h"
#include "frezzer.h"
#include "frezzer_controller.h"
#include "sensor_placeholders.h"
#include <string>

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

// BT settings
bool scanBT_enabled = true;

// BLE scan interval in milliseconds between scan cycles
#define BLE_SCAN_INTERVAL_MS 10000

// Maximum number of telemetry readings per HTTP batch
#define MAX_TELEMETRY_READINGS 20

// Ruuvi tag configuration - placeholder MAC addresses for known tags
// These should be configured in secrets.h for production deployments
#ifndef RUUVI_TAG_COUNT
#define RUUVI_TAG_COUNT 0
#endif
// Example: In secrets.h, define known Ruuvi tags like:
// #define RUUVI_TAG_COUNT 4
// static const char* RUUVI_TAG_MACS[] = {"AA:BB:CC:DD:EE:01", "AA:BB:CC:DD:EE:02", "AA:BB:CC:DD:EE:03", "AA:BB:CC:DD:EE:04"};
// static const char* RUUVI_TAG_LOCATIONS[] = {"cabin", "kitchen", "lounge", "dryer"};

// Frezzer compressor fridge configuration
// These should be configured in secrets.h for production deployments
#ifndef FREZZER_COUNT
#define FREZZER_COUNT 0
#endif
// Example: In secrets.h, define known Frezzer fridges like:
// #define FREZZER_COUNT 1
// static const char* FREZZER_MACS[] = {"AA:BB:CC:DD:EE:FF"};
// static const char* FREZZER_LOCATIONS[] = {"van_fridge"};

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

#define PIN 2

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

Payload payloads[30];
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
void initRuuviTags();
void createRuuviPayloads(const char* timestamp);
void createPlaceholderPayloads(const char* timestamp);
void initDeviceId();
void scanBT(void* parameter);
void goToSleep();
void updateDisplay();
void initFrezzerDevices();
void createFrezzerPayloads(const char* timestamp);
void handleFrezzerMqttCommand(const char* topic, const char* payload);

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

    Serial.println("Setup Wifi Connection...");
    WiFi.mode(WIFI_STA);
    //connect_wifi();

    // Initialize device ID from MAC address
    initDeviceId();

    Serial.println("Setup MQTT Connection...");
    timeClient.begin();
    client.setServer(mqtt_server, mqtt_port);

    // Initialize paku-iot HTTP client if enabled
    initPakuIot();

    // Initialize RuuviTag scanner
    Serial.println("Setup RuuviTag Scanner...");
    initRuuviTags();

    // Initialize Frezzer controller
    Serial.println("Setup Frezzer Controller...");
    initFrezzerDevices();

    Serial.println("Setup Sensor...");
    pinMode(PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN), countRisingEdges, RISING);
    
    // Initialize intervals based on heater status
    updateIntervals();
    
    // Create a task for scanning Bluetooth devices
    xTaskCreate(
      scanBT,          // Function to be called
      "scanBT",        // Name of the task
      10000,           // Stack size (bytes)
      NULL,            // Parameter to pass
      1,               // Task priority
      NULL             // Task handle
    );

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
  // Update heater status here
  // heaterStatus = ...;  // Retrieve the actual heater status
  // Update intervals based on heater status
  updateIntervals();
  processData();

  if (WiFi.status() != WL_CONNECTED) {
    connect_wifi();
  }
  if (!client.connected()) {
      connectMQTT();
  }

  updateDisplay(); 
  client.loop();
  timeClient.update();
  sendToMQTT();
  sendToPakuIot();  
  goToSleep();

}

void goToSleep() {
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
    // Configure the ESP32 to wake up after 15 seconds
    esp_sleep_enable_timer_wakeup(15 * 1000000);
    esp_deep_sleep_start();
  } else {
    delay(1000);
  }
}

void updateDisplay() {
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
    
    // Display Frezzer fridge data first (most important for van use)
    const FrezzerDevice* freshFridges[MAX_FREZZER_DEVICES];
    uint8_t fridgeCount = getFreshFrezzerDevices(freshFridges, MAX_FREZZER_DEVICES, millis());
    
    if (fridgeCount > 0) {
      tft.setTextSize(2);
      tft.setTextColor(TFT_BLUE, TFT_BLACK);
      tft.println("Fridge");
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      
      for (uint8_t i = 0; i < fridgeCount && i < 2; i++) {  // Show max 2 fridges
        const FrezzerDevice* fridge = freshFridges[i];
        if (fridge->hasData && fridge->lastData.valid) {
          tft.print(fridge->location);
          tft.print(": ");
          tft.print(String(fridge->lastData.currentTemp, 1));
          tft.print("C/");
          tft.print(String(fridge->lastData.targetTemp, 0));
          tft.print("C ");
          if (fridge->lastData.compressor == FrezzerCompressorState::RUNNING) {
            tft.setTextColor(TFT_GREEN, TFT_BLACK);
            tft.println("ON");
          } else {
            tft.setTextColor(TFT_YELLOW, TFT_BLACK);
            tft.println("OFF");
          }
          tft.setTextColor(TFT_WHITE, TFT_BLACK);
        }
      }
      tft.println("");
    }
    
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
      // Adjust max tags shown based on whether fridge data is displayed
      uint8_t maxTags = (fridgeCount > 0) ? 2 : 4;
      for (uint8_t i = 0; i < freshCount && i < maxTags; i++) {
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
    detachInterrupt(digitalPinToInterrupt(PIN));
    
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
    attachInterrupt(digitalPinToInterrupt(PIN), countRisingEdges, RISING);
 
    // Create payloads for all data
    
    // 1. RuuviTag sensor data (temperature, humidity, pressure from BLE sensors)
    createRuuviPayloads(timestamp.c_str());
    
    // 2. Frezzer fridge data (temperature, status from BLE compressor fridges)
    createFrezzerPayloads(timestamp.c_str());
    
    // 3. Placeholder data for sensors not yet implemented (disabled by default)
    createPlaceholderPayloads(timestamp.c_str());
    
    // 3. Flow sensor data - consolidated payload with all metrics
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
    
    if (payloadIndex < 30) {
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
        wifi_status += WiFi.localIP();
        wifi_status += ")";
        updateDisplay();
        return;
      } else {
        Serial.println("");
        Serial.print("Failed to connect to ");
        Serial.println(WIFI_SSIDS[i]);
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
    //tft.fillScreen(TFT_BLACK);
    //tft.setCursor(0, 0);
    //tft.setTextColor(TFT_WHITE, TFT_BLACK);
    //tft.setTextSize(2);
    //tft.println("Attempting MQTT connection...");

    if (client.connect("ESP32Client")) {  // Use a unique client ID for ESP32
      Serial.println("MQTT connected and subscribed to 'paku/control'");
      //tft.println("MQTT connected and subscribed to 'paku/control'");
      client.subscribe("paku/control");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
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
    
    if (payloadIndex < 30) {
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
      
      // Get device name if available
      const char* devName = nullptr;
      if (device.haveName()) {
        devName = device.getName().c_str();
      }
      
      // Get MAC address bytes
      uint8_t macBytes[6];
      esp_bd_addr_t* nativeAddr = device.getAddress().getNative();
      for (int j = 0; j < 6; j++) {
        macBytes[j] = (*nativeAddr)[j];
      }
      
      // Check if this is a Frezzer device by name
      if (devName != nullptr && isFrezzerDevice(devName)) {
        Serial.print("Frezzer device found: ");
        Serial.print(devName);
        Serial.print(" - ");
        Serial.println(device.getAddress().toString().c_str());
        
        // Update Frezzer device in our registry
        int rssi = device.getRSSI();
        if (updateFrezzerFromScan(macBytes, devName, rssi, millis())) {
          Serial.println("  -> Frezzer device registered/updated");
        }
      }
      
      // Check if device has manufacturer data (for RuuviTag)
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
 * @note The function ensures that the payloadIndex does not exceed the size of the payloads array (assumed to be 30).
 */
void createPayload(String topic, float value, String timestamp) {
  if (payloadIndex < 30) { // Ensure we don't exceed array size
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
 * Iterates through all registered devices with fresh data and creates
 * temperature, status, and control payloads using the architecture-compliant
 * topic structure: paku/fridge/{location}/data
 * 
 * @param timestamp Current timestamp string
 */
void createFrezzerPayloads(const char* timestamp) {
  const FrezzerDevice* freshDevices[MAX_FREZZER_DEVICES];
  uint8_t freshCount = getFreshFrezzerDevices(freshDevices, MAX_FREZZER_DEVICES, millis());
  
  for (uint8_t i = 0; i < freshCount; i++) {
    const FrezzerDevice* device = freshDevices[i];
    if (!device->hasData || !device->lastData.valid) continue;
    
    // Build consolidated payload with all metrics for this device
    JsonDocument doc;
    doc["timestamp"] = String(timestamp);
    doc["device_id"] = String("frezzer_") + device->location;
    doc["location"] = device->location;
    doc["mac"] = device->macString;
    if (strlen(device->deviceName) > 0) {
      doc["device_name"] = device->deviceName;
    }
    
    JsonObject metrics = doc["metrics"].to<JsonObject>();
    metrics["current_temp_c"] = device->lastData.currentTemp;
    metrics["target_temp_c"] = device->lastData.targetTemp;
    metrics["battery_voltage"] = device->lastData.batteryVoltage;
    metrics["mode"] = frezzerModeToString(device->lastData.mode);
    metrics["compressor"] = frezzerCompressorStateToString(device->lastData.compressor);
    metrics["power_level"] = device->lastData.powerLevel;
    metrics["lid_open"] = device->lastData.lidOpen;
    metrics["low_voltage_protection"] = device->lastData.lowVoltageProtection;
    metrics["error"] = frezzerErrorToString(device->lastData.error);
    metrics["connected"] = device->connected;
    
    // Serialize and add to payload queue
    String payload;
    serializeJson(doc, payload);
    
    // Construct topic as paku/fridge/{location}/data
    String topic = String("paku/fridge/") + device->location + "/data";
    
    if (payloadIndex < 30) {
      payloads[payloadIndex].topic = topic;
      payloads[payloadIndex].data = payload;
      payloadIndex++;
    }
    
    Serial.print("Frezzer [");
    Serial.print(device->location);
    Serial.print("]: T=");
    Serial.print(device->lastData.currentTemp);
    Serial.print("°C, Target=");
    Serial.print(device->lastData.targetTemp);
    Serial.print("°C, Mode=");
    Serial.println(frezzerModeToString(device->lastData.mode));
  }
}

/**
 * @brief Handles MQTT commands for Frezzer devices
 * 
 * Parses incoming MQTT commands and sends them to the appropriate Frezzer device.
 * Command format: {"command": "set_temp", "value": -8.0}
 * 
 * @param topic MQTT topic (paku/fridge/{location}/cmd)
 * @param payload JSON command payload
 */
void handleFrezzerMqttCommand(const char* topic, const char* payload) {
  // Parse topic to extract location
  // Expected format: paku/fridge/{location}/cmd
  String topicStr = String(topic);
  if (!topicStr.startsWith("paku/fridge/") || !topicStr.endsWith("/cmd")) {
    return;
  }
  
  // Extract location from topic
  int startIdx = 12;  // Length of "paku/fridge/"
  int endIdx = topicStr.lastIndexOf("/cmd");
  if (endIdx <= startIdx) return;
  
  String location = topicStr.substring(startIdx, endIdx);
  
  // Find the device
  // Note: This is a simplified lookup - in production you'd want a more efficient method
  const FrezzerDevice* device = nullptr;
  for (uint8_t i = 0; i < getFrezzerDeviceCount(); i++) {
    const FrezzerDevice* d = getFrezzerDevice(i);
    if (d != nullptr && String(d->location) == location) {
      device = d;
      break;
    }
  }
  
  if (device == nullptr) {
    Serial.print("Frezzer command: device not found for location ");
    Serial.println(location);
    return;
  }
  
  // Parse command JSON
  JsonDocument cmdDoc;
  DeserializationError error = deserializeJson(cmdDoc, payload);
  if (error) {
    Serial.print("Frezzer command: JSON parse error - ");
    Serial.println(error.c_str());
    return;
  }
  
  const char* command = cmdDoc["command"];
  if (command == nullptr) {
    Serial.println("Frezzer command: missing 'command' field");
    return;
  }
  
  // Note: We need a non-const pointer for control functions
  // This is safe because we're modifying the internal device state
  FrezzerDevice* mutableDevice = const_cast<FrezzerDevice*>(device);
  
  FrezzerResult result = FrezzerResult::UNKNOWN_ERROR;
  
  if (strcmp(command, "set_temp") == 0) {
    float temp = cmdDoc["value"].as<float>();
    result = setFrezzerTargetTemp(mutableDevice, temp);
    Serial.print("Frezzer set_temp ");
    Serial.print(temp);
  } else if (strcmp(command, "set_mode") == 0) {
    const char* modeStr = cmdDoc["value"];
    FrezzerMode mode = FrezzerMode::UNKNOWN;
    if (strcmp(modeStr, "off") == 0) mode = FrezzerMode::OFF;
    else if (strcmp(modeStr, "fridge") == 0) mode = FrezzerMode::FRIDGE;
    else if (strcmp(modeStr, "freezer") == 0) mode = FrezzerMode::FREEZER;
    else if (strcmp(modeStr, "eco") == 0) mode = FrezzerMode::ECO;
    else if (strcmp(modeStr, "max_cool") == 0) mode = FrezzerMode::MAX_COOL;
    
    result = setFrezzerMode(mutableDevice, mode);
    Serial.print("Frezzer set_mode ");
    Serial.print(modeStr);
  } else if (strcmp(command, "power") == 0) {
    const char* powerStr = cmdDoc["value"];
    if (strcmp(powerStr, "on") == 0) {
      result = turnFrezzerOn(mutableDevice);
    } else if (strcmp(powerStr, "off") == 0) {
      result = turnFrezzerOff(mutableDevice);
    }
    Serial.print("Frezzer power ");
    Serial.print(powerStr);
  } else {
    Serial.print("Frezzer command: unknown command ");
    Serial.println(command);
    return;
  }
  
  Serial.print(" -> ");
  Serial.println(frezzerResultToString(result));
}

// TFT Pin check
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

// NOTE: ESP-IDF 5.0+ and Arduino ESP32 3.0+ are now supported.
// The LEDC API differences are handled at lines 143-150.