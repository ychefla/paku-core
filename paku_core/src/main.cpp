#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

// WiFi settings
const char* wifi_ssid[] = {"iPhone", "JosPar_2.4G", "JosPar_paku"};
const char* wifi_password[] = {"1234567890", "V11vuskaBaabuska", "V11vuskaBaabuska"};
const char* mqtt_server = "static.107.192.27.37.clients.your-server.de";
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60000);  // GMT, update interval 1 minute

#define PIN 2

unsigned long lastTime_sensor = 0;
unsigned long lastTime_mqtt = 0;
unsigned long mqttFastInterval = 5000;  // 1 minute interval in ms
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

void IRAM_ATTR countRisingEdges() {
  count++;
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
    

    Serial.println("Setup Wifi Connection...");
    WiFi.mode(WIFI_STA);
    connect_wifi();

    Serial.println("Setup MQTT Connection...");
    timeClient.begin();
    client.setServer(mqtt_server, mqtt_port);

    Serial.println("Setup Sensor...");
    pinMode(PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN), countRisingEdges, RISING);
    
    // Initialize intervals based on heater status
    updateIntervals();
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
void loop() {

  if (WiFi.status() != WL_CONNECTED) {
    connect_wifi();
  }

  if (!client.connected()) {
    connectMQTT();
  }

  client.loop();
  timeClient.update();

  // Update heater status here
  // heaterStatus = ...;  // Retrieve the actual heater status

  // Update intervals based on heater status
  updateIntervals();
  
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
    String timestamp = timeClient.getFormattedTime();

    count = 0;
    lastTime_sensor = currentTime;
    attachInterrupt(digitalPinToInterrupt(PIN), countRisingEdges, RISING);
 

    // Create payload for all data

    // Humidity
    createPayload("paku/humidity/moko/cabin", -1000, timestamp);
    createPayload("paku/humidity/moko/dryer", -1000, timestamp);
    createPayload("paku/humidity/moko/kitchen", -1000, timestamp);
    createPayload("paku/humidity/moko/lounge", -1000, timestamp);

    // Temperatures
    createPayload("paku/temperature/moko/cabin", -1000, timestamp);
    createPayload("paku/temperature/moko/dryer", -1000, timestamp);
    createPayload("paku/temperature/moko/kitchen", -1000, timestamp);
    createPayload("paku/temperature/moko/lounge", -1000, timestamp);
    createPayload("paku/temperature/heating/floor", -1000, timestamp);
    createPayload("paku/temperature/heating/heater_in", -1000, timestamp);
    createPayload("paku/temperature/heating/heater_out", -1000, timestamp);
    createPayload("paku/temperature/heating/required_dt", requiredDeltaT, timestamp);
    
    // Flow
    createPayload("paku/flow/coolant_frequency", frequency, timestamp);
    createPayload("paku/flow/coolant", flowRate, timestamp);
    
    // Heating power
    createPayload("paku/power/heat", -1000, timestamp);
    createPayload("paku/power/cool", -1000, timestamp);
    
    // Battery voltage
    createPayload("paku/voltage/car", -1000, timestamp);
    createPayload("paku/voltage/leisure", -1000, timestamp);
    
    // Heater status
    createPayload("paku/status/heater", -1000, timestamp);
    createPayload("paku/status/heater_timer", -1000, timestamp);
    createPayload("paku/status/pump", -1000, timestamp);


 }

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
  
  while (WiFi.status() != WL_CONNECTED){
    for (int i = 0; i < sizeof(wifi_ssid)/sizeof(wifi_ssid[0]); i++) {
      Serial.print("Connecting to ");
      Serial.println(wifi_ssid[i]);
      WiFi.begin(wifi_ssid[i], wifi_password[i]);

      int attempts = 0;
      while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
      }

      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("");
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
        return;
      } else {
        Serial.println("");
        Serial.println("Failed to connect to ");
        Serial.println(wifi_ssid[i]);
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
    
    if (client.connect("ESP32Client")) {  // Use a unique client ID for ESP32
      Serial.println("connected");
      client.subscribe("paku/control");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
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
  if (heaterStatus == 1) {
    mqttInterval = mqttFastInterval;
    sensorInterval = sensorFastInterval;
  } else {
    mqttInterval = mqttSlowInterval;
    sensorInterval = sensorSlowInterval;
  }
}