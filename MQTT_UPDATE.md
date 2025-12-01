# MQTT Topic Update for paku-core

## Changes Required

### 1. New Topic Structure
Change from: `paku/devices/{device_id}/telemetry/{type}/{location}`  
Change to: `{site_id}/{system}/{device_id}/data`

Example topics:
- `paku/sensors/ruuvi_cabin/data`
- `paku/flow/coolant/data`
- `paku/power/leisure_battery/data`

### 2. New Payload Structure

Instead of individual metrics per topic:
```json
{"value": 21.5, "timestamp": "12:34:56"}
```

Send all metrics per device in one message:
```json
{
  "timestamp": "2025-12-01T20:00:00Z",
  "device_id": "ruuvi_cabin",
  "location": "cabin",
  "metrics": {
    "temperature_c": 21.5,
    "humidity_percent": 45.2,
    "pressure_hpa": 1013.25,
    "battery_mv": 2870
  }
}
```

### 3. Required Code Changes in main.cpp

#### Change 1: Update createRuuviPayloads() function

**Old approach** (lines ~770-805):
```cpp
void createRuuviPayloads(const char* timestamp) {
  // Creates separate payload for each metric
  String tempTopic = String("paku/devices/") + deviceId + "/telemetry/temperature/" + tag->location;
  createPayload(tempTopic, tag->lastData.temperature, timestamp);
  
  String humidTopic = String("paku/devices/") + deviceId + "/telemetry/humidity/" + tag->location;
  createPayload(humidTopic, tag->lastData.humidity, timestamp);
}
```

**New approach**:
```cpp
void createRuuviPayloads(const char* timestamp) {
  const RuuviTag* freshTags[MAX_RUUVI_TAGS];
  uint8_t freshCount = getFreshTags(freshTags, MAX_RUUVI_TAGS, millis());
  
  for (uint8_t i = 0; i < freshCount; i++) {
    const RuuviTag* tag = freshTags[i];
    if (!tag->hasData || !tag->lastData.valid) continue;
    
    // Build metrics JSON object
    StaticJsonDocument<256> doc;
    doc["timestamp"] = String(timestamp);
    doc["device_id"] = String("ruuvi_") + tag->location;
    doc["location"] = tag->location;
    
    JsonObject metrics = doc.createNestedObject("metrics");
    metrics["temperature_c"] = tag->lastData.temperature;
    metrics["humidity_percent"] = tag->lastData.humidity;
    if (tag->lastData.pressure > 0) {
      metrics["pressure_hpa"] = tag->lastData.pressure / 100.0f;
    }
    if (tag->lastData.batteryVoltage > 0) {
      metrics["battery_mv"] = tag->lastData.batteryVoltage;
    }
    
    // Serialize and publish
    String payload;
    serializeJson(doc, payload);
    String topic = String("paku/sensors/ruuvi_") + tag->location + "/data";
    
    if (payloadIndex < 30) {
      payloads[payloadIndex].topic = topic;
      payloads[payloadIndex].data = payload;
      payloadIndex++;
    }
  }
}
```

#### Change 2: Update flow sensor payloads

**Old** (lines ~428-430):
```cpp
createPayload(String("paku/devices/") + deviceId + "/telemetry/temperature/required_dt", requiredDeltaT, timestamp);
createPayload(String("paku/devices/") + deviceId + "/telemetry/flow/coolant_frequency", frequency, timestamp);
createPayload(String("paku/devices/") + deviceId + "/telemetry/flow/coolant", flowRate, timestamp);
```

**New**:
```cpp
// Build flow sensor payload
StaticJsonDocument<256> doc;
doc["timestamp"] = String(timestamp);
doc["device_id"] = "coolant";
doc["location"] = "coolant_line";

JsonObject metrics = doc.createNestedObject("metrics");
metrics["flow_rate_lpm"] = flowRate;
metrics["frequency_hz"] = frequency;
metrics["required_dt_c"] = requiredDeltaT;

String payload;
serializeJson(doc, payload);
String topic = "paku/flow/coolant/data";

if (payloadIndex < 30) {
  payloads[payloadIndex].topic = topic;
  payloads[payloadIndex].data = payload;
  payloadIndex++;
}
```

#### Change 3: Remove old createPayload() function or update it

The old `createPayload(String topic, float value, String timestamp)` function is no longer needed with the new structure. Replace calls to it with the new pattern above.

### 4. Testing

After changes:
1. Deploy updated paku-core
2. Use MQTT Explorer to verify topics appear as `paku/sensors/ruuvi_cabin/data`
3. Check payload structure matches new schema
4. Verify collector inserts data into database
5. Check Grafana can query the new structure

### 5. Configuration

In secrets.h, define:
```cpp
#define SITE_ID "paku"  // Can be "paku", "car", "home", etc.
```

Use SITE_ID in topic construction:
```cpp
String topic = String(SITE_ID) + "/sensors/ruuvi_" + tag->location + "/data";
```
