# S4-1: Codebase Review and Feature Assessment

> Review document for the paku-core repository, identifying reusable components, required changes, and features to preserve for the S4 sprint.

## 1. Executive Summary

The paku-core repository contains ESP32-S3 firmware for the Paku IoT edge device. The codebase is functional but requires modifications to support Ruuvi tag data collection and paku-iot integration. Key findings:

- **Core infrastructure is solid**: WiFi, MQTT, BLE scanning, and display modules work together
- **BLE scanning exists** but needs enhancement for Ruuvi-specific data parsing
- **MQTT publishing works** but topic structure needs alignment with paku-iot
- **Many bundled libraries are unused** and can remain ignored during build
- **No existing tests** — test infrastructure needs to be established

---

## 2. Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                        paku-core (ESP32-S3)                     │
├─────────────────────────────────────────────────────────────────┤
│  main.cpp                                                       │
│  ├── setup()          - Initializes WiFi, MQTT, TFT, sensors    │
│  ├── loop()           - Main control loop                       │
│  ├── scanBT()         - FreeRTOS task for BLE scanning          │
│  ├── connect_wifi()   - Multi-SSID connection handler           │
│  ├── connectMQTT()    - MQTT broker connection                  │
│  ├── processData()    - Sensor data aggregation                 │
│  ├── sendToMQTT()     - Payload publishing                      │
│  └── updateDisplay()  - TFT status display                      │
├─────────────────────────────────────────────────────────────────┤
│  Configuration                                                  │
│  ├── secrets.h         - WiFi/MQTT credentials (git-ignored)    │
│  ├── pin_config.h      - Hardware pin assignments               │
│  └── platformio.ini    - Build configuration                    │
├─────────────────────────────────────────────────────────────────┤
│  Libraries (bundled in lib/)                                    │
│  ├── TFT_eSPI          - Display driver (ACTIVE)                │
│  ├── lvgl              - UI library (IGNORED in build)          │
│  ├── OneButton         - Button handling (available)            │
│  ├── SensorLib         - Sensor abstractions (available)        │
│  └── Others            - Various libraries (IGNORED in build)   │
└─────────────────────────────────────────────────────────────────┘
```

---

## 3. Component Analysis

### 3.1 Reusable with Minimal Changes

| Component | Location | Status | Notes |
|-----------|----------|--------|-------|
| WiFi connection | `main.cpp:connect_wifi()` | ✅ Ready | Multi-SSID fallback works well |
| MQTT client | `main.cpp:connectMQTT()` | ✅ Ready | Uses PubSubClient, stable |
| BLE scanning | `main.cpp:scanBT()` | ⚠️ Needs work | Generic scan, needs Ruuvi parsing |
| TFT display | `main.cpp:updateDisplay()` | ✅ Ready | Shows status, useful for debugging |
| Secrets template | `include/secrets.h.template` | ✅ Ready | Well-documented template |
| Board definition | `boards/lilygo-t-displays3.json` | ✅ Ready | Correct for T-Display S3 |
| Pin configuration | `src/pin_config.h` | ✅ Ready | Hardware-specific, complete |

### 3.2 Modules Requiring Moderate Refactor

| Component | Location | Required Changes |
|-----------|----------|------------------|
| `scanBT()` | `main.cpp:497-535` | Add Ruuvi advertisement parsing, extract temperature/humidity/pressure from manufacturer data |
| `createPayload()` | `main.cpp:550-557` | Update JSON structure to match paku-iot expected format |
| `processData()` | `main.cpp:310-368` | Integrate BLE scan results into payload creation |
| Topic structure | `main.cpp` | Change from `paku/{type}/{location}/{device}` to paku-iot compatible format |
| Interval configuration | `main.cpp:63-70` | Make configurable via config.h or runtime |

### 3.3 Modules to Remove or Replace

| Component | Reason | Action |
|-----------|--------|--------|
| Flow sensor code | Not relevant to Ruuvi | **Preserve** but disable (out of scope) |
| Heater control | Not in S4 scope | **Preserve** but disable (out of scope) |
| Hardware-specific LCD init | Legacy code | **Keep** — needed for display |

### 3.4 Out-of-Scope Features to Preserve

These features exist in the codebase and should **NOT be removed**. They represent future functionality:

| Feature | Location | Reason to Keep |
|---------|----------|----------------|
| Heater monitoring | `main.cpp:76-88, 310-368` | Future home automation integration |
| Flow sensor | `main.cpp:101-104, 310-330` | Future flow meter support |
| Temperature topics (floor, heater_in, heater_out) | `main.cpp:346-349` | Future sensor expansion |
| Power topics | `main.cpp:356-357` | Future power monitoring |
| Voltage monitoring | `main.cpp:360-361` | Future battery monitoring |
| Sleep mode | `main.cpp:253-270` | Power optimization, may be useful |
| Humidity topics | `main.cpp:336-339` | Future sensor expansion |
| lvgl library | `lib/lvgl/` | Future advanced UI |
| OneButton library | `lib/OneButton/` | Future button interactions |
| Audio library | `lib/ESP32-audioI2S-3.0.6/` | Future audio notifications |

---

## 4. Ruuvi Tag Integration Requirements

### 4.1 What Ruuvi Tags Broadcast

Ruuvi tags broadcast BLE advertisements containing sensor data in manufacturer-specific format (RAWv2 format):

```
Manufacturer ID: 0x0499 (Ruuvi Innovations)
Data Format: 5 (RAWv2)
Fields: temperature, humidity, pressure, acceleration (x,y,z), battery voltage, tx power, movement counter, measurement sequence
```

### 4.2 Required Code Changes

1. **Parse Ruuvi advertisements in `scanBT()`**:
   - Filter for manufacturer ID 0x0499
   - Decode RAWv2 data format
   - Extract: temperature, humidity, pressure, battery voltage

2. **Create Ruuvi-specific payloads**:
   - New function: `parseRuuviData(BLEAdvertisedDevice& device)`
   - New function: `createRuuviPayload(RuuviData& data)`

3. **Update MQTT topics** for paku-iot compatibility:
   - Current: `paku/temperature/moko/cabin`
   - Proposed: `devices/{device_id}/telemetry` or as specified by paku-iot

---

## 5. Build and Configuration Issues

### 5.1 Known Build Considerations

| Issue | Severity | Resolution |
|-------|----------|------------|
| `secrets.h` missing | Blocking | Copy from template (documented in README) |
| ESP-IDF v5 unsupported | Warning | Code has `#error` for IDF v5, use Arduino ESP32 < 3.0 |
| TFT library config | Warning | Requires `Setup206_LilyGo_T_Display_S3.h` selection |
| Platform version locked | Info | `espressif32@6.5.0` is explicit, good for reproducibility |

### 5.2 Dependencies (platformio.ini)

| Library | Version | Purpose | Status |
|---------|---------|---------|--------|
| PubSubClient | ^2.8 | MQTT client | ✅ Required |
| ArduinoJson | ^7.2.0 | JSON serialization | ✅ Required |
| NTPClient | ^3.2.1 | Time synchronization | ✅ Required |
| TFT_eSPI | bundled | Display driver | ✅ Required |

---

## 6. Risks and Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| BLE + WiFi coexistence | Medium | ESP32-S3 handles this well, but test carefully |
| Memory pressure | Medium | Limit BLE scan results, use static buffers |
| MQTT message loss | Low | Current QoS 0 is fine; consider QoS 1 for critical data |
| Display refresh blocking | Low | Current 1s update is acceptable |

---

## 7. Concrete Code Changes for S4-3 and S4-4

### S4-3: Fix Build and Run

1. **Document secrets setup** — already done in README
2. **Verify build** with `pio run`
3. **Add build CI workflow** — `.github/workflows/build.yml`
4. **Smoke test** — device boots, connects WiFi, publishes heartbeat

### S4-4: paku-iot Integration

1. **Add Ruuvi parser module**:
   ```cpp
   // src/ruuvi_parser.h / ruuvi_parser.cpp
   struct RuuviData {
     String mac;
     float temperature;
     float humidity;
     float pressure;
     float battery;
     int rssi;
   };
   bool parseRuuviRAWv2(uint8_t* data, size_t len, RuuviData& result);
   ```

2. **Update scanBT() to use parser**:
   ```cpp
   // In scanBT loop
   if (device.haveManufacturerData()) {
     std::string mfr = device.getManufacturerData();
     if (mfr.length() > 2 && mfr[0] == 0x99 && mfr[1] == 0x04) {
       RuuviData ruuvi;
       if (parseRuuviRAWv2((uint8_t*)mfr.data(), mfr.length(), ruuvi)) {
         // Create and queue payload
       }
     }
   }
   ```

3. **Create paku-iot compatible payload**:
   ```cpp
   // JSON format to match paku-iot expectations
   {
     "device_id": "ruuvi-{MAC}",
     "timestamp": "2024-01-01T12:00:00Z",
     "temperature": 21.5,
     "humidity": 45.0,
     "pressure": 1013.25,
     "battery": 2.95,
     "rssi": -65
   }
   ```

4. **Add configuration for paku-iot endpoint**:
   ```cpp
   // In secrets.h.template
   #define PAKU_IOT_MQTT_TOPIC_PREFIX  "paku-iot/ingest"
   ```

---

## 8. Test Strategy (for S4-5)

### Unit Tests
- Ruuvi RAWv2 parser (can be tested on host with mock data)
- JSON payload generation
- Configuration loading

### Integration Tests
- WiFi connection + MQTT publish (requires device)
- BLE scan + data extraction (requires Ruuvi tag)

### Mocking Strategy
- Use simulated BLE data for CI
- Mock MQTT broker for integration tests

---

## 9. Files Modified/Added Summary

| File | Action | Purpose |
|------|--------|---------|
| `docs/s4-review.md` | **Added** | This review document |
| `src/ruuvi_parser.h` | To Add (S4-4) | Ruuvi data structures |
| `src/ruuvi_parser.cpp` | To Add (S4-4) | Ruuvi parsing implementation |
| `src/main.cpp` | To Modify (S4-4) | Integration of Ruuvi parser |
| `include/config.h` | To Add (S4-4) | Configuration options |
| `.github/workflows/build.yml` | To Add (S4-3) | CI build verification |
| `docs/ARCHITECTURE.md` | To Add (S4-2) | Architecture documentation |
| `docs/INTEGRATION.md` | To Add (S4-2) | paku-iot integration docs |

---

## 10. Recommendations

1. **Start with S4-3** — ensure the project builds cleanly before adding features
2. **Create minimal Ruuvi parser** — don't over-engineer; parse only needed fields
3. **Preserve existing features** — use `#ifdef` guards rather than deletion
4. **Add CI early** — prevents regression during integration work
5. **Document as you go** — update README/docs with each change

---

## Appendix: Reference Links

- [Ruuvi RAWv2 Data Format](https://docs.ruuvi.com/communication/bluetooth-advertisements/data-format-5-rawv2)
- [paku-iot Repository](https://github.com/ychefla/paku-iot)
- [ESP32 BLE Arduino Documentation](https://github.com/espressif/arduino-esp32/tree/master/libraries/BLE)
- [PlatformIO ESP32 Documentation](https://docs.platformio.org/en/latest/platforms/espressif32.html)
