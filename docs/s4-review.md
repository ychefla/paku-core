# S4-1 Code Review — Reusable Features Analysis

**Date:** 2025-11-28  
**Branch:** `s4/S4-1-review` (based on `dev`)  
**Purpose:** Identify reusable modules, refactor candidates, and out-of-scope features for the Sprint 4 work.

---

## Executive Summary

The current `paku-core` codebase is an ESP32-based IoT edge device firmware built with PlatformIO and Arduino framework. It targets the LilyGo T-Display S3 board and provides:
- Multi-network WiFi connectivity
- MQTT telemetry publishing
- TFT display output
- Bluetooth LE scanning
- Flow sensor monitoring with ISR-based pulse counting
- Deep sleep power management

The firmware is functional but monolithic (~613 lines in `main.cpp`). To support sprint goals (S4-3, S4-4), the code needs modularization and cleanup.

---

## 1. Modules/Components — Reusability Assessment

### 1.1 Reusable with Minimal Changes ✅

| Component | Location | Description | Sprint Relevance |
|-----------|----------|-------------|------------------|
| **WiFi Connection** | `main.cpp:414-460` | Multi-SSID fallback with retry logic | Core requirement |
| **Pin Configuration** | `src/pin_config.h` | Hardware abstraction for pins | Foundation |
| **Secrets Template** | `include/secrets.h.template` | Credential management pattern | Best practice |
| **NTP Time Sync** | `main.cpp:59, 245` | Time synchronization via NTPClient | Telemetry timestamps |
| **MQTT Client Setup** | `main.cpp:157-158, 471-494` | PubSubClient configuration | Core requirement |
| **Build Configuration** | `platformio.ini` | Board, platform, and library setup | Foundation |

**Rationale:** These components follow established patterns and have clear boundaries. They can be extracted into separate modules with minimal refactoring.

### 1.2 Modules Requiring Moderate Refactor 🔧

| Component | Location | Issue | Recommended Action |
|-----------|----------|-------|-------------------|
| **Display Initialization** | `main.cpp:117-150` | Mixed with setup; uses raw LCD commands | Extract to `display.cpp/h`; use TFT_eSPI properly |
| **Payload System** | `main.cpp:80-88, 550-557` | Fixed-size array (30), no overflow protection | Refactor to circular buffer or dynamic list |
| **Flow Sensor** | `main.cpp:310-368` | Hardcoded calibration, mixed concerns | Extract to `sensors/flow.cpp` with config |
| **Interval Management** | `main.cpp:63-78, 566-586` | Static state, heater-coupled logic | Decouple; make intervals configurable |
| **sendToMQTT** | `main.cpp:380-393` | Topic hardcoded, no error handling | Add topic prefix config per requirements.md |

**Refactor Priority:** Medium — These need cleanup before S4-3/S4-4 work to avoid technical debt accumulation.

### 1.3 Modules Likely to Be Removed or Replaced ❌

| Component | Location | Reason | Recommendation |
|-----------|----------|--------|----------------|
| **LCD_MODULE_CMD_1 commands** | `main.cpp:22-44` | ST7789V raw commands; TFT_eSPI handles this | Remove; rely on library init |
| **Hardcoded test mode** | `main.cpp:77, 318-320` | `testMode = true` simulates random data | Remove or make runtime configurable |
| **Duplicate loop docstrings** | `main.cpp:181-228` | Redundant documentation | Keep one, remove duplicate |
| **Version check errors** | `main.cpp:612-613` | Outdated ESP-IDF version constraint | Update or remove for compatibility |
| **Unused BLE globals** | `main.cpp:48` | `scanBT_enabled` never toggled runtime | Consider removing if not needed |

### 1.4 Third-Party Libraries Assessment

| Library | Status in `lib/` | Used | Action |
|---------|------------------|------|--------|
| TFT_eSPI | Local copy | ✅ Yes | Keep — display driver |
| lvgl + lv_conf.h | Local copy | ❌ No (lib_ignore) | Remove if not planned |
| TouchLib | Local copy | ❌ No (lib_ignore) | Remove if not planned |
| SensorLib | Local copy | ❌ No | Evaluate for sensor abstraction |
| arduino-nofrendo | Local copy | ❌ No (lib_ignore) | Remove — NES emulator irrelevant |
| OneButton | Local copy | ❌ No | Keep — useful for button handling |
| DabbleESP32 | Local copy | ❌ No (lib_ignore) | Remove if not needed |
| Adafruit_MPR121 | Local copy | ❌ No (lib_ignore) | Remove if not needed |
| PCA95x5, PCF8575 | Local copy | ❌ No (lib_ignore) | Remove if not needed |
| GFX Library | Local copy | ❌ No (lib_ignore) | Remove — redundant with TFT_eSPI |
| ESP32-audioI2S | Local copy | ❌ No | Remove if not planned |

**Recommendation:** Clean up unused libraries to reduce repository size and build complexity. Keep only TFT_eSPI, OneButton, and SensorLib (if sensors are planned).

---

## 2. Features Out of Current Sprint Scope — Preserve for Later

These features exist in the codebase but are not part of the immediate sprint work. They should be preserved and cataloged for future implementation.

| Feature | Location | Status | Preservation Notes |
|---------|----------|--------|-------------------|
| **Bluetooth LE Scanning** | `main.cpp:497-535` | Functional (FreeRTOS task) | Keep code; document for future proximity features |
| **Deep Sleep** | `main.cpp:253-270` | Partially implemented | Keep; essential for battery operation |
| **Heater Control Logic** | `main.cpp:569-586` | Stub (status hardcoded) | Keep structure; add actual control later |
| **Multiple Sensor Types** | `main.cpp:336-366` | Placeholders (`-1000` values) | Keep topic structure; implement sensors later |
| **Touch Display Support** | `lib/TouchLib` | Library present, not used | Keep for interactive UI later |
| **LVGL Graphics** | `lib/lvgl` | Library present, ignored | Keep for rich UI; low priority |
| **Battery Voltage Reading** | `pin_config.h:42` | Pin defined, not read | Keep; implement battery monitoring |
| **OTA Updates** | Not implemented | Mentioned in requirements.md | Plan for v1.1 per roadmap |

**Action:** Tag these features as `// TODO(future): <description>` comments to ensure they're not accidentally removed.

---

## 3. Risks and Technical Debt

### 3.1 High-Risk Issues

| Risk | Impact | Location | Mitigation |
|------|--------|----------|------------|
| **Monolithic main.cpp** | Hard to test, maintain | `src/main.cpp` | Split into modules (network, display, sensors, mqtt) |
| **Blocking WiFi/MQTT loops** | Watchdog timeout risk | `connect_wifi`, `connectMQTT` | Add timeout limits; use non-blocking patterns |
| **No error handling for MQTT publish** | Silent data loss | `main.cpp:387` | Check return value of `client.publish()` |
| **ESP-IDF version constraint** | Won't compile on newer SDK | `main.cpp:612-613` | Update or conditionally compile |
| **Hardcoded topic prefixes** | Violates requirements.md | `main.cpp:336-366` | Use `devices/{device_id}/` per spec |

### 3.2 Medium-Risk Issues

| Risk | Impact | Mitigation |
|------|--------|------------|
| Fixed payload buffer (30) | Overflow if too many sensors | Use dynamic allocation or larger buffer |
| No device_id implementation | Can't identify devices | Generate from MAC address per requirements.md |
| Missing telemetry heartbeat | No health monitoring | Add per requirements.md 2.3 |
| Serial-only logging | No remote visibility | Add optional MQTT logging per 2.4 |

### 3.3 Technical Debt

- Duplicate code between `tft.println()` for console and display
- Magic numbers throughout (e.g., `30`, `5000`, `120`)
- Inconsistent naming (`wifi_ssid` vs `mqtt_server`, `payloadIndex` vs `lastTime_sensor`)
- No unit tests in `test/` directory
- Large unused library footprint in `lib/`

---

## 4. Recommended Code Changes for Sprint Work

### 4.1 Prerequisites for S4-3 and S4-4

Based on the sprint requirements, the following changes are needed to enable subsequent tasks:

#### Priority 1 — Foundation (Required for S4-3)

1. **Create modular structure**
   ```
   src/
   ├── main.cpp          # Setup/loop only
   ├── network/
   │   ├── wifi.cpp/h    # WiFi connection logic
   │   └── mqtt.cpp/h    # MQTT client wrapper
   ├── display/
   │   └── display.cpp/h # TFT initialization and updates
   ├── sensors/
   │   └── flow.cpp/h    # Flow sensor handling
   └── config.h          # Configurable constants
   ```

2. **Implement device_id**
   ```cpp
   // In config.h or identity.cpp
   String getDeviceId() {
       uint8_t mac[6];
       WiFi.macAddress(mac);
       // Uses last 3 bytes of MAC for device ID (unique per device on same network).
       // For large deployments, consider using all 6 bytes or a UUID.
       return String("paku-") + String(mac[3], HEX) + String(mac[4], HEX) + String(mac[5], HEX);
   }
   ```

3. **Fix topic structure** — Change from `paku/humidity/...` to `devices/{device_id}/telemetry`

4. **Add config.h** for compile-time settings (intervals, log level, topic prefix)

#### Priority 2 — Cleanup (Recommended for S4-4)

5. **Remove unused libraries** from `lib/`
6. **Remove ESP-IDF version check** or update for compatibility
7. **Add basic error handling** to WiFi/MQTT connection loops
8. **Implement non-blocking reconnection** for network failures
9. **Add firmware version macro** per requirements.md

#### Priority 3 — Nice to Have

10. Clean up duplicate docstrings
11. Add `.clang-format` for consistent code style
12. Create stub unit tests in `test/`

---

## 5. Cleanup Checklist

### Files to Keep

- [x] `paku_core/src/main.cpp` — refactor, don't remove
- [x] `paku_core/src/pin_config.h` — keep as-is
- [x] `paku_core/src/img_logo.h` — keep for boot splash
- [x] `paku_core/include/secrets.h.template` — keep
- [x] `paku_core/platformio.ini` — keep, update lib_ignore
- [x] `paku_core/boards/lilygo-t-displays3.json` — keep
- [x] `paku_core/lib/TFT_eSPI/` — keep (display driver)
- [x] `paku_core/lib/OneButton/` — keep (button handler)
- [x] All `docs/` files — keep and update

### Files/Directories to Remove

- [ ] `paku_core/lib/arduino-nofrendo/` — NES emulator, not needed
- [ ] `paku_core/lib/GFX Library for Arduino/` — redundant
- [ ] `paku_core/lib/DabbleESP32/` — not used
- [ ] `paku_core/lib/Adafruit_MPR121/` — not used
- [ ] `paku_core/lib/PCA95x5/` — not used
- [ ] `paku_core/lib/PCF8575 library/` — not used
- [ ] `paku_core/lib/ESP32-audioI2S-3.0.6/` — not used
- [ ] `paku_core/lib/lvgl/` — not currently used (keep if UI planned)
- [ ] `paku_core/lib/lv_conf.h` — not currently used
- [ ] `todo` file at root — merge into issue tracker

### Files to Evaluate

- [ ] `paku_core/lib/SensorLib/` — evaluate for sensor abstraction
- [ ] `paku_core/lib/TouchLib/` — keep if touch UI planned

---

## 6. Next Steps (Prioritized)

| # | Task | Sprint | Effort |
|---|------|--------|--------|
| 1 | Create `config.h` with constants | S4-3 | Low |
| 2 | Implement `getDeviceId()` from MAC | S4-3 | Low |
| 3 | Fix MQTT topic structure | S4-3 | Medium |
| 4 | Extract WiFi to `network/wifi.cpp` | S4-3 | Medium |
| 5 | Extract MQTT to `network/mqtt.cpp` | S4-3 | Medium |
| 6 | Remove unused libraries | S4-4 | Low |
| 7 | Add error handling to network code | S4-4 | Medium |
| 8 | Extract display code | S4-4 | Medium |
| 9 | Add telemetry heartbeat | S4-4 | Low |
| 10 | Create unit test stubs | Future | Medium |

---

## 7. Appendix

### A. File Statistics

```
paku_core/src/main.cpp:     613 lines
paku_core/src/pin_config.h:  53 lines
Total project source:       ~666 lines (excluding libraries)
```

### B. External Dependencies (from platformio.ini — verified)

```ini
lib_deps = 
    knolleary/PubSubClient@^2.8
    bblanchon/ArduinoJson@^7.2.0
    arduino-libraries/NTPClient@^3.2.1
```

### C. Reference Documents

- `docs/requirements.md` — EDGE device requirements specification
- `docs/README for paku-core.md` — Project overview
- `docs/development-modes.md` — Container vs. local development
- `docs/edge/quickstart.md` — Build and flash instructions
- `docs/edge/config.md` — Configuration reference

---

*Review completed by automated analysis. Manual verification recommended before implementing changes.*
