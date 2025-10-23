# Paku Core — Requirements (ESP32)

**Scope:** Firmware that runs on the ESP32 device at the edge (historically called “CORE”, hereafter **EDGE device**).  
**Out of scope:** Cloud/backend services (see `paku-iot` repo).

---

## 1) Terminology & Identity
- **Repository name:** `paku-core`
- **Device name in code/docs:** **EDGE** (legacy references to CORE may remain in code during transition).
- **Device ID:** stable identifier derived from chip MAC (e.g., `paku-{MAC6}`). Exposed as `device_id`.
- **Firmware version:** Semantic Versioning `MAJOR.MINOR.PATCH` (e.g., `1.2.0`), compile-defined macro.

---

## 2) Functional Requirements

### 2.1 Wi-Fi
- Support **multiple SSIDs** with password fallbacks defined in `include/secrets.h` (git-ignored).
- Auto-retry with exponential backoff; reconnect if disconnected.
- Expose connection state and RSSI via telemetry.

### 2.2 MQTT (primary control/telemetry)
- Connect to configurable broker (host, port, optional TLS).
- Topics (prefix `devices/{device_id}`):
  - `state` — online/offline + boot reason; retain last known state.
  - `telemetry` — periodic metrics (see 2.3).
  - `cmd` — inbound commands (JSON).
  - `lwt` — **Last Will** set to `offline`; published by broker on unexpected disconnect.
- QoS: default **0**; allow override per message type.
- Reconnect on failure; backoff aligned with Wi-Fi.

### 2.3 Telemetry
- Publish every **60s** (configurable):
  - `uptime_s`, `rssi`, `heap_free`, `cpu_temp` (if available), `fw_version`, `device_id`.
  - Sensor readings when present (see 2.5).
- Provide a `telemetry/heartbeat` boolean or counter.

### 2.4 Logging
- Serial log at **115200** baud; log levels via compile-time macro (`LOG_LEVEL`).
- Optional: forward important events to `devices/{device_id}/logs` (rate-limited).

### 2.5 Sensors / Actuators (placeholder until hardware is fixed)
- Abstraction layer for sensors/actuators; non-blocking loop.
- Each sensor reports JSON payload and timestamp.
- Safe defaults when hardware absent (no crashes).

### 2.6 Configuration
- **Build-time**: `include/secrets.h` (private), `include/config.h` (public defaults).
- **Runtime** (future): optional JSON file or MQTT config messages.
- Documented in `docs/edge/config.md` and `docs/edge/quickstart.md`.

### 2.7 OTA (optional v1 / recommended v2)
- Support OTA updates (ArduinoOTA or HTTP/MQTT-triggered).
- Verify image integrity; reject downgrade unless `ALLOW_DOWNGRADE` set.

---

## 3) Non-Functional Requirements

### 3.1 Platform & Tooling
- **PlatformIO** (Arduino framework for ESP32).
- Builds via `pio run`; flash via `pio run -t upload`.
- Code compiles with current `platform-espressif32` LTS.

### 3.2 Performance & Reliability
- Boot to operational (Wi-Fi + MQTT connected) in **< 5s** under normal conditions.
- Main loop non-blocking; watchdog friendly.
- Handle network loss gracefully; no hard resets unless unrecoverable.

### 3.3 Resource Constraints
- Heap headroom **> 20 KB** under steady state (target boards: ESP32-WROOM32 class).
- Avoid dynamic allocation in hot paths; prefer static buffers.

### 3.4 Security
- No credentials in repo; `include/secrets.h` is **.gitignored**.
- Use MQTT TLS when broker supports it; store CA cert in firmware if enabled.
- Set MQTT LWT to `offline`.
- Pre-push secret scan with `trufflehog` recommended.

### 3.5 Observability
- Heartbeat telemetry and retained `state`.
- Unique boot counter; store boot reason.
- Optional LED blink codes for error states.

---

## 4) Testing & QA

### 4.1 Local
- Build succeeds: `pio run`.
- Lint/style (if configured) passes.
- Manual smoke test: Wi-Fi connect → MQTT connect → telemetry publishes.

### 4.2 Bench Acceptance Checklist
- Power cycle → device publishes `state=online` within 5s.
- Telemetry interval respected (±10%).
- Wi-Fi drop → reconnects with backoff; MQTT LWT flips to `offline` then back to `online`.
- Command on `cmd` topic is parsed and acknowledged (even if it’s a NOP).

---

## 5) Configuration Keys (reference)
- `WIFI_SSIDS[]`, `WIFI_PASSWORDS[]` — ordered preference.
- `MQTT_HOST`, `MQTT_PORT`, `MQTT_TLS`, `MQTT_USER`, `MQTT_PASS` (in `secrets.h`).
- `TELEMETRY_INTERVAL_SEC`, `LOG_LEVEL`, `TOPIC_PREFIX` (in `config.h`).

---

## 6) Versioning & Releases
- Tag format: `core-vMAJOR.MINOR.PATCH` (e.g., `core-v1.0.0`).
- Pre-releases: `-dev.N` on `dev` branch.
- Changelog entry required for each tag.

---

## 7) Roadmap (short)
- v1.0: stable Wi-Fi/MQTT/telemetry/logging; docs complete.
- v1.1: OTA enablement.
- v1.2: runtime config via MQTT.
