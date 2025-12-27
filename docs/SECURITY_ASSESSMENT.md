# Security Assessment — paku-core (EDGE Firmware)

**Date:** 2025-12-27  
**Phase:** Phase 1 - Security Review & Threat Assessment  
**Related Issue:** #46 (sub-issue of #31 - Securing System)  
**Status:** DRAFT  
**Version:** 1.0.0

---

## Executive Summary

This document provides a comprehensive security assessment of **paku-core**, the ESP32/ESP8266-based edge device firmware for the Paku IoT monitoring system. This assessment identifies sensitive data flows, credential handling practices, and security gaps in the current implementation.

### Key Findings

**Severity Levels:**
- 🔴 **CRITICAL**: Immediate action required - high risk of compromise
- 🟠 **HIGH**: Should be addressed soon - significant security concern
- 🟡 **MEDIUM**: Recommended improvement - moderate risk
- 🟢 **LOW**: Best practice enhancement - minimal risk

#### Critical Issues Found
1. 🔴 **No MQTT Authentication** - MQTT connections use no username/password
2. 🔴 **No Transport Encryption** - All MQTT traffic sent in plaintext over port 1883
3. 🔴 **WiFi Credentials Stored in Plain Text** - Passwords in NVS/EEPROM without encryption

#### High-Risk Issues
4. 🟠 **OTA Updates via HTTP Only** - No HTTPS support, vulnerable to MITM attacks
5. 🟠 **No OTA Signature Verification** - Only optional SHA256 checksum (can be bypassed)
6. 🟠 **Serial Logging of SSIDs** - Network names logged, potential information disclosure

#### Medium-Risk Issues
7. 🟡 **No Input Validation on MQTT Commands** - Command injection possible
8. 🟡 **No Rate Limiting** - Vulnerable to DoS via MQTT command flooding
9. 🟡 **No Device Authentication** - Any device can publish to broker if accessible

---

## 1. Current Security State

### 1.1 Authentication & Authorization

#### MQTT Broker Connection
**Location:** `paku_core/src/main.cpp:1260`

```cpp
if (client.connect(deviceId)) {  // Use unique device ID as MQTT client ID
```

**Issues:**
- 🔴 **No authentication**: `PubSubClient::connect()` called without username/password
- 🔴 **No authorization**: Any device can connect if broker is accessible
- 🟡 **Device ID only used as client ID**: Not validated against server-side registry

**Impact:** 
- Unauthorized devices can connect to MQTT broker
- Cannot distinguish legitimate vs rogue devices
- No way to revoke device access without network-level blocking

**Recommendation:**
- Add MQTT username/password authentication
- Implement device certificates (X.509) for mutual TLS
- Add server-side device allowlist/denylist

#### API Key for paku-iot HTTP Client
**Location:** `paku_core/include/secrets.h.template:45`

```cpp
#define PAKU_IOT_API_KEY              "your-api-key"
```

**Issues:**
- 🟡 **API key in header file**: Stored in compile-time constant
- 🟢 **Template properly excludes from git**: `.gitignore` covers secrets.h

**Status:** ✅ Acceptable for current use case (HTTP client is optional feature)

### 1.2 Data Encryption

#### MQTT Transport Security
**Location:** `paku_core/include/secrets.h.template:38`

```cpp
#define MQTT_PORT                     1883
```

**Issues:**
- 🔴 **Plaintext MQTT**: Port 1883 is unencrypted MQTT
- 🔴 **No TLS/SSL**: `PubSubClient` not configured with WiFiClientSecure
- 🔴 **Sensitive data in transit**: Sensor readings, device status, configuration sent unencrypted

**Impact:**
- Network eavesdropping can capture all telemetry data
- Man-in-the-middle attacks can modify commands
- WiFi packet sniffing reveals device behavior patterns

**Recommendation:**
- Switch to MQTTS (port 8883) with TLS 1.2 or higher
- Use WiFiClientSecure with server certificate verification
- Consider ESP32 hardware acceleration for TLS

#### WiFi Password Storage
**Location:** `paku_core/src/wifi_manager.cpp:66-67`

```cpp
strncpy(nvsNetworks[i].password, password.c_str(), MAX_WIFI_PASSWORD_LEN);
nvsNetworks[i].password[MAX_WIFI_PASSWORD_LEN] = '\0';
```

**Issues:**
- 🔴 **Plaintext storage**: WiFi passwords stored unencrypted in NVS (ESP32) or EEPROM (ESP8266)
- 🟠 **Readable via flash dump**: Anyone with physical access can extract passwords
- 🟡 **No secure enclave**: ESP32 supports flash encryption, but not enabled

**Impact:**
- Physical access to device reveals all WiFi credentials
- Stolen/discarded devices expose network passwords
- Flash memory dump tools can extract secrets

**Recommendation:**
- Enable ESP32 flash encryption feature
- Use ESP32 NVS encryption
- Consider storing only password hashes (requires WPA2-Enterprise)
- Implement secure boot to prevent unauthorized firmware

### 1.3 OTA (Over-The-Air) Updates

#### Firmware Download Security
**Location:** `paku_core/lib/OtaClient/src/OtaClient.cpp:137-169`

**Current Implementation:**
- ✅ SHA256 checksum verification (optional)
- ✅ Atomic updates with rollback support (ESP32)
- ❌ No HTTPS enforcement
- ❌ No signature verification
- ❌ No version validation

**Issues:**
- 🟠 **HTTP Downloads**: OTA firmware fetched via HTTP, not HTTPS
  ```cpp
  // No certificate verification in _http.begin()
  ```
- 🟠 **Optional Checksum**: Verification only if `expectedChecksum` provided
  ```cpp
  if (_config.expectedChecksum && strlen(_config.expectedChecksum) > 0) {
      setState(OtaState::VERIFYING, "Verifying firmware");
  ```
- 🔴 **No Signature Verification**: Anyone can generate valid SHA256, no PKI
- 🟡 **No Firmware Signing**: Unsigned binaries accepted

**Attack Scenarios:**
1. **MITM Attack**: Attacker intercepts OTA URL request, serves malicious firmware
2. **Compromised Update Server**: If server breached, all devices affected
3. **Replay Attack**: Old vulnerable firmware can be re-deployed

**Recommendation:**
- Enforce HTTPS for OTA downloads with certificate pinning
- Implement RSA/ECDSA signature verification
- Add firmware version checking (prevent downgrade attacks)
- Use ESP32 secure boot to validate firmware on boot

#### OTA Command Injection
**Location:** `paku_core/src/main.cpp:1406-1450` (handleMqttMessage)

```cpp
const char* url = doc["url"];
const char* checksum = doc["checksum"];
// ... no validation of URL format or domain
```

**Issues:**
- 🟡 **No URL validation**: Accepts any URL, including malicious domains
- 🟡 **No allowlist**: No restriction on permitted update servers
- 🟢 **JSON parsing**: Uses ArduinoJson, which has bounds checking

**Recommendation:**
- Validate OTA URL against allowlist of trusted domains
- Enforce HTTPS URLs only
- Add rate limiting for OTA commands

### 1.4 Credential Management

#### Secrets Template System
**Location:** `paku_core/include/secrets.h.template`

**Good Practices:** ✅
- Template file committed, actual secrets file git-ignored
- Clear documentation in template
- Comprehensive .gitignore rules

**Issues:**
- 🟡 **No runtime secret injection**: Secrets must be compiled into firmware
- 🟡 **No secret rotation**: Changing WiFi/MQTT credentials requires reflash
- 🟢 **MQTT credentials can be updated remotely**: Via config topic (good)

**Status:** ✅ Acceptable for single-user/private deployment

#### Hardcoded Defaults
**Search Results:** No hardcoded credentials found in source code ✅

### 1.5 Input Validation & Command Handling

#### MQTT Command Processing
**Location:** `paku_core/src/main.cpp:2690-2950` (handleMqttMessage)

**Issues:**
- 🟡 **Limited input validation**: JSON parsed but values not range-checked
- 🟡 **No command authentication**: Any MQTT client can send commands
- 🟡 **No rate limiting**: Command flooding possible
- 🟢 **Uses ArduinoJson**: Memory-safe JSON parsing library

**Example - Config Update:**
```cpp
if (doc["timing"]["wake_interval_s"].isNull()) {
  uint32_t newValue = doc["timing"]["wake_interval_s"];
  // No bounds checking! Could set to 0 or UINT32_MAX
  deviceConfig.timing.wake_interval_s = newValue;
}
```

**Attack Scenarios:**
- Set wake_interval_s to 0 → device never sleeps, drains battery
- Set wifi_connect_timeout_s to UINT32_MAX → device hangs indefinitely
- Add 50+ WiFi networks → exhaust NVS storage

**Recommendation:**
- Add bounds checking for all configuration values
- Implement command signing/MAC for critical operations
- Add rate limiting per device ID
- Validate all string inputs for length and content

#### WiFi Network Addition via MQTT
**Location:** `paku_core/src/main.cpp:2946-2952`

```cpp
const char* ssid = doc["ssid"];
const char* password = doc["password"];
bool success = wifiManager.addNetwork(ssid, password ? password : "");
```

**Issues:**
- 🟠 **Remote credential injection**: Anyone with MQTT access can add WiFi networks
- 🟡 **No validation**: SSID/password not checked for validity
- 🟡 **Information disclosure**: Success/failure reported, enables SSID enumeration

**Recommendation:**
- Require authentication token for credential operations
- Add maximum number of stored networks (currently 10, good)
- Sanitize SSID/password input

---

## 2. Sensitive Data Flows

### 2.1 Data Classification

| Data Type | Sensitivity | Storage | Transit | Access |
|-----------|-------------|---------|---------|--------|
| **WiFi Credentials** | 🔴 Critical | NVS/EEPROM plaintext | MQTT commands (plaintext) | Network stack |
| **MQTT Broker Address** | 🟡 Medium | Secrets.h (compile-time) | N/A | Config topic |
| **API Keys** | 🟠 High | Secrets.h (compile-time) | HTTPS headers | paku-iot client |
| **Device ID** | 🟢 Low | Generated from MAC | MQTT topics | Public |
| **Sensor Telemetry** | 🟢 Low | RAM buffer | MQTT (plaintext) | MQTT subscribers |
| **Device Configuration** | 🟡 Medium | NVS/EEPROM plaintext | MQTT config topic | MQTT subscribers |
| **Firmware Binaries** | 🟠 High | Flash memory | HTTP (plaintext) | OTA system |

### 2.2 Data Flow Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│ ESP32/ESP8266 Device (paku-core)                                │
│                                                                   │
│  ┌─────────────┐         ┌──────────────┐                       │
│  │  Secrets.h  │────────▶│   WiFi Stack │                       │
│  │ (plaintext) │         │  (plaintext) │                       │
│  └─────────────┘         └──────┬───────┘                       │
│                                  │                               │
│  ┌─────────────┐         ┌──────▼───────┐    ┌──────────────┐  │
│  │ NVS/EEPROM  │◀────────┤ WiFi Manager │    │ Sensor Data  │  │
│  │ (plaintext) │         │              │    │ (RAM buffer) │  │
│  └─────────────┘         └──────┬───────┘    └──────┬───────┘  │
│                                  │                    │          │
│                          ┌───────▼────────────────────▼───────┐ │
│                          │    MQTT Client (PubSubClient)      │ │
│                          │         NO TLS/Authentication      │ │
│                          └───────┬───────────────────┬────────┘ │
└──────────────────────────────────┼───────────────────┼──────────┘
                                   │                   │
                        Plaintext  │                   │  Plaintext
                        Port 1883  │                   │  Commands
                                   ▼                   ▼
                          ┌────────────────────────────────┐
                          │    MQTT Broker (mosquitto)      │
                          │   NO AUTH / NO TLS (port 1883) │
                          └────────────────────────────────┘
                                   │
                                   │ Plaintext subscription
                                   ▼
                          ┌────────────────────────────────┐
                          │  paku-iot Backend Services     │
                          │  (Grafana, PostgreSQL, etc.)   │
                          └────────────────────────────────┘
```

**Key Vulnerabilities:**
- 🔴 **No encryption layer**: All data flows in plaintext
- 🔴 **No authentication boundaries**: Any MQTT client can publish/subscribe
- 🔴 **Credential storage unencrypted**: Easy to extract from device

---

## 3. Attack Surface Analysis

### 3.1 Network Attack Vectors

#### 3.1.1 WiFi Layer
- **Evil Twin Attack**: Fake access point with known SSID captures credentials
  - Mitigation: ❌ None (WPA2 PSK assumed secure)
- **Deauthentication Attack**: Force device to reconnect, capture handshake
  - Mitigation: ❌ None
- **Packet Sniffing**: Capture WiFi traffic to analyze patterns
  - Mitigation: ❌ None (MQTT plaintext visible)

#### 3.1.2 MQTT Layer
- **MQTT Topic Injection**: Publish to device-specific topics
  - Mitigation: ❌ No ACLs on broker
- **Command Replay**: Re-send captured MQTT commands
  - Mitigation: ❌ No nonce/timestamp validation
- **Message Tampering**: Modify MQTT messages in transit
  - Mitigation: ❌ No message signing or encryption

#### 3.1.3 HTTP Layer (OTA)
- **Man-in-the-Middle**: Intercept OTA firmware download
  - Mitigation: ⚠️ Optional SHA256 checksum (weak)
- **DNS Spoofing**: Redirect OTA URL to malicious server
  - Mitigation: ❌ None
- **Downgrade Attack**: Force installation of old vulnerable firmware
  - Mitigation: ❌ No version checking

### 3.2 Physical Attack Vectors

#### 3.2.1 Device Access
- **Flash Memory Read**: Extract firmware via USB/JTAG
  - Mitigation: ❌ Flash encryption not enabled
- **Serial Console Access**: Read debug logs via UART
  - Mitigation: ⚠️ SSIDs logged (line 1156, 1223)
- **Hardware Tampering**: Replace firmware, add malicious components
  - Mitigation: ❌ No secure boot

#### 3.2.2 Credential Extraction
- **NVS Dump**: Read stored WiFi passwords from flash
  - Mitigation: ❌ Plaintext storage
- **Firmware Reverse Engineering**: Decompile binary to find secrets
  - Mitigation: ✅ Secrets not compiled in (template pattern)

### 3.3 Logical Attack Vectors

#### 3.3.1 Configuration Manipulation
- **Parameter Injection**: Send malicious config values
  - Example: `wake_interval_s: 0` → battery drain
  - Mitigation: ❌ No bounds checking
- **Storage Exhaustion**: Add maximum WiFi networks
  - Mitigation: ✅ Limited to MAX_NVS_WIFI_NETWORKS (10)

#### 3.3.2 Denial of Service
- **Command Flooding**: Send high-rate MQTT commands
  - Mitigation: ❌ No rate limiting
- **Watchdog Trigger**: Send invalid config causing hang
  - Mitigation: ⚠️ Watchdog present but may not catch all cases

---

## 4. Security Gaps Summary

### 4.1 Critical Gaps (Fix Immediately)

| Gap | Affected Component | Risk | Effort to Fix |
|-----|-------------------|------|---------------|
| No MQTT TLS | MQTT Client | Data interception, MITM | Medium |
| No MQTT Auth | MQTT Client | Unauthorized access | Low |
| WiFi Plaintext Storage | WiFi Manager | Credential theft | High (requires flash encryption) |
| No OTA HTTPS | OTA Client | Malicious firmware | Medium |
| No OTA Signatures | OTA Client | Malicious firmware | High (requires PKI) |

### 4.2 High-Priority Gaps (Fix Soon)

| Gap | Affected Component | Risk | Effort to Fix |
|-----|-------------------|------|---------------|
| No input validation | MQTT command handler | Config manipulation | Low |
| No rate limiting | MQTT commands | DoS attacks | Medium |
| Serial logging SSIDs | Debug output | Info disclosure | Low |
| No firmware signing | OTA system | Supply chain attack | High |

### 4.3 Medium-Priority Gaps (Recommended)

| Gap | Affected Component | Risk | Effort to Fix |
|-----|-------------------|------|---------------|
| No certificate pinning | HTTP/MQTT TLS | MITM with rogue CA | Medium |
| No device attestation | Device identity | Rogue devices | High |
| No secure boot | Boot process | Firmware tampering | Medium |
| No command authentication | MQTT | Unauthorized commands | Medium |

### 4.4 Low-Priority Enhancements (Nice to Have)

| Gap | Affected Component | Risk | Effort to Fix |
|-----|-------------------|------|---------------|
| No encrypted logs | Serial output | Info disclosure | Low |
| No audit logging | All operations | Forensics | Medium |
| No anomaly detection | Telemetry | Compromised devices | High |

---

## 5. Compliance & Standards

### 5.1 IoT Security Best Practices

**OWASP IoT Top 10 Compliance:**

| Vulnerability | Status | Compliance |
|---------------|--------|------------|
| I1: Weak, Guessable, or Hardcoded Passwords | ⚠️ WiFi passwords user-provided | Partial |
| I2: Insecure Network Services | ❌ MQTT plaintext | Non-compliant |
| I3: Insecure Ecosystem Interfaces | ⚠️ MQTT broker open | Partial |
| I4: Lack of Secure Update Mechanism | ❌ No OTA signatures | Non-compliant |
| I5: Use of Insecure or Outdated Components | ✅ Recent libraries | Compliant |
| I6: Insufficient Privacy Protection | ❌ Plaintext telemetry | Non-compliant |
| I7: Insecure Data Transfer and Storage | ❌ No encryption | Non-compliant |
| I8: Lack of Device Management | ⚠️ Basic management | Partial |
| I9: Insecure Default Settings | ✅ No defaults | Compliant |
| I10: Lack of Physical Hardening | ❌ No secure boot | Non-compliant |

**Overall Compliance:** 2/10 Compliant, 3/10 Partial, 5/10 Non-compliant

### 5.2 Regulatory Considerations

**Note:** Paku is stated as "not for commercial use" but security is still important for:
- Personal data protection (indoor environment data)
- Network security (device on home network)
- Privacy (activity patterns visible in telemetry)

---

## 6. Risk Assessment Matrix

| Threat | Likelihood | Impact | Risk Level | Priority |
|--------|-----------|--------|------------|----------|
| MQTT eavesdropping | High | Medium | 🔴 High | P1 |
| WiFi credential theft | Medium | High | 🔴 High | P1 |
| Malicious OTA firmware | Low | Critical | 🟠 High | P2 |
| Unauthorized MQTT commands | Medium | Medium | 🟡 Medium | P2 |
| Physical flash extraction | Low | High | 🟡 Medium | P3 |
| Config parameter injection | Medium | Low | 🟢 Low | P3 |
| DoS via command flooding | Low | Low | 🟢 Low | P3 |

**Priority Levels:**
- **P1**: Address in Phase 2 planning
- **P2**: Address in Phase 3 implementation
- **P3**: Address if time/resources permit

---

## 7. Recommendations

### 7.1 Quick Wins (Low Effort, High Impact)

1. **Enable MQTT Username/Password** (1-2 hours)
   - Modify `client.connect()` to include credentials
   - Store MQTT credentials in secrets.h
   - Configure mosquitto broker with authentication

2. **Add Input Validation** (2-3 hours)
   - Add bounds checking for all config values
   - Validate SSID/password lengths
   - Sanitize string inputs

3. **Remove Serial Logging of SSIDs** (30 minutes)
   - Replace SSID logging with generic "Attempting network X"
   - Keep error messages but remove identifying info

### 7.2 Medium-Term Improvements (Phase 3)

4. **Implement MQTTS (TLS)** (1-2 days)
   - Switch to port 8883
   - Use WiFiClientSecure
   - Generate and install server certificate
   - Enable certificate verification

5. **Add OTA HTTPS** (1 day)
   - Enforce HTTPS URLs for OTA
   - Implement certificate verification
   - Add domain allowlist

6. **Enable ESP32 Flash Encryption** (2-3 days)
   - Enable flash encryption in menuconfig
   - Test with encrypted flash
   - Document setup for new devices

### 7.3 Long-Term Security Hardening (Phase 4+)

7. **Implement OTA Signature Verification** (3-5 days)
   - Generate signing keys (RSA-2048 or ECDSA-256)
   - Sign firmware during build
   - Verify signature before install
   - Store public key in secure partition

8. **Enable ESP32 Secure Boot** (3-5 days)
   - Generate secure boot keys
   - Enable secure boot in bootloader
   - Test boot flow
   - Document key management

9. **Add Command Authentication** (2-3 days)
   - Implement HMAC-SHA256 for critical commands
   - Add shared secret management
   - Validate signature on each command

### 7.4 Infrastructure Recommendations (paku-iot)

10. **Configure MQTT ACLs** (1 day)
    - Define per-device topic permissions
    - Restrict subscribe/publish access
    - Enable ACL enforcement in mosquitto

11. **Setup TLS Certificates** (1-2 days)
    - Generate CA certificate
    - Issue per-device certificates (optional)
    - Configure broker for TLS

---

## 8. Conclusion

### Current Security Posture

paku-core implements a **functional but security-immature** IoT edge device firmware. While the code quality is good and the architecture is sound, **critical security controls are missing**:

- ❌ No encryption for data in transit (MQTT, OTA)
- ❌ No authentication for device or commands
- ❌ No protection for credentials at rest
- ⚠️ Limited protection against physical attacks
- ⚠️ Minimal input validation

### Risk Profile

For a **personal/home use** deployment with trusted network access:
- **Current Risk**: 🟡 **MEDIUM** 
- **Acceptable for:** Private home network with WPA2, trusted users only
- **Not acceptable for:** Any multi-user or exposed deployment

### Path Forward

Addressing the **5 critical gaps** identified in Section 4.1 will significantly improve security posture:

1. Enable MQTT TLS → Protects data in transit
2. Enable MQTT authentication → Controls device access
3. Enable flash encryption → Protects stored credentials
4. Add OTA HTTPS → Prevents firmware tampering
5. Add OTA signatures → Ensures firmware authenticity

**Estimated Effort:** 1-2 weeks of focused development for critical fixes

---

## 9. Appendix

### A. Tools Used for Analysis

- Manual code review of paku_core/src/ and paku_core/lib/
- grep searches for sensitive patterns (passwords, keys, credentials)
- Review of configuration files and templates
- Analysis of data flow through MQTT and WiFi stacks

### B. References

- [OWASP IoT Top 10](https://owasp.org/www-project-internet-of-things/)
- [ESP32 Security Features](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/security/index.html)
- [MQTT Security Fundamentals](https://www.hivemq.com/blog/mqtt-security-fundamentals/)
- [Secure Boot and Flash Encryption (ESP32)](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/security/secure-boot-v2.html)

### C. Related Documents

- `docs/requirements.md` - Functional requirements (not security-focused)
- `paku_core/platformio.ini` - Build configuration
- `paku_core/include/secrets.h.template` - Credential template
- Issue #31 - Parent issue for security hardening effort
- Issue #46 - This security assessment (Phase 1)

---

**Document Status:** Draft for Review  
**Next Phase:** Phase 2 - Plan and design actionable security improvements  
**Review By:** Project maintainer (@ychefla)  
**Last Updated:** 2025-12-27
