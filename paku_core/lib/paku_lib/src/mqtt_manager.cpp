/**
 * @file mqtt_manager.cpp
 * @brief MQTT broker failover manager implementation.
 *
 * Key design: owns both a WiFiClient (plain TCP) and WiFiClientSecure (TLS).
 * PubSubClient is re-pointed to the correct transport via setClient() each
 * time the active broker changes, so a plain-TCP local broker and a TLS
 * cloud broker coexist without the caller managing two client objects.
 *
 * @see mqtt_manager.h for architecture overview.
 */

#include "mqtt_manager.h"

// Logging macros — fall back to Serial.printf if logging.h is not available
// (mqtt_manager lives in paku_lib which may not access project-level includes)
#if __has_include("logging.h")
  #include "logging.h"
#else
  #define LOG_INFO(cat, fmt, ...)      Serial.printf("[INFO] [%s] " fmt "\n", cat, ##__VA_ARGS__)
  #define LOG_DEBUG_MQTT(fmt, ...)     do {} while(0)
#endif

#ifndef ESP8266
#include <ESPmDNS.h>
#endif

// ============================================================================
//  Public API
// ============================================================================

void MqttManager::begin(PubSubClient& client,
                        const char* deviceId,
                        const MqttBrokerConfig& primary,
                        const MqttBrokerConfig& fallback,
                        OnConnectCallback onConnect) {
    _client     = &client;
    _deviceId   = deviceId;
    _primary    = primary;
    _fallback   = fallback;
    _onConnect  = onConnect;

    _activeBroker      = MqttBroker::PRIMARY_LOCAL;
    _failCount         = 0;
    _lastPrimaryRetry  = 0;
    _lastConnectAttempt = 0;
    _initialProbed     = false;

    // Apply initial broker config (sets server + transport client)
    applyBrokerConfig(_primary);
}

bool MqttManager::maintain() {
    if (!_client) return false;

    // --- Initial probe: decide which broker to target on first call ---
    if (!_initialProbed) {
        _initialProbed = true;
        LOG_INFO("MqttMgr", "Initial probe of local broker %s:%d ...",
                 _primary.host, _primary.port);

        if (isLocalReachable()) {
            LOG_INFO("MqttMgr", "Local broker reachable — using PRIMARY");
            _activeBroker = MqttBroker::PRIMARY_LOCAL;
            applyBrokerConfig(_primary);
        } else {
            LOG_INFO("MqttMgr", "Local broker unreachable — using CLOUD fallback");
            _activeBroker = MqttBroker::FALLBACK_CLOUD;
            applyBrokerConfig(_fallback);
        }
    }

    // --- Already connected ---
    if (_client->connected()) {
        // If on fallback, periodically try to return to primary
        if (isOnFallback() &&
            (millis() - _lastPrimaryRetry >= PRIMARY_RETRY_INTERVAL_MS)) {
            _lastPrimaryRetry = millis();

            LOG_INFO("MqttMgr", "Probing primary broker for return...");
            if (isLocalReachable()) {
                LOG_INFO("MqttMgr", "Primary reachable — switching back from cloud");
                _client->disconnect();
                _activeBroker = MqttBroker::PRIMARY_LOCAL;
                applyBrokerConfig(_primary);
                _failCount = 0;
                return false;  // Will reconnect on next maintain()
            } else {
                LOG_DEBUG_MQTT("Primary still unreachable, staying on cloud");
            }
        }
        return true;
    }

    // --- Not connected — attempt connection (throttled) ---
    return tryConnectOnce();
}

bool MqttManager::tryConnectOnce() {
    if (!_client) return false;

    // Throttle: don't retry faster than CONNECT_RETRY_DELAY_MS
    unsigned long now = millis();
    if (_lastConnectAttempt != 0 &&
        (now - _lastConnectAttempt) < CONNECT_RETRY_DELAY_MS) {
        return false;
    }
    _lastConnectAttempt = now;

    const MqttBrokerConfig& cfg =
        (_activeBroker == MqttBroker::PRIMARY_LOCAL) ? _primary : _fallback;

    // Skip if broker host is empty/unconfigured
    if (!cfg.host || strlen(cfg.host) == 0) {
        LOG_INFO("MqttMgr", "%s broker not configured — skipping",
                 activeBrokerName());
        _failCount = MAX_CONNECT_FAILURES;  // Force switch
        switchBroker();
        return false;
    }

    applyBrokerConfig(cfg);

    LOG_INFO("MqttMgr", "Connecting to %s broker %s:%d ...",
             activeBrokerName(), cfg.host, cfg.port);

    // Build LWT topic: paku/edge/{deviceId}/status
    String lwtTopic = String("paku/edge/") + _deviceId + "/status";

    bool connected;
    if (cfg.user && strlen(cfg.user) > 0) {
        connected = _client->connect(
            _deviceId,
            cfg.user, cfg.pass,
            lwtTopic.c_str(), 1, true,
            "{\"online\":false}"
        );
    } else {
        connected = _client->connect(
            _deviceId,
            nullptr, nullptr,
            lwtTopic.c_str(), 1, true,
            "{\"online\":false}"
        );
    }

    if (connected) {
        _failCount = 0;
        LOG_INFO("MqttMgr", "Connected to %s broker %s:%d",
                 activeBrokerName(), cfg.host, cfg.port);

        if (_onConnect) {
            _onConnect(*_client, _activeBroker);
        }
        return true;
    }

    _failCount++;
    LOG_INFO("MqttMgr", "Connect failed (rc=%d, attempt %d/%d) on %s",
             _client->state(), _failCount, MAX_CONNECT_FAILURES, cfg.host);

    if (_failCount >= MAX_CONNECT_FAILURES) {
        switchBroker();
    }

    return false;
}

void MqttManager::disconnect() {
    if (_client && _client->connected()) {
        _client->disconnect();
        LOG_INFO("MqttMgr", "Disconnected from %s broker", activeBrokerName());
    }
}

const char* MqttManager::activeBrokerName() const {
    return (_activeBroker == MqttBroker::PRIMARY_LOCAL) ? "LOCAL" : "CLOUD";
}

const char* MqttManager::activeHost() const {
    return (_activeBroker == MqttBroker::PRIMARY_LOCAL)
        ? _primary.host : _fallback.host;
}

uint16_t MqttManager::activePort() const {
    return (_activeBroker == MqttBroker::PRIMARY_LOCAL)
        ? _primary.port : _fallback.port;
}

// ============================================================================
//  Broker Probing & Resolution
// ============================================================================

bool MqttManager::isLocalReachable() {
    WiFiClient probe;
    LOG_DEBUG_MQTT("TCP probe %s:%d (timeout %d ms)...",
                   _primary.host, _primary.port, LOCAL_PROBE_TIMEOUT_MS);

    IPAddress resolved = resolveLocalBroker();
    bool ok = probe.connect(resolved, _primary.port, LOCAL_PROBE_TIMEOUT_MS);
    probe.stop();

    LOG_DEBUG_MQTT("Probe result: %s", ok ? "REACHABLE" : "UNREACHABLE");
    return ok;
}

IPAddress MqttManager::resolveLocalBroker() {
#ifndef ESP8266
    String host(_primary.host);
    if (host.endsWith(".local")) {
        String mdnsName = host.substring(0, host.length() - 6);
        IPAddress resolved = MDNS.queryHost(mdnsName.c_str(), LOCAL_PROBE_TIMEOUT_MS);
        if (resolved != IPAddress(0, 0, 0, 0) && resolved != INADDR_NONE) {
            LOG_DEBUG_MQTT("mDNS resolved %s -> %s",
                          _primary.host, resolved.toString().c_str());
            return resolved;
        }
        LOG_DEBUG_MQTT("mDNS resolution failed for %s", _primary.host);
    }
#endif

    IPAddress ip;
    if (ip.fromString(_primary.host)) {
        return ip;
    }
    return INADDR_NONE;
}

// ============================================================================
//  Internal
// ============================================================================

void MqttManager::applyBrokerConfig(const MqttBrokerConfig& cfg) {
    // Select the correct transport client based on TLS requirement
#ifndef ESP8266
    if (cfg.useTls) {
        if (cfg.caCert) {
            _secureClient.setCACert(cfg.caCert);
        } else {
            _secureClient.setInsecure();  // Accept any cert (not ideal, but functional)
        }
        _client->setClient(_secureClient);
        LOG_DEBUG_MQTT("Using WiFiClientSecure (TLS) for %s:%d", cfg.host, cfg.port);
    } else {
        _client->setClient(_plainClient);
        LOG_DEBUG_MQTT("Using WiFiClient (plain TCP) for %s:%d", cfg.host, cfg.port);
    }
#else
    // ESP8266 doesn't support WiFiClientSecure in the same way
    _client->setClient(_plainClient);
#endif

    _client->setServer(cfg.host, cfg.port);
}

void MqttManager::switchBroker() {
    _failCount = 0;

    if (_activeBroker == MqttBroker::PRIMARY_LOCAL) {
        _activeBroker = MqttBroker::FALLBACK_CLOUD;
        _lastPrimaryRetry = millis();
        LOG_INFO("MqttMgr", ">>> Switching to CLOUD fallback broker (%s:%d)",
                 _fallback.host, _fallback.port);
    } else {
        _activeBroker = MqttBroker::PRIMARY_LOCAL;
        LOG_INFO("MqttMgr", ">>> Switching back to LOCAL primary broker (%s:%d)",
                 _primary.host, _primary.port);
    }

    const MqttBrokerConfig& cfg =
        (_activeBroker == MqttBroker::PRIMARY_LOCAL) ? _primary : _fallback;
    applyBrokerConfig(cfg);
}
