/**
 * @file mqtt_manager.cpp
 * @brief MQTT broker failover manager implementation.
 *
 * Implements local-preferred, cloud-synced MQTT broker selection
 * with automatic failover and return-to-primary logic.
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
  #define LOG_DEBUG_MQTT(fmt, ...)     do {} while(0)  // Disabled without logging.h
#endif

#ifndef ESP8266
#include <ESPmDNS.h>
#endif

// ============================================================================
//  Public API
// ============================================================================

void MqttManager::begin(PubSubClient& client,
                        Client& netClient,
                        const char* deviceId,
                        const MqttBrokerConfig& primary,
                        const MqttBrokerConfig& fallback,
                        OnConnectCallback onConnect) {
    _client     = &client;
    _netClient  = &netClient;
    _deviceId   = deviceId;
    _primary    = primary;
    _fallback   = fallback;
    _onConnect  = onConnect;

    _activeBroker     = MqttBroker::PRIMARY_LOCAL;
    _failCount        = 0;
    _lastPrimaryRetry = 0;
    _initialProbed    = false;
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
        } else {
            LOG_INFO("MqttMgr", "Local broker unreachable — using CLOUD fallback");
            _activeBroker = MqttBroker::FALLBACK_CLOUD;
        }
        applyBrokerConfig(_activeBroker == MqttBroker::PRIMARY_LOCAL ? _primary : _fallback);
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
                // Will reconnect on next maintain() call
                return false;
            } else {
                LOG_DEBUG_MQTT("Primary still unreachable, staying on cloud");
            }
        }
        return true;
    }

    // --- Not connected — attempt connection ---
    return tryConnectOnce();
}

bool MqttManager::tryConnectOnce() {
    if (!_client) return false;

    const MqttBrokerConfig& cfg =
        (_activeBroker == MqttBroker::PRIMARY_LOCAL) ? _primary : _fallback;

    applyBrokerConfig(cfg);

    LOG_INFO("MqttMgr", "Connecting to %s broker %s:%d ...",
             activeBrokerName(), cfg.host, cfg.port);

    // Build LWT topic: paku/edge/{deviceId}/status
    String lwtTopic = String("paku/edge/") + _deviceId + "/status";

    bool connected;
    if (strlen(cfg.user) > 0) {
        connected = _client->connect(
            _deviceId,
            cfg.user, cfg.pass,
            lwtTopic.c_str(), 1, true,  // QoS 1, retained
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

        // Invoke callback so caller can subscribe to topics
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

    // Try mDNS-resolved IP first, fall back to hostname
    IPAddress resolved = resolveLocalBroker();
    bool ok = probe.connect(resolved, _primary.port, LOCAL_PROBE_TIMEOUT_MS);
    probe.stop();

    LOG_DEBUG_MQTT("Probe result: %s", ok ? "REACHABLE" : "UNREACHABLE");
    return ok;
}

IPAddress MqttManager::resolveLocalBroker() {
#ifndef ESP8266
    // Try mDNS first (e.g., "homeassistant.local" → 192.168.x.x)
    // Strip ".local" suffix if present for MDNS.queryHost()
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

    // Fall back to direct IP parsing or DNS
    IPAddress ip;
    if (ip.fromString(_primary.host)) {
        return ip;  // Already an IP address
    }

    // Let WiFiClient handle DNS resolution via hostname
    // Return INADDR_NONE — caller will use hostname string directly
    return INADDR_NONE;
}

// ============================================================================
//  Internal
// ============================================================================

void MqttManager::applyBrokerConfig(const MqttBrokerConfig& cfg) {
    _client->setServer(cfg.host, cfg.port);

#ifndef ESP8266
    // Apply TLS certificate if this broker uses TLS
    if (cfg.useTls && cfg.caCert) {
        WiFiClientSecure* secClient = static_cast<WiFiClientSecure*>(_netClient);
        secClient->setCACert(cfg.caCert);
        LOG_DEBUG_MQTT("TLS CA cert applied for %s", cfg.host);
    }
#endif
}

void MqttManager::switchBroker() {
    _failCount = 0;

    if (_activeBroker == MqttBroker::PRIMARY_LOCAL) {
        _activeBroker = MqttBroker::FALLBACK_CLOUD;
        _lastPrimaryRetry = millis();  // Start retry timer
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
