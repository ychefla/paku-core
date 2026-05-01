#pragma once
/**
 * @file mqtt_manager.h
 * @brief MQTT broker failover manager — local-preferred, cloud-synced.
 *
 * Manages connections to two MQTT brokers with automatic failover:
 *   1. **Primary (local):** RPi / Home Assistant Mosquitto (homeassistant.local)
 *   2. **Fallback (cloud):** Cloud MQTT broker for when RPi is unreachable
 *
 * Architecture:
 *   - On boot, quickly probes the local broker via TCP (2 s timeout).
 *   - If reachable → connects locally (sub-ms latency, works offline).
 *   - If unreachable → connects to cloud broker immediately.
 *   - While on cloud, re-probes local every PRIMARY_RETRY_INTERVAL_MS.
 *   - When local comes back, disconnects from cloud and switches to local.
 *
 * The RPi side runs a Mosquitto bridge to the cloud broker, so data always
 * reaches both HA and the cloud backend regardless of which broker the
 * ESP32 is connected to.
 *
 * TLS handling:
 *   The manager owns both a WiFiClient (plain) and WiFiClientSecure (TLS).
 *   PubSubClient is re-pointed to the correct transport when switching
 *   brokers, so a plain-TCP local broker and a TLS cloud broker work
 *   seamlessly without the caller needing to manage two client objects.
 *
 * @note This class does NOT own the PubSubClient instance.
 *       The caller must create and pass it in via begin().
 */

#ifndef ESP8266
#include <WiFi.h>
#include <WiFiClientSecure.h>
#else
#include <ESP8266WiFi.h>
#endif

#include <PubSubClient.h>

/// @brief TCP probe timeout for local broker reachability check (ms).
///        Keep short to avoid blocking the GUI when RPi is on a different network.
static constexpr uint16_t LOCAL_PROBE_TIMEOUT_MS = 500;

/// @brief Consecutive connection failures before switching to fallback broker.
static constexpr uint8_t  MAX_CONNECT_FAILURES = 3;

/// @brief How often to re-probe the primary broker while on fallback (ms).
static constexpr unsigned long PRIMARY_RETRY_INTERVAL_MS = 60000;

/// @brief Minimum delay between individual connection attempts (ms).
///        Prevents hammering the broker in a tight loop.
static constexpr unsigned long CONNECT_RETRY_DELAY_MS = 2000;

/// @brief Which broker the manager is currently targeting.
enum class MqttBroker : uint8_t {
    PRIMARY_LOCAL,    ///< RPi Mosquitto (homeassistant.local / hardcoded IP)
    FALLBACK_CLOUD    ///< Cloud MQTT broker
};

/// @brief Connection parameters for a single MQTT broker.
struct MqttBrokerConfig {
    const char* host;       ///< Hostname or IP address
    uint16_t    port;       ///< MQTT port (1883 plain, 8883 TLS)
    const char* user;       ///< Username (empty string for anonymous)
    const char* pass;       ///< Password (empty string for anonymous)
    bool        useTls;     ///< Whether to use TLS (WiFiClientSecure)
    const char* caCert;     ///< CA certificate PEM (nullptr if no TLS)
};

/**
 * @brief Manages MQTT connection with automatic failover between local and cloud brokers.
 *
 * Owns its own WiFiClient (plain) and WiFiClientSecure (TLS) instances.
 * PubSubClient is re-pointed to the correct transport automatically.
 *
 * Usage:
 * @code
 *   PubSubClient client;
 *   MqttManager mqttMgr;
 *   mqttMgr.begin(client, deviceId, primaryCfg, fallbackCfg, onConnect);
 *   // In loop():
 *   mqttMgr.maintain();
 * @endcode
 */
class MqttManager {
public:
    /// @brief Callback invoked after a successful MQTT connection.
    ///        Use this to subscribe to topics and publish status.
    using OnConnectCallback = void (*)(PubSubClient& client, MqttBroker broker);

    /**
     * @brief Initialize the manager with broker configurations.
     *
     * @param client       Reference to PubSubClient (caller-owned)
     * @param deviceId     MQTT client ID (must remain valid for lifetime)
     * @param primary      Local broker configuration (RPi)
     * @param fallback     Cloud broker configuration
     * @param onConnect    Callback invoked on successful connection
     */
    void begin(PubSubClient& client,
               const char* deviceId,
               const MqttBrokerConfig& primary,
               const MqttBrokerConfig& fallback,
               OnConnectCallback onConnect = nullptr);

    /**
     * @brief Call from loop(). Handles connection, failover, and return-to-primary.
     *
     * Non-blocking: performs at most one TCP probe or one MQTT connect attempt
     * per call.  Safe to call every loop() iteration.
     *
     * @return true if currently connected to a broker
     */
    bool maintain();

    /**
     * @brief Perform a single MQTT connect attempt to the currently targeted broker.
     *
     * This is the non-blocking equivalent used by the Phase 2 state machine.
     * Throttled by CONNECT_RETRY_DELAY_MS to prevent hammering.
     *
     * @return true if connected
     */
    bool tryConnectOnce();

    /// @brief Disconnect from the current broker.
    void disconnect();

    /// @brief Get the currently active (or targeted) broker.
    MqttBroker activeBroker() const { return _activeBroker; }

    /// @brief Check if currently using the fallback cloud broker.
    bool isOnFallback() const { return _activeBroker == MqttBroker::FALLBACK_CLOUD; }

    /// @brief Get human-readable name of the active broker.
    const char* activeBrokerName() const;

    /// @brief Get the host of the currently active broker.
    const char* activeHost() const;

    /// @brief Get the port of the currently active broker.
    uint16_t activePort() const;

    /// @brief Get consecutive failure count.
    uint8_t failCount() const { return _failCount; }

    /// @brief True if the last tryConnectOnce() actually attempted a connection
    ///        (vs. returning early due to throttle).
    bool didAttempt() const { return _lastAttemptWasReal; }

    /**
     * @brief Probe local broker reachability via TCP connect.
     * @return true if local broker responded to TCP connect
     */
    bool isLocalReachable();

    /**
     * @brief Resolve the local broker hostname via mDNS.
     * @return Resolved IP address (INADDR_NONE on failure)
     */
    IPAddress resolveLocalBroker();

private:
    void applyBrokerConfig(const MqttBrokerConfig& cfg);
    void switchBroker();

    PubSubClient*      _client      = nullptr;
    const char*        _deviceId    = nullptr;

    MqttBrokerConfig   _primary     = {};
    MqttBrokerConfig   _fallback    = {};

    MqttBroker         _activeBroker     = MqttBroker::PRIMARY_LOCAL;
    uint8_t            _failCount        = 0;
    unsigned long      _lastPrimaryRetry = 0;
    unsigned long      _lastConnectAttempt = 0;
    bool               _initialProbed    = false;
    bool               _lastAttemptWasReal = false;  ///< True when tryConnectOnce() actually tried

    OnConnectCallback  _onConnect   = nullptr;

    // --- Transport clients (owned by this class) ---
    WiFiClient         _plainClient;
#ifndef ESP8266
    WiFiClientSecure   _secureClient;
#endif
};
