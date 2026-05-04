#ifndef TIMING_CONFIG_H
#define TIMING_CONFIG_H

#include <stdint.h>

/**
 * @brief Device timing and sensor configuration structure
 * 
 * Separates concerns between sensor sampling, network transmission, and power management.
 * All timing values are in seconds unless otherwise specified.
 */
struct DeviceConfig {
    /**
     * @brief Network connection timing
     */
    struct {
        uint32_t wake_interval_s;           // How often to wake and sample sensors (default: 60)
        uint32_t connection_duration_max_s; // Max time to stay connected (default: 30)
        uint32_t wifi_connect_timeout_s;    // WiFi connection timeout (default: 10)
        uint32_t mqtt_connect_timeout_s;    // MQTT connection timeout (default: 5)
        char timezone[64];                  // POSIX timezone string (default: "EET-2EEST,M3.5.0/3,M10.5.0/4" for Finland with auto DST)
    } timing;
    
    /**
     * @brief Sensor sampling configuration
     */
    struct {
        struct {
            bool enabled;                   // BLE scanning enabled
            uint32_t scan_duration_s;       // How long to scan for BLE devices (default: 20)
            bool scan_active;               // Active vs passive scanning
        } ble;
        
        struct {
            bool enabled;                   // Wired sensors enabled
            uint8_t sample_count;           // Number of samples to take (default: 3)
            uint16_t sample_interval_ms;    // Time between samples (default: 100)
        } wired;
        
        struct {
            bool enabled;                   // Flow sensor enabled
            uint32_t measurement_duration_s; // How long to measure flow (default: 5)
        } flow;
    } sensors;
    
    /**
     * @brief Power management settings
     */
    struct {
        bool deep_sleep_enabled;            // Use deep sleep between wake cycles
        bool light_sleep_during_wait;       // Use light sleep during short waits
        bool battery_monitor_enabled;       // Monitor and report battery level
    } power;

    /**
     * @brief Runtime peripheral manifest — which hardware is physically connected.
     *
     * Complements compile-time HAS_xxx macros: the code must be compiled in,
     * but these flags let you activate only what is wired without a reflash.
     * Defaults all true so existing boards behave unchanged until overridden
     * via paku/edge/{device_id}/config/set.  Takes effect on next reboot.
     */
    struct {
        bool ble_ruuvi;     // Ruuvi BLE tag scanning
        bool ble_moko;      // MoKo BLE sensor scanning
        bool ble_frezzer;   // Frezzer compressor fridge BLE control
        bool wired_ds18b20; // DS18B20 1-Wire temperature sensors
        bool heater;        // Hydronic heater (Autoterm UART)
        bool fan_ir;        // MaxxFan IR control
        bool milight;       // MiLight/MIBO NRF24L01+ lights
    } peripherals;

    /**
     * @brief Home Assistant integration settings.
     */
    struct {
        bool enabled;       // Connect to local HA MQTT broker (mDNS discovery)
    } ha;
    
    /**
     * @brief Load default configuration
     */
    void loadDefaults() {
        // Timing defaults
        timing.wake_interval_s = 60;
        timing.connection_duration_max_s = 30;
        timing.wifi_connect_timeout_s = 10;
        timing.mqtt_connect_timeout_s = 5;
        // Finland: EET (UTC+2) with automatic DST to EEST (UTC+3)
        // DST: last Sunday of March at 3 AM -> last Sunday of October at 4 AM
        strncpy(timing.timezone, "EET-2EEST,M3.5.0/3,M10.5.0/4", sizeof(timing.timezone) - 1);
        timing.timezone[sizeof(timing.timezone) - 1] = '\0';
        
        // Sensor defaults
        sensors.ble.enabled = true;
        sensors.ble.scan_duration_s = 20;
        sensors.ble.scan_active = true;
        
        sensors.wired.enabled = true;
        sensors.wired.sample_count = 3;
        sensors.wired.sample_interval_ms = 100;
        
        sensors.flow.enabled = false;  // Disabled by default (no hardware)
        sensors.flow.measurement_duration_s = 5;
        
        // Power defaults
        power.deep_sleep_enabled = false;  // Disabled by default for safety
        power.light_sleep_during_wait = true;
        power.battery_monitor_enabled = false;

        // Peripheral defaults — all enabled so existing boards behave unchanged
        peripherals.ble_ruuvi    = true;
        peripherals.ble_moko     = true;
        peripherals.ble_frezzer  = true;
        peripherals.wired_ds18b20 = true;
        peripherals.heater       = true;
        peripherals.fan_ir       = true;
        peripherals.milight      = true;

        // HA integration default — use local broker when available
        ha.enabled = true;
    }
    
    /**
     * @brief Apply scenario-specific settings
     * @param scenario Scenario name ("default", "heater_active", "power_save")
     */
    void applyScenario(const char* scenario) {
        if (strcmp(scenario, "heater_active") == 0) {
            // Fast updates when heater is active
            timing.wake_interval_s = 10;
            sensors.ble.scan_duration_s = 10;
        } else if (strcmp(scenario, "power_save") == 0) {
            // Ultra low power mode
            timing.wake_interval_s = 300;
            sensors.ble.scan_duration_s = 30;
        } else {
            // Default balanced mode
            timing.wake_interval_s = 60;
            sensors.ble.scan_duration_s = 20;
        }
    }
};

/**
 * @brief Device operational state machine
 */
enum DeviceState {
    STATE_INIT,              // Initial boot/setup
    STATE_SENSOR_SAMPLING,   // Reading sensors (WiFi off)
    STATE_DATA_TRANSMISSION, // Connecting and sending data (WiFi on)
    STATE_SLEEP_PREP,        // Preparing for sleep
    STATE_DEEP_SLEEP,        // In deep sleep
    STATE_CONTINUOUS         // Continuous operation (no sleep)
};

#endif // TIMING_CONFIG_H
