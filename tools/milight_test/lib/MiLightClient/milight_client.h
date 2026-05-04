/**
 * @file milight_client.h
 * @brief Simplified MiLight/MIBO CCT light controller client
 * 
 * This is a lightweight wrapper around NRF24L01+ radio for controlling
 * MiLight/MIBO CCT (Color Temperature) LED receivers.
 * 
 * Protocol implementation based on esp8266_milight_hub by Chris Mullins (MIT License)
 * https://github.com/sidoh/esp8266_milight_hub
 * 
 * Supports two protocols:
 * - CCT v1 (FUT007/FUT011) - Original CCT protocol
 * - FUT091 (CCT v2) - Newer CCT protocol
 * 
 * Hardware: ESP32-S3 + NRF24L01+ module
 * 
 * @note This is a simplified implementation focused on CCT control only.
 *       Full RGB/RGBW support and advanced features are not included.
 */

#pragma once

#include <Arduino.h>
#include <stdint.h>

/**
 * @brief MiLight protocol types supported
 */
enum MiLightProtocol {
    PROTOCOL_CCT = 0,      ///< CCT v1 (FUT007/FUT011) - 7-byte packets
    PROTOCOL_FUT091 = 1    ///< FUT091 (CCT v2) - encoded packets
};

/**
 * @brief MiLight device state
 */
struct MiLightState {
    bool on;               ///< Light on/off state
    uint8_t brightness;    ///< Brightness 0-100
    uint16_t color_temp;   ///< Color temperature in mireds (153=cool, 500=warm)
    uint8_t channel;       ///< Channel 1-4 (receiver group)
    MiLightProtocol protocol; ///< Protocol to use
    uint16_t device_id;    ///< Device ID for pairing (0 = use default)
};

/**
 * @brief Initialize MiLight radio and NRF24 module
 * 
 * @param ce_pin GPIO pin for NRF24 CE (Chip Enable)
 * @param csn_pin GPIO pin for NRF24 CSN (SPI Chip Select)
 * @param sck_pin GPIO pin for SPI SCK (clock)
 * @param mosi_pin GPIO pin for SPI MOSI (data out)
 * @param miso_pin GPIO pin for SPI MISO (data in)
 * @return true if initialization successful, false otherwise
 */
bool milight_init(uint8_t ce_pin, uint8_t csn_pin, uint8_t sck_pin, uint8_t mosi_pin, uint8_t miso_pin);

/**
 * @brief Send current state to MiLight receiver
 * 
 * @param state Light state to send
 * @return true if transmission successful
 */
bool milight_send_state(const MiLightState& state);

/**
 * @brief Pair receiver to this controller
 * 
 * @param channel Channel to pair (1-4)
 * @param protocol Protocol to use
 * @param device_id Device ID to use (0 = auto-generate)
 * @return true if pairing command sent successfully
 */
bool milight_pair(uint8_t channel, MiLightProtocol protocol, uint16_t device_id = 0);

/**
 * @brief Unpair receiver from this controller
 * 
 * @param channel Channel to unpair (1-4)
 * @param protocol Protocol to use
 * @return true if unpair command sent successfully
 */
bool milight_unpair(uint8_t channel, MiLightProtocol protocol);

/**
 * @brief Get current assumed state for a channel
 * 
 * @param channel Channel (1-4)
 * @return Reference to state object
 */
MiLightState& milight_get_state(uint8_t channel);

/**
 * @brief Set device ID for all channels
 * 
 * @param device_id Device ID to use (0 = auto-generate from MAC)
 */
void milight_set_device_id(uint16_t device_id);

/**
 * @brief Get current device ID
 * 
 * @return Current device ID
 */
uint16_t milight_get_device_id();

/**
 * @brief Convert protocol name to enum
 * 
 * @param protocol_name Protocol name ("cct" or "fut091")
 * @return Protocol enum value, or PROTOCOL_CCT if unknown
 */
MiLightProtocol milight_protocol_from_string(const char* protocol_name);

/**
 * @brief Convert protocol enum to name
 * 
 * @param protocol Protocol enum
 * @return Protocol name string
 */
const char* milight_protocol_to_string(MiLightProtocol protocol);
