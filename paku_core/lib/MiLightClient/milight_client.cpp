/**
 * @file milight_client.cpp
 * @brief Implementation of MiLight/MIBO CCT light controller
 * 
 * Protocol implementation based on esp8266_milight_hub by Chris Mullins (MIT License)
 * https://github.com/sidoh/esp8266_milight_hub
 */

#include "milight_client.h"
#include <RF24.h>
#include <SPI.h>

#ifdef ESP8266
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#endif

// DRY_RUN_LIGHT mode - log packets instead of transmitting
#ifdef DRY_RUN_LIGHT
#define MILIGHT_DRY_RUN 1
#else
#define MILIGHT_DRY_RUN 0
#endif

// CCT v1 Protocol Constants (7-byte packets)
#define CCT_PACKET_SIZE 7
#define CCT_PREAMBLE_SIZE 2
#define CCT_SYNCWORD0 0x55
#define CCT_SYNCWORD1 0xAA

// CCT v1 Commands
#define CCT_ALL_ON           0x05
#define CCT_ALL_OFF          0x09
#define CCT_GROUP_1_ON       0x08
#define CCT_GROUP_1_OFF      0x0B
#define CCT_GROUP_2_ON       0x0D
#define CCT_GROUP_2_OFF      0x03
#define CCT_GROUP_3_ON       0x07
#define CCT_GROUP_3_OFF      0x0A
#define CCT_GROUP_4_ON       0x02
#define CCT_GROUP_4_OFF      0x06
#define CCT_BRIGHTNESS_DOWN  0x04
#define CCT_BRIGHTNESS_UP    0x0C
#define CCT_TEMPERATURE_UP   0x0E
#define CCT_TEMPERATURE_DOWN 0x0F

// FUT091 (CCT v2) Protocol Constants
#define FUT091_SYNCWORD0 0x5A
#define FUT091_SYNCWORD1 0xA5
#define FUT091_PACKET_ID 0x21  // Protocol identifier for CCT v2

// NRF24 Configuration
#define MILIGHT_RF_CHANNELS 3
static const uint8_t MILIGHT_CHANNELS[MILIGHT_RF_CHANNELS] = {9, 40, 71};  // 2.409, 2.440, 2.471 GHz

// Static variables
static RF24* radio = nullptr;
static bool initialized = false;
static MiLightState channel_states[4];  // State for channels 1-4
static uint16_t current_device_id = 0;
static uint8_t sequence_num = 0;

// Forward declarations
static void cct_build_packet(uint8_t* packet, uint16_t device_id, uint8_t command, uint8_t arg);
static void fut091_build_packet(uint8_t* packet, uint16_t device_id, uint8_t group, uint8_t command, uint8_t arg);
static bool transmit_packet(const uint8_t* packet, size_t len);
static void log_packet_hex(const char* prefix, const uint8_t* packet, size_t len);

/**
 * @brief Initialize MiLight radio and NRF24 module
 */
bool milight_init(uint8_t ce_pin, uint8_t csn_pin, uint8_t sck_pin, uint8_t mosi_pin, uint8_t miso_pin) {
#if MILIGHT_DRY_RUN
    Serial.println("[MILIGHT] DRY_RUN mode - no hardware initialization");
    initialized = true;
    
    // Generate default device ID from last 2 bytes of MAC
    uint8_t mac[6];
    WiFi.macAddress(mac);
    current_device_id = (mac[4] << 8) | mac[5];
    
    // Initialize channel states
    for (int i = 0; i < 4; i++) {
        channel_states[i].on = false;
        channel_states[i].brightness = 100;
        channel_states[i].color_temp = 350;  // Neutral
        channel_states[i].channel = i + 1;
        channel_states[i].protocol = PROTOCOL_CCT;
        channel_states[i].device_id = current_device_id;
    }
    
    Serial.printf("[MILIGHT] Initialized with device_id=0x%04X\n", current_device_id);
    return true;
#else
    // Initialize SPI bus with custom pins
    SPI.begin(sck_pin, miso_pin, mosi_pin, csn_pin);
    
    // Create RF24 instance
    radio = new RF24(ce_pin, csn_pin);
    
    if (!radio->begin()) {
        Serial.println("[MILIGHT] ERROR: NRF24 initialization failed");
        delete radio;
        radio = nullptr;
        return false;
    }
    
    // Configure NRF24 for MiLight protocol
    radio->setAutoAck(false);
    radio->setDataRate(RF24_1MBPS);
    radio->setPALevel(RF24_PA_MAX);  // Maximum transmit power
    radio->setPayloadSize(CCT_PACKET_SIZE);
    radio->openWritingPipe(0x4C494748);  // "LIGH" in hex
    radio->stopListening();
    
    // Set initial channel
    radio->setChannel(MILIGHT_CHANNELS[0]);
    
    // Generate default device ID from last 2 bytes of MAC
    uint8_t mac[6];
    WiFi.macAddress(mac);
    current_device_id = (mac[4] << 8) | mac[5];
    
    // Initialize channel states
    for (int i = 0; i < 4; i++) {
        channel_states[i].on = false;
        channel_states[i].brightness = 100;
        channel_states[i].color_temp = 350;  // Neutral
        channel_states[i].channel = i + 1;
        channel_states[i].protocol = PROTOCOL_CCT;
        channel_states[i].device_id = current_device_id;
    }
    
    initialized = true;
    Serial.printf("[MILIGHT] Initialized on SPI2 - device_id=0x%04X\n", current_device_id);
    return true;
#endif
}

/**
 * @brief Build CCT v1 protocol packet
 */
static void cct_build_packet(uint8_t* packet, uint16_t device_id, uint8_t command, uint8_t arg) {
    // Packet format: [device_id_high, device_id_low, 0x00, 0x00, command, arg, checksum]
    packet[0] = (device_id >> 8) & 0xFF;
    packet[1] = device_id & 0xFF;
    packet[2] = 0x00;
    packet[3] = 0x00;
    packet[4] = command;
    packet[5] = arg;
    
    // Calculate checksum (XOR of all bytes)
    packet[6] = 0;
    for (int i = 0; i < 6; i++) {
        packet[6] ^= packet[i];
    }
}

/**
 * @brief Build FUT091 (CCT v2) protocol packet
 */
static void fut091_build_packet(uint8_t* packet, uint16_t device_id, uint8_t group, uint8_t command, uint8_t arg) {
    // FUT091 uses encoded packets with sequence numbers
    // Simplified implementation - packet format:
    // [device_id_high, device_id_low, 0x00, sequence, protocol_id, group, command, arg, checksum]
    packet[0] = (device_id >> 8) & 0xFF;
    packet[1] = device_id & 0xFF;
    packet[2] = 0x00;
    packet[3] = sequence_num++;
    packet[4] = FUT091_PACKET_ID;
    packet[5] = group;
    packet[6] = command;
    packet[7] = arg;
    
    // Calculate checksum
    packet[8] = 0;
    for (int i = 0; i < 8; i++) {
        packet[8] ^= packet[i];
    }
}

/**
 * @brief Transmit packet via NRF24
 */
static bool transmit_packet(const uint8_t* packet, size_t len) {
#if MILIGHT_DRY_RUN
    // In dry-run mode, just log the packet
    return true;
#else
    if (!initialized || !radio) {
        return false;
    }
    
    // Transmit on all 3 channels for better reliability
    bool success = false;
    for (int i = 0; i < MILIGHT_RF_CHANNELS; i++) {
        radio->setChannel(MILIGHT_CHANNELS[i]);
        if (radio->write(packet, len)) {
            success = true;
        }
        delayMicroseconds(1000);  // Small delay between transmissions
    }
    
    return success;
#endif
}

/**
 * @brief Log packet as hex string
 */
static void log_packet_hex(const char* prefix, const uint8_t* packet, size_t len) {
    Serial.print(prefix);
    for (size_t i = 0; i < len; i++) {
        if (i > 0) Serial.print(" ");
        if (packet[i] < 0x10) Serial.print("0");
        Serial.print(packet[i], HEX);
    }
    Serial.println();
}

/**
 * @brief Send current state to MiLight receiver
 */
bool milight_send_state(const MiLightState& state) {
    if (!initialized) {
        Serial.println("[MILIGHT] ERROR: Not initialized");
        return false;
    }
    
    // Update stored state
    if (state.channel >= 1 && state.channel <= 4) {
        channel_states[state.channel - 1] = state;
    }
    
    uint16_t device_id = (state.device_id != 0) ? state.device_id : current_device_id;
    
    if (state.protocol == PROTOCOL_CCT) {
        // CCT v1 protocol
        uint8_t packet[CCT_PACKET_SIZE];
        
        // Send ON/OFF command
        uint8_t on_off_cmd = 0;
        if (state.channel == 1) {
            on_off_cmd = state.on ? CCT_GROUP_1_ON : CCT_GROUP_1_OFF;
        } else if (state.channel == 2) {
            on_off_cmd = state.on ? CCT_GROUP_2_ON : CCT_GROUP_2_OFF;
        } else if (state.channel == 3) {
            on_off_cmd = state.on ? CCT_GROUP_3_ON : CCT_GROUP_3_OFF;
        } else if (state.channel == 4) {
            on_off_cmd = state.on ? CCT_GROUP_4_ON : CCT_GROUP_4_OFF;
        }
        
        cct_build_packet(packet, device_id, on_off_cmd, 0x00);
        log_packet_hex("[MILIGHT] CCT ON/OFF: ", packet, CCT_PACKET_SIZE);
        transmit_packet(packet, CCT_PACKET_SIZE);
        delay(100);  // Delay between commands
        
        if (state.on) {
            // Send brightness commands (CCT uses step commands)
            // Target: state.brightness (0-100)
            // Map to ~10 steps
            uint8_t steps = (state.brightness * 10) / 100;
            for (uint8_t i = 0; i < steps; i++) {
                cct_build_packet(packet, device_id, CCT_BRIGHTNESS_UP, 0x00);
                transmit_packet(packet, CCT_PACKET_SIZE);
                delay(50);
            }
            
            // Send color temperature commands
            // color_temp: 153 (cool) to 500 (warm), middle = ~350
            // Map to temperature up/down steps
            if (state.color_temp < 300) {
                // Cooler - send temperature down commands
                uint8_t steps = (300 - state.color_temp) / 20;
                for (uint8_t i = 0; i < steps; i++) {
                    cct_build_packet(packet, device_id, CCT_TEMPERATURE_DOWN, 0x00);
                    transmit_packet(packet, CCT_PACKET_SIZE);
                    delay(50);
                }
            } else if (state.color_temp > 400) {
                // Warmer - send temperature up commands
                uint8_t steps = (state.color_temp - 400) / 20;
                for (uint8_t i = 0; i < steps; i++) {
                    cct_build_packet(packet, device_id, CCT_TEMPERATURE_UP, 0x00);
                    transmit_packet(packet, CCT_PACKET_SIZE);
                    delay(50);
                }
            }
        }
        
        return true;
        
    } else if (state.protocol == PROTOCOL_FUT091) {
        // FUT091 (CCT v2) protocol - supports direct value commands
        uint8_t packet[9];  // FUT091 packets are 9 bytes
        
        // ON/OFF command
        fut091_build_packet(packet, device_id, state.channel, 0x01, state.on ? 0x01 : 0x00);
        log_packet_hex("[MILIGHT] FUT091 ON/OFF: ", packet, 9);
        transmit_packet(packet, 9);
        delay(100);
        
        if (state.on) {
            // Brightness command (0-100)
            fut091_build_packet(packet, device_id, state.channel, 0x02, state.brightness);
            log_packet_hex("[MILIGHT] FUT091 BRIGHTNESS: ", packet, 9);
            transmit_packet(packet, 9);
            delay(100);
            
            // Color temperature command
            // Map mireds (153-500) to 0-100 (cool to warm)
            uint8_t temp_value = map(state.color_temp, 153, 500, 0, 100);
            fut091_build_packet(packet, device_id, state.channel, 0x03, temp_value);
            log_packet_hex("[MILIGHT] FUT091 TEMP: ", packet, 9);
            transmit_packet(packet, 9);
        }
        
        return true;
    }
    
    return false;
}

/**
 * @brief Pair receiver to this controller
 */
bool milight_pair(uint8_t channel, MiLightProtocol protocol, uint16_t device_id) {
    if (!initialized) {
        return false;
    }
    
    if (device_id == 0) {
        device_id = current_device_id;
    }
    
    Serial.printf("[MILIGHT] Pairing channel %d with device_id=0x%04X protocol=%s\n",
                  channel, device_id, milight_protocol_to_string(protocol));
    
    if (protocol == PROTOCOL_CCT) {
        // CCT pairing: Turn on the specific group multiple times
        uint8_t packet[CCT_PACKET_SIZE];
        uint8_t on_cmd = CCT_GROUP_1_ON;
        
        if (channel == 2) on_cmd = CCT_GROUP_2_ON;
        else if (channel == 3) on_cmd = CCT_GROUP_3_ON;
        else if (channel == 4) on_cmd = CCT_GROUP_4_ON;
        
        for (int i = 0; i < 5; i++) {
            cct_build_packet(packet, device_id, on_cmd, 0x00);
            log_packet_hex("[MILIGHT] CCT PAIR: ", packet, CCT_PACKET_SIZE);
            transmit_packet(packet, CCT_PACKET_SIZE);
            delay(200);
        }
    } else if (protocol == PROTOCOL_FUT091) {
        // FUT091 pairing: Send ON command multiple times
        uint8_t packet[9];
        for (int i = 0; i < 5; i++) {
            fut091_build_packet(packet, device_id, channel, 0x01, 0x01);
            log_packet_hex("[MILIGHT] FUT091 PAIR: ", packet, 9);
            transmit_packet(packet, 9);
            delay(200);
        }
    }
    
    return true;
}

/**
 * @brief Unpair receiver from this controller
 */
bool milight_unpair(uint8_t channel, MiLightProtocol protocol) {
    if (!initialized) {
        return false;
    }
    
    Serial.printf("[MILIGHT] Unpairing channel %d protocol=%s\n",
                  channel, milight_protocol_to_string(protocol));
    
    if (protocol == PROTOCOL_CCT) {
        // CCT unpairing: Turn off the specific group multiple times
        uint8_t packet[CCT_PACKET_SIZE];
        uint8_t off_cmd = CCT_GROUP_1_OFF;
        
        if (channel == 2) off_cmd = CCT_GROUP_2_OFF;
        else if (channel == 3) off_cmd = CCT_GROUP_3_OFF;
        else if (channel == 4) off_cmd = CCT_GROUP_4_OFF;
        
        for (int i = 0; i < 5; i++) {
            cct_build_packet(packet, current_device_id, off_cmd, 0x00);
            log_packet_hex("[MILIGHT] CCT UNPAIR: ", packet, CCT_PACKET_SIZE);
            transmit_packet(packet, CCT_PACKET_SIZE);
            delay(200);
        }
    } else if (protocol == PROTOCOL_FUT091) {
        // FUT091 unpairing: Send OFF command multiple times
        uint8_t packet[9];
        for (int i = 0; i < 5; i++) {
            fut091_build_packet(packet, current_device_id, channel, 0x01, 0x00);
            log_packet_hex("[MILIGHT] FUT091 UNPAIR: ", packet, 9);
            transmit_packet(packet, 9);
            delay(200);
        }
    }
    
    return true;
}

/**
 * @brief Get current assumed state for a channel
 */
MiLightState& milight_get_state(uint8_t channel) {
    if (channel >= 1 && channel <= 4) {
        return channel_states[channel - 1];
    }
    return channel_states[0];  // Default to channel 1
}

/**
 * @brief Set device ID for all channels
 */
void milight_set_device_id(uint16_t device_id) {
    if (device_id == 0) {
        // Auto-generate from MAC
        uint8_t mac[6];
        WiFi.macAddress(mac);
        device_id = (mac[4] << 8) | mac[5];
    }
    current_device_id = device_id;
    
    for (int i = 0; i < 4; i++) {
        channel_states[i].device_id = device_id;
    }
    
    Serial.printf("[MILIGHT] Device ID set to 0x%04X\n", device_id);
}

/**
 * @brief Get current device ID
 */
uint16_t milight_get_device_id() {
    return current_device_id;
}

/**
 * @brief Convert protocol name to enum
 */
MiLightProtocol milight_protocol_from_string(const char* protocol_name) {
    if (strcmp(protocol_name, "fut091") == 0 || strcmp(protocol_name, "FUT091") == 0) {
        return PROTOCOL_FUT091;
    }
    return PROTOCOL_CCT;  // Default to CCT
}

/**
 * @brief Convert protocol enum to name
 */
const char* milight_protocol_to_string(MiLightProtocol protocol) {
    switch (protocol) {
        case PROTOCOL_FUT091:
            return "fut091";
        case PROTOCOL_CCT:
        default:
            return "cct";
    }
}
