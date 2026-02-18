/**
 * @file maxxfan_ir.cpp
 * @brief MaxxFan Deluxe IR control library implementation
 * 
 * Implements IR transmission using ESP32 RMT peripheral with 38 kHz carrier.
 * Protocol: Custom RS232-like encoding (1 start bit, 8 data bits LSB first, 2 stop bits)
 * Bit period: 800 µs
 * 
 * Ported from: https://github.com/brown-studios/esphome-maxxfan-protocol (MIT license)
 */

#include "maxxfan_ir.h"

#if defined(ESP32)
#include "driver/rmt.h"
#endif

// Protocol constants
static const uint8_t PREAMBLE[] = { 0x5A, 0xA5, 0x80, 0x7F, 0x40, 0xBF, 0x20, 0xDF, 0x10, 0xCC };
static const uint32_t BIT_TIME_US = 800;
static const uint32_t CARRIER_FREQ_HZ = 38000;

// RMT configuration
static rmt_channel_t rmt_channel = RMT_CHANNEL_0;
static uint8_t ir_gpio_pin = 0;
static bool initialized = false;

// Current assumed state (IR is one-directional)
static MaxxFanState current_state;

/**
 * @brief Convert microseconds to RMT ticks
 * @param us Duration in microseconds
 * @return Duration in RMT ticks (80 MHz clock / 80 = 1 MHz = 1 µs per tick)
 */
static inline uint32_t us_to_rmt_ticks(uint32_t us) {
    return us; // 1 µs per tick with default RMT clock divider of 80
}

void maxxfan_ir_init(uint8_t gpio_pin) {
#if defined(ESP32)
    ir_gpio_pin = gpio_pin;
    
    rmt_config_t rmt_tx_config = {};
    rmt_tx_config.rmt_mode = RMT_MODE_TX;
    rmt_tx_config.channel = rmt_channel;
    rmt_tx_config.gpio_num = static_cast<gpio_num_t>(gpio_pin);
    rmt_tx_config.mem_block_num = 1;
    rmt_tx_config.clk_div = 80; // 80 MHz / 80 = 1 MHz = 1 µs per tick
    rmt_tx_config.tx_config.loop_en = false;
    rmt_tx_config.tx_config.carrier_en = true;
    rmt_tx_config.tx_config.carrier_freq_hz = CARRIER_FREQ_HZ;
    rmt_tx_config.tx_config.carrier_duty_percent = 33; // 33% duty cycle
    rmt_tx_config.tx_config.carrier_level = RMT_CARRIER_LEVEL_HIGH;
    rmt_tx_config.tx_config.idle_output_en = true;
    rmt_tx_config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    
    esp_err_t err = rmt_config(&rmt_tx_config);
    if (err != ESP_OK) {
        Serial.printf("[FAN_IR] RMT config failed: %d\n", err);
        return;
    }
    
    err = rmt_driver_install(rmt_channel, 0, 0);
    if (err != ESP_OK) {
        Serial.printf("[FAN_IR] RMT driver install failed: %d\n", err);
        return;
    }
    
    initialized = true;
    Serial.printf("[FAN_IR] Initialized on GPIO %d\n", gpio_pin);
#else
    Serial.println("[FAN_IR] ERROR: RMT peripheral only available on ESP32");
#endif
}

void maxxfan_ir_send(const MaxxFanState& state) {
    // Update internal state
    current_state = state;
    
    // Build 16-byte packet
    uint8_t packet[16];
    memcpy(packet, PREAMBLE, sizeof(PREAMBLE));
    
    // Byte 10: State flags
    packet[10] = 0 |
        (state.fan_on ? 0x01 : 0) |
        (state.special ? 0x02 : 0) |
        (state.exhaust ? 0x04 : 0) |
        (state.lid_open ? 0x08 : 0) |
        (state.auto_mode ? 0x10 : 0) |
        (state.warn ? 0x20 : 0);
    
    packet[11] = state.speed;           // Byte 11: Fan speed (0-100)
    packet[12] = state.auto_temp_f;     // Byte 12: Auto temp setpoint in °F
    packet[13] = 0xFF;                  // Byte 13: Fixed value
    packet[14] = 0x23;                  // Byte 14: Fixed value
    
    // Byte 15: XOR checksum of bytes 10-14
    packet[15] = packet[10] ^ packet[11] ^ packet[12] ^ packet[13] ^ packet[14];
    
#ifdef DRY_RUN_FAN
    // DRY_RUN mode: Log packet instead of transmitting
    Serial.print("[FAN_IR] DRY_RUN TX: ");
    for (size_t i = 0; i < sizeof(packet); i++) {
        Serial.printf("%02X ", packet[i]);
    }
    Serial.println();
    return;
#endif
    
#if defined(ESP32)
    if (!initialized) {
        Serial.println("[FAN_IR] ERROR: Not initialized. Call maxxfan_ir_init() first.");
        return;
    }
    
    // Encode packet as RMT items
    // Each byte: 1 start bit (0), 8 data bits (LSB first), 2 stop bits (1)
    // RS232-like encoding: mark = carrier on, space = carrier off
    // Bit 0: mark for BIT_TIME_US
    // Bit 1: space for BIT_TIME_US
    
    // Calculate required RMT items
    // Per byte: 1 start + 8 data + 2 stop = 11 bits = 11 items
    // Plus 1 item for final space
    // Total: 16 * 11 + 1 = 177 items
    
    const size_t max_items = 200; // Sufficient buffer
    rmt_item32_t items[max_items];
    size_t item_idx = 0;
    
    for (size_t byte_idx = 0; byte_idx < sizeof(packet); byte_idx++) {
        uint8_t byte_val = packet[byte_idx];
        
        // Start bit (0): mark
        items[item_idx].level0 = 1;
        items[item_idx].duration0 = us_to_rmt_ticks(BIT_TIME_US);
        items[item_idx].level1 = 0; // Placeholder, will be filled
        items[item_idx].duration1 = 0;
        item_idx++;
        
        // 8 data bits (LSB first)
        for (uint8_t bit_idx = 0; bit_idx < 8; bit_idx++) {
            bool bit_value = (byte_val >> bit_idx) & 0x01;
            
            if (bit_value) {
                // Bit 1: space
                items[item_idx].level0 = 0;
                items[item_idx].duration0 = us_to_rmt_ticks(BIT_TIME_US);
            } else {
                // Bit 0: mark
                items[item_idx].level0 = 1;
                items[item_idx].duration0 = us_to_rmt_ticks(BIT_TIME_US);
            }
            items[item_idx].level1 = 0; // Placeholder
            items[item_idx].duration1 = 0;
            item_idx++;
        }
        
        // 2 stop bits (1): space + space
        items[item_idx].level0 = 0;
        items[item_idx].duration0 = us_to_rmt_ticks(BIT_TIME_US * 2);
        items[item_idx].level1 = 0;
        items[item_idx].duration1 = 0;
        item_idx++;
    }
    
    // Final long space (end of transmission)
    items[item_idx].level0 = 0;
    items[item_idx].duration0 = us_to_rmt_ticks(BIT_TIME_US * 8);
    items[item_idx].level1 = 0;
    items[item_idx].duration1 = 0;
    item_idx++;
    
    // Transmit
    esp_err_t err = rmt_write_items(rmt_channel, items, item_idx, true); // true = wait until done
    if (err != ESP_OK) {
        Serial.printf("[FAN_IR] RMT transmit failed: %d\n", err);
        return;
    }
    
    Serial.println("[FAN_IR] Packet transmitted");
#else
    Serial.println("[FAN_IR] ERROR: RMT peripheral only available on ESP32");
#endif
}

MaxxFanState& maxxfan_ir_get_state() {
    return current_state;
}
