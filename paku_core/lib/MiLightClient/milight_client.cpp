/**
 * @file milight_client.cpp
 * @brief MiLight/MIBO CCT V2 (FUT091) light controller
 *
 * Protocol implementation derived from:
 *   - esp8266_milight_hub by Chris Mullins (MIT)  https://github.com/sidoh/esp8266_milight_hub
 *   - paku-core TX testbench  tools/milight_test/src_tx/main.cpp
 *
 * Key protocol facts (all verified empirically against the bulbs):
 *   - 12-byte TX frame = 10-byte PL1167 frame (V2-encoded) + 2-byte PL1167 CRC
 *   - Every byte is bit-reversed before handing to the nRF24
 *   - nRF24 hardware CRC must be DISABLED (PL1167 layer handles CRC)
 *   - RF channels are +2 from the esp8266_milight_hub config: {6, 41, 76}
 *   - Request type for CCT V2: 0x45
 *   - Commands: 0x11=ON, 0x12=OFF, 0x1C=BRIGHT(0-100), 0x19=TEMP(0=warm,100=cool)
 *   - For ON/OFF arg == group number; for BRIGHT/TEMP arg is the value
 */

#include "milight_client.h"

#ifdef MILIGHT_ENABLED

#include <RF24.h>
#include <SPI.h>
#include <esp_random.h>

#ifdef ESP8266
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#endif

#ifdef DRY_RUN_LIGHT
#define MILIGHT_DRY_RUN 1
#else
#define MILIGHT_DRY_RUN 0
#endif

static inline void ml_delay(unsigned long ms) {
#if !MILIGHT_DRY_RUN
    delay(ms);
#endif
}

// ---------------------------------------------------------------------------
//  CCT v1 (legacy) protocol constants — kept for pair/unpair only
// ---------------------------------------------------------------------------
#define CCT_PACKET_SIZE      7
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

// ---------------------------------------------------------------------------
//  FUT091 (CCT V2) protocol constants
// ---------------------------------------------------------------------------
#define FUT091_REQ_TYPE  0x45   // request type byte in the un-encoded payload
#define FUT091_CMD_ON    0x11   // arg = group
#define FUT091_CMD_OFF   0x12   // arg = group
#define FUT091_CMD_BRIGHT 0x1C  // arg = 0-100 (absolute brightness)
#define FUT091_CMD_TEMP  0x19   // arg = 0-100 (0=warm, 100=cool)
#define FUT091_FRAME_SIZE 12    // 10-byte PL1167 frame + 2-byte CRC

// ---------------------------------------------------------------------------
//  NRF24 RF config
// ---------------------------------------------------------------------------
#define MILIGHT_RF_CHANNELS 3
// esp8266_milight_hub stores {4,39,74} but applies +2 in PL1167_nRF24::
// recalc_parameters, so actual on-air channels are {6, 41, 76}.
static const uint8_t MILIGHT_CHANNELS[MILIGHT_RF_CHANNELS] = {6, 41, 76};
static const uint8_t MILIGHT_CCT_ADDRESS[5] = {0xAA, 0x5A, 0x05, 0x0A, 0x55};

#define MILIGHT_TX_REPS        8    // repetitions per command (CCT / single-frame)
#define MILIGHT_TX_REPS_FUT091 10   // per-rep rebuilds for FUT091 (seq_num increments each rep)
#define MILIGHT_TX_DELAY_US    400  // µs between reps

// ---------------------------------------------------------------------------
//  V2 RF encoding (matches sidoh/esp8266_milight_hub V2RFEncoding.cpp)
// ---------------------------------------------------------------------------
#define V2_OFFSET_JUMP_START 0x54

static const uint8_t V2_OFFSETS[8][4] = {
    {0x45, 0x1F, 0x14, 0x5C},
    {0x2B, 0xC9, 0xE3, 0x11},
    {0x6D, 0x5F, 0x8A, 0x2B},
    {0xAF, 0x03, 0x1D, 0xF3},
    {0x1A, 0xE2, 0xF0, 0xD1},
    {0x04, 0xD8, 0x71, 0x42},
    {0xAF, 0x04, 0xDD, 0x07},
    {0x61, 0x13, 0x38, 0x64},
};

static uint8_t bitReverse(uint8_t b) {
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
}

static uint16_t pl1167Crc(const uint8_t *d, size_t n) {
    uint16_t s = 0;
    for (size_t i = 0; i < n; i++) {
        uint8_t b = d[i];
        for (int j = 0; j < 8; j++) {
            if ((b ^ s) & 1) s = (s >> 1) ^ 0x8408;
            else             s = (s >> 1);
            b >>= 1;
        }
    }
    return s;
}

static uint8_t v2XorKey(uint8_t key) {
    const uint8_t shift = (key & 0x0F) < 0x04 ? 0 : 1;
    const uint8_t x = (((key & 0xF0) >> 4) + shift + 6) % 8;
    return ((((4 + x) ^ 1) & 0x0F) << 4) | ((((key & 0x0F) + 4) ^ 2) & 0x0F);
}

static uint8_t v2Off(uint8_t idx, uint8_t key, uint8_t js) {
    uint8_t b = V2_OFFSETS[idx - 1][key % 4];
    if (js > 0 && key >= js && key < js + 0x80) b += 0x80;
    return b;
}

// V2 encode in-place: p[0]=key (unchanged), p[1..7] encoded, p[8]=checksum
static void v2Encode(uint8_t *p) {
    uint8_t xk  = v2XorKey(p[0]);
    uint8_t sum = xk;
    for (int i = 1; i <= 7; i++) {
        sum  += p[i];
        p[i]  = (p[i] ^ xk) + v2Off(i, p[0], V2_OFFSET_JUMP_START);
    }
    p[8] = ((sum + 2) ^ xk) + V2_OFFSETS[7][p[0] % 4];
}

// ---------------------------------------------------------------------------
//  State
// ---------------------------------------------------------------------------
static RF24*        radio = nullptr;
static bool         initialized = false;
static MiLightState channel_states[4];
static uint16_t     current_device_id = 0;
static uint8_t      sequence_num = 0;

// Stored so milight_send_state can do a lazy reinit if setup() failed
// (NRF24L01+ often needs >25 s from power-on before responding; by the
// time the first MQTT command arrives the module is usually ready).
static bool    _spi_started   = false;
static uint8_t _ce_pin        = 0;
static uint8_t _csn_pin       = 0;
static unsigned long _last_reinit_ms = 0;

// ---------------------------------------------------------------------------
//  Forward declarations
// ---------------------------------------------------------------------------
static void cct_build_packet(uint8_t* packet, uint16_t device_id, uint8_t command, uint8_t arg);
static void fut091_build_frame(uint8_t* tx12, uint16_t device_id, uint8_t group, uint8_t cmd, uint8_t arg);
static bool transmit_packet(const uint8_t* packet, size_t len);
static bool transmit_fut091_cmd(uint16_t device_id, uint8_t group, uint8_t cmd, uint8_t arg);
static void log_packet_hex(const char* prefix, const uint8_t* packet, size_t len);
static void nrf24_configure();

// ---------------------------------------------------------------------------
//  Init
// ---------------------------------------------------------------------------
bool milight_init(uint8_t ce_pin, uint8_t csn_pin, uint8_t sck_pin, uint8_t mosi_pin, uint8_t miso_pin) {
    uint8_t mac[6];
    WiFi.macAddress(mac);
    current_device_id = (mac[4] << 8) | mac[5];

    for (int i = 0; i < 4; i++) {
        channel_states[i].on          = false;
        channel_states[i].brightness  = 100;
        channel_states[i].color_temp  = 350;
        channel_states[i].channel     = i + 1;
        channel_states[i].protocol    = PROTOCOL_FUT091;
        channel_states[i].device_id   = current_device_id;
    }

#if MILIGHT_DRY_RUN
    Serial.println("[MILIGHT] DRY_RUN mode — no hardware init");
    initialized = true;
    Serial.printf("[MILIGHT] device_id=0x%04X\n", current_device_id);
    return true;
#else
    // SPI.begin() called ONCE — repeated calls can corrupt USB CDC on ESP32-S3.
    // We MUST use radio->begin(&SPI) — NOT radio->begin(), which resets the GPIO
    // Matrix to variant defaults (MISO=13, SCK=12), breaking our custom mapping.
    SPI.begin(sck_pin, miso_pin, mosi_pin, csn_pin);
    _spi_started = true;
    _ce_pin  = ce_pin;
    _csn_pin = csn_pin;
    delay(100);

    if (radio) { delete radio; }
    radio = new RF24(ce_pin, csn_pin);

    // Single attempt here — NRF24L01+ often needs >25 s from cold power-on
    // before MISO responds.  If it fails we keep 'radio' allocated and let
    // milight_send_state() do a lazy reinit when the first MQTT command
    // arrives (typically ~20-30 s after boot, by which time the module is
    // ready).  No blocking retry here so the rest of setup() is not delayed.
    if (radio->begin(&SPI)) {
        nrf24_configure();
        initialized = true;
        Serial.printf("[MILIGHT] Initialized — device_id=0x%04X\n", current_device_id);
        return true;
    }

    Serial.println("[MILIGHT] NRF24 not ready at setup — will retry on first command");
    return false;
#endif
}

// ---------------------------------------------------------------------------
//  NRF24 hardware configuration (called after begin() succeeds)
// ---------------------------------------------------------------------------
static void nrf24_configure() {
    radio->setAutoAck(false);
    radio->setDataRate(RF24_1MBPS);
    radio->setPALevel(RF24_PA_MAX);
    radio->setAddressWidth(5);
    radio->disableCRC();           // PL1167 layer handles CRC
    radio->setPayloadSize(FUT091_FRAME_SIZE);
    radio->openWritingPipe(MILIGHT_CCT_ADDRESS);
    radio->stopListening();
    radio->setChannel(MILIGHT_CHANNELS[0]);
}

// ---------------------------------------------------------------------------
//  CCT v1 packet builder (7 bytes, used for pair/unpair only)
// ---------------------------------------------------------------------------
static void cct_build_packet(uint8_t* packet, uint16_t device_id, uint8_t command, uint8_t arg) {
    packet[0] = (device_id >> 8) & 0xFF;
    packet[1] = device_id & 0xFF;
    packet[2] = 0x00;
    packet[3] = 0x00;
    packet[4] = command;
    packet[5] = arg;
    packet[6] = 0;
    for (int i = 0; i < 6; i++) packet[6] ^= packet[i];
}

// ---------------------------------------------------------------------------
//  FUT091 V2 frame builder — produces 12-byte ready-to-transmit frame
// ---------------------------------------------------------------------------
static void fut091_build_frame(uint8_t* tx12, uint16_t device_id, uint8_t group,
                               uint8_t cmd, uint8_t arg) {
    uint8_t p[9];
    p[0] = (uint8_t)esp_random();   // random key byte
    p[1] = FUT091_REQ_TYPE;         // 0x45
    p[2] = (device_id >> 8) & 0xFF;
    p[3] = device_id & 0xFF;
    p[4] = cmd;
    p[5] = arg;
    p[6] = sequence_num++;
    p[7] = group;
    p[8] = 0;
    v2Encode(p);

    uint8_t frame[10];
    frame[0] = 0x09;  // PL1167 length byte
    memcpy(frame + 1, p, 9);
    uint16_t crc = pl1167Crc(frame, 10);

    for (int i = 0; i < 10; i++) tx12[i] = bitReverse(frame[i]);
    tx12[10] = bitReverse(crc & 0xFF);
    tx12[11] = bitReverse((crc >> 8) & 0xFF);
}

// ---------------------------------------------------------------------------
//  Transmit
// ---------------------------------------------------------------------------
static bool transmit_packet(const uint8_t* packet, size_t len) {
#if MILIGHT_DRY_RUN
    return true;
#else
    if (!initialized || !radio) return false;

    bool success = false;
    for (int rep = 0; rep < MILIGHT_TX_REPS; rep++) {
        for (int i = 0; i < MILIGHT_RF_CHANNELS; i++) {
            radio->setChannel(MILIGHT_CHANNELS[i]);
            if (radio->write(packet, len)) success = true;
            delayMicroseconds(MILIGHT_TX_DELAY_US);
        }
    }
    Serial.printf("[MILIGHT] TX result: %s (isFifoFull=%d)\n",
                  success ? "OK" : "FAIL", radio->isFifo(true, true));
    return success;
#endif
}

// FUT091-specific transmit: rebuilds frame each repetition so sequence_num
// increments per-rep, preventing MiLight from deduplicating identical frames.
static bool transmit_fut091_cmd(uint16_t device_id, uint8_t group, uint8_t cmd, uint8_t arg) {
#if MILIGHT_DRY_RUN
    return true;
#else
    if (!initialized || !radio) return false;
    bool success = false;
    for (int rep = 0; rep < MILIGHT_TX_REPS_FUT091; rep++) {
        uint8_t tx[FUT091_FRAME_SIZE];
        fut091_build_frame(tx, device_id, group, cmd, arg);
        for (int ch = 0; ch < MILIGHT_RF_CHANNELS; ch++) {
            radio->setChannel(MILIGHT_CHANNELS[ch]);
            if (radio->write(tx, FUT091_FRAME_SIZE)) success = true;
            delayMicroseconds(MILIGHT_TX_DELAY_US);
        }
    }
    Serial.printf("[MILIGHT] FUT091 TX cmd=0x%02X result: %s\n", cmd, success ? "OK" : "FAIL");
    return success;
#endif
}

static void log_packet_hex(const char* prefix, const uint8_t* packet, size_t len) {
    Serial.print(prefix);
    for (size_t i = 0; i < len; i++) {
        if (i > 0) Serial.print(" ");
        if (packet[i] < 0x10) Serial.print("0");
        Serial.print(packet[i], HEX);
    }
    Serial.println();
}

// ---------------------------------------------------------------------------
//  Send state
// ---------------------------------------------------------------------------
bool milight_send_state(const MiLightState& state) {
    // Always persist the state so GUI and status publishing work even when
    // the NRF24 hardware is absent.
    if (state.channel >= 1 && state.channel <= 4)
        channel_states[state.channel - 1] = state;

    if (!initialized) {
#if !MILIGHT_DRY_RUN
        // Lazy reinit: NRF24 may have settled since setup() failed.
        // Throttle to once every 30 s to avoid log spam.
        unsigned long now = millis();
        if (_spi_started && radio && (now - _last_reinit_ms >= 30000UL)) {
            _last_reinit_ms = now;
            Serial.println("[MILIGHT] Lazy reinit attempt...");
            if (radio->begin(&SPI)) {
                nrf24_configure();
                initialized = true;
                Serial.printf("[MILIGHT] Lazy reinit OK — device_id=0x%04X\n", current_device_id);
            } else {
                Serial.println("[MILIGHT] Lazy reinit failed — NRF24 still not ready");
            }
        }
#endif
        if (!initialized) {
            Serial.println("[MILIGHT] ERROR: not initialized");
            return false;
        }
    }

    uint16_t device_id = (state.device_id != 0) ? state.device_id : current_device_id;

    if (state.protocol == PROTOCOL_CCT) {
        uint8_t packet[CCT_PACKET_SIZE];

        uint8_t on_off_cmd = 0;
        if      (state.channel == 1) on_off_cmd = state.on ? CCT_GROUP_1_ON : CCT_GROUP_1_OFF;
        else if (state.channel == 2) on_off_cmd = state.on ? CCT_GROUP_2_ON : CCT_GROUP_2_OFF;
        else if (state.channel == 3) on_off_cmd = state.on ? CCT_GROUP_3_ON : CCT_GROUP_3_OFF;
        else if (state.channel == 4) on_off_cmd = state.on ? CCT_GROUP_4_ON : CCT_GROUP_4_OFF;

        cct_build_packet(packet, device_id, on_off_cmd, 0x00);
        log_packet_hex("[MILIGHT] CCT ON/OFF: ", packet, CCT_PACKET_SIZE);
        transmit_packet(packet, CCT_PACKET_SIZE);
        ml_delay(100);

        if (state.on) {
            uint8_t steps = (state.brightness * 10) / 100;
            for (uint8_t i = 0; i < steps; i++) {
                cct_build_packet(packet, device_id, CCT_BRIGHTNESS_UP, 0x00);
                transmit_packet(packet, CCT_PACKET_SIZE);
                ml_delay(50);
            }
            if (state.color_temp < 300) {
                uint8_t s = (300 - state.color_temp) / 20;
                for (uint8_t i = 0; i < s; i++) {
                    cct_build_packet(packet, device_id, CCT_TEMPERATURE_DOWN, 0x00);
                    transmit_packet(packet, CCT_PACKET_SIZE);
                    ml_delay(50);
                }
            } else if (state.color_temp > 400) {
                uint8_t s = (state.color_temp - 400) / 20;
                for (uint8_t i = 0; i < s; i++) {
                    cct_build_packet(packet, device_id, CCT_TEMPERATURE_UP, 0x00);
                    transmit_packet(packet, CCT_PACKET_SIZE);
                    ml_delay(50);
                }
            }
        }
        return true;

    } else if (state.protocol == PROTOCOL_FUT091) {
        uint8_t cmd = state.on ? FUT091_CMD_ON : FUT091_CMD_OFF;

        transmit_fut091_cmd(device_id, state.channel, cmd, state.channel);
        ml_delay(100);

        if (state.on) {
            transmit_fut091_cmd(device_id, state.channel, FUT091_CMD_BRIGHT, state.brightness);
            ml_delay(100);

            // FUT091 0x19 arg: 0=warm, 100=cool — verified empirically against the
            // user's MIBO bulbs after the test-code labels were found to be inverted.
            // The MIBO CCT bulbs are spec'd at 2700-6500 K = 153..370 mireds, so we
            // clamp + map across that range. Inputs outside (e.g. HA's default
            // 153..500 range) simply saturate to the closest endpoint.
            uint16_t ct = state.color_temp;
            if (ct < 153) ct = 153;
            if (ct > 370) ct = 370;
            uint8_t temp_val = (uint8_t)map(ct, 153, 370, 100, 0);
            transmit_fut091_cmd(device_id, state.channel, FUT091_CMD_TEMP, temp_val);
        }
        return true;
    }

    return false;
}

// ---------------------------------------------------------------------------
//  Pair / Unpair
// ---------------------------------------------------------------------------
bool milight_pair(uint8_t channel, MiLightProtocol protocol, uint16_t device_id) {
    if (!initialized) return false;
    if (device_id == 0) device_id = current_device_id;

    Serial.printf("[MILIGHT] Pairing ch=%d device_id=0x%04X proto=%s\n",
                  channel, device_id, milight_protocol_to_string(protocol));

    if (protocol == PROTOCOL_CCT) {
        uint8_t packet[CCT_PACKET_SIZE];
        uint8_t on_cmd = (channel == 2) ? CCT_GROUP_2_ON :
                         (channel == 3) ? CCT_GROUP_3_ON :
                         (channel == 4) ? CCT_GROUP_4_ON : CCT_GROUP_1_ON;
        for (int i = 0; i < 5; i++) {
            cct_build_packet(packet, device_id, on_cmd, 0x00);
            log_packet_hex("[MILIGHT] CCT PAIR: ", packet, CCT_PACKET_SIZE);
            transmit_packet(packet, CCT_PACKET_SIZE);
            ml_delay(200);
        }
    } else if (protocol == PROTOCOL_FUT091) {
        uint8_t tx[FUT091_FRAME_SIZE];
        for (int i = 0; i < 5; i++) {
            fut091_build_frame(tx, device_id, channel, FUT091_CMD_ON, channel);
            log_packet_hex("[MILIGHT] FUT091 PAIR: ", tx, FUT091_FRAME_SIZE);
            transmit_packet(tx, FUT091_FRAME_SIZE);
            ml_delay(200);
        }
    }
    return true;
}

bool milight_unpair(uint8_t channel, MiLightProtocol protocol) {
    if (!initialized) return false;

    Serial.printf("[MILIGHT] Unpairing ch=%d proto=%s\n",
                  channel, milight_protocol_to_string(protocol));

    if (protocol == PROTOCOL_CCT) {
        uint8_t packet[CCT_PACKET_SIZE];
        uint8_t off_cmd = (channel == 2) ? CCT_GROUP_2_OFF :
                          (channel == 3) ? CCT_GROUP_3_OFF :
                          (channel == 4) ? CCT_GROUP_4_OFF : CCT_GROUP_1_OFF;
        for (int i = 0; i < 5; i++) {
            cct_build_packet(packet, current_device_id, off_cmd, 0x00);
            log_packet_hex("[MILIGHT] CCT UNPAIR: ", packet, CCT_PACKET_SIZE);
            transmit_packet(packet, CCT_PACKET_SIZE);
            ml_delay(200);
        }
    } else if (protocol == PROTOCOL_FUT091) {
        uint8_t tx[FUT091_FRAME_SIZE];
        for (int i = 0; i < 5; i++) {
            fut091_build_frame(tx, current_device_id, channel, FUT091_CMD_OFF, channel);
            log_packet_hex("[MILIGHT] FUT091 UNPAIR: ", tx, FUT091_FRAME_SIZE);
            transmit_packet(tx, FUT091_FRAME_SIZE);
            ml_delay(200);
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
//  Accessors
// ---------------------------------------------------------------------------
MiLightState& milight_get_state(uint8_t channel) {
    if (channel >= 1 && channel <= 4) return channel_states[channel - 1];
    return channel_states[0];
}

void milight_set_device_id(uint16_t device_id) {
    if (device_id == 0) {
        uint8_t mac[6];
        WiFi.macAddress(mac);
        device_id = (mac[4] << 8) | mac[5];
    }
    current_device_id = device_id;
    for (int i = 0; i < 4; i++) channel_states[i].device_id = device_id;
    Serial.printf("[MILIGHT] device_id set to 0x%04X\n", device_id);
}

uint16_t milight_get_device_id() {
    return current_device_id;
}

MiLightProtocol milight_protocol_from_string(const char* name) {
    if (strcasecmp(name, "fut091") == 0) return PROTOCOL_FUT091;
    return PROTOCOL_CCT;
}

const char* milight_protocol_to_string(MiLightProtocol protocol) {
    switch (protocol) {
        case PROTOCOL_FUT091: return "fut091";
        case PROTOCOL_CCT:
        default:              return "cct";
    }
}

#endif // MILIGHT_ENABLED
