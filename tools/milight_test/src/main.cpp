#include <Arduino.h>
#include <RF24.h>
#include <SPI.h>

// --- NRF24 pins ---
#define PIN_CE   22
#define PIN_CSN  21
#define PIN_SCK  18
#define PIN_MOSI 23
#define PIN_MISO 19

// ---- MiLight V2 RF Encoding ----
// From esp8266_milight_hub V2RFEncoding.cpp
#define V2_OFFSET_JUMP_START 0x54

static const uint8_t V2_OFFSETS[8][4] = {
  {0x45, 0x1F, 0x14, 0x5C}, // byte1: request type
  {0x2B, 0xC9, 0xE3, 0x11}, // byte2: id1
  {0x6D, 0x5F, 0x8A, 0x2B}, // byte3: id2
  {0xAF, 0x03, 0x1D, 0xF3}, // byte4: command
  {0x1A, 0xE2, 0xF0, 0xD1}, // byte5: argument
  {0x04, 0xD8, 0x71, 0x42}, // byte6: sequence
  {0xAF, 0x04, 0xDD, 0x07}, // byte7: group
  {0x61, 0x13, 0x38, 0x64}, // byte8: checksum
};

// Returns offset for byte index (1-based) given key and optional jump start
static uint8_t v2Offset(uint8_t byteIdx, uint8_t key, uint8_t jumpStart) {
  uint8_t base = V2_OFFSETS[byteIdx - 1][key % 4];
  if (jumpStart > 0 && key >= jumpStart && key < jumpStart + 0x80) {
    base += 0x80;
  }
  return base;
}

static uint8_t v2XorKey(uint8_t key) {
  const uint8_t shift = (key & 0x0F) < 0x04 ? 0 : 1;
  const uint8_t x = (((key & 0xF0) >> 4) + shift + 6) % 8;
  const uint8_t msn = (((4 + x) ^ 1) & 0x0F) << 4;
  const uint8_t lsn = ((((key & 0x0F) + 4) ^ 2) & 0x0F);
  return msn | lsn;
}

// Decode packet in-place (bytes 1-8, byte 0 is the key byte)
static void v2Decode(uint8_t *p) {
  uint8_t key = v2XorKey(p[0]);
  for (int i = 1; i <= 8; i++) {
    uint8_t off = v2Offset(i, p[0], V2_OFFSET_JUMP_START);
    p[i] = (p[i] - off) ^ key;
  }
}
// ----------------------------------

// CCT syncword address (5 bytes, same as transmit side)
// Derived from syncword0=0x050A, syncword3=0x55AA, preamble=0xAA, trailer=0x05
static const uint8_t CCT_ADDRESS[5] = {0xAA, 0x5A, 0x05, 0x0A, 0x55};

// CCT channels (2.404, 2.439, 2.474 GHz)
static const uint8_t CCT_CHANNELS[3] = {4, 39, 74};

static RF24 radio(PIN_CE, PIN_CSN);
static uint8_t cur_ch = 0;
static uint32_t last_channel_switch = 0;
static uint32_t pkts_received = 0;
static uint8_t last_raw[10] = {0};
static uint32_t dup_count = 0;
static uint32_t unique_count = 0;

static uint8_t reverseBits(uint8_t b) {
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("=== MiLight CCT SNIFFER ===");
    Serial.println("Paina kaukosaatimen nappeja. Kaapaan kaikki paketit.");
    Serial.println("Kanavat: 4, 39, 74  |  Osoite: AA 5A 05 0A 55");

    SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CSN);

    if (!radio.begin()) {
        Serial.println("FAIL: NRF24 ei vastaa - tarkista kytkentä!");
        while (1) delay(1000);
    }

    Serial.printf("isChipConnected: %s\n", radio.isChipConnected() ? "YES" : "NO");
    Serial.printf("isPVariant (nRF24L01+): %s\n", radio.isPVariant() ? "YES" : "NO");

    radio.setAutoAck(false);
    radio.setDataRate(RF24_1MBPS);
    radio.setPALevel(RF24_PA_MAX);
    radio.setAddressWidth(5);
    radio.setPayloadSize(10);       // 1 length byte + 9 payload bytes (actual data from remote)
    radio.disableCRC();             // CRC handled in PL1167 wrapper (already stripped by HW)
    radio.openReadingPipe(1, CCT_ADDRESS);
    radio.setChannel(CCT_CHANNELS[cur_ch]);
    radio.startListening();

    Serial.printf("Kuunnellaan kanavaa %d...\n", CCT_CHANNELS[cur_ch]);
    last_channel_switch = millis();
}

void loop() {
    // Switch channel every 50ms (fixed, independent of packet reception)
    if (millis() - last_channel_switch > 50) {
        cur_ch = (cur_ch + 1) % 3;
        radio.setChannel(CCT_CHANNELS[cur_ch]);
        last_channel_switch = millis();
    }

    if (radio.available()) {
        uint8_t raw[10] = {0};
        radio.read(raw, 10);
        pkts_received++;

        // Duplicate filter - skip if same as last packet
        if (memcmp(raw, last_raw, 10) == 0) {
            dup_count++;
            return;  // same packet, skip
        }
        memcpy(last_raw, raw, 10);
        dup_count = 0;
        unique_count++;

        // First byte is NRF24 length marker
        uint8_t len_byte = reverseBits(raw[0]);
        
        if (len_byte != 0x09) {
            // Not a valid 9-byte CCT packet
            Serial.printf("[CH%2d] #%lu LEN: 0x%02X (expected 0x09) | ",
                          CCT_CHANNELS[cur_ch], pkts_received, len_byte);
            for (int i = 0; i < 10; i++) Serial.printf("%02X", raw[i]);
            Serial.println();
            return;  // Skip this packet
        }

        // Bit-reverse the 9 payload bytes (NRF24+PL1167 layer)
        uint8_t p[9];
        for (int i = 0; i < 9; i++) p[i] = reverseBits(raw[i + 1]);

        // Apply V2 RF decoding (MiLight application layer)
        v2Decode(p);

        // After V2 decode:
        // p[0] = raw key byte (NOT decoded, used as V2 seed)
        // p[1] = request type  (0x5A = CCT V2 command)
        // p[2] = device ID high
        // p[3] = device ID low
        // p[4] = command
        // p[5] = argument/value
        // p[6] = sequence counter
        // p[7] = group/zone (0=all, 1-4=groups)
        // p[8] = checksum
        uint8_t key_byte = p[0];
        uint8_t req_type = p[1];
        uint8_t id1      = p[2];
        uint8_t id2      = p[3];
        uint8_t command  = p[4];
        uint8_t argument = p[5];
        uint8_t sequence = p[6];
        uint8_t group    = p[7];
        uint8_t chk_recv = p[8];

        // Checksum validate: sum of decoded bytes 0-6 should equal checksum
        uint8_t chk_calc = v2XorKey(p[0]);
        for (int i = 1; i <= 7; i++) chk_calc += p[i];
        // (V2 encodeV2Packet adds sum+2 for final byte — we just report raw)

        const char* action = "?";
        switch (command) {
            // Confirmed from captures (type=0x45, id=0532, remote model 42373 CCT):
            case 0x11: action = "GRP-ON";  break;  // arg = group number (toggle state A)
            case 0x12: action = "GRP-ON";  break;  // arg = group number (toggle state B)
            case 0x1C: action = "BRIGHT";  break;  // arg = 0-100 absolute brightness
            case 0x19: action = "TEMP";    break;  // arg = 0-100 absolute color temp
        }

        // Print raw bytes (for replay test)
        Serial.printf("RAW: ");
        for (int i = 0; i < 10; i++) Serial.printf("%02X ", raw[i]);
        Serial.printf("| [CH%2d] #%lu | type=0x%02X id=%02X%02X grp=%d cmd=0x%02X(%s) arg=%d seq=%d\n",
                      CCT_CHANNELS[cur_ch], unique_count,
                      req_type, id1, id2, group, command, action, argument, sequence);
    }
}
