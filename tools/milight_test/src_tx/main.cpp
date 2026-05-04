// MiLight CCT V2 (FUT091) impersonator / TX testbench.
//
// Three modes (compile-time):
//   TX_MODE_ENCODED12  — generate V2-encoded packets with a real device_id
//                        (default — this is what impersonates the remote)
//   TX_MODE_REPLAY12   — retransmit captured 10-byte payloads with a freshly
//                        computed PL1167 CRC (useful for "is the radio path
//                        identical to the real remote?" checks)
//   TX_MODE_REPLAY10   — emit the 10-byte payload with NO CRC, padded by the
//                        nRF24 to fixed payload size. Receiver will reject
//                        these; only useful to confirm the sniffer sees TX.
//
// Defaults to ENCODED12 with DEVICE_ID = 0x0532 (the remote captured in
// REPLAY1/2). Cycles GRP3 ON/OFF/BRIGHT/TEMP every few seconds so the
// receiver's reaction is visible.

#include <Arduino.h>
#include <RF24.h>
#include <SPI.h>

#define PIN_CE   22
#define PIN_CSN  21
#define PIN_SCK  18
#define PIN_MOSI 23
#define PIN_MISO 19

// ---- V2 encoding (matches sidoh/esp8266_milight_hub V2RFEncoding.cpp) ----
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

// PL1167 CRC-16 over un-bit-reversed bytes. The bytes passed here MUST be
// what the PL1167 chip would see (length + payload), NOT the nRF24-wire
// bit-reversed bytes.
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

// V2 encode: p[0]=key byte (unchanged), p[1..7] encoded, p[8]=checksum
static void v2Encode(uint8_t *p) {
  uint8_t xk  = v2XorKey(p[0]);
  uint8_t sum = xk;
  for (int i = 1; i <= 7; i++) {
    sum  += p[i];
    p[i]  = (p[i] ^ xk) + v2Off(i, p[0], V2_OFFSET_JUMP_START);
  }
  p[8] = ((sum + 2) ^ xk) + V2_OFFSETS[7][p[0] % 4];
}

// ---- MiLight CCT radio config ----
static const uint8_t CCT_ADDRESS[5]  = {0xAA, 0x5A, 0x05, 0x0A, 0x55};
// {6, 41, 76} — esp8266_milight_hub's MiLightRadioConfig stores {4,39,74}
// and applies +2 in PL1167_nRF24::recalc_parameters before writing RF_CH,
// so the actual on-air channels are 2 MHz higher than the config values.
static const uint8_t CCT_CHANNELS[3] = {6, 41, 76};
static const uint8_t REQ_TYPE        = 0x45;

#ifndef DEVICE_ID_HI
#define DEVICE_ID_HI 0x05
#endif
#ifndef DEVICE_ID_LO
#define DEVICE_ID_LO 0x32
#endif

#ifndef TX_MODE_REPLAY12
#define TX_MODE_REPLAY12 0
#endif
#ifndef TX_MODE_REPLAY10
#define TX_MODE_REPLAY10 0
#endif
#ifndef TX_MODE_ENCODED12
// Default: act as a remote with the captured device_id.
#define TX_MODE_ENCODED12 (!(TX_MODE_REPLAY12 || TX_MODE_REPLAY10))
#endif

#ifndef TX_REPS
#define TX_REPS 8
#endif
#ifndef TX_DELAY_US
#define TX_DELAY_US 400
#endif
#ifndef TX_SCAN_ALL_CHANNELS
#define TX_SCAN_ALL_CHANNELS 0
#endif
#ifndef TX_FIXED_CHANNEL
#define TX_FIXED_CHANNEL 4
#endif
#ifndef TX_GROUP
#define TX_GROUP 3
#endif

// Captured 10-byte (bit-reversed, no-CRC) payloads from the real remote.
// REPLAY12 mode uses these and computes the missing PL1167 CRC.
static const uint8_t REPLAY1[10] = {0x90, 0x20, 0x2C, 0x5B, 0xA0, 0xE6, 0xC3, 0xF9, 0x1A, 0x3C};
static const uint8_t REPLAY2[10] = {0x90, 0x73, 0x9D, 0x13, 0x3A, 0xF0, 0xCB, 0x22, 0x03, 0x81};

static RF24 radio(PIN_CE, PIN_CSN);
static uint8_t seq = 100;     // incremented per encoded TX
static uint8_t key_byte = 0;  // randomized per encoded TX

// Build a 12-byte transmit frame from a captured 10-byte (already bit-reversed)
// payload by computing the correct PL1167 CRC and bit-reversing it for nRF24.
//
// CRITICAL: the CRC must be computed on the *un-bit-reversed* PL1167 bytes,
// not on the captured (bit-reversed) ones. Earlier versions of this file did
// the wrong thing and every replayed packet had an invalid CRC.
static void buildReplayPacket(const uint8_t *raw10_air, uint8_t *tx12, uint16_t *crcOut = nullptr) {
  uint8_t orig[10];
  for (int i = 0; i < 10; i++) orig[i] = bitReverse(raw10_air[i]);
  uint16_t crc = pl1167Crc(orig, 10);

  // Bytes 0..9 go on the air bit-reversed (= same as captured).
  for (int i = 0; i < 10; i++) tx12[i] = raw10_air[i];
  // Append CRC bit-reversed for nRF24.
  tx12[10] = bitReverse(crc & 0xFF);
  tx12[11] = bitReverse((crc >> 8) & 0xFF);
  if (crcOut) *crcOut = crc;
}

// In real captured remote packets the V2 key byte (p[0]) and sequence
// byte (p[6]) are independent values — the key looks random, the sequence
// increments per button press. Generate them the same way so receivers
// that dedupe on the sequence field accept successive commands.
static void buildEncodedPacket(uint8_t cmd, uint8_t arg, uint8_t group,
                               uint8_t key, uint8_t sequence, uint8_t *tx12,
                               uint16_t *crcOut = nullptr) {
  uint8_t p[9];
  p[0] = key;
  p[1] = REQ_TYPE;
  p[2] = DEVICE_ID_HI;
  p[3] = DEVICE_ID_LO;
  p[4] = cmd;
  p[5] = arg;
  p[6] = sequence;
  p[7] = group;
  p[8] = 0;
  v2Encode(p);

  uint8_t frame[10];
  frame[0] = 0x09;                       // PL1167 length byte
  memcpy(frame + 1, p, 9);
  uint16_t crc = pl1167Crc(frame, 10);

  for (int i = 0; i < 10; i++) tx12[i] = bitReverse(frame[i]);
  tx12[10] = bitReverse(crc & 0xFF);
  tx12[11] = bitReverse((crc >> 8) & 0xFF);
  if (crcOut) *crcOut = crc;
}

static void transmitOnAllChannels(const uint8_t *tx, uint8_t len, const char *label) {
  for (int rep = 0; rep < TX_REPS; rep++) {
    for (int ch = 0; ch < 3; ch++) {
      radio.setChannel(CCT_CHANNELS[ch]);
      bool ok = radio.write(tx, len);
      if (rep == 0) {
        Serial.printf("%s rep=%d ch=%d write=%s\n", label, rep + 1, CCT_CHANNELS[ch], ok ? "OK" : "FAIL");
      }
      delayMicroseconds(TX_DELAY_US);
    }
  }
}

static void transmitOnSingleChannel(const uint8_t *tx, uint8_t len, uint8_t channel,
                                    int reps, const char *label) {
  for (int rep = 0; rep < reps; rep++) {
    radio.setChannel(channel);
    bool ok = radio.write(tx, len);
    Serial.printf("%s rep=%d ch=%d write=%s\n", label, rep + 1, channel, ok ? "OK" : "FAIL");
    delay(80);
  }
}

static void printHex(const uint8_t *buf, size_t len) {
  for (size_t i = 0; i < len; i++) {
    if (buf[i] < 0x10) Serial.print('0');
    Serial.print(buf[i], HEX);
    if (i + 1 < len) Serial.print(' ');
  }
}

static void sendEncoded(uint8_t cmd, uint8_t arg, uint8_t group, const char *name) {
  uint8_t tx[12];
  uint16_t crc = 0;
  key_byte = (uint8_t)esp_random();
  uint8_t this_seq = seq;
  buildEncodedPacket(cmd, arg, group, key_byte, this_seq, tx, &crc);

  Serial.printf("ENC %s cmd=0x%02X arg=%u grp=%u key=%02X seq=%u id=%02X%02X CRC=%04X tx12: ",
                name, cmd, arg, group, key_byte, this_seq,
                DEVICE_ID_HI, DEVICE_ID_LO, crc);
  printHex(tx, 12);
  Serial.println();

  transmitOnAllChannels(tx, 12, "ENC");
  seq++;
}

static void sendReplay(const uint8_t *raw10, const char *label) {
#if TX_MODE_REPLAY10
  // RAW10: send captured bytes verbatim — no CRC. Only useful to test if
  // the sniffer sees our TX; receivers will reject.
  uint8_t tx[10];
  memcpy(tx, raw10, 10);
  Serial.printf("%s raw10: ", label);
  printHex(tx, 10);
  Serial.println();
  transmitOnAllChannels(tx, 10, label);
#else
  uint8_t tx[12];
  uint16_t crc = 0;
  buildReplayPacket(raw10, tx, &crc);
  Serial.printf("%s tx12 (CRC=%04X): ", label, crc);
  printHex(tx, 12);
  Serial.println();
  transmitOnAllChannels(tx, 12, label);
#endif
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== MiLight CCT V2 TX testbench ===");
#if TX_MODE_REPLAY10
  Serial.println("Mode: REPLAY10 (raw 10 bytes, NO CRC — sniffer test only)");
#elif TX_MODE_REPLAY12
  Serial.println("Mode: REPLAY12 (captured 10 bytes + computed PL1167 CRC)");
#else
  Serial.println("Mode: ENCODED12 (V2-encoded with device_id — impersonates remote)");
#endif
  Serial.printf("device_id=%02X%02X group=%u TX_REPS=%d TX_DELAY_US=%d\n",
                DEVICE_ID_HI, DEVICE_ID_LO, TX_GROUP, TX_REPS, TX_DELAY_US);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CSN);
  if (!radio.begin()) {
    Serial.println("FAIL: NRF24 not responding");
    while (1) delay(1000);
  }
  Serial.printf("isChipConnected: %s\n", radio.isChipConnected() ? "YES" : "NO");

  radio.setAutoAck(false);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setAddressWidth(5);
#if TX_MODE_REPLAY10
  radio.setPayloadSize(10);
#else
  radio.setPayloadSize(12);
#endif
  radio.disableCRC();
  radio.openWritingPipe(CCT_ADDRESS);
  radio.stopListening();

  Serial.println("Starting in 3s...");
  for (int i = 3; i > 0; i--) { Serial.printf("%d ", i); delay(1000); }
  Serial.println();
}

void loop() {
#if TX_MODE_ENCODED12
  // V2 (FUT091/MIBO) command set, verified empirically against the user's
  // bulbs (group 3) — earlier labels for ON/OFF were inverted:
  //   cmd 0x11 GRP-ON,  arg = group (1..4)
  //   cmd 0x12 GRP-OFF, arg = group (1..4)
  //   cmd 0x1C BRIGHT,  arg = absolute brightness 0..100
  //   cmd 0x19 TEMP,    arg = absolute color temp 0..100 (0=cool, 100=warm)
  sendEncoded(0x11, TX_GROUP, TX_GROUP, "GRP-ON");
  delay(2000);

  sendEncoded(0x1C, 80, TX_GROUP, "BRIGHT=80");
  delay(1500);
  sendEncoded(0x1C, 20, TX_GROUP, "BRIGHT=20");
  delay(1500);

  sendEncoded(0x19, 100, TX_GROUP, "TEMP=warm");
  delay(1500);
  sendEncoded(0x19, 0, TX_GROUP, "TEMP=cool");
  delay(1500);

  sendEncoded(0x12, TX_GROUP, TX_GROUP, "GRP-OFF");
  delay(2500);

#elif TX_SCAN_ALL_CHANNELS
  uint8_t tx[12];
  uint16_t crc;
  buildReplayPacket(REPLAY1, tx, &crc);
  Serial.printf("=== R1 channel scan (CRC=%04X) ===\n", crc);
  for (int channel = 0; channel <= 125; channel++) {
    transmitOnSingleChannel(tx, 12, (uint8_t)channel, 3, "R1");
  }
  delay(1500);

#else
  sendReplay(REPLAY1, "R1");
  delay(300);
  sendReplay(REPLAY2, "R2");
  delay(1500);
#endif
}
