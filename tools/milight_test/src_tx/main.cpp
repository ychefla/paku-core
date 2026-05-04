#include <Arduino.h>
#include <RF24.h>
#include <SPI.h>

#define PIN_CE   22
#define PIN_CSN  21
#define PIN_SCK  18
#define PIN_MOSI 23
#define PIN_MISO 19

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
  return ((((4 + x) ^ 1) & 0x0F) << 4) | (((key & 0x0F) + 4) ^ 2) & 0x0F;
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
  // Checksum byte (index 8, jumpStart=0 per reference impl)
  p[8] = ((sum + 2) ^ xk) + V2_OFFSETS[7][p[0] % 4];
}

static const uint8_t CCT_ADDRESS[5]  = {0xAA, 0x5A, 0x05, 0x0A, 0x55};
static const uint8_t CCT_CHANNELS[3] = {4, 39, 74};
static const uint8_t DEVICE_ID_HI    = 0x05;
static const uint8_t DEVICE_ID_LO    = 0x32;
static const uint8_t REQ_TYPE        = 0x45;

#ifndef TX_MODE_REPLAY12
#define TX_MODE_REPLAY12 1
#endif

#ifndef TX_MODE_REPLAY10
#define TX_MODE_REPLAY10 0
#endif

#ifndef TX_MODE_ENCODED12
#define TX_MODE_ENCODED12 0
#endif

#ifndef TX_REPS
#define TX_REPS 8
#endif

#ifndef TX_SCAN_ALL_CHANNELS
#define TX_SCAN_ALL_CHANNELS 1
#endif

#ifndef TX_FIXED_CHANNEL
#define TX_FIXED_CHANNEL 4
#endif

#ifndef TX_DELAY_US
#define TX_DELAY_US 400
#endif

// Exact raw bytes captured from the real remote (sniffer RAW: output)
// Packet 1: cmd=0x12(GRP-ON) grp=3 seq=49
static const uint8_t REPLAY1[10] = {0x90, 0x20, 0x2C, 0x5B, 0xA0, 0xE6, 0xC3, 0xF9, 0x1A, 0x3C};
// Packet 2: cmd=0x12(GRP-ON) grp=3 seq=51
static const uint8_t REPLAY2[10] = {0x90, 0x73, 0x9D, 0x13, 0x3A, 0xF0, 0xCB, 0x22, 0x03, 0x81};

static RF24 radio(PIN_CE, PIN_CSN);
static uint8_t seq = 100;

static void printReplayMetadata(const char *label, uint8_t seqNum) {
  Serial.printf(
    "%s meta: type=0x%02X id=%02X%02X grp=3 cmd=0x12(GRP-ON) arg=3 seq=%u\n",
    label,
    REQ_TYPE,
    DEVICE_ID_HI,
    DEVICE_ID_LO,
    seqNum
  );
}

static void transmit12(const uint8_t *tx, const char *label) {
  for (int rep = 0; rep < TX_REPS; rep++) {
    for (int ch = 0; ch < 3; ch++) {
      radio.setChannel(CCT_CHANNELS[ch]);
      bool ok = radio.write(tx, 12);
      Serial.printf("%s rep=%d ch=%d write=%s\n", label, rep + 1, CCT_CHANNELS[ch], ok ? "OK" : "FAIL");
      delayMicroseconds(TX_DELAY_US);
    }
  }
}

static void transmit10(const uint8_t *tx, const char *label) {
  for (int rep = 0; rep < TX_REPS; rep++) {
    for (int ch = 0; ch < 3; ch++) {
      radio.setChannel(CCT_CHANNELS[ch]);
      bool ok = radio.write(tx, 10);
      Serial.printf("%s rep=%d ch=%d write=%s\n", label, rep + 1, CCT_CHANNELS[ch], ok ? "OK" : "FAIL");
      delayMicroseconds(TX_DELAY_US);
    }
  }
}

static void transmit12SingleChannel(const uint8_t *tx, const char *label, uint8_t channel, int reps) {
  for (int rep = 0; rep < reps; rep++) {
    radio.setChannel(channel);
    bool ok = radio.write(tx, 12);
    Serial.printf("%s rep=%d ch=%d write=%s\n", label, rep + 1, channel, ok ? "OK" : "FAIL");
    delay(80);
  }
}

static void buildReplayPacket(const uint8_t *raw10, uint8_t *tx12, uint16_t *crcOut = nullptr) {
  uint16_t crc = pl1167Crc(raw10, 10);
  memcpy(tx12, raw10, 10);
  tx12[10] = bitReverse(crc & 0xFF);
  tx12[11] = bitReverse((crc >> 8) & 0xFF);
  if (crcOut) *crcOut = crc;
}

static void sendReplayPacket(const uint8_t *raw10, const char *label) {
#if TX_MODE_REPLAY10
  Serial.printf("%s raw10: ", label);
  for (int i = 0; i < 10; i++) Serial.printf("%02X ", raw10[i]);
  Serial.println();

  transmit10(raw10, label);
#else
  uint8_t tx[12];
  uint16_t crc = 0;
  buildReplayPacket(raw10, tx, &crc);

  Serial.printf("%s raw10: ", label);
  for (int i = 0; i < 10; i++) Serial.printf("%02X ", raw10[i]);
  Serial.println();

  Serial.printf("%s tx12:  ", label);
  for (int i = 0; i < 12; i++) Serial.printf("%02X ", tx[i]);
  Serial.printf("(CRC=%04X)\n", crc);

  transmit12(tx, label);
#endif
}


static void scanAllChannels(const uint8_t *raw10, const char *label, int repsPerChannel) {
#if TX_MODE_REPLAY10
  if (label[0] == 'R' && label[1] == '1') printReplayMetadata(label, 49);
  if (label[0] == 'R' && label[1] == '2') printReplayMetadata(label, 51);
  Serial.printf("=== %s channel scan start (raw10, reps=%d) ===\n", label, repsPerChannel);
  for (int channel = 0; channel <= 125; channel++) {
    Serial.printf("%s scan channel=%d\n", label, channel);
    for (int rep = 0; rep < repsPerChannel; rep++) {
      radio.setChannel(static_cast<uint8_t>(channel));
      bool ok = radio.write(raw10, 10);
      Serial.printf("%s rep=%d ch=%d write=%s\n", label, rep + 1, channel, ok ? "OK" : "FAIL");
      delay(80);
    }
  }
  Serial.printf("=== %s channel scan done ===\n", label);
#else
  uint8_t tx[12];
  uint16_t crc = 0;
  buildReplayPacket(raw10, tx, &crc);

  if (label[0] == 'R' && label[1] == '1') printReplayMetadata(label, 49);
  if (label[0] == 'R' && label[1] == '2') printReplayMetadata(label, 51);
  Serial.printf("=== %s channel scan start (CRC=%04X, reps=%d) ===\n", label, crc, repsPerChannel);
  for (int channel = 0; channel <= 125; channel++) {
    Serial.printf("%s scan channel=%d\n", label, channel);
    transmit12SingleChannel(tx, label, static_cast<uint8_t>(channel), repsPerChannel);
  }
  Serial.printf("=== %s channel scan done ===\n", label);
#endif
}

static void sendPacket(uint8_t cmd, uint8_t arg, uint8_t group, bool verbose = false) {
  uint8_t p[9];
  p[0] = seq;        // key byte (also used as sequence)
  p[1] = REQ_TYPE;
  p[2] = DEVICE_ID_HI;
  p[3] = DEVICE_ID_LO;
  p[4] = cmd;
  p[5] = arg;
  p[6] = seq;        // sequence counter
  p[7] = group;
  p[8] = 0;
  v2Encode(p);

  uint8_t frame[10];
  frame[0] = 0x09;
  memcpy(frame + 1, p, 9);

  uint16_t crc = pl1167Crc(frame, 10);

  uint8_t tx[12];
  for (int i = 0; i < 10; i++) tx[i] = bitReverse(frame[i]);
  tx[10] = bitReverse(crc & 0xFF);
  tx[11] = bitReverse((crc >> 8) & 0xFF);

  if (verbose) {
    Serial.printf("  raw[12]: ");
    for (int i = 0; i < 12; i++) Serial.printf("%02X ", tx[i]);
    Serial.printf("(CRC=%04X)\n", crc);
  }

  transmit12(tx, "ENC");
  Serial.printf("TX cmd=0x%02X arg=%d grp=%d seq=%d\n", cmd, arg, group, seq);
  seq++;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== MiLight CCT TX testbench ===");
#if TX_MODE_REPLAY10
  Serial.println("Mode: TX_MODE_REPLAY10 (captured 10-byte raw payload)");
#elif TX_MODE_ENCODED12
  Serial.println("Mode: TX_MODE_ENCODED12 (generated command + 2-byte PL1167 CRC)");
#else
  Serial.println("Mode: TX_MODE_REPLAY12 (captured raw10 + computed 2-byte PL1167 CRC)");
#endif
  Serial.printf("TX_REPS=%d TX_SCAN_ALL_CHANNELS=%d TX_FIXED_CHANNEL=%d TX_DELAY_US=%d\n",
                TX_REPS, TX_SCAN_ALL_CHANNELS, TX_FIXED_CHANNEL, TX_DELAY_US);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CSN);
  if (!radio.begin()) { Serial.println("FAIL: NRF24!"); while(1) delay(1000); }
  Serial.printf("isChipConnected: %s\n", radio.isChipConnected() ? "YES" : "NO");

  radio.setAutoAck(false);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setAddressWidth(5);
  radio.setPayloadSize(12);
  radio.disableCRC();
  radio.openWritingPipe(CCT_ADDRESS);
  radio.stopListening();

  Serial.println("Aloitan 5s kuluttua...");
  for (int i = 5; i > 0; i--) { Serial.printf("%d ", i); delay(1000); }
  Serial.println();

  printReplayMetadata("R1", 49);
  printReplayMetadata("R2", 51);
#if TX_SCAN_ALL_CHANNELS
  Serial.println("=== Brute-force channel scan 0..125 ===");
#else
  Serial.printf("=== Fixed channel test: %d ===\n", TX_FIXED_CHANNEL);
#endif
}

void loop() {
#if TX_MODE_ENCODED12
  sendPacket(0x12, 3, 3, true);
  delay(500);
#elif TX_SCAN_ALL_CHANNELS
  scanAllChannels(REPLAY1, "R1", 3);
  delay(300);
  scanAllChannels(REPLAY2, "R2", 3);
  delay(1500);
#else
#if TX_MODE_REPLAY10
  transmit10(REPLAY1, "R1");
  delay(300);
  transmit10(REPLAY2, "R2");
#else
  uint8_t tx[12];
  uint16_t crc = 0;
  buildReplayPacket(REPLAY1, tx, &crc);
  transmit12SingleChannel(tx, "R1", TX_FIXED_CHANNEL, TX_REPS);
  delay(300);
  buildReplayPacket(REPLAY2, tx, &crc);
  transmit12SingleChannel(tx, "R2", TX_FIXED_CHANNEL, TX_REPS);
#endif
  delay(1500);
#endif
}
