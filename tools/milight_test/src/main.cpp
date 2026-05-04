// MiLight CCT V2 (FUT091) sniffer.
//
// Listens on the MiLight CCT syncword and prints every validated frame.
// "Validated" means: PL1167 length byte == 9, PL1167 CRC matches, and the
// V2-decoded request type byte == 0x45 (CCT V2). Anything else is counted
// but not printed (see SNIFFER_PRINT_INVALID).
//
// Output format per packet:
//   PKT,<ms>,ch=<n>,key=<KK>,type=<TT>,id=<HHHH>,grp=<g>,cmd=<CC>(<name>),arg=<a>,seq=<s>,raw=<24 hex chars>
//
// Heartbeat every SNIFFER_HEARTBEAT_MS shows alive + counts so you can tell
// the difference between "no packets being sent" and "sniffer hung".

#include <Arduino.h>
#include <RF24.h>
#include <SPI.h>

// --- NRF24 pins ---
#define PIN_CE   22
#define PIN_CSN  21
#define PIN_SCK  18
#define PIN_MOSI 23
#define PIN_MISO 19
#define PIN_IRQ  27

#ifndef SNIFFER_FIXED_CHANNEL
#define SNIFFER_FIXED_CHANNEL -1
#endif

#ifndef SNIFFER_USE_IRQ
#define SNIFFER_USE_IRQ 0
#endif

#ifndef SNIFFER_IRQ_EDGE
#define SNIFFER_IRQ_EDGE 0
#endif

// Time spent on each channel before hopping. Real remotes burst across all
// three channels in roughly 30 ms, so 50 ms gives reasonable hit rate while
// keeping cycle responsive. Bump to 200+ ms if you want fewer hops.
#ifndef SNIFFER_CHANNEL_SWITCH_MS
#define SNIFFER_CHANNEL_SWITCH_MS 50
#endif

// Print packets that fail length/checksum/type checks. ON by default so
// you can see signs of life when the validator rejects everything. Set to
// 0 once you trust the radio path and want clean decoded-only output.
#ifndef SNIFFER_PRINT_INVALID
#define SNIFFER_PRINT_INVALID 1
#endif

// Suppress repeats of the same decoded fields within this many ms.
#ifndef SNIFFER_DEDUP_MS
#define SNIFFER_DEDUP_MS 0
#endif

#define SNIFFER_MAX_PKTS_PER_LOOP 32
#define SNIFFER_POLL_DELAY_US 100
#define SNIFFER_IRQ_EMPTY_BACKOFF_MS 20
#define SNIFFER_HEARTBEAT_MS 5000
#define SNIFFER_STATS_PRINT_MS 15000
#define SNIFFER_MAX_STATS 48

// nRF24 in fixed-payload mode only asserts RX_DR after exactly this many
// bytes have been clocked in. Many MIBO/MiLight remotes transmit only the
// length byte + 9 V2 payload bytes after the syncword (no PL1167 CRC
// trailer); asking for 12 here makes the radio sit forever and rx stays 0.
// Keep at 10 unless you know your remote includes the CRC.
#define MILIGHT_FRAME_LEN 10
#define MILIGHT_PAYLOAD_LEN 9
#define MILIGHT_CCT_REQ_TYPE 0x45

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

static uint8_t v2XorKey(uint8_t key) {
  const uint8_t shift = (key & 0x0F) < 0x04 ? 0 : 1;
  const uint8_t x = (((key & 0xF0) >> 4) + shift + 6) % 8;
  const uint8_t msn = (((4 + x) ^ 1) & 0x0F) << 4;
  const uint8_t lsn = ((((key & 0x0F) + 4) ^ 2) & 0x0F);
  return msn | lsn;
}

static uint8_t v2Offset(uint8_t byteIdx, uint8_t key, uint8_t jumpStart) {
  uint8_t base = V2_OFFSETS[byteIdx - 1][key % 4];
  if (jumpStart > 0 && key >= jumpStart && key < jumpStart + 0x80) base += 0x80;
  return base;
}

// Decode 9-byte V2 payload in place. p[0] is the key byte (untouched);
// p[1..7] become {type,id_hi,id_lo,cmd,arg,seq,group}; p[8] is the encoded
// checksum (decoded with jumpStart=0 to match v2Encode).
static void v2Decode(uint8_t *p) {
  uint8_t key = v2XorKey(p[0]);
  for (int i = 1; i <= 8; i++) {
    uint8_t off = v2Offset(i, p[0], i == 8 ? 0 : V2_OFFSET_JUMP_START);
    p[i] = (p[i] - off) ^ key;
  }
}

static uint8_t reverseBits(uint8_t b) {
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
  return b;
}

// CCT syncword address, 5 bytes, derived for nRF24 from PL1167
// preamble 0xAA + syncword 0x050A 0x55AA. Same value used by sniffer + TX.
static const uint8_t CCT_ADDRESS[5] = {0xAA, 0x5A, 0x05, 0x0A, 0x55};

// CCT channels. esp8266_milight_hub stores {4,39,74} in MiLightRadioConfig
// and then writes `setChannel(2 + ch)` to the nRF24, so the actual channels
// in use are {6, 41, 76} (= 2.406, 2.441, 2.476 GHz). Earlier comments in
// this repo claimed 2.404/2.439/2.474 but that was a 2 MHz miss; with the
// wrong channel the nRF24 hears adjacent-channel RF on the RPD but never
// matches the syncword.
static const uint8_t CCT_CHANNELS[3] = {6, 41, 76};

static RF24 radio(PIN_CE, PIN_CSN);
static bool radio_ok = false;
static bool irq_ok = true;
static volatile bool irq_fired = false;
static uint8_t cur_ch = 0;
static uint32_t last_channel_switch = 0;
static uint32_t last_heartbeat = 0;
static uint32_t last_stats_print = 0;

static uint32_t pkts_received = 0;
static uint32_t bad_length = 0;
static uint32_t bad_chk = 0;
static uint32_t bad_type = 0;
static uint32_t valid_count = 0;
static uint32_t dedup_count = 0;
static uint32_t rpd_hits = 0;

struct ActionStat {
  uint8_t req_type;
  uint8_t id1;
  uint8_t id2;
  uint8_t group;
  uint8_t cmd;
  uint8_t arg;
  uint32_t count;
  uint32_t first_ms;
  uint32_t last_ms;
};
static ActionStat stats[SNIFFER_MAX_STATS] = {};
static uint8_t stats_used = 0;

// Last decoded packet, for deduplication.
static uint8_t last_dec[8] = {0};
static uint32_t last_dec_ms = 0;
static bool last_dec_valid = false;

static const char* actionName(uint8_t command) {
  switch (command) {
    case 0x01: return "ON";
    case 0x02: return "OFF";
    case 0x05: return "WHITE";
    case 0x07: return "BRIGHTNESS";
    case 0x09: return "TEMPERATURE";
    case 0x11: return "GRP-ON";
    case 0x12: return "GRP-OFF";
    case 0x19: return "TEMP";
    case 0x1C: return "BRIGHT";
    default:   return "?";
  }
}

static void updateStats(uint8_t req_type, uint8_t id1, uint8_t id2, uint8_t group,
                        uint8_t cmd, uint8_t arg, uint32_t now_ms) {
  for (uint8_t i = 0; i < stats_used; i++) {
    ActionStat &s = stats[i];
    if (s.req_type == req_type && s.id1 == id1 && s.id2 == id2 &&
        s.group == group && s.cmd == cmd && s.arg == arg) {
      s.count++;
      s.last_ms = now_ms;
      return;
    }
  }
  if (stats_used >= SNIFFER_MAX_STATS) return;
  ActionStat &n = stats[stats_used++];
  n.req_type = req_type;
  n.id1 = id1;
  n.id2 = id2;
  n.group = group;
  n.cmd = cmd;
  n.arg = arg;
  n.count = 1;
  n.first_ms = now_ms;
  n.last_ms = now_ms;
}

static void printStats(uint32_t now_ms) {
  Serial.printf("MAP,total_unique=%u,uptime_ms=%lu\n", stats_used, now_ms);
  for (uint8_t i = 0; i < stats_used; i++) {
    const ActionStat &s = stats[i];
    Serial.printf("MAP,%u,type=0x%02X,id=%02X%02X,grp=%u,cmd=0x%02X(%s),arg=%u,count=%lu,first_ms=%lu,last_ms=%lu\n",
                  i, s.req_type, s.id1, s.id2, s.group, s.cmd,
                  actionName(s.cmd), s.arg, s.count, s.first_ms, s.last_ms);
  }
}

static uint8_t configuredChannel() {
#if SNIFFER_FIXED_CHANNEL >= 0
  return (uint8_t)SNIFFER_FIXED_CHANNEL;
#else
  return CCT_CHANNELS[cur_ch];
#endif
}

static void tuneToCurrentChannel() {
  radio.setChannel(configuredChannel());
  radio.startListening();
}

static void IRAM_ATTR onRadioIrq() {
  irq_fired = true;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== MiLight CCT V2 sniffer ===");
  Serial.printf("payload=%u, channel=%s, addr=AA 5A 05 0A 55\n",
                (unsigned)MILIGHT_FRAME_LEN,
#if SNIFFER_FIXED_CHANNEL >= 0
                "fixed"
#else
                "hop 4/39/74"
#endif
  );

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CSN);

  if (!radio.begin()) {
    Serial.println("FAIL: NRF24 not responding - check wiring/power.");
    return;
  }
  Serial.printf("isChipConnected: %s, isPVariant: %s\n",
                radio.isChipConnected() ? "YES" : "NO",
                radio.isPVariant() ? "YES" : "NO");
  radio_ok = true;

  radio.setAutoAck(false);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setAddressWidth(5);
  radio.setPayloadSize(MILIGHT_FRAME_LEN);   // length(1) + 9-byte V2 payload
  radio.disableCRC();                        // remote uses no PL1167 CRC trailer
  radio.openReadingPipe(1, CCT_ADDRESS);

#if SNIFFER_USE_IRQ
  pinMode(PIN_IRQ, INPUT_PULLUP);
  radio.maskIRQ(true, true, false);
#endif
  tuneToCurrentChannel();

  // Dump registers AFTER all config + tune so we see the actual state the
  // chip is operating in — earlier this was placed before config and showed
  // post-begin() defaults, which was misleading.
  Serial.println("--- radio.printDetails() (post-config) ---");
  radio.printDetails();
  Serial.println("--- end printDetails ---");

#if SNIFFER_USE_IRQ
  radio.clearStatusFlags();
  delay(5);
  if (digitalRead(PIN_IRQ) == LOW) {
    Serial.println("FAIL: IRQ stuck LOW. Check IRQ wiring.");
    Serial.flush();
    irq_ok = false;
    return;
  }
#if SNIFFER_IRQ_EDGE
  attachInterrupt(digitalPinToInterrupt(PIN_IRQ), onRadioIrq, FALLING);
  Serial.println("IRQ edge mode enabled.");
#endif
#endif

  last_channel_switch = millis();
  last_heartbeat = last_channel_switch;
  last_stats_print = last_channel_switch;
}

static void maybeHopChannel() {
#if SNIFFER_FIXED_CHANNEL < 0
  if (millis() - last_channel_switch > SNIFFER_CHANNEL_SWITCH_MS) {
    cur_ch = (cur_ch + 1) % 3;
    // Original-working sniffer just wrote RF_CH while still listening.
    // Going through stopListening() clears the RX FIFO and may drop a
    // packet that was being clocked in at the moment we hop.
    radio.setChannel(CCT_CHANNELS[cur_ch]);
    last_channel_switch = millis();
  }
#endif
}

static void printPacket(uint32_t ms, uint8_t ch, const uint8_t *raw12,
                        const uint8_t *dec) {
  // dec layout after v2Decode: [0]=key, [1]=type, [2]=id_hi, [3]=id_lo,
  //                            [4]=cmd, [5]=arg, [6]=seq, [7]=group, [8]=cksum
  Serial.printf("PKT,%lu,ch=%u,key=%02X,type=%02X,id=%02X%02X,grp=%u,cmd=%02X(%s),arg=%u,seq=%u,raw=",
                ms, ch,
                dec[0], dec[1], dec[2], dec[3], dec[7],
                dec[4], actionName(dec[4]), dec[5], dec[6]);
  for (int i = 0; i < MILIGHT_FRAME_LEN; i++) {
    if (raw12[i] < 0x10) Serial.print('0');
    Serial.print(raw12[i], HEX);
  }
  Serial.println();
}

static bool isDuplicate(const uint8_t *dec, uint32_t now_ms) {
#if SNIFFER_DEDUP_MS > 0
  if (last_dec_valid && (now_ms - last_dec_ms) < SNIFFER_DEDUP_MS) {
    if (memcmp(last_dec, dec, 8) == 0) return true;
  }
  memcpy(last_dec, dec, 8);
  last_dec_ms = now_ms;
  last_dec_valid = true;
#endif
  return false;
}

static void printRawHex(const uint8_t *raw, size_t len) {
  for (size_t i = 0; i < len; i++) {
    if (raw[i] < 0x10) Serial.print('0');
    Serial.print(raw[i], HEX);
  }
}

static void handleFrame(const uint8_t *raw_air) {
  // Convert from nRF24 wire order (bit-reversed PL1167) to PL1167 bytes.
  uint8_t pkt[MILIGHT_FRAME_LEN];
  for (int i = 0; i < MILIGHT_FRAME_LEN; i++) pkt[i] = reverseBits(raw_air[i]);

  uint32_t now = millis();
  uint8_t ch = configuredChannel();

  if (pkt[0] != MILIGHT_PAYLOAD_LEN) {
    bad_length++;
#if SNIFFER_PRINT_INVALID
    Serial.printf("BAD,len=0x%02X,ch=%u,raw_air=", pkt[0], ch);
    printRawHex(raw_air, MILIGHT_FRAME_LEN);
    Serial.println();
#endif
    return;
  }

  // V2-decode the 9-byte payload. dec[0] is the key (untouched by decode);
  // dec[1..7] become {type,id_hi,id_lo,cmd,arg,seq,group}; dec[8] becomes
  // the integrity sum (= xk + sum(dec[1..7]) + 2 modulo 256).
  uint8_t dec[9];
  memcpy(dec, &pkt[1], MILIGHT_PAYLOAD_LEN);
  v2Decode(dec);

  uint8_t xk = v2XorKey(dec[0]);
  uint8_t expected = (uint8_t)(xk + dec[1] + dec[2] + dec[3] + dec[4] + dec[5] + dec[6] + dec[7] + 2);
  if (dec[8] != expected) {
    bad_chk++;
#if SNIFFER_PRINT_INVALID
    Serial.printf("BAD,chk=%02X(want=%02X),ch=%u,pl1167=", dec[8], expected, ch);
    printRawHex(pkt, MILIGHT_FRAME_LEN);
    Serial.println();
#endif
    return;
  }

  if (dec[1] != MILIGHT_CCT_REQ_TYPE) {
    bad_type++;
#if SNIFFER_PRINT_INVALID
    Serial.printf("BAD,type=0x%02X,ch=%u,pl1167=", dec[1], ch);
    printRawHex(pkt, MILIGHT_FRAME_LEN);
    Serial.println();
#endif
    return;
  }

  valid_count++;
  if (isDuplicate(dec, now)) { dedup_count++; return; }

  updateStats(dec[1], dec[2], dec[3], dec[7], dec[4], dec[5], now);
  printPacket(now, ch, pkt, dec);
}

void loop() {
  if (!radio_ok || !irq_ok) {
    delay(1000);
    return;
  }

  uint32_t now = millis();
  if (now - last_heartbeat >= SNIFFER_HEARTBEAT_MS) {
    Serial.printf("[ALIVE] ch=%u rx=%lu valid=%lu len_bad=%lu chk_bad=%lu type_bad=%lu dedup=%lu rpd=%lu\n",
                  configuredChannel(), pkts_received, valid_count,
                  bad_length, bad_chk, bad_type, dedup_count, rpd_hits);
    last_heartbeat = now;
  }

  // RPD (Received Power Detector) latches high when the radio sees signal
  // >-64 dBm on the current channel. Reading it clears the latch. If
  // rpd_hits stays 0 while the remote is being pressed nearby, RF isn't
  // reaching the antenna. If rpd_hits climbs but rx stays 0, the syncword
  // / address is wrong.
  if (radio.testRPD()) rpd_hits++;
  if (stats_used > 0 && now - last_stats_print >= SNIFFER_STATS_PRINT_MS) {
    printStats(now);
    last_stats_print = now;
  }

#if SNIFFER_USE_IRQ
#if SNIFFER_IRQ_EDGE
  if (!irq_fired) { delay(1); return; }
  irq_fired = false;
#else
  if (digitalRead(PIN_IRQ) != LOW) {
    maybeHopChannel();
    delay(1);
    return;
  }
#endif
#else
  maybeHopChannel();
#endif

  uint8_t drained = 0;
  while (radio.available() && drained < SNIFFER_MAX_PKTS_PER_LOOP) {
    drained++;
    uint8_t raw[MILIGHT_FRAME_LEN] = {0};
    radio.read(raw, MILIGHT_FRAME_LEN);
    pkts_received++;
    handleFrame(raw);
  }
  if (drained > 0) Serial.flush();

#if SNIFFER_USE_IRQ
  if (drained == 0) {
    radio.clearStatusFlags();
    delay(SNIFFER_IRQ_EMPTY_BACKOFF_MS);
    return;
  }
#endif

  delayMicroseconds(SNIFFER_POLL_DELAY_US);
}
