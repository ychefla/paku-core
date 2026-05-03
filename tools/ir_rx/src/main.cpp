/**
 * MaxxFan IR Receiver / Decoder
 *
 * Listens on GPIO27 (VS1838B) and decodes MaxxFan Deluxe IR packets.
 * Protocol: RS232-like UART at 1250 baud (800 us per bit), 38 kHz carrier.
 *   VS1838B: LOW = carrier (mark), HIGH = no carrier (space)
 *   Start bit : LOW  (800 us)
 *   Data bits : LSB first, LOW=0 HIGH=1  (8 bits)
 *   Stop bits : HIGH (2 x 800 us = 1600 us)
 */

#include <Arduino.h>

#define PIN_IR_RX    27
#define BIT_US       800
#define SILENCE_US   15000
#define MAX_EDGES    512

static const uint8_t PREAMBLE[10] = { 0x5A, 0xA5, 0x80, 0x7F, 0x40,
                                       0xBF, 0x20, 0xDF, 0x10, 0xCC };

volatile uint32_t g_times[MAX_EDGES];
volatile uint8_t  g_lvls[MAX_EDGES];
volatile int      g_cnt  = 0;
volatile uint32_t g_last = 0;

void IRAM_ATTR ir_isr() {
    uint32_t now = micros();
    int lv = digitalRead(PIN_IR_RX);
    if (g_cnt < MAX_EDGES) {
        g_times[g_cnt] = now;
        g_lvls[g_cnt]  = (uint8_t)lv;
        g_cnt++;
    }
    g_last = now;
}

// Build flat bit array from edge run-lengths.
// lvls[i] = VS1838B level during [times[i], times[i+1]]
static int build_bits(const uint32_t* times, const uint8_t* lvls, int cnt,
                      uint8_t* bits, int max_bits) {
    int nbit = 0;
    for (int i = 0; i < cnt - 1 && nbit < max_bits - 8; i++) {
        uint32_t dur = times[i + 1] - times[i];
        int n = (int)((dur + BIT_US / 2) / BIT_US);
        if (n < 1) n = 1;
        if (n > 16) n = 16;
        uint8_t lv = lvls[i];
        for (int j = 0; j < n; j++) bits[nbit++] = lv;
    }
    // Pad with HIGH so last byte's stop bits are always present
    for (int j = 0; j < 8 && nbit < max_bits; j++) bits[nbit++] = 1;
    return nbit;
}

// Decode RS232-like UART bytes from bit array.
static int decode_bytes(const uint8_t* bits, int nbit, uint8_t* out, int max_bytes) {
    int byte_count = 0;
    int pos = 0;
    while (pos < nbit && byte_count < max_bytes) {
        while (pos < nbit && bits[pos] != 0) pos++;  // hunt start bit (LOW)
        if (pos >= nbit) break;
        pos++;  // consume start bit
        if (pos + 8 > nbit) break;
        uint8_t bval = 0;
        for (int b = 0; b < 8; b++)
            if (bits[pos + b]) bval |= (1 << b);  // LSB first
        pos += 8;
        out[byte_count++] = bval;
        while (pos < nbit && bits[pos] == 1) pos++;  // skip stop bits
    }
    return byte_count;
}

static void print_command(const uint8_t* pkt) {
    uint8_t flags  = pkt[10];
    uint8_t speed  = pkt[11];
    uint8_t temp_f = pkt[12];
    Serial.println("==================================");
    Serial.println("    MaxxFan IR Command received");
    Serial.println("==================================");
    Serial.printf("  Fan      : %s\n", (flags & 0x01) ? "ON" : "OFF");
    Serial.printf("  Direction: %s\n", (flags & 0x04) ? "EXHAUST" : "INTAKE");
    Serial.printf("  Lid      : %s\n", (flags & 0x08) ? "OPEN" : "CLOSED");
    Serial.printf("  Mode     : %s\n", (flags & 0x10) ? "AUTO" : "MANUAL");
    if (flags & 0x10)
        Serial.printf("  Setpoint : %dF (%.0fC)\n", temp_f,
                      (temp_f - 32.0f) * 5.0f / 9.0f);
    Serial.printf("  Speed    : %d%%\n", speed);
    if (flags & 0x02) Serial.println("  Special  : YES");
    if (flags & 0x20) Serial.println("  Warn     : YES");
    Serial.println("==================================");
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("[ir_rx] MaxxFan IR decoder ready.");
    Serial.printf("[ir_rx] Listening GPIO%d — point remote and press a button\n", PIN_IR_RX);
    pinMode(PIN_IR_RX, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_IR_RX), ir_isr, CHANGE);
}

static uint32_t snap_times[MAX_EDGES];
static uint8_t  snap_lvls[MAX_EDGES];
static uint8_t  bits[2048];

void loop() {
    if (g_cnt == 0) return;
    if ((uint32_t)(micros() - g_last) < SILENCE_US) return;

    detachInterrupt(digitalPinToInterrupt(PIN_IR_RX));
    int cnt = g_cnt;
    memcpy(snap_times, (const void*)g_times, cnt * sizeof(uint32_t));
    memcpy(snap_lvls,  (const void*)g_lvls,  cnt);
    g_cnt = 0;
    attachInterrupt(digitalPinToInterrupt(PIN_IR_RX), ir_isr, CHANGE);

    if (cnt < 30) return;  // ignore noise

    int nbit   = build_bits(snap_times, snap_lvls, cnt, bits, 2048);
    uint8_t pkt[32] = {};
    int nbytes = decode_bytes(bits, nbit, pkt, 32);

    Serial.printf("\n[rx] %d edges -> %d bits -> %d bytes\n", cnt, nbit, nbytes);
    Serial.print("[raw] ");
    for (int i = 0; i < nbytes && i < 20; i++) Serial.printf("%02X ", pkt[i]);
    Serial.println();

    if (nbytes < 16) {
        Serial.println("[rx] short packet");
        return;
    }

    if (memcmp(pkt, PREAMBLE, 10) != 0) {
        Serial.println("[rx] preamble mismatch");
        Serial.print("[exp] ");
        for (int i = 0; i < 10; i++) Serial.printf("%02X ", PREAMBLE[i]);
        Serial.println();
    }

    uint8_t csum = pkt[10] ^ pkt[11] ^ pkt[12] ^ pkt[13] ^ pkt[14];
    if (csum != pkt[15]) {
        Serial.printf("[rx] checksum FAIL got=%02X exp=%02X\n", pkt[15], csum);
        return;
    }

    print_command(pkt);
}
