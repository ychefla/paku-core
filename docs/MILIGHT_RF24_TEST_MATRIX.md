# MiLight RF24 Test Matrix — FUT091/CCT-v2 (Findings)

## Scope

Tools under test:

- `tools/milight_test/src/main.cpp` — sniffer (10-byte RX path)
- `tools/milight_test/src_tx/main.cpp` — TX impersonator (12-byte frame)

## Run Commands

From repo root `paku-core`:

```bash
cd tools/milight_test
# Sniffer
pio run -e milight-test -t upload && pio device monitor -e milight-test

# Transmitter (different terminal / after closing sniffer monitor)
pio run -e milight-tx -t upload && pio device monitor -e milight-tx
```

---

## Critical Findings (verified empirically)

### 1. Channel offset: +2 MHz vs sidoh's config table

`esp8266_milight_hub`'s `MiLightRadioConfig` stores nominal channels `{4, 39, 74}` for CCT.
`PL1167_nRF24::recalc_parameters` unconditionally does `_radio.setChannel(2 + _channel)` before
writing the RF_CH register. The actual on-air frequencies are therefore **2 MHz higher**:

| Protocol | Config values | Actual on-air | Frequencies (GHz) |
|----------|--------------|---------------|-------------------|
| CCT      | {4, 39, 74}  | **{6, 41, 76}**   | 2.406 / 2.441 / 2.476 |
| RGBW     | {9, 40, 71}  | {11, 42, 73}  | 2.411 / 2.442 / 2.473 |
| FUT089   | {8, 39, 70}  | {10, 41, 72}  | 2.410 / 2.441 / 2.472 |
| RGB      | {3, 38, 73}  | {5, 40, 75}   | 2.405 / 2.440 / 2.475 |
| FUT020   | {6, 41, 76}  | {8, 43, 78}   | 2.408 / 2.443 / 2.478 |

Using the config-table values directly makes the nRF24 miss every syncword. The chip's RPD
still fires (adjacent-channel leakage), which can mask the bug as "occasional" captures.

**Always use `{6, 41, 76}` (not `{4, 39, 74}`) when configuring a raw nRF24 for CCT.**

### 2. Frame length: 10 bytes — remote sends NO PL1167 CRC trailer

The physical remote transmits:

```
[5-byte syncword] [0x09 length] [9-byte V2 payload]
```

Total over the air: 1 + 9 = **10 bytes** after the syncword. There is no PL1167 CRC trailer
appended by the remote. Setting `radio.setPayloadSize(12)` causes the nRF24 to wait for bytes
11 and 12 that never arrive — `radio.available()` stays false forever.

**Set `radio.setPayloadSize(10)` on the sniffer side.**

### 3. TX must append a PL1167 CRC (12 bytes total)

The hub firmware (`PL1167_nRF24`) computes a 16-bit PL1167-CRC16 and appends it, producing a
12-byte frame. MIBO receivers accept either 10 or 12 bytes because the CRC is validated at the
PL1167 layer, not by the nRF24 itself. Using 12 bytes and a correct CRC gives receivers
additional validation opportunity; use it on the TX side.

**Set `radio.setPayloadSize(12)` on the TX side.** The two CRC bytes are bit-reversed for nRF24
just like all other bytes (PL1167 = LSB-first, nRF24 = MSB-first).

### 4. Bit order: every byte must be bit-reversed

PL1167 sends bytes LSB-first; nRF24 sends/receives MSB-first. Every byte in the frame —
including the length byte, all payload bytes, and the two CRC bytes — must be bit-reversed
before writing/after reading.

### 5. V2 checksum validation

After V2-decoding the 9-byte payload `dec[0..8]`:

```
xk  = v2XorKey(dec[0])
expected = (xk + dec[1] + dec[2] + dec[3] + dec[4] + dec[5] + dec[6] + dec[7] + 2) & 0xFF
valid = (dec[8] == expected)
```

Packets that fail this check are noise, not MiLight frames.

### 6. Radio initialisation: keep it minimal

These settings caused rx=0 regressions when added; do not use them on the sniffer:

- `WiFi.mode(WIFI_OFF)` / `btStop()` — no measurable benefit, caused instability on some builds
- `radio.setPALevel(RF24_PA_LOW)` — reduces receive sensitivity; use `RF24_PA_MAX`
- Custom `SPI.begin()` with reduced clock — default SPI speed is fine
- `radio.flush_rx()` on every channel hop — clears packets still being transferred
- `radio.stopListening()` + `radio.startListening()` on channel hop — unnecessary, write
  `radio.setChannel()` only; the chip re-tunes within microseconds

### 7. V2 command / argument semantics (empirically verified against FUT091/MIBO)

| Command byte | Name    | Arg meaning |
|-------------|---------|-------------|
| `0x11`      | GRP-ON  | group number (1–4) |
| `0x12`      | GRP-OFF | group number (1–4) |
| `0x1C`      | BRIGHT  | absolute brightness 0–100 |
| `0x19`      | TEMP    | absolute color temperature 0–100 (0 = cool, 100 = warm) |

Note: early analysis had 0x11/0x12 swapped. This has been corrected in both sniffer and TX
after testing against actual bulbs (group 3, device_id=0x0532).

### 8. Syncword / address

5-byte nRF24 address derived from PL1167 preamble + syncword bytes:

```
{0xAA, 0x5A, 0x05, 0x0A, 0x55}
```

---

## Sniffer Performance

With correct channels ({6, 41, 76}) and payload size (10), the sniffer captures real remote
traffic at **>99%** valid-packet rate over a 30-second session (541/543 valid in reference run).

Output format per packet:

```
PKT,<ms>,ch=<n>,key=<KK>,type=<TT>,id=<HHHH>,grp=<g>,cmd=<CC>(<name>),arg=<a>,seq=<s>,raw=<20hex>
```

Heartbeat every 5 s shows running counters (rx / valid / len_bad / chk_bad / type_bad / dedup / rpd).

---

## TX Modes (compile-time)

| Build env              | Mode              | Description |
|------------------------|-------------------|-------------|
| `milight-tx`           | ENCODED12 (default) | Generates V2-encoded packets, device_id=0x0532, group 3 |
| `milight-tx-replay12-fixed` | REPLAY12   | Retransmits captured 10-byte frames + computed CRC |
| `milight-tx-replay10`  | REPLAY10          | Sends raw 10 bytes, no CRC — sniffer-only test |

---

## Replay CRC computation

`buildReplayPacket` in `src_tx/main.cpp`:

1. Bit-reverse each of the 10 captured (air-order) bytes → original PL1167 bytes
2. Compute PL1167-CRC16 on those original bytes
3. Wire bytes 0–9: original captured bytes (already in air order)
4. Wire bytes 10–11: bit-reverse of CRC low byte, bit-reverse of CRC high byte

Computing CRC on the bit-reversed (air) bytes gives a wrong CRC and receivers reject the packet.

---

## Exit Criteria (met)

1. Sniffer captures real remote at ≥95% valid over 30 attempts — **PASS (99.6%)**
2. TX encoded mode produces packets that the sniffer decodes with matching semantics — **PASS**
3. TX encoded mode produces receiver reaction (bulbs on/off/dim/temp) — **PASS**
