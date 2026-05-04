# MiLight RF24 Test Matrix (FUT091/CCT-v2)

## Scope

This test plan is based on current code in:
- `tools/milight_test/src/main.cpp` (sniffer, 10-byte RX path)
- `tools/milight_test/src_tx/main.cpp` (TX, 12-byte frame with appended CRC)

I do not have a verified source that this exactly matches every MiLight remote model, so treat this as a measurement-first debug plan for your current implementation.

## Goal

Find the first protocol layer where generated TX diverges from real remote behavior.

Layers under test:
1. RF24 PHY settings (channel, data rate, CRC/ACK, payload width)
2. PL1167-compatible framing assumptions
3. Bit order / byte order
4. FUT091/V2 encoding and command semantics

## Preconditions

1. Hardware
- One ESP32 + nRF24 as sniffer
- One ESP32 + nRF24 as transmitter
- Real MiLight remote + paired receiver bulb/controller

2. Wiring and power
- Stable 3.3V for nRF24
- 10uF capacitor across nRF24 VCC/GND recommended

3. Build environments
- Sniffer: `tools/milight_test`, env `milight-test`
- Transmitter: `tools/milight_test`, env `milight-tx`

## Run Commands

From repo root `paku-core`:

```bash
cd tools/milight_test
pio run -e milight-test -t upload
pio device monitor -e milight-test
```

In another terminal:

```bash
cd tools/milight_test
pio run -e milight-tx -t upload
pio device monitor -e milight-tx
```

## Data Collection Template

For each test case, record:
- test_id
- timestamp
- sniffer channel seen (4/39/74 or other)
- sniffer RAW bytes
- sniffer decoded fields (`type`, `id`, `grp`, `cmd`, `arg`, `seq`)
- bulb/receiver reaction (yes/no, latency)
- notes

Minimum sample size per case: 30 button presses or 30 TX attempts.

## Test Matrix

### Phase 1: Baseline capture (real remote only)

| ID | Action | Expected | Decision |
|---|---|---|---|
| B1 | Press one known button repeatedly (same group/button) | Sniffer sees stable `type/id/cmd` pattern with changing `seq` | If unstable -> fix sniff quality before TX analysis |
| B2 | Repeat for all relevant buttons (ON, OFF, BRIGHT, TEMP) | Distinct command signatures by button | Build golden reference set |
| B3 | Capture channel distribution | Most hits should cluster on expected channels | If not, channel-hopping assumptions may be wrong |

Pass criterion for Phase 1:
- You have a golden dataset for each tested button and can identify its decoded command signature.

### Phase 2: Replay validation (TX side)

Current TX sends 12-byte packets formed from captured 10-byte data + computed CRC.

| ID | Action | Expected | Interpretation |
|---|---|---|---|
| R1 | TX replay using captured packet `REPLAY1` | Sniffer sees matching RAW pattern and receiver reacts | Full path likely correct for this command |
| R2 | TX replay using captured packet `REPLAY2` | Same as R1 | Confirms reproducibility |
| R3 | Compare sniffer-captured TX bytes vs golden remote bytes | Byte-for-byte match over air (or explainable delta) | Any unexplained delta points to framing/order mismatch |

If sniffer sees TX but receiver does not react:
- Likely mismatch in packet structure assumptions (e.g., appended CRC not expected by receiver path) or timing/repetition profile.

### Phase 3: RF24 framing toggles

These are controlled experiments. Change one variable at a time and re-run R1/R2.

| ID | Variable | Variant A | Variant B | Outcome use |
|---|---|---|---|---|
| F1 | RF24 CRC | `disableCRC()` | `setCRCLength(RF24_CRC_16)` | Check if receiver path expects/depends on RF24 CRC behavior |
| F2 | TX payload width | 12 bytes (current) | 10 bytes (raw only) | Tests whether appended CRC bytes are harming compatibility |
| F3 | Repetition profile | current burst | slower/lower burst | Detect receiver timing sensitivity |
| F4 | Channel strategy | hop 4/39/74 | fixed single channel | Detect channel timing mismatch |

Important: F2 requires a small TX code path for 10-byte raw write. Add it as an explicit mode and log selected mode in serial.

### Phase 4: Bit/byte order verification

| ID | Action | Expected | Interpretation |
|---|---|---|---|
| O1 | Disable bit-reversal in TX for one test mode | Usually no receiver reaction | Confirms bit-reversal is required |
| O2 | Reverse CRC byte order only | Should fail if order is wrong | Identifies endianness issue in trailer |
| O3 | Compare decoded fields from sniffed TX vs golden remote | Same semantic fields (`type/id/cmd/arg`) | If semantics differ, V2 encoding path is wrong |

### Phase 5: Command-level semantics

| ID | Action | Expected | Interpretation |
|---|---|---|---|
| S1 | Send only known-good command pair from golden set | Receiver reaction should mirror remote | Verifies minimal command path |
| S2 | Vary only `seq` while other fields fixed | Usually still accepted in small range | If rejected, sequence handling may be stricter than assumed |
| S3 | Vary `group` with fixed command | Group-specific behavior | Confirms zone mapping |

## Decision Tree (Fast)

1. If baseline remote capture is not stable:
- Stop TX work, fix sniffer reliability first.

2. If TX replay appears on sniffer but receiver never reacts:
- Prioritize F2 (10-byte vs 12-byte TX) and F3 timing profile.

3. If TX replay does not appear on sniffer as expected:
- Prioritize RF24 setup parity and O1/O2 bit-order checks.

4. If replay works but encoded command generation fails:
- Focus on V2 encode path and command field mapping.

## Practical Next Edits (Small, High Value)

1. Add TX compile-time mode flags:
- `TX_MODE_REPLAY12` (current)
- `TX_MODE_REPLAY10`
- `TX_MODE_ENCODED12`

2. Print explicit mode banner at boot in `src_tx/main.cpp`.

3. Add one-line CSV output in sniffer:
- `ts_ms,ch,raw_hex,type,id,grp,cmd,arg,seq`

These changes turn the current setup into a measurement tool instead of a guessing tool.

## Exit Criteria

You can declare protocol path validated when all are true:
1. At least one replay mode reproduces receiver reaction >= 95% over 30 attempts.
2. Sniffer-captured TX bytes match golden remote bytes (or documented deterministic transform).
3. Encoded command mode produces same decoded semantics as remote for ON/OFF/BRIGHT/TEMP.
