# Frezzer PRO BLE Emulator (ESP32)

ESP32 firmware that pretends to be a **Frezzer PRO 65L** compressor fridge
(Alpicool OEM BLE platform). Useful for:

| Goal | How |
|---|---|
| Test paku-core BLE discovery + data flow | Flash emulator to ESP32 #2, run paku-core on ESP32 #1 |
| Verify Alpicool protocol correctness | Connect with the official Frezzer/Alpicool app |
| Raw GATT inspection | Connect with nRF Connect or LightBlue |

---

## Hardware

Any ESP32 board works:
- Generic ESP32 devkit (default)
- LilyGo T-Display S3 (change `board` in `platformio.ini`)
- ESP32-C3, ESP32-S3 — just update `board` accordingly

---

## Build & flash

```bash
cd tools/frezzer-emulator
pio run -t upload
pio device monitor   # 115200 baud
```

---

## What it advertises

```
Device name : WT-0001
Service     : 00001234-0000-1000-8000-00805f9b34fb
Write char  : 00001235-0000-1000-8000-00805f9b34fb  (commands in)
Notify char : 00001236-0000-1000-8000-00805f9b34fb  (status out)
```

---

## Serial commands (115200 baud)

| Command | Effect |
|---|---|
| `+` / `-` | Current temperature ±1 °C |
| `t<n>` | Set target temp, e.g. `t-10` or `t3` |
| `m0` | MAX_COOL mode |
| `m1` | ECO mode |
| `mf` | Power off |
| `l` | Toggle panel lock |
| `v<n>` | Battery voltage, e.g. `v11.8` |
| `e<0-3>` | Error code (0=none, 1=temp_sensor, 2=compressor, 3=low_bat) |
| `s` | Print current status frame in hex |
| `?` | Help |

---

## Protocol summary

```
Frame:  FE FE | lenByte | cmd | body... | cs_hi cs_lo
Checksum: 16-bit big-endian sum of all preceding bytes

Commands  (client → fridge, write to char 1235):
  0x00  BIND             6 bytes
  0x01  QUERY            6 bytes — fridge responds with 24-byte status
  0x02  SET              full settings payload
  0x04  RESET            6 bytes
  0x05  SET_LEFT_TARGET  7 bytes, body[0] = signed int8 °C

Status response  (fridge → client, notify on char 1236):
  FE FE 15 01 [18 body bytes] cs_hi cs_lo = 24 bytes total
  body[14] = current temp (signed int8 °C)
  body[4]  = target temp  (signed int8 °C)
  body[1]  = poweredOn
  body[2]  = runMode (0=Max, 1=Eco)
  body[15] = batPercent
  body[16..17] = battery voltage (int + dec/10)
```

Source: [klightspeed/BrassMonkeyFridgeMonitor](https://github.com/klightspeed/BrassMonkeyFridgeMonitor) (MIT)

---

## Connecting paku-core

If you want paku-core to connect to a specific emulator MAC, add to `secrets.h`:

```c
#define FREZZER_COUNT 1
static const char* FREZZER_MACS[]      = {"AA:BB:CC:DD:EE:FF"};  // emulator MAC
static const char* FREZZER_LOCATIONS[] = {"test_fridge"};
```

Find the emulator's MAC from Serial Monitor on startup, or from `pio device monitor`.

If no MAC is configured, paku-core will auto-discover any `WT-0001` in range.
