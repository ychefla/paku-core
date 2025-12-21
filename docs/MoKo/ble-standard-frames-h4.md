# BLE Standard Advertising Frames (H4 / H4 Pro)

## Scope
This document contains **only BLE standard advertising frames**
used by **MOKO H4 and H4 Pro sensors**.

Included standards:
- iBeacon (Apple)
- Eddystone-UID
- Eddystone-URL
- Eddystone-TLM

No vendor-specific sensor payloads are included here.

---

## BLE AD Structure

```
| Length (1) | AD Type (1) | AD Data (Length-1 bytes) |
```

---

## iBeacon

**AD Type:** 0xFF  
**Company ID:** 0x004C

| Offset | Size | Field |
|------:|-----:|------|
| 0 | 1 | Length |
| 1 | 1 | AD Type = 0xFF |
| 2–3 | 2 | Company ID |
| 4 | 1 | iBeacon type (0x02) |
| 5 | 1 | iBeacon length (0x15) |
| 6–21 | 16 | UUID |
| 22–23 | 2 | Major |
| 24–25 | 2 | Minor |
| 26 | 1 | Measured Power (RSSI @1m) |

---

## Eddystone-UID

**Service UUID:** 0xFEAA

| Offset | Size | Field |
|------:|-----:|------|
| 0 | 1 | Length |
| 1 | 1 | AD Type = 0x16 |
| 2–3 | 2 | Service UUID |
| 4 | 1 | Frame Type = 0x00 |
| 5 | 1 | Tx Power |
| 6–15 | 10 | Namespace |
| 16–21 | 6 | Instance |
| 22–23 | 2 | RFU |

---

## Eddystone-URL

| Offset | Size | Field |
|------:|-----:|------|
| 4 | 1 | Frame Type = 0x10 |
| 5 | 1 | Tx Power |
| 6 | 1 | URL Prefix |
| 7… | N | Encoded URL |

---

## Eddystone-TLM

| Offset | Size | Field |
|------:|-----:|------|
| 4 | 1 | Frame Type = 0x20 |
| 5 | 1 | Version |
| 6–7 | 2 | Battery voltage (mV) |
| 8–9 | 2 | Temperature |
| 10–13 | 4 | ADV count |
| 14–17 | 4 | Time since boot |

---
