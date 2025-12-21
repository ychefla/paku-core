# MOKO H4 / H4 Pro – Sensor Advertising (Documented)

## Scope
This document describes **vendor-specific advertising formats**
for **MOKO H4 and H4 Pro sensors only**, based on the official
*ADV Format Summary Sheet*.

No other MOKO products are included.

---

## Sensor Info ADV Frame

**AD Type:** 0xFF (Manufacturer Specific)

| Offset | Field | Description |
|------:|------|-------------|
| 0–1 | Company ID | Assigned by MOKO |
| 2 | ADV Type | 0x80 (documented, v1.1+) |
| 3 | Device Type | H4 / H4 Pro |
| 4… | Sensor Data | Temp / RH / Motion |
| … | Battery | Battery voltage |
| … | Reserved | — |

### Revision Notes
- ADV Type changed from **0x81 → 0x80** in v1.1
- Payload meaning defined by MOKO documentation

---

## Supported Sensors (H4 / H4 Pro)

- Temperature
- Humidity
- Motion (accelerometer, H4 Pro)

---

## Notes
- Payload field scaling and units are defined in the Excel sheet
- No undocumented fields are interpreted
- Backward compatibility depends on firmware revision

---
