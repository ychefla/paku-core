# MiLight/MIBO Light Controller Feature

## Overview

The paku-core firmware now supports controlling MiLight/MIBO CCT (Color Temperature) LED receivers using an NRF24L01+ radio module. This allows direct control of 2.4GHz RF-controlled lights from the ESP32-S3.

## Hardware Requirements

### Components
- LilyGo T-Display S3 (ESP32-S3)
- NRF24L01+ radio module
- 10µF capacitor (recommended across NRF24 VCC/GND)
- MIBO CCT LED receivers or compatible MiLight devices

### Wiring

| NRF24L01+ Pin | ESP32-S3 GPIO | Notes |
|---------------|---------------|-------|
| SCK | 11 | SPI2 clock (former SD_CLK, unused) |
| MOSI | 13 | SPI2 data out (former SD_CMD, unused) |
| MISO | 12 | SPI2 data in (former SD_D0, unused) |
| CE | 1 | Chip Enable — free GPIO |
| CSN | 2 | SPI Chip Select — free GPIO |
| VCC | 3.3V | Board 3.3V rail. **Add 10µF cap across VCC/GND** |
| GND | GND | Common ground |

## Build Configuration

### Environment Selection

Use the `lilygo-t-display-s3-light` environment in `platformio.ini`:

```bash
pio run -e lilygo-t-display-s3-light
```

This environment enables:
- `MILIGHT_ENABLED` - Compiles MiLight controller code
- `DRY_RUN_LIGHT` - Logs packets to Serial instead of transmitting (for testing)
- `nrf24/RF24@^1.4.0` - NRF24L01+ radio driver dependency

### Removing DRY_RUN Mode

For production use with actual NRF24 hardware, remove `-DDRY_RUN_LIGHT` from the build flags in `platformio.ini`.

## MQTT Control

### Command Topic

Subscribe to: `paku/edge/{deviceId}/cmd/light`

### Status Topic

Publishes to: `paku/edge/{deviceId}/status/light`

### Command Examples

#### Turn light ON/OFF
```json
{"state": "on", "channel": 1}
{"state": "off", "channel": 1}
{"state": "toggle", "channel": 1}
```

#### Set brightness (0-100)
```json
{"state": "on", "brightness": 80, "channel": 1}
```

#### Set color temperature (mireds: 153=cool, 500=warm)
```json
{"state": "on", "color_temp": 350, "channel": 1}
```

#### Combined command
```json
{
  "state": "on",
  "brightness": 75,
  "color_temp": 400,
  "channel": 1
}
```

#### Pair receiver
```json
{
  "cmd": "pair",
  "channel": 1,
  "protocol": "cct"
}
```

To pair:
1. Send the pair command via MQTT
2. Within 3 seconds, power cycle the LED receiver
3. The receiver should flash to confirm pairing

#### Unpair receiver
```json
{
  "cmd": "unpair",
  "channel": 1,
  "protocol": "cct"
}
```

#### Switch protocol
```json
{
  "cmd": "set_protocol",
  "protocol": "fut091"
}
```

Supported protocols:
- `cct` - CCT v1 (FUT007/FUT011) - Original protocol
- `fut091` - FUT091 (CCT v2) - Newer protocol

## Display UI

### Light Screen

The light controller adds a new screen to the display UI:

- **Light state**: ON (green) / OFF (red)
- **Brightness**: 0-100%
- **Color temperature**: Cool / Neutral / Warm + mireds value
- **Channel**: 1-4
- **Protocol**: CCT or FUT091

### Button Controls

- **Button 2 (short press)**: Cycle through screens
- **Button 2 (long press on Light screen)**: Toggle light ON/OFF on channel 1

## Channels

The MiLight protocol supports 4 channels (groups) per controller. This allows controlling up to 4 separate receivers or receiver groups from a single device.

## Protocol Notes (CCT V2 / FUT091)

These are non-obvious facts verified by RF sniffing — they affect both this firmware and any
raw nRF24 port of the MiLight protocol.

### Channel offset: +2 MHz

`esp8266_milight_hub`'s `MiLightRadioConfig` stores `{4, 39, 74}` for CCT, but
`PL1167_nRF24::recalc_parameters` writes `setChannel(2 + ch)`. The actual on-air channels
are `{6, 41, 76}` (2.406 / 2.441 / 2.476 GHz). Using the config-table values with a raw
nRF24 causes every syncword match to be missed. The same +2 offset applies to all protocol
families (RGBW, FUT089, RGB, FUT020).

### Bit order

PL1167 sends bytes LSB-first; nRF24 sends/receives MSB-first. Every byte in the frame
(length, payload, CRC trailer) must be bit-reversed when crossing between the two.

### Frame lengths

- **Remote → receiver**: 10 bytes (1 length + 9 V2 payload). No PL1167 CRC trailer.
  A sniffer must use `payloadSize=10`; asking for 12 makes the nRF24 wait forever.
- **Hub/firmware → receiver**: 12 bytes (same + 2-byte PL1167 CRC appended). Receivers
  accept both, but including the CRC is preferred.

### V2 command / argument map (FUT091/MIBO, empirically verified)

| Command | Name    | Arg |
|---------|---------|-----|
| `0x11`  | GRP-ON  | group 1–4 |
| `0x12`  | GRP-OFF | group 1–4 |
| `0x1C`  | BRIGHT  | 0–100 absolute |
| `0x19`  | TEMP    | 0–100 (0 = cool, 100 = warm) |

BRIGHT and TEMP use absolute values, not step/relative commands.

---

## Troubleshooting

### DRY_RUN Mode

When `DRY_RUN_LIGHT` is defined, the controller logs packet hex to Serial instead of transmitting:

```
[MILIGHT] DRY_RUN mode - no hardware initialization
[MILIGHT] Initialized with device_id=0xABCD
[MILIGHT] CCT ON/OFF: B0 2F 08 64 00 00 00
[MILIGHT] Pairing channel 1 with device_id=0xABCD protocol=cct
[MILIGHT] CCT PAIR: B0 2F 08 64 00 00 00
```

This allows testing the full MQTT → JSON parse → packet encode → log flow without NRF24 hardware.

### Receiver Not Responding

1. **Check wiring**: Ensure all connections are secure
2. **Check power**: NRF24 modules need stable 3.3V (add capacitor!)
3. **Try other protocol**: Some receivers use `fut091` instead of `cct`
4. **Re-pair**: Power cycle the receiver and send pair command again
5. **Check distance**: NRF24 range is typically 10-30m depending on obstacles

### Wrong Protocol

If the receiver doesn't respond to commands, try switching protocols:

```json
{"cmd": "set_protocol", "protocol": "fut091"}
```

Then send your command again.

## Device ID Persistence

The controller auto-generates a device ID from the ESP32 MAC address. This ID is used for pairing and must remain consistent. Future versions may add NVS persistence for custom device IDs.

## License

The MiLight protocol implementation is based on [esp8266_milight_hub](https://github.com/sidoh/esp8266_milight_hub) by Chris Mullins, licensed under the MIT License. See `paku_core/lib/MiLightClient/README.md` for full license text.
