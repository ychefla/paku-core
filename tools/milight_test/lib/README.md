# MiLightClient Library

Simplified MiLight/MIBO CCT light controller for ESP32-S3 + NRF24L01+.

## Overview

This library provides a lightweight interface for controlling MiLight/MIBO CCT (Color Temperature) LED receivers using an NRF24L01+ radio module connected to an ESP32-S3.

## Supported Protocols

- **CCT v1** (FUT007/FUT011) - Original CCT protocol with 7-byte packets
- **FUT091** (CCT v2) - Newer CCT protocol with encoded packets

## Hardware Requirements

- ESP32-S3 (LilyGo T-Display S3)
- NRF24L01+ radio module
- Wiring:
  - SCK  → GPIO 11
  - MOSI → GPIO 13
  - MISO → GPIO 12
  - CE   → GPIO 1
  - CSN  → GPIO 2
  - VCC  → 3.3V (add 10µF capacitor across VCC/GND)
  - GND  → GND

## Dependencies

- `nrf24/RF24@^1.4.0` - NRF24L01+ radio driver

## Usage

```cpp
#include "milight_client.h"

void setup() {
    // Initialize MiLight radio
    milight_init(PIN_NRF24_CE, PIN_NRF24_CSN, 
                 PIN_NRF24_SCK, PIN_NRF24_MOSI, PIN_NRF24_MISO);
    
    // Pair receiver to channel 1 (CCT protocol)
    milight_pair(1, PROTOCOL_CCT);
}

void loop() {
    // Create state
    MiLightState state;
    state.on = true;
    state.brightness = 80;
    state.color_temp = 350;  // Neutral white
    state.channel = 1;
    state.protocol = PROTOCOL_CCT;
    
    // Send state
    milight_send_state(state);
}
```

## DRY_RUN Mode

When `DRY_RUN_LIGHT` is defined, the library logs packet hex to Serial instead of transmitting via SPI. This allows testing the full command flow without NRF24 hardware.

## License Attribution

Protocol implementation based on [esp8266_milight_hub](https://github.com/sidoh/esp8266_milight_hub) by Chris Mullins, licensed under the MIT License.

```
MIT License

Copyright (c) 2018 Chris Mullins

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## Notes

This is a simplified implementation focused on CCT control only. Full RGB/RGBW support and advanced features from the original library are not included.
