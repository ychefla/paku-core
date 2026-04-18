#pragma once

// Include secrets (WiFi and MQTT credentials)
// Copy include/secrets.h.template to include/secrets.h and fill in your credentials
#include "secrets.h"


#define WIFI_CONNECT_WAIT_MAX        (30 * 1000)

#define NTP_SERVER1                  "pool.ntp.org"
#define NTP_SERVER2                  "time.nist.gov"
#define GMT_OFFSET_SEC               0
#define DAY_LIGHT_OFFSET_SEC         0
// if CUSTOM_TIMEZONE is not defined then TIMEZONE API used based on IP, check zones.h
// #define CUSTOM_TIMEZONE             "Europe/London"

/* Automatically update local time */
#define GET_TIMEZONE_API             "https://ipapi.co/timezone/"

/*ESP32S3*/
// =============================================================================
// Board-specific pin definitions
// =============================================================================

#if defined(DEVICE_LILYGO_T_DISPLAY_S3)
// --- LilyGo T-Display S3 (ST7789V SPI LCD) ---

#define PIN_LCD_BL                   38

#define PIN_LCD_D0                   39
#define PIN_LCD_D1                   40
#define PIN_LCD_D2                   41
#define PIN_LCD_D3                   42
#define PIN_LCD_D4                   45
#define PIN_LCD_D5                   46
#define PIN_LCD_D6                   47
#define PIN_LCD_D7                   48

#define PIN_POWER_ON                 15

#define PIN_LCD_RES                  5
#define PIN_LCD_CS                   6
#define PIN_LCD_DC                   7
#define PIN_LCD_WR                   8
#define PIN_LCD_RD                   9

#define PIN_BUTTON_1                 0
#define PIN_BUTTON_2                 14
#define PIN_BAT_VOLT                 4

#define PIN_IIC_SCL                  17
#define PIN_IIC_SDA                  18

#define PIN_TOUCH_INT                16
#define PIN_TOUCH_RES                21

/* External expansion */
#define PIN_SD_CMD                   13
#define PIN_SD_CLK                   11
#define PIN_SD_D0                    12

/* MaxxFan IR LED */
#define PIN_IR_LED                   3    // GPIO 3 - free on T-Display S3

/* NRF24L01+ for MiLight/MIBO control (reuses former SD card pins) */
#define PIN_NRF24_CE                 1   // Chip Enable
#define PIN_NRF24_CSN                2   // SPI Chip Select
#define PIN_NRF24_SCK                11  // SPI Clock (former SD_CLK)
#define PIN_NRF24_MOSI               13  // SPI MOSI (former SD_CMD)
#define PIN_NRF24_MISO               12  // SPI MISO (former SD_D0)

#elif defined(DEVICE_WAVESHARE_LCD_4_3) || defined(DEVICE_WAVESHARE_LCD_5)
// =============================================================================
// Waveshare ESP32-S3-Touch-LCD-4.3 / 5 — Common pins
// =============================================================================
// Both boards share the same PCB family. Pin data sourced from the
// Espressif ESP32_Display_Panel library and Waveshare demo code.
// Confidence: HIGH for I2C, Touch, SD, RS485, CAN.  LCD RGB bus pins
// verified against ESP32_Display_Panel board profile for these SKUs.

/* I2C bus (shared: touch GT911 @ 0x5D, IO expander CH422G @ 0x24,
   and on the 5" board: RTC PCF85063A @ 0x51) */
#define PIN_IIC_SDA                  8
#define PIN_IIC_SCL                  9

/* GT911 capacitive touch */
#define PIN_TOUCH_INT                4    // Touch interrupt (active LOW)
// Note: Touch RESET is on CH422G EXIO1 (not a direct GPIO)

/* CH422G I2C IO expander (addr 0x24) — directly addressable GPIOs:
   EXIO1 = TP_RST (touch reset)
   EXIO2 = LCD_BL (backlight enable)
   EXIO3 = LCD_RST (LCD reset)
   EXIO4 = SD_CS (SD card chip select)
   EXIO5 = USB_SEL (USB path selection)             */
#define CH422G_I2C_ADDR              0x24
#define CH422G_EXIO_TP_RST           1    // Touch panel reset
#define CH422G_EXIO_LCD_BL           2    // LCD backlight
#define CH422G_EXIO_LCD_RST          3    // LCD reset
#define CH422G_EXIO_SD_CS            4    // SD card chip-select
#define CH422G_EXIO_USB_SEL          5    // USB path selection

/* SD card (directly wired GPIOs — CS via CH422G EXIO4 above) */
#define PIN_SD_CMD                   11   // SPI MOSI
#define PIN_SD_CLK                   12   // SPI CLK
#define PIN_SD_D0                    13   // SPI MISO

/* RS485 transceiver */
#define PIN_RS485_TX                 15
#define PIN_RS485_RX                 16

/* CAN / TWAI transceiver */
#define PIN_CAN_TX                   20
#define PIN_CAN_RX                   19

/* MaxxFan IR LED (connect external IR LED here) */
#define PIN_IR_LED                   43   // GPIO43 (shared with UART header TX on 4.3")

/* NRF24L01+ for MiLight/MIBO control (no dedicated pins on Waveshare —
   reuse SD SPI bus when SD card is not active; CE/CSN on UART header) */
#define PIN_NRF24_CE                 43   // GPIO43 (UART header TX)
#define PIN_NRF24_CSN                44   // GPIO44 (UART header RX)
#define PIN_NRF24_SCK                12   // SPI CLK (shared with SD_CLK)
#define PIN_NRF24_MOSI               11   // SPI MOSI (shared with SD_CMD)
#define PIN_NRF24_MISO               13   // SPI MISO (shared with SD_D0)

/* ST7262 RGB LCD bus (800×480, RGB565 parallel)
   Defined here for reference; the RGB LCD driver is not yet implemented. */
#define PIN_LCD_HSYNC                46
#define PIN_LCD_VSYNC                3
#define PIN_LCD_DE                   5
#define PIN_LCD_PCLK                 7
/* Blue [4:0] */
#define PIN_LCD_B0                   14
#define PIN_LCD_B1                   38
#define PIN_LCD_B2                   18
#define PIN_LCD_B3                   17
#define PIN_LCD_B4                   10
/* Green [5:0] */
#define PIN_LCD_G0                   39
#define PIN_LCD_G1                   0
#define PIN_LCD_G2                   45
#define PIN_LCD_G3                   48
#define PIN_LCD_G4                   47
#define PIN_LCD_G5                   21
/* Red [4:0] */
#define PIN_LCD_R0                   1
#define PIN_LCD_R1                   2
#define PIN_LCD_R2                   42
#define PIN_LCD_R3                   41
#define PIN_LCD_R4                   40

// ---- Board-specific extras ----

#if defined(DEVICE_WAVESHARE_LCD_4_3)
/* External UART header (PH2.0 connector) */
#define PIN_UART_EXT_TX              43
#define PIN_UART_EXT_RX              44
/* External ADC header (PH2.0 connector) */
#define PIN_ADC_EXT                  6    // ADC1_CH5 on ESP32-S3

#elif defined(DEVICE_WAVESHARE_LCD_5)
/* PCF85063A RTC on the shared I2C bus */
#define PCF85063A_I2C_ADDR           0x51
/* Isolated IO is routed through CH422G — no extra direct GPIOs */
#endif  // DEVICE_WAVESHARE_LCD_4_3 / _5

#endif  // Board selection
