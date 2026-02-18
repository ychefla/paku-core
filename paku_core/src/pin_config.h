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

/* NRF24L01+ for MiLight/MIBO control (reuses former SD card pins) */
#define PIN_NRF24_CE                 1   // Chip Enable
#define PIN_NRF24_CSN                2   // SPI Chip Select
#define PIN_NRF24_SCK                11  // SPI Clock (former SD_CLK)
#define PIN_NRF24_MOSI               13  // SPI MOSI (former SD_CMD)
#define PIN_NRF24_MISO               12  // SPI MISO (former SD_D0)
