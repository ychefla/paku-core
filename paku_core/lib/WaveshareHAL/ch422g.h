/**
 * @file ch422g.h
 * @brief CH422G I2C IO-expander driver for Waveshare ESP32-S3-Touch-LCD boards.
 *
 * The CH422G provides 8 extended IO pins (EXIO0-7) controllable via I2C
 * at address 0x24.  On the Waveshare boards the relevant pins are:
 *   EXIO1 = TP_RST   (touch panel reset)
 *   EXIO2 = LCD_BL   (backlight enable)
 *   EXIO3 = LCD_RST  (LCD reset)
 *   EXIO4 = SD_CS    (SD card chip-select)
 *   EXIO5 = USB_SEL  (USB path selection)
 *
 * @note Implementation based on Waveshare demo code and ESP32_Display_Panel
 *       board profiles.  The register-level protocol is inferred from those
 *       sources; I have not verified it against an official CH422G datasheet.
 */
#pragma once

#include <Arduino.h>
#include <Wire.h>

/**
 * @brief Simple driver for the CH422G I2C IO expander.
 *
 * Provides output-only control of the 8 EXIO pins.  Reads are not
 * implemented because the Waveshare boards only use outputs.
 */
class CH422G {
public:
    /**
     * @brief Construct a CH422G driver.
     * @param wire   Reference to a TwoWire instance (default Wire).
     * @param addr   I2C address (normally 0x24).
     */
    explicit CH422G(TwoWire &wire = Wire, uint8_t addr = 0x24);

    /**
     * @brief Initialise the expander.
     *
     * Enables "push-pull output" mode so all EXIO pins can drive loads.
     * Must be called after Wire.begin().
     *
     * @return true on success, false if I2C communication fails.
     */
    bool begin();

    /**
     * @brief Set one EXIO pin HIGH or LOW.
     * @param pin   Pin number 0-7 (maps to EXIO0-EXIO7).
     * @param level HIGH / LOW.
     */
    void digitalWrite(uint8_t pin, uint8_t level);

    /**
     * @brief Write the full 8-bit output register at once.
     * @param value  Bitmask – bit N controls EXIO-N.
     */
    void writeOutputs(uint8_t value);

private:
    TwoWire &_wire;
    uint8_t  _addr;
    uint8_t  _outputState;  ///< Shadow register for the 8 EXIO pins

    /** Write a single byte to a CH422G register address.  */
    bool _write(uint8_t regAddr, uint8_t data);
};
