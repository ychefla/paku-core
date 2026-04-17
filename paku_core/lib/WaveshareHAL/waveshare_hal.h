/**
 * @file waveshare_hal.h
 * @brief Hardware Abstraction Layer for Waveshare ESP32-S3-Touch-LCD boards.
 *
 * Initialises the RGB parallel LCD (ST7262 800×480) via Arduino_GFX,
 * the GT911 capacitive touch panel via TouchLib, the CH422G IO expander,
 * and wires everything into LVGL display + input drivers.
 *
 * Call sequence from application:
 *   1. waveshare_hal_init()   — once in setup()
 *   2. waveshare_hal_loop()   — every loop() iteration (calls lv_timer_handler)
 *   3. waveshare_hal_set_backlight(pct)  — optional brightness 0-100
 *
 * @note Pin assignments and RGB timing values are sourced from the
 *       ESP32_Display_Panel board profiles and Waveshare demo code.
 *       Confidence: HIGH for pin mapping, MEDIUM for exact timing parameters
 *       (may need fine-tuning on real hardware).
 */
#pragma once

#include <Arduino.h>

/// Screen dimensions
#define WS_LCD_WIDTH   800
#define WS_LCD_HEIGHT  480

/**
 * @brief One-time initialisation of the Waveshare display hardware + LVGL.
 *
 * This performs (in order):
 *   1. I2C bus init
 *   2. CH422G IO expander init (backlight ON, LCD/touch reset)
 *   3. Arduino_GFX RGB panel + display init (800×480 @ ~16 MHz PCLK)
 *   4. GT911 touch controller init
 *   5. LVGL core init, display driver, input driver
 *
 * @return true on success.
 */
bool waveshare_hal_init();

/**
 * @brief Call every loop() iteration.
 *
 * Runs lv_timer_handler() so LVGL can process rendering and input.
 */
void waveshare_hal_loop();

/**
 * @brief Set LCD backlight brightness.
 * @param percent  0 = off, 100 = full.  Values outside range are clamped.
 *
 * On these boards the backlight is a digital enable via the CH422G
 * IO expander, so any value > 0 turns it fully on.  PWM backlight
 * control is not available without hardware modification.
 */
void waveshare_hal_set_backlight(uint8_t percent);
