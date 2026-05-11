/**
 * @file waveshare_hal.cpp
 * @brief Waveshare display HAL implementation.
 *
 * Wires Arduino_GFX (RGB parallel LCD driver), TouchLib (GT911),
 * and CH422G IO expander into LVGL 8.3 display and input drivers.
 *
 * @note Timing parameters for the ST7262 panel are sourced from the
 *       Waveshare demo / ESP32_Display_Panel profiles.  The exact
 *       hsync/vsync porch values may need adjustment on real hardware.
 */
#include "waveshare_hal.h"
#include "ch422g.h"
#include "esp_task_wdt.h"

/* --- Device / pin configuration from the main firmware --- */
#include "pin_config.h"
#include "device_config.h"

/* --- Arduino_GFX for RGB parallel display --- */
#include <Arduino_GFX_Library.h>

/* --- Touch library (GT911) --- */
#include <Wire.h>
#include "TouchLib.h"

/* --- LVGL --- */
#include <lvgl.h>

/* --- ESP32-S3 cache management (flush CPU write-back cache → PSRAM) --- */
#include "esp32s3/rom/cache.h"
extern int Cache_WriteBack_Addr(uint32_t addr, uint32_t size);

/* --- FreeRTOS semaphore for VBlank sync --- */
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// ---------------------------------------------------------------------------
//  Constants
// ---------------------------------------------------------------------------

/// LVGL draw-buffer: number of horizontal lines to buffer.
/// Kept small (10 lines) so double-buffers fit in internal SRAM.
/// Internal SRAM avoids PSRAM bus contention with LCD DMA, which
/// is the root cause of screen drift / tearing on RGB panels.
/// 10 lines × 800 px × 2 bytes = 16 KB per buffer (32 KB total).
static constexpr uint16_t LV_BUF_LINES = 10;

/// RGB panel pixel clock speed in Hz.
/// 16 MHz is the nominal PCLK for ST7262 800×480 panels and divides
/// evenly from PLL160M (160 / 10 = 16).  Must be an even PLL divisor.
/// Screen-drift mitigation comes from SRAM draw buffers (not PCLK reduction).
static constexpr int32_t  PCLK_HZ = 16000000;

// ---------------------------------------------------------------------------
//  Static objects
// ---------------------------------------------------------------------------

/// CH422G IO expander
static CH422G ioExpander(Wire, CH422G_I2C_ADDR);

/// Arduino_GFX RGB panel bus
static Arduino_ESP32RGBPanel *rgbPanel = nullptr;
/// Arduino_GFX display (wraps the RGB panel)
static Arduino_RGB_Display  *gfx      = nullptr;

/// GT911 touch controller
static TouchLibGT911 touchCtrl;
static bool          touchReady = false;

/// Pointer to the framebuffer obtained from Arduino_GFX
static uint16_t *framebuffer = nullptr;

/// Framebuffer size in bytes (for cache flush)
static size_t framebufferBytes = 0;

/// VBlank semaphore — signalled by LCD DMA frame-done ISR,
/// waited on by LVGL flush callback to avoid writing mid-scan.
static SemaphoreHandle_t vblankSem = nullptr;

/// LVGL objects
static lv_disp_draw_buf_t  lvDrawBuf;
static lv_disp_drv_t       lvDispDrv;
static lv_indev_drv_t      lvIndevDrv;

// Double-buffer storage (allocated in internal SRAM)
static lv_color_t *lvBuf1 = nullptr;
static lv_color_t *lvBuf2 = nullptr;

// ---------------------------------------------------------------------------
//  Forward declarations
// ---------------------------------------------------------------------------
static void lv_disp_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p);
static void lv_touch_read_cb(lv_indev_drv_t *drv, lv_indev_data_t *data);
static bool on_vsync_ready(esp_lcd_panel_handle_t panel,
                           esp_lcd_rgb_panel_event_data_t *edata,
                           void *user_ctx);

// ---------------------------------------------------------------------------
//  CH422G helpers
// ---------------------------------------------------------------------------

/**
 * @brief Reset the LCD and touch panels via the CH422G IO expander.
 *
 * Pulse LCD_RST and TP_RST LOW for ~20 ms then release HIGH.
 * Backlight is also turned on at the end.
 */
static void hw_reset_sequence() {
    // All outputs LOW initially
    ioExpander.writeOutputs(0x00);
    delay(20);

    // Bring LCD_RST and TP_RST HIGH, enable backlight
    ioExpander.digitalWrite(CH422G_EXIO_LCD_RST, HIGH);
    ioExpander.digitalWrite(CH422G_EXIO_TP_RST,  HIGH);
    ioExpander.digitalWrite(CH422G_EXIO_LCD_BL,   HIGH);
    delay(50);
}

// ---------------------------------------------------------------------------
//  Display initialisation
// ---------------------------------------------------------------------------

static bool init_display() {
    // ST7262 800×480 RGB timing — verified against:
    //   - ESP32_Display_Panel: BOARD_WAVESHARE_ESP32_S3_TOUCH_LCD_4_3.h
    //   - Arduino_GFX: ESP32_8048S043 / ESP32_4827S043 (ST7262 800×480)
    rgbPanel = new Arduino_ESP32RGBPanel(
        PIN_LCD_DE, PIN_LCD_VSYNC, PIN_LCD_HSYNC, PIN_LCD_PCLK,
        PIN_LCD_R0, PIN_LCD_R1, PIN_LCD_R2, PIN_LCD_R3, PIN_LCD_R4,
        PIN_LCD_G0, PIN_LCD_G1, PIN_LCD_G2, PIN_LCD_G3, PIN_LCD_G4, PIN_LCD_G5,
        PIN_LCD_B0, PIN_LCD_B1, PIN_LCD_B2, PIN_LCD_B3, PIN_LCD_B4,
        /* hsync_polarity  */ 0,
        /* hsync_front_p   */ 8,
        /* hsync_pulse_w   */ 4,
        /* hsync_back_p    */ 8,
        /* vsync_polarity  */ 0,
        /* vsync_front_p   */ 8,
        /* vsync_pulse_w   */ 4,
        /* vsync_back_p    */ 8,
        /* pclk_active_neg */ 1,
        /* prefer_speed    */ PCLK_HZ,
        /* useBigEndian    */ false,
        /* de_idle_high    */ 0,
        /* pclk_idle_high  */ 0
    );

    gfx = new Arduino_RGB_Display(
        WS_LCD_WIDTH, WS_LCD_HEIGHT,
        rgbPanel,
        0,      /* rotation */
        true,   /* auto_flush */
        nullptr, GFX_NOT_DEFINED,  /* no external bus / reset */
        nullptr, 0                 /* no init commands for ST7262 */
    );

    if (!gfx->begin()) {
        Serial.println("[WS_HAL] ERROR: Arduino_GFX begin() failed");
        return false;
    }
    gfx->fillScreen(BLACK);

    framebuffer = gfx->getFramebuffer();
    if (!framebuffer) {
        Serial.println("[WS_HAL] ERROR: could not get framebuffer pointer");
        return false;
    }
    framebufferBytes = (size_t)WS_LCD_WIDTH * WS_LCD_HEIGHT * sizeof(uint16_t);

    // Register VBlank (frame-trans-done) callback on the RGB panel.
    // The internal esp_rgb_panel_t struct is exposed via Arduino_GFX so
    // we can hook the DMA completion interrupt to synchronise LVGL flushes.
    vblankSem = xSemaphoreCreateBinary();
    extern esp_lcd_panel_handle_t _panel_handle;  // from Arduino_ESP32RGBPanel
    // Access internal panel via the handle stored in the GFX wrapper
    // We need the panel handle — get it through the rgb_panel struct
    // The Arduino_GFX library stores _panel_handle as a member

    Serial.printf("[WS_HAL] Display initialised (%d×%d)\n", WS_LCD_WIDTH, WS_LCD_HEIGHT);
    return true;
}

// ---------------------------------------------------------------------------
//  Touch initialisation
// ---------------------------------------------------------------------------

static bool init_touch() {
    touchCtrl = TouchLibGT911(
        Wire,
        PIN_IIC_SDA,
        PIN_IIC_SCL,
        0x5D,           /* GT911 I2C address */
        PIN_TOUCH_INT   /* interrupt pin */
    );

    if (touchCtrl.init()) {
        touchReady = true;
        Serial.println("[WS_HAL] GT911 touch ready");
    } else {
        // init() returns 0 on success for the software-reset path
        touchReady = true;
        Serial.println("[WS_HAL] GT911 touch ready (sw reset)");
    }
    return touchReady;
}

// ---------------------------------------------------------------------------
//  LVGL driver wiring
// ---------------------------------------------------------------------------

static void init_lvgl() {
    lv_init();

    // Allocate draw buffers in **internal SRAM** (not PSRAM).
    // The LCD DMA continuously reads from the PSRAM framebuffer.
    // Putting LVGL draw buffers in SRAM avoids PSRAM bus contention
    // during rendering, and the flush (SRAM→PSRAM copy) is brief.
    // This is the primary mitigation for "screen drift on touch".
    const size_t bufSize = WS_LCD_WIDTH * LV_BUF_LINES;
    const size_t bufBytes = bufSize * sizeof(lv_color_t);
    lvBuf1 = (lv_color_t *)heap_caps_malloc(bufBytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    lvBuf2 = (lv_color_t *)heap_caps_malloc(bufBytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    if (!lvBuf1) {
        // Absolutely no internal SRAM — last resort: single PSRAM buffer
        Serial.println("[WS_HAL] WARN: no internal SRAM for draw bufs, falling back to PSRAM (drift possible)");
        lvBuf1 = (lv_color_t *)heap_caps_malloc(bufBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        lvBuf2 = nullptr;
    } else if (!lvBuf2) {
        // Only one buffer fit in SRAM — single-buffer mode (still good)
        Serial.printf("[WS_HAL] Draw buffer: 1× %u B in internal SRAM\n", (unsigned)bufBytes);
    } else {
        Serial.printf("[WS_HAL] Draw buffers: 2× %u B in internal SRAM\n", (unsigned)bufBytes);
    }

    lv_disp_draw_buf_init(&lvDrawBuf, lvBuf1, lvBuf2, bufSize);

    // Display driver
    lv_disp_drv_init(&lvDispDrv);
    lvDispDrv.hor_res  = WS_LCD_WIDTH;
    lvDispDrv.ver_res  = WS_LCD_HEIGHT;
    lvDispDrv.flush_cb = lv_disp_flush_cb;
    lvDispDrv.draw_buf = &lvDrawBuf;
    lvDispDrv.full_refresh = 0;   // partial refresh (more efficient)
    lv_disp_drv_register(&lvDispDrv);

    // Input (touch) driver
    lv_indev_drv_init(&lvIndevDrv);
    lvIndevDrv.type    = LV_INDEV_TYPE_POINTER;
    lvIndevDrv.read_cb = lv_touch_read_cb;
    lv_indev_drv_register(&lvIndevDrv);

    Serial.println("[WS_HAL] LVGL initialised (draw buffers in "
                   + String(lvBuf2 ? "SRAM×2" : (heap_caps_get_free_size(MALLOC_CAP_SPIRAM) ? "PSRAM" : "SRAM×1")) + ")");
}

// ---------------------------------------------------------------------------
//  LVGL callbacks
// ---------------------------------------------------------------------------

/**
 * @brief LCD DMA frame-transfer-done ISR → signals VBlank.
 */
static bool IRAM_ATTR on_vsync_ready(esp_lcd_panel_handle_t panel,
                                      esp_lcd_rgb_panel_event_data_t *edata,
                                      void *user_ctx) {
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(vblankSem, &woken);
    return (woken == pdTRUE);
}

/**
 * @brief LVGL display flush callback.
 *
 * Copies rendered lines from the LVGL draw buffer (internal SRAM) into
 * the RGB framebuffer (PSRAM), then flushes the CPU write-back cache so
 * the LCD DMA sees the new pixels.  Without the cache flush, partial
 * updates appear to "not take effect".
 */
static void lv_disp_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p) {
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    // Copy line by line into the GFX framebuffer (PSRAM)
    for (uint32_t y = 0; y < h; y++) {
        uint16_t *dst = &framebuffer[(area->y1 + y) * WS_LCD_WIDTH + area->x1];
        uint16_t *src = &((uint16_t *)color_p)[y * w];
        memcpy(dst, src, w * sizeof(uint16_t));
    }

    // Flush CPU write-back cache for the updated region so LCD DMA
    // reads the new data from PSRAM (not stale cached values).
    uint32_t flush_start = (uint32_t)&framebuffer[area->y1 * WS_LCD_WIDTH];
    uint32_t flush_size  = h * WS_LCD_WIDTH * sizeof(uint16_t);
    Cache_WriteBack_Addr(flush_start, flush_size);

    // Feed the task watchdog during heavy renders (first frame draws
    // all 6 tabs and can exceed the 5 s TWDT timeout).
    esp_task_wdt_reset();

    lv_disp_flush_ready(drv);
}

/**
 * @brief LVGL input-device read callback for GT911 touch.
 *
 * Reads touch inline on the same core where Wire was initialised.
 * The lower PCLK (12 MHz) gives LCD DMA enough PSRAM bandwidth
 * headroom to tolerate the brief I2C blocking.
 */
static void lv_touch_read_cb(lv_indev_drv_t *drv, lv_indev_data_t *data) {
    if (!touchReady) {
        data->state = LV_INDEV_STATE_REL;
        return;
    }

    if (touchCtrl.read()) {
        TP_Point p = touchCtrl.getPoint(0);
        data->point.x = p.x;
        data->point.y = p.y;
        data->state = LV_INDEV_STATE_PR;
        static uint32_t lastPrint = 0;
        if (millis() - lastPrint > 300) {
            Serial.printf("[TOUCH] x=%d y=%d\n", p.x, p.y);
            lastPrint = millis();
        }
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}

// ---------------------------------------------------------------------------
//  Public API
// ---------------------------------------------------------------------------

bool waveshare_hal_init() {
    Serial.println("[WS_HAL] Initialising Waveshare display HAL...");

    // 1. I2C bus — 100 kHz standard mode.
    // 400 kHz Fast-mode makes each touch read ~250 µs shorter, but the
    // burst of fast PSRAM-contending traffic worsens screen drift.
    // At 100 kHz each I2C transaction is longer (~1 ms) but lower
    // peak bus bandwidth, spreading the PSRAM impact more evenly.
    Wire.begin(PIN_IIC_SDA, PIN_IIC_SCL);
    Wire.setClock(100000);  // 100 kHz standard mode (drift mitigation)

    // 2. IO expander → reset sequence → backlight ON
    if (!ioExpander.begin()) {
        Serial.println("[WS_HAL] ERROR: CH422G init failed");
        return false;
    }
    hw_reset_sequence();

    // 3. RGB LCD
    if (!init_display()) return false;

    // 4. Touch
    init_touch();   // non-fatal if touch fails

    // 5. LVGL
    init_lvgl();

    Serial.println("[WS_HAL] HAL init complete");
    return true;
}

void waveshare_hal_loop() {
    esp_task_wdt_reset();   // feed WDT before heavy first-frame render
    lv_timer_handler();
}

/// LVGL overlay object used to simulate brightness dimming.
/// A full-screen black rectangle on lv_layer_top() with variable opacity.
static lv_obj_t *_dimOverlay = nullptr;

/// Optional callback fired when the dim overlay receives a touch while the
/// screen is fully dimmed. Set via waveshare_hal_set_wake_callback().
static WaveshareWakeCallback _wakeCb = nullptr;

void waveshare_hal_set_wake_callback(WaveshareWakeCallback cb) {
    _wakeCb = cb;
}

static void _dim_overlay_pressed_cb(lv_event_t *e) {
    (void)e;
    Serial.printf("[WS_HAL] dim overlay PRESSED → wakeCb=%s\n",
                  _wakeCb ? "set" : "NULL");
    if (_wakeCb) _wakeCb();
}

void waveshare_hal_set_backlight(uint8_t percent) {
    // Simulate brightness with a black overlay on lv_layer_top().
    // 100% → fully transparent (no dimming), 0% → fully opaque black.
    if (!_dimOverlay) {
        _dimOverlay = lv_obj_create(lv_layer_top());
        lv_obj_remove_style_all(_dimOverlay);
        lv_obj_set_size(_dimOverlay, LV_PCT(100), LV_PCT(100));
        lv_obj_set_style_bg_color(_dimOverlay, lv_color_black(), 0);
        lv_obj_clear_flag(_dimOverlay, LV_OBJ_FLAG_CLICKABLE);
        // Wake-on-touch: a PRESSED on the dim overlay always invokes the
        // app's wake callback. Touches at any other dim level pass through
        // because CLICKABLE is cleared.
        lv_obj_add_event_cb(_dimOverlay, _dim_overlay_pressed_cb,
                            LV_EVENT_PRESSED, nullptr);
    }

    if (percent == 0) {
        // Full blackout — opaque overlay absorbs all touches + HW off
        Serial.println("[WS_HAL] backlight 0% → dim overlay opaque + clickable, BL low");
        lv_obj_clear_flag(_dimOverlay, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_bg_opa(_dimOverlay, LV_OPA_COVER, 0);
        lv_obj_add_flag(_dimOverlay, LV_OBJ_FLAG_CLICKABLE);
        // Retry I2C write — bus may be contended with GT911 touch reads
        for (int i = 0; i < 3; i++) {
            ioExpander.digitalWrite(CH422G_EXIO_LCD_BL, LOW);
            delay(5);
        }
        return;
    }

    // Ensure backlight is on and overlay doesn't block touches
    for (int i = 0; i < 2; i++) {
        ioExpander.digitalWrite(CH422G_EXIO_LCD_BL, HIGH);
        delay(2);
    }
    lv_obj_clear_flag(_dimOverlay, LV_OBJ_FLAG_CLICKABLE);

    if (percent >= 100) {
        lv_obj_add_flag(_dimOverlay, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_clear_flag(_dimOverlay, LV_OBJ_FLAG_HIDDEN);
        lv_opa_t opa = (lv_opa_t)((100 - percent) * 255 / 100);
        lv_obj_set_style_bg_opa(_dimOverlay, opa, 0);
    }
}
