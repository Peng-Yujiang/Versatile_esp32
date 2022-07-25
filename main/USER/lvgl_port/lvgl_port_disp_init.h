#ifndef LVGL_PORT_DISP_INIT
#define LVGL_PORT_DISP_INIT
#include "lvgl.h"
#include "esp_log.h"

static const char *TAG_LVGL = "lvgl";

#define LVGL_TICK_MS 5

// #define DISP_BUF_SIZE  (g_screen_width * 64)
// #define SIZE_TO_PIXEL(v) ((v) / sizeof(lv_color_t))
// #define PIXEL_TO_SIZE(v) ((v) * sizeof(lv_color_t))
// #define BUFFER_NUMBER (2)

static void ex_disp_flush(lv_disp_drv_t *, scr_driver_t, const lv_area_t *, lv_color_t *);

static esp_err_t lvgl_port_display_init(scr_driver_t *);

#endif