#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"

// Self-defined lib
#include "keys_on_board.h"
#include "LED_on_board.h"
#include "iot_button.h"
#include "unity.h"

#define ESP_INTR_FLAG_DEFAULT 0

/******************** configure on board LED ********************/
static led_indicator_handle_t led_handle_0 = NULL; // led on board
void led_indicator_init()
{
    led_indicator_config_t config = {
        .off_level = 0,
        .mode = LED_GPIO_MODE,
    };
    led_handle_0 = led_indicator_create(LED_GPIO, &config);
}

/******************** configure on board OLED ********************/
#include "screen_driver.h"
#include "ssd1306.h"

static const char *TAG_OLED = "mono oled";

static const unsigned char bmp_image_128_64[1024] = {
    0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0XF8,
    0XFE, 0XFE, 0X3E, 0X78, 0X70, 0X70, 0X78, 0X3E, 0XFE, 0XF8, 0XF8, 0X00, 0XF8, 0XF8, 0X00, 0X00,
    0XF8, 0XF8, 0X00, 0X00, 0X00, 0X00, 0XF8, 0XFE, 0XFE, 0X0E, 0X0E, 0X0E, 0X00, 0X00, 0X00, 0XB0,
    0XB0, 0XF0, 0XF0, 0XC0, 0X00, 0XF0, 0XF0, 0XF0, 0X40, 0X70, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
    0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X0E, 0X31, 0X46, 0X31, 0X31, 0X0E,
    0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
    0X00, 0X00, 0X00, 0X00, 0X38, 0X46, 0X46, 0X86, 0X08, 0X86, 0X46, 0X46, 0X38, 0X00, 0X00, 0X00,
    0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
    0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X1F,
    0X1F, 0X1F, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X1F, 0X1F, 0X1F, 0X00, 0X43, 0X5F, 0X5C, 0X5C,
    0X7F, 0X3F, 0X00, 0X00, 0X00, 0X00, 0X03, 0X1F, 0X1F, 0X1F, 0X1F, 0X1F, 0X00, 0X00, 0X03, 0X1F,
    0X1C, 0X1F, 0X1F, 0X1F, 0X00, 0X1F, 0X1F, 0X1F, 0X00, 0X00, 0X00, 0X00, 0X00, 0X1F, 0X1F, 0X00,
    0X00, 0X1F, 0X1F, 0X00, 0X1F, 0X1F, 0X1F, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
    0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0XC0, 0X20, 0X20, 0X1C, 0X03, 0X00, 0X00, 0X00, 0X00, 0X00,
    0X00, 0X00, 0X80, 0X00, 0X80, 0X00, 0X00, 0X00, 0X03, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
    0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
    0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0XE0,
    0X18, 0X18, 0X18, 0X20, 0X18, 0X18, 0X18, 0XE0, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
    0X00, 0X00, 0X38, 0XC4, 0XC4, 0X18, 0XC4, 0X38, 0X38, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
    0XC0, 0XC0, 0XC0, 0XE0, 0X20, 0X20, 0XF8, 0XF8, 0XF8, 0XD8, 0XF8, 0XF8, 0XF8, 0X3C, 0X3C, 0X3C,
    0X3C, 0X3C, 0X3C, 0X3C, 0X3C, 0X3C, 0X3C, 0XFC, 0X3C, 0X3C, 0X3C, 0XDC, 0XDC, 0XD8, 0XD8, 0XD8,
    0X38, 0X3C, 0X3C, 0X3C, 0XE4, 0XE2, 0XE1, 0XC0, 0XC0, 0XC0, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
    0X02, 0X02, 0X05, 0X1A, 0X05, 0X02, 0X02, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
    0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
    0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
    0X01, 0X01, 0X02, 0X0C, 0X02, 0X02, 0X01, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
    0X00, 0X00, 0X00, 0X00, 0X00, 0X81, 0XE0, 0XF0, 0XF0, 0X70, 0X1C, 0X0C, 0X7E, 0X7E, 0XF3, 0XFF,
    0XFD, 0X1E, 0X1E, 0X0E, 0X01, 0X81, 0X8D, 0X8D, 0X0C, 0X0C, 0X0C, 0X0C, 0X0C, 0X0C, 0X0C, 0X0C,
    0X0C, 0X0C, 0X0C, 0X0C, 0X00, 0X00, 0X80, 0XFF, 0XFE, 0XFE, 0XFE, 0XFC, 0XE0, 0X80, 0X80, 0X10,
    0X11, 0X01, 0X83, 0X83, 0X82, 0X8C, 0X9C, 0XF1, 0XF1, 0XE1, 0X83, 0X0F, 0X0F, 0X0E, 0X1E, 0X7C,
    0X7C, 0X7C, 0XF0, 0XE0, 0X80, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X0E, 0X11, 0X11,
    0X62, 0X11, 0X0E, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
    0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
    0X00, 0X00, 0X80, 0X80, 0XC0, 0XC0, 0XC0, 0X70, 0X70, 0X78, 0X78, 0X78, 0X78, 0X38, 0X78, 0X78,
    0X4E, 0X4E, 0X4E, 0X0E, 0X0E, 0X8F, 0X87, 0XB9, 0XB9, 0XBE, 0X3E, 0X3E, 0X3E, 0X3E, 0X3F, 0X3F,
    0X3F, 0X3E, 0X3E, 0X0E, 0X0E, 0X0F, 0X0F, 0X0F, 0X0E, 0X0E, 0X0E, 0X0E, 0X0E, 0X0E, 0X0E, 0X0E,
    0X0E, 0X0E, 0X0E, 0X0E, 0X8E, 0X8E, 0XCF, 0X8F, 0X8F, 0X8F, 0X8F, 0X0F, 0X0F, 0X0F, 0X0F, 0X0E,
    0X0E, 0X0E, 0X0F, 0X0F, 0X0F, 0X0F, 0X0F, 0X8F, 0X8F, 0X87, 0X87, 0X46, 0X46, 0X40, 0X00, 0X30,
    0X30, 0X30, 0X31, 0X37, 0X3F, 0X3E, 0X3E, 0X3E, 0X38, 0XF8, 0XC0, 0XC0, 0XC0, 0X00, 0X00, 0X00,
    0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
    0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X80, 0XE0, 0XF8,
    0X64, 0X64, 0X67, 0X1F, 0X03, 0X03, 0X00, 0X80, 0XC0, 0XE0, 0XE0, 0XE0, 0XE0, 0XF8, 0XF8, 0XF8,
    0XE0, 0XE0, 0XE0, 0XC0, 0XC0, 0X80, 0X23, 0X43, 0X43, 0X84, 0X1F, 0X78, 0XC0, 0XC0, 0X00, 0X00,
    0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
    0X00, 0X00, 0X00, 0X00, 0X03, 0X03, 0X03, 0X03, 0X03, 0X03, 0XFF, 0X00, 0X00, 0X00, 0X00, 0X00,
    0X00, 0XC0, 0XF8, 0XF8, 0X78, 0X1C, 0XC7, 0X63, 0X63, 0XF8, 0XF8, 0XF8, 0XF8, 0XE0, 0XE0, 0XF8,
    0XF8, 0XF8, 0XF8, 0XF8, 0XF8, 0XE0, 0XE0, 0XE0, 0XC0, 0X80, 0X83, 0X83, 0X07, 0X1F, 0XBC, 0XBC,
    0XF8, 0XE0, 0XC0, 0X80, 0X80, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
    0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X3E, 0X3E, 0XFF, 0XFB, 0XFF,
    0X3E, 0X3E, 0X06, 0X02, 0XFC, 0XFC, 0XFE, 0XFF, 0XFF, 0XFF, 0XFF, 0XFB, 0X03, 0X07, 0X07, 0X07,
    0X03, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFE, 0XFC, 0XFC, 0X01, 0X00, 0X00, 0XFF, 0XFF, 0XC0, 0X00,
    0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
    0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0XFF, 0X00, 0X00, 0X00, 0X00, 0X00,
    0X00, 0XC7, 0X3F, 0X3F, 0X3C, 0X06, 0X03, 0XFC, 0XFC, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0X03,
    0X07, 0X07, 0X07, 0X03, 0X3F, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFE, 0X06, 0XFF, 0XFF,
    0XFF, 0XFF, 0X3F, 0X07, 0X07, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
    0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X01, 0X03, 0X8F,
    0X8E, 0X8E, 0XE0, 0XE0, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFE, 0XFE, 0X8E, 0X82, 0XF1, 0XF1, 0XF1,
    0X80, 0XFC, 0XFE, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFE, 0XF0, 0XF0, 0XF1, 0XF1, 0XF1, 0XF0,
    0XF0, 0XF2, 0XF2, 0XF2, 0XF2, 0XF2, 0XF2, 0XF2, 0XF2, 0XF2, 0XF2, 0XF2, 0XF2, 0XF2, 0XF2, 0XF2,
    0XF2, 0XF3, 0XF3, 0XF3, 0XF3, 0XF3, 0XF3, 0XF3, 0XF3, 0XF3, 0XF1, 0XF0, 0XF0, 0XF1, 0XF1, 0XF1,
    0XF1, 0XF1, 0XF0, 0XF0, 0XF0, 0XF0, 0XFE, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFE, 0XFE, 0XE1,
    0XF1, 0XF1, 0XF1, 0XE0, 0XEC, 0XFE, 0XFE, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XF3, 0XF2, 0XE3, 0XE3,
    0XE3, 0XE1, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
};
static const unsigned char bmp_image_52_24[156] = { /* 0X02,0X01,0X34,0X00,0X18,0X00, */
    0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X1E, 0X82, 0X86, 0X02, 0X1E, 0X06, 0X38, 0X1E, 0X80, 0X06,
    0X1B, 0X1B, 0X00, 0X1A, 0X1C, 0X00, 0X9C, 0X80, 0X80, 0X98, 0X98, 0X80, 0X98, 0X80, 0X83, 0X81,
    0X80, 0X80, 0X00, 0X20, 0X08, 0X00, 0X00, 0X40, 0XC0, 0X04, 0X00, 0X04, 0X00, 0X00, 0X00, 0X00,
    0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X01, 0X80, 0XC0, 0X41, 0X60, 0X60,
    0X20, 0X70, 0X31, 0X18, 0XAC, 0X24, 0X3E, 0X3E, 0X27, 0X31, 0X31, 0X25, 0X35, 0X34, 0X34, 0X20,
    0XF0, 0XBF, 0X3C, 0X39, 0X25, 0X32, 0X39, 0XBD, 0X1A, 0X42, 0X04, 0X2C, 0X38, 0X30, 0X20, 0XC0,
    0X80, 0X02, 0X06, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X3C, 0X3F, 0X87,
    0XF8, 0XFC, 0XFE, 0XA6, 0X87, 0XDE, 0XFE, 0XFC, 0XE0, 0XC1, 0XDE, 0XC0, 0XE0, 0XE0, 0XE0, 0XE0,
    0XE0, 0XE0, 0XA0, 0XA0, 0XA0, 0XA0, 0XC0, 0XC0, 0XC0, 0XDE, 0XC9, 0XE2, 0XFF, 0XFF, 0XDE, 0X87,
    0X87, 0XFE, 0XFE, 0XFC, 0XAD, 0XBF, 0X1E, 0X00, 0X00, 0X00, 0X00, 0X00
};

scr_driver_t oled_init()
{
    scr_driver_t oled;
    scr_info_t oled_info;
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 21,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = 22,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    i2c_bus_handle_t i2c_bus = i2c_bus_create(I2C_NUM_0, &i2c_conf);
    TEST_ASSERT_NOT_NULL(i2c_bus);

    scr_interface_i2c_config_t iface_cfg = {
        .i2c_bus = i2c_bus,
        .clk_speed = 400000,
        .slave_addr = 0x3C,
    };

    scr_interface_driver_t *iface_drv;
    TEST_ASSERT(ESP_OK == scr_interface_create(SCREEN_IFACE_I2C, &iface_cfg, &iface_drv));
    scr_controller_config_t oled_cfg = {0};
    oled_cfg.interface_drv = iface_drv,
    oled_cfg.pin_num_rst = 0,
    oled_cfg.pin_num_bckl = -1,
    oled_cfg.rst_active_level = 0,
    oled_cfg.bckl_active_level = 1,
    oled_cfg.width = 128;
    oled_cfg.height = 64;
    oled_cfg.rotate = SCR_DIR_LRTB; // defined in screen_utility.c
    scr_find_driver(SCREEN_CONTROLLER_SSD1306, &oled);
    oled.init(&oled_cfg);

    oled.get_info(&oled_info);
    ESP_LOGI(TAG_OLED, "Screen name:%s | width:%d | height:%d", oled_info.name, oled_info.width, oled_info.height);
    oled.draw_bitmap(0, 0, 128, 64, (uint16_t *)bmp_image_128_64);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    uint8_t i = 0, j = 0;
    for (i = 0; i < 128 - 52; i++) {
        oled.draw_bitmap(i, j, 52, 24, (uint16_t *)bmp_image_52_24);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    return oled;
}

/******************** configure lvgl user interface ********************/
/* LVGL includes */
#include "lvgl.h"
static const char *TAG_LVGL = "lvgl";
/* Creates a semaphore to handle concurrent call to lvgl stuff
 * If you wish to call *any* lvgl function from other threads/tasks
 * you should lock on the very same semaphore! */
static SemaphoreHandle_t xGuiSemaphore = NULL;
static TaskHandle_t g_lvgl_task_handle;

#define LVGL_TICK_MS 5

/*Write the internal buffer (VDB) to the display. 'lv_flush_ready()' has to be called when finished*/
static void ex_disp_flush(lv_disp_drv_t *drv, scr_driver_t lcd_obj, const lv_area_t *area, lv_color_t *color_map)
{
    lcd_obj.draw_bitmap(area->x1, area->y1, (uint16_t)(area->x2 - area->x1 + 1), (uint16_t)(area->y2 - area->y1 + 1), (uint16_t *)color_map);

    /* IMPORTANT!!!
     * Inform the graphics library that you are ready with the flushing*/
    lv_disp_flush_ready(drv);
}

#define DISP_BUF_SIZE  (g_screen_width * 64)
#define SIZE_TO_PIXEL(v) ((v) / sizeof(lv_color_t))
#define PIXEL_TO_SIZE(v) ((v) * sizeof(lv_color_t))
#define BUFFER_NUMBER (2)

// static esp_err_t lvgl_display_init(scr_driver_t *driver, scr_driver_t lcd_obj)
// {
//     if (NULL == driver) {
//         ESP_LOGE(TAG_LVGL, "Pointer of lcd driver is invalid");
//         return ESP_ERR_INVALID_ARG;
//     }

//     lcd_obj = *driver;
//     scr_info_t info;
//     lcd_obj.get_info(&info);
//     uint16_t g_screen_width = info.width;
//     uint16_t g_screen_height = info.height;

//     lv_disp_drv_t disp_drv;      /*Descriptor of a display driver*/
//     lv_disp_drv_init(&disp_drv); /*Basic initialization*/
//     disp_drv.hor_res = g_screen_width;
//     disp_drv.ver_res = g_screen_height;

//     disp_drv.flush_cb = ex_disp_flush; /*Used in buffered mode (LV_VDB_SIZE != 0  in lv_conf.h)*/

//     size_t free_size = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
//     const size_t remain_size = 60 * 1024; /**< Remain for other functions */
//     size_t alloc_pixel = DISP_BUF_SIZE;
//     if (((BUFFER_NUMBER * PIXEL_TO_SIZE(alloc_pixel)) + remain_size) > free_size) {
//         size_t allow_size = (free_size - remain_size) & 0xfffffffc;
//         alloc_pixel = SIZE_TO_PIXEL(allow_size / BUFFER_NUMBER);
//         ESP_LOGW(TAG_LVGL, "Exceeded max free size, force shrink to %u Byte", allow_size);
//     }

//     lv_color_t *buf1 = heap_caps_malloc(PIXEL_TO_SIZE(alloc_pixel), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
//     if (NULL == buf1) {
//         ESP_LOGE(TAG_LVGL, "Display buffer memory not enough");
//         return ESP_ERR_NO_MEM;
//     }
// #if (BUFFER_NUMBER == 2)
//     lv_color_t *buf2 = heap_caps_malloc(PIXEL_TO_SIZE(alloc_pixel), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
//     if (NULL == buf2) {
//         heap_caps_free(buf1);
//         ESP_LOGE(TAG_LVGL, "Display buffer memory not enough");
//         return ESP_ERR_NO_MEM;
//     }
// #endif

//     ESP_LOGI(TAG_LVGL, "Alloc memory total size: %u Byte", BUFFER_NUMBER * PIXEL_TO_SIZE(alloc_pixel));

//     static lv_disp_buf_t disp_buf;

// #if (BUFFER_NUMBER == 2)
//     lv_disp_buf_init(&disp_buf, buf1, buf2, alloc_pixel);
// #else
//     lv_disp_buf_init(&disp_buf, buf1, NULL, alloc_pixel);
// #endif

//     disp_drv.buffer = &disp_buf;

//     /* Finally register the driver */
//     lv_disp_drv_register(&disp_drv);
//     return ESP_OK;
// }


/******************** configure on board iot buttons ********************/
#define BUTTON_ACTIVE_LEVEL   0
#define BUTTON_NUM 2
static const char *TAG_BUTTON = "iot button";
static button_handle_t g_btns[BUTTON_NUM] = {0};

static char* get_btn_index(button_handle_t btn)
{
    static char *which_button = "";
    for (size_t i = 0; i < BUTTON_NUM; i++) {
        if (btn == g_btns[i]) {
            if (i == 0)
                which_button = " Left";
            else if (i == 1)
                which_button = " Right";
            else
                which_button = "-1";
            return which_button;
        }
    }
    which_button ="-1";
    return which_button;
}

static void button_press_down_cb(void *arg)
{
    //TEST_ASSERT_EQUAL_HEX(BUTTON_PRESS_DOWN, iot_button_get_event(arg));
    ESP_LOGI(TAG_BUTTON, "BTN%s: BUTTON_PRESS_DOWN", get_btn_index((button_handle_t)arg));
}

static void button_press_up_cb(void *arg)
{
    //TEST_ASSERT_EQUAL_HEX(BUTTON_PRESS_UP, iot_button_get_event(arg));
    ESP_LOGI(TAG_BUTTON, "BTN%s: BUTTON_PRESS_UP", get_btn_index((button_handle_t)arg));
}

static void button_single_click_cb(void *arg)
{
    //TEST_ASSERT_EQUAL_HEX(BUTTON_SINGLE_CLICK, iot_button_get_event(arg));
    ESP_LOGI(TAG_BUTTON, "BTN%s: BUTTON_SINGLE_CLICK", get_btn_index((button_handle_t)arg));
    led_indicator_start(led_handle_0, BLINK_SINGLE_CLICK);
    vTaskDelay(500 / portTICK_RATE_MS);
    led_indicator_stop(led_handle_0, BLINK_SINGLE_CLICK);
}

static void button_double_click_cb(void *arg)
{
    //TEST_ASSERT_EQUAL_HEX(BUTTON_DOUBLE_CLICK, iot_button_get_event(arg));
    led_indicator_start(led_handle_0, BLINK_DOUBLE_CLICK);
    vTaskDelay(500 / portTICK_RATE_MS);
    led_indicator_stop(led_handle_0, BLINK_DOUBLE_CLICK);
    ESP_LOGI(TAG_BUTTON, "BTN%s: BUTTON_DOUBLE_CLICK", get_btn_index((button_handle_t)arg));
}

static void button_press_repeat_cb(void *arg)
{
    //led_indicator_start(led_handle_0, BLINK_SINGLE_CLICK);
    ESP_LOGI(TAG_BUTTON, "BTN%s: BUTTON_PRESS_REPEAT[%d]", get_btn_index((button_handle_t)arg), iot_button_get_repeat((button_handle_t)arg));
}

static void button_long_press_start_cb(void *arg)
{
    //TEST_ASSERT_EQUAL_HEX(BUTTON_LONG_PRESS_START, iot_button_get_event(arg));
    ESP_LOGI(TAG_BUTTON, "BTN%s: BUTTON_LONG_PRESS_START", get_btn_index((button_handle_t)arg));
}

static void button_long_press_hold_cb(void *arg)
{
    //TEST_ASSERT_EQUAL_HEX(BUTTON_LONG_PRESS_HOLD, iot_button_get_event(arg));
    led_indicator_start(led_handle_0, BLINK_LONG_PRESS);
    // vTaskDelay(1000 / portTICK_RATE_MS);
    // led_indicator_stop(led_handle_0, BLINK_DOUBLE_CLICK);
    ESP_LOGI(TAG_BUTTON, "BTN%s: BUTTON_LONG_PRESS_HOLD", get_btn_index((button_handle_t)arg));
}

void iot_buttons_init(void)
{
    button_config_t left_cfg = {
        .type = BUTTON_TYPE_GPIO,
        .gpio_button_config = {
            .gpio_num = LEFT_KEY_GPIO,
            .active_level = 0,
        },
    };
    button_config_t right_cfg = {
        .type = BUTTON_TYPE_GPIO,
        .gpio_button_config = {
            .gpio_num = RIGHT_KEY_GPIO,
            .active_level = 0,
        },
    };
    g_btns[0] = iot_button_create(&left_cfg);
    g_btns[1] = iot_button_create(&right_cfg);
    for (int i = 0; i<2; i++)
    {
    //TEST_ASSERT_NOT_NULL(g_btns[i]);
    iot_button_register_cb(g_btns[i], BUTTON_PRESS_DOWN, button_press_down_cb ,NULL);
    iot_button_register_cb(g_btns[i], BUTTON_PRESS_UP, button_press_up_cb ,NULL);
    iot_button_register_cb(g_btns[i], BUTTON_SINGLE_CLICK, button_single_click_cb, NULL);
    iot_button_register_cb(g_btns[i], BUTTON_DOUBLE_CLICK, button_double_click_cb, NULL);
    iot_button_register_cb(g_btns[i], BUTTON_PRESS_REPEAT, button_press_repeat_cb ,NULL);
    iot_button_register_cb(g_btns[i], BUTTON_LONG_PRESS_START, button_long_press_start_cb ,NULL);
    iot_button_register_cb(g_btns[i], BUTTON_LONG_PRESS_HOLD, button_long_press_hold_cb ,NULL);
    }
}

void app_main(void)
{
/******************** Initialize on board gpio keys ********************/
    //init_keys();  // ordinary gpio keys

/******************** Initialize on board led ********************/
    led_indicator_init();

/******************** Initialize on board iot buttons ********************/
    iot_buttons_init();

/******************** Initialize on board oled ********************/
    //scr_driver_t *oled = &
    oled_init();

/******************** Configure LittlevGL GUI ********************/


    int cnt = 0;
    while (1)
    {
        printf("cnt: %d\n", cnt++);
        printf("Current state: %s\n\n", led_state() ? "On" : "Off");
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}

/* using free RTOS */
// xQueueHandle keys_evt_queue = NULL;

// // Define ISR handler of keys
// static void IRAM_ATTR keys_isr_handler(void* arg)
// {
//     //xTimerResetFromISR(key_timer_handle, NULL);
//     uint32_t key_gpio_num = (uint32_t)arg;
//     xQueueSendFromISR(keys_evt_queue, &key_gpio_num, NULL);
// }

// static void key_task_callback(void* arg)
// {
//     uint32_t io_num;
//     //bool curr_led_state;
//     for (;;) {
//         if (xQueueReceive(keys_evt_queue, &io_num, portMAX_DELAY)) {
//             //printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
//             if (io_num == LEFT_KEY_GPIO)
//             {
//                 //curr_led_state = gpio_get_level(LED_GPIO);//LED_GPIO
//                 //reverse_led(); // change the state of led
//                 led_indicator_start(led_handle_0, BLINK_SINGLE_CLICK);
//                 vTaskDelay(1000 / portTICK_RATE_MS);
//                 led_indicator_stop(led_handle_0, BLINK_SINGLE_CLICK);
//                 printf("Reverse led\n");
//             }
//             else if (io_num == RIGHT_KEY_GPIO)
//             {
//                 //turn_off_led(); // change the state of led
//                 led_indicator_start(led_handle_0, BLINK_DOUBLE_CLICK);
//                 vTaskDelay(1000 / portTICK_RATE_MS);
//                 led_indicator_stop(led_handle_0, BLINK_DOUBLE_CLICK);
//                 printf("Turn off led\n");
//             }
//         }
//     }
// }


// void app_main(void)
// {
//     /* Configure the gpio pins of keys on board */
//     init_keys();

//     /* Configure the LED on board */
//     led_indicator_init();


//     //change gpio intrrupt type for left key pin
//     //gpio_set_intr_type(LEFT_KEY_GPIO, GPIO_INTR_POSEDGE);

//     //create a queue to handle gpio event from isr
//     keys_evt_queue = xQueueCreate(10, sizeof(uint32_t));
//     //start gpio task
//     xTaskCreate(key_task_callback, "key_gpio_task", 2048, NULL, 10, NULL);

//     //install gpio isr service
//     gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
//     //hook isr handler for left gpio pin
//     gpio_isr_handler_add(LEFT_KEY_GPIO, keys_isr_handler, (void*)LEFT_KEY_GPIO);
//     //hook isr handler for right gpio pin
//     gpio_isr_handler_add(RIGHT_KEY_GPIO, keys_isr_handler, (void*)RIGHT_KEY_GPIO);
//     printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());
//     printf("Keys' gpio pins configured!\n\n");

//     int cnt = 0;
//     while (1)
//     {
//         printf("cnt: %d\n\n", cnt++);
//         printf("Current state: %s\n", led_state() ? "On" : "Off");
//         vTaskDelay(1000 / portTICK_RATE_MS);
//     }

// }