/******************** 
 * configure on board OLED
 ********************/
#include "screen_driver.h"
#include "ssd1306.h"
#include "esp_log.h"
#include "oled_init.h"

#define CONFIG_EVALUATE 1
static const char *TAG_OLED = "mono oled";
static const unsigned char bmp_image_128_64[1024];
static const unsigned char bmp_image_52_24[156];

scr_driver_t oled;

scr_driver_t oled_init(void)
{
    scr_info_t oled_info;
    i2c_port_t sda = 21;
    i2c_port_t scl = 22;
    uint32_t clk_speed = 400000;
    uint16_t addr = 0x3C;

    /*Configure i2c*/
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda, // 21
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = scl, // 22
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = clk_speed,
    };

    i2c_bus_handle_t i2c_bus = i2c_bus_create(I2C_NUM_0, &i2c_conf);

    scr_interface_i2c_config_t iface_cfg = {
        .i2c_bus = i2c_bus,
        .clk_speed = clk_speed,
        .slave_addr = addr,
    };

    scr_interface_driver_t *iface_drv;
    scr_interface_create(SCREEN_IFACE_I2C, &iface_cfg, &iface_drv);

    scr_controller_config_t oled_cfg = {
        .interface_drv = iface_drv,
        .pin_num_rst = 0,
        .pin_num_bckl = -1,
        .rst_active_level = 0,
        .bckl_active_level = 1,
        .width = 128,
        .height = 64,
        .rotate = SCR_DIR_LRTB, // defined in screen_utility.c
    };

    scr_find_driver(SCREEN_CONTROLLER_SSD1306, &oled);
    oled.init(&oled_cfg);

    oled.get_info(&oled_info);

    ESP_LOGI(TAG_OLED, "Screen name:%s | width:%d | height:%d", oled_info.name, oled_info.width, oled_info.height);
    
#if CONFIG_EVALUATE
    oled.draw_bitmap(0, 0, 128, 64, (uint16_t *)bmp_image_128_64);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    uint8_t i = 0, j = 0;
    for (i = 0; i < 128 - 52; i++) {
        oled.draw_bitmap(i, j, 52, 24, (uint16_t *)bmp_image_52_24);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
#endif
    return oled;
}

#if CONFIG_EVALUATE
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
#endif