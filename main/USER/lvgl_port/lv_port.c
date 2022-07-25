#include "lv_port.h"
#include "lvgl.h"
#include "esp_log.h"
#include "oled_init.h"
#include "lv_port_disp_init.h"

void lvgl_port_init(void)
{
    lvgl_init();

#ifdef CONFIG_ENCODER
    //lvgl_port_indev_init();
#endif

#ifdef CONFIG_FILE_SYSTEM
    //lvgl_port_fs_init();
#endif

}