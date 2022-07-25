/*********************
 *      INCLUDES
 *********************/

#include "lvgl_port_disp_init.h"

/* Creates a semaphore to handle concurrent call to lvgl stuff
 * If you wish to call *any* lvgl function from other threads/tasks
 * you should lock on the very same semaphore! */
static SemaphoreHandle_t xGuiSemaphore = NULL;
static TaskHandle_t g_lvgl_task_handle;



/*Write the internal buffer (VDB) to the display. 'lv_flush_ready()' has to be called when finished*/
static void ex_disp_flush(lv_disp_drv_t *drv, scr_driver_t lcd_obj, const lv_area_t *area, lv_color_t *color_map)
{
    lcd_obj.draw_bitmap(area->x1, area->y1, (uint16_t)(area->x2 - area->x1 + 1), (uint16_t)(area->y2 - area->y1 + 1), (uint16_t *)color_map);

    /* IMPORTANT!!!
     * Inform the graphics library that you are ready with the flushing*/
    lv_disp_flush_ready(drv);
}

static esp_err_t lvgl_port_display_init(scr_driver_t *display_driver)
{
    if (NULL == display_driver) {
        ESP_LOGE(TAG_LVGL, "Pointer of lcd driver is invalid");
        return ESP_ERR_INVALID_ARG;
    }

    // scr_driver_t display_d;
    // display_d = *display_driver;
    // scr_info_t info;
    // display_d.get_info(&info);
    // uint16_t g_screen_width = info.width;
    // uint16_t g_screen_height = info.height;

    lv_disp_drv_t disp_drv;      /*Descriptor of a display driver*/
    lv_disp_drv_init(&disp_drv); /*Basic initialization*/
    disp_drv.hor_res = 128; //g_screen_width;
    disp_drv.ver_res = 64; //g_screen_height;

    disp_drv.flush_cb = ex_disp_flush; /*Used in buffered mode (LV_VDB_SIZE != 0  in lv_conf.h)*/

    size_t free_size = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    const size_t remain_size = 60 * 1024; /**< Remain for other functions */
    size_t alloc_pixel = DISP_BUF_SIZE;
    if (((BUFFER_NUMBER * PIXEL_TO_SIZE(alloc_pixel)) + remain_size) > free_size) {
        size_t allow_size = (free_size - remain_size) & 0xfffffffc;
        alloc_pixel = SIZE_TO_PIXEL(allow_size / BUFFER_NUMBER);
        ESP_LOGW(TAG_LVGL, "Exceeded max free size, force shrink to %u Byte", allow_size);
    }

    lv_color_t *buf1 = heap_caps_malloc(PIXEL_TO_SIZE(alloc_pixel), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (NULL == buf1) {
        ESP_LOGE(TAG_LVGL, "Display buffer memory not enough");
        return ESP_ERR_NO_MEM;
    }
#if (BUFFER_NUMBER == 2)
    lv_color_t *buf2 = heap_caps_malloc(PIXEL_TO_SIZE(alloc_pixel), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (NULL == buf2) {
        heap_caps_free(buf1);
        ESP_LOGE(TAG_LVGL, "Display buffer memory not enough");
        return ESP_ERR_NO_MEM;
    }
#endif

    ESP_LOGI(TAG_LVGL, "Alloc memory total size: %u Byte", BUFFER_NUMBER * PIXEL_TO_SIZE(alloc_pixel));

    static lv_disp_drv_t disp_buf;

#if (BUFFER_NUMBER == 2)
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, alloc_pixel);
#else
    lv_disp_buf_init(&disp_buf, buf1, NULL, alloc_pixel);
#endif

    disp_drv.draw_buf = &disp_buf;

    /* Finally register the driver */
    lv_disp_drv_register(&disp_drv);
    return ESP_OK;
}