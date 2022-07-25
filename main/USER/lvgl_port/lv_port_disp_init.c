/**
 * @file lv_port_disp_templ.c
 *
 */

/*Copy this file as "lv_port_disp.c" and set this value to "1" to enable content*/

/*********************
 *      INCLUDES
 *********************/
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "screen_driver.h"
#include "lvgl.h"
#include "lv_port_disp_init.h"
#include <stdbool.h>
#include "oled_init.h"

/*********************
 *      DEFINES
 *********************/
#define CONFIG_BUF_MODE 2 // 0: one row a time; 1: two lines a time; 2: two frame a time
#define LVGL_TICK_MS 5
#define CONFIG_EVALUATE 1
#define DISP_HOR_RES    128
#define DISP_VER_RES    64

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static esp_err_t lv_port_disp_init(scr_driver_t *driver);
scr_driver_t oled_init(void);
void disp_enable_update(void);
void disp_disable_update(void);
static void disp_flush(lv_disp_drv_t *, const lv_area_t *, lv_color_t *);
static void lv_tick_timercb(void *);
static void gui_task(void *);
void lvgl_init(void);
void lvgl_acquire(void);
void lvgl_acquire(void);

/**********************
 *  STATIC VARIABLES
 **********************/
static const char *TAG_LVGL = "lvgl_gui";
static const char *TAG_OLED = "mono oled";
/* Creates a semaphore to handle concurrent call to lvgl stuff
 * If you wish to call *any* lvgl function from other threads/tasks
 * you should lock on the very same semaphore! */
static SemaphoreHandle_t xGuiSemaphore = NULL;
static TaskHandle_t g_lvgl_task_handle;


scr_driver_t oled;

/**********************
 *      MACROS
 **********************/
#define SIZE_TO_PIXEL(v) ((v) / sizeof(lv_color_t))
#define PIXEL_TO_SIZE(v) ((v) * sizeof(lv_color_t))

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

static esp_err_t lv_port_disp_init(scr_driver_t *driver)
{
    oled_init();
    // initiate the display in lv_port.c
    if (NULL == driver) {
        ESP_LOGE(TAG_LVGL, "Pointer of lcd driver is invalid");
        return ESP_ERR_INVALID_ARG;
    }

    /*-------------------------
     * Get information of the display
     * -----------------------*/
    scr_info_t info;
    driver->get_info(&info);
    uint16_t g_screen_width = info.width;
    uint16_t g_screen_height = info.height;

    /*-----------------------------
     * Create a buffer for drawing
     *----------------------------*/

    /**
     * LVGL requires a buffer where it internally draws the widgets.
     * Later this buffer will passed to your display driver's `flush_cb` to copy its content to your display.
     * The buffer has to be greater than 1 display row
     *
     * There are 3 buffering configurations:
     * 1. Create ONE buffer:
     *      LVGL will draw the display's content here and writes it to your display
     *
     * 2. Create TWO buffer:
     *      LVGL will draw the display's content to a buffer and writes it your display.
     *      You should use DMA to write the buffer's content to the display.
     *      It will enable LVGL to draw the next part of the screen to the other buffer while
     *      the data is being sent form the first buffer. It makes rendering and flushing parallel.
     *
     * 3. Double buffering
     *      Set 2 screens sized buffers and set disp_drv.full_refresh = 1.
     *      This way LVGL will always provide the whole rendered screen in `flush_cb`
     *      and you only need to change the frame buffer's address.
     */

    static int buffer_number;
#if (CONFIG_BUF_MODE==0)
    /* Example for 1) */ 
    static lv_color_t DISP_BUF_SIZE[DISP_HOR_RES * 8];
    static lv_disp_draw_buf_t draw_buf_dsc;
    static lv_color_t buf_1[DISP_HOR_RES * 8];                          /*A buffer for 10 rows*/
    //lv_disp_draw_buf_init(&draw_buf_dsc, buf_1, NULL, DISP_BUF_SIZE);   /*Initialize the display buffer*/
    buffer_number = 1;
#elif (CONFIG_BUF_MODE==1)
    /* Example for 2) */
    static lv_color_t DISP_BUF_SIZE[g_screen_width * 8];
    static lv_disp_draw_buf_t draw_buf_dsc;
    static lv_color_t buf_1[DISP_HOR_RES * 8];                        /*A buffer for 10 rows*/
    static lv_color_t buf_2[DISP_HOR_RES * 8];                        /*An other buffer for 10 rows*/
    //lv_disp_draw_buf_init(&draw_buf_dsc, buf_1, buf_2, DISP_BUF_SIZE);   /*Initialize the display buffer*/
    buffer_number = 2;
#elif (CONFIG_BUF_MODE==2)
    /* Example for 3) also set disp_drv.full_refresh = 1 below*/
    static int DISP_BUF_SIZE;
    DISP_BUF_SIZE = DISP_HOR_RES * 64;
    static lv_disp_draw_buf_t draw_buf_dsc;
    static lv_color_t buf_1[DISP_HOR_RES * DISP_VER_RES];            /*A screen sized buffer*/
    static lv_color_t buf_2[DISP_HOR_RES * DISP_VER_RES];            /*Another screen sized buffer*/
    //lv_disp_draw_buf_init(&draw_buf_dsc, buf_1, buf_2, DISP_BUF_SIZE);   /*Initialize the display buffer*/
    buffer_number = 2;
#endif

    /*-----------------------------------
     * Register the display in LVGL
     *----------------------------------*/

    static lv_disp_drv_t disp_drv;                         /*Descriptor of a display driver*/
    lv_disp_drv_init(&disp_drv);                    /*Basic initialization*/

    /*Set up the functions to access to your display*/

    /*Set the resolution of the display*/
    disp_drv.hor_res = g_screen_width;
    disp_drv.ver_res = g_screen_height;
    
    /*Used to copy the buffer's content to the display*/
    disp_drv.flush_cb = disp_flush;

    /*Set a display buffer*/
    disp_drv.draw_buf = &draw_buf_dsc;

    /*Required for Example 3)*/
#if (CONFIG_BUF_MODE==2)
    disp_drv.full_refresh = 1;
#endif

    /*Report memory usage*/
    size_t free_size = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    const size_t remain_size = 60 * 1024; /**< Remain for other functions */
    size_t alloc_pixel = DISP_BUF_SIZE;
    if (((buffer_number * PIXEL_TO_SIZE(alloc_pixel)) + remain_size) > free_size) {
        size_t allow_size = (free_size - remain_size) & 0xfffffffc;
        alloc_pixel = SIZE_TO_PIXEL(allow_size / buffer_number);
        ESP_LOGW(TAG_LVGL, "Exceeded max free size, force shrink to %u Byte", allow_size);
    }

    lv_color_t *buf1 = heap_caps_malloc(PIXEL_TO_SIZE(alloc_pixel), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (NULL == buf1) {
        ESP_LOGE(TAG_LVGL, "Display buffer memory not enough");
        return ESP_ERR_NO_MEM;
    }
#if (CONFIG_BUF_MODE != 0)
    lv_color_t *buf2 = heap_caps_malloc(PIXEL_TO_SIZE(alloc_pixel), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (NULL == buf2) {
        heap_caps_free(buf1);
        ESP_LOGE(TAG_LVGL, "Display buffer memory not enough");
        return ESP_ERR_NO_MEM;
    }
#endif

    ESP_LOGI(TAG_LVGL, "Alloc memory total size: %u Byte", buffer_number * PIXEL_TO_SIZE(alloc_pixel));

    static lv_disp_drv_t disp_buf;

#if (CONFIG_BUF_MODE == 0)
    lv_disp_draw_buf_init(&draw_buf_dsc, buf1, NULL, alloc_pixel);
#else
    lv_disp_draw_buf_init(&draw_buf_dsc, buf1, buf2, alloc_pixel);
#endif

    /* Fill a memory array with a color if you have GPU.
     * Note that, in lv_conf.h you can enable GPUs that has built-in support in LVGL.
     * But if you have a different GPU you can use with this callback.*/
    //disp_drv.gpu_fill_cb = gpu_fill;

    /*Finally register the driver*/
    lv_disp_drv_register(&disp_drv);
    return ESP_OK;
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/*Initialize your display and the required peripherals.*/
// scr_driver_t oled_init(void)
// {
//     scr_info_t oled_info;
//     i2c_port_t sda = 21;
//     i2c_port_t scl = 22;
//     uint32_t clk_speed = 400000;
//     uint16_t addr = 0x3C;

//     /*Configure i2c*/
//     i2c_config_t i2c_conf = {
//         .mode = I2C_MODE_MASTER,
//         .sda_io_num = sda, // 21
//         .sda_pullup_en = GPIO_PULLUP_ENABLE,
//         .scl_io_num = scl, // 22
//         .scl_pullup_en = GPIO_PULLUP_ENABLE,
//         .master.clk_speed = clk_speed,
//     };

//     i2c_bus_handle_t i2c_bus = i2c_bus_create(I2C_NUM_0, &i2c_conf);

//     scr_interface_i2c_config_t iface_cfg = {
//         .i2c_bus = i2c_bus,
//         .clk_speed = clk_speed,
//         .slave_addr = addr,
//     };

//     scr_interface_driver_t *iface_drv;
//     scr_interface_create(SCREEN_IFACE_I2C, &iface_cfg, &iface_drv);

//     scr_controller_config_t oled_cfg = {
//         .interface_drv = iface_drv,
//         .pin_num_rst = 0,
//         .pin_num_bckl = -1,
//         .rst_active_level = 0,
//         .bckl_active_level = 1,
//         .width = 128,
//         .height = 64,
//         .rotate = SCR_DIR_LRTB, // defined in screen_utility.c
//     };

//     scr_find_driver(SCREEN_CONTROLLER_SSD1306, &oled);
//     oled.init(&oled_cfg);

//     oled.get_info(&oled_info);

//     ESP_LOGI(TAG_OLED, "Screen name:%s | width:%d | height:%d", oled_info.name, oled_info.width, oled_info.height);
    
// #if CONFIG_EVALUATE
//     oled.draw_bitmap(0, 0, 128, 64, (uint16_t *)bmp_image_128_64);
//     vTaskDelay(100 / portTICK_PERIOD_MS);
//     uint8_t i = 0, j = 0;
//     for (i = 0; i < 128 - 52; i++) {
//         oled.draw_bitmap(i, j, 52, 24, (uint16_t *)bmp_image_52_24);
//         vTaskDelay(100 / portTICK_PERIOD_MS);
//     }
// #endif
//     return oled;
// }

volatile bool disp_flush_enabled = true;

/* Enable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_enable_update(void)
{
    disp_flush_enabled = true;
}

/* Disable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_disable_update(void)
{
    disp_flush_enabled = false;
}

/*Flush the content of the internal buffer the specific area on the display
 *You can use DMA or any hardware acceleration to do this operation in the background but
 *'lv_disp_flush_ready()' has to be called when finished.*/
static void disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
    if(disp_flush_enabled) {
        /*The most simple case (but also the slowest) to put all pixels to the screen one-by-one*/

        int32_t x;
        int32_t y;
        for(y = area->y1; y <= area->y2; y++) {
            for(x = area->x1; x <= area->x2; x++) {
                /*Put a pixel to the display. For example:*/
                /*put_px(x, y, *color_p)*/
                color_p++;
            }
        }
    }
    // or use this
    //disp_obj.draw_bitmap(area->x1, area->y1, (uint16_t)(area->x2 - area->x1 + 1), (uint16_t)(area->y2 - area->y1 + 1), (uint16_t *)color_map);

    /*IMPORTANT!!!
     *Inform the graphics library that you are ready with the flushing*/
    lv_disp_flush_ready(disp_drv);
}

/*OPTIONAL: GPU INTERFACE*/

/*If your MCU has hardware accelerator (GPU) then you can use it to fill a memory with a color*/
//static void gpu_fill(lv_disp_drv_t * disp_drv, lv_color_t * dest_buf, lv_coord_t dest_width,
//                    const lv_area_t * fill_area, lv_color_t color)
//{
//    /*It's an example code which should be done by your GPU*/
//    int32_t x, y;
//    dest_buf += dest_width * fill_area->y1; /*Go to the first line*/
//
//    for(y = fill_area->y1; y <= fill_area->y2; y++) {
//        for(x = fill_area->x1; x <= fill_area->x2; x++) {
//            dest_buf[x] = color;
//        }
//        dest_buf+=dest_width;    /*Go to the next line*/
//    }
//}

static void lv_tick_timercb(void *timer)
{
    lv_tick_inc(LVGL_TICK_MS);
}

static void gui_task(void *args)
{
    ESP_LOGI(TAG_LVGL, "Start to run LVGL");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));

        /* Try to take the semaphore, call lvgl related function on success */
        if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
            lv_task_handler();
            xSemaphoreGive(xGuiSemaphore);
        }
    }
}


void lvgl_init(void)
{
    esp_timer_handle_t tick_timer = NULL;

    /* Initialize LittlevGL */
    lv_init();

    /* Display interface */
    oled = oled_init();
    lv_port_disp_init(&oled);

#if defined(CONFIG_LVGL_DRIVER_TOUCH_SCREEN_ENABLE)
    if (NULL != touch_drv) {
        /* Input device interface */
        lvgl_indev_init(touch_drv);

    }
#endif

    esp_timer_create_args_t timer_conf = {
        .callback = lv_tick_timercb,
        .name     = "lv_tick_timer"
    };
    esp_timer_create(&timer_conf, &tick_timer);

    xGuiSemaphore = xSemaphoreCreateMutex();

#if CONFIG_FREERTOS_UNICORE == 0
    xTaskCreatePinnedToCore(gui_task, "lv gui", 1024 * 8, NULL, 5, &g_lvgl_task_handle, 1);
#else
    xTaskCreatePinnedToCore(gui_task, "lv gui", 1024 * 8, NULL, 5, &g_lvgl_task_handle, 0);
#endif

    esp_timer_start_periodic(tick_timer, LVGL_TICK_MS * 1000U);
}

void lvgl_acquire(void)
{
    TaskHandle_t task = xTaskGetCurrentTaskHandle();
    if (g_lvgl_task_handle != task) {
        xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
    }
}

void lvgl_release(void)
{
    TaskHandle_t task = xTaskGetCurrentTaskHandle();
    if (g_lvgl_task_handle != task) {
        xSemaphoreGive(xGuiSemaphore);
    }
}