/**
 * @file lv_port_disp_templ.h
 *
 */

/*Copy this file as "lv_port_disp.h" and set this value to "1" to enable content*/
#if 1

#ifndef LV_PORT_DISP_TEMPL_H
#define LV_PORT_DISP_TEMPL_H

// #ifdef __cplusplus
// extern "C" {
// #endif

/*********************
 *      INCLUDES
 *********************/
#if defined(LV_LVGL_H_INCLUDE_SIMPLE)
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/
/* Initialize low level display driver */
/**
 * @brief Initialize lvgl and register driver to lvgl
 * 
 * @note  This function will create a task to run the lvgl handler. The task will always Pinned on the APP_CPU
 * 
 * @param disp_drv Pointer of display driver
 * @param touch_drv  Pointer of touch driver. If you don't have a touch panel, set it to NULL
 * 
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_TIMEOUT Operation timeout
 */

void lvgl_init(void);

/* Enable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_enable_update(void);

/* Disable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_disable_update(void);

/**
 * @brief Acquire lock for LVGL
 * 
 * @note If you wish to call any lvgl function from other threads/tasks you should lock on the very same semaphore
 * 
 */
void lvgl_acquire(void);

/**
 * @brief Release lock for LVGL
 * 
 */
void lvgl_release(void);

/**********************
 *      MACROS
 **********************/

// #ifdef __cplusplus
// } /*extern "C"*/
// #endif

#endif /*LV_PORT_DISP_TEMPL_H*/

#endif /*Disable/Enable content*/
