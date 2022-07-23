// Copyright 2020-2021 Espressif Systems (Shanghai) CO LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// defines the left and right keys on esp32 board
// Modified by Yujiang Peng
// Juli 22, 2022

#include <stdbool.h>
#include <string.h>
#include <sys/queue.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"

#include "LED_on_board.h"

static const char* TAG_LED = "On board LED";

/*********************************** original ***********************************/

/**
 * Brief: Driver for on board LED
 * GPIO23: On board LED
 */
#if CONFIG_KEY_CONTROL_LED
void turn_on_led(void)
{
    ///* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(LED_GPIO, 1);
}

void turn_off_led(void)
{
    ///* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(LED_GPIO, 0);
}

bool led_state(void)
{
    return gpio_get_level(LED_GPIO);
}

void reverse_led(void)
{
    bool curr_led_state = false;
    curr_led_state = gpio_get_level(LED_GPIO);
    printf("Current state: %s\n", curr_led_state ? "On" : "Off");
    if (curr_led_state)
    {
        gpio_set_level(LED_GPIO, 0);
        ESP_LOGI(TAG_LED, "Turn off\n");
    }
    else if (!curr_led_state)
    {
        gpio_set_level(LED_GPIO, 1);
        ESP_LOGI(TAG_LED, "Turn on\n");
    }
}
#elif CONFIG_BLINK_LED
void init_led(void)
{
    ESP_LOGI(TAG_LED, "Configured to blink GPIO LED!\n");
    while (1)
    {
        /* Set the GPIO level according to the state (LOW or HIGH)*/
        gpio_set_level(LED_GPIO, s_led_state);
        s_led_state = !s_led_state;
        vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
    }
}
#endif


#define LED_INDICATOR_CHECK(a, str, ret) if(!(a)) { \
        ESP_LOGE(TAG_LED,"%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, str); \
        return (ret); \
    }

#define LED_INDICATOR_CHECK_GOTO(a, str, lable) if(!(a)) { \
        ESP_LOGE(TAG_LED,"%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, str); \
        goto lable; \
    }

#define NULL_ACTIVE_BLINK -1

/*********************************** Config LED Blink List in Different Conditions ***********************************/

/* 
 * @brief Button single click
 * // the total time should be equal to the delay in callback
 */
static const blink_step_t single_click[] = {
    {LED_BLINK_HOLD, LED_STATE_ON, 20},
    {LED_BLINK_HOLD, LED_STATE_OFF, 480},
    //{LED_BLINK_LOOP, 0, 0},
};

/* 
 * @brief Button double click
 */
static const blink_step_t double_click[] = {
    {LED_BLINK_HOLD, LED_STATE_ON, 50},
    {LED_BLINK_HOLD, LED_STATE_OFF, 50},
    {LED_BLINK_HOLD, LED_STATE_ON, 50},
    {LED_BLINK_HOLD, LED_STATE_OFF, 350},
    //{LED_BLINK_LOOP, 0, 0},
};

/* 
 * @brief Button long press
 */
static const blink_step_t long_press[] = {
    {LED_BLINK_HOLD, LED_STATE_ON, 1000},
    //{LED_BLINK_LOOP, 0, 0},
};

/* 
 * @brief Button long press
 */
static const blink_step_t no_press[] = {
    {LED_BLINK_HOLD, LED_STATE_OFF, 1000},
    {LED_BLINK_LOOP, 0, 0},
};


/**
 * @brief led blink lists, the index like BLINK_FACTORY_RESET defined the priority of the blink
 * 
 */
blink_step_t const * led_indicator_blink_lists[] = {
    [BLINK_SINGLE_CLICK] = single_click,
    [BLINK_DOUBLE_CLICK] = double_click,
    [BLINK_LONG_PRESS] = long_press,
    [BLINK_NO_PRESS] = no_press,
    [BLINK_MAX] = NULL,
};

/* Led blink_steps handling machine implementation */
#define BLINK_LIST_NUM (sizeof(led_indicator_blink_lists)/sizeof(led_indicator_blink_lists[0]))

/**
 * @brief led indicator object
 * 
 */
typedef struct {
    bool off_level; /*!< gpio level during led turn off */
    int io_num; /*!< gpio number of the led indicator */
    led_indicator_mode_t mode; /*!< led work mode, eg. gpio or pwm mode */
    int active_blink; /*!< active blink list*/
    int *p_blink_steps; /*!< stage of each blink list */
    SemaphoreHandle_t mutex; /*!< mutex to achive thread-safe */
    TimerHandle_t h_timer; /*!< led timmer handle, invalid if works in pwm mode */
}_led_indicator_t;

typedef struct _led_indicator_slist_t{
    SLIST_ENTRY(_led_indicator_slist_t) next;
    _led_indicator_t *p_led_indicator;
}_led_indicator_slist_t;

static SLIST_HEAD(_led_indicator_head_t, _led_indicator_slist_t) s_led_indicator_slist_head = SLIST_HEAD_INITIALIZER(s_led_indicator_slist_head);

static esp_err_t _led_indicator_add_node(_led_indicator_t *p_led_indicator)
{
    LED_INDICATOR_CHECK(p_led_indicator != NULL, "pointer can not be NULL", ESP_ERR_INVALID_ARG);
    _led_indicator_slist_t *node = calloc(1, sizeof(_led_indicator_slist_t));
    LED_INDICATOR_CHECK(node != NULL, "calloc node failed", ESP_ERR_NO_MEM);
    node->p_led_indicator = p_led_indicator;
    SLIST_INSERT_HEAD(&s_led_indicator_slist_head, node, next);
    return ESP_OK;
}

static esp_err_t _led_indicator_remove_node(_led_indicator_t *p_led_indicator)
{
    LED_INDICATOR_CHECK(p_led_indicator != NULL, "pointer can not be NULL", ESP_ERR_INVALID_ARG);
    _led_indicator_slist_t *node;
    SLIST_FOREACH(node, &s_led_indicator_slist_head, next) {
        if (node->p_led_indicator == p_led_indicator) {
            SLIST_REMOVE(&s_led_indicator_slist_head, node, _led_indicator_slist_t, next);
            free(node);
            break;
        }
    }
    return ESP_OK;
}

/**
 * @brief init a gpio to control led
 * 
 * @param io_num gpio number of the led
 * @return true init succeed
 * @return false init failed
 */
static bool _led_gpio_init(int io_num)
{
    gpio_config_t io_conf = {0};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set
    io_conf.pin_bit_mask = 1ULL << io_num;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    esp_err_t ret = gpio_config(&io_conf);

    if (ret != ESP_OK) {
        return false;
    }

    return true;
}

/**
 * @brief deinit a gpio to control led
 * 
 * @param io_num gpio number of the led
 * @return true deinit succeed
 * @return false deinit failed
 */
static bool _led_gpio_deinit(int io_num)
{
    esp_err_t ret = gpio_reset_pin(io_num);

    if (ret != ESP_OK) {
        return false;
    }

    return true;
}

/**
 * @brief turn on or off of the led
 * 
 * @param io_num gpio number of the led
 * @param off_level gpio level when off, 0 if attach led positive side to esp32 gpio pin, 1 if attach led negative side
 * @param state target state
 * @return esp_err_t 
 */
static esp_err_t _led_set_state(int io_num, bool off_level, blink_step_state_t state)
{
    uint32_t level = 0;

    switch (state)
    {
    case LED_STATE_ON:
        level = 1;
        if (off_level) level = 0;
        break;
    case LED_STATE_OFF:
    default :
        level = 0;
        if (off_level) level = 1;
        break;
    }

    return gpio_set_level(io_num, level);
}

/**
 * @brief switch to the first high priority incomplete blink steps
 * 
 * @param p_led_indicator pointer to led indicator
 */
static void _blink_list_switch(_led_indicator_t *p_led_indicator)
{
    p_led_indicator->active_blink = NULL_ACTIVE_BLINK; //stop active blink
    for(size_t index = 0; index < BLINK_LIST_NUM; index ++) //find the first incomplete blink
    {
        if (p_led_indicator->p_blink_steps[index] != LED_BLINK_STOP)
        {
            p_led_indicator->active_blink = index;
            break;
        }
    }
}

/**
 * @brief timmer callback to control led and counter steps
 * 
 * @param xTimer handle of the timmer instance
 */
static void _blink_list_runner(TimerHandle_t xTimer)
{
    _led_indicator_t * p_led_indicator = (_led_indicator_t *)pvTimerGetTimerID(xTimer);
    bool leave = false;

    while(!leave) {

        if (p_led_indicator->active_blink == NULL_ACTIVE_BLINK)
        return;

        int active_blink = p_led_indicator->active_blink;
        int active_step = p_led_indicator->p_blink_steps[active_blink];
        const blink_step_t *p_blink_step = &led_indicator_blink_lists[active_blink][active_step];

        p_led_indicator->p_blink_steps[active_blink] += 1;

        if (pdFALSE == xSemaphoreTake(p_led_indicator->mutex, pdMS_TO_TICKS(100))) {
            ESP_LOGW(TAG_LED, "blinks runner blockTime expired, try repairing...");
            xTimerChangePeriod(p_led_indicator->h_timer, pdMS_TO_TICKS(100), 0);
            xTimerStart(p_led_indicator->h_timer, 0);
            break;
        }

        switch(p_blink_step->type) {
            case LED_BLINK_LOOP:
                    p_led_indicator->p_blink_steps[active_blink] = 0;
                break;

            case LED_BLINK_STOP:
                    p_led_indicator->p_blink_steps[active_blink] = LED_BLINK_STOP;
                    _blink_list_switch(p_led_indicator);
                break;

            case LED_BLINK_HOLD:
                    _led_set_state(p_led_indicator->io_num, p_led_indicator->off_level, p_blink_step->on_off);
                    if (p_blink_step->hold_time_ms == 0)
                    break;
                    xTimerChangePeriod(p_led_indicator->h_timer, pdMS_TO_TICKS(p_blink_step->hold_time_ms), 0);
                    xTimerStart(p_led_indicator->h_timer, 0);
                    leave=true;
                break;

            default:
                    assert(false && "invalid state");
                break;
        }
        xSemaphoreGive(p_led_indicator->mutex);
    }
}

led_indicator_handle_t led_indicator_create(int io_num, const led_indicator_config_t* config)
{
    LED_INDICATOR_CHECK(config != NULL, "invalid config pointer", NULL);
    char timmer_name[16] = {'\0'};
    snprintf(timmer_name, sizeof(timmer_name) - 1, "%s%02x", "led_tmr_", io_num);
    _led_indicator_t *p_led_indicator = (_led_indicator_t *)calloc(1, sizeof(_led_indicator_t));
    LED_INDICATOR_CHECK(p_led_indicator != NULL, "calloc indicator memory failed", NULL);
    p_led_indicator->off_level = config->off_level;
    p_led_indicator->io_num = io_num;
    p_led_indicator->mode = config->mode;
    p_led_indicator->active_blink = NULL_ACTIVE_BLINK;
    p_led_indicator->p_blink_steps = (int *)calloc(BLINK_LIST_NUM, sizeof(int));
    LED_INDICATOR_CHECK_GOTO(p_led_indicator->p_blink_steps != NULL, "calloc blink_steps memory failed", cleanup_indicator);
    p_led_indicator->mutex = xSemaphoreCreateMutex();
    LED_INDICATOR_CHECK_GOTO(p_led_indicator->mutex != NULL, "create mutex failed", cleanup_indicator_blinkstep);
    
    for(size_t j = 0; j < BLINK_LIST_NUM; j++) {
        *(p_led_indicator->p_blink_steps + j) = LED_BLINK_STOP;
    }

    switch (p_led_indicator->mode)
    {
        case LED_GPIO_MODE:        /**< blink with max brightness*/
            {
            bool ininted = _led_gpio_init(p_led_indicator->io_num);
            LED_INDICATOR_CHECK_GOTO(ininted != false, "init led gpio failed", cleanup_all);
            p_led_indicator->h_timer = xTimerCreate(timmer_name, (pdMS_TO_TICKS(100)), pdFALSE, (void *)p_led_indicator, _blink_list_runner);
            LED_INDICATOR_CHECK_GOTO(p_led_indicator->h_timer != NULL, "led timmer create failed", cleanup_all);
            }
            break;

        default:
            LED_INDICATOR_CHECK_GOTO(false, "mode not supported", cleanup_all);
            break;
    }

    _led_indicator_add_node(p_led_indicator);
    return (led_indicator_handle_t)p_led_indicator;

cleanup_indicator:
    free(p_led_indicator);
    return NULL;
cleanup_indicator_blinkstep:
    free(p_led_indicator->p_blink_steps);
    free(p_led_indicator);
    return NULL;
cleanup_all:
    vSemaphoreDelete(p_led_indicator->mutex);
    free(p_led_indicator->p_blink_steps);
    free(p_led_indicator);
    return NULL;
}

led_indicator_handle_t led_indicator_get_handle(int io_num)
{
    _led_indicator_slist_t *node;
    SLIST_FOREACH(node, &s_led_indicator_slist_head, next) {
        if (node->p_led_indicator->io_num == io_num) {
            return (led_indicator_handle_t)(node->p_led_indicator);
        }
    }
    return NULL;
}

esp_err_t led_indicator_delete(led_indicator_handle_t* p_handle)
{
    LED_INDICATOR_CHECK(p_handle != NULL && *p_handle != NULL, "invalid p_handle", ESP_ERR_INVALID_ARG);
    _led_indicator_t *p_led_indicator = (_led_indicator_t *)(*p_handle);
    xSemaphoreTake(p_led_indicator->mutex, portMAX_DELAY);

    switch (p_led_indicator->mode)
    {
        case LED_GPIO_MODE:
            {
            bool deinited = _led_gpio_deinit(p_led_indicator->io_num);
            LED_INDICATOR_CHECK(deinited != false, "deinit led gpio failed", ESP_FAIL);
            BaseType_t ret = xTimerDelete(p_led_indicator->h_timer, portMAX_DELAY);
            LED_INDICATOR_CHECK(ret == pdPASS, "led timmer delete failed", ESP_FAIL);
            }
            break;

        default:
            LED_INDICATOR_CHECK(false, "mode not supported", ESP_ERR_NOT_SUPPORTED);
            break;
    }
    _led_indicator_remove_node(p_led_indicator);
    vSemaphoreDelete(p_led_indicator->mutex);
    free(p_led_indicator->p_blink_steps);
    free(*p_handle);
    *p_handle = NULL;
    return ESP_OK;
}

esp_err_t led_indicator_start(led_indicator_handle_t handle, led_indicator_blink_type_t blink_type)
{
    LED_INDICATOR_CHECK(handle != NULL && blink_type >= 0 && blink_type < BLINK_MAX, "invalid p_handle", ESP_ERR_INVALID_ARG);
    LED_INDICATOR_CHECK(led_indicator_blink_lists[blink_type] != NULL, "undefined blink_type", ESP_ERR_INVALID_ARG);
    _led_indicator_t *p_led_indicator = (_led_indicator_t *)handle;
    xSemaphoreTake(p_led_indicator->mutex, portMAX_DELAY);
    p_led_indicator->p_blink_steps[blink_type] = 0;
    _blink_list_switch(p_led_indicator);
    xSemaphoreGive(p_led_indicator->mutex);

    if(p_led_indicator->active_blink == blink_type) { //re-run from first step
        _blink_list_runner(p_led_indicator->h_timer);
    }

    return ESP_OK;
}

esp_err_t led_indicator_stop(led_indicator_handle_t handle, led_indicator_blink_type_t blink_type)
{
    LED_INDICATOR_CHECK(handle != NULL && blink_type >= 0 && blink_type < BLINK_MAX, "invalid p_handle", ESP_ERR_INVALID_ARG);
    LED_INDICATOR_CHECK(led_indicator_blink_lists[blink_type] != NULL, "undefined blink_type", ESP_ERR_INVALID_ARG);
    _led_indicator_t *p_led_indicator = (_led_indicator_t *)handle;
    xSemaphoreTake(p_led_indicator->mutex, portMAX_DELAY);
    p_led_indicator->p_blink_steps[blink_type] = LED_BLINK_STOP;
    _blink_list_switch(p_led_indicator); //stop and swith to next blink steps
    xSemaphoreGive(p_led_indicator->mutex);

    if(p_led_indicator->active_blink == blink_type) { //re-run from first step
        _blink_list_runner(p_led_indicator->h_timer);
    }

    return ESP_OK;
}