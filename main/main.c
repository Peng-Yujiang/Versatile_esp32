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
// #include "test_utils.h"
// #include "screen_driver.h"
// #include "ssd1306.h"


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
    /******************** Configure the gpio pins of keys on board ********************/
    //init_keys();  // ordinary gpio keys

    /******************** Configure the LED on board ********************/
    led_indicator_init();

    /******************** Configure on board iot buttons ********************/
    iot_buttons_init();

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