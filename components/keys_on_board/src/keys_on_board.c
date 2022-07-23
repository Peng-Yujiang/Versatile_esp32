// defines the left and right keys on esp32 board
// Created by Yujiang Peng
// 
#include "keys_on_board.h"

/**
 * Brief:
 * This shows how to configure key gpios.
 *
 * GPIO status:
 * GPIO13: left input, pulled up, interrupt from rising edge and falling edge
 * GPIO15: right input, pulled up, interrupt from rising edge.
 */
static const char* TAG_KEYS = "Keys on board";

void init_keys(void)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = GPIO_KEY_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);
    ESP_LOGI(TAG_KEYS, "initiated\n");
}