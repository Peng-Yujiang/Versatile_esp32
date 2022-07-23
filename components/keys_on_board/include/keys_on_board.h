// defines the left and right keys on esp32 board
// Created by Yujiang Peng
// 
#ifndef KEYS_ON_BOARD_H
#define KEYS_ON_BOARD_H

#include "esp_log.h"
#include "driver/gpio.h"

#define LEFT_KEY_GPIO CONFIG_LEFT_KEY_GPIO // Left key, GPIO13
#define RIGHT_KEY_GPIO CONFIG_RIGHT_KEY_GPIO // Right key, GPIO15
#define GPIO_KEY_PIN_SEL  ((1ULL<<LEFT_KEY_GPIO) | (1ULL<<RIGHT_KEY_GPIO))

void init_keys(void);

#endif