#ifndef OLED_INIT_H
#define OLED_INIT_H
#include "screen_driver.h"
static const unsigned char bmp_image_128_64[1024];
static const unsigned char bmp_image_52_24[156];


scr_driver_t oled_init(void);


#endif