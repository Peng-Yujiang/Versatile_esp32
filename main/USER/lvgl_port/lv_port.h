#ifndef LV_PORT_H
#define LV_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

void lvgl_port_init(void);

#ifdef CONFIG_ENCODER
//void lvgl_port_indev_init(void);
#endif

#ifdef CONFIG_FILE_SYSTEM
//void lvgl_port_fs_init(void);
#endif

#endif

#ifdef __cplusplus
} /*extern "C"*/
#endif