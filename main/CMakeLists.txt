set(srcs 	"main.c"
		"USER/lvgl_port/lv_port.c"
		"USER/lvgl_port/lv_port_disp_init.c"
		"USER/lvgl_port/oled_init.c"
		)


idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS ".")
