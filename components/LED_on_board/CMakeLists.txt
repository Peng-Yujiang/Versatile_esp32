set(srcs "src/LED_on_board.c")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "include")