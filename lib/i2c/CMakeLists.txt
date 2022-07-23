set(srcs "src/keys_on_board.c")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "include")
