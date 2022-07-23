set(srcs "src/i2c.c")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "include")
