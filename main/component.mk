#
# Main component makefile.
#
# This Makefile can be left empty. By default, it will take the sources in the
# src/ directory, compile them and link them into lib(subdirectory_name).a
# in the build directory. This behaviour is entirely configurable,
# please read the ESP-IDF documents if you need to do this.
#
# ESP-IDF component file for make based commands

USER_SRCDIRS := . \
                  USER \
                  USER/lvgl_port \
                  USER/on_board \


COMPONENT_ADD_INCLUDEDIRS := $(USER_SRCDIRS) .
COMPONENT_SRCDIRS := $(USER_SRCDIRS) .