# LED Controller Component CMakeLists.txt
# This file defines how the LED controller component should be built

idf_component_register(
    SRCS "led_controller.c"          # Source files to compile
    INCLUDE_DIRS "include"            # Public header directories
    REQUIRES "driver"                 # Dependencies (GPIO driver)
)