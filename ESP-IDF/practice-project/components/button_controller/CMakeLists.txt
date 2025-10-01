# Button Controller Component CMakeLists.txt
# This file defines how the button controller component should be built

idf_component_register(
    SRCS "button_controller.c"        # Source files to compile
    INCLUDE_DIRS "include"            # Public header directories  
    REQUIRES "driver" "freertos" "esp_timer"      # Dependencies (GPIO driver + FreeRTOS + ESP Timer)
)