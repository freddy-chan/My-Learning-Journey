# System Configuration Component CMakeLists.txt
# This component provides centralized configuration management with menuconfig integration

idf_component_register(
    SRCS "system_config.c"            # Source files to compile
    INCLUDE_DIRS "include"            # Public header directories
    REQUIRES "driver" "esp_timer"     # Dependencies (GPIO driver + ESP Timer)
)