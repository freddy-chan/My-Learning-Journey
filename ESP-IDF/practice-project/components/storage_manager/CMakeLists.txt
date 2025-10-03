# Storage Manager Component
# 
# This component provides storage functionality for the Smart Environment Data Logger
# including SD card interface and data logging capabilities.

idf_component_register(
    SRCS "sd_card_storage.c"
    INCLUDE_DIRS "include"
    REQUIRES "driver" "fatfs" "spi_flash" "esp_timer"
)