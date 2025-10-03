# OTA Update Component CMakeLists.txt
# This file defines how the OTA update component should be built

idf_component_register(
    SRCS "ota_update.c"                 # Source files to compile
    INCLUDE_DIRS "include"             # Public header directories
    REQUIRES "app_update"
             "esp_http_client"
             "esp_https_ota"
             "esp_wifi"
             "freertos"
             "esp_timer"
)