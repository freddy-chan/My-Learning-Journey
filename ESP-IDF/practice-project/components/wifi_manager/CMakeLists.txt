# WiFi Manager Component CMakeLists.txt
# This file defines how the WiFi manager component should be built

idf_component_register(
    SRCS "wifi_manager.c"              # Source files to compile
    INCLUDE_DIRS "include"             # Public header directories
    REQUIRES "system_config"           # Dependencies on other components
             "esp_wifi"
             "esp_event"
             "esp_netif"
             "driver"
             "freertos"
)