# MQTT Client Component CMakeLists.txt
# This file defines how the MQTT client component should be built

idf_component_register(
    SRCS "mqtt_client.c"               # Source files to compile
    INCLUDE_DIRS "include"             # Public header directories
    REQUIRES "mqtt"
             "esp_wifi"
             "esp_event"
             "freertos"
             "esp_system"
)