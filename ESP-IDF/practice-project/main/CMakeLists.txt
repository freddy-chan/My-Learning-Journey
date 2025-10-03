idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES led_controller button_controller system_config dht22_sensor mpu6050_sensor storage_manager wifi_manager mqtt_client ota_update)