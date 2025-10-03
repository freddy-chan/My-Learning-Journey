# MPU6050 Accelerometer/Gyroscope Sensor Component
# 
# This component provides a complete MPU6050 sensor interface with:
# - I2C communication protocol
# - DMP (Digital Motion Processor) support
# - Calibration and offset compensation
# - Thread-safe operations with mutex protection
# - Professional error handling and statistics

idf_component_register(
    SRCS "mpu6050_sensor.c"
    INCLUDE_DIRS "include"
    REQUIRES "driver" "esp_timer" "freertos"
)