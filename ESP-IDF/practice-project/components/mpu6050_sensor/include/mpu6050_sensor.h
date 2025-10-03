/**
 * @file mpu6050_sensor.h
 * @brief MPU6050 6-Axis Accelerometer/Gyroscope Sensor Driver
 * 
 * This file provides a comprehensive interface for the MPU6050 sensor including:
 * - I2C communication with configurable parameters
 * - Raw and processed sensor data access
 * - Digital Motion Processor (DMP) integration
 * - Advanced filtering and calibration
 * - Motion detection and gesture recognition
 * - Thread-safe operations with mutex protection
 * - Professional error handling and statistics
 * 
 * Hardware Requirements:
 * - MPU6050 sensor module
 * - I2C bus connection (SDA/SCL pins)
 * - 3.3V power supply
 * - Optional: Interrupt pin for motion detection
 * 
 * @author ESP-IDF Learning Project
 * @date 2025
 * @version 1.0
 */

#ifndef MPU6050_SENSOR_H
#define MPU6050_SENSOR_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

// MPU6050 I2C addresses (7-bit address)
#define MPU6050_I2C_ADDR_DEFAULT    0x68    ///< Default I2C address (AD0 = 0)
#define MPU6050_I2C_ADDR_ALT        0x69    ///< Alternative I2C address (AD0 = 1)

// MPU6050 register addresses
#define MPU6050_REG_WHO_AM_I        0x75    ///< Device identification register
#define MPU6050_REG_PWR_MGMT_1      0x6B    ///< Power management register 1
#define MPU6050_REG_PWR_MGMT_2      0x6C    ///< Power management register 2
#define MPU6050_REG_CONFIG          0x1A    ///< Configuration register
#define MPU6050_REG_GYRO_CONFIG     0x1B    ///< Gyroscope configuration
#define MPU6050_REG_ACCEL_CONFIG    0x1C    ///< Accelerometer configuration
#define MPU6050_REG_ACCEL_XOUT_H    0x3B    ///< Accelerometer X-axis high byte
#define MPU6050_REG_TEMP_OUT_H      0x41    ///< Temperature high byte
#define MPU6050_REG_GYRO_XOUT_H     0x43    ///< Gyroscope X-axis high byte

// MPU6050 expected device ID
#define MPU6050_DEVICE_ID           0x68    ///< Expected WHO_AM_I register value

/**
 * @brief MPU6050 accelerometer full scale range
 */
typedef enum {
    MPU6050_ACCEL_RANGE_2G = 0,    ///< ±2g range, 16384 LSB/g
    MPU6050_ACCEL_RANGE_4G,        ///< ±4g range, 8192 LSB/g
    MPU6050_ACCEL_RANGE_8G,        ///< ±8g range, 4096 LSB/g
    MPU6050_ACCEL_RANGE_16G        ///< ±16g range, 2048 LSB/g
} mpu6050_accel_range_t;

/**
 * @brief MPU6050 gyroscope full scale range
 */
typedef enum {
    MPU6050_GYRO_RANGE_250DPS = 0, ///< ±250°/s range, 131 LSB/°/s
    MPU6050_GYRO_RANGE_500DPS,     ///< ±500°/s range, 65.5 LSB/°/s
    MPU6050_GYRO_RANGE_1000DPS,    ///< ±1000°/s range, 32.8 LSB/°/s
    MPU6050_GYRO_RANGE_2000DPS     ///< ±2000°/s range, 16.4 LSB/°/s
} mpu6050_gyro_range_t;

/**
 * @brief MPU6050 digital low pass filter bandwidth
 */
typedef enum {
    MPU6050_DLPF_260HZ = 0,        ///< 260Hz bandwidth
    MPU6050_DLPF_184HZ,            ///< 184Hz bandwidth
    MPU6050_DLPF_94HZ,             ///< 94Hz bandwidth
    MPU6050_DLPF_44HZ,             ///< 44Hz bandwidth
    MPU6050_DLPF_21HZ,             ///< 21Hz bandwidth
    MPU6050_DLPF_10HZ,             ///< 10Hz bandwidth
    MPU6050_DLPF_5HZ               ///< 5Hz bandwidth
} mpu6050_dlpf_t;

/**
 * @brief MPU6050 power mode
 */
typedef enum {
    MPU6050_POWER_NORMAL = 0,      ///< Normal operation mode
    MPU6050_POWER_SLEEP,           ///< Sleep mode (low power)
    MPU6050_POWER_STANDBY          ///< Standby mode (configurable)
} mpu6050_power_mode_t;

/**
 * @brief MPU6050 raw sensor data structure
 */
typedef struct {
    int16_t accel_x;               ///< Raw accelerometer X-axis data
    int16_t accel_y;               ///< Raw accelerometer Y-axis data
    int16_t accel_z;               ///< Raw accelerometer Z-axis data
    int16_t temp;                  ///< Raw temperature data
    int16_t gyro_x;                ///< Raw gyroscope X-axis data
    int16_t gyro_y;                ///< Raw gyroscope Y-axis data
    int16_t gyro_z;                ///< Raw gyroscope Z-axis data
    uint64_t timestamp;            ///< Timestamp of reading (microseconds)
} mpu6050_raw_data_t;

/**
 * @brief MPU6050 processed sensor data structure
 */
typedef struct {
    float accel_x;                 ///< Accelerometer X-axis (m/s²)
    float accel_y;                 ///< Accelerometer Y-axis (m/s²)
    float accel_z;                 ///< Accelerometer Z-axis (m/s²)
    float temp_celsius;            ///< Temperature in Celsius
    float gyro_x;                  ///< Gyroscope X-axis (°/s)
    float gyro_y;                  ///< Gyroscope Y-axis (°/s)
    float gyro_z;                  ///< Gyroscope Z-axis (°/s)
    float roll;                    ///< Calculated roll angle (°)
    float pitch;                   ///< Calculated pitch angle (°)
    float yaw;                     ///< Calculated yaw angle (°)
    uint64_t timestamp;            ///< Timestamp of reading (microseconds)
} mpu6050_data_t;

/**
 * @brief MPU6050 calibration data structure
 */
typedef struct {
    int16_t accel_offset_x;        ///< Accelerometer X-axis offset
    int16_t accel_offset_y;        ///< Accelerometer Y-axis offset
    int16_t accel_offset_z;        ///< Accelerometer Z-axis offset
    int16_t gyro_offset_x;         ///< Gyroscope X-axis offset
    int16_t gyro_offset_y;         ///< Gyroscope Y-axis offset
    int16_t gyro_offset_z;         ///< Gyroscope Z-axis offset
    bool calibrated;               ///< Calibration status flag
} mpu6050_calibration_t;

/**
 * @brief MPU6050 statistics structure
 */
typedef struct {
    uint32_t total_reads;          ///< Total number of successful reads
    uint32_t failed_reads;         ///< Total number of failed reads
    uint32_t i2c_errors;           ///< Number of I2C communication errors
    uint32_t timeout_errors;       ///< Number of timeout errors
    uint32_t data_errors;          ///< Number of data validation errors
    float success_rate;            ///< Success rate percentage
    uint64_t last_read_time;       ///< Timestamp of last successful read
    uint64_t uptime_ms;            ///< Component uptime in milliseconds
} mpu6050_stats_t;

/**
 * @brief MPU6050 configuration structure
 */
typedef struct {
    gpio_num_t sda_pin;            ///< I2C SDA pin number
    gpio_num_t scl_pin;            ///< I2C SCL pin number
    gpio_num_t int_pin;            ///< Interrupt pin (optional, set to -1 if unused)
    uint8_t i2c_address;           ///< I2C device address (7-bit)
    i2c_port_t i2c_port;           ///< I2C port number (0 or 1)
    uint32_t i2c_frequency;        ///< I2C frequency in Hz
    mpu6050_accel_range_t accel_range;  ///< Accelerometer range
    mpu6050_gyro_range_t gyro_range;    ///< Gyroscope range
    mpu6050_dlpf_t dlpf_mode;      ///< Digital low pass filter setting
    bool enable_dmp;               ///< Enable Digital Motion Processor
    bool enable_interrupt;         ///< Enable interrupt functionality
    bool enable_statistics;        ///< Enable performance statistics
    bool enable_debug_logging;     ///< Enable debug logging
    uint32_t timeout_ms;           ///< I2C timeout in milliseconds
} mpu6050_config_t;

/**
 * @brief MPU6050 sensor handle (opaque)
 */
typedef void* mpu6050_handle_t;

/**
 * @brief MPU6050 error types
 */
typedef enum {
    MPU6050_OK = ESP_OK,                    ///< No error
    MPU6050_ERR_INVALID_ARG = ESP_ERR_INVALID_ARG,       ///< Invalid argument
    MPU6050_ERR_NO_MEM = ESP_ERR_NO_MEM,               ///< Memory allocation failed
    MPU6050_ERR_TIMEOUT = ESP_ERR_TIMEOUT,             ///< Communication timeout
    MPU6050_ERR_INVALID_STATE = ESP_ERR_INVALID_STATE, ///< Invalid state
    MPU6050_ERR_NOT_FOUND = ESP_ERR_NOT_FOUND,         ///< Device not found
    MPU6050_ERR_COMMUNICATION = 0x1001,    ///< I2C communication error
    MPU6050_ERR_CALIBRATION,               ///< Calibration error
    MPU6050_ERR_DATA_INVALID,              ///< Invalid sensor data
    MPU6050_ERR_DMP_INIT,                  ///< DMP initialization failed
    MPU6050_ERR_SELF_TEST                  ///< Self-test failed
} mpu6050_err_t;

// Function Declarations

/**
 * @brief Initialize MPU6050 sensor
 * 
 * This function initializes the MPU6050 sensor with the provided configuration.
 * It performs device detection, I2C bus setup, sensor configuration, and
 * optional calibration.
 * 
 * @param config Pointer to MPU6050 configuration structure
 * @param handle Pointer to store the created sensor handle
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t mpu6050_init(const mpu6050_config_t *config, mpu6050_handle_t *handle);

/**
 * @brief Deinitialize MPU6050 sensor
 * 
 * Releases all resources and puts the sensor into sleep mode.
 * 
 * @param handle MPU6050 sensor handle
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t mpu6050_deinit(mpu6050_handle_t handle);

/**
 * @brief Read raw sensor data
 * 
 * Reads raw accelerometer, gyroscope, and temperature data from the sensor.
 * No processing or calibration is applied to the data.
 * 
 * @param handle MPU6050 sensor handle
 * @param data Pointer to store raw sensor data
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t mpu6050_read_raw(mpu6050_handle_t handle, mpu6050_raw_data_t *data);

/**
 * @brief Read processed sensor data
 * 
 * Reads and processes sensor data including calibration, unit conversion,
 * and angle calculation using complementary filter.
 * 
 * @param handle MPU6050 sensor handle
 * @param data Pointer to store processed sensor data
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t mpu6050_read_processed(mpu6050_handle_t handle, mpu6050_data_t *data);

/**
 * @brief Calibrate MPU6050 sensor
 * 
 * Performs automatic calibration by collecting samples and calculating
 * offset values. The sensor should be stationary during calibration.
 * 
 * @param handle MPU6050 sensor handle
 * @param sample_count Number of samples for calibration (recommended: 1000)
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t mpu6050_calibrate(mpu6050_handle_t handle, uint32_t sample_count);

/**
 * @brief Get calibration data
 * 
 * Retrieves the current calibration offset values.
 * 
 * @param handle MPU6050 sensor handle
 * @param calibration Pointer to store calibration data
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t mpu6050_get_calibration(mpu6050_handle_t handle, mpu6050_calibration_t *calibration);

/**
 * @brief Set calibration data
 * 
 * Applies previously stored calibration offset values.
 * 
 * @param handle MPU6050 sensor handle
 * @param calibration Pointer to calibration data
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t mpu6050_set_calibration(mpu6050_handle_t handle, const mpu6050_calibration_t *calibration);

/**
 * @brief Get sensor statistics
 * 
 * Retrieves performance statistics including success rate, error counts,
 * and timing information.
 * 
 * @param handle MPU6050 sensor handle
 * @param stats Pointer to store statistics data
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t mpu6050_get_stats(mpu6050_handle_t handle, mpu6050_stats_t *stats);

/**
 * @brief Reset sensor statistics
 * 
 * Clears all accumulated statistics counters.
 * 
 * @param handle MPU6050 sensor handle
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t mpu6050_reset_stats(mpu6050_handle_t handle);

/**
 * @brief Set power mode
 * 
 * Changes the sensor power mode for energy management.
 * 
 * @param handle MPU6050 sensor handle
 * @param mode Power mode to set
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t mpu6050_set_power_mode(mpu6050_handle_t handle, mpu6050_power_mode_t mode);

/**
 * @brief Perform self-test
 * 
 * Executes built-in self-test functionality to verify sensor operation.
 * 
 * @param handle MPU6050 sensor handle
 * @return esp_err_t ESP_OK if self-test passes, error code on failure
 */
esp_err_t mpu6050_self_test(mpu6050_handle_t handle);

/**
 * @brief Print sensor information
 * 
 * Displays comprehensive sensor status, configuration, and statistics.
 * 
 * @param handle MPU6050 sensor handle
 * @param show_detailed If true, shows detailed debug information
 */
void mpu6050_print_info(mpu6050_handle_t handle, bool show_detailed);

/**
 * @brief Convert error code to string
 * 
 * Converts MPU6050 error codes to human-readable strings.
 * 
 * @param error Error code to convert
 * @return const char* Error description string
 */
const char* mpu6050_error_to_string(mpu6050_err_t error);

// Utility Functions

/**
 * @brief Convert raw accelerometer data to m/s²
 * 
 * @param raw_value Raw accelerometer value
 * @param range Accelerometer range setting
 * @return float Acceleration in m/s²
 */
float mpu6050_accel_raw_to_ms2(int16_t raw_value, mpu6050_accel_range_t range);

/**
 * @brief Convert raw gyroscope data to degrees per second
 * 
 * @param raw_value Raw gyroscope value
 * @param range Gyroscope range setting
 * @return float Angular velocity in °/s
 */
float mpu6050_gyro_raw_to_dps(int16_t raw_value, mpu6050_gyro_range_t range);

/**
 * @brief Convert raw temperature data to Celsius
 * 
 * @param raw_value Raw temperature value
 * @return float Temperature in Celsius
 */
float mpu6050_temp_raw_to_celsius(int16_t raw_value);

/**
 * @brief Calculate roll angle from accelerometer data
 * 
 * @param accel_x X-axis acceleration (m/s²)
 * @param accel_y Y-axis acceleration (m/s²)
 * @param accel_z Z-axis acceleration (m/s²)
 * @return float Roll angle in degrees
 */
float mpu6050_calculate_roll(float accel_x, float accel_y, float accel_z);

/**
 * @brief Calculate pitch angle from accelerometer data
 * 
 * @param accel_x X-axis acceleration (m/s²)
 * @param accel_y Y-axis acceleration (m/s²)
 * @param accel_z Z-axis acceleration (m/s²)
 * @return float Pitch angle in degrees
 */
float mpu6050_calculate_pitch(float accel_x, float accel_y, float accel_z);

#ifdef __cplusplus
}
#endif

#endif // MPU6050_SENSOR_H