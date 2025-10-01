/**
 * @file dht22_sensor.h
 * @brief DHT22 Temperature and Humidity Sensor Component
 * 
 * This component provides a comprehensive interface for the DHT22 (AM2302) sensor,
 * featuring professional-grade error handling, data validation, and performance monitoring.
 * 
 * Features:
 * - Accurate temperature and humidity readings
 * - Built-in data validation and checksum verification
 * - Automatic retry mechanism for failed readings
 * - Comprehensive error reporting and diagnostics
 * - Thread-safe operations with FreeRTOS integration
 * - Configurable timing parameters for different environmental conditions
 * - Statistical tracking and sensor health monitoring
 * 
 * The DHT22 sensor uses a single-wire digital protocol with precise timing requirements.
 * This implementation handles all low-level protocol details while providing a clean,
 * easy-to-use API for application developers.
 * 
 * @author ESP-IDF Learning Project
 * @date 2025
 * @version 1.0
 */

#ifndef DHT22_SENSOR_H
#define DHT22_SENSOR_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief DHT22 sensor error codes
 */
#define DHT22_ERR_BASE              0x6000
#define DHT22_ERR_TIMEOUT           (DHT22_ERR_BASE + 1)   ///< Communication timeout
#define DHT22_ERR_CHECKSUM          (DHT22_ERR_BASE + 2)   ///< Data checksum mismatch
#define DHT22_ERR_NO_RESPONSE       (DHT22_ERR_BASE + 3)   ///< Sensor not responding
#define DHT22_ERR_INVALID_DATA      (DHT22_ERR_BASE + 4)   ///< Invalid sensor data
#define DHT22_ERR_TOO_FREQUENT      (DHT22_ERR_BASE + 5)   ///< Reading too frequent (< 2s interval)
#define DHT22_ERR_NOT_INITIALIZED   (DHT22_ERR_BASE + 6)   ///< Component not initialized

/**
 * @brief DHT22 sensor measurement limits and specifications
 */
#define DHT22_TEMP_MIN_C            -40.0f      ///< Minimum temperature in Celsius
#define DHT22_TEMP_MAX_C            80.0f       ///< Maximum temperature in Celsius
#define DHT22_HUMIDITY_MIN          0.0f        ///< Minimum humidity percentage
#define DHT22_HUMIDITY_MAX          100.0f      ///< Maximum humidity percentage
#define DHT22_TEMP_RESOLUTION       0.1f        ///< Temperature resolution
#define DHT22_HUMIDITY_RESOLUTION   0.1f        ///< Humidity resolution
#define DHT22_MIN_INTERVAL_MS       2000        ///< Minimum interval between readings (ms)

/**
 * @brief DHT22 timing parameters (microseconds)
 * These are based on the DHT22 datasheet specifications
 */
#define DHT22_START_LOW_TIME        1000        ///< Host start signal low time
#define DHT22_START_HIGH_TIME       30          ///< Host start signal high time
#define DHT22_RESPONSE_TIMEOUT      100         ///< Maximum time to wait for sensor response
#define DHT22_BIT_TIMEOUT           100         ///< Maximum time to wait for each bit
#define DHT22_TOTAL_BITS            40          ///< Total data bits (humidity + temperature + checksum)

/**
 * @brief DHT22 sensor reading structure
 */
typedef struct {
    float temperature;          ///< Temperature in Celsius
    float humidity;             ///< Relative humidity percentage (0-100%)
    uint64_t timestamp_us;      ///< Timestamp when reading was taken (microseconds since boot)
    bool is_valid;              ///< Whether the reading is valid
    uint8_t checksum_received;  ///< Checksum received from sensor
    uint8_t checksum_calculated;///< Calculated checksum for validation
} dht22_reading_t;

/**
 * @brief DHT22 sensor statistics
 */
typedef struct {
    uint32_t total_readings;        ///< Total number of reading attempts
    uint32_t successful_readings;   ///< Number of successful readings
    uint32_t timeout_errors;        ///< Number of timeout errors
    uint32_t checksum_errors;       ///< Number of checksum errors
    uint32_t invalid_data_errors;   ///< Number of invalid data errors
    uint32_t too_frequent_errors;   ///< Number of too frequent reading errors
    uint64_t last_successful_read;  ///< Timestamp of last successful reading
    float last_temperature;         ///< Last valid temperature reading
    float last_humidity;            ///< Last valid humidity reading
    float min_temperature;          ///< Minimum temperature recorded
    float max_temperature;          ///< Maximum temperature recorded
    float min_humidity;             ///< Minimum humidity recorded
    float max_humidity;             ///< Maximum humidity recorded
} dht22_stats_t;

/**
 * @brief DHT22 sensor configuration
 */
typedef struct {
    gpio_num_t data_pin;            ///< GPIO pin connected to DHT22 data line
    uint32_t max_retries;           ///< Maximum number of retries for failed readings
    uint32_t retry_delay_ms;        ///< Delay between retries in milliseconds
    bool enable_statistics;         ///< Whether to track sensor statistics
    bool enable_debug_logging;      ///< Whether to enable debug logging
    uint32_t timeout_us;            ///< Custom timeout in microseconds (0 = use default)
} dht22_config_t;

/**
 * @brief DHT22 sensor handle (opaque structure)
 */
typedef struct dht22_sensor_context* dht22_handle_t;

/**
 * @brief Initialize DHT22 sensor
 */
esp_err_t dht22_init(const dht22_config_t *config, dht22_handle_t *handle);

/**
 * @brief Deinitialize DHT22 sensor
 */
esp_err_t dht22_deinit(dht22_handle_t handle);

/**
 * @brief Read temperature and humidity from DHT22 sensor
 */
esp_err_t dht22_read(dht22_handle_t handle, dht22_reading_t *reading);

/**
 * @brief Read only temperature from DHT22 sensor
 */
esp_err_t dht22_read_temperature(dht22_handle_t handle, float *temperature);

/**
 * @brief Read only humidity from DHT22 sensor
 */
esp_err_t dht22_read_humidity(dht22_handle_t handle, float *humidity);

/**
 * @brief Get DHT22 sensor statistics
 */
esp_err_t dht22_get_stats(dht22_handle_t handle, dht22_stats_t *stats);

/**
 * @brief Reset DHT22 sensor statistics
 */
esp_err_t dht22_reset_stats(dht22_handle_t handle);

/**
 * @brief Check if DHT22 sensor is ready for reading
 */
bool dht22_is_ready(dht22_handle_t handle);

/**
 * @brief Get time until next reading is allowed
 */
uint32_t dht22_time_until_ready(dht22_handle_t handle);

/**
 * @brief Convert DHT22 error code to string
 */
const char* dht22_error_to_string(esp_err_t error);

/**
 * @brief Validate DHT22 reading values
 */
bool dht22_validate_reading(float temperature, float humidity);

/**
 * @brief Print DHT22 sensor information
 */
void dht22_print_info(dht22_handle_t handle, bool detailed);

/**
 * @brief Convert temperature from Celsius to Fahrenheit
 */
static inline float dht22_celsius_to_fahrenheit(float celsius) {
    return (celsius * 9.0f / 5.0f) + 32.0f;
}

/**
 * @brief Convert temperature from Fahrenheit to Celsius
 */
static inline float dht22_fahrenheit_to_celsius(float fahrenheit) {
    return (fahrenheit - 32.0f) * 5.0f / 9.0f;
}

/**
 * @brief Calculate heat index (apparent temperature)
 */
float dht22_calculate_heat_index(float temperature_c, float humidity);

/**
 * @brief Calculate dew point
 */
float dht22_calculate_dew_point(float temperature_c, float humidity);

#ifdef __cplusplus
}
#endif

#endif // DHT22_SENSOR_H