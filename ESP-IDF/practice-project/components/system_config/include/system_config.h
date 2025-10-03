/**
 * @file system_config.h
 * @brief System Configuration Manager Component Header
 * 
 * This component provides centralized configuration management for the Smart Environment
 * Data Logger system. It integrates with ESP-IDF's menuconfig system to provide
 * runtime configuration capabilities and validates all system parameters.
 * 
 * Features:
 * - Centralized configuration management for all system components
 * - Integration with ESP-IDF menuconfig system for user-configurable parameters
 * - Runtime configuration validation and error reporting
 * - Default value management with fallback mechanisms
 * - Configuration persistence and recovery capabilities
 * - Type-safe configuration access with compile-time checks
 * 
 * @author Learning ESP-IDF
 * @date 2025
 * @version 1.0
 */

#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief System configuration validation result enumeration
 * 
 * Defines validation states for configuration parameters
 */
typedef enum {
    CONFIG_VALID = 0,           /**< Configuration parameter is valid */
    CONFIG_INVALID_RANGE,       /**< Parameter is outside valid range */
    CONFIG_INVALID_GPIO,        /**< GPIO pin is invalid or unavailable */
    CONFIG_INVALID_COMBINATION, /**< Parameter combination is invalid */
    CONFIG_NOT_INITIALIZED      /**< Configuration system not initialized */
} config_validation_result_t;

/**
 * @brief GPIO configuration structure
 * 
 * Contains all GPIO pin assignments for the system with validation
 */
typedef struct {
    gpio_num_t led_pin;              /**< LED GPIO pin number */
    bool led_active_level;           /**< LED active level (true=high, false=low) */
    gpio_num_t button_pin;           /**< Button GPIO pin number */
    bool button_active_level;        /**< Button active level (true=high, false=low) */
    gpio_num_t dht22_pin;           /**< DHT22 data GPIO pin number */
    gpio_num_t mpu6050_sda_pin;     /**< MPU6050 I2C SDA pin number */
    gpio_num_t mpu6050_scl_pin;     /**< MPU6050 I2C SCL pin number */
    gpio_num_t sd_cs_pin;           /**< SD card SPI CS pin number */
    gpio_num_t sd_mosi_pin;         /**< SD card SPI MOSI pin number */
    gpio_num_t sd_miso_pin;         /**< SD card SPI MISO pin number */
    gpio_num_t sd_clk_pin;          /**< SD card SPI CLK pin number */
} system_gpio_config_t;

/**
 * @brief Timing configuration structure
 * 
 * Contains all timing parameters for system operation
 */
typedef struct {
    uint32_t button_debounce_ms;        /**< Button debounce time in milliseconds */
    uint32_t button_long_press_ms;      /**< Long press detection time in milliseconds */
    uint32_t button_double_click_ms;    /**< Double click timeout in milliseconds */
    uint32_t sensor_read_interval_ms;   /**< Sensor reading interval in milliseconds */
    uint32_t data_log_interval_ms;      /**< Data logging interval in milliseconds */
    uint32_t mqtt_publish_interval_ms;  /**< MQTT publish interval in milliseconds */
} system_timing_config_t;

/**
 * @brief Network configuration structure
 * 
 * Contains network-related configuration parameters
 */
typedef struct {
    char wifi_ssid[64];              /**< WiFi SSID (null-terminated string) */
    char wifi_password[64];          /**< WiFi password (null-terminated string) */
    char mqtt_broker_uri[128];       /**< MQTT broker URI (null-terminated string) */
    char mqtt_topic_prefix[32];      /**< MQTT topic prefix (null-terminated string) */
    uint16_t mqtt_port;              /**< MQTT broker port number */
    bool mqtt_use_tls;               /**< Enable TLS for MQTT connection */
} system_network_config_t;

/**
 * @brief Sensor configuration structure
 * 
 * Contains sensor-specific configuration parameters
 */
typedef struct {
    // MPU6050 configuration
    uint8_t mpu6050_i2c_address;    /**< MPU6050 I2C address (0x68 or 0x69) */
    uint32_t mpu6050_i2c_frequency; /**< I2C bus frequency for MPU6050 */
    uint8_t mpu6050_accel_range;    /**< Accelerometer range (0-3) */
    uint8_t mpu6050_gyro_range;     /**< Gyroscope range (0-3) */
    uint8_t mpu6050_dlpf_mode;      /**< Digital low pass filter mode (0-6) */
    bool mpu6050_enable_calibration; /**< Enable auto-calibration */
    uint32_t mpu6050_calibration_samples; /**< Calibration sample count */
    bool mpu6050_enable_statistics; /**< Enable performance statistics */
    bool mpu6050_enable_debug;      /**< Enable debug logging */
} system_sensor_config_t;

/**
 * @brief Complete system configuration structure
 * 
 * Aggregates all configuration subsystems into a single structure
 */
typedef struct {
    system_gpio_config_t gpio;       /**< GPIO pin assignments */
    system_timing_config_t timing;   /**< Timing parameters */
    system_network_config_t network; /**< Network configuration */
    system_sensor_config_t sensors;  /**< Sensor configuration */
    bool initialized;                /**< Configuration initialization flag */
    uint32_t config_version;         /**< Configuration version for compatibility */
} system_config_t;

/**
 * @brief Configuration change callback function type
 * 
 * User-defined callback function that gets called when configuration changes.
 * This allows components to react to configuration updates at runtime.
 * 
 * @param config Pointer to the updated configuration structure
 * @param user_data User-provided data pointer (set during callback registration)
 */
typedef void (*config_change_callback_t)(const system_config_t *config, void *user_data);

/**
 * @brief Initialize system configuration manager
 * 
 * This function initializes the configuration system, loads values from menuconfig,
 * validates all parameters, and prepares the system for operation. It must be
 * called before any other configuration functions.
 * 
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
esp_err_t system_config_init(void);

/**
 * @brief Get complete system configuration
 * 
 * Returns a pointer to the current system configuration. The returned pointer
 * is valid until the next configuration update.
 * 
 * @return Pointer to system configuration, or NULL if not initialized
 */
const system_config_t* system_config_get(void);

/**
 * @brief Get GPIO configuration subset
 * 
 * Returns only the GPIO-related configuration parameters. This is a convenience
 * function for components that only need GPIO settings.
 * 
 * @return Pointer to GPIO configuration, or NULL if not initialized
 */
const system_gpio_config_t* system_config_get_gpio(void);

/**
 * @brief Get timing configuration subset
 * 
 * Returns only the timing-related configuration parameters.
 * 
 * @return Pointer to timing configuration, or NULL if not initialized
 */
const system_timing_config_t* system_config_get_timing(void);

/**
 * @brief Get network configuration subset
 * 
 * Returns only the network-related configuration parameters.
 * 
 * @return Pointer to network configuration, or NULL if not initialized
 */
const system_network_config_t* system_config_get_network(void);

/**
 * @brief Get sensor configuration subset
 * 
 * Returns only the sensor-related configuration parameters.
 * 
 * @return Pointer to sensor configuration, or NULL if not initialized
 */
const system_sensor_config_t* system_config_get_sensors(void);

/**
 * @brief Validate GPIO pin assignment
 * 
 * Validates that a GPIO pin is suitable for its intended purpose and doesn't
 * conflict with other pin assignments.
 * 
 * @param pin GPIO pin number to validate
 * @param is_input true if pin will be used as input, false for output
 * @param allow_internal true to allow internal/restricted pins
 * @return CONFIG_VALID if pin is valid, error code otherwise
 */
config_validation_result_t system_config_validate_gpio(gpio_num_t pin, bool is_input, bool allow_internal);

/**
 * @brief Validate timing parameter
 * 
 * Validates that a timing parameter is within acceptable range and doesn't
 * conflict with system requirements.
 * 
 * @param value Timing value to validate (in milliseconds)
 * @param min_value Minimum acceptable value
 * @param max_value Maximum acceptable value
 * @return CONFIG_VALID if value is valid, CONFIG_INVALID_RANGE otherwise
 */
config_validation_result_t system_config_validate_timing(uint32_t value, uint32_t min_value, uint32_t max_value);

/**
 * @brief Validate complete configuration
 * 
 * Performs comprehensive validation of the entire system configuration,
 * checking for parameter ranges, GPIO conflicts, and logical consistency.
 * 
 * @return CONFIG_VALID if all parameters are valid, error code for first invalid parameter
 */
config_validation_result_t system_config_validate_all(void);

/**
 * @brief Register configuration change callback
 * 
 * Registers a callback function that will be called whenever the system
 * configuration is updated. Multiple callbacks can be registered.
 * 
 * @param callback Callback function to register
 * @param user_data User data to pass to callback function
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
esp_err_t system_config_register_callback(config_change_callback_t callback, void *user_data);

/**
 * @brief Update configuration from menuconfig
 * 
 * Reloads configuration values from the ESP-IDF menuconfig system and
 * validates the new configuration. This can be called at runtime if
 * dynamic reconfiguration is needed.
 * 
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
esp_err_t system_config_reload(void);

/**
 * @brief Reset configuration to defaults
 * 
 * Resets all configuration parameters to their default values and
 * reinitializes the configuration system.
 * 
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
esp_err_t system_config_reset_to_defaults(void);

/**
 * @brief Print current configuration
 * 
 * Prints the current system configuration to the console in a human-readable
 * format. Useful for debugging and system status reporting.
 * 
 * @param show_sensitive true to include sensitive data (passwords), false to mask them
 */
void system_config_print(bool show_sensitive);

/**
 * @brief Get configuration validation error string
 * 
 * Converts a configuration validation result to a human-readable error message.
 * 
 * @param result Validation result to convert
 * @return String description of the validation result
 */
const char* system_config_validation_to_string(config_validation_result_t result);

/**
 * @brief Deinitialize configuration system
 * 
 * Cleans up all resources used by the configuration system and unregisters
 * all callbacks. Should be called during system shutdown.
 * 
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
esp_err_t system_config_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // SYSTEM_CONFIG_H