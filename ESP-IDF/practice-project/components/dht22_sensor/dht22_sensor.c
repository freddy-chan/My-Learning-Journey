/**
 * @file dht22_sensor.c
 * @brief DHT22 Temperature and Humidity Sensor Implementation
 * 
 * This file implements the complete DHT22 sensor interface with professional-grade
 * error handling, data validation, and performance monitoring.
 * 
 * Key Features:
 * - Precise timing control using esp_timer
 * - Comprehensive error detection and recovery
 * - Thread-safe operations with mutex protection
 * - Statistical tracking and performance monitoring
 * - Configurable retry mechanisms
 * - Data validation and checksum verification
 * 
 * @author ESP-IDF Learning Project
 * @date 2025
 * @version 1.0
 */

#include "dht22_sensor.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <math.h>

static const char *TAG = "DHT22";

/**
 * @brief DHT22 sensor context structure
 * 
 * This structure contains all the state information for a DHT22 sensor instance.
 * It is kept opaque to the user to maintain encapsulation.
 */
struct dht22_sensor_context {
    dht22_config_t config;          ///< Sensor configuration
    dht22_stats_t stats;            ///< Sensor statistics
    SemaphoreHandle_t mutex;        ///< Mutex for thread safety
    uint64_t last_read_time;        ///< Timestamp of last successful reading
    bool initialized;               ///< Initialization status
};

/**
 * @brief Validate DHT22 configuration
 * 
 * @param config Configuration to validate
 * @return ESP_OK if valid, ESP_ERR_INVALID_ARG if invalid
 */
static esp_err_t validate_config(const dht22_config_t *config) {
    if (config == NULL) {
        ESP_LOGE(TAG, "Configuration pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!GPIO_IS_VALID_GPIO(config->data_pin)) {
        ESP_LOGE(TAG, "Invalid GPIO pin: %d", config->data_pin);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (config->max_retries > 10) {
        ESP_LOGW(TAG, "High retry count: %lu (recommended: <= 3)", config->max_retries);
    }
    
    if (config->retry_delay_ms > 5000) {
        ESP_LOGW(TAG, "Long retry delay: %lu ms (recommended: <= 1000)", config->retry_delay_ms);
    }
    
    return ESP_OK;
}

/**
 * @brief Wait for GPIO pin to reach specified level with timeout
 * 
 * @param pin GPIO pin to monitor
 * @param level Expected level (0 or 1)
 * @param timeout_us Timeout in microseconds
 * @return true if level reached within timeout, false otherwise
 */
static bool wait_for_level(gpio_num_t pin, int level, uint32_t timeout_us) {
    uint64_t start_time = esp_timer_get_time();
    
    while ((gpio_get_level(pin) != level)) {
        if ((esp_timer_get_time() - start_time) > timeout_us) {
            return false;
        }
        // Small delay to prevent tight polling
        ets_delay_us(1);
    }
    
    return true;
}

/**
 * @brief Read raw data bits from DHT22 sensor
 * 
 * This function implements the low-level DHT22 communication protocol.
 * It reads 40 bits of data (humidity + temperature + checksum).
 * 
 * @param pin GPIO pin connected to DHT22
 * @param data Buffer to store the 5 bytes of data
 * @return ESP_OK on success, DHT22_ERR_* on failure
 */
static esp_err_t read_raw_data(gpio_num_t pin, uint8_t data[5]) {
    uint32_t timeout_us = DHT22_RESPONSE_TIMEOUT;
    
    // Disable interrupts during critical timing section
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    portENTER_CRITICAL(&mux);
    
    // Send start signal
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, 0);
    ets_delay_us(DHT22_START_LOW_TIME);  // 1ms low
    
    gpio_set_level(pin, 1);
    ets_delay_us(DHT22_START_HIGH_TIME);  // 30us high
    
    // Switch to input mode
    gpio_set_direction(pin, GPIO_MODE_INPUT);
    
    // Wait for sensor response (should go low)
    if (!wait_for_level(pin, 0, timeout_us)) {
        portEXIT_CRITICAL(&mux);
        ESP_LOGD(TAG, "No response from sensor (timeout waiting for low)");
        return DHT22_ERR_NO_RESPONSE;
    }
    
    // Wait for sensor to go high (start of response)
    if (!wait_for_level(pin, 1, timeout_us)) {
        portEXIT_CRITICAL(&mux);
        ESP_LOGD(TAG, "No response from sensor (timeout waiting for high)");
        return DHT22_ERR_NO_RESPONSE;
    }
    
    // Wait for sensor to go low (end of response pulse)
    if (!wait_for_level(pin, 0, timeout_us)) {
        portEXIT_CRITICAL(&mux);
        ESP_LOGD(TAG, "Sensor response timeout");
        return DHT22_ERR_TIMEOUT;
    }
    
    // Read 40 bits of data
    memset(data, 0, 5);
    
    for (int i = 0; i < 40; i++) {
        // Wait for start of bit (high)
        if (!wait_for_level(pin, 1, DHT22_BIT_TIMEOUT)) {
            portEXIT_CRITICAL(&mux);
            ESP_LOGD(TAG, "Timeout waiting for bit %d start", i);
            return DHT22_ERR_TIMEOUT;
        }
        
        // Measure high pulse duration
        uint64_t bit_start = esp_timer_get_time();
        
        // Wait for end of bit (low)
        if (!wait_for_level(pin, 0, DHT22_BIT_TIMEOUT)) {
            portEXIT_CRITICAL(&mux);
            ESP_LOGD(TAG, "Timeout waiting for bit %d end", i);
            return DHT22_ERR_TIMEOUT;
        }
        
        uint32_t bit_duration = esp_timer_get_time() - bit_start;
        
        // Determine bit value based on pulse duration
        // Short pulse (~26-28us) = 0, Long pulse (~70us) = 1
        if (bit_duration > 40) {  // Threshold: 40us
            data[i / 8] |= (1 << (7 - (i % 8)));
        }
    }
    
    portEXIT_CRITICAL(&mux);
    
    return ESP_OK;
}

/**
 * @brief Parse raw data and validate checksum
 * 
 * @param raw_data Raw 5-byte data from sensor
 * @param reading Output reading structure
 * @return ESP_OK if valid, DHT22_ERR_CHECKSUM if checksum fails
 */
static esp_err_t parse_and_validate_data(const uint8_t raw_data[5], dht22_reading_t *reading) {
    // Calculate checksum
    uint8_t calculated_checksum = raw_data[0] + raw_data[1] + raw_data[2] + raw_data[3];
    reading->checksum_received = raw_data[4];
    reading->checksum_calculated = calculated_checksum;
    
    if (calculated_checksum != raw_data[4]) {
        ESP_LOGD(TAG, "Checksum mismatch: calculated=0x%02X, received=0x%02X", 
                calculated_checksum, raw_data[4]);
        return DHT22_ERR_CHECKSUM;
    }
    
    // Parse humidity (first 2 bytes)
    uint16_t humidity_raw = (raw_data[0] << 8) | raw_data[1];
    reading->humidity = humidity_raw / 10.0f;
    
    // Parse temperature (next 2 bytes)
    uint16_t temperature_raw = (raw_data[2] << 8) | raw_data[3];
    
    // Check for negative temperature (MSB of temperature high byte)
    if (temperature_raw & 0x8000) {
        temperature_raw &= 0x7FFF;  // Remove sign bit
        reading->temperature = -(temperature_raw / 10.0f);
    } else {
        reading->temperature = temperature_raw / 10.0f;
    }
    
    // Set timestamp
    reading->timestamp_us = esp_timer_get_time();
    
    // Validate data ranges
    if (!dht22_validate_reading(reading->temperature, reading->humidity)) {
        ESP_LOGD(TAG, "Data out of range: T=%.1f°C, H=%.1f%%", 
                reading->temperature, reading->humidity);
        return DHT22_ERR_INVALID_DATA;
    }
    
    reading->is_valid = true;
    return ESP_OK;
}

/**
 * @brief Update sensor statistics
 * 
 * @param ctx Sensor context
 * @param result Result of the operation
 * @param reading Reading data (if successful)
 */
static void update_statistics(struct dht22_sensor_context *ctx, esp_err_t result, const dht22_reading_t *reading) {
    if (!ctx->config.enable_statistics) {
        return;
    }
    
    ctx->stats.total_readings++;
    
    switch (result) {
        case ESP_OK:
            ctx->stats.successful_readings++;
            ctx->stats.last_successful_read = reading->timestamp_us;
            ctx->stats.last_temperature = reading->temperature;
            ctx->stats.last_humidity = reading->humidity;
            
            // Update min/max values
            if (ctx->stats.successful_readings == 1) {
                // First successful reading
                ctx->stats.min_temperature = reading->temperature;
                ctx->stats.max_temperature = reading->temperature;
                ctx->stats.min_humidity = reading->humidity;
                ctx->stats.max_humidity = reading->humidity;
            } else {
                if (reading->temperature < ctx->stats.min_temperature) {
                    ctx->stats.min_temperature = reading->temperature;
                }
                if (reading->temperature > ctx->stats.max_temperature) {
                    ctx->stats.max_temperature = reading->temperature;
                }
                if (reading->humidity < ctx->stats.min_humidity) {
                    ctx->stats.min_humidity = reading->humidity;
                }
                if (reading->humidity > ctx->stats.max_humidity) {
                    ctx->stats.max_humidity = reading->humidity;
                }
            }
            break;
            
        case DHT22_ERR_TIMEOUT:
            ctx->stats.timeout_errors++;
            break;
            
        case DHT22_ERR_CHECKSUM:
            ctx->stats.checksum_errors++;
            break;
            
        case DHT22_ERR_INVALID_DATA:
            ctx->stats.invalid_data_errors++;
            break;
            
        case DHT22_ERR_TOO_FREQUENT:
            ctx->stats.too_frequent_errors++;
            break;
            
        default:
            break;
    }
}

// Public API implementations

esp_err_t dht22_init(const dht22_config_t *config, dht22_handle_t *handle) {
    if (handle == NULL) {
        ESP_LOGE(TAG, "Handle pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = validate_config(config);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Allocate context
    struct dht22_sensor_context *ctx = calloc(1, sizeof(struct dht22_sensor_context));
    if (ctx == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for sensor context");
        return ESP_ERR_NO_MEM;
    }
    
    // Copy configuration
    memcpy(&ctx->config, config, sizeof(dht22_config_t));
    
    // Initialize statistics
    memset(&ctx->stats, 0, sizeof(dht22_stats_t));
    
    // Create mutex for thread safety
    ctx->mutex = xSemaphoreCreateMutex();
    if (ctx->mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        free(ctx);
        return ESP_ERR_NO_MEM;
    }
    
    // Configure GPIO
    gpio_config_t gpio_conf = {
        .pin_bit_mask = (1ULL << config->data_pin),
        .mode = GPIO_MODE_OUTPUT_OD,  // Open drain for bidirectional communication
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    
    ret = gpio_config(&gpio_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO: %s", esp_err_to_name(ret));
        vSemaphoreDelete(ctx->mutex);
        free(ctx);
        return ret;
    }
    
    // Set initial state
    gpio_set_level(config->data_pin, 1);
    ctx->initialized = true;
    ctx->last_read_time = 0;
    
    *handle = ctx;
    
    ESP_LOGI(TAG, "DHT22 sensor initialized on GPIO %d", config->data_pin);
    return ESP_OK;
}

esp_err_t dht22_deinit(dht22_handle_t handle) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    struct dht22_sensor_context *ctx = (struct dht22_sensor_context*)handle;
    
    if (!ctx->initialized) {
        return DHT22_ERR_NOT_INITIALIZED;
    }
    
    // Reset GPIO to default state
    gpio_reset_pin(ctx->config.data_pin);
    
    // Clean up resources
    if (ctx->mutex) {
        vSemaphoreDelete(ctx->mutex);
    }
    
    ctx->initialized = false;
    free(ctx);
    
    ESP_LOGI(TAG, "DHT22 sensor deinitialized");
    return ESP_OK;
}

esp_err_t dht22_read(dht22_handle_t handle, dht22_reading_t *reading) {
    if (handle == NULL || reading == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    struct dht22_sensor_context *ctx = (struct dht22_sensor_context*)handle;
    
    if (!ctx->initialized) {
        return DHT22_ERR_NOT_INITIALIZED;
    }
    
    // Take mutex for thread safety
    if (xSemaphoreTake(ctx->mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    esp_err_t result = ESP_OK;
    
    // Check minimum interval
    uint64_t current_time = esp_timer_get_time();
    if (ctx->last_read_time > 0) {
        uint64_t time_since_last = current_time - ctx->last_read_time;
        if (time_since_last < (DHT22_MIN_INTERVAL_MS * 1000)) {
            result = DHT22_ERR_TOO_FREQUENT;
            goto cleanup;
        }
    }
    
    // Initialize reading structure
    memset(reading, 0, sizeof(dht22_reading_t));
    
    // Attempt reading with retries
    uint8_t raw_data[5];
    uint32_t retry_count = 0;
    
    do {
        if (retry_count > 0) {
            ESP_LOGD(TAG, "Retry attempt %lu/%lu", retry_count, ctx->config.max_retries);
            vTaskDelay(pdMS_TO_TICKS(ctx->config.retry_delay_ms));
        }
        
        result = read_raw_data(ctx->config.data_pin, raw_data);
        
        if (result == ESP_OK) {
            result = parse_and_validate_data(raw_data, reading);
            if (result == ESP_OK) {
                ctx->last_read_time = current_time;
                break;
            }
        }
        
        retry_count++;
        
    } while (retry_count <= ctx->config.max_retries && result != ESP_OK);
    
    // Update statistics
    update_statistics(ctx, result, reading);
    
    if (ctx->config.enable_debug_logging && result == ESP_OK) {
        ESP_LOGD(TAG, "Reading: T=%.1f°C, H=%.1f%%, Checksum=0x%02X", 
                reading->temperature, reading->humidity, reading->checksum_received);
    }
    
cleanup:
    xSemaphoreGive(ctx->mutex);
    return result;
}

esp_err_t dht22_read_temperature(dht22_handle_t handle, float *temperature) {
    if (temperature == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    dht22_reading_t reading;
    esp_err_t ret = dht22_read(handle, &reading);
    
    if (ret == ESP_OK) {
        *temperature = reading.temperature;
    }
    
    return ret;
}

esp_err_t dht22_read_humidity(dht22_handle_t handle, float *humidity) {
    if (humidity == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    dht22_reading_t reading;
    esp_err_t ret = dht22_read(handle, &reading);
    
    if (ret == ESP_OK) {
        *humidity = reading.humidity;
    }
    
    return ret;
}

esp_err_t dht22_get_stats(dht22_handle_t handle, dht22_stats_t *stats) {
    if (handle == NULL || stats == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    struct dht22_sensor_context *ctx = (struct dht22_sensor_context*)handle;
    
    if (!ctx->initialized) {
        return DHT22_ERR_NOT_INITIALIZED;
    }
    
    if (xSemaphoreTake(ctx->mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(stats, &ctx->stats, sizeof(dht22_stats_t));
        xSemaphoreGive(ctx->mutex);
        return ESP_OK;
    }
    
    return ESP_ERR_TIMEOUT;
}

esp_err_t dht22_reset_stats(dht22_handle_t handle) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    struct dht22_sensor_context *ctx = (struct dht22_sensor_context*)handle;
    
    if (!ctx->initialized) {
        return DHT22_ERR_NOT_INITIALIZED;
    }
    
    if (xSemaphoreTake(ctx->mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memset(&ctx->stats, 0, sizeof(dht22_stats_t));
        xSemaphoreGive(ctx->mutex);
        return ESP_OK;
    }
    
    return ESP_ERR_TIMEOUT;
}

bool dht22_is_ready(dht22_handle_t handle) {
    if (handle == NULL) {
        return false;
    }
    
    struct dht22_sensor_context *ctx = (struct dht22_sensor_context*)handle;
    
    if (!ctx->initialized) {
        return false;
    }
    
    if (ctx->last_read_time == 0) {
        return true;  // Never read before
    }
    
    uint64_t current_time = esp_timer_get_time();
    uint64_t time_since_last = current_time - ctx->last_read_time;
    
    return (time_since_last >= (DHT22_MIN_INTERVAL_MS * 1000));
}

uint32_t dht22_time_until_ready(dht22_handle_t handle) {
    if (handle == NULL) {
        return UINT32_MAX;
    }
    
    struct dht22_sensor_context *ctx = (struct dht22_sensor_context*)handle;
    
    if (!ctx->initialized) {
        return UINT32_MAX;
    }
    
    if (ctx->last_read_time == 0) {
        return 0;  // Ready now
    }
    
    uint64_t current_time = esp_timer_get_time();
    uint64_t time_since_last = current_time - ctx->last_read_time;
    uint64_t required_interval = DHT22_MIN_INTERVAL_MS * 1000;
    
    if (time_since_last >= required_interval) {
        return 0;  // Ready now
    }
    
    return (required_interval - time_since_last) / 1000;  // Convert to ms
}

const char* dht22_error_to_string(esp_err_t error) {
    switch (error) {
        case ESP_OK:
            return "Success";
        case DHT22_ERR_TIMEOUT:
            return "Communication timeout";
        case DHT22_ERR_CHECKSUM:
            return "Checksum mismatch";
        case DHT22_ERR_NO_RESPONSE:
            return "Sensor not responding";
        case DHT22_ERR_INVALID_DATA:
            return "Invalid sensor data";
        case DHT22_ERR_TOO_FREQUENT:
            return "Reading too frequent";
        case DHT22_ERR_NOT_INITIALIZED:
            return "Component not initialized";
        default:
            return esp_err_to_name(error);
    }
}

bool dht22_validate_reading(float temperature, float humidity) {
    return (temperature >= DHT22_TEMP_MIN_C && temperature <= DHT22_TEMP_MAX_C &&
            humidity >= DHT22_HUMIDITY_MIN && humidity <= DHT22_HUMIDITY_MAX);
}

void dht22_print_info(dht22_handle_t handle, bool detailed) {
    if (handle == NULL) {
        ESP_LOGE(TAG, "Invalid handle");
        return;
    }
    
    struct dht22_sensor_context *ctx = (struct dht22_sensor_context*)handle;
    
    if (!ctx->initialized) {
        ESP_LOGE(TAG, "Sensor not initialized");
        return;
    }
    
    ESP_LOGI(TAG, "=== DHT22 Sensor Information ===");
    ESP_LOGI(TAG, "GPIO Pin: %d", ctx->config.data_pin);
    ESP_LOGI(TAG, "Max Retries: %lu", ctx->config.max_retries);
    ESP_LOGI(TAG, "Retry Delay: %lu ms", ctx->config.retry_delay_ms);
    ESP_LOGI(TAG, "Statistics Enabled: %s", ctx->config.enable_statistics ? "Yes" : "No");
    ESP_LOGI(TAG, "Debug Logging: %s", ctx->config.enable_debug_logging ? "Yes" : "No");
    
    if (detailed && ctx->config.enable_statistics) {
        ESP_LOGI(TAG, "--- Statistics ---");
        ESP_LOGI(TAG, "Total Readings: %lu", ctx->stats.total_readings);
        ESP_LOGI(TAG, "Successful: %lu (%.1f%%)", 
                ctx->stats.successful_readings, 
                ctx->stats.total_readings > 0 ? 
                    (100.0f * ctx->stats.successful_readings / ctx->stats.total_readings) : 0.0f);
        ESP_LOGI(TAG, "Timeout Errors: %lu", ctx->stats.timeout_errors);
        ESP_LOGI(TAG, "Checksum Errors: %lu", ctx->stats.checksum_errors);
        ESP_LOGI(TAG, "Invalid Data Errors: %lu", ctx->stats.invalid_data_errors);
        ESP_LOGI(TAG, "Too Frequent Errors: %lu", ctx->stats.too_frequent_errors);
        
        if (ctx->stats.successful_readings > 0) {
            ESP_LOGI(TAG, "Last Temperature: %.1f°C", ctx->stats.last_temperature);
            ESP_LOGI(TAG, "Last Humidity: %.1f%%", ctx->stats.last_humidity);
            ESP_LOGI(TAG, "Temperature Range: %.1f°C to %.1f°C", 
                    ctx->stats.min_temperature, ctx->stats.max_temperature);
            ESP_LOGI(TAG, "Humidity Range: %.1f%% to %.1f%%", 
                    ctx->stats.min_humidity, ctx->stats.max_humidity);
        }
    }
    
    ESP_LOGI(TAG, "Ready for reading: %s", dht22_is_ready(handle) ? "Yes" : "No");
    if (!dht22_is_ready(handle)) {
        ESP_LOGI(TAG, "Time until ready: %lu ms", dht22_time_until_ready(handle));
    }
    ESP_LOGI(TAG, "=============================");
}

float dht22_calculate_heat_index(float temperature_c, float humidity) {
    // Convert to Fahrenheit for calculation
    float temp_f = dht22_celsius_to_fahrenheit(temperature_c);
    
    // Heat index calculation (Rothfusz equation)
    float hi = 0.5 * (temp_f + 61.0 + ((temp_f - 68.0) * 1.2) + (humidity * 0.094));
    
    // If heat index is above 80°F, use more complex formula
    if (hi >= 80.0f) {
        float c1 = -42.379;
        float c2 = 2.04901523;
        float c3 = 10.14333127;
        float c4 = -0.22475541;
        float c5 = -6.83783e-3;
        float c6 = -5.481717e-2;
        float c7 = 1.22874e-3;
        float c8 = 8.5282e-4;
        float c9 = -1.99e-6;
        
        hi = c1 + (c2 * temp_f) + (c3 * humidity) + (c4 * temp_f * humidity) +
             (c5 * temp_f * temp_f) + (c6 * humidity * humidity) +
             (c7 * temp_f * temp_f * humidity) + (c8 * temp_f * humidity * humidity) +
             (c9 * temp_f * temp_f * humidity * humidity);
    }
    
    // Convert back to Celsius
    return dht22_fahrenheit_to_celsius(hi);
}

float dht22_calculate_dew_point(float temperature_c, float humidity) {
    // Magnus formula approximation
    float a = 17.27;
    float b = 237.7;
    
    float alpha = ((a * temperature_c) / (b + temperature_c)) + logf(humidity / 100.0f);
    float dew_point = (b * alpha) / (a - alpha);
    
    return dew_point;
}