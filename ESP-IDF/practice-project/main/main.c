/**
 * @file main.c
 * @brief Smart Environment Data Logger - Main Application
 * 
 * This is the main application file for the Smart Environment Data Logger project.
 * It demonstrates the progression from basic ESP-IDF concepts to advanced IoT features.
 * 
 * Current Implementation: Day 8-10 - DHT22 Sensor Integration
 * 
 * This version showcases:
 * - Professional DHT22 sensor integration with comprehensive error handling
 * - Real-time environmental monitoring with temperature and humidity readings
 * - Advanced LED pattern indicators based on environmental conditions
 * - Interactive button controls for sensor management
 * - System health monitoring and performance tracking
 * - Component interaction patterns for professional firmware development
 * 
 * @author Learning ESP-IDF  
 * @date 2025
 * @version 3.0
 */

#include <stdio.h>
#include "esp_log.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "led_controller.h"
#include "button_controller.h"
#include "system_config.h"
#include "dht22_sensor.h"
#include "mpu6050_sensor.h"
#include "sd_card_storage.h"  // Uncommented SD card storage header
#include "driver/spi_master.h"  // Added for SPI_HOST definitions
#include "esp_timer.h"  // Added for esp_timer_get_time function
#include <math.h>
#include <time.h>  // Added for time functions

#include "wifi_manager.h"
#include "mqtt_client.h"
#include "ota_update.h"

// Forward declaration for SD card handle (temporary workaround)
typedef void* sd_card_handle_t;

// Application tag for logging
static const char *TAG = "MAIN_APP";

// GPIO pin definitions - Day 8-10: Adding DHT22 sensor
#define LED_GPIO_PIN            GPIO_NUM_2     ///< LED pin
#define LED_ACTIVE_LEVEL        1              ///< LED active level
#define BUTTON_GPIO_PIN         GPIO_NUM_0     ///< Button pin
#define BUTTON_ACTIVE_LEVEL     0              ///< Button active level (active low)
#define BUTTON_DEBOUNCE_MS      50             ///< Button debounce time
#define BUTTON_LONG_PRESS_MS    1000           ///< Long press time
#define BUTTON_DOUBLE_CLICK_MS  500            ///< Double click timeout
#define DHT22_DATA_PIN          GPIO_NUM_4     ///< DHT22 sensor data pin

// System health monitoring variables
static uint32_t g_system_uptime_seconds = 0;
static uint32_t g_heap_low_watermark = 0;
static bool g_system_healthy = true;

// Component handles
static button_controller_handle_t g_button_handle = NULL;
static dht22_handle_t g_dht22_handle = NULL;
static mpu6050_handle_t g_mpu6050_handle = NULL;
static sd_card_handle_t g_sd_card_handle = NULL;

// Component handles for new components
static wifi_manager_handle_t g_wifi_handle = NULL;
static mqtt_client_handle_t g_mqtt_handle = NULL;
static ota_handle_t g_ota_handle = NULL;

// DHT22 sensor data
static dht22_reading_t g_last_sensor_reading = {0};
static bool g_sensor_data_valid = false;

// MPU6050 sensor data
static mpu6050_data_t g_last_mpu_reading = {0};
static bool g_mpu_data_valid = false;

// System state management
static volatile bool g_led_auto_mode = true;  // Flag for automatic LED blinking
static volatile int g_blink_pattern = 0;      // Current blink pattern index
static bool g_components_initialized = false;

// FreeRTOS timer for system monitoring
static TimerHandle_t g_system_monitor_timer = NULL;

/**
 * @brief System health monitoring callback
 * 
 * This function is called periodically to monitor system health,
 * track resource usage, and detect potential issues.
 * 
 * @param xTimer Timer handle (unused)
 */
void system_health_monitor_callback(TimerHandle_t xTimer) {
    // Update system uptime
    g_system_uptime_seconds += 10; // Called every 10 seconds
    
    // Monitor heap usage
    uint32_t free_heap = esp_get_free_heap_size();
    uint32_t min_free_heap = esp_get_minimum_free_heap_size();
    
    // Update low watermark if this is the lowest we've seen
    if (g_heap_low_watermark == 0 || min_free_heap < g_heap_low_watermark) {
        g_heap_low_watermark = min_free_heap;
    }
    
    // Check system health indicators
    bool heap_healthy = (free_heap > 50000); // Require at least 50KB free
    bool uptime_reasonable = (g_system_uptime_seconds < 86400); // Less than 24 hours is fine
    
    g_system_healthy = heap_healthy && uptime_reasonable;
    
    // Log detailed health information every minute (6 calls)
    static uint8_t health_log_counter = 0;
    health_log_counter++;
    
    if (health_log_counter >= 6) {
        health_log_counter = 0;
        
        ESP_LOGI(TAG, "=== System Health Report ===");
        ESP_LOGI(TAG, "Uptime: %lu seconds (%lu minutes)", g_system_uptime_seconds, g_system_uptime_seconds / 60);
        ESP_LOGI(TAG, "Free Heap: %lu bytes (Low watermark: %lu bytes)", free_heap, g_heap_low_watermark);
        ESP_LOGI(TAG, "System Status: %s", g_system_healthy ? "HEALTHY" : "WARNING");
        
        // Get button statistics if available
        if (g_button_handle) {
            uint32_t presses, releases, long_presses, double_clicks;
            if (button_controller_get_stats(g_button_handle, &presses, &releases, &long_presses, &double_clicks) == ESP_OK) {
                ESP_LOGI(TAG, "Button Stats: P:%lu R:%lu LP:%lu DC:%lu", presses, releases, long_presses, double_clicks);
            }
        }
        ESP_LOGI(TAG, "=========================");
        
        // Trigger garbage collection if heap is getting low
        if (free_heap < 100000) { // Less than 100KB
            ESP_LOGW(TAG, "Low heap detected, triggering cleanup");
            // In a real application, you might clean up caches, buffers, etc.
        }
    }
}

/**
 * @brief Configuration change callback
 * 
 * This function is called when the system configuration changes,
 * allowing the application to adapt to new settings.
 * 
 * @param config Pointer to the updated configuration
 * @param user_data User data (unused)
 */
void configuration_change_callback(const system_config_t *config, void *user_data) {
    ESP_LOGI(TAG, "Configuration change detected");
    
    // In a real application, you would:
    // 1. Update component configurations
    // 2. Restart affected services
    // 3. Validate new settings
    
    ESP_LOGI(TAG, "New LED pin: %d, Button pin: %d", 
             config->gpio.led_pin, config->gpio.button_pin);
    ESP_LOGI(TAG, "New timing - Debounce: %lu ms, Long press: %lu ms",
             config->timing.button_debounce_ms, config->timing.button_long_press_ms);
}

/**
 * @brief Initialize system health monitoring
 * 
 * Sets up the system health monitoring timer and related infrastructure.
 * 
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
esp_err_t init_system_monitoring(void) {
    ESP_LOGI(TAG, "Initializing system health monitoring");
    
    // Create system monitor timer (10 second interval)
    g_system_monitor_timer = xTimerCreate(
        "SysMonitor",                    // Timer name
        pdMS_TO_TICKS(10000),           // Period: 10 seconds
        pdTRUE,                         // Auto-reload
        NULL,                           // Timer ID (unused)
        system_health_monitor_callback  // Callback function
    );
    
    if (g_system_monitor_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create system monitor timer");
        return ESP_ERR_NO_MEM;
    }
    
    // Start the timer
    if (xTimerStart(g_system_monitor_timer, pdMS_TO_TICKS(1000)) != pdPASS) {
        ESP_LOGE(TAG, "Failed to start system monitor timer");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Initialize health tracking variables
    g_heap_low_watermark = esp_get_free_heap_size();
    g_system_uptime_seconds = 0;
    g_system_healthy = true;
    
    ESP_LOGI(TAG, "System health monitoring started");
    return ESP_OK;
}

/**
 * @brief Advanced button event callback with error handling
 * 
 * This enhanced version includes comprehensive error handling,
 * state validation, and performance monitoring.
 * 
 * @param event The button event that occurred
 * @param user_data User data pointer (unused in this example)
 */
void button_event_callback(button_event_t event, void *user_data) {
    // Validate system state before processing events
    if (!g_components_initialized) {
        ESP_LOGW(TAG, "Button event received before components initialized");
        return;
    }
    
    // Log the event with timestamp for debugging
    const char *event_str = button_controller_event_to_string(event);
    ESP_LOGI(TAG, "Button Event: %s (Uptime: %lu s)", event_str, g_system_uptime_seconds);
    
    // Handle different button events with error checking
    esp_err_t result = ESP_OK;
    
    switch (event) {
        case BUTTON_EVENT_PRESSED:
            // Turn on LED immediately when button is pressed
            result = led_controller_set_state(LED_STATE_ON);
            if (result != ESP_OK) {
                ESP_LOGE(TAG, "Failed to turn on LED: %s", esp_err_to_name(result));
            }
            break;
            
        case BUTTON_EVENT_RELEASED:
            // Turn off LED when button is released (if not in auto mode)
            if (!g_led_auto_mode) {
                result = led_controller_set_state(LED_STATE_OFF);
                if (result != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to turn off LED: %s", esp_err_to_name(result));
                }
            }
            break;
            
        case BUTTON_EVENT_CLICK:
            // Single click: Toggle auto mode
            g_led_auto_mode = !g_led_auto_mode;
            ESP_LOGI(TAG, "Auto LED mode %s", g_led_auto_mode ? "ENABLED" : "DISABLED");
            
            if (!g_led_auto_mode) {
                // If auto mode disabled, turn off LED
                result = led_controller_set_state(LED_STATE_OFF);
                if (result != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to disable auto mode: %s", esp_err_to_name(result));
                }
            }
            break;
            
        case BUTTON_EVENT_LONG_PRESS:
            // Long press: Show system status and reset statistics
            ESP_LOGI(TAG, "=== Long Press - System Status ===");
            ESP_LOGI(TAG, "DHT22 sensor information will be displayed");
            
            // Reset button statistics
            result = button_controller_reset(g_button_handle);
            if (result != ESP_OK) {
                ESP_LOGE(TAG, "Failed to reset button controller: %s", esp_err_to_name(result));
            } else {
                ESP_LOGI(TAG, "Button statistics reset");
            }
            
            // Reset to known good state
            g_led_auto_mode = true;
            g_blink_pattern = 0;
            break;
            
        case BUTTON_EVENT_DOUBLE_CLICK:
            // Double click: Change blink pattern
            g_blink_pattern = (g_blink_pattern + 1) % 4;  // Cycle through 4 patterns
            ESP_LOGI(TAG, "Blink pattern changed to: %d", g_blink_pattern);
            break;
            
        default:
            ESP_LOGW(TAG, "Unhandled button event: %d", event);
            break;
    }
    
    // Update system health based on component responses
    if (result != ESP_OK) {
        g_system_healthy = false;
        ESP_LOGW(TAG, "System health degraded due to component error");
    }
}

/**
 * @brief Execute different LED blink patterns
 * 
 * Demonstrates different timing patterns based on button interactions.
 * 
 * @param pattern Pattern index (0-3)
 */
void execute_blink_pattern(int pattern) {
    switch (pattern) {
        case 0:
            // Pattern 0: Normal blink (500ms on, 500ms off)
            led_controller_blink(500, 500, 2000);
            break;
            
        case 1:
            // Pattern 1: Fast blink (200ms on, 200ms off)
            led_controller_blink(200, 200, 1600);
            break;
            
        case 2:
            // Pattern 2: Slow blink (1000ms on, 1000ms off)
            led_controller_blink(1000, 1000, 4000);
            break;
            
        case 3:
            // Pattern 3: Morse SOS (Short-Short-Short-Long-Long-Long-Short-Short-Short)
            // Short blinks
            for (int i = 0; i < 3; i++) {
                led_controller_set_state(LED_STATE_ON);
                vTaskDelay(pdMS_TO_TICKS(150));
                led_controller_set_state(LED_STATE_OFF);
                vTaskDelay(pdMS_TO_TICKS(150));
            }
            
            vTaskDelay(pdMS_TO_TICKS(300));
            
            // Long blinks
            for (int i = 0; i < 3; i++) {
                led_controller_set_state(LED_STATE_ON);
                vTaskDelay(pdMS_TO_TICKS(500));
                led_controller_set_state(LED_STATE_OFF);
                vTaskDelay(pdMS_TO_TICKS(150));
            }
            
            vTaskDelay(pdMS_TO_TICKS(300));
            
            // Short blinks again
            for (int i = 0; i < 3; i++) {
                led_controller_set_state(LED_STATE_ON);
                vTaskDelay(pdMS_TO_TICKS(150));
                led_controller_set_state(LED_STATE_OFF);
                vTaskDelay(pdMS_TO_TICKS(150));
            }
            break;
    }
}

/**
 * @brief Print system status and statistics
 * 
 * Displays current system state and button usage statistics.
 */
void print_system_status(void) {
    // Get button statistics
    uint32_t total_presses, total_releases, total_long_presses, total_double_clicks;
    button_controller_get_stats(g_button_handle, 
                               &total_presses, 
                               &total_releases, 
                               &total_long_presses, 
                               &total_double_clicks);
    
    // Get current button state
    button_state_t btn_state = button_controller_get_state(g_button_handle);
    bool is_pressed = button_controller_is_pressed(g_button_handle);
    uint32_t press_duration = button_controller_get_press_duration(g_button_handle);
    
    // Get LED state
    led_state_t led_state = led_controller_get_state();
    
    ESP_LOGI(TAG, "=== System Status ===");
    ESP_LOGI(TAG, "Button State: %s (Raw: %s)", 
             is_pressed ? "PRESSED" : "RELEASED",
             btn_state == BUTTON_STATE_PRESSED ? "PRESSED" : "RELEASED");
    
    if (is_pressed) {
        ESP_LOGI(TAG, "Press Duration: %lu ms", press_duration);
    }
    
    ESP_LOGI(TAG, "LED State: %s", led_state == LED_STATE_ON ? "ON" : "OFF");
    ESP_LOGI(TAG, "Auto Mode: %s, Pattern: %d", 
             g_led_auto_mode ? "ENABLED" : "DISABLED", g_blink_pattern);
    
    ESP_LOGI(TAG, "Button Statistics:");
    ESP_LOGI(TAG, "  Total Presses: %lu", total_presses);
    ESP_LOGI(TAG, "  Total Releases: %lu", total_releases);
    ESP_LOGI(TAG, "  Long Presses: %lu", total_long_presses);
    ESP_LOGI(TAG, "  Double Clicks: %lu", total_double_clicks);
    
    ESP_LOGI(TAG, "Free Heap: %lu bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "=====================");
}

/**
 * @brief Read all sensors and display data
 * 
 * This function reads all available sensors (DHT22, MPU6050) and displays the data
 * through different LED patterns and logging.
 */
void read_and_display_sensor_data(void) {
    bool any_sensor_valid = false;
    
    // Read DHT22 sensor if available
    if (g_dht22_handle != NULL) {
        esp_err_t ret = dht22_read(g_dht22_handle, &g_last_sensor_reading);
        
        if (ret == ESP_OK) {
            g_sensor_data_valid = true;
            any_sensor_valid = true;
            
            ESP_LOGI(TAG, "Environmental Data:");
            ESP_LOGI(TAG, "  Temperature: %.1f°C (%.1f°F)", 
                    g_last_sensor_reading.temperature, 
                    dht22_celsius_to_fahrenheit(g_last_sensor_reading.temperature));
            ESP_LOGI(TAG, "  Humidity: %.1f%%", g_last_sensor_reading.humidity);
            ESP_LOGI(TAG, "  Heat Index: %.1f°C", 
                    dht22_calculate_heat_index(g_last_sensor_reading.temperature, g_last_sensor_reading.humidity));
            ESP_LOGI(TAG, "  Dew Point: %.1f°C", 
                    dht22_calculate_dew_point(g_last_sensor_reading.temperature, g_last_sensor_reading.humidity));
        } else {
            ESP_LOGW(TAG, "Failed to read DHT22 sensor: %s", dht22_error_to_string(ret));
            g_sensor_data_valid = false;
        }
    }
    
    // Read MPU6050 sensor if available
    if (g_mpu6050_handle != NULL) {
        mpu6050_data_t mpu_data;
        esp_err_t ret = mpu6050_read_processed(g_mpu6050_handle, &mpu_data);
        
        if (ret == ESP_OK) {
            g_mpu_data_valid = true;
            any_sensor_valid = true;
            
            // Store the data for use in LED patterns
            g_last_mpu_reading = mpu_data;
            
            ESP_LOGI(TAG, "Motion Data:");
            ESP_LOGI(TAG, "  Acceleration: X=%.2f Y=%.2f Z=%.2f m/s²", 
                    mpu_data.accel_x, mpu_data.accel_y, mpu_data.accel_z);
            ESP_LOGI(TAG, "  Gyroscope: X=%.2f Y=%.2f Z=%.2f °/s", 
                    mpu_data.gyro_x, mpu_data.gyro_y, mpu_data.gyro_z);
            ESP_LOGI(TAG, "  Orientation: Roll=%.1f° Pitch=%.1f° Yaw=%.1f°", 
                    mpu_data.roll, mpu_data.pitch, mpu_data.yaw);
            ESP_LOGI(TAG, "  Temperature: %.1f°C", mpu_data.temp_celsius);
        } else {
            ESP_LOGW(TAG, "Failed to read MPU6050 sensor: %s", mpu6050_error_to_string(ret));
            g_mpu_data_valid = false;
        }
    }
    
    // Log sensor data to SD card if available
    if (any_sensor_valid && g_sd_card_handle != NULL) {
        // Forward declaration for sensor log entry (temporary workaround)
        /*
        typedef struct {
            time_t timestamp;
            float temperature;
            float humidity;
            float accel_x;
            float accel_y;
            float accel_z;
            float gyro_x;
            float gyro_y;
            float gyro_z;
            float pressure;
            char location[32];
        */
        
        // Create sensor log entry
        sensor_log_entry_t log_entry = {0};
        log_entry.timestamp = (time_t)(esp_timer_get_time() / 1000000);  // Convert microseconds to seconds
        
        // Fill in data from available sensors
        if (g_sensor_data_valid) {
            log_entry.temperature = g_last_sensor_reading.temperature;
            log_entry.humidity = g_last_sensor_reading.humidity;
        }
        
        if (g_mpu_data_valid) {
            log_entry.accel_x = g_last_mpu_reading.accel_x;
            log_entry.accel_y = g_last_mpu_reading.accel_y;
            log_entry.accel_z = g_last_mpu_reading.accel_z;
            log_entry.gyro_x = g_last_mpu_reading.gyro_x;
            log_entry.gyro_y = g_last_mpu_reading.gyro_y;
            log_entry.gyro_z = g_last_mpu_reading.gyro_z;
        }
        
        // Set default location
        strncpy(log_entry.location, "default", sizeof(log_entry.location) - 1);
        log_entry.location[sizeof(log_entry.location) - 1] = '\0';
        
        // Log data to SD card (commented out until component is fully integrated)
        esp_err_t ret = sd_card_write_sensor_log(g_sd_card_handle, "data.csv", &log_entry);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to log sensor data to SD card: %s", sd_card_error_to_string(ret));
        } else {
            ESP_LOGD(TAG, "Sensor data logged to SD card successfully");
        }
    }
    
    // LED indication based on sensor data
    if (any_sensor_valid) {
        // If we have environmental data, use temperature-based patterns
        if (g_sensor_data_valid) {
            if (g_last_sensor_reading.temperature < 20.0f) {
                // Cold - slow blue-ish blink
                led_controller_blink(1000, 1000, 2000);
            } else if (g_last_sensor_reading.temperature < 25.0f) {
                // Comfortable - normal green blink
                led_controller_blink(500, 500, 1000);
            } else {
                // Warm/Hot - fast red-ish blink
                led_controller_blink(200, 200, 800);
            }
        } 
        // If we only have motion data, use acceleration-based patterns
        else if (g_mpu_data_valid) {
            float total_accel = sqrtf(g_last_mpu_reading.accel_x * g_last_mpu_reading.accel_x +
                                     g_last_mpu_reading.accel_y * g_last_mpu_reading.accel_y +
                                     g_last_mpu_reading.accel_z * g_last_mpu_reading.accel_z);
            
            if (total_accel < 9.0f) {
                // Stationary - slow blink
                led_controller_blink(800, 800, 1600);
            } else if (total_accel < 12.0f) {
                // Low motion - medium blink
                led_controller_blink(400, 400, 800);
            } else {
                // High motion - fast blink
                led_controller_blink(100, 100, 400);
            }
        }
    } else {
        // No valid sensor data - error pattern
        ESP_LOGW(TAG, "No valid sensor data available");
        
        // Error pattern - quick double blink
        led_controller_set_state(LED_STATE_ON);
        vTaskDelay(pdMS_TO_TICKS(100));
        led_controller_set_state(LED_STATE_OFF);
        vTaskDelay(pdMS_TO_TICKS(100));
        led_controller_set_state(LED_STATE_ON);
        vTaskDelay(pdMS_TO_TICKS(100));
        led_controller_set_state(LED_STATE_OFF);
    }
}

/**
 * @brief WiFi event callback
 * 
 * Handles WiFi events and updates system state accordingly
 * 
 * @param event The WiFi event that occurred
 * @param user_data User data (unused)
 */
void wifi_event_callback(wifi_event_t event, void* user_data) {
    ESP_LOGI(TAG, "WiFi Event: %s", wifi_manager_event_to_string(event));
    
    switch (event) {
        case WIFI_EVENT_CONNECTED:
            ESP_LOGI(TAG, "WiFi connected, initializing MQTT client");
            // Start MQTT client when WiFi connects
            if (g_mqtt_handle) {
                mqtt_client_connect(g_mqtt_handle, false);
            }
            break;
            
        case WIFI_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "WiFi disconnected, stopping MQTT client");
            // Stop MQTT client when WiFi disconnects
            if (g_mqtt_handle) {
                mqtt_client_disconnect(g_mqtt_handle);
            }
            break;
            
        case WIFI_EVENT_OFFLINE_MODE_ENTERED:
            ESP_LOGI(TAG, "Entered offline mode");
            // Notify MQTT client of offline mode
            if (g_mqtt_handle) {
                mqtt_client_enter_offline_mode(g_mqtt_handle);
            }
            // Defer OTA updates
            if (g_ota_handle) {
                ota_set_deferred(g_ota_handle, true);
            }
            break;
            
        case WIFI_EVENT_OFFLINE_MODE_EXITED:
            ESP_LOGI(TAG, "Exited offline mode");
            // Notify MQTT client to exit offline mode
            if (g_mqtt_handle) {
                mqtt_client_exit_offline_mode(g_mqtt_handle);
            }
            // Enable OTA updates
            if (g_ota_handle) {
                ota_set_deferred(g_ota_handle, false);
            }
            break;
            
        default:
            break;
    }
}

/**
 * @brief MQTT event callback
 * 
 * Handles MQTT events and updates system state accordingly
 * 
 * @param event The MQTT event that occurred
 * @param message Pointer to message data (if applicable)
 * @param user_data User data (unused)
 */
void mqtt_event_callback(mqtt_event_t event, const mqtt_message_t* message, void* user_data) {
    ESP_LOGI(TAG, "MQTT Event: %s", mqtt_client_event_to_string(event));
    
    switch (event) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT connected, subscribing to topics");
            // Subscribe to relevant topics
            mqtt_client_subscribe(g_mqtt_handle, "smart_logger/commands", MQTT_QOS_AT_LEAST_ONCE);
            break;
            
        case MQTT_EVENT_DATA_RECEIVED:
            ESP_LOGI(TAG, "MQTT data received on topic: %s", message->topic);
            // Process received MQTT messages
            break;
            
        case MQTT_EVENT_OFFLINE_SYNC_COMPLETED:
            ESP_LOGI(TAG, "Completed synchronization of offline messages");
            break;
            
        default:
            break;
    }
}

/**
 * @brief OTA event callback
 * 
 * Handles OTA events and updates system state accordingly
 * 
 * @param event The OTA event that occurred
 * @param progress Pointer to progress information (if applicable)
 * @param user_data User data (unused)
 */
void ota_event_callback(ota_event_t event, const ota_progress_t* progress, void* user_data) {
    ESP_LOGI(TAG, "OTA Event: %s", ota_event_to_string(event));
    
    switch (event) {
        case OTA_EVENT_UPDATE_AVAILABLE:
            ESP_LOGI(TAG, "New firmware update available");
            // In a real application, you might want to automatically start the update
            // or notify the user based on your application requirements
            break;
            
        case OTA_EVENT_UPDATE_SUCCESS:
            ESP_LOGI(TAG, "Firmware update successful, rebooting...");
            // In a real application, you would reboot here
            // esp_restart();
            break;
            
        case OTA_EVENT_UPDATE_FAILED:
            ESP_LOGE(TAG, "Firmware update failed");
            break;
            
        default:
            break;
    }
}

/**
 * @brief Main application entry point
 * 
 * This function demonstrates advanced ESP-IDF concepts:
 * - Button interrupt handling with debouncing
 * - Event-driven programming with callbacks
 * - Component interaction (LED + Button)
 * - FreeRTOS task management
 * - System monitoring and statistics
 */
void app_main(void)
{
    // Declare all variables at function start
    esp_err_t ret;
    
    ESP_LOGI(TAG, "=== Smart Environment Data Logger Started ===");
    ESP_LOGI(TAG, "Day 8-10: DHT22 Sensor + System Config Integration");
    ESP_LOGI(TAG, "ESP-IDF Version: %s", esp_get_idf_version());
    
    // Step 1: Initialize system configuration first
    ret = system_config_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize system config: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "System configuration initialized");
    
    // Step 2: Register for configuration change notifications
    ret = system_config_register_callback(configuration_change_callback, NULL);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to register config callback: %s", esp_err_to_name(ret));
    }
    
    // Step 3: Get configuration values
    const system_config_t *config = system_config_get();
    if (config == NULL) {
        ESP_LOGE(TAG, "Failed to get system configuration");
        return;
    }
    
    // Get network configuration
    const system_network_config_t *network_config = system_config_get_network();
    if (network_config == NULL) {
        ESP_LOGE(TAG, "Failed to get network configuration");
        return;
    }
    
    // Get sensor configuration
    const system_sensor_config_t *sensor_config = system_config_get_sensors();
    if (sensor_config == NULL) {
        ESP_LOGE(TAG, "Failed to get sensor configuration");
        return;
    }
    
    // Step 4: Initialize components using configuration values
    led_config_t led_config = {
        .gpio_pin = config->gpio.led_pin,
        .active_level = config->gpio.led_active_level
    };
    
    button_config_t button_config = {
        .gpio_pin = config->gpio.button_pin,
        .active_level = config->gpio.button_active_level,
        .pull_up_enable = true,
        .pull_down_enable = false,
        .debounce_time_ms = config->timing.button_debounce_ms,
        .long_press_time_ms = config->timing.button_long_press_ms,
        .double_click_timeout_ms = config->timing.button_double_click_ms
    };
    
    dht22_config_t dht22_config = {
        .data_pin = config->gpio.dht22_pin,
        .max_retries = 3,
        .retry_delay_ms = 100,
        .enable_statistics = true,
        .enable_debug_logging = true,
        .timeout_us = 0
    };
    
    // MPU6050 configuration using system config values
    mpu6050_config_t mpu6050_config = {
        .sda_pin = config->gpio.mpu6050_sda_pin,
        .scl_pin = config->gpio.mpu6050_scl_pin,
        .int_pin = -1,  // No interrupt pin used
        .i2c_address = sensor_config->mpu6050_i2c_address,
        .i2c_port = I2C_NUM_0,
        .i2c_frequency = sensor_config->mpu6050_i2c_frequency,
        .accel_range = sensor_config->mpu6050_accel_range,
        .gyro_range = sensor_config->mpu6050_gyro_range,
        .dlpf_mode = sensor_config->mpu6050_dlpf_mode,
        .enable_dmp = false,  // DMP not implemented yet
        .enable_interrupt = false,
        .enable_statistics = sensor_config->mpu6050_enable_statistics,
        .enable_debug_logging = sensor_config->mpu6050_enable_debug,
        .timeout_ms = 100
    };
    
    ESP_LOGI(TAG, "Initializing components with configuration-based settings...");
    ESP_LOGI(TAG, "Configuration: LED=GPIO%d, Button=GPIO%d, DHT22=GPIO%d", 
             config->gpio.led_pin, config->gpio.button_pin, config->gpio.dht22_pin);
    
    // Initialize LED controller
    ret = led_controller_init(&led_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LED controller: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "LED controller initialized");
    
    // Initialize button controller with callback
    ret = button_controller_init(&button_config, button_event_callback, NULL, &g_button_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize button controller: %s", esp_err_to_name(ret));
        led_controller_deinit();
        return;
    }
    ESP_LOGI(TAG, "Button controller initialized with interrupt support");
    
    // Initialize DHT22 sensor
    ret = dht22_init(&dht22_config, &g_dht22_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize DHT22 sensor: %s", esp_err_to_name(ret));
        button_controller_deinit(g_button_handle);
        led_controller_deinit();
        return;
    }
    ESP_LOGI(TAG, "DHT22 sensor initialized on GPIO %d", DHT22_DATA_PIN);
    
    // Initialize MPU6050 sensor
    ret = mpu6050_init(&mpu6050_config, &g_mpu6050_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MPU6050 sensor: %s", esp_err_to_name(ret));
        // Don't exit - MPU6050 is optional, continue with other sensors
        g_mpu6050_handle = NULL;
    } else {
        ESP_LOGI(TAG, "MPU6050 sensor initialized on SDA=GPIO%d, SCL=GPIO%d", 
                 mpu6050_config.sda_pin, mpu6050_config.scl_pin);
        
        // Perform auto-calibration if enabled
        if (sensor_config->mpu6050_enable_calibration) {
            ESP_LOGI(TAG, "Starting MPU6050 auto-calibration...");
            ret = mpu6050_calibrate(g_mpu6050_handle, sensor_config->mpu6050_calibration_samples);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "MPU6050 calibration completed successfully");
            } else {
                ESP_LOGW(TAG, "MPU6050 calibration failed: %s", mpu6050_error_to_string(ret));
            }
        }
    }
    
    // Initialize SD card storage component
    // Get SD card configuration from system config
    const system_gpio_config_t *gpio_config = system_config_get_gpio();
    if (gpio_config != NULL) {
        sd_card_config_t sd_card_config = {
            .cs_pin = gpio_config->sd_cs_pin,
            .mosi_pin = gpio_config->sd_mosi_pin,
            .miso_pin = gpio_config->sd_miso_pin,
            .clk_pin = gpio_config->sd_clk_pin,
            .spi_host = SPI2_HOST,  // Use the correct SPI host definition
            .max_files = 5,
            .format_if_mount_failed = false,
            .allocation_unit_size = 16 * 1024
        };
        
        // Initialize SD card storage (commented out until component is fully integrated)
        ret = sd_card_init(&sd_card_config, &g_sd_card_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize SD card storage: %s", sd_card_error_to_string(ret));
            g_sd_card_handle = NULL;
        } else {
            ESP_LOGI(TAG, "SD card storage initialized on SPI2 (CS=GPIO%d)", sd_card_config.cs_pin);
            
            // Create initial log file
            ret = sd_card_create_log_file(g_sd_card_handle, "data.csv");
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to create sensor log file: %s", sd_card_error_to_string(ret));
            } else {
                ESP_LOGI(TAG, "Sensor log file created successfully");
            }
        }
    } else {
        ESP_LOGW(TAG, "Failed to get GPIO configuration for SD card");
        g_sd_card_handle = NULL;
    }
    
    // Initialize WiFi manager
    wifi_manager_config_t wifi_config = {
        .ssid = network_config->wifi_ssid,
        .password = network_config->wifi_password,
        .connect_timeout_ms = 10000,
        .retry_interval_ms = 5000,
        .max_retry_count = 3,
        .auto_reconnect = true
    };
    
    ret = wifi_manager_init(&wifi_config, &g_wifi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize WiFi manager: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "WiFi manager initialized");
        // Register WiFi event callback
        wifi_manager_register_callback(g_wifi_handle, wifi_event_callback, NULL);
    }
    
    // Initialize MQTT client
    mqtt_client_config_t mqtt_config = {
        .broker_uri = network_config->mqtt_broker_uri,
        .client_id = "smart_logger_client",
        .username = NULL,  // Add if your broker requires authentication
        .password = NULL,  // Add if your broker requires authentication
        .connect_timeout_ms = 10000,
        .retry_interval_ms = 5000,
        .max_retry_count = 3,
        .auto_reconnect = true,
        .keepalive = 120,
        .use_tls = network_config->mqtt_use_tls
    };
    
    ret = mqtt_client_init(&mqtt_config, &g_mqtt_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "MQTT client initialized");
        // Register MQTT event callback
        mqtt_client_register_callback(g_mqtt_handle, mqtt_event_callback, NULL);
    }
    
    // Initialize OTA update component
    ota_config_t ota_config = {
        .update_url = "https://your-update-server.com/firmware.bin",  // Update with your server URL
        .cert_pem = NULL,  // Add certificate for secure connection if needed
        .timeout_ms = 10000,
        .retry_interval_ms = 5000,
        .max_retry_count = 3,
        .auto_update = false,  // Set to true for automatic updates
        .buffer_size = 1024,
        .check_interval_ms = 3600000  // Check for updates every hour
    };
    
    ret = ota_init(&ota_config, &g_ota_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize OTA update component: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "OTA update component initialized");
        // Register OTA event callback
        ota_register_callback(g_ota_handle, ota_event_callback, NULL);
    }
    
    // Initialize system monitoring
    ret = init_system_monitoring();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to initialize system monitoring: %s", esp_err_to_name(ret));
        // Continue anyway - this is not critical
    }
    
    // Mark components as initialized
    g_components_initialized = true;
    
    // Display usage instructions
    ESP_LOGI(TAG, "=== Usage Instructions ===");
    ESP_LOGI(TAG, "- Single Click: Toggle auto LED mode");
    ESP_LOGI(TAG, "- Double Click: Change blink pattern (0-3)");
    ESP_LOGI(TAG, "- Long Press (1s): Reset statistics and show sensor info");
    ESP_LOGI(TAG, "- Hold Button: LED stays on while pressed");
    ESP_LOGI(TAG, "- Automatic: DHT22 readings every 5 seconds with LED patterns");
    ESP_LOGI(TAG, "==========================");
    
    // Initial status display
    vTaskDelay(pdMS_TO_TICKS(1000));
    print_system_status();
    
    // Wait 3 seconds before first sensor reading (DHT22 needs time to stabilize)
    ESP_LOGI(TAG, "Waiting for DHT22 sensor to stabilize...");
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // Connect to WiFi (this will trigger MQTT connection via callback when successful)
    if (g_wifi_handle) {
        wifi_manager_connect(g_wifi_handle, false);
    }
    
    // Main application loop with network integration
    int loop_count = 0;
    uint32_t last_sensor_read = 0;
    uint32_t last_ota_check = 0;
    const uint32_t SENSOR_READ_INTERVAL = 5000; // 5 seconds
    const uint32_t OTA_CHECK_INTERVAL = 300000; // 5 minutes
    
    while (1) {
        loop_count++;
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        // Read sensor every 5 seconds
        if (current_time - last_sensor_read >= SENSOR_READ_INTERVAL) {
            ESP_LOGI(TAG, "=== Sensor Reading %d ===", loop_count);
            read_and_display_sensor_data();
            last_sensor_read = current_time;
            
            // Show sensor statistics periodically
            if (loop_count % 5 == 0) {
                dht22_print_info(g_dht22_handle, true);
            }
            
            // Publish sensor data to MQTT if connected
            if (g_mqtt_handle && mqtt_client_is_connected(g_mqtt_handle)) {
                char payload[256];
                snprintf(payload, sizeof(payload), 
                        "{\"temperature\": %.1f, \"humidity\": %.1f, \"timestamp\": %lu}",
                        g_last_sensor_reading.temperature,
                        g_last_sensor_reading.humidity,
                        (unsigned long)time(NULL));
                
                mqtt_client_publish(g_mqtt_handle, 
                                  "smart_logger/sensor_data", 
                                  payload, 
                                  strlen(payload), 
                                  MQTT_QOS_AT_LEAST_ONCE, 
                                  false);
            }
        }
        
        // Check for OTA updates periodically
        if (current_time - last_ota_check >= OTA_CHECK_INTERVAL) {
            if (g_ota_handle && !wifi_manager_is_offline_mode(g_wifi_handle)) {
                ESP_LOGI(TAG, "Checking for firmware updates");
                ota_check_for_updates(g_ota_handle, false);
            }
            last_ota_check = current_time;
        }
        
        // Auto LED blinking mode (only if no recent sensor activity)
        if (g_led_auto_mode && !g_sensor_data_valid) {
            ESP_LOGD(TAG, "Executing auto blink pattern %d (loop %d)", g_blink_pattern, loop_count);
            execute_blink_pattern(g_blink_pattern);
        } else if (!g_led_auto_mode) {
            // In manual mode, just wait
            ESP_LOGD(TAG, "Manual mode - waiting for button events (loop %d)", loop_count);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        
        // Print system status every 10 loops (approximately every 50 seconds)
        if (loop_count % 10 == 0) {
            print_system_status();
            
            // Print network status
            if (g_wifi_handle) {
                ESP_LOGI(TAG, "WiFi State: %s", wifi_manager_state_to_string(wifi_manager_get_state(g_wifi_handle)));
                if (wifi_manager_is_connected(g_wifi_handle)) {
                    char ip_addr[16];
                    wifi_manager_get_ip_info(g_wifi_handle, ip_addr, NULL, NULL);
                    ESP_LOGI(TAG, "IP Address: %s", ip_addr);
                }
            }
            
            if (g_mqtt_handle) {
                ESP_LOGI(TAG, "MQTT State: %s", mqtt_client_state_to_string(mqtt_client_get_state(g_mqtt_handle)));
                uint32_t queued_count;
                if (mqtt_client_get_queued_message_count(g_mqtt_handle, &queued_count) == ESP_OK) {
                    ESP_LOGI(TAG, "Queued Messages: %lu", queued_count);
                }
            }
        }
        
        // Add delay between main loop iterations
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    // Cleanup (this code won't be reached due to infinite loop above)
    ESP_LOGI(TAG, "Cleaning up components...");
    dht22_deinit(g_dht22_handle);
    if (g_mpu6050_handle) {
        mpu6050_deinit(g_mpu6050_handle);
    }
    // SD card deinitialization (commented out until component is fully integrated)
    if (g_sd_card_handle) {
        sd_card_deinit(g_sd_card_handle);
    }
    if (g_ota_handle) {
        ota_deinit(g_ota_handle);
    }
    if (g_mqtt_handle) {
        mqtt_client_deinit(g_mqtt_handle);
    }
    if (g_wifi_handle) {
        wifi_manager_deinit(g_wifi_handle);
    }
    button_controller_deinit(g_button_handle);
    led_controller_deinit();
    if (g_system_monitor_timer) {
        xTimerDelete(g_system_monitor_timer, 0);
    }
    ESP_LOGI(TAG, "Application finished");
}
