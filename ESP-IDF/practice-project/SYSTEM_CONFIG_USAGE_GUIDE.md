# System Config Component Usage Guide

## Overview

The **system_config** component provides centralized configuration management for the ESP-IDF project, integrating with the menuconfig system to offer runtime access to all configurable parameters including GPIO assignments, timing parameters, and network settings.

## Key Features

✅ **Centralized Configuration Management** - Single source of truth for all project settings  
✅ **Menuconfig Integration** - Uses ESP-IDF's built-in configuration system  
✅ **Runtime Configuration Access** - Get configuration values during application runtime  
✅ **Validation and Conflict Detection** - Prevents GPIO conflicts and invalid settings  
✅ **Thread-Safe Operations** - Safe concurrent access from multiple tasks  
✅ **Configuration Change Callbacks** - Notify components when settings change  

## How to Use System Config

### Step 1: Add Dependency to Your Component

Update your component's `CMakeLists.txt` to include the system_config dependency:

```cmake
idf_component_register(SRCS "your_component.c"
                    INCLUDE_DIRS "include"
                    REQUIRES system_config)
```

### Step 2: Include the Header in Your Code

```c
#include "system_config.h"
```

### Step 3: Initialize the System Configuration

Call this once during application startup:

```c
void app_main(void) {
    // Initialize system configuration first
    esp_err_t ret = system_config_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize system config: %s", esp_err_to_name(ret));
        return;
    }
    
    // Now you can use configuration values
    const system_config_t *config = system_config_get();
    if (config != NULL) {
        ESP_LOGI(TAG, "LED Pin: %d", config->gpio.led_pin);
        ESP_LOGI(TAG, "Button Pin: %d", config->gpio.button_pin);
    }
}
```

### Step 4: Access Configuration Values

```c
/**
 * @brief Example function showing how to access configuration values
 */
void use_configuration_example(void) {
    // Get the current configuration
    const system_config_t *config = system_config_get();
    if (config == NULL) {
        ESP_LOGE(TAG, "Configuration not available");
        return;
    }
    
    // Access GPIO configuration
    gpio_num_t led_pin = config->gpio.led_pin;
    bool led_active_level = config->gpio.led_active_level;
    gpio_num_t button_pin = config->gpio.button_pin;
    gpio_num_t dht22_pin = config->gpio.dht22_pin;
    
    // Access timing configuration
    uint32_t debounce_time = config->timing.button_debounce_ms;
    uint32_t sensor_interval = config->timing.sensor_read_interval_ms;
    
    // Access network configuration
    const char* wifi_ssid = config->network.wifi_ssid;
    const char* mqtt_broker = config->network.mqtt_broker_uri;
    
    // Use the values to initialize your components
    ESP_LOGI(TAG, "Configuring LED on GPIO %d (active %s)", 
             led_pin, led_active_level ? "HIGH" : "LOW");
}
```

### Step 5: Register Configuration Change Callbacks (Optional)

```c
/**
 * @brief Configuration change callback function
 */
void my_config_change_callback(const system_config_t *config, void *user_data) {
    ESP_LOGI(TAG, "Configuration changed!");
    
    // Respond to configuration changes
    // For example, reconfigure GPIO pins, update timing, etc.
    
    // Access the new configuration values
    ESP_LOGI(TAG, "New LED pin: %d", config->gpio.led_pin);
    ESP_LOGI(TAG, "New sensor interval: %lu ms", config->timing.sensor_read_interval_ms);
}

/**
 * @brief Register for configuration change notifications
 */
void register_for_config_changes(void) {
    esp_err_t ret = system_config_register_callback(my_config_change_callback, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register config callback: %s", esp_err_to_name(ret));
    }
}
```

## Complete Integration Example

Here's how to integrate system_config into your main application:

```c
#include <stdio.h>
#include "esp_log.h"
#include "system_config.h"
#include "led_controller.h"
#include "button_controller.h"
#include "dht22_sensor.h"

static const char *TAG = "MAIN_APP";

/**
 * @brief Configuration change callback
 */
void configuration_change_callback(const system_config_t *config, void *user_data) {
    ESP_LOGI(TAG, "Configuration change detected");
    ESP_LOGI(TAG, "New LED pin: %d, Button pin: %d", 
             config->gpio.led_pin, config->gpio.button_pin);
    ESP_LOGI(TAG, "New timing - Debounce: %lu ms, Long press: %lu ms",
             config->timing.button_debounce_ms, config->timing.button_long_press_ms);
}

void app_main(void) {
    ESP_LOGI(TAG, "=== Smart Environment Data Logger Started ===");
    ESP_LOGI(TAG, "Day 8-10: DHT22 + System Config Integration");
    
    // Step 1: Initialize system configuration
    esp_err_t ret = system_config_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize system config: %s", esp_err_to_name(ret));
        return;
    }
    
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
    
    // Initialize components with configuration-based settings
    ret = led_controller_init(&led_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LED controller: %s", esp_err_to_name(ret));
        return;
    }
    
    button_controller_handle_t button_handle;
    ret = button_controller_init(&button_config, NULL, NULL, &button_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize button controller: %s", esp_err_to_name(ret));
        return;
    }
    
    dht22_handle_t dht22_handle;
    ret = dht22_init(&dht22_config, &dht22_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize DHT22 sensor: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "All components initialized using system configuration");
    ESP_LOGI(TAG, "LED: GPIO%d, Button: GPIO%d, DHT22: GPIO%d", 
             config->gpio.led_pin, config->gpio.button_pin, config->gpio.dht22_pin);
    
    // Main application loop
    while (1) {
        // Your application logic here
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

## Configuration Management

### Customizing Settings via menuconfig

1. **Run menuconfig**:
   ```bash
   idf.py menuconfig
   ```

2. **Navigate to**: `Smart Environment Data Logger Configuration`

3. **Configure sections**:
   - **GPIO Configuration**: Set pin assignments for LED, button, sensors
   - **Sensor Configuration**: Configure sensor-specific pins
   - **Storage Configuration**: Set SD card SPI pins  
   - **Network Configuration**: Set WiFi credentials and MQTT settings
   - **Application Settings**: Configure timing intervals

4. **Save and build**:
   ```bash
   idf.py build
   ```

### Available Configuration Sections

#### GPIO Configuration
- LED GPIO Pin (default: GPIO2)
- LED Active Level (default: Active High)
- Button GPIO Pin (default: GPIO0)  
- Button Active Level (default: Active Low)

#### Sensor Configuration  
- DHT22 Data GPIO Pin (default: GPIO4)
- MPU6050 I2C SDA Pin (default: GPIO21)
- MPU6050 I2C SCL Pin (default: GPIO22)

#### Storage Configuration
- SD Card SPI CS Pin (default: GPIO5)
- SD Card SPI MOSI Pin (default: GPIO23)
- SD Card SPI MISO Pin (default: GPIO19)
- SD Card SPI CLK Pin (default: GPIO18)

#### Network Configuration
- WiFi SSID (default: "your_wifi_ssid")
- WiFi Password (default: "your_wifi_password")
- MQTT Broker URI (default: "mqtt://mqtt.broker.address:1883")
- MQTT Topic Prefix (default: "smart_logger")

#### Application Settings
- Sensor Reading Interval (default: 5000ms)
- Data Logging Interval (default: 10000ms)
- MQTT Publish Interval (default: 30000ms)

## API Reference

### Core Functions

```c
// Initialize the system configuration
esp_err_t system_config_init(void);

// Get current configuration (read-only)
const system_config_t* system_config_get(void);

// Register for configuration change notifications
esp_err_t system_config_register_callback(config_change_callback_t callback, void *user_data);

// Unregister configuration change callback
esp_err_t system_config_unregister_callback(config_change_callback_t callback);

// Validate current configuration
config_validation_result_t system_config_validate_all(void);

// Print current configuration to console
void system_config_print(bool detailed);
```

### Configuration Structures

```c
typedef struct {
    system_gpio_config_t gpio;        // GPIO pin assignments
    system_timing_config_t timing;    // Timing parameters
    system_network_config_t network;  // Network settings
    uint32_t config_version;          // Configuration version
    bool initialized;                 // Initialization status
} system_config_t;
```

## Benefits of Using System Config

1. **Centralized Management**: All configuration in one place
2. **User Customization**: Easy configuration via menuconfig  
3. **Conflict Prevention**: Automatic GPIO conflict detection
4. **Runtime Flexibility**: Change behavior without recompiling
5. **Professional Structure**: Industry-standard configuration patterns
6. **Maintainability**: Easier to manage complex projects

## Build and Integration

To use the system_config component:

1. **Ensure proper dependencies** in your component's CMakeLists.txt
2. **Initialize early** in your application startup
3. **Use configuration values** instead of hardcoded constants
4. **Register callbacks** if you need to respond to changes
5. **Build with menuconfig** to customize settings

The system_config component bridges the gap between compile-time configuration (Kconfig) and runtime configuration access, providing a professional foundation for scalable ESP-IDF applications.