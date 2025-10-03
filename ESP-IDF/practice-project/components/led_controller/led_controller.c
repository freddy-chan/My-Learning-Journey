/**
 * @file led_controller.c
 * @brief LED Controller Component Implementation
 *
 * This file implements the LED controller functionality defined in
 * led_controller.h. It provides a complete example of ESP-IDF component
 * structure and GPIO usage.
 *
 * @author Learning ESP-IDF
 * @date 2025
 */

#include "led_controller.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// Component tag for logging
static const char *TAG = "LED_CTRL";

// Static variables to maintain LED state
static led_config_t s_led_config = {0};
static led_state_t s_current_state = LED_STATE_OFF;
static bool s_initialized = false;
static SemaphoreHandle_t s_mutex = NULL;  // Mutex for thread safety

/**
 * @brief Initialize LED controller
 */
esp_err_t led_controller_init(const led_config_t *config) {
  // Input parameter validation
  if (config == NULL) {
    ESP_LOGE(TAG, "Configuration pointer is NULL");
    return ESP_ERR_INVALID_ARG;
  }

  // Validate GPIO pin number
  if (!GPIO_IS_VALID_OUTPUT_GPIO(config->gpio_pin)) {
    ESP_LOGE(TAG, "Invalid GPIO pin: %d", config->gpio_pin);
    return ESP_ERR_INVALID_ARG;
  }

  // Copy configuration
  s_led_config = *config;

  // Create mutex for thread safety
  if (s_mutex == NULL) {
    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) {
      ESP_LOGE(TAG, "Failed to create mutex");
      return ESP_ERR_NO_MEM;
    }
  }

  // Configure GPIO
  gpio_config_t io_conf = {
      .intr_type = GPIO_INTR_DISABLE,             // Disable interrupts
      .mode = GPIO_MODE_OUTPUT,                   // Set as output mode
      .pin_bit_mask = (1ULL << config->gpio_pin), // Bit mask for the pin
      .pull_down_en = GPIO_PULLDOWN_DISABLE,      // Disable pull-down
      .pull_up_en = GPIO_PULLUP_DISABLE,          // Disable pull-up
  };

  // Apply configuration
  esp_err_t ret = gpio_config(&io_conf);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure GPIO %d: %s", config->gpio_pin,
             esp_err_to_name(ret));
    return ret;
  }

  // Mark as initialized BEFORE calling set_state
  s_initialized = true;

  // Initialize LED to OFF state
  ret = led_controller_set_state(LED_STATE_OFF);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize LED state: %s", esp_err_to_name(ret));
    // Reset initialization flag on failure
    s_initialized = false;
    return ret;
  }

  ESP_LOGI(TAG, "LED controller initialized on GPIO %d (active %s)",
           config->gpio_pin, config->active_level ? "HIGH" : "LOW");

  return ESP_OK;
}



/**
 * @brief Set LED state with thread safety
 */
esp_err_t led_controller_set_state(led_state_t state) {
  // Check if component is initialized
  if (!s_initialized) {
    ESP_LOGE(TAG, "LED controller not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  // Take mutex for thread safety
  if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    ESP_LOGE(TAG, "Failed to take mutex");
    return ESP_ERR_TIMEOUT;
  }

  uint32_t gpio_level;
  esp_err_t ret = ESP_OK;

  switch (state) {
  case LED_STATE_ON:
    // Set GPIO level based on active level configuration
    gpio_level = s_led_config.active_level ? 1 : 0;
    s_current_state = LED_STATE_ON;
    ESP_LOGD(TAG, "LED turned ON");
    break;

  case LED_STATE_OFF:
    // Set GPIO level opposite to active level
    gpio_level = s_led_config.active_level ? 0 : 1;
    s_current_state = LED_STATE_OFF;
    ESP_LOGD(TAG, "LED turned OFF");
    break;

  case LED_STATE_TOGGLE:
    // Toggle current state
    if (s_current_state == LED_STATE_ON) {
      // Release mutex before recursive call
      xSemaphoreGive(s_mutex);
      return led_controller_set_state(LED_STATE_OFF);
    } else {
      // Release mutex before recursive call
      xSemaphoreGive(s_mutex);
      return led_controller_set_state(LED_STATE_ON);
    }

  default:
    ESP_LOGE(TAG, "Invalid LED state: %d", state);
    ret = ESP_ERR_INVALID_ARG;
    goto exit;
  }

  // Apply the GPIO level
  ret = gpio_set_level(s_led_config.gpio_pin, gpio_level);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set GPIO level: %s", esp_err_to_name(ret));
  }

exit:
  // Release mutex
  xSemaphoreGive(s_mutex);
  return ret;
}

/**
 * @brief Get current LED state (thread-safe)
 */
led_state_t led_controller_get_state(void) {
  if (!s_initialized || s_mutex == NULL) {
    return LED_STATE_OFF;  // Safe default
  }

  // Take mutex for thread safety
  if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
    ESP_LOGW(TAG, "Failed to take mutex for get_state");
    return s_current_state;  // Return current value without protection
  }

  led_state_t current_state = s_current_state;
  
  // Release mutex
  xSemaphoreGive(s_mutex);
  
  return current_state;
}

/**
 * @brief Blink LED for specified duration
 */
esp_err_t led_controller_blink(uint32_t on_time_ms, uint32_t off_time_ms,
                               uint32_t duration_ms) {
  // Input validation
  if (on_time_ms == 0 || off_time_ms == 0 || duration_ms == 0) {
    ESP_LOGE(TAG, "Invalid blink parameters");
    return ESP_ERR_INVALID_ARG;
  }

  // Check if component is initialized
  if (!s_initialized) {
    ESP_LOGE(TAG, "LED controller not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  ESP_LOGI(TAG, "Starting LED blink: ON=%ldms, OFF=%ldms, Duration=%ldms",
           on_time_ms, off_time_ms, duration_ms);

  // Track elapsed time for precise duration control
  uint32_t elapsed_time = 0;

  // Blink loop
  while (elapsed_time < duration_ms) {
    // Turn LED on
    esp_err_t ret = led_controller_set_state(LED_STATE_ON);
    if (ret != ESP_OK) {
      return ret;
    }

    // Wait for ON duration (or remaining time if less)
    uint32_t on_delay = (elapsed_time + on_time_ms <= duration_ms)
                            ? on_time_ms
                            : (duration_ms - elapsed_time);
    vTaskDelay(pdMS_TO_TICKS(on_delay));
    elapsed_time += on_delay;

    if (elapsed_time >= duration_ms) {
      break;
    }

    // Turn LED off
    ret = led_controller_set_state(LED_STATE_OFF);
    if (ret != ESP_OK) {
      return ret;
    }

    // Wait for OFF duration (or remaining time if less)
    uint32_t off_delay = (elapsed_time + off_time_ms <= duration_ms)
                             ? off_time_ms
                             : (duration_ms - elapsed_time);
    vTaskDelay(pdMS_TO_TICKS(off_delay));
    elapsed_time += off_delay;
  }

  // Ensure LED is off at the end
  esp_err_t ret = led_controller_set_state(LED_STATE_OFF);
  ESP_LOGI(TAG, "LED blink completed");

  return ret;
}

/**
 * @brief Deinitialize LED controller with proper cleanup
 */
esp_err_t led_controller_deinit(void) {
  if (!s_initialized) {
    ESP_LOGW(TAG, "LED controller already deinitialized");
    return ESP_OK;
  }

  // Turn off LED before deinitializing
  esp_err_t ret = led_controller_set_state(LED_STATE_OFF);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to turn off LED during deinit: %s",
             esp_err_to_name(ret));
  }

  // Reset GPIO pin (optional - set to input mode)
  gpio_config_t io_conf = {
      .intr_type = GPIO_INTR_DISABLE,
      .mode = GPIO_MODE_INPUT,
      .pin_bit_mask = (1ULL << s_led_config.gpio_pin),
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .pull_up_en = GPIO_PULLUP_DISABLE,
  };

  ret = gpio_config(&io_conf);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to reset GPIO during deinit: %s",
             esp_err_to_name(ret));
  }

  // Clean up mutex
  if (s_mutex != NULL) {
    vSemaphoreDelete(s_mutex);
    s_mutex = NULL;
  }

  // Reset internal state
  s_initialized = false;
  s_current_state = LED_STATE_OFF;

  ESP_LOGI(TAG, "LED controller deinitialized");
  return ESP_OK;
}