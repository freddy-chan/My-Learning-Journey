/**
 * @file button_controller.c
 * @brief Button Controller Component Implementation
 * 
 * This file implements comprehensive button handling with hardware interrupts,
 * software debouncing, and event-driven architecture. It demonstrates advanced
 * ESP-IDF concepts including interrupt service routines, FreeRTOS queues,
 * timers, and proper resource management.
 * 
 * @author Learning ESP-IDF
 * @date 2025
 */

#include "button_controller.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdlib.h>

// Component tag for logging
static const char *TAG = "BTN_CTRL";

// Maximum number of button instances supported
#define MAX_BUTTON_INSTANCES 8

// Internal event queue size 
#define BUTTON_EVENT_QUEUE_SIZE 10

/**
 * @brief Internal button state structure
 * 
 * Contains all internal state information for a button instance
 */
struct button_controller_handle_s {
    // Configuration
    button_config_t config;                    // Button configuration
    button_event_callback_t callback;          // Event callback function
    void *user_data;                          // User data for callback
    
    // State management
    button_state_t current_state;             // Current debounced button state
    button_state_t last_raw_state;            // Last raw GPIO reading
    bool events_enabled;                      // Event detection enabled flag
    
    // Timing
    uint64_t last_press_time;                 // Timestamp of last press (microseconds)
    uint64_t last_release_time;               // Timestamp of last release (microseconds)
    uint64_t last_debounce_time;              // Timestamp of last state change (microseconds)
    uint64_t last_click_time;                 // Timestamp of last click for double-click detection
    
    // Statistics
    uint32_t total_presses;                   // Total number of presses detected
    uint32_t total_releases;                  // Total number of releases detected
    uint32_t total_long_presses;              // Total number of long presses detected
    uint32_t total_double_clicks;             // Total number of double clicks detected
    
    // Timers
    esp_timer_handle_t debounce_timer;        // Debounce timer handle
    esp_timer_handle_t long_press_timer;      // Long press timer handle
    
    // Synchronization
    SemaphoreHandle_t mutex;                  // Mutex for thread-safe access
    
    // Validation
    uint32_t magic;                          // Magic number for handle validation
};

// Magic number for handle validation
#define BUTTON_HANDLE_MAGIC 0xB17700CC

// Global ISR handler tracking
static button_controller_handle_t g_button_instances[MAX_BUTTON_INSTANCES] = {0};
static SemaphoreHandle_t g_instances_mutex = NULL;
static bool g_isr_service_installed = false;

/**
 * @brief Validate button handle
 * 
 * Checks if the provided handle is valid and points to an initialized button.
 * 
 * @param handle Button handle to validate
 * @return true if handle is valid, false otherwise
 */
static bool button_handle_is_valid(button_controller_handle_t handle) {
    return (handle != NULL && handle->magic == BUTTON_HANDLE_MAGIC);
}

/**
 * @brief Get current time in microseconds
 * 
 * @return Current time in microseconds since boot
 */
static uint64_t button_get_time_us(void) {
    return esp_timer_get_time();
}

/**
 * @brief Convert microseconds to milliseconds
 * 
 * @param time_us Time in microseconds
 * @return Time in milliseconds
 */
static uint32_t button_us_to_ms(uint64_t time_us) {
    return (uint32_t)(time_us / 1000);
}

/**
 * @brief Read raw button state from GPIO
 * 
 * Reads the current GPIO level and converts it to button state based on
 * the active level configuration.
 * 
 * @param handle Button handle
 * @return Raw button state (pressed/released)
 */
static button_state_t button_read_raw_state(button_controller_handle_t handle) {
    // Read current GPIO level
    int gpio_level = gpio_get_level(handle->config.gpio_pin);
    
    // Convert to button state based on active level
    bool is_pressed = (gpio_level == handle->config.active_level);
    
    return is_pressed ? BUTTON_STATE_PRESSED : BUTTON_STATE_RELEASED;
}

/**
 * @brief Process button event
 * 
 * Handles button event processing including statistics update and callback invocation.
 * This function can be called from interrupt context.
 * 
 * @param handle Button handle
 * @param event Button event to process
 */
static void button_process_event(button_controller_handle_t handle, button_event_t event) {
    // Update statistics
    switch (event) {
        case BUTTON_EVENT_PRESSED:
            handle->total_presses++;
            handle->last_press_time = button_get_time_us();
            ESP_LOGD(TAG, "Button pressed (total: %lu)", handle->total_presses);
            break;
            
        case BUTTON_EVENT_RELEASED:
            handle->total_releases++;
            handle->last_release_time = button_get_time_us();
            ESP_LOGD(TAG, "Button released (total: %lu)", handle->total_releases);
            break;
            
        case BUTTON_EVENT_LONG_PRESS:
            handle->total_long_presses++;
            ESP_LOGD(TAG, "Long press detected (total: %lu)", handle->total_long_presses);
            break;
            
        case BUTTON_EVENT_DOUBLE_CLICK:
            handle->total_double_clicks++;
            ESP_LOGD(TAG, "Double click detected (total: %lu)", handle->total_double_clicks);
            break;
            
        case BUTTON_EVENT_CLICK:
            ESP_LOGD(TAG, "Single click detected");
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown button event: %d", event);
            return;
    }
    
    // Call user callback if events are enabled
    if (handle->events_enabled && handle->callback != NULL) {
        handle->callback(event, handle->user_data);
    }
}

/**
 * @brief Debounce timer callback
 * 
 * Called by ESP timer when debounce period expires. Processes the actual
 * state change and generates appropriate events.
 * 
 * @param arg Button handle (passed as void pointer)
 */
static void button_debounce_timer_callback(void *arg) {
    button_controller_handle_t handle = (button_controller_handle_t)arg;
    
    // Validate handle
    if (!button_handle_is_valid(handle)) {
        ESP_LOGE(TAG, "Invalid handle in debounce callback");
        return;
    }
    
    // Take mutex for thread safety
    if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to take mutex in debounce callback");
        return;
    }
    
    // Read current raw state
    button_state_t raw_state = button_read_raw_state(handle);
    
    // Check if state is stable (same as when timer was started)
    if (raw_state == handle->last_raw_state) {
        // State is stable, update current state
        button_state_t old_state = handle->current_state;
        handle->current_state = raw_state;
        
        // Generate events based on state change
        if (old_state != handle->current_state) {
            if (handle->current_state == BUTTON_STATE_PRESSED) {
                button_process_event(handle, BUTTON_EVENT_PRESSED);
            } else {
                button_process_event(handle, BUTTON_EVENT_RELEASED);
                
                // Check for click and double-click
                uint64_t current_time = button_get_time_us();
                uint32_t press_duration = button_us_to_ms(current_time - handle->last_press_time);
                
                // Generate click event if press was short enough
                if (press_duration < handle->config.long_press_time_ms) {
                    button_process_event(handle, BUTTON_EVENT_CLICK);
                    
                    // Check for double-click
                    uint32_t time_since_last_click = button_us_to_ms(current_time - handle->last_click_time);
                    if (time_since_last_click < handle->config.double_click_timeout_ms) {
                        button_process_event(handle, BUTTON_EVENT_DOUBLE_CLICK);
                    }
                    
                    handle->last_click_time = current_time;
                }
            }
        }
    } else {
        // State changed during debounce, restart debouncing
        ESP_LOGD(TAG, "State changed during debounce, restarting");
        handle->last_raw_state = raw_state;
        handle->last_debounce_time = button_get_time_us();
    }
    
    // Release mutex
    xSemaphoreGive(handle->mutex);
}

/**
 * @brief Long press timer callback
 * 
 * Called when button has been held for long press duration.
 * 
 * @param arg Button handle (passed as void pointer)
 */
static void button_long_press_timer_callback(void *arg) {
    button_controller_handle_t handle = (button_controller_handle_t)arg;
    
    // Validate handle
    if (!button_handle_is_valid(handle)) {
        ESP_LOGE(TAG, "Invalid handle in long press callback");
        return;
    }
    
    // Take mutex for thread safety
    if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to take mutex in long press callback");
        return;
    }
    
    // Check if button is still pressed
    if (handle->current_state == BUTTON_STATE_PRESSED) {
        button_process_event(handle, BUTTON_EVENT_LONG_PRESS);
    }
    
    // Release mutex
    xSemaphoreGive(handle->mutex);
}

/**
 * @brief GPIO interrupt service routine
 * 
 * Called when GPIO interrupt occurs. Records the event and schedules
 * debounce processing. This function runs in interrupt context and must
 * be fast and not perform blocking operations.
 * 
 * @param arg Button handle (passed as void pointer)
 */
static void IRAM_ATTR button_isr_handler(void *arg) {
    button_controller_handle_t handle = (button_controller_handle_t)arg;
    
    // Quick validation (minimal in ISR)
    if (handle == NULL || handle->magic != BUTTON_HANDLE_MAGIC) {
        return;
    }
    
    // Read current raw state
    button_state_t raw_state = button_read_raw_state(handle);
    
    // Update last raw state and debounce time
    handle->last_raw_state = raw_state;
    handle->last_debounce_time = button_get_time_us();
    
    // Stop existing timers to prevent conflicts
    if (handle->debounce_timer) {
        esp_timer_stop(handle->debounce_timer);
    }
    if (handle->long_press_timer) {
        esp_timer_stop(handle->long_press_timer);
    }
    
    // Start debounce timer
    if (handle->debounce_timer) {
        esp_timer_start_once(handle->debounce_timer, handle->config.debounce_time_ms * 1000);
    }
    
    // Start long press timer if button was pressed
    if (raw_state == BUTTON_STATE_PRESSED && handle->long_press_timer) {
        esp_timer_start_once(handle->long_press_timer, handle->config.long_press_time_ms * 1000);
    }
}

/**
 * @brief Find available button instance slot
 * 
 * Finds an empty slot in the global button instances array.
 * 
 * @return Index of available slot, or -1 if no slots available
 */
static int button_find_available_slot(void) {
    for (int i = 0; i < MAX_BUTTON_INSTANCES; i++) {
        if (g_button_instances[i] == NULL) {
            return i;
        }
    }
    return -1;
}

/**
 * @brief Register button instance globally
 * 
 * Adds button handle to global tracking for ISR management.
 * 
 * @param handle Button handle to register
 * @return ESP_OK on success, ESP_ERR_NO_MEM if no slots available
 */
static esp_err_t button_register_instance(button_controller_handle_t handle) {
    // Create global mutex if not exists
    if (g_instances_mutex == NULL) {
        g_instances_mutex = xSemaphoreCreateMutex();
        if (g_instances_mutex == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }
    
    // Take global mutex
    if (xSemaphoreTake(g_instances_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    // Find available slot
    int slot = button_find_available_slot();
    if (slot < 0) {
        xSemaphoreGive(g_instances_mutex);
        return ESP_ERR_NO_MEM;
    }
    
    // Register instance
    g_button_instances[slot] = handle;
    
    xSemaphoreGive(g_instances_mutex);
    return ESP_OK;
}

/**
 * @brief Unregister button instance
 * 
 * Removes button handle from global tracking.
 * 
 * @param handle Button handle to unregister
 */
static void button_unregister_instance(button_controller_handle_t handle) {
    if (g_instances_mutex == NULL) {
        return;
    }
    
    // Take global mutex
    if (xSemaphoreTake(g_instances_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }
    
    // Find and remove instance
    for (int i = 0; i < MAX_BUTTON_INSTANCES; i++) {
        if (g_button_instances[i] == handle) {
            g_button_instances[i] = NULL;
            break;
        }
    }
    
    xSemaphoreGive(g_instances_mutex);
}

// Public API Implementation

esp_err_t button_controller_init(const button_config_t *config,
                                button_event_callback_t callback,
                                void *user_data,
                                button_controller_handle_t *handle) {
    // Input validation
    if (config == NULL || handle == NULL) {
        ESP_LOGE(TAG, "Invalid parameters: config or handle is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Validate GPIO pin
    if (!GPIO_IS_VALID_GPIO(config->gpio_pin)) {
        ESP_LOGE(TAG, "Invalid GPIO pin: %d", config->gpio_pin);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Validate timing parameters
    if (config->debounce_time_ms == 0 || config->debounce_time_ms > 1000) {
        ESP_LOGE(TAG, "Invalid debounce time: %lu ms (valid range: 1-1000)", config->debounce_time_ms);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Allocate handle
    button_controller_handle_t btn_handle = calloc(1, sizeof(struct button_controller_handle_s));
    if (btn_handle == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for button handle");
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize handle
    btn_handle->config = *config;
    btn_handle->callback = callback;
    btn_handle->user_data = user_data;
    btn_handle->current_state = BUTTON_STATE_RELEASED;
    btn_handle->last_raw_state = BUTTON_STATE_RELEASED;
    btn_handle->events_enabled = true;
    btn_handle->magic = BUTTON_HANDLE_MAGIC;
    
    // Create mutex for thread safety
    btn_handle->mutex = xSemaphoreCreateMutex();
    if (btn_handle->mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        free(btn_handle);
        return ESP_ERR_NO_MEM;
    }
    
    // Create debounce timer
    esp_timer_create_args_t debounce_timer_args = {
        .callback = button_debounce_timer_callback,
        .arg = btn_handle,
        .name = "btn_debounce"
    };
    
    esp_err_t ret = esp_timer_create(&debounce_timer_args, &btn_handle->debounce_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create debounce timer: %s", esp_err_to_name(ret));
        vSemaphoreDelete(btn_handle->mutex);
        free(btn_handle);
        return ret;
    }
    
    // Create long press timer
    esp_timer_create_args_t long_press_timer_args = {
        .callback = button_long_press_timer_callback,
        .arg = btn_handle,
        .name = "btn_longpress"
    };
    
    ret = esp_timer_create(&long_press_timer_args, &btn_handle->long_press_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create long press timer: %s", esp_err_to_name(ret));
        esp_timer_delete(btn_handle->debounce_timer);
        vSemaphoreDelete(btn_handle->mutex);
        free(btn_handle);
        return ret;
    }
    
    // Configure GPIO
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,           // Interrupt on both edges
        .mode = GPIO_MODE_INPUT,                  // Set as input mode
        .pin_bit_mask = (1ULL << config->gpio_pin), // Bit mask for the pin
        .pull_down_en = config->pull_down_enable ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
        .pull_up_en = config->pull_up_enable ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
    };
    
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO %d: %s", config->gpio_pin, esp_err_to_name(ret));
        esp_timer_delete(btn_handle->long_press_timer);
        esp_timer_delete(btn_handle->debounce_timer);
        vSemaphoreDelete(btn_handle->mutex);
        free(btn_handle);
        return ret;
    }
    
    // Install GPIO ISR service if not already installed
    if (!g_isr_service_installed) {
        ret = gpio_install_isr_service(ESP_INTR_FLAG_EDGE);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "Failed to install ISR service: %s", esp_err_to_name(ret));
            esp_timer_delete(btn_handle->long_press_timer);
            esp_timer_delete(btn_handle->debounce_timer);
            vSemaphoreDelete(btn_handle->mutex);
            free(btn_handle);
            return ret;
        }
        g_isr_service_installed = true;
    }
    
    // Add ISR handler for this pin
    ret = gpio_isr_handler_add(config->gpio_pin, button_isr_handler, btn_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ISR handler for GPIO %d: %s", config->gpio_pin, esp_err_to_name(ret));
        esp_timer_delete(btn_handle->long_press_timer);
        esp_timer_delete(btn_handle->debounce_timer);
        vSemaphoreDelete(btn_handle->mutex);
        free(btn_handle);
        return ret;
    }
    
    // Register instance globally
    ret = button_register_instance(btn_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register button instance: %s", esp_err_to_name(ret));
        gpio_isr_handler_remove(config->gpio_pin);
        esp_timer_delete(btn_handle->long_press_timer);
        esp_timer_delete(btn_handle->debounce_timer);
        vSemaphoreDelete(btn_handle->mutex);
        free(btn_handle);
        return ret;
    }
    
    // Read initial state
    btn_handle->current_state = button_read_raw_state(btn_handle);
    btn_handle->last_raw_state = btn_handle->current_state;
    
    *handle = btn_handle;
    
    ESP_LOGI(TAG, "Button controller initialized on GPIO %d (active %s, debounce %lu ms)", 
             config->gpio_pin, 
             config->active_level ? "HIGH" : "LOW",
             config->debounce_time_ms);
    
    return ESP_OK;
}

button_state_t button_controller_get_state(button_controller_handle_t handle) {
    if (!button_handle_is_valid(handle)) {
        ESP_LOGE(TAG, "Invalid button handle");
        return BUTTON_STATE_RELEASED;
    }
    
    return handle->current_state;
}

bool button_controller_is_pressed(button_controller_handle_t handle) {
    return (button_controller_get_state(handle) == BUTTON_STATE_PRESSED);
}

uint32_t button_controller_get_press_duration(button_controller_handle_t handle) {
    if (!button_handle_is_valid(handle)) {
        ESP_LOGE(TAG, "Invalid button handle");
        return 0;
    }
    
    if (handle->current_state != BUTTON_STATE_PRESSED) {
        return 0;
    }
    
    uint64_t current_time = button_get_time_us();
    return button_us_to_ms(current_time - handle->last_press_time);
}

esp_err_t button_controller_set_callback(button_controller_handle_t handle,
                                        button_event_callback_t callback,
                                        void *user_data) {
    if (!button_handle_is_valid(handle)) {
        ESP_LOGE(TAG, "Invalid button handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Take mutex for thread safety
    if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    handle->callback = callback;
    handle->user_data = user_data;
    
    xSemaphoreGive(handle->mutex);
    
    ESP_LOGI(TAG, "Button callback %s", callback ? "updated" : "disabled");
    return ESP_OK;
}

esp_err_t button_controller_enable_events(button_controller_handle_t handle, bool enable) {
    if (!button_handle_is_valid(handle)) {
        ESP_LOGE(TAG, "Invalid button handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    handle->events_enabled = enable;
    ESP_LOGI(TAG, "Button events %s", enable ? "enabled" : "disabled");
    
    return ESP_OK;
}

esp_err_t button_controller_reset(button_controller_handle_t handle) {
    if (!button_handle_is_valid(handle)) {
        ESP_LOGE(TAG, "Invalid button handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Take mutex for thread safety
    if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    // Reset state and statistics
    handle->current_state = button_read_raw_state(handle);
    handle->last_raw_state = handle->current_state;
    handle->total_presses = 0;
    handle->total_releases = 0;
    handle->total_long_presses = 0;
    handle->total_double_clicks = 0;
    handle->last_press_time = 0;
    handle->last_release_time = 0;
    handle->last_click_time = 0;
    
    xSemaphoreGive(handle->mutex);
    
    ESP_LOGI(TAG, "Button controller reset");
    return ESP_OK;
}

esp_err_t button_controller_get_stats(button_controller_handle_t handle,
                                     uint32_t *total_presses,
                                     uint32_t *total_releases,
                                     uint32_t *total_long_presses,
                                     uint32_t *total_double_clicks) {
    if (!button_handle_is_valid(handle)) {
        ESP_LOGE(TAG, "Invalid button handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Copy statistics (reading is atomic for 32-bit values)
    if (total_presses) *total_presses = handle->total_presses;
    if (total_releases) *total_releases = handle->total_releases;
    if (total_long_presses) *total_long_presses = handle->total_long_presses;
    if (total_double_clicks) *total_double_clicks = handle->total_double_clicks;
    
    return ESP_OK;
}

esp_err_t button_controller_deinit(button_controller_handle_t handle) {
    if (!button_handle_is_valid(handle)) {
        ESP_LOGE(TAG, "Invalid button handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Remove ISR handler
    gpio_isr_handler_remove(handle->config.gpio_pin);
    
    // Unregister from global tracking
    button_unregister_instance(handle);
    
    // Stop and delete timers
    if (handle->debounce_timer) {
        esp_timer_stop(handle->debounce_timer);
        esp_timer_delete(handle->debounce_timer);
    }
    if (handle->long_press_timer) {
        esp_timer_stop(handle->long_press_timer);
        esp_timer_delete(handle->long_press_timer);
    }
    
    // Reset GPIO pin to default state
    gpio_reset_pin(handle->config.gpio_pin);
    
    // Clean up mutex
    if (handle->mutex) {
        vSemaphoreDelete(handle->mutex);
    }
    
    // Invalidate handle
    handle->magic = 0;
    
    // Free memory
    free(handle);
    
    ESP_LOGI(TAG, "Button controller deinitialized");
    return ESP_OK;
}

const char* button_controller_event_to_string(button_event_t event) {
    switch (event) {
        case BUTTON_EVENT_PRESSED:     return "PRESSED";
        case BUTTON_EVENT_RELEASED:    return "RELEASED";
        case BUTTON_EVENT_LONG_PRESS:  return "LONG_PRESS";
        case BUTTON_EVENT_CLICK:       return "CLICK";
        case BUTTON_EVENT_DOUBLE_CLICK: return "DOUBLE_CLICK";
        case BUTTON_EVENT_INVALID:     return "INVALID";
        default:                       return "UNKNOWN";
    }
}