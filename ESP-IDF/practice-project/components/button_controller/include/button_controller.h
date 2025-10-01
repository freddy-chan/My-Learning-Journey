/**
 * @file button_controller.h
 * @brief Button Controller Component Header
 * 
 * This component provides comprehensive button handling with interrupt support,
 * debouncing, and event-driven architecture. It demonstrates proper ESP-IDF
 * interrupt handling and FreeRTOS integration.
 * 
 * Features:
 * - Hardware interrupt-based button detection
 * - Software debouncing with configurable timing
 * - Event callback system for button actions
 * - Support for multiple button types (pull-up/pull-down)
 * - Press, release, and long-press detection
 * 
 * @author ChanPyae
 * @date 2025
 */

#ifndef BUTTON_CONTROLLER_H
#define BUTTON_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Button event types enumeration
 * 
 * Defines all possible button events that can be detected
 */
typedef enum {
    BUTTON_EVENT_PRESSED = 0,     /**< Button was just pressed */
    BUTTON_EVENT_RELEASED,        /**< Button was just released */
    BUTTON_EVENT_LONG_PRESS,      /**< Button held for long press duration */
    BUTTON_EVENT_CLICK,           /**< Single click (press + release) */
    BUTTON_EVENT_DOUBLE_CLICK,    /**< Double click within timeout */
    BUTTON_EVENT_INVALID          /**< Invalid event (for error handling) */
} button_event_t;

/**
 * @brief Button state enumeration
 * 
 * Current physical state of the button
 */
typedef enum {
    BUTTON_STATE_RELEASED = 0,    /**< Button is not pressed */
    BUTTON_STATE_PRESSED = 1,     /**< Button is currently pressed */
    BUTTON_STATE_DEBOUNCING       /**< Button state is being debounced */
} button_state_t;

/**
 * @brief Button configuration structure
 * 
 * Contains all parameters needed to configure a button instance
 */
typedef struct {
    gpio_num_t gpio_pin;              /**< GPIO pin number for the button */
    bool active_level;                /**< Active level: true = active high, false = active low */
    bool pull_up_enable;              /**< Enable internal pull-up resistor */
    bool pull_down_enable;            /**< Enable internal pull-down resistor */
    uint32_t debounce_time_ms;        /**< Debounce time in milliseconds (default: 50ms) */
    uint32_t long_press_time_ms;      /**< Long press duration in milliseconds (default: 1000ms) */
    uint32_t double_click_timeout_ms; /**< Double click timeout in milliseconds (default: 300ms) */
} button_config_t;

/**
 * @brief Button event callback function type
 * 
 * User-defined callback function that gets called when button events occur.
 * This function will be called from interrupt context, so it should be fast
 * and not perform blocking operations.
 * 
 * @param event The button event that occurred
 * @param user_data User-provided data pointer (set during initialization)
 */
typedef void (*button_event_callback_t)(button_event_t event, void *user_data);

/**
 * @brief Button controller handle structure
 * 
 * Internal handle structure for button controller instances.
 * Users should not access members directly.
 */
typedef struct button_controller_handle_s* button_controller_handle_t;

/**
 * @brief Initialize button controller
 * 
 * This function configures the specified GPIO pin for button input with interrupt
 * support. It sets up debouncing, pull resistors, and event detection.
 * 
 * @param config Pointer to button configuration structure
 * @param callback Event callback function (can be NULL if events not needed)
 * @param user_data User data pointer passed to callback (can be NULL)
 * @param handle Pointer to store the button handle
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
esp_err_t button_controller_init(const button_config_t *config, 
                                button_event_callback_t callback,
                                void *user_data,
                                button_controller_handle_t *handle);

/**
 * @brief Get current button state
 * 
 * Returns the current debounced state of the button.
 * This function can be called from any context.
 * 
 * @param handle Button controller handle
 * @return Current button state (BUTTON_STATE_PRESSED or BUTTON_STATE_RELEASED)
 */
button_state_t button_controller_get_state(button_controller_handle_t handle);

/**
 * @brief Check if button is currently pressed
 * 
 * Convenience function to check if button is in pressed state.
 * 
 * @param handle Button controller handle
 * @return true if button is pressed, false otherwise
 */
bool button_controller_is_pressed(button_controller_handle_t handle);

/**
 * @brief Get button press duration
 * 
 * Returns how long the button has been pressed in milliseconds.
 * Returns 0 if button is not currently pressed.
 * 
 * @param handle Button controller handle
 * @return Press duration in milliseconds
 */
uint32_t button_controller_get_press_duration(button_controller_handle_t handle);

/**
 * @brief Set event callback function
 * 
 * Updates the event callback function for an initialized button.
 * Can be used to change callback at runtime or disable events by passing NULL.
 * 
 * @param handle Button controller handle
 * @param callback New callback function (NULL to disable events)
 * @param user_data New user data pointer
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
esp_err_t button_controller_set_callback(button_controller_handle_t handle,
                                        button_event_callback_t callback,
                                        void *user_data);

/**
 * @brief Enable or disable button event detection
 * 
 * Temporarily enable or disable button event detection without changing
 * the callback function. Useful for temporarily ignoring button events.
 * 
 * @param handle Button controller handle
 * @param enable true to enable events, false to disable
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
esp_err_t button_controller_enable_events(button_controller_handle_t handle, bool enable);

/**
 * @brief Reset button state and counters
 * 
 * Resets internal state, clears any pending events, and resets counters.
 * Useful for handling edge cases or system recovery.
 * 
 * @param handle Button controller handle
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
esp_err_t button_controller_reset(button_controller_handle_t handle);

/**
 * @brief Get button statistics
 * 
 * Retrieves statistical information about button usage.
 * 
 * @param handle Button controller handle
 * @param total_presses Pointer to store total press count (can be NULL)
 * @param total_releases Pointer to store total release count (can be NULL)
 * @param total_long_presses Pointer to store long press count (can be NULL)
 * @param total_double_clicks Pointer to store double click count (can be NULL)
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
esp_err_t button_controller_get_stats(button_controller_handle_t handle,
                                     uint32_t *total_presses,
                                     uint32_t *total_releases,
                                     uint32_t *total_long_presses,
                                     uint32_t *total_double_clicks);

/**
 * @brief Deinitialize button controller
 * 
 * Releases all resources, disables interrupts, and frees memory.
 * The handle becomes invalid after this call.
 * 
 * @param handle Button controller handle
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
esp_err_t button_controller_deinit(button_controller_handle_t handle);

/**
 * @brief Get event name string
 * 
 * Utility function to convert button event enum to human-readable string.
 * Useful for logging and debugging.
 * 
 * @param event Button event type
 * @return String representation of the event
 */
const char* button_controller_event_to_string(button_event_t event);

#ifdef __cplusplus
}
#endif

#endif // BUTTON_CONTROLLER_H