/**
 * @file led_controller.h
 * @brief LED Controller Component Header
 * 
 * This component provides a simple interface for controlling LEDs using ESP32 GPIO.
 * It demonstrates proper ESP-IDF component structure and GPIO configuration.
 * 
 * @author Learning ESP-IDF
 * @date 2025
 */

#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief LED controller configuration structure
 * 
 * This structure contains all necessary parameters for LED configuration
 */
typedef struct {
    gpio_num_t gpio_pin;      /**< GPIO pin number for the LED */
    bool active_level;        /**< Active level: true = active high, false = active low */
} led_config_t;

/**
 * @brief LED states enumeration
 */
typedef enum {
    LED_STATE_OFF = 0,        /**< LED is turned off */
    LED_STATE_ON = 1,         /**< LED is turned on */
    LED_STATE_TOGGLE = 2      /**< Toggle LED state */
} led_state_t;

/**
 * @brief Initialize LED controller
 * 
 * This function configures the specified GPIO pin for LED control.
 * It sets up the pin as output and initializes it to OFF state.
 * 
 * @param config Pointer to LED configuration structure
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
esp_err_t led_controller_init(const led_config_t *config);

/**
 * @brief Set LED state
 * 
 * Controls the LED state - turn on, off, or toggle.
 * 
 * @param state Desired LED state (LED_STATE_ON, LED_STATE_OFF, or LED_STATE_TOGGLE)
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
esp_err_t led_controller_set_state(led_state_t state);

/**
 * @brief Get current LED state
 * 
 * Returns the current state of the LED.
 * 
 * @return Current LED state (LED_STATE_ON or LED_STATE_OFF)
 */
led_state_t led_controller_get_state(void);

/**
 * @brief Blink LED for specified duration
 * 
 * Makes the LED blink with specified on/off periods for a given duration.
 * This is a blocking function.
 * 
 * @param on_time_ms Time LED stays on (milliseconds)
 * @param off_time_ms Time LED stays off (milliseconds) 
 * @param duration_ms Total blink duration (milliseconds)
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
esp_err_t led_controller_blink(uint32_t on_time_ms, uint32_t off_time_ms, uint32_t duration_ms);

/**
 * @brief Deinitialize LED controller
 * 
 * Releases resources and resets the GPIO pin.
 * 
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
esp_err_t led_controller_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // LED_CONTROLLER_H