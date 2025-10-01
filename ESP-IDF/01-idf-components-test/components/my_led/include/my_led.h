#pragma once
#include "esp_err.h"
#include "driver/gpio.h"

esp_err_t my_led_init(gpio_num_t gpio_num);
esp_err_t my_led_on(void);
esp_err_t my_led_off(void);
esp_err_t my_led_toggle(void);
