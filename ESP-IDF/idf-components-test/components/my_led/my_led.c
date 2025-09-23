#include "my_led.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"  // only needed if you use CONFIG_ from Kconfig


static const char *TAG = "my_led";
static int s_led_gpio = -1;
static bool s_led_state = false;

esp_err_t my_led_init(gpio_num_t gpio_num)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << gpio_num),
        .pull_down_en = 0,
        .pull_up_en = 0
    };

    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config failed: %d", err);
        return err;
    }
    s_led_gpio = (int)gpio_num;
    s_led_state = false;
    gpio_set_level((gpio_num_t)s_led_gpio, 0);
    ESP_LOGI(TAG, "LED init on GPIO %d", s_led_gpio);
    return ESP_OK;
}

esp_err_t my_led_on(void)
{
    if (s_led_gpio < 0) return ESP_ERR_INVALID_STATE;
    gpio_set_level((gpio_num_t)s_led_gpio, 1);
    s_led_state = true;
    return ESP_OK;
}

esp_err_t my_led_off(void)
{
    if (s_led_gpio < 0) return ESP_ERR_INVALID_STATE;
    gpio_set_level((gpio_num_t)s_led_gpio, 0);
    s_led_state = false;
    return ESP_OK;
}

esp_err_t my_led_toggle(void)
{
    if (s_led_gpio < 0) return ESP_ERR_INVALID_STATE;
    s_led_state = !s_led_state;
    gpio_set_level((gpio_num_t)s_led_gpio, s_led_state ? 1 : 0);
    return ESP_OK;
}
