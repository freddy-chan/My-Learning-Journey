#include "driver/gpio.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define BUTTON_PIN GPIO_NUM_0
#define LED_PIN    GPIO_NUM_2

static const char *TAG = "main";
static volatile int led_state = 0; // keep led state globally

// Interrupt Service Routine (ISR)
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    led_state = !led_state;
    gpio_set_level(LED_PIN, led_state);

    // ISR-safe logging
    ESP_EARLY_LOGI(TAG, "LED state is %s", led_state ? "ON" : "OFF");
}

void app_main(void)
{
    // Configure button as INPUT with pull-up
    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,   // ✅ Enable pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&btn_conf);

    // Configure LED as OUTPUT
    gpio_config_t led_conf = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,           // ✅ Correct mode
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&led_conf);

    // Install ISR service
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN, gpio_isr_handler, (void *)BUTTON_PIN);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "Hello Human");
    }
}
