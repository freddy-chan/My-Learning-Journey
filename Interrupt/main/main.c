

#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#define LED_PIN 14
#define BUTTON_PIN 13
// Create queue variable
static QueueHandle_t gpio_evt_queue = NULL;
static volatile bool led_state = false;
static volatile bool should_blink = true;

// Interrupt Service Handler
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    // Blockဖြစ်စေမဲ့အရာတွေမရေးနဲ့
    uint32_t gpio_num = (uint32_t)arg;
    // Send to queue form ISR
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void led_task(void *arg)
{
    while (1)
    {
        if (should_blink)
        {
            led_state = !led_state;             // if true ,toggle the led_state
            gpio_set_level(LED_PIN, led_state); // set the led
        }
        vTaskDelay(pdMS_TO_TICKS(500));

        /* code */
    }
}

static void button_task(void *arg)
{
    uint32_t gpio_num;
    while (1)
    {
        // Need to receive form queue cuz ISR
        if (xQueueReceive(gpio_evt_queue, &gpio_num, portMAX_DELAY))
        {
            // for debounce time
            vTaskDelay(pdMS_TO_TICKS(50));
            should_blink = !should_blink;
            gpio_set_level(LED_PIN, led_state);
        }
        /* code */
    }
}
void app_main(void)
{
    // Create Queue
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    // Configuration for ledpin
    gpio_config_t led_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LED_PIN),
        .pull_down_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE};
    gpio_config(&led_conf);

    // Configuration for Button PIN
    gpio_config_t button_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .pull_up_en = GPIO_PULLUP_ENABLE,

    };
    gpio_config(&button_conf);

    // Install ISR
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN, gpio_isr_handler, (void *)BUTTON_PIN);

    // For task
    xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);
    xTaskCreate(led_task, "led_task", 2048, NULL, 10, NULL);
}