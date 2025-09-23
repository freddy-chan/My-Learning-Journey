#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "my_led.h"
#include "sdkconfig.h"

void app_main(void)
{
    my_led_init(CONFIG_MY_LED_GPIO); // ✅ call inside a function

    while (1)
    {
        my_led_toggle();
        vTaskDelay(pdMS_TO_TICKS(500));
        my_led_toggle();
        vTaskDelay(pdMS_TO_TICKS(1000));
        my_led_on();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
