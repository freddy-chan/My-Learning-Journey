#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "main";

void myTask(void *pvParameters)
{
    for (;;)
    {
        vTaskDelay(500 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "myTask");
    }
}

void app_main(void)
{
 
    TaskHandle_t taskHandle = NULL;
    xTaskCreate(myTask, "myTask", 2048, NULL, 1, &taskHandle);

    vTaskDelay(2000 / portTICK_PERIOD_MS);
    if (taskHandle != NULL)
    {
        vTaskDelete(taskHandle);
    }
}