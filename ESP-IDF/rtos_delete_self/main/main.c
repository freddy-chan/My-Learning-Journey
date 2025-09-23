#include <stdio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>


static const char *TAG = "main";

void myTask(void *pvParameters)
{
     vTaskDelay(1000 /portTICK_PERIOD_MS);
     ESP_LOGI(TAG,"myTask 1");
     vTaskDelay(1000/ portTICK_PERIOD_MS);
     ESP_LOGI(TAG,"myTask 2");

     vTaskDelete(NULL);

}
void app_main(void)
{
     TaskHandle_t taskHandle =NULL;
     xTaskCreate(myTask, "myTask",2048, NULL, 1, &taskHandle);
}