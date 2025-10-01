// /*
// this program for testing the vTaskResume and xTaskResume FromISR functions
// FreeRTOS provides multiple methods to display and 
// analyze task information to help developers understand system operation status,
// optimize performance, and locate problems.၏
// 1. vTaskList(): Outputs a list of all tasks and their states.
// 2. vTaskGetInfo(): Retrieves detailed information about a specific task.
// 3. uxTaskGetSystemState(): Provides a snapshot of the current state of all tasks.
// 4. uxTaskGetStackHighWaterMark(): Returns the minimum amount of stack space
//    that has remained for a task since it started executing.
// */
// #include <stdio.h>
// #include "esp_log_level.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/projdefs.h"
// #include "freertos/task.h"
// #include "esp_log.h"

// static const char *TAG = "main";

// TaskHandle_t taskHandle_1 = NULL; // Task handle for Task 1

// // Task 1
// void Task_1(void *ppvParameters)
// {
//     while (1)
//     {
//         ESP_LOGI(TAG, "Task 1 is running ");
//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }
// }
// // Task 2
// void Task_2(void *pvParameters)
// {
//     while (1)
//     {
//         ESP_LOGI(TAG, "Task 2 is running ");
//         vTaskSuspend(taskHandle_1);      // Suspend Task 1)
//         vTaskDelay(pdMS_TO_TICKS(5000)); // Delay for 5 seconds

//         ESP_LOGI(TAG, "Resuming Task 1 from Task 2");
//         vTaskResume(taskHandle_1);       // Resume Task 1
//         vTaskDelay(pdMS_TO_TICKS(1000)); // Delay to allow Task 1 to run
//     }
// }

// void app_main(void)
// {
//     // Create Task 1
//     xTaskCreate(Task_1,
//                 "Task_1",
//                 2048,
//                 NULL,
//                 5,
//                 &taskHandle_1);
//     // Create Task 2
//     xTaskCreate(Task_2,
//                 "Task_2",
//                 2048,
//                 NULL,
//                 5,
//                 NULL);
// }



/*
vTaskList: Output task list
It can be used to vTaskList()assist in analyzing the current task status 
of the operating system to help optimize memory, help locate stack overflow problems, 
and help understand and learn knowledge related to operating system principles.
0 နားနီးလေလေ stackoverflow risk များလေးဖြစ်ပါတယ်။

*/
#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "main";

void Task_1(void *pvParameters)
{
    for (;;)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Task_1");
    }
    // vTaskDelete(NULL);
}

void Task_2(void *pvParameters)
{
    for (;;)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Task_2");
    }
    // vTaskDelete(NULL);
}

void app_main(void)
{
    TaskHandle_t taskHandle_1 = NULL;
    TaskHandle_t taskHandle_2 = NULL;
    
    xTaskCreate(Task_1, "Task_1", 1024, NULL, 12, &taskHandle_1);
    xTaskCreate(Task_2, "Task_2", 2048, NULL, 12, &taskHandle_2);

    
    // 输出任务列表
    static char cBuffer[512]={0};
    vTaskList(cBuffer);
    ESP_LOGI(TAG, "vTaskList ：\n%s", cBuffer);

    while (1)
    {
        int istack = uxTaskGetStackHighWaterMark(taskHandle_1);
        ESP_LOGI(TAG, "Task_1 Stack :%d", istack);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        int istack2 = uxTaskGetStackHighWaterMark(taskHandle_2);
        ESP_LOGI(TAG, "Task_2 Stack :%d", istack2);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}