#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "MAIN";
typedef struct
{
    int Int;
    int Array[3];
} MyStruct;

// Task1 for integer
void Task_1(void *pvParameters)
{
    int *pInt = (int *)pvParameters;
    ESP_LOGI(TAG, "Task 1 for int : %d", *pInt);
    vTaskDelete(NULL);
}
// Task 2 for array
void Task_2(void *pvParameters)
{
    int *pArray = (int *)pvParameters;
    ESP_LOGI(TAG, "Task 2 for array: %d %d %d", *pArray, *(pArray + 1), *(pArray + 2));
    vTaskDelete(NULL);
}

// Task 3 for strcut
void Task_3(void *pvParameters)
{
    MyStruct *pStruct = (MyStruct *)pvParameters;
    ESP_LOGI(TAG, "Task 3 for struct : %d %d %d %d", pStruct->Int, pStruct->Array[0], pStruct->Array[1], pStruct->Array[2]);
    vTaskDelete(NULL);
}

// Task 4 for Char
void Task_4(void *pvParameters)
{
    char *pChar = (char *)pvParameters;
    ESP_LOGI(TAG, "Task 4 for Char : %s", pChar);
    vTaskDelete(NULL);
}

int Parameters_1 = 1;
int Parameters_2[3] = {1, 2, 3};
MyStruct Parameters_3 = {1, {1, 2, 3}};
static const char *Parameters_4 = "Hello World!";

void app_main(void)
{
    xTaskCreate(Task_1, "Task_1", 2048, (void *)&Parameters_1, 25, NULL);
    xTaskCreate(Task_2, "Task_2", 2048, (void *)&Parameters_2, 1, NULL);
    xTaskCreate(Task_3, "Task_3", 3048, (void *)&Parameters_3, 1, NULL);
    xTaskCreate(Task_4, "Task_4", 3048, (void *)Parameters_4, 1, NULL);
}