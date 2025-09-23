#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_task_wdt.h"

/*
watchdog example for FreeRTOS on ESP32
- Two tasks are created: Task A and Task B
- Task A feeds the watchdog every 2 seconds (within the 5s timeout)
- Task B feeds the watchdog 3 times, then simulates a hang by not feeding it anymore
- The watchdog is configured to reset the system if any task fails to feed it within the timeout    
- The default TWDT is deinitialized and reconfigured with custom settings
- Logs are printed to indicate the status of each task and watchdog events 
  including when Task B simulates a hang.

*/
#define TAG "WDT_MULTI"
#define WDT_TIMEOUT_S 5  // 5 seconds watchdog timeout

TaskHandle_t taskA_handle = NULL;
TaskHandle_t taskB_handle = NULL;

// ---------------- Task A ----------------
void taskA(void *pvParameters)
{
    esp_task_wdt_add(NULL);  // Register this task to WDT
    while (1) {
        ESP_LOGI(TAG, "Task A: Feeding watchdog...");
        esp_task_wdt_reset(); // Feed watchdog
        vTaskDelay(pdMS_TO_TICKS(2000)); // 2s delay < timeout
    }
}

// ---------------- Task B ----------------
void taskB(void *pvParameters)
{
    esp_task_wdt_add(NULL);  // Register this task to WDT
    for (int i = 0; i < 3; i++) {
        ESP_LOGI(TAG, "Task B: Feeding watchdog (iteration %d)...", i+1);
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGW(TAG, "Task B: Simulating hang! Will not feed watchdog anymore...");
    while (1) {
        ESP_LOGI(TAG, "Task B: Running, but not feeding watchdog!");
        vTaskDelay(pdMS_TO_TICKS(1000));
        // NO watchdog reset here -> causes WDT trigger
    }
}

// ---------------- Main ----------------
void app_main(void)
{
    ESP_LOGI(TAG, "Reconfiguring Task Watchdog...");

    // Step 1: Deinit the default TWDT
    esp_task_wdt_deinit();

    // Step 2: Configure TWDT with custom settings
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = WDT_TIMEOUT_S * 1000,              // 5s timeout
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1, // Watch all idle tasks
        .trigger_panic = true,                           // Reset on trigger
    };

    ESP_ERROR_CHECK(esp_task_wdt_init(&twdt_config));

    // Step 3: Create tasks
    xTaskCreate(taskA, "Task_A", 2048, NULL, 5, &taskA_handle);
    xTaskCreate(taskB, "Task_B", 2048, NULL, 5, &taskB_handle);
}
