#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

static const char *TAG = "TIMER_DEMO";

void TimerCallback(TimerHandle_t xTimer)
{
    const char *name = pcTimerGetName(xTimer);
    int id = (int)pvTimerGetTimerID(xTimer);
    ESP_LOGI(TAG, "Timer '%s' fired! ID=%d", name, id);
}

void app_main(void)
{
    // Periodic timer: fires every 1s
    TimerHandle_t periodicTimer = xTimerCreate("Periodic", pdMS_TO_TICKS(1000),
                                               pdTRUE, (void *)0, TimerCallback);

    // One-shot timer: fires once after 3s
    TimerHandle_t oneShotTimer = xTimerCreate("OneShot", pdMS_TO_TICKS(3000),
                                              pdFALSE, (void *)1, TimerCallback);

    xTimerStart(periodicTimer, 0);
    xTimerStart(oneShotTimer, 0);
}

