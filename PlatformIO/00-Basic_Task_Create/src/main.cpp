#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void high_prio_task(void *pvParam) {
  while(1) {
      printf("High priority task\n");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void low_prio_task(void *pvParam) {
  while(1) {
      printf("Low priority task\n");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void setup() {
  xTaskCreate(high_prio_task, "high_prio_task", 2048, NULL, 3, NULL);
  xTaskCreate(low_prio_task, "low_prio_task", 2048, NULL, 1, NULL);
}
void loop()
{
}