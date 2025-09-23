#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// #include "esp_head_caps.h"

TaskHandle_t taskHandle;

void momoryTask(void *pvParameters)
{
  UBaseType_t highWaterMark;
  size_t heapBefore, heapAfter;
  void *ptr;

  printf("Starting memory task...\n");

  highWaterMark = uxTaskGetStackHighWaterMark(NULL);
  printf("Initial Stack High Water Mark: %u bytes\n", highWaterMark);

  // Show heap size before malloc
  heapBefore = esp_get_free_heap_size();
  printf("Heap before malloc: %u bytes\n", heapBefore);

  // Allocate 1024 bytes dynamically
  ptr = pvPortMalloc(1024);
  if (ptr == NULL)
  {
    printf("Memory allocation failed!\n");
  }
  else
  {
    printf("Memory allocated: 1024 bytes\n");
  }
  // Show  heap size after malloc
  heapAfter = esp_get_free_heap_size();
  printf("Heap after malloc: %u bytes\n", heapAfter);
  // Check High Water Mark again
  highWaterMark = uxTaskGetStackHighWaterMark(NULL);
  printf("Stack High Water Mark after malloc: %u bytes\n", highWaterMark);

  // Free allocated memory
  if (ptr != NULL)
  {
    vPortFree(ptr);
    printf("Memory freed.\n");
  }
  vTaskDelete(NULL); // Delete this task
}

void setup()
{
  xTaskCreate(momoryTask, "MemoryTask", 2048, NULL, 1, &taskHandle);
}
void loop(){

}