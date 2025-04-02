#include <Arduino.h>
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/queue.h"

QueueHandle_t queue;

void sender_task(void *pvParam)
{
  int data = 0;
  while (1)
  {
    UBaseType_t highWaterMark = uxTaskGetStackHighWaterMark(NULL);
    printf("High Water Mark: %u\n", highWaterMark);
    xQueueSend(queue, &data, 0); // Send data to queue
    data++;
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void receiver_task(void *pvParam)
{
  int received_data;
  while (1)
  {
    if (xQueueReceive(queue, &received_data, portMAX_DELAY))
    {
      printf("Received: %d\n", received_data);
    }
  }
}

void setup()
{
  queue = xQueueCreate(5, sizeof(int)); // Queue holds 5 integers
  xTaskCreate(sender_task, "sender", 2048, NULL, 2, NULL);
  xTaskCreate(receiver_task, "receiver", 2048, NULL, 2, NULL);
}
void loop()
{
}