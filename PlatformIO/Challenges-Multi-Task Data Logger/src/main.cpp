/*
- Objective : Log sensor data to an SD card using queues and task.
-Steps:
  1-Task1: Generate fake sensor data and send to a queue
  2-Task2: Receive data form the queue and "write" to a file (simulate with printf)
  3-Challenge: Handle queue overflow when the SD card is slow (use blocking/timeout in xQueueSend()).

  in there you will be face with queue overflow so solve it
*/

#include <Arduino.h>
#include "freertos/task.h"
#include "freertos/queue.h"

// Create the queue handler
QueueHandle_t sensor_queue;
// Task for sensor
void sensor_task(void *pvParam) {
  int data = 0;
  while (1) {
    data = rand() % 100;
    // Try to send data with a 100ms timeout
    if (xQueueSend(sensor_queue, &data, pdMS_TO_TICKS(100)) != pdPASS) {
      printf("Queue overflow! Data lost: %d\n", data);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}
// Task for data logger
void logger_task(void *pvParam) {
  int received_data;
  while (1) {
    if (xQueueReceive(sensor_queue, &received_data, portMAX_DELAY)) {
      printf("Logged: %d\n", received_data);
    }
    // Check queue length
    UBaseType_t items = uxQueueMessagesWaiting(sensor_queue);
    printf("Queue items: %u\n", items);
    vTaskDelay(1000/ portTICK_PERIOD_MS);
  }
}
void setup()
{
  sensor_queue=xQueueCreate(10,sizeof(int));
  xTaskCreate(sensor_task,"SensorTask",2048,NULL,2,NULL);
  xTaskCreate(logger_task,"LoggerTask",2048,NULL,2,NULL);
}

void loop()
{
  // put your main code here, to run repeatedly:
}
