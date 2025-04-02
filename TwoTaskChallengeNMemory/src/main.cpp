#include <Arduino.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

TaskHandle_t TaskAHandle = NULL;
TaskHandle_t TaskBHandle = NULL;
char *message = NULL;

void TaskA(void *pvParameters)
{
  String input = "";
  while (1)
  {
    if (Serial.available() > 0)
    {
      char c = Serial.read();
      if (c == '\n')
      { // End of message
        message = (char *)malloc(input.length() + 1);
        if (message != NULL)
        {
          strcpy(message, input.c_str());
          xTaskNotifyGive(TaskBHandle); // Notify Task B
        }
        input = ""; // Clear input buffer
      }
      else
      {
        input += c; // Append char to input
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // Small delay to   prevent watchdog trigger
  }
}

void TaskB(void *pvParameters)
{
  while (1)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait for notification
    if (message != NULL)
    {
      Serial.print("Received: ");
      Serial.println(message);
      free(message);  // Free allocated heap memory
      message = NULL; // Reset Pointer
    }
  }
}
void setup()
{
  Serial.begin(115200);
  xTaskCreatePinnedToCore(TaskA, "Task A", 2048, NULL, 1, &TaskAHandle, 0);
  xTaskCreatePinnedToCore(TaskB, "Task B", 2048, NULL, 1, &TaskBHandle, 1);
}

void loop()
{

}
