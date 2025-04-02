#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LED_PIN 2 // Default LED
// Global variable to stroe the delay time (in ms)
volatile int blinkDelay = 1000;

// Task handles
TaskHandle_t serialTaskHandle = NULL;
TaskHandle_t blinkTaskHandle = NULL;

// Function to convert task state to human-readable format
const char *getTaskState(eTaskState state)
{
    switch (state)
    {
    case eRunning:
        return "Running";
        break;
    case eReady:
        return "Ready";
        break;
    case eBlocked:
        return "Blocked";
        break;
    case eSuspended:
        return "Suspended";
        break;
    case eDeleted:
        return "Deleted";
        break;
    default:
        break;
    }
}
// Task to print task states
void TaskStateTask(void *pvParameters){
    (void)pvParameters;
    while (1)
    {
       Serial.println("Task states: ");
       Serial.print("Serial Task: ");
       Serial.println(getTaskState(eTaskGetState(serialTaskHandle)));
       Serial.print("Blink Task: ");
       Serial.println(getTaskState(eTaskGetState(blinkTaskHandle)));
       Serial.println("-------------------");
       vTaskDelay(pdMS_TO_TICKS(2000)); 
    }
    
}
// Task to read input from serial terminal
void SerailTask(void *pvParameters)
{
    (void)pvParameters;
    Serial.begin(115200);
    Serial.println("Enter delay time in ms for LED blinking(between 100ms and 5000ms):");
    while (1)
    {
        if (Serial.available() > 0)
        {
            int input = Serial.parseInt(); // Read integer form serial
            if (input >= 100 && input <= 5000)
            {
                blinkDelay = input; // Update blink delay
                Serial.print("Updated blink delay to: ");
                Serial.print(blinkDelay);
                Serial.println(" ms");
            }
            else
            {
                Serial.println("Pleae enter a value between 100 and 5000ms");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
void BlinkTask(void *pvParameters)
{
    (void)pvParameters;
    pinMode(LED_PIN, OUTPUT);
    while (1)
    {
        digitalWrite(LED_PIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(blinkDelay));
        digitalWrite(LED_BUILTIN, LOW);
        vTaskDelay(pdMS_TO_TICKS(blinkDelay));
    }
}

void setup()
{
    // Create Tasks
    xTaskCreate(SerailTask, "SerialTask", 1024, NULL, 2, &serialTaskHandle);
    xTaskCreate(BlinkTask, "BlinkTask", 1024, NULL, 1, &blinkTaskHandle);
    xTaskCreate(TaskStateTask,"TaskStateTask",1024,NULL,1,NULL);
}

void loop()
{
}