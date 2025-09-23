#include <stdio.h>
#include <string.h>
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/uart.h"

// Hardware Configuration
#define LED_PIN         2
#define UART_NUM        UART_NUM_0
#define BUF_SIZE        1024
#define QUEUE_SIZE      5

// Message Types
typedef struct {
    char message[50];
} queue2_msg_t;

// Global Resources
QueueHandle_t queue1;
QueueHandle_t queue2;
int blink_delay = 500;

void uart_init() {
    uart_config_t uart_cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB
    };
    
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_cfg));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE, BUF_SIZE, 0, NULL, 0));
}

void TaskA(void *pvParam) {
    char input[32] = {0};
    uint8_t index = 0;
    queue2_msg_t msg;

    printf("System Ready. Enter delay (1-5000ms):\n");

    while(1) {
        // Check for status messages
        if(xQueueReceive(queue2, &msg, 0)) {
            printf("[Status] %s\n", msg.message);
        }

        // Handle UART input
        uint8_t data;
        int len = uart_read_bytes(UART_NUM, &data, 1, pdMS_TO_TICKS(20));
        
        if(len > 0) {
            if(data == '\r' || data == '\n') {
                input[index] = '\0';
                int new_delay = atoi(input);
                
                if(new_delay >= 1 && new_delay <= 5000) {
                    xQueueSend(queue1, &new_delay, portMAX_DELAY);
                    printf("Delay updated to: %dms\n", new_delay);
                } else {
                    printf("Invalid delay! Valid range: 1-5000ms\n");
                }
                
                index = 0;
                memset(input, 0, sizeof(input));
                printf("Enter new delay: ");
            }
            else if(data == '\b' || data == 0x7F) {
                if(index > 0) {
                    index--;
                    input[index] = '\0';
                    printf("\b \b");
                }
            }
            else if(index < sizeof(input) - 1 && data >= 32 && data <= 126) {
                input[index++] = data;
                printf("%c", data);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void TaskB(void *pvParam) {
    uint32_t blink_count = 0;
    queue2_msg_t status;
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    while(1) {
        // Update delay
        int new_delay;
        if(xQueueReceive(queue1, &new_delay, 0)) {
            blink_delay = new_delay;
        }

        // Blink LED
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(blink_delay));
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(blink_delay));

        // Send status
        if(++blink_count % 100 == 0) {
            snprintf(status.message, sizeof(status.message),
                   "Blinked %lu times @ %dms", blink_count, blink_delay);
            xQueueSend(queue2, &status, portMAX_DELAY);
        }
    }
}

void setup() {
    // Initialize hardware
    uart_init();
    gpio_reset_pin(LED_PIN);
    
    // Create queues
    queue1 = xQueueCreate(QUEUE_SIZE, sizeof(int));
    queue2 = xQueueCreate(QUEUE_SIZE, sizeof(queue2_msg_t));

    // Create tasks
    xTaskCreate(TaskA, "Terminal", 4096, NULL, 2, NULL);
    xTaskCreate(TaskB, "Blinker", 2048, NULL, 1, NULL);

    // Cleanup initial task
    vTaskDelete(NULL);
}
void loop(){

}