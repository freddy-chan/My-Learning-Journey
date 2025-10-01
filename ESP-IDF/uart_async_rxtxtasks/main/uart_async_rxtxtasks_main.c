#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "string.h"

#define TXD_PIN GPIO_NUM_17   // TX2
#define RXD_PIN GPIO_NUM_16  // RX2
#define UART_PORT UART_NUM_2

static const char *TAG = "UART_ECHO";

void uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));

    // Set TX/RX pins
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, TXD_PIN, RXD_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Install UART driver with RX buffer
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, 2048, 0, 0, NULL, 0));

    ESP_LOGI(TAG, "UART Echo initialized on TX=%d RX=%d", TXD_PIN, RXD_PIN);
}

void uart_echo_task(void *pvParameters)
{
    uint8_t data[128];

    while (1)
    {
        // Read data from UART
        int len = uart_read_bytes(UART_PORT, data, sizeof(data), pdMS_TO_TICKS(1000));
        if (len > 0)
        {
            // Echo back the same data
            uart_write_bytes(UART_PORT, (const char *)data, len);

            // Also log to ESP32 console
            ESP_LOGI(TAG, "Echoed: %.*s", len, data);
        }
    }
}

void app_main(void)
{
    uart_init();
    xTaskCreate(uart_echo_task, "uart_echo_task", 4096, NULL, 5, NULL);
}
