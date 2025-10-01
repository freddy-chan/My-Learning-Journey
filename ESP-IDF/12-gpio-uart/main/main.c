#include "esp_err.h"
#include "freertos/projdefs.h"
#include "hal/uart_types.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include <stdio.h>
#include <string.h>

#define TX_GPIO_NUM 17
#define RX_GPIO_NUM 16

//@brief  Uart  Configuration
void uart_config(void)
{
    const uart_port_t uart_num = UART_NUM_2;
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };
    // UART parameter configuration
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    // Set UART pins (using UART0 default pins ie no changes.)
    ESP_ERROR_CHECK(uart_set_pin(uart_num, TX_GPIO_NUM, RX_GPIO_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    const int uart_buffer_size = (1024 * 2);
    // We won't use a event queue here, but we need to tell the driver to install
    // one. We also won't use a buffer for sending data.
    ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size, 0, 0, NULL, 0));
}
// Uart send receive data
void uart_send_receive_demo(void)
{
    const uart_port_t uart_num = UART_NUM_2;
    char *test_str = "This is uart send and receive demo\r\n";
    uint8_t data[128];
    // Send data
    uart_write_bytes(uart_num, test_str, strlen(test_str));
    // Read data
    while (1)
    {
        /* code */
        vTaskDelay(1);
        int length = 0;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t *)&length));
        if (length > 0)

        {
            int read_length = uart_read_bytes(uart_num, data, length, pdMS_TO_TICKS(1000));
            // Print what is read
            if (read_length > 0)
            {
                printf("Read %d bytes: '%.*s'\n", read_length, read_length, data);
            }
        }
    }
}
void app_main(void)
{

    // Uart configuration 
    uart_config();
    // Uart send receive demo
    uart_send_receive_demo();
}