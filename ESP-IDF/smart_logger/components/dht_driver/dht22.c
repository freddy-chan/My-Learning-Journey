#include "dht22.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "main.h"

static const char *TAG = "DHT22";
static int dht_gpio;

esp_err_t dht22_init(gpio_num_t gpio)
{
    dht_gpio = gpio;
    
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << dht_gpio),
        .mode = GPIO_MODE_OUTPUT_OD, // Open drain
        .pull_up_en = 1,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    
    ESP_LOGI(TAG, "DHT22 initialized on GPIO %d", gpio);
    return ESP_OK;
}

static esp_err_t dht22_read_raw(uint8_t *data)
{
    int pulse_counts[85];
    
    // Start signal
    gpio_set_direction(dht_gpio, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(dht_gpio, 0);
    vTaskDelay(pdMS_TO_TICKS(20)); // 20ms low pulse
    gpio_set_level(dht_gpio, 1);
    ets_delay_us(30); // 30us high pulse
    
    // Switch to input
    gpio_set_direction(dht_gpio, GPIO_MODE_INPUT);
    
    // Wait for response from DHT22
    int loop_count = 0;
    while (gpio_get_level(dht_gpio) == 1 && loop_count++ < 100) {
        ets_delay_us(1);
    }
    
    if (loop_count >= 100) {
        ESP_LOGE(TAG, "DHT22 not responding");
        return ESP_FAIL;
    }
    
    loop_count = 0;
    while (gpio_get_level(dht_gpio) == 0 && loop_count++ < 100) {
        ets_delay_us(1);
    }
    
    if (loop_count >= 100) {
        ESP_LOGE(TAG, "DHT22 response error");
        return ESP_FAIL;
    }
    
    // Read 85 pulses (40 data + 1 start + 44 sync)
    for (int i = 0; i < 85; i++) {
        loop_count = 0;
        while (gpio_get_level(dht_gpio) == 1 && loop_count++ < 100) {
            ets_delay_us(1);
        }
        pulse_counts[i] = loop_count;
        
        loop_count = 0;
        while (gpio_get_level(dht_gpio) == 0 && loop_count++ < 100) {
            ets_delay_us(1);
        }
    }
    
    // Parse data (skip first 3 pulses, take next 40)
    for (int i = 0; i < 5; i++) {
        uint8_t byte = 0;
        for (int j = 0; j < 8; j++) {
            int pulse_idx = 3 + i * 8 + j;
            if (pulse_counts[pulse_idx] > 3) {
                byte |= (1 << (7 - j));
            }
        }
        data[i] = byte;
    }
    
    return ESP_OK;
}

static esp_err_t dht22_read(float *temperature, float *humidity)
{
    uint8_t data[5];
    
    if (dht22_read_raw(data) != ESP_OK) {
        return ESP_FAIL;
    }
    
    // Verify checksum
    uint8_t checksum = data[0] + data[1] + data[2] + data[3];
    if (checksum != data[4]) {
        ESP_LOGE(TAG, "DHT22 checksum error: %d != %d", checksum, data[4]);
        return ESP_FAIL;
    }
    
    // Calculate temperature and humidity
    uint16_t raw_humidity = (data[0] << 8) | data[1];
    uint16_t raw_temperature = ((data[2] & 0x7F) << 8) | data[3];
    
    *humidity = raw_humidity / 10.0;
    *temperature = raw_temperature / 10.0;
    
    if (data[2] & 0x80) { // Negative temperature
        *temperature = -*temperature;
    }
    
    return ESP_OK;
}

void dht_task(void *pvParameters)
{
    float temperature, humidity;
    
    while (1) {
        if (dht22_read(&temperature, &humidity) == ESP_OK) {
            dht_data_t dht_data = {
                .temperature = temperature,
                .humidity = humidity,
                .timestamp = esp_timer_get_time()
            };
            
            ESP_LOGI(TAG, "T: %.2f°C, H: %.2f%%", temperature, humidity);
            
            if (xQueueSend(dht_queue, &dht_data, portMAX_DELAY) != pdTRUE) {
                ESP_LOGW(TAG, "Failed to send DHT data to queue");
            }
        } else {
            ESP_LOGE(TAG, "Failed to read DHT22");
        }
        
        vTaskDelay(pdMS_TO_TICKS(5000)); // Read every 5 seconds
    }
}