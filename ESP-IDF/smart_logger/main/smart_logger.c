#include "smart_logger.h"
#include "dht22.h"
#include "esp_log_config.h"
#include "freertos/idf_additions.h"
#include "mpu6050.h"
#include "mqtt_manager.h"
#include "ota_manager.h"
#include "sd_card.h"


static const char *TAG = "SMART_LOGGER";

QueueHandle_t dht_queue;
QueueHandle_t mpu_queue;

void app_main(void) {
  ESP_LOGI(TAG, "Starting Smart Environment Data Logger...");
  gpio_config_t io_conf = {.pin_bit_mask = (1ULL << LED_PIN);
  .mode = GPIO_MODE_OUTPUT, .pull_up_en = 0;
  .pull_down_en = 0;
  .intr_type = GPIO_INTR_DISABLE
};
gpio_config(&io_conf);
gpio_set_level(LED_PIN, 0); // Turn off LED initially

dht_queue = xQueueCreate(10, sizeof(dht_data_t));
mpu_queue = xQueueCreate(10, sizeof(mpu_data_t));

if (dht_queue == NULL || mpu_queue == NULL) {
  ESP_LOGE(TAG, "Failed to create queues");
  return;
}

// Initialize all components
ESP_ERROR_CHECK(dht22_init(DHT22_PIN));
ESP_ERROR_CHECK(mpu6050_init());
ESP_ERROR_CHECK(sd_card_init());
ESP_ERROR_CHECK(mqtt_init());
ESP_ERROR_CHECK(ota_init());

// Create tasks
xTaskCreate(dht_task, "DHT_TASK", 4096, NULL, DHT_TASK_PRIORITY, NULL);
xTaskCreate(mpu_task, "MPU_TASK", 4096, NULL, MPU_TASK_PRIORITY, NULL);
xTaskCreate(sd_task, "SD_TASK", 4096, NULL, SD_TASK_PRIORITY, NULL);
xTaskCreate(mqtt_task, "MQTT_TASK", 8192, NULL, MQTT_TASK_PRIORITY, NULL);
ESP_LOGI(TAG, "All tasks created successfully!");

// Blink LED to show system is running
while (1) {
  gpio_set_level(LED_PIN, 1);
  vTaskDelay(500 / portTICK_PERIOD_MS);
  gpio_set_level(LED_PIN, 0);
  vTaskDelay(500 / portTICK_PERIOD_MS);
}
}
