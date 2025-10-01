#ifndef MAIN_H
#define MAIN_H

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

// Pin definitions
#define DHT22_PIN           4
#define MPU_SDA_PIN         21
#define MPU_SCL_PIN         22
#define SD_MOSI_PIN         13
#define SD_MISO_PIN         12
#define SD_SCK_PIN          14
#define SD_CS_PIN           15
#define LED_PIN             2
#define BUTTON_PIN          0

// Task priorities
#define DHT_TASK_PRIORITY   3
#define MPU_TASK_PRIORITY   3
#define SD_TASK_PRIORITY    2
#define MQTT_TASK_PRIORITY  2
#define OTA_TASK_PRIORITY   1

// Data structures
typedef struct {
    float temperature;
    float humidity;
    uint64_t timestamp;
} dht_data_t;

typedef struct {
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    uint64_t timestamp;
} mpu_data_t;

// External queues
extern QueueHandle_t dht_queue;
extern QueueHandle_t mpu_queue;

#endif