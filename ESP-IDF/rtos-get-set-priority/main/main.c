// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_log.h"

// static const char *TAG = "TASK_DEMO";

// TaskHandle_t taskHandle_1 = NULL;
// TaskHandle_t taskHandle_2 = NULL;
// TaskHandle_t taskHandle_3 = NULL;

// // ================= Task 1 =================
// void Task_1(void *pvParameters)
// {
//     while (1) {
//         ESP_LOGI(TAG, "Hello from Task_1 (Priority: %d)", uxTaskPriorityGet(NULL));
//         vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second delay
//     }
// }

// // ================= Task 2 =================
// void Task_2(void *pvParameters)
// {
//     int counter = 0;
//     while (1) {
//         ESP_LOGI(TAG, "Hello from Task_2 (Priority: %d)", uxTaskPriorityGet(NULL));
//         vTaskDelay(pdMS_TO_TICKS(1500)); // 1.5 second delay

//         counter++;

//         // After 5 iterations, boost Task_1’s priority
//         if (counter == 5) {
//             ESP_LOGW(TAG, "Boosting Task_1 priority to 15!");
//             vTaskPrioritySet(taskHandle_1, 15);
//         }

//         // After 10 iterations, lower Task_1’s priority back to 12
//         if (counter == 10) {
//             ESP_LOGW(TAG, "Lowering Task_1 priority back to 12!");
//             vTaskPrioritySet(taskHandle_1, 12);
//             counter = 0; // Reset counter, so this cycle repeats
//         }
//     }
// }

// // ================= Task 3 =================
// void Task_3(void *pvParameters)
// {
//     while (1) {
//         ESP_LOGI(TAG, "Background Task_3 running (Priority: %d)", uxTaskPriorityGet(NULL));
//         vTaskDelay(pdMS_TO_TICKS(2000)); // Runs every 2 sec
//     }
// }

// // ================= Main =================
// void app_main(void)
// {
//     // Create Task_1 (Normal work task)
//     xTaskCreate(Task_1, "Task_1", 2048, NULL, 12, &taskHandle_1);
//     ESP_LOGI(TAG, "Task_1 Initial Priority: %d", uxTaskPriorityGet(taskHandle_1));

//     // Create Task_2 (Control task that manages priorities)
//     xTaskCreate(Task_2, "Task_2", 2048, NULL, 12, &taskHandle_2);
//     ESP_LOGI(TAG, "Task_2 Initial Priority: %d", uxTaskPriorityGet(taskHandle_2));

//     // Create Task_3 (Background task with lower priority)
//     xTaskCreate(Task_3, "Task_3", 2048, NULL, 5, &taskHandle_3);
//     ESP_LOGI(TAG, "Task_3 Initial Priority: %d", uxTaskPriorityGet(taskHandle_3));
// }

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "esp_spiffs.h"

static const char *TAG = "SMART_HOME";

// Task handles
TaskHandle_t taskHandle_1 = NULL; // Sensor Task
TaskHandle_t taskHandle_2 = NULL; // MQTT Task
TaskHandle_t taskHandle_3 = NULL; // Logging Task

// Global sensor values (simulated)
static float temperature = 25.0;
static float humidity    = 50.0;
static float lightLevel  = 300.0;

// MQTT client handle
esp_mqtt_client_handle_t mqtt_client = NULL;

// ===================== SENSOR TASK =====================
void Task_Sensor(void *pvParameters)
{
    while (1) {
        // Simulate sensor readings
        temperature += 0.1f;
        humidity    += 0.2f;
        lightLevel  += 1.0f;

        ESP_LOGI(TAG, "Sensor: Temp=%.2f°C, Hum=%.2f%%, Light=%.2f lux, Priority=%d",
                 temperature, humidity, lightLevel, uxTaskPriorityGet(NULL));

        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second
    }
}

// ===================== MQTT TASK =====================
void Task_MQTT(void *pvParameters)
{
    char msg[128];
    int counter = 0;

    while (1) {
        if (mqtt_client) {
            // Prepare JSON message
            snprintf(msg, sizeof(msg),
                     "{\"temperature\": %.2f, \"humidity\": %.2f, \"light\": %.2f}",
                     temperature, humidity, lightLevel);

            esp_mqtt_client_publish(mqtt_client, "/home/sensors", msg, 0, 1, 0);
            ESP_LOGI(TAG, "MQTT Published: %s", msg);

            // Every 5 publishes, boost sensor task priority
            counter++;
            if (counter == 5) {
                ESP_LOGW(TAG, "Boosting Task_Sensor priority to 15!");
                vTaskPrioritySet(taskHandle_1, 15);
            }

            // Every 10 publishes, lower sensor task priority back
            if (counter == 10) {
                ESP_LOGW(TAG, "Lowering Task_Sensor priority back to 12!");
                vTaskPrioritySet(taskHandle_1, 12);
                counter = 0;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5000)); // Publish every 5 seconds
    }
}

// ===================== LOGGING TASK =====================
void Task_Logger(void *pvParameters)
{
    FILE *f;
    while (1) {
        f = fopen("/spiffs/log.txt", "a");
        if (f) {
            fprintf(f, "Temp=%.2f Hum=%.2f Light=%.2f\n", temperature, humidity, lightLevel);
            fclose(f);
            ESP_LOGI(TAG, "Logged sensor data to SPIFFS");
        } else {
            ESP_LOGE(TAG, "Failed to open log file for writing");
        }

        vTaskDelay(pdMS_TO_TICKS(5000)); // Log every 5 sec (low priority)
    }
}

// ===================== WIFI EVENT HANDLER =====================
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "Retrying Wi-Fi connection...");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "Wi-Fi connected, got IP");
    }
}

// ===================== WIFI INIT =====================
static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "POS_Server",
            .password = "asdffdsa",
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

// ===================== MQTT EVENT HANDLER =====================
static void mqtt_event_handler_cb(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            esp_mqtt_client_subscribe(client, "home/fan/cmd", 0);
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT_EVENT_DISCONNECTED");
            break;

        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA: topic=%.*s data=%.*s",
                     event->topic_len, event->topic,
                     event->data_len, event->data);
            // Example: If "ON", increase Task_1 priority
            if (strncmp(event->data, "ON", event->data_len) == 0) {
                ESP_LOGI(TAG, "Command: Turn ON → Boost Task_1");
                vTaskPrioritySet(taskHandle_1, 15);
            } else if (strncmp(event->data, "OFF", event->data_len) == 0) {
                ESP_LOGI(TAG, "Command: Turn OFF → Lower Task_1");
                vTaskPrioritySet(taskHandle_1, 12);
            }
            break;

        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://192.168.0.100", // Public broker for demo
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler_cb, mqtt_client);
    esp_mqtt_client_start(mqtt_client);
}

// ===================== SPIFFS INIT =====================
static void spiffs_init(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };

     ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));

    size_t total = 0, used = 0;
    esp_spiffs_info(NULL, &total, &used);
    ESP_LOGI(TAG, "SPIFFS: total=%d, used=%d", total, used);
}

// ===================== APP MAIN =====================
void app_main(void)
{
    // Init NVS (required for Wi-Fi and MQTT)
    ESP_ERROR_CHECK(nvs_flash_init());

    // Init subsystems
    spiffs_init();
    wifi_init();
    mqtt_app_start();

    // Create Tasks
    xTaskCreate(Task_Sensor, "Task_Sensor", 4096, NULL, 12, &taskHandle_1);
    xTaskCreate(Task_MQTT,   "Task_MQTT",   4096, NULL, 12, &taskHandle_2);
    xTaskCreate(Task_Logger, "Task_Logger", 4096, NULL, 5,  &taskHandle_3);

    ESP_LOGI(TAG, "Smart Home Controller Started!");
}

