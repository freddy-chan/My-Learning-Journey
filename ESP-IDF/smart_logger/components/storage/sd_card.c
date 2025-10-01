#include "sd_card.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include "esp_log.h"
#include "main.h"

static const char *TAG = "SD_CARD";
static sdmmc_card_t *card;

esp_err_t sd_card_init(void)
{
    esp_err_t ret;
    
    // Configuration
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_MOSI_PIN,
        .miso_io_num = SD_MISO_PIN,
        .sclk_io_num = SD_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    
    ret = spi_bus_initialize(SDSPI_HOST, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }
    
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_CS_PIN;
    slot_config.host_id = SDSPI_HOST;
    
    sdmmc_host_t host = SDSPI_HOST_CONFIG_DEFAULT();
    
    // Mount configuration
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    
    ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Print card info
    sdmmc_card_print_info(stdout, card);
    
    ESP_LOGI(TAG, "SD card mounted successfully");
    return ESP_OK;
}

void sd_task(void *pvParameters)
{
    FILE *file = NULL;
    dht_data_t dht_data;
    mpu_data_t mpu_data;
    
    // Open file for appending
    file = fopen("/sdcard/sensor_data.csv", "a");
    if (file == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }
    
    // Write header if file is empty
    fseek(file, 0, SEEK_END);
    if (ftell(file) == 0) {
        fprintf(file, "timestamp,temp,humidity,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z\n");
        fflush(file);
    }
    
    while (1) {
        // Check for DHT data
        if (xQueueReceive(dht_queue, &dht_data, pdMS_TO_TICKS(10)) == pdTRUE) {
            // Process DHT data with MPU data if available
            if (xQueuePeek(mpu_queue, &mpu_data, pdMS_TO_TICKS(10)) == pdTRUE) {
                fprintf(file, "%llu,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\n",
                        dht_data.timestamp,
                        dht_data.temperature,
                        dht_data.humidity,
                        mpu_data.accel_x,
                        mpu_data.accel_y,
                        mpu_data.accel_z,
                        mpu_data.gyro_x,
                        mpu_data.gyro_y,
                        mpu_data.gyro_z);
                
                fflush(file);
                ESP_LOGI(TAG, "Data written to SD card");
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100)); // Small delay to prevent task starvation
    }
    
    fclose(file);
}