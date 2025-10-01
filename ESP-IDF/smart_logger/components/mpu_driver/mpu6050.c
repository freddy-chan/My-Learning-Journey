#include "mpu6050.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "main.h"

static const char *TAG = "MPU6050";

#define MPU6050_ADDR          0x68
#define MPU6050_PWR_MGMT_1    0x6B
#define MPU6050_ACCEL_XOUT_H  0x3B
#define MPU6050_GYRO_XOUT_H   0x43

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = MPU_SDA_PIN,
        .scl_io_num = MPU_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000
    };
    
    esp_err_t ret = i2c_param_config(I2C_NUM_0, &conf);
    if (ret != ESP_OK) {
        return ret;
    }
    
    return i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

static esp_err_t mpu6050_write_reg(uint8_t reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

static esp_err_t mpu6050_read_reg(uint8_t reg, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, &data[len - 1], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

esp_err_t mpu6050_init(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    
    // Wake up MPU6050
    ESP_ERROR_CHECK(mpu6050_write_reg(MPU6050_PWR_MGMT_1, 0x00));
    
    ESP_LOGI(TAG, "MPU6050 initialized successfully");
    return ESP_OK;
}

static esp_err_t mpu6050_read_raw(int16_t *accel_data, int16_t *gyro_data)
{
    uint8_t buffer[14];
    
    ESP_ERROR_CHECK(mpu6050_read_reg(MPU6050_ACCEL_XOUT_H, buffer, 14));
    
    // Convert raw data
    accel_data[0] = (int16_t)(buffer[0] << 8 | buffer[1]);  // X
    accel_data[1] = (int16_t)(buffer[2] << 8 | buffer[3]);  // Y
    accel_data[2] = (int16_t)(buffer[4] << 8 | buffer[5]);  // Z
    gyro_data[0] = (int16_t)(buffer[8] << 8 | buffer[9]);   // X
    gyro_data[1] = (int16_t)(buffer[10] << 8 | buffer[11]); // Y
    gyro_data[2] = (int16_t)(buffer[12] << 8 | buffer[13]); // Z
    
    return ESP_OK;
}

void mpu_task(void *pvParameters)
{
    int16_t accel_raw[3], gyro_raw[3];
    mpu_data_t mpu_data;
    
    while (1) {
        if (mpu6050_read_raw(accel_raw, gyro_raw) == ESP_OK) {
            // Convert to g and degrees/second
            mpu_data.accel_x = accel_raw[0] / 16384.0; // ±2g range
            mpu_data.accel_y = accel_raw[1] / 16384.0;
            mpu_data.accel_z = accel_raw[2] / 16384.0;
            
            mpu_data.gyro_x = gyro_raw[0] / 131.0; // ±250°/s range
            mpu_data.gyro_y = gyro_raw[1] / 131.0;
            mpu_data.gyro_z = gyro_raw[2] / 131.0;
            
            mpu_data.timestamp = esp_timer_get_time();
            
            ESP_LOGI(TAG, "Accel: X=%.2f, Y=%.2f, Z=%.2f | Gyro: X=%.2f, Y=%.2f, Z=%.2f",
                     mpu_data.accel_x, mpu_data.accel_y, mpu_data.accel_z,
                     mpu_data.gyro_x, mpu_data.gyro_y, mpu_data.gyro_z);
            
            if (xQueueSend(mpu_queue, &mpu_data, portMAX_DELAY) != pdTRUE) {
                ESP_LOGW(TAG, "Failed to send MPU data to queue");
            }
        } else {
            ESP_LOGE(TAG, "Failed to read MPU6050");
        }
        
        vTaskDelay(pdMS_TO_TICKS(100)); // Read every 100ms
    }
}