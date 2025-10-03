# SD Card Storage Integration Guide

## Overview

This guide documents the implementation of SD card storage functionality for the Smart Environment Data Logger project as part of Week 2 Phase 1 (Days 15-17). The SD card storage component provides persistent data logging capabilities for sensor readings.

## Component Structure

The SD card storage component is implemented in the `storage_manager` directory with the following structure:

```
components/storage_manager/
├── CMakeLists.txt
├── include/
│   └── sd_card_storage.h
└── sd_card_storage.c
```

## Key Features

1. **SPI Interface**: Uses ESP-IDF's SPI master driver for communication with SD cards
2. **FAT Filesystem**: Integrates with ESP-IDF's FAT filesystem support
3. **Thread Safety**: Implements mutex protection for concurrent access
4. **Data Logging**: Provides structured sensor data logging in CSV format
5. **Error Handling**: Comprehensive error handling with detailed error codes
6. **Statistics Tracking**: Performance monitoring and usage statistics
7. **Configuration Management**: Flexible configuration through system_config integration

## API Overview

### Initialization and Configuration

```c
// SD card configuration structure
typedef struct {
    gpio_num_t cs_pin;           // Chip Select pin
    gpio_num_t mosi_pin;         // Master Out Slave In pin
    gpio_num_t miso_pin;         // Master In Slave Out pin
    gpio_num_t clk_pin;          // Clock pin
    spi_host_device_t spi_host;  // SPI host device
    int max_files;               // Maximum open files
    bool format_if_mount_failed; // Format if mounting fails
    uint32_t allocation_unit_size; // Allocation unit size
} sd_card_config_t;

// Initialize SD card storage
esp_err_t sd_card_init(const sd_card_config_t *config, sd_card_handle_t *handle);
```

### Data Logging

```c
// Sensor data log entry structure
typedef struct {
    time_t timestamp;            // Unix timestamp
    float temperature;           // Temperature in Celsius
    float humidity;             // Relative humidity in %
    float accel_x, accel_y, accel_z; // Accelerometer data
    float gyro_x, gyro_y, gyro_z;    // Gyroscope data
    float pressure;             // Atmospheric pressure
    char location[32];          // Location identifier
} sensor_log_entry_t;

// Write sensor data to log file
esp_err_t sd_card_write_sensor_log(sd_card_handle_t handle, const char *filename, const sensor_log_entry_t *data);
```

### Management Functions

```c
// Create a new log file
esp_err_t sd_card_create_log_file(sd_card_handle_t handle, const char *filename);

// Get storage statistics
esp_err_t sd_card_get_stats(sd_card_handle_t handle, sd_card_stats_t *stats);

// Deinitialize SD card storage
esp_err_t sd_card_deinit(sd_card_handle_t handle);
```

## Integration with Main Application

The SD card storage component is integrated into the main application with the following key changes:

1. **Component Handle**: Added `g_sd_card_handle` to track the SD card instance
2. **Configuration**: Uses GPIO pin assignments from system_config
3. **Initialization**: Added SD card initialization after sensor initialization
4. **Data Logging**: Modified sensor reading function to log data to SD card
5. **Cleanup**: Added SD card deinitialization in cleanup section

## Configuration Integration

The SD card storage component uses GPIO pin assignments from the system configuration:

- CS pin: `config->gpio.sd_cs_pin`
- MOSI pin: `config->gpio.sd_mosi_pin`
- MISO pin: `config->gpio.sd_miso_pin`
- CLK pin: `config->gpio.sd_clk_pin`

## Error Handling

The component provides detailed error codes for troubleshooting:

- `SD_CARD_OK`: No error
- `SD_CARD_ERR_CARD_NOT_PRESENT`: SD card not detected
- `SD_CARD_ERR_MOUNT_FAILED`: Failed to mount filesystem
- `SD_CARD_ERR_FILE_OPERATION`: File operation error
- `SD_CARD_ERR_SPI_COMMUNICATION`: SPI communication error

## Usage Example

```c
// Configure SD card
sd_card_config_t sd_card_config = {
    .cs_pin = GPIO_NUM_5,
    .mosi_pin = GPIO_NUM_23,
    .miso_pin = GPIO_NUM_19,
    .clk_pin = GPIO_NUM_18,
    .spi_host = SPI2_HOST,
    .max_files = 5,
    .format_if_mount_failed = false,
    .allocation_unit_size = 16 * 1024
};

// Initialize SD card
sd_card_handle_t sd_card_handle;
esp_err_t ret = sd_card_init(&sd_card_config, &sd_card_handle);

// Create log file
sd_card_create_log_file(sd_card_handle, "sensor_data.csv");

// Log sensor data
sensor_log_entry_t log_entry = {
    .timestamp = time(NULL),
    .temperature = 25.0,
    .humidity = 60.0,
    // ... other sensor data
};
sd_card_write_sensor_log(sd_card_handle, "sensor_data.csv", &log_entry);
```

## Future Enhancements

1. **Data Retrieval**: Implement functions to read logged data
2. **File Management**: Add support for listing and deleting log files
3. **Data Analysis**: Implement basic data analysis functions
4. **Compression**: Add data compression for efficient storage
5. **Backup**: Implement automatic backup to cloud storage

## Testing

The SD card storage component should be tested with:

1. SD card insertion/removal detection
2. File creation and writing operations
3. Error handling under various failure conditions
4. Performance testing with large data sets
5. Concurrent access testing with multiple tasks

## Dependencies

- ESP-IDF FAT filesystem support
- SPI master driver
- FreeRTOS for mutex implementation
- Standard C library for file operations

This completes the implementation of Week 2 Phase 1: Storage & Communication (Days 15-17) for the Smart Environment Data Logger project.