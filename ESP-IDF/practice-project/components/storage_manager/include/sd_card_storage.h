/**
 * @file sd_card_storage.h
 * @brief SD Card Storage Manager Component Header
 * 
 * This component provides a complete SD card storage interface for the Smart Environment
 * Data Logger system. It handles SD card initialization, file operations, data logging,
 * and error management with professional-grade reliability.
 * 
 * Features:
 * - SD card initialization and detection with robust error handling
 * - FAT file system integration with automatic mounting
 * - Thread-safe file operations with mutex protection
 * - Data logging with timestamp and sensor data formatting
 * - Performance monitoring and statistics tracking
 * - Comprehensive error handling with recovery mechanisms
 * - Memory-efficient buffer management for large data writes
 * 
 * Hardware Requirements:
 * - MicroSD card slot with SPI interface
 * - Proper level shifting if needed (3.3V to 5V)
 * - Adequate power supply for SD card operation
 * 
 * @author Learning ESP-IDF
 * @date 2025
 * @version 1.0
 */

#ifndef SD_CARD_STORAGE_H
#define SD_CARD_STORAGE_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief SD card storage configuration structure
 * 
 * Contains all configuration parameters for SD card initialization and operation
 */
typedef struct {
    gpio_num_t cs_pin;           /**< Chip Select (CS) pin for SPI communication */
    gpio_num_t mosi_pin;         /**< Master Out Slave In (MOSI) pin for SPI */
    gpio_num_t miso_pin;         /**< Master In Slave Out (MISO) pin for SPI */
    gpio_num_t clk_pin;          /**< Clock (CLK) pin for SPI */
    spi_host_device_t spi_host;  /**< SPI host device (SPI2_HOST or SPI3_HOST) */
    int max_files;               /**< Maximum number of simultaneously open files */
    bool format_if_mount_failed; /**< Format SD card if mounting fails */
    uint32_t allocation_unit_size; /**< Allocation unit size for FATFS */
} sd_card_config_t;

/**
 * @brief SD card storage handle (opaque)
 * 
 * This handle represents an initialized SD card storage instance and is used
 * in all SD card operations.
 */
typedef void* sd_card_handle_t;

/**
 * @brief Sensor data log entry structure
 * 
 * Represents a single sensor data entry for logging to the SD card
 */
typedef struct {
    time_t timestamp;            /**< Unix timestamp of the reading */
    float temperature;           /**< Temperature in Celsius */
    float humidity;             /**< Relative humidity in % */
    float accel_x;              /**< Accelerometer X-axis in m/s² */
    float accel_y;              /**< Accelerometer Y-axis in m/s² */
    float accel_z;              /**< Accelerometer Z-axis in m/s² */
    float gyro_x;               /**< Gyroscope X-axis in °/s */
    float gyro_y;               /**< Gyroscope Y-axis in °/s */
    float gyro_z;               /**< Gyroscope Z-axis in °/s */
    float pressure;             /**< Atmospheric pressure in hPa (if available) */
    char location[32];          /**< Location identifier (GPS coordinates, room name, etc.) */
} sensor_log_entry_t;

/**
 * @brief SD card statistics structure
 * 
 * Contains performance and usage statistics for the SD card storage system
 */
typedef struct {
    uint32_t total_writes;       /**< Total number of write operations */
    uint32_t failed_writes;      /**< Number of failed write operations */
    uint32_t total_reads;        /**< Total number of read operations */
    uint32_t failed_reads;       /**< Number of failed read operations */
    uint32_t bytes_written;      /**< Total bytes written to SD card */
    uint32_t bytes_read;         /**< Total bytes read from SD card */
    uint32_t file_operations;    /**< Total file open/close operations */
    float success_rate;          /**< Overall success rate percentage */
    uint64_t last_operation_time; /**< Timestamp of last successful operation */
} sd_card_stats_t;

/**
 * @brief SD card storage error types
 * 
 * Extended error codes for SD card storage operations
 */
typedef enum {
    SD_CARD_OK = ESP_OK,                    /**< No error */
    SD_CARD_ERR_INVALID_ARG = ESP_ERR_INVALID_ARG, /**< Invalid argument */
    SD_CARD_ERR_NO_MEM = ESP_ERR_NO_MEM,    /**< Memory allocation failed */
    SD_CARD_ERR_TIMEOUT = ESP_ERR_TIMEOUT,  /**< Operation timeout */
    SD_CARD_ERR_INVALID_STATE = ESP_ERR_INVALID_STATE, /**< Invalid state */
    SD_CARD_ERR_NOT_FOUND = ESP_ERR_NOT_FOUND, /**< Resource not found */
    SD_CARD_ERR_CARD_NOT_PRESENT = 0x2000,   /**< SD card not detected */
    SD_CARD_ERR_MOUNT_FAILED = 0x2001,       /**< Failed to mount filesystem */
    SD_CARD_ERR_FORMAT_FAILED = 0x2002,      /**< Failed to format SD card */
    SD_CARD_ERR_FILE_OPERATION = 0x2003,     /**< File operation error */
    SD_CARD_ERR_SPI_COMMUNICATION = 0x2004,  /**< SPI communication error */
    SD_CARD_ERR_WRITE_PROTECTED = 0x2005,    /**< SD card is write protected */
    SD_CARD_ERR_CARD_ERROR = 0x2006          /**< General card error */
} sd_card_err_t;

// Function Declarations

/**
 * @brief Initialize SD card storage system
 * 
 * This function initializes the SD card storage system with the provided configuration.
 * It sets up SPI communication, detects the SD card, mounts the filesystem, and
 * prepares the system for file operations.
 * 
 * @param config Pointer to SD card configuration structure
 * @param handle Pointer to store the created storage handle
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t sd_card_init(const sd_card_config_t *config, sd_card_handle_t *handle);

/**
 * @brief Deinitialize SD card storage system
 * 
 * Releases all resources and safely unmounts the SD card filesystem.
 * 
 * @param handle SD card storage handle
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t sd_card_deinit(sd_card_handle_t handle);

/**
 * @brief Check if SD card is present and accessible
 * 
 * Performs a quick check to determine if the SD card is properly inserted
 * and accessible for read/write operations.
 * 
 * @param handle SD card storage handle
 * @return true if SD card is present and accessible, false otherwise
 */
bool sd_card_is_present(sd_card_handle_t handle);

/**
 * @brief Get SD card storage statistics
 * 
 * Retrieves performance statistics and usage information for the SD card system.
 * 
 * @param handle SD card storage handle
 * @param stats Pointer to store statistics data
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t sd_card_get_stats(sd_card_handle_t handle, sd_card_stats_t *stats);

/**
 * @brief Reset SD card storage statistics
 * 
 * Clears all accumulated statistics counters to zero.
 * 
 * @param handle SD card storage handle
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t sd_card_reset_stats(sd_card_handle_t handle);

/**
 * @brief Write sensor data to log file
 * 
 * Writes a sensor data entry to the specified log file with timestamp formatting.
 * Creates the file if it doesn't exist, and appends data to existing files.
 * 
 * @param handle SD card storage handle
 * @param filename Name of the log file to write to
 * @param data Pointer to sensor data entry to write
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t sd_card_write_sensor_log(sd_card_handle_t handle, const char *filename, const sensor_log_entry_t *data);

/**
 * @brief Read sensor data from log file
 * 
 * Reads sensor data entries from the specified log file. Supports reading
 * specific entries or ranges of entries.
 * 
 * @param handle SD card storage handle
 * @param filename Name of the log file to read from
 * @param entry_index Index of the entry to read (0-based)
 * @param data Pointer to store read sensor data
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t sd_card_read_sensor_log(sd_card_handle_t handle, const char *filename, uint32_t entry_index, sensor_log_entry_t *data);

/**
 * @brief Get total number of entries in log file
 * 
 * Returns the total number of sensor data entries in the specified log file.
 * 
 * @param handle SD card storage handle
 * @param filename Name of the log file to check
 * @param count Pointer to store entry count
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t sd_card_get_log_entry_count(sd_card_handle_t handle, const char *filename, uint32_t *count);

/**
 * @brief Create a new log file
 * 
 * Creates a new log file with the specified name and writes a header if needed.
 * 
 * @param handle SD card storage handle
 * @param filename Name of the log file to create
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t sd_card_create_log_file(sd_card_handle_t handle, const char *filename);

/**
 * @brief Delete a log file
 * 
 * Deletes the specified log file from the SD card.
 * 
 * @param handle SD card storage handle
 * @param filename Name of the log file to delete
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t sd_card_delete_log_file(sd_card_handle_t handle, const char *filename);

/**
 * @brief List all log files on SD card
 * 
 * Retrieves a list of all log files stored on the SD card.
 * 
 * @param handle SD card storage handle
 * @param file_list Buffer to store file names
 * @param max_files Maximum number of file names to retrieve
 * @param actual_files Pointer to store actual number of files found
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t sd_card_list_log_files(sd_card_handle_t handle, char **file_list, uint32_t max_files, uint32_t *actual_files);

/**
 * @brief Format SD card
 * 
 * Formats the SD card with FAT filesystem. This will erase all data on the card.
 * 
 * @param handle SD card storage handle
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t sd_card_format(sd_card_handle_t handle);

/**
 * @brief Get SD card information
 * 
 * Retrieves information about the SD card including capacity, free space, and filesystem type.
 * 
 * @param handle SD card storage handle
 * @param total_bytes Pointer to store total card capacity in bytes
 * @param free_bytes Pointer to store free space in bytes
 * @param fs_type Buffer to store filesystem type string
 * @param fs_type_len Length of filesystem type buffer
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t sd_card_get_info(sd_card_handle_t handle, uint64_t *total_bytes, uint64_t *free_bytes, char *fs_type, size_t fs_type_len);

/**
 * @brief Print SD card storage information
 * 
 * Displays comprehensive information about the SD card storage system including
 * configuration, statistics, and card information.
 * 
 * @param handle SD card storage handle
 * @param show_detailed If true, shows detailed debug information
 */
void sd_card_print_info(sd_card_handle_t handle, bool show_detailed);

/**
 * @brief Convert error code to string
 * 
 * Converts SD card storage error codes to human-readable strings.
 * 
 * @param error Error code to convert
 * @return const char* Error description string
 */
const char* sd_card_error_to_string(sd_card_err_t error);

#ifdef __cplusplus
}
#endif

#endif // SD_CARD_STORAGE_H