/**
 * @file ota_update.h
 * @brief OTA Update Component for Offline-Ready IoT Systems
 * 
 * This component implements over-the-air (OTA) update functionality with
 * offline capabilities for IoT devices. It handles firmware updates,
 * rollback mechanisms, and integration with the offline architecture.
 * 
 * Key Features:
 * - Secure OTA update with verification
 * - Rollback mechanism for failed updates
 * - Integration with system configuration for update settings
 * - Progress reporting and status monitoring
 * - Event-based architecture with callback support
 * - Robust error handling and recovery mechanisms
 * 
 * Offline Architecture Support:
 * - Defers updates when in offline mode
 * - Implements conflict resolution for update conflicts
 * - Supports staged updates for large firmware
 * - Maintains system stability during update process
 * 
 * @author Offline-Ready IoT System
 * @date 2025
 * @version 1.0
 */

#ifndef OTA_UPDATE_H
#define OTA_UPDATE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "system_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief OTA update status enumeration
 * 
 * Represents the current status of the OTA update process
 */
typedef enum {
    OTA_STATE_IDLE = 0,             /**< No update in progress */
    OTA_STATE_CHECKING,             /**< Checking for updates */
    OTA_STATE_DOWNLOADING,          /**< Downloading update */
    OTA_STATE_VERIFYING,            /**< Verifying update integrity */
    OTA_STATE_FLASHING,             /**< Flashing update to device */
    OTA_STATE_REBOOTING,            /**< Rebooting to apply update */
    OTA_STATE_ROLLBACK,             /**< Rolling back to previous version */
    OTA_STATE_OFFLINE_DEFERRED      /**< Update deferred due to offline mode */
} ota_state_t;

/**
 * @brief OTA update event types
 * 
 * Events that can be reported by the OTA update component
 */
typedef enum {
    OTA_EVENT_CHECK_STARTED = 0,    /**< Started checking for updates */
    OTA_EVENT_UPDATE_AVAILABLE,     /**< New update is available */
    OTA_EVENT_DOWNLOAD_STARTED,     /**< Started downloading update */
    OTA_EVENT_DOWNLOAD_PROGRESS,    /**< Download progress update */
    OTA_EVENT_DOWNLOAD_COMPLETE,    /**< Download completed successfully */
    OTA_EVENT_VERIFY_STARTED,       /**< Started verifying update */
    OTA_EVENT_VERIFY_SUCCESS,       /**< Update verification successful */
    OTA_EVENT_VERIFY_FAILED,        /**< Update verification failed */
    OTA_EVENT_FLASH_STARTED,        /**< Started flashing update */
    OTA_EVENT_FLASH_PROGRESS,       /**< Flash progress update */
    OTA_EVENT_FLASH_COMPLETE,       /**< Flashing completed successfully */
    OTA_EVENT_UPDATE_SUCCESS,       /**< Update completed successfully */
    OTA_EVENT_UPDATE_FAILED,        /**< Update failed */
    OTA_EVENT_ROLLBACK_STARTED,     /**< Started rollback process */
    OTA_EVENT_ROLLBACK_COMPLETE,    /**< Rollback completed successfully */
    OTA_EVENT_OFFLINE_DEFERRED,     /**< Update deferred due to offline mode */
    OTA_EVENT_OFFLINE_RESUMED       /**< Resumed deferred update */
} ota_event_t;

/**
 * @brief OTA update configuration structure
 * 
 * Contains configuration parameters for the OTA update component
 */
typedef struct {
    const char* update_url;         /**< URL for firmware updates */
    const char* cert_pem;           /**< Certificate for secure connection (optional) */
    uint32_t timeout_ms;            /**< Network timeout in milliseconds */
    uint32_t retry_interval_ms;     /**< Retry interval after failed attempt */
    uint8_t max_retry_count;        /**< Maximum retry attempts */
    bool auto_update;               /**< Automatically apply updates when available */
    size_t buffer_size;             /**< Buffer size for download operations */
    uint32_t check_interval_ms;     /**< Interval between update checks */
} ota_config_t;

/**
 * @brief OTA update progress structure
 * 
 * Contains progress information for ongoing OTA operations
 */
typedef struct {
    size_t total_size;              /**< Total size of update */
    size_t downloaded_size;         /**< Amount downloaded so far */
    size_t flashed_size;            /**< Amount flashed so far */
    uint8_t progress_percent;       /**< Progress percentage (0-100) */
    ota_state_t current_state;      /**< Current update state */
} ota_progress_t;

/**
 * @brief OTA update handle (opaque)
 * 
 * This handle represents an initialized OTA update instance
 */
typedef void* ota_handle_t;

/**
 * @brief OTA event callback function type
 * 
 * Function signature for OTA event callbacks
 * 
 * @param event The OTA event that occurred
 * @param progress Pointer to progress information (if applicable)
 * @param user_data User-provided data pointer
 */
typedef void (*ota_event_callback_t)(ota_event_t event, const ota_progress_t* progress, void* user_data);

/**
 * @brief Initialize OTA update component
 * 
 * Initializes the OTA update component with the provided configuration.
 * 
 * @param config Pointer to OTA update configuration
 * @param handle Pointer to store the created OTA update handle
 * @return ESP_OK on success, error code on failure
 */
esp_err_t ota_init(const ota_config_t* config, ota_handle_t* handle);

/**
 * @brief Deinitialize OTA update component
 * 
 * Cleans up all resources used by the OTA update component.
 * 
 * @param handle OTA update handle
 * @return ESP_OK on success, error code on failure
 */
esp_err_t ota_deinit(ota_handle_t handle);

/**
 * @brief Check for available updates
 * 
 * Checks if a new firmware update is available from the configured update server.
 * 
 * @param handle OTA update handle
 * @param blocking If true, function blocks until check completes
 * @return ESP_OK if check completed, error code on failure
 */
esp_err_t ota_check_for_updates(ota_handle_t handle, bool blocking);

/**
 * @brief Start firmware update
 * 
 * Starts the firmware update process by downloading and applying the update.
 * 
 * @param handle OTA update handle
 * @param blocking If true, function blocks until update completes
 * @return ESP_OK if update started successfully, error code on failure
 */
esp_err_t ota_start_update(ota_handle_t handle, bool blocking);

/**
 * @brief Cancel ongoing update
 * 
 * Cancels an ongoing update process if possible.
 * 
 * @param handle OTA update handle
 * @return ESP_OK on success, error code on failure
 */
esp_err_t ota_cancel_update(ota_handle_t handle);

/**
 * @brief Trigger system rollback
 * 
 * Triggers a rollback to the previous firmware version.
 * 
 * @param handle OTA update handle
 * @return ESP_OK on success, error code on failure
 */
esp_err_t ota_trigger_rollback(ota_handle_t handle);

/**
 * @brief Get current OTA state
 * 
 * Returns the current state of the OTA update component.
 * 
 * @param handle OTA update handle
 * @return Current OTA state
 */
ota_state_t ota_get_state(ota_handle_t handle);

/**
 * @brief Check if update is in progress
 * 
 * Convenience function to check if an update is currently in progress.
 * 
 * @param handle OTA update handle
 * @return true if update is in progress, false otherwise
 */
bool ota_is_update_in_progress(ota_handle_t handle);

/**
 * @brief Get update progress information
 * 
 * Retrieves progress information for the current update operation.
 * 
 * @param handle OTA update handle
 * @param progress Pointer to store progress information
 * @return ESP_OK on success, error code on failure
 */
esp_err_t ota_get_progress(ota_handle_t handle, ota_progress_t* progress);

/**
 * @brief Set deferred update flag
 * 
 * Sets or clears the deferred update flag for offline mode handling.
 * 
 * @param handle OTA update handle
 * @param deferred If true, defer updates; if false, allow updates
 * @return ESP_OK on success, error code on failure
 */
esp_err_t ota_set_deferred(ota_handle_t handle, bool deferred);

/**
 * @brief Check if update is deferred
 * 
 * Checks if updates are currently deferred due to offline mode.
 * 
 * @param handle OTA update handle
 * @param deferred Pointer to store deferred status
 * @return ESP_OK on success, error code on failure
 */
esp_err_t ota_is_deferred(ota_handle_t handle, bool* deferred);

/**
 * @brief Register OTA event callback
 * 
 * Registers a callback function to be notified of OTA events.
 * 
 * @param handle OTA update handle
 * @param callback Callback function to register
 * @param user_data User data to pass to callback
 * @return ESP_OK on success, error code on failure
 */
esp_err_t ota_register_callback(ota_handle_t handle, ota_event_callback_t callback, void* user_data);

/**
 * @brief Unregister OTA event callback
 * 
 * Unregisters a previously registered OTA event callback.
 * 
 * @param handle OTA update handle
 * @param callback Callback function to unregister
 * @return ESP_OK on success, error code on failure
 */
esp_err_t ota_unregister_callback(ota_handle_t handle, ota_event_callback_t callback);

/**
 * @brief Convert OTA event to string
 * 
 * Converts an OTA event to a human-readable string for logging and debugging.
 * 
 * @param event OTA event to convert
 * @return String representation of the event
 */
const char* ota_event_to_string(ota_event_t event);

/**
 * @brief Convert OTA state to string
 * 
 * Converts an OTA state to a human-readable string for logging and debugging.
 * 
 * @param state OTA state to convert
 * @return String representation of the state
 */
const char* ota_state_to_string(ota_state_t state);

#ifdef __cplusplus
}
#endif

#endif // OTA_UPDATE_H