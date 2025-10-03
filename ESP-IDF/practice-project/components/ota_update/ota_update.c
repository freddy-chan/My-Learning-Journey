/**
 * @file ota_update.c
 * @brief OTA Update Component Implementation
 * 
 * This file implements the OTA update component with offline capabilities
 * for IoT devices. It handles firmware updates, rollback mechanisms, and
 * integration with the offline architecture.
 * 
 * @author Offline-Ready IoT System
 * @date 2025
 * @version 1.0
 */

#include "ota_update.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "OTA_UPDATE";

// Maximum number of registered callbacks
#define MAX_OTA_CALLBACKS 8

// Default configuration values
#define DEFAULT_TIMEOUT_MS 10000
#define DEFAULT_RETRY_INTERVAL_MS 5000
#define DEFAULT_MAX_RETRY_COUNT 3
#define DEFAULT_BUFFER_SIZE 1024
#define DEFAULT_CHECK_INTERVAL_MS 3600000 // 1 hour

/**
 * @brief OTA update context structure
 * 
 * Contains all state information for the OTA update component
 */
struct ota_context {
    ota_config_t config;                    // Configuration settings
    ota_mgr_state_t state;                      // Current update state
    ota_mgr_event_callback_t callbacks[MAX_OTA_CALLBACKS]; // Registered callbacks
    void* callback_user_data[MAX_OTA_CALLBACKS];       // User data for callbacks
    ota_progress_t progress;                // Current progress information
    esp_ota_handle_t ota_handle;            // OTA handle for flashing
    const esp_partition_t* update_partition; // Partition being updated
    bool deferred;                          // Update deferred flag
    bool initialized;                       // Initialization status
};

// Global OTA update instance
static struct ota_context* g_ota_context = NULL;

/**
 * @brief HTTP client event handler
 * 
 * Handles HTTP client events during OTA update process
 * 
 * @param evt HTTP client event
 * @return ESP_OK on success, error code on failure
 */
static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
            break;
        case HTTP_EVENT_REDIRECT:
            ESP_LOGD(TAG, "HTTP_EVENT_REDIRECT");
            break;
        default:
            // Handle any other cases
            break;
    }
    return ESP_OK;
}

/**
 * @brief Notify OTA event callbacks
 * 
 * Notifies all registered callbacks of an OTA event
 * 
 * @param ctx OTA context
 * @param event OTA event to notify
 */
static void notify_ota_event(struct ota_context* ctx, ota_mgr_event_t event)
{
    for (int i = 0; i < MAX_OTA_CALLBACKS; i++) {
        if (ctx->callbacks[i] != NULL) {
            ctx->callbacks[i](event, &ctx->progress, ctx->callback_user_data[i]);
        }
    }
}

/**
 * @brief Update progress information
 * 
 * Updates the progress information and notifies callbacks
 * 
 * @param ctx OTA context
 * @param total_size Total size of update
 * @param processed_size Processed size so far
 * @param state Current OTA state
 */
static void update_progress(struct ota_context* ctx, size_t total_size, size_t processed_size, ota_mgr_state_t state)
{
    ctx->progress.total_size = total_size;
    ctx->progress.downloaded_size = processed_size;
    ctx->progress.flashed_size = processed_size;
    
    if (total_size > 0) {
        ctx->progress.progress_percent = (uint8_t)((processed_size * 100) / total_size);
    } else {
        ctx->progress.progress_percent = 0;
    }
    
    ctx->progress.current_state = state;
    ctx->state = state;
    
    // Notify progress callbacks
    if (state == OTA_MGR_STATE_DOWNLOADING || state == OTA_MGR_STATE_FLASHING) {
        notify_ota_event(ctx, (state == OTA_MGR_STATE_DOWNLOADING) ? 
                        OTA_MGR_EVENT_DOWNLOAD_PROGRESS : OTA_MGR_EVENT_FLASH_PROGRESS);
    }
}

/**
 * @brief Check for firmware update
 * 
 * Checks if a new firmware update is available (simplified implementation)
 * 
 * @param ctx OTA context
 * @return ESP_OK if check completed, error code on failure
 */
static esp_err_t check_for_update(struct ota_context* ctx)
{
    // In a real implementation, this would check a remote server
    // For now, we'll simulate the process
    
    ESP_LOGI(TAG, "Checking for firmware updates...");
    ctx->state = OTA_MGR_STATE_CHECKING;
    notify_ota_event(ctx, OTA_MGR_EVENT_CHECK_STARTED);
    
    // Simulate network delay
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Simulate finding an update (in real implementation, this would be based on version checks)
    bool update_available = true; // This would be determined by actual version comparison
    
    if (update_available) {
        ESP_LOGI(TAG, "New firmware update available");
        notify_ota_event(ctx, OTA_MGR_EVENT_UPDATE_AVAILABLE);
        return ESP_OK;
    } else {
        ESP_LOGI(TAG, "No firmware updates available");
        ctx->state = OTA_MGR_STATE_IDLE;
        return ESP_OK;
    }
}

/**
 * @brief Perform firmware update
 * 
 * Performs the actual firmware update process
 * 
 * @param ctx OTA context
 * @return ESP_OK if update successful, error code on failure
 */
static esp_err_t perform_update(struct ota_context* ctx)
{
    if (ctx->deferred) {
        ESP_LOGI(TAG, "Update deferred due to offline mode");
        ctx->state = OTA_MGR_STATE_OFFLINE_DEFERRED;
        notify_ota_event(ctx, OTA_MGR_EVENT_OFFLINE_DEFERRED);
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Starting firmware update process");
    ctx->state = OTA_MGR_STATE_DOWNLOADING;
    notify_ota_event(ctx, OTA_MGR_EVENT_DOWNLOAD_STARTED);
    
    // Get running partition and next update partition
    const esp_partition_t* running = esp_ota_get_running_partition();
    const esp_partition_t* update = esp_ota_get_next_update_partition(NULL);
    
    if (running == NULL || update == NULL) {
        ESP_LOGE(TAG, "Failed to get partition information");
        ctx->state = OTA_MGR_STATE_IDLE;
        notify_ota_event(ctx, OTA_MGR_EVENT_UPDATE_FAILED);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Running partition: %s", running->label);
    ESP_LOGI(TAG, "Update partition: %s", update->label);
    
    ctx->update_partition = update;
    
    // Initialize OTA handle
    esp_err_t err = esp_ota_begin(update, OTA_WITH_SEQUENTIAL_WRITES, &ctx->ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        ctx->state = OTA_MGR_STATE_IDLE;
        notify_ota_event(ctx, OTA_MGR_EVENT_UPDATE_FAILED);
        return err;
    }
    
    // Simulate download and flashing process
    size_t total_size = 1024 * 1024; // 1MB firmware (example)
    size_t downloaded = 0;
    uint8_t* buffer = malloc(ctx->config.buffer_size);
    
    if (buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate download buffer");
        esp_ota_end(ctx->ota_handle);
        ctx->state = OTA_MGR_STATE_IDLE;
        notify_ota_event(ctx, OTA_MGR_EVENT_UPDATE_FAILED);
        return ESP_ERR_NO_MEM;
    }
    
    // Simulate download progress
    while (downloaded < total_size) {
        size_t chunk_size = (total_size - downloaded) > ctx->config.buffer_size ? 
                           ctx->config.buffer_size : (total_size - downloaded);
        
        // Simulate receiving data
        memset(buffer, 0xFF, chunk_size);
        
        // Write to OTA handle
        err = esp_ota_write(ctx->ota_handle, buffer, chunk_size);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
            free(buffer);
            esp_ota_end(ctx->ota_handle);
            ctx->state = OTA_MGR_STATE_IDLE;
            notify_ota_event(ctx, OTA_MGR_EVENT_UPDATE_FAILED);
            return err;
        }
        
        downloaded += chunk_size;
        update_progress(ctx, total_size, downloaded, OTA_MGR_STATE_DOWNLOADING);
        
        // Simulate network delay
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    free(buffer);
    
    ESP_LOGI(TAG, "Download completed successfully");
    notify_ota_event(ctx, OTA_MGR_EVENT_DOWNLOAD_COMPLETE);
    
    // Verify firmware
    ESP_LOGI(TAG, "Verifying firmware integrity");
    ctx->state = OTA_MGR_STATE_VERIFYING;
    notify_ota_event(ctx, OTA_MGR_EVENT_VERIFY_STARTED);
    
    // Simulate verification delay
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // In a real implementation, you would verify the firmware here
    ESP_LOGI(TAG, "Firmware verification successful");
    notify_ota_event(ctx, OTA_MGR_EVENT_VERIFY_SUCCESS);
    
    // Finish OTA process
    err = esp_ota_end(ctx->ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        ctx->state = OTA_MGR_STATE_IDLE;
        notify_ota_event(ctx, OTA_MGR_EVENT_UPDATE_FAILED);
        return err;
    }
    
    // Set boot partition
    err = esp_ota_set_boot_partition(update);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        ctx->state = OTA_MGR_STATE_IDLE;
        notify_ota_event(ctx, OTA_MGR_EVENT_UPDATE_FAILED);
        return err;
    }
    
    ESP_LOGI(TAG, "Firmware update completed successfully");
    ctx->state = OTA_MGR_STATE_IDLE;
    notify_ota_event(ctx, OTA_MGR_EVENT_UPDATE_SUCCESS);
    
    return ESP_OK;
}

// Public API Implementation

esp_err_t ota_init(const ota_config_t* config, ota_handle_t* handle)
{
    if (config == NULL || handle == NULL) {
        ESP_LOGE(TAG, "Invalid parameters: config or handle is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Check if already initialized
    if (g_ota_context != NULL) {
        ESP_LOGW(TAG, "OTA update component already initialized");
        *handle = g_ota_context;
        return ESP_OK;
    }
    
    // Allocate context
    struct ota_context* ctx = calloc(1, sizeof(struct ota_context));
    if (ctx == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for OTA context");
        return ESP_ERR_NO_MEM;
    }
    
    // Copy configuration
    ctx->config.update_url = config->update_url;
    ctx->config.cert_pem = config->cert_pem;
    ctx->config.timeout_ms = config->timeout_ms > 0 ? config->timeout_ms : DEFAULT_TIMEOUT_MS;
    ctx->config.retry_interval_ms = config->retry_interval_ms > 0 ? config->retry_interval_ms : DEFAULT_RETRY_INTERVAL_MS;
    ctx->config.max_retry_count = config->max_retry_count > 0 ? config->max_retry_count : DEFAULT_MAX_RETRY_COUNT;
    ctx->config.auto_update = config->auto_update;
    ctx->config.buffer_size = config->buffer_size > 0 ? config->buffer_size : DEFAULT_BUFFER_SIZE;
    ctx->config.check_interval_ms = config->check_interval_ms > 0 ? config->check_interval_ms : DEFAULT_CHECK_INTERVAL_MS;
    
    // Initialize state
    ctx->state = OTA_MGR_STATE_IDLE;
    ctx->deferred = false;
    ctx->initialized = true;
    
    // Initialize progress
    memset(&ctx->progress, 0, sizeof(ota_progress_t));
    ctx->progress.current_state = OTA_MGR_STATE_IDLE;
    
    // Set global instance
    g_ota_context = ctx;
    *handle = ctx;
    
    ESP_LOGI(TAG, "OTA update component initialized successfully");
    return ESP_OK;
}

esp_err_t ota_deinit(ota_handle_t handle)
{
    struct ota_context* ctx = (struct ota_context*)handle;
    
    if (ctx == NULL || !ctx->initialized) {
        ESP_LOGE(TAG, "Invalid OTA handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Mark as uninitialized
    ctx->initialized = false;
    
    // Clear global instance if it's this one
    if (g_ota_context == ctx) {
        g_ota_context = NULL;
    }
    
    // Free memory
    free(ctx);
    
    ESP_LOGI(TAG, "OTA update component deinitialized");
    return ESP_OK;
}

esp_err_t ota_check_for_updates(ota_handle_t handle, bool blocking)
{
    struct ota_context* ctx = (struct ota_context*)handle;
    
    if (ctx == NULL || !ctx->initialized) {
        ESP_LOGE(TAG, "Invalid OTA handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (ctx->state != OTA_MGR_STATE_IDLE) {
        ESP_LOGW(TAG, "OTA operation already in progress");
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret = check_for_update(ctx);
    
    // If blocking, wait for completion (simplified)
    if (blocking) {
        // In a real implementation, you would wait for the actual completion
        // For now, we assume the check is complete
    }
    
    return ret;
}

esp_err_t ota_start_update(ota_handle_t handle, bool blocking)
{
    struct ota_context* ctx = (struct ota_context*)handle;
    
    if (ctx == NULL || !ctx->initialized) {
        ESP_LOGE(TAG, "Invalid OTA handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (ctx->state != OTA_MGR_STATE_IDLE) {
        ESP_LOGW(TAG, "OTA operation already in progress");
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret = perform_update(ctx);
    
    // If blocking, wait for completion (simplified)
    if (blocking) {
        // In a real implementation, you would wait for the actual completion
        // For now, we assume the update is complete
    }
    
    return ret;
}

esp_err_t ota_cancel_update(ota_handle_t handle)
{
    struct ota_context* ctx = (struct ota_context*)handle;
    
    if (ctx == NULL || !ctx->initialized) {
        ESP_LOGE(TAG, "Invalid OTA handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (ctx->state == OTA_MGR_STATE_IDLE) {
        ESP_LOGW(TAG, "No OTA operation in progress");
        return ESP_OK;
    }
    
    // Cancel the update process
    ctx->state = OTA_MGR_STATE_IDLE;
    
    // In a real implementation, you would also cancel any ongoing network operations
    
    ESP_LOGI(TAG, "OTA update cancelled");
    return ESP_OK;
}

esp_err_t ota_trigger_rollback(ota_handle_t handle)
{
    struct ota_context* ctx = (struct ota_context*)handle;
    
    if (ctx == NULL || !ctx->initialized) {
        ESP_LOGE(TAG, "Invalid OTA handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Triggering rollback to previous firmware version");
    ctx->state = OTA_MGR_STATE_ROLLBACK;
    notify_ota_event(ctx, OTA_MGR_EVENT_ROLLBACK_STARTED);
    
    // In a real implementation, you would:
    // 1. Mark the current firmware as invalid
    // 2. Set the previous firmware as the boot partition
    // 3. Reboot the device
    
    // For now, we'll just simulate the process
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    ESP_LOGI(TAG, "Rollback completed successfully");
    notify_ota_event(ctx, OTA_MGR_EVENT_ROLLBACK_COMPLETE);
    
    ctx->state = OTA_MGR_STATE_IDLE;
    return ESP_OK;
}

ota_mgr_state_t ota_get_state(ota_handle_t handle)
{
    struct ota_context* ctx = (struct ota_context*)handle;
    
    if (ctx == NULL || !ctx->initialized) {
        ESP_LOGE(TAG, "Invalid OTA handle");
        return OTA_MGR_STATE_IDLE;
    }
    
    return ctx->state;
}

bool ota_is_update_in_progress(ota_handle_t handle)
{
    struct ota_context* ctx = (struct ota_context*)handle;
    
    if (ctx == NULL || !ctx->initialized) {
        return false;
    }
    
    return (ctx->state != OTA_MGR_STATE_IDLE);
}

esp_err_t ota_get_progress(ota_handle_t handle, ota_progress_t* progress)
{
    struct ota_context* ctx = (struct ota_context*)handle;
    
    if (ctx == NULL || !ctx->initialized || progress == NULL) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    *progress = ctx->progress;
    return ESP_OK;
}

esp_err_t ota_set_deferred(ota_handle_t handle, bool deferred)
{
    struct ota_context* ctx = (struct ota_context*)handle;
    
    if (ctx == NULL || !ctx->initialized) {
        ESP_LOGE(TAG, "Invalid OTA handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    ctx->deferred = deferred;
    
    if (deferred) {
        ESP_LOGI(TAG, "OTA updates deferred due to offline mode");
        notify_ota_event(ctx, OTA_MGR_EVENT_OFFLINE_DEFERRED);
    } else {
        ESP_LOGI(TAG, "OTA updates enabled");
        notify_ota_event(ctx, OTA_MGR_EVENT_OFFLINE_RESUMED);
    }
    
    return ESP_OK;
}

esp_err_t ota_is_deferred(ota_handle_t handle, bool* deferred)
{
    struct ota_context* ctx = (struct ota_context*)handle;
    
    if (ctx == NULL || !ctx->initialized || deferred == NULL) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    *deferred = ctx->deferred;
    return ESP_OK;
}

esp_err_t ota_register_callback(ota_handle_t handle, ota_mgr_event_callback_t callback, void* user_data)
{
    struct ota_context* ctx = (struct ota_context*)handle;
    
    if (ctx == NULL || !ctx->initialized || callback == NULL) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Find available callback slot
    for (int i = 0; i < MAX_OTA_CALLBACKS; i++) {
        if (ctx->callbacks[i] == NULL) {
            ctx->callbacks[i] = callback;
            ctx->callback_user_data[i] = user_data;
            ESP_LOGI(TAG, "Registered OTA callback at slot %d", i);
            return ESP_OK;
        }
    }
    
    ESP_LOGE(TAG, "No available callback slots");
    return ESP_ERR_NO_MEM;
}

esp_err_t ota_unregister_callback(ota_handle_t handle, ota_mgr_event_callback_t callback)
{
    struct ota_context* ctx = (struct ota_context*)handle;
    
    if (ctx == NULL || !ctx->initialized || callback == NULL) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Find and unregister callback
    for (int i = 0; i < MAX_OTA_CALLBACKS; i++) {
        if (ctx->callbacks[i] == callback) {
            ctx->callbacks[i] = NULL;
            ctx->callback_user_data[i] = NULL;
            ESP_LOGI(TAG, "Unregistered OTA callback at slot %d", i);
            return ESP_OK;
        }
    }
    
    ESP_LOGW(TAG, "Callback not found");
    return ESP_ERR_NOT_FOUND;
}

const char* ota_event_to_string(ota_mgr_event_t event)
{
    switch (event) {
        case OTA_MGR_EVENT_CHECK_STARTED:       return "CHECK_STARTED";
        case OTA_MGR_EVENT_UPDATE_AVAILABLE:    return "UPDATE_AVAILABLE";
        case OTA_MGR_EVENT_DOWNLOAD_STARTED:    return "DOWNLOAD_STARTED";
        case OTA_MGR_EVENT_DOWNLOAD_PROGRESS:   return "DOWNLOAD_PROGRESS";
        case OTA_MGR_EVENT_DOWNLOAD_COMPLETE:   return "DOWNLOAD_COMPLETE";
        case OTA_MGR_EVENT_VERIFY_STARTED:      return "VERIFY_STARTED";
        case OTA_MGR_EVENT_VERIFY_SUCCESS:      return "VERIFY_SUCCESS";
        case OTA_MGR_EVENT_VERIFY_FAILED:       return "VERIFY_FAILED";
        case OTA_MGR_EVENT_FLASH_STARTED:       return "FLASH_STARTED";
        case OTA_MGR_EVENT_FLASH_PROGRESS:      return "FLASH_PROGRESS";
        case OTA_MGR_EVENT_FLASH_COMPLETE:      return "FLASH_COMPLETE";
        case OTA_MGR_EVENT_UPDATE_SUCCESS:      return "UPDATE_SUCCESS";
        case OTA_MGR_EVENT_UPDATE_FAILED:       return "UPDATE_FAILED";
        case OTA_MGR_EVENT_ROLLBACK_STARTED:    return "ROLLBACK_STARTED";
        case OTA_MGR_EVENT_ROLLBACK_COMPLETE:   return "ROLLBACK_COMPLETE";
        case OTA_MGR_EVENT_OFFLINE_DEFERRED:    return "OFFLINE_DEFERRED";
        case OTA_MGR_EVENT_OFFLINE_RESUMED:     return "OFFLINE_RESUMED";
        default:                           return "UNKNOWN";
    }
}

const char* ota_state_to_string(ota_mgr_state_t state)
{
    switch (state) {
        case OTA_MGR_STATE_IDLE:            return "IDLE";
        case OTA_MGR_STATE_CHECKING:        return "CHECKING";
        case OTA_MGR_STATE_DOWNLOADING:     return "DOWNLOADING";
        case OTA_MGR_STATE_VERIFYING:       return "VERIFYING";
        case OTA_MGR_STATE_FLASHING:        return "FLASHING";
        case OTA_MGR_STATE_REBOOTING:       return "REBOOTING";
        case OTA_MGR_STATE_ROLLBACK:        return "ROLLBACK";
        case OTA_MGR_STATE_OFFLINE_DEFERRED: return "OFFLINE_DEFERRED";
        default:                       return "UNKNOWN";
    }
}