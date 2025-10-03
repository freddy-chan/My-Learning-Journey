/**
 * @file wifi_manager.c
 * @brief WiFi Manager Component Implementation
 * 
 * This file implements the WiFi manager component with offline capabilities
 * for IoT devices. It handles connection management, offline mode transitions,
 * and provides status information to other components in the system.
 * 
 * @author Offline-Ready IoT System
 * @date 2025
 * @version 1.0
 */

#include "wifi_manager.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

static const char *TAG = "WIFI_MGR";

// WiFi event bits
#define WIFI_MGR_CONNECTED_BIT BIT0
#define WIFI_MGR_FAIL_BIT      BIT1
#define WIFI_MGR_DISCONNECTED_BIT BIT2

// Maximum number of registered callbacks
#define MAX_WIFI_CALLBACKS 8

/**
 * @brief WiFi manager context structure
 * 
 * Contains all state information for the WiFi manager
 */
struct wifi_manager_context {
    wifi_manager_config_t config;              // Configuration settings
    wifi_mgr_connection_state_t state;         // Current connection state
    esp_netif_t *netif;                        // Network interface
    EventGroupHandle_t wifi_event_group;       // Event group for synchronization
    wifi_mgr_event_callback_t callbacks[MAX_WIFI_CALLBACKS]; // Registered callbacks
    void* callback_user_data[MAX_WIFI_CALLBACKS];        // User data for callbacks
    uint8_t retry_count;                       // Current retry count
    bool offline_mode;                         // Offline mode flag
    bool initialized;                          // Initialization status
};

// Global WiFi manager instance
static struct wifi_manager_context* g_wifi_manager = NULL;

/**
 * @brief WiFi event handler
 * 
 * Handles WiFi events from the ESP-IDF event loop
 * 
 * @param arg User argument (WiFi manager context)
 * @param event_base Event base
 * @param event_id Event ID
 * @param event_data Event data
 */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data)
{
    struct wifi_manager_context* ctx = (struct wifi_manager_context*)arg;
    
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                ESP_LOGI(TAG, "WiFi station started");
                if (!ctx->offline_mode) {
                    esp_wifi_connect();
                }
                break;
                
            case WIFI_EVENT_STA_DISCONNECTED:
                ESP_LOGI(TAG, "Disconnected from WiFi");
                ctx->state = WIFI_MGR_STATE_DISCONNECTED;
                
                // Notify callbacks
                for (int i = 0; i < MAX_WIFI_CALLBACKS; i++) {
                    if (ctx->callbacks[i] != NULL) {
                        ctx->callbacks[i](WIFI_MGR_EVENT_DISCONNECTED, ctx->callback_user_data[i]);
                    }
                }
                
                // Attempt reconnection if enabled and not in offline mode
                if (ctx->config.auto_reconnect && !ctx->offline_mode) {
                    if (ctx->retry_count < ctx->config.max_retry_count) {
                        ctx->retry_count++;
                        ctx->state = WIFI_MGR_STATE_CONNECTING;
                        ESP_LOGI(TAG, "Attempting to reconnect (attempt %d/%d)", 
                                ctx->retry_count, ctx->config.max_retry_count);
                        vTaskDelay(pdMS_TO_TICKS(ctx->config.retry_interval_ms));
                        esp_wifi_connect();
                    } else {
                        ESP_LOGE(TAG, "Failed to connect after %d attempts", ctx->config.max_retry_count);
                        xEventGroupSetBits(ctx->wifi_event_group, WIFI_MGR_FAIL_BIT);
                    }
                } else {
                    xEventGroupSetBits(ctx->wifi_event_group, WIFI_MGR_DISCONNECTED_BIT);
                }
                break;
                
            default:
                break;
        }
    } else if (event_base == IP_EVENT) {
        switch (event_id) {
            case IP_EVENT_STA_GOT_IP:
                ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
                ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
                ctx->retry_count = 0;
                ctx->state = WIFI_MGR_STATE_CONNECTED;
                xEventGroupSetBits(ctx->wifi_event_group, WIFI_MGR_CONNECTED_BIT);
                
                // Notify callbacks
                for (int i = 0; i < MAX_WIFI_CALLBACKS; i++) {
                    if (ctx->callbacks[i] != NULL) {
                        ctx->callbacks[i](WIFI_MGR_EVENT_GOT_IP, ctx->callback_user_data[i]);
                        ctx->callbacks[i](WIFI_MGR_EVENT_CONNECTED, ctx->callback_user_data[i]);
                    }
                }
                break;
                
            default:
                break;
        }
    }
}

/**
 * @brief Initialize WiFi subsystem
 * 
 * Initializes the ESP-IDF WiFi subsystem and network interface
 * 
 * @param ctx WiFi manager context
 * @return ESP_OK on success, error code on failure
 */
static esp_err_t init_wifi_subsystem(struct wifi_manager_context* ctx)
{
    // Initialize TCP/IP network interface
    ESP_ERROR_CHECK(esp_netif_init());
    
    // Create default event loop
    esp_err_t ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to create default event loop: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize WiFi: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Set WiFi storage to RAM
    ret = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi storage: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Create default WiFi station AFTER WiFi is initialized
    ctx->netif = esp_netif_create_default_wifi_sta();
    if (ctx->netif == NULL) {
        ESP_LOGE(TAG, "Failed to create default WiFi station");
        return ESP_FAIL;
    }
    
    // Register event handlers
    ret = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, ctx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register WiFi event handler: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, ctx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register IP event handler: %s", esp_err_to_name(ret));
        return ret;
    }
    
    return ESP_OK;
}

/**
 * @brief Configure WiFi station
 * 
 * Configures the WiFi station with the provided SSID and password
 * 
 * @param ctx WiFi manager context
 * @return ESP_OK on success, error code on failure
 */
static esp_err_t configure_wifi_station(struct wifi_manager_context* ctx)
{
    // Set WiFi mode to station
    esp_err_t ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi mode: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure WiFi station
    wifi_config_t wifi_config = {0};
    strncpy((char*)wifi_config.sta.ssid, ctx->config.ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char*)wifi_config.sta.password, ctx->config.password, sizeof(wifi_config.sta.password) - 1);
    
    ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi config: %s", esp_err_to_name(ret));
        return ret;
    }
    
    return ESP_OK;
}

// Public API Implementation

esp_err_t wifi_manager_init(const wifi_manager_config_t* config, wifi_manager_handle_t* handle)
{
    if (config == NULL || handle == NULL) {
        ESP_LOGE(TAG, "Invalid parameters: config or handle is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Check if already initialized
    if (g_wifi_manager != NULL) {
        ESP_LOGW(TAG, "WiFi manager already initialized");
        *handle = g_wifi_manager;
        return ESP_OK;
    }
    
    // Allocate context
    struct wifi_manager_context* ctx = calloc(1, sizeof(struct wifi_manager_context));
    if (ctx == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for WiFi manager context");
        return ESP_ERR_NO_MEM;
    }
    
    // Copy configuration
    ctx->config.ssid = config->ssid;
    ctx->config.password = config->password;
    ctx->config.connect_timeout_ms = config->connect_timeout_ms > 0 ? config->connect_timeout_ms : 10000;
    ctx->config.retry_interval_ms = config->retry_interval_ms > 0 ? config->retry_interval_ms : 5000;
    ctx->config.max_retry_count = config->max_retry_count > 0 ? config->max_retry_count : 3;
    ctx->config.auto_reconnect = config->auto_reconnect;
    
    // Initialize state
    ctx->state = WIFI_MGR_STATE_DISCONNECTED;
    ctx->offline_mode = false;
    ctx->retry_count = 0;
    ctx->initialized = true;
    
    // Create event group
    ctx->wifi_event_group = xEventGroupCreate();
    if (ctx->wifi_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create event group");
        free(ctx);
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize WiFi subsystem
    esp_err_t ret = init_wifi_subsystem(ctx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize WiFi subsystem");
        vEventGroupDelete(ctx->wifi_event_group);
        free(ctx);
        return ret;
    }
    
    // Configure WiFi station
    ret = configure_wifi_station(ctx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure WiFi station");
        vEventGroupDelete(ctx->wifi_event_group);
        free(ctx);
        return ret;
    }
    
    // Start WiFi
    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start WiFi: %s", esp_err_to_name(ret));
        vEventGroupDelete(ctx->wifi_event_group);
        free(ctx);
        return ret;
    }
    
    // Set global instance
    g_wifi_manager = ctx;
    *handle = ctx;
    
    ESP_LOGI(TAG, "WiFi manager initialized successfully");
    return ESP_OK;
}

esp_err_t wifi_manager_deinit(wifi_manager_handle_t handle)
{
    struct wifi_manager_context* ctx = (struct wifi_manager_context*)handle;
    
    if (ctx == NULL || !ctx->initialized) {
        ESP_LOGE(TAG, "Invalid WiFi manager handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Disconnect if connected
    if (ctx->state == WIFI_MGR_STATE_CONNECTED) {
        esp_wifi_disconnect();
    }
    
    // Stop WiFi
    esp_wifi_stop();
    
    // Deinitialize WiFi
    esp_wifi_deinit();
    
    // Clean up event group
    if (ctx->wifi_event_group) {
        vEventGroupDelete(ctx->wifi_event_group);
    }
    
    // Mark as uninitialized
    ctx->initialized = false;
    
    // Clear global instance if it's this one
    if (g_wifi_manager == ctx) {
        g_wifi_manager = NULL;
    }
    
    // Free memory
    free(ctx);
    
    ESP_LOGI(TAG, "WiFi manager deinitialized");
    return ESP_OK;
}

esp_err_t wifi_manager_connect(wifi_manager_handle_t handle, bool blocking)
{
    struct wifi_manager_context* ctx = (struct wifi_manager_context*)handle;
    
    if (ctx == NULL || !ctx->initialized) {
        ESP_LOGE(TAG, "Invalid WiFi manager handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Cannot connect in offline mode
    if (ctx->offline_mode) {
        ESP_LOGW(TAG, "Cannot connect while in offline mode");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Reset event group bits
    xEventGroupClearBits(ctx->wifi_event_group, WIFI_MGR_CONNECTED_BIT | WIFI_MGR_FAIL_BIT | WIFI_MGR_DISCONNECTED_BIT);
    
    // Set state to connecting
    ctx->state = WIFI_MGR_STATE_CONNECTING;
    ctx->retry_count = 0;
    
    // Start connection
    esp_err_t ret = esp_wifi_connect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initiate WiFi connection: %s", esp_err_to_name(ret));
        ctx->state = WIFI_MGR_STATE_DISCONNECTED;
        return ret;
    }
    
    // Wait for connection result if blocking
    if (blocking) {
        EventBits_t bits = xEventGroupWaitBits(ctx->wifi_event_group,
                                              WIFI_MGR_CONNECTED_BIT | WIFI_MGR_FAIL_BIT,
                                              pdFALSE,
                                              pdFALSE,
                                              pdMS_TO_TICKS(ctx->config.connect_timeout_ms));
        
        if (bits & WIFI_MGR_CONNECTED_BIT) {
            ESP_LOGI(TAG, "Successfully connected to WiFi network");
            return ESP_OK;
        } else if (bits & WIFI_MGR_FAIL_BIT) {
            ESP_LOGE(TAG, "Failed to connect to WiFi network");
            ctx->state = WIFI_MGR_STATE_DISCONNECTED;
            return ESP_FAIL;
        } else {
            ESP_LOGE(TAG, "WiFi connection timeout");
            ctx->state = WIFI_MGR_STATE_DISCONNECTED;
            return ESP_ERR_TIMEOUT;
        }
    }
    
    return ESP_OK;
}

esp_err_t wifi_manager_disconnect(wifi_manager_handle_t handle)
{
    struct wifi_manager_context* ctx = (struct wifi_manager_context*)handle;
    
    if (ctx == NULL || !ctx->initialized) {
        ESP_LOGE(TAG, "Invalid WiFi manager handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Disconnect from WiFi
    esp_err_t ret = esp_wifi_disconnect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disconnect from WiFi: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Update state
    ctx->state = WIFI_MGR_STATE_DISCONNECTED;
    ctx->offline_mode = false;
    
    return ESP_OK;
}

esp_err_t wifi_manager_enter_offline_mode(wifi_manager_handle_t handle)
{
    struct wifi_manager_context* ctx = (struct wifi_manager_context*)handle;
    
    if (ctx == NULL || !ctx->initialized) {
        ESP_LOGE(TAG, "Invalid WiFi manager handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    // If already connected, disconnect
    if (ctx->state == WIFI_MGR_STATE_CONNECTED) {
        esp_wifi_disconnect();
    }
    
    // Set offline mode flag
    ctx->offline_mode = true;
    ctx->state = WIFI_MGR_STATE_OFFLINE_MODE;
    
    // Notify callbacks
    for (int i = 0; i < MAX_WIFI_CALLBACKS; i++) {
        if (ctx->callbacks[i] != NULL) {
            ctx->callbacks[i](WIFI_MGR_EVENT_OFFLINE_MODE_ENTERED, ctx->callback_user_data[i]);
        }
    }
    
    ESP_LOGI(TAG, "Entered offline mode");
    return ESP_OK;
}

esp_err_t wifi_manager_exit_offline_mode(wifi_manager_handle_t handle)
{
    struct wifi_manager_context* ctx = (struct wifi_manager_context*)handle;
    
    if (ctx == NULL || !ctx->initialized) {
        ESP_LOGE(TAG, "Invalid WiFi manager handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Clear offline mode flag
    ctx->offline_mode = false;
    ctx->state = WIFI_MGR_STATE_DISCONNECTED;
    
    // Notify callbacks
    for (int i = 0; i < MAX_WIFI_CALLBACKS; i++) {
        if (ctx->callbacks[i] != NULL) {
            ctx->callbacks[i](WIFI_MGR_EVENT_OFFLINE_MODE_EXITED, ctx->callback_user_data[i]);
        }
    }
    
    ESP_LOGI(TAG, "Exited offline mode, attempting to reconnect");
    return wifi_manager_connect(handle, false);
}

wifi_mgr_connection_state_t wifi_manager_get_state(wifi_manager_handle_t handle)
{
    struct wifi_manager_context* ctx = (struct wifi_manager_context*)handle;
    
    if (ctx == NULL || !ctx->initialized) {
        ESP_LOGE(TAG, "Invalid WiFi manager handle");
        return WIFI_MGR_STATE_DISCONNECTED;
    }
    
    return ctx->state;
}

bool wifi_manager_is_connected(wifi_manager_handle_t handle)
{
    struct wifi_manager_context* ctx = (struct wifi_manager_context*)handle;
    
    if (ctx == NULL || !ctx->initialized) {
        return false;
    }
    
    return (ctx->state == WIFI_MGR_STATE_CONNECTED);
}

bool wifi_manager_is_offline_mode(wifi_manager_handle_t handle)
{
    struct wifi_manager_context* ctx = (struct wifi_manager_context*)handle;
    
    if (ctx == NULL || !ctx->initialized) {
        return false;
    }
    
    return ctx->offline_mode;
}

esp_err_t wifi_manager_get_rssi(wifi_manager_handle_t handle, int32_t* rssi)
{
    struct wifi_manager_context* ctx = (struct wifi_manager_context*)handle;
    
    if (ctx == NULL || !ctx->initialized || rssi == NULL) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (ctx->state != WIFI_MGR_STATE_CONNECTED) {
        ESP_LOGE(TAG, "Not connected to WiFi");
        return ESP_ERR_INVALID_STATE;
    }
    
    wifi_ap_record_t ap_info;
    esp_err_t ret = esp_wifi_sta_get_ap_info(&ap_info);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get AP info: %s", esp_err_to_name(ret));
        return ret;
    }
    
    *rssi = ap_info.rssi;
    return ESP_OK;
}

esp_err_t wifi_manager_get_ip_info(wifi_manager_handle_t handle, char* ip_addr, char* gateway, char* netmask)
{
    struct wifi_manager_context* ctx = (struct wifi_manager_context*)handle;
    
    if (ctx == NULL || !ctx->initialized) {
        ESP_LOGE(TAG, "Invalid WiFi manager handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (ctx->state != WIFI_MGR_STATE_CONNECTED) {
        ESP_LOGE(TAG, "Not connected to WiFi");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Get IP information
    esp_netif_ip_info_t ip_info;
    esp_err_t ret = esp_netif_get_ip_info(ctx->netif, &ip_info);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get IP info: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Convert to string format
    if (ip_addr) {
        esp_ip4addr_ntoa(&ip_info.ip, ip_addr, 16);
    }
    
    if (gateway) {
        esp_ip4addr_ntoa(&ip_info.gw, gateway, 16);
    }
    
    if (netmask) {
        esp_ip4addr_ntoa(&ip_info.netmask, netmask, 16);
    }
    
    return ESP_OK;
}

esp_err_t wifi_manager_register_callback(wifi_manager_handle_t handle, wifi_mgr_event_callback_t callback, void* user_data)
{
    struct wifi_manager_context* ctx = (struct wifi_manager_context*)handle;
    
    if (ctx == NULL || !ctx->initialized || callback == NULL) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Find available callback slot
    for (int i = 0; i < MAX_WIFI_CALLBACKS; i++) {
        if (ctx->callbacks[i] == NULL) {
            ctx->callbacks[i] = callback;
            ctx->callback_user_data[i] = user_data;
            ESP_LOGI(TAG, "Registered WiFi callback at slot %d", i);
            return ESP_OK;
        }
    }
    
    ESP_LOGE(TAG, "No available callback slots");
    return ESP_ERR_NO_MEM;
}

esp_err_t wifi_manager_unregister_callback(wifi_manager_handle_t handle, wifi_mgr_event_callback_t callback)
{
    struct wifi_manager_context* ctx = (struct wifi_manager_context*)handle;
    
    if (ctx == NULL || !ctx->initialized || callback == NULL) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Find and unregister callback
    for (int i = 0; i < MAX_WIFI_CALLBACKS; i++) {
        if (ctx->callbacks[i] == callback) {
            ctx->callbacks[i] = NULL;
            ctx->callback_user_data[i] = NULL;
            ESP_LOGI(TAG, "Unregistered WiFi callback at slot %d", i);
            return ESP_OK;
        }
    }
    
    ESP_LOGW(TAG, "Callback not found");
    return ESP_ERR_NOT_FOUND;
}

const char* wifi_manager_event_to_string(wifi_mgr_event_t event)
{
    switch (event) {
        case WIFI_MGR_EVENT_CONNECTED:          return "CONNECTED";
        case WIFI_MGR_EVENT_DISCONNECTED:       return "DISCONNECTED";
        case WIFI_MGR_EVENT_CONNECTION_FAILED:  return "CONNECTION_FAILED";
        case WIFI_MGR_EVENT_OFFLINE_MODE_ENTERED: return "OFFLINE_MODE_ENTERED";
        case WIFI_MGR_EVENT_OFFLINE_MODE_EXITED:  return "OFFLINE_MODE_EXITED";
        case WIFI_MGR_EVENT_GOT_IP:             return "GOT_IP";
        case WIFI_MGR_EVENT_LOST_IP:            return "LOST_IP";
        default:                            return "UNKNOWN";
    }
}

const char* wifi_manager_state_to_string(wifi_mgr_connection_state_t state)
{
    switch (state) {
        case WIFI_MGR_STATE_DISCONNECTED:       return "DISCONNECTED";
        case WIFI_MGR_STATE_CONNECTING:         return "CONNECTING";
        case WIFI_MGR_STATE_CONNECTED:          return "CONNECTED";
        case WIFI_MGR_STATE_CONNECTION_LOST:    return "CONNECTION_LOST";
        case WIFI_MGR_STATE_OFFLINE_MODE:       return "OFFLINE_MODE";
        default:                            return "UNKNOWN";
    }
}