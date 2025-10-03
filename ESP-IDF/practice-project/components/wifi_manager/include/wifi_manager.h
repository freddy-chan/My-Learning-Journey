/**
 * @file wifi_manager.h
 * @brief WiFi Manager Component for Offline-Ready IoT Systems
 * 
 * This component implements a robust WiFi management system with offline capabilities
 * for IoT devices. It handles connection management, offline mode transitions, and
 * provides status information to other components in the system.
 * 
 * Key Features:
 * - Automatic WiFi connection management with retry logic
 * - Offline mode detection and transition handling
 * - Connection status monitoring and reporting
 * - Integration with system configuration for network settings
 * - Event-based architecture with callback support
 * - Robust error handling and recovery mechanisms
 * 
 * Offline Architecture Support:
 * - Detects and reports connectivity status
 * - Triggers offline mode transitions
 * - Maintains connection state for other components
 * - Supports graceful degradation when network is unavailable
 * 
 * @author Offline-Ready IoT System
 * @date 2025
 * @version 1.0
 */

#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "system_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief WiFi connection status enumeration
 * 
 * Represents the current state of the WiFi connection
 */
typedef enum {
    WIFI_STATE_DISCONNECTED = 0,    /**< Not connected to any network */
    WIFI_STATE_CONNECTING,          /**< In the process of connecting */
    WIFI_STATE_CONNECTED,           /**< Successfully connected to network */
    WIFI_STATE_CONNECTION_LOST,     /**< Connection lost, attempting recovery */
    WIFI_STATE_OFFLINE_MODE         /**< Deliberately in offline mode */
} wifi_connection_state_t;

/**
 * @brief WiFi event types
 * 
 * Events that can be reported by the WiFi manager
 */
typedef enum {
    WIFI_EVENT_CONNECTED = 0,       /**< Successfully connected to WiFi */
    WIFI_EVENT_DISCONNECTED,        /**< Disconnected from WiFi */
    WIFI_EVENT_CONNECTION_FAILED,   /**< Failed to connect to WiFi */
    WIFI_EVENT_OFFLINE_MODE_ENTERED,/**< Entered offline mode */
    WIFI_EVENT_OFFLINE_MODE_EXITED, /**< Exited offline mode */
    WIFI_EVENT_GOT_IP,              /**< Obtained IP address */
    WIFI_EVENT_LOST_IP              /**< Lost IP address */
} wifi_event_t;

/**
 * @brief WiFi manager configuration structure
 * 
 * Contains configuration parameters for the WiFi manager
 */
typedef struct {
    const char* ssid;               /**< WiFi network SSID */
    const char* password;           /**< WiFi network password */
    uint32_t connect_timeout_ms;    /**< Connection timeout in milliseconds */
    uint32_t retry_interval_ms;     /**< Retry interval after failed connection */
    uint8_t max_retry_count;        /**< Maximum connection retry attempts */
    bool auto_reconnect;            /**< Automatically reconnect on disconnection */
} wifi_manager_config_t;

/**
 * @brief WiFi manager handle (opaque)
 * 
 * This handle represents an initialized WiFi manager instance
 */
typedef void* wifi_manager_handle_t;

/**
 * @brief WiFi event callback function type
 * 
 * Function signature for WiFi event callbacks
 * 
 * @param event The WiFi event that occurred
 * @param user_data User-provided data pointer
 */
typedef void (*wifi_event_callback_t)(wifi_event_t event, void* user_data);

/**
 * @brief Initialize WiFi manager
 * 
 * Initializes the WiFi manager with the provided configuration. This function
 * sets up the WiFi subsystem and prepares it for connection attempts.
 * 
 * @param config Pointer to WiFi manager configuration
 * @param handle Pointer to store the created WiFi manager handle
 * @return ESP_OK on success, error code on failure
 */
esp_err_t wifi_manager_init(const wifi_manager_config_t* config, wifi_manager_handle_t* handle);

/**
 * @brief Deinitialize WiFi manager
 * 
 * Cleans up all resources used by the WiFi manager and shuts down the WiFi subsystem.
 * 
 * @param handle WiFi manager handle
 * @return ESP_OK on success, error code on failure
 */
esp_err_t wifi_manager_deinit(wifi_manager_handle_t handle);

/**
 * @brief Connect to WiFi network
 * 
 * Attempts to connect to the configured WiFi network. This function can operate
 * in blocking or non-blocking mode based on the blocking parameter.
 * 
 * @param handle WiFi manager handle
 * @param blocking If true, function blocks until connection attempt completes
 * @return ESP_OK on successful connection, error code on failure
 */
esp_err_t wifi_manager_connect(wifi_manager_handle_t handle, bool blocking);

/**
 * @brief Disconnect from WiFi network
 * 
 * Disconnects from the current WiFi network and stops any ongoing connection attempts.
 * 
 * @param handle WiFi manager handle
 * @return ESP_OK on success, error code on failure
 */
esp_err_t wifi_manager_disconnect(wifi_manager_handle_t handle);

/**
 * @brief Enter offline mode
 * 
 * Manually enters offline mode, disconnecting from WiFi and notifying other
 * components that the device is operating offline.
 * 
 * @param handle WiFi manager handle
 * @return ESP_OK on success, error code on failure
 */
esp_err_t wifi_manager_enter_offline_mode(wifi_manager_handle_t handle);

/**
 * @brief Exit offline mode
 * 
 * Exits offline mode and attempts to reconnect to the configured WiFi network.
 * 
 * @param handle WiFi manager handle
 * @return ESP_OK on success, error code on failure
 */
esp_err_t wifi_manager_exit_offline_mode(wifi_manager_handle_t handle);

/**
 * @brief Get current WiFi connection state
 * 
 * Returns the current connection state of the WiFi manager.
 * 
 * @param handle WiFi manager handle
 * @return Current WiFi connection state
 */
wifi_connection_state_t wifi_manager_get_state(wifi_manager_handle_t handle);

/**
 * @brief Check if device is connected to WiFi
 * 
 * Convenience function to check if the device is currently connected to a WiFi network.
 * 
 * @param handle WiFi manager handle
 * @return true if connected, false otherwise
 */
bool wifi_manager_is_connected(wifi_manager_handle_t handle);

/**
 * @brief Check if device is in offline mode
 * 
 * Convenience function to check if the device is deliberately operating in offline mode.
 * 
 * @param handle WiFi manager handle
 * @return true if in offline mode, false otherwise
 */
bool wifi_manager_is_offline_mode(wifi_manager_handle_t handle);

/**
 * @brief Get WiFi signal strength
 * 
 * Returns the current WiFi signal strength in dBm.
 * 
 * @param handle WiFi manager handle
 * @param rssi Pointer to store RSSI value
 * @return ESP_OK on success, error code on failure
 */
esp_err_t wifi_manager_get_rssi(wifi_manager_handle_t handle, int32_t* rssi);

/**
 * @brief Get IP address information
 * 
 * Retrieves the current IP address information if connected.
 * 
 * @param handle WiFi manager handle
 * @param ip_addr Buffer to store IP address string (at least 16 bytes)
 * @param gateway Buffer to store gateway address string (at least 16 bytes)
 * @param netmask Buffer to store netmask string (at least 16 bytes)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t wifi_manager_get_ip_info(wifi_manager_handle_t handle, char* ip_addr, char* gateway, char* netmask);

/**
 * @brief Register WiFi event callback
 * 
 * Registers a callback function to be notified of WiFi events.
 * 
 * @param handle WiFi manager handle
 * @param callback Callback function to register
 * @param user_data User data to pass to callback
 * @return ESP_OK on success, error code on failure
 */
esp_err_t wifi_manager_register_callback(wifi_manager_handle_t handle, wifi_event_callback_t callback, void* user_data);

/**
 * @brief Unregister WiFi event callback
 * 
 * Unregisters a previously registered WiFi event callback.
 * 
 * @param handle WiFi manager handle
 * @param callback Callback function to unregister
 * @return ESP_OK on success, error code on failure
 */
esp_err_t wifi_manager_unregister_callback(wifi_manager_handle_t handle, wifi_event_callback_t callback);

/**
 * @brief Convert WiFi event to string
 * 
 * Converts a WiFi event to a human-readable string for logging and debugging.
 * 
 * @param event WiFi event to convert
 * @return String representation of the event
 */
const char* wifi_manager_event_to_string(wifi_event_t event);

/**
 * @brief Convert WiFi connection state to string
 * 
 * Converts a WiFi connection state to a human-readable string for logging and debugging.
 * 
 * @param state WiFi connection state to convert
 * @return String representation of the state
 */
const char* wifi_manager_state_to_string(wifi_connection_state_t state);

#ifdef __cplusplus
}
#endif

#endif // WIFI_MANAGER_H