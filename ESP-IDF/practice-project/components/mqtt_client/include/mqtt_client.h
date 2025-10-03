/**
 * @file mqtt_client.h
 * @brief MQTT Client Component for Offline-Ready IoT Systems
 * 
 * This component implements a robust MQTT client with offline capabilities
 * for IoT devices. It handles connection management, message queuing for
 * offline operation, and synchronization with the MQTT broker.
 * 
 * Key Features:
 * - Automatic MQTT connection management with retry logic
 * - Offline message queuing and synchronization
 * - Quality of Service (QoS) support for message delivery
 * - Integration with system configuration for broker settings
 * - Event-based architecture with callback support
 * - Robust error handling and recovery mechanisms
 * 
 * Offline Architecture Support:
 * - Queues messages when offline
 * - Synchronizes queued messages when connectivity is restored
 * - Implements conflict resolution strategies
 * - Supports both push and pull synchronization patterns
 * 
 * @author Offline-Ready IoT System
 * @date 2025
 * @version 1.0
 */

#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief MQTT client connection status enumeration
 * 
 * Represents the current state of the MQTT client connection
 */
typedef enum {
    MQTT_MGR_STATE_DISCONNECTED = 0,    /**< Not connected to broker */
    MQTT_MGR_STATE_CONNECTING,          /**< In the process of connecting */
    MQTT_MGR_STATE_CONNECTED,           /**< Successfully connected to broker */
    MQTT_MGR_STATE_CONNECTION_LOST,     /**< Connection lost, attempting recovery */
    MQTT_MGR_STATE_OFFLINE_QUEUING      /**< Operating in offline mode with message queuing */
} mqtt_mgr_connection_state_t;

/**
 * @brief MQTT Quality of Service levels
 * 
 * Defines the MQTT QoS levels for message delivery
 */
typedef enum {
    MQTT_MGR_QOS_AT_MOST_ONCE = 0,      /**< At most once delivery (0) */
    MQTT_MGR_QOS_AT_LEAST_ONCE = 1,     /**< At least once delivery (1) */
    MQTT_MGR_QOS_EXACTLY_ONCE = 2       /**< Exactly once delivery (2) */
} mqtt_mgr_qos_t;

/**
 * @brief MQTT event types
 * 
 * Events that can be reported by the MQTT client
 */
typedef enum {
    MQTT_MGR_EVENT_CONNECTED = 0,       /**< Successfully connected to broker */
    MQTT_MGR_EVENT_DISCONNECTED,        /**< Disconnected from broker */
    MQTT_MGR_EVENT_CONNECTION_FAILED,   /**< Failed to connect to broker */
    MQTT_MGR_EVENT_DATA_RECEIVED,       /**< Data received on subscribed topic */
    MQTT_MGR_EVENT_PUBLISHED,           /**< Message successfully published */
    MQTT_MGR_EVENT_OFFLINE_QUEUING_STARTED, /**< Started offline message queuing */
    MQTT_MGR_EVENT_OFFLINE_SYNC_STARTED,/**< Started synchronization of queued messages */
    MQTT_MGR_EVENT_OFFLINE_SYNC_COMPLETED /**< Completed synchronization of queued messages */
} mqtt_mgr_event_t;

/**
 * @brief MQTT client configuration structure
 * 
 * Contains configuration parameters for the MQTT client
 */
typedef struct {
    const char* broker_uri;         /**< MQTT broker URI (e.g., mqtt://broker.hivemq.com:1883) */
    const char* client_id;          /**< Client identifier */
    const char* username;           /**< Username for authentication (optional) */
    const char* password;           /**< Password for authentication (optional) */
    uint32_t connect_timeout_ms;    /**< Connection timeout in milliseconds */
    uint32_t retry_interval_ms;     /**< Retry interval after failed connection */
    uint8_t max_retry_count;        /**< Maximum connection retry attempts */
    bool auto_reconnect;            /**< Automatically reconnect on disconnection */
    uint16_t keepalive;             /**< Keepalive time in seconds */
    bool use_tls;                   /**< Use TLS for secure connection */
} mqtt_client_config_t;

/**
 * @brief MQTT message structure
 * 
 * Represents an MQTT message with topic and payload
 */
typedef struct {
    const char* topic;              /**< Topic string */
    const char* payload;            /**< Payload data */
    size_t payload_len;             /**< Length of payload data */
    mqtt_mgr_qos_t qos;                 /**< Quality of Service level */
    bool retain;                    /**< Retain flag */
    uint16_t message_id;            /**< Message identifier (for QoS > 0) */
} mqtt_message_t;

/**
 * @brief MQTT client handle (opaque)
 * 
 * This handle represents an initialized MQTT client instance
 */
typedef void* mqtt_client_handle_t;

/**
 * @brief MQTT event callback function type
 * 
 * Function signature for MQTT event callbacks
 * 
 * @param event The MQTT event that occurred
 * @param message Pointer to message data (if applicable)
 * @param user_data User-provided data pointer
 */
typedef void (*mqtt_mgr_event_callback_t)(mqtt_mgr_event_t event, const mqtt_message_t* message, void* user_data);

/**
 * @brief Initialize MQTT client
 * 
 * Initializes the MQTT client with the provided configuration. This function
 * sets up the MQTT client and prepares it for connection attempts.
 * 
 * @param config Pointer to MQTT client configuration
 * @param handle Pointer to store the created MQTT client handle
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mqtt_client_init(const mqtt_client_config_t* config, mqtt_client_handle_t* handle);

/**
 * @brief Deinitialize MQTT client
 * 
 * Cleans up all resources used by the MQTT client and shuts down the connection.
 * 
 * @param handle MQTT client handle
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mqtt_client_deinit(mqtt_client_handle_t handle);

/**
 * @brief Connect to MQTT broker
 * 
 * Attempts to connect to the configured MQTT broker. This function can operate
 * in blocking or non-blocking mode based on the blocking parameter.
 * 
 * @param handle MQTT client handle
 * @param blocking If true, function blocks until connection attempt completes
 * @return ESP_OK on successful connection, error code on failure
 */
esp_err_t mqtt_client_connect(mqtt_client_handle_t handle, bool blocking);

/**
 * @brief Disconnect from MQTT broker
 * 
 * Disconnects from the current MQTT broker and stops any ongoing connection attempts.
 * 
 * @param handle MQTT client handle
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mqtt_client_disconnect(mqtt_client_handle_t handle);

/**
 * @brief Publish MQTT message
 * 
 * Publishes a message to the specified topic. In offline mode, messages are queued
 * for later synchronization.
 * 
 * @param handle MQTT client handle
 * @param topic Topic to publish to
 * @param data Payload data
 * @param len Length of payload data
 * @param qos Quality of Service level
 * @param retain Retain flag
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mqtt_client_publish(mqtt_client_handle_t handle, const char* topic, const char* data, 
                             size_t len, mqtt_mgr_qos_t qos, bool retain);

/**
 * @brief Subscribe to MQTT topic
 * 
 * Subscribes to the specified topic to receive messages.
 * 
 * @param handle MQTT client handle
 * @param topic Topic to subscribe to
 * @param qos Quality of Service level
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mqtt_client_subscribe(mqtt_client_handle_t handle, const char* topic, mqtt_mgr_qos_t qos);

/**
 * @brief Unsubscribe from MQTT topic
 * 
 * Unsubscribes from the specified topic.
 * 
 * @param handle MQTT client handle
 * @param topic Topic to unsubscribe from
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mqtt_client_unsubscribe(mqtt_client_handle_t handle, const char* topic);

/**
 * @brief Enter offline queuing mode
 * 
 * Manually enters offline queuing mode, where messages are queued for later
 * synchronization instead of being sent immediately.
 * 
 * @param handle MQTT client handle
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mqtt_client_enter_offline_mode(mqtt_client_handle_t handle);

/**
 * @brief Exit offline queuing mode and sync messages
 * 
 * Exits offline queuing mode and attempts to synchronize queued messages
 * with the MQTT broker.
 * 
 * @param handle MQTT client handle
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mqtt_client_exit_offline_mode(mqtt_client_handle_t handle);

/**
 * @brief Get current MQTT connection state
 * 
 * Returns the current connection state of the MQTT client.
 * 
 * @param handle MQTT client handle
 * @return Current MQTT connection state
 */
mqtt_mgr_connection_state_t mqtt_client_get_state(mqtt_client_handle_t handle);

/**
 * @brief Check if client is connected to MQTT broker
 * 
 * Convenience function to check if the client is currently connected to an MQTT broker.
 * 
 * @param handle MQTT client handle
 * @return true if connected, false otherwise
 */
bool mqtt_client_is_connected(mqtt_client_handle_t handle);

/**
 * @brief Get number of queued messages
 * 
 * Returns the number of messages currently queued for offline synchronization.
 * 
 * @param handle MQTT client handle
 * @param count Pointer to store message count
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mqtt_client_get_queued_message_count(mqtt_client_handle_t handle, uint32_t* count);

/**
 * @brief Clear queued messages
 * 
 * Clears all queued messages. Use with caution as this will result in data loss.
 * 
 * @param handle MQTT client handle
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mqtt_client_clear_queued_messages(mqtt_client_handle_t handle);

/**
 * @brief Register MQTT event callback
 * 
 * Registers a callback function to be notified of MQTT events.
 * 
 * @param handle MQTT client handle
 * @param callback Callback function to register
 * @param user_data User data to pass to callback
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mqtt_client_register_callback(mqtt_client_handle_t handle, mqtt_mgr_event_callback_t callback, void* user_data);

/**
 * @brief Unregister MQTT event callback
 * 
 * Unregisters a previously registered MQTT event callback.
 * 
 * @param handle MQTT client handle
 * @param callback Callback function to unregister
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mqtt_client_unregister_callback(mqtt_client_handle_t handle, mqtt_mgr_event_callback_t callback);

/**
 * @brief Convert MQTT event to string
 * 
 * Converts an MQTT event to a human-readable string for logging and debugging.
 * 
 * @param event MQTT event to convert
 * @return String representation of the event
 */
const char* mqtt_client_event_to_string(mqtt_mgr_event_t event);

/**
 * @brief Convert MQTT connection state to string
 * 
 * Converts an MQTT connection state to a human-readable string for logging and debugging.
 * 
 * @param state MQTT connection state to convert
 * @return String representation of the state
 */
const char* mqtt_client_state_to_string(mqtt_mgr_connection_state_t state);

#ifdef __cplusplus
}
#endif

#endif // MQTT_CLIENT_H