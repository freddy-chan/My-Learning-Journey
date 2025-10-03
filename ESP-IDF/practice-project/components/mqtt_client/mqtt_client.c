/**
 * @file mqtt_client.c
 * @brief MQTT Client Component Implementation
 * 
 * This file implements the MQTT client component with offline capabilities
 * for IoT devices. It handles connection management, message queuing for
 * offline operation, and synchronization with the MQTT broker.
 * 
 * @author Offline-Ready IoT System
 * @date 2025
 * @version 1.0
 */

#include "mqtt_client.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "MQTT_CLIENT";

// Maximum number of registered callbacks
#define MAX_MQTT_CALLBACKS 8

// Maximum number of queued messages
#define MAX_QUEUED_MESSAGES 100

// Default configuration values
#define DEFAULT_CONNECT_TIMEOUT_MS 10000
#define DEFAULT_RETRY_INTERVAL_MS 5000
#define DEFAULT_MAX_RETRY_COUNT 3
#define DEFAULT_KEEPALIVE 120

/**
 * @brief Queued message structure
 * 
 * Represents a message queued for offline synchronization
 */
typedef struct {
    char* topic;                    // Topic string (dynamically allocated)
    char* payload;                  // Payload data (dynamically allocated)
    size_t payload_len;             // Length of payload data
    mqtt_qos_t qos;                 // Quality of Service level
    bool retain;                    // Retain flag
    uint64_t timestamp;             // Timestamp when message was queued
} queued_message_t;

/**
 * @brief MQTT client context structure
 * 
 * Contains all state information for the MQTT client
 */
struct mqtt_client_context {
    mqtt_client_config_t config;              // Configuration settings
    mqtt_connection_state_t state;            // Current connection state
    esp_mqtt_client_handle_t esp_mqtt_client; // ESP-IDF MQTT client handle
    mqtt_event_callback_t callbacks[MAX_MQTT_CALLBACKS]; // Registered callbacks
    void* callback_user_data[MAX_MQTT_CALLBACKS];        // User data for callbacks
    uint8_t retry_count;                      // Current retry count
    bool offline_mode;                        // Offline queuing mode flag
    QueueHandle_t message_queue;              // Queue for offline messages
    SemaphoreHandle_t queue_mutex;            // Mutex for queue access
    uint32_t queued_message_count;            // Number of queued messages
    bool initialized;                         // Initialization status
};

// Global MQTT client instance
static struct mqtt_client_context* g_mqtt_client = NULL;

/**
 * @brief MQTT event handler
 * 
 * Handles MQTT events from the ESP-IDF MQTT client
 * 
 * @param handler_args User argument (MQTT client context)
 * @param base Event base
 * @param event_id Event ID
 * @param event_data Event data
 */
static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    struct mqtt_client_context* ctx = (struct mqtt_client_context*)event->user_context;
    
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT connected");
            ctx->state = MQTT_STATE_CONNECTED;
            ctx->retry_count = 0;
            
            // Notify callbacks
            for (int i = 0; i < MAX_MQTT_CALLBACKS; i++) {
                if (ctx->callbacks[i] != NULL) {
                    ctx->callbacks[i](MQTT_EVENT_CONNECTED, NULL, ctx->callback_user_data[i]);
                }
            }
            
            // If in offline mode, start synchronization
            if (ctx->offline_mode) {
                mqtt_client_exit_offline_mode(ctx);
            }
            break;
            
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT disconnected");
            ctx->state = MQTT_STATE_DISCONNECTED;
            
            // Notify callbacks
            for (int i = 0; i < MAX_MQTT_CALLBACKS; i++) {
                if (ctx->callbacks[i] != NULL) {
                    ctx->callbacks[i](MQTT_EVENT_DISCONNECTED, NULL, ctx->callback_user_data[i]);
                }
            }
            
            // Attempt reconnection if enabled and not in offline mode
            if (ctx->config.auto_reconnect && !ctx->offline_mode) {
                if (ctx->retry_count < ctx->config.max_retry_count) {
                    ctx->retry_count++;
                    ctx->state = MQTT_STATE_CONNECTING;
                    ESP_LOGI(TAG, "Attempting to reconnect (attempt %d/%d)", 
                            ctx->retry_count, ctx->config.max_retry_count);
                }
            }
            break;
            
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT data received on topic %.*s", event->topic_len, event->topic);
            
            // Create message structure for callback
            mqtt_message_t message = {
                .topic = event->topic,
                .payload = event->data,
                .payload_len = event->data_len,
                .qos = MQTT_QOS_AT_MOST_ONCE, // QoS not available in event
                .retain = false, // Retain flag not available in event
                .message_id = event->msg_id
            };
            
            // Notify callbacks
            for (int i = 0; i < MAX_MQTT_CALLBACKS; i++) {
                if (ctx->callbacks[i] != NULL) {
                    ctx->callbacks[i](MQTT_EVENT_DATA_RECEIVED, &message, ctx->callback_user_data[i]);
                }
            }
            break;
            
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT message published (msg_id: %d)", event->msg_id);
            
            // Notify callbacks
            for (int i = 0; i < MAX_MQTT_CALLBACKS; i++) {
                if (ctx->callbacks[i] != NULL) {
                    ctx->callbacks[i](MQTT_EVENT_PUBLISHED, NULL, ctx->callback_user_data[i]);
                }
            }
            break;
            
        default:
            ESP_LOGD(TAG, "Other MQTT event id:%d", event->event_id);
            break;
    }
    
    return ESP_OK;
}

/**
 * @brief Create ESP-IDF MQTT client configuration
 * 
 * Creates and configures the ESP-IDF MQTT client based on our configuration
 * 
 * @param ctx MQTT client context
 * @param esp_config Pointer to ESP-IDF MQTT client configuration to populate
 */
static void create_esp_mqtt_config(struct mqtt_client_context* ctx, esp_mqtt_client_config_t* esp_config)
{
    // Initialize configuration
    memset(esp_config, 0, sizeof(esp_mqtt_client_config_t));
    
    // Set broker URI
    esp_config->broker.address.uri = ctx->config.broker_uri;
    
    // Set client ID if provided
    if (ctx->config.client_id) {
        esp_config->credentials.client_id = ctx->config.client_id;
    }
    
    // Set authentication if provided
    if (ctx->config.username) {
        esp_config->credentials.username = ctx->config.username;
    }
    
    if (ctx->config.password) {
        esp_config->credentials.authentication.password = ctx->config.password;
    }
    
    // Set connection parameters
    esp_config->session.keepalive = ctx->config.keepalive;
    esp_config->network.timeout_ms = ctx->config.connect_timeout_ms;
    
    // Set TLS configuration if enabled
    if (ctx->config.use_tls) {
        esp_config->broker.verification.skip_cert_common_name_check = false;
    }
}

/**
 * @brief Queue message for offline synchronization
 * 
 * Adds a message to the offline queue for later synchronization
 * 
 * @param ctx MQTT client context
 * @param topic Topic to publish to
 * @param data Payload data
 * @param len Length of payload data
 * @param qos Quality of Service level
 * @param retain Retain flag
 * @return ESP_OK on success, error code on failure
 */
static esp_err_t queue_message_for_offline(struct mqtt_client_context* ctx, 
                                          const char* topic, const char* data, 
                                          size_t len, mqtt_qos_t qos, bool retain)
{
    // Check if queue is full
    if (ctx->queued_message_count >= MAX_QUEUED_MESSAGES) {
        ESP_LOGE(TAG, "Message queue is full, dropping message");
        return ESP_ERR_NO_MEM;
    }
    
    // Allocate memory for queued message
    queued_message_t* queued_msg = calloc(1, sizeof(queued_message_t));
    if (queued_msg == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for queued message");
        return ESP_ERR_NO_MEM;
    }
    
    // Copy topic
    queued_msg->topic = strdup(topic);
    if (queued_msg->topic == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for topic");
        free(queued_msg);
        return ESP_ERR_NO_MEM;
    }
    
    // Copy payload
    queued_msg->payload = malloc(len);
    if (queued_msg->payload == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for payload");
        free(queued_msg->topic);
        free(queued_msg);
        return ESP_ERR_NO_MEM;
    }
    
    memcpy(queued_msg->payload, data, len);
    queued_msg->payload_len = len;
    queued_msg->qos = qos;
    queued_msg->retain = retain;
    queued_msg->timestamp = esp_timer_get_time() / 1000; // Convert to milliseconds
    
    // Add to queue
    if (xQueueSend(ctx->message_queue, &queued_msg, 0) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to add message to queue");
        free(queued_msg->payload);
        free(queued_msg->topic);
        free(queued_msg);
        return ESP_FAIL;
    }
    
    // Update count
    ctx->queued_message_count++;
    
    ESP_LOGI(TAG, "Message queued for offline synchronization (total: %lu)", ctx->queued_message_count);
    return ESP_OK;
}

/**
 * @brief Synchronize queued messages
 * 
 * Publishes all queued messages to the MQTT broker
 * 
 * @param ctx MQTT client context
 * @return ESP_OK on success, error code on failure
 */
static esp_err_t synchronize_queued_messages(struct mqtt_client_context* ctx)
{
    if (ctx->state != MQTT_STATE_CONNECTED) {
        ESP_LOGE(TAG, "Cannot synchronize messages: not connected to broker");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Starting synchronization of %lu queued messages", ctx->queued_message_count);
    
    // Notify callbacks
    for (int i = 0; i < MAX_MQTT_CALLBACKS; i++) {
        if (ctx->callbacks[i] != NULL) {
            ctx->callbacks[i](MQTT_EVENT_OFFLINE_SYNC_STARTED, NULL, ctx->callback_user_data[i]);
        }
    }
    
    // Process all queued messages
    queued_message_t* queued_msg;
    uint32_t synchronized_count = 0;
    
    while (ctx->queued_message_count > 0 && 
           xQueueReceive(ctx->message_queue, &queued_msg, 0) == pdTRUE) {
        
        // Publish message
        int msg_id = esp_mqtt_client_publish(ctx->esp_mqtt_client,
                                           queued_msg->topic,
                                           queued_msg->payload,
                                           queued_msg->payload_len,
                                           (int)queued_msg->qos,
                                           queued_msg->retain);
        
        if (msg_id < 0) {
            ESP_LOGE(TAG, "Failed to publish queued message to topic %s", queued_msg->topic);
            // Re-queue the message for next sync attempt?
        } else {
            synchronized_count++;
            ESP_LOGI(TAG, "Published queued message to topic %s (msg_id: %d)", 
                    queued_msg->topic, msg_id);
        }
        
        // Free memory
        free(queued_msg->payload);
        free(queued_msg->topic);
        free(queued_msg);
        
        // Update count
        ctx->queued_message_count--;
    }
    
    ESP_LOGI(TAG, "Completed synchronization: %lu messages synchronized", synchronized_count);
    
    // Notify callbacks
    for (int i = 0; i < MAX_MQTT_CALLBACKS; i++) {
        if (ctx->callbacks[i] != NULL) {
            ctx->callbacks[i](MQTT_EVENT_OFFLINE_SYNC_COMPLETED, NULL, ctx->callback_user_data[i]);
        }
    }
    
    return ESP_OK;
}

// Public API Implementation

esp_err_t mqtt_client_init(const mqtt_client_config_t* config, mqtt_client_handle_t* handle)
{
    if (config == NULL || handle == NULL) {
        ESP_LOGE(TAG, "Invalid parameters: config or handle is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Check if already initialized
    if (g_mqtt_client != NULL) {
        ESP_LOGW(TAG, "MQTT client already initialized");
        *handle = g_mqtt_client;
        return ESP_OK;
    }
    
    // Allocate context
    struct mqtt_client_context* ctx = calloc(1, sizeof(struct mqtt_client_context));
    if (ctx == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for MQTT client context");
        return ESP_ERR_NO_MEM;
    }
    
    // Copy configuration
    ctx->config.broker_uri = config->broker_uri;
    ctx->config.client_id = config->client_id;
    ctx->config.username = config->username;
    ctx->config.password = config->password;
    ctx->config.connect_timeout_ms = config->connect_timeout_ms > 0 ? config->connect_timeout_ms : DEFAULT_CONNECT_TIMEOUT_MS;
    ctx->config.retry_interval_ms = config->retry_interval_ms > 0 ? config->retry_interval_ms : DEFAULT_RETRY_INTERVAL_MS;
    ctx->config.max_retry_count = config->max_retry_count > 0 ? config->max_retry_count : DEFAULT_MAX_RETRY_COUNT;
    ctx->config.auto_reconnect = config->auto_reconnect;
    ctx->config.keepalive = config->keepalive > 0 ? config->keepalive : DEFAULT_KEEPALIVE;
    ctx->config.use_tls = config->use_tls;
    
    // Initialize state
    ctx->state = MQTT_STATE_DISCONNECTED;
    ctx->offline_mode = false;
    ctx->retry_count = 0;
    ctx->queued_message_count = 0;
    ctx->initialized = true;
    
    // Create message queue
    ctx->message_queue = xQueueCreate(MAX_QUEUED_MESSAGES, sizeof(queued_message_t*));
    if (ctx->message_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create message queue");
        free(ctx);
        return ESP_ERR_NO_MEM;
    }
    
    // Create queue mutex
    ctx->queue_mutex = xSemaphoreCreateMutex();
    if (ctx->queue_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create queue mutex");
        vQueueDelete(ctx->message_queue);
        free(ctx);
        return ESP_ERR_NO_MEM;
    }
    
    // Create ESP-IDF MQTT client configuration
    esp_mqtt_client_config_t esp_config = {0};
    create_esp_mqtt_config(ctx, &esp_config);
    esp_config.user_context = ctx;
    
    // Create ESP-IDF MQTT client
    ctx->esp_mqtt_client = esp_mqtt_client_init(&esp_config);
    if (ctx->esp_mqtt_client == NULL) {
        ESP_LOGE(TAG, "Failed to create ESP-IDF MQTT client");
        vSemaphoreDelete(ctx->queue_mutex);
        vQueueDelete(ctx->message_queue);
        free(ctx);
        return ESP_FAIL;
    }
    
    // Register event handler
    esp_err_t ret = esp_mqtt_client_register_event(ctx->esp_mqtt_client, 
                                                  MQTT_EVENT_ANY, 
                                                  mqtt_event_handler, 
                                                  ctx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register MQTT event handler: %s", esp_err_to_name(ret));
        esp_mqtt_client_destroy(ctx->esp_mqtt_client);
        vSemaphoreDelete(ctx->queue_mutex);
        vQueueDelete(ctx->message_queue);
        free(ctx);
        return ret;
    }
    
    // Set global instance
    g_mqtt_client = ctx;
    *handle = ctx;
    
    ESP_LOGI(TAG, "MQTT client initialized successfully");
    return ESP_OK;
}

esp_err_t mqtt_client_deinit(mqtt_client_handle_t handle)
{
    struct mqtt_client_context* ctx = (struct mqtt_client_context*)handle;
    
    if (ctx == NULL || !ctx->initialized) {
        ESP_LOGE(TAG, "Invalid MQTT client handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Disconnect if connected
    if (ctx->state == MQTT_STATE_CONNECTED) {
        esp_mqtt_client_disconnect(ctx->esp_mqtt_client);
    }
    
    // Stop MQTT client
    esp_mqtt_client_stop(ctx->esp_mqtt_client);
    
    // Destroy MQTT client
    esp_mqtt_client_destroy(ctx->esp_mqtt_client);
    
    // Clean up queue
    if (ctx->message_queue) {
        // Free any remaining queued messages
        queued_message_t* queued_msg;
        while (xQueueReceive(ctx->message_queue, &queued_msg, 0) == pdTRUE) {
            free(queued_msg->payload);
            free(queued_msg->topic);
            free(queued_msg);
        }
        vQueueDelete(ctx->message_queue);
    }
    
    // Clean up mutex
    if (ctx->queue_mutex) {
        vSemaphoreDelete(ctx->queue_mutex);
    }
    
    // Mark as uninitialized
    ctx->initialized = false;
    
    // Clear global instance if it's this one
    if (g_mqtt_client == ctx) {
        g_mqtt_client = NULL;
    }
    
    // Free memory
    free(ctx);
    
    ESP_LOGI(TAG, "MQTT client deinitialized");
    return ESP_OK;
}

esp_err_t mqtt_client_connect(mqtt_client_handle_t handle, bool blocking)
{
    struct mqtt_client_context* ctx = (struct mqtt_client_context*)handle;
    
    if (ctx == NULL || !ctx->initialized) {
        ESP_LOGE(TAG, "Invalid MQTT client handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Cannot connect in offline mode
    if (ctx->offline_mode) {
        ESP_LOGW(TAG, "Cannot connect while in offline mode");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Set state to connecting
    ctx->state = MQTT_STATE_CONNECTING;
    ctx->retry_count = 0;
    
    // Start connection
    esp_err_t ret = esp_mqtt_client_start(ctx->esp_mqtt_client);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MQTT client: %s", esp_err_to_name(ret));
        ctx->state = MQTT_STATE_DISCONNECTED;
        return ret;
    }
    
    // If blocking, wait for connection (simplified - in real implementation
    // you would use events or timeouts)
    if (blocking) {
        // Wait up to connect timeout for connection
        uint32_t wait_time = 0;
        const uint32_t check_interval = 100; // ms
        
        while (ctx->state == MQTT_STATE_CONNECTING && 
               wait_time < ctx->config.connect_timeout_ms) {
            vTaskDelay(pdMS_TO_TICKS(check_interval));
            wait_time += check_interval;
        }
        
        if (ctx->state == MQTT_STATE_CONNECTED) {
            ESP_LOGI(TAG, "Successfully connected to MQTT broker");
            return ESP_OK;
        } else {
            ESP_LOGE(TAG, "Failed to connect to MQTT broker");
            ctx->state = MQTT_STATE_DISCONNECTED;
            return ESP_FAIL;
        }
    }
    
    return ESP_OK;
}

esp_err_t mqtt_client_disconnect(mqtt_client_handle_t handle)
{
    struct mqtt_client_context* ctx = (struct mqtt_client_context*)handle;
    
    if (ctx == NULL || !ctx->initialized) {
        ESP_LOGE(TAG, "Invalid MQTT client handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Disconnect from broker
    esp_err_t ret = esp_mqtt_client_disconnect(ctx->esp_mqtt_client);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disconnect from MQTT broker: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Update state
    ctx->state = MQTT_STATE_DISCONNECTED;
    ctx->offline_mode = false;
    
    return ESP_OK;
}

esp_err_t mqtt_client_publish(mqtt_client_handle_t handle, const char* topic, const char* data, 
                             size_t len, mqtt_qos_t qos, bool retain)
{
    struct mqtt_client_context* ctx = (struct mqtt_client_context*)handle;
    
    if (ctx == NULL || !ctx->initialized || topic == NULL || data == NULL) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    // If in offline mode or not connected, queue the message
    if (ctx->offline_mode || ctx->state != MQTT_STATE_CONNECTED) {
        return queue_message_for_offline(ctx, topic, data, len, qos, retain);
    }
    
    // Publish immediately
    int msg_id = esp_mqtt_client_publish(ctx->esp_mqtt_client, topic, data, len, (int)qos, retain);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "Failed to publish message to topic %s", topic);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Published message to topic %s (msg_id: %d)", topic, msg_id);
    return ESP_OK;
}

esp_err_t mqtt_client_subscribe(mqtt_client_handle_t handle, const char* topic, mqtt_qos_t qos)
{
    struct mqtt_client_context* ctx = (struct mqtt_client_context*)handle;
    
    if (ctx == NULL || !ctx->initialized || topic == NULL) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (ctx->state != MQTT_STATE_CONNECTED) {
        ESP_LOGE(TAG, "Not connected to MQTT broker");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Subscribe to topic
    int msg_id = esp_mqtt_client_subscribe(ctx->esp_mqtt_client, topic, (int)qos);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "Failed to subscribe to topic %s", topic);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Subscribed to topic %s (msg_id: %d)", topic, msg_id);
    return ESP_OK;
}

esp_err_t mqtt_client_unsubscribe(mqtt_client_handle_t handle, const char* topic)
{
    struct mqtt_client_context* ctx = (struct mqtt_client_context*)handle;
    
    if (ctx == NULL || !ctx->initialized || topic == NULL) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (ctx->state != MQTT_STATE_CONNECTED) {
        ESP_LOGE(TAG, "Not connected to MQTT broker");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Unsubscribe from topic
    int msg_id = esp_mqtt_client_unsubscribe(ctx->esp_mqtt_client, topic);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "Failed to unsubscribe from topic %s", topic);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Unsubscribed from topic %s (msg_id: %d)", topic, msg_id);
    return ESP_OK;
}

esp_err_t mqtt_client_enter_offline_mode(mqtt_client_handle_t handle)
{
    struct mqtt_client_context* ctx = (struct mqtt_client_context*)handle;
    
    if (ctx == NULL || !ctx->initialized) {
        ESP_LOGE(TAG, "Invalid MQTT client handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Set offline mode flag
    ctx->offline_mode = true;
    ctx->state = MQTT_STATE_OFFLINE_QUEUING;
    
    // Notify callbacks
    for (int i = 0; i < MAX_MQTT_CALLBACKS; i++) {
        if (ctx->callbacks[i] != NULL) {
            ctx->callbacks[i](MQTT_EVENT_OFFLINE_QUEUING_STARTED, NULL, ctx->callback_user_data[i]);
        }
    }
    
    ESP_LOGI(TAG, "Entered offline queuing mode");
    return ESP_OK;
}

esp_err_t mqtt_client_exit_offline_mode(mqtt_client_handle_t handle)
{
    struct mqtt_client_context* ctx = (struct mqtt_client_context*)handle;
    
    if (ctx == NULL || !ctx->initialized) {
        ESP_LOGE(TAG, "Invalid MQTT client handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Clear offline mode flag
    ctx->offline_mode = false;
    
    // If connected, synchronize queued messages
    if (ctx->state == MQTT_STATE_CONNECTED && ctx->queued_message_count > 0) {
        return synchronize_queued_messages(ctx);
    }
    
    ESP_LOGI(TAG, "Exited offline queuing mode");
    return ESP_OK;
}

mqtt_connection_state_t mqtt_client_get_state(mqtt_client_handle_t handle)
{
    struct mqtt_client_context* ctx = (struct mqtt_client_context*)handle;
    
    if (ctx == NULL || !ctx->initialized) {
        ESP_LOGE(TAG, "Invalid MQTT client handle");
        return MQTT_STATE_DISCONNECTED;
    }
    
    return ctx->state;
}

bool mqtt_client_is_connected(mqtt_client_handle_t handle)
{
    struct mqtt_client_context* ctx = (struct mqtt_client_context*)handle;
    
    if (ctx == NULL || !ctx->initialized) {
        return false;
    }
    
    return (ctx->state == MQTT_STATE_CONNECTED);
}

esp_err_t mqtt_client_get_queued_message_count(mqtt_client_handle_t handle, uint32_t* count)
{
    struct mqtt_client_context* ctx = (struct mqtt_client_context*)handle;
    
    if (ctx == NULL || !ctx->initialized || count == NULL) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    *count = ctx->queued_message_count;
    return ESP_OK;
}

esp_err_t mqtt_client_clear_queued_messages(mqtt_client_handle_t handle)
{
    struct mqtt_client_context* ctx = (struct mqtt_client_context*)handle;
    
    if (ctx == NULL || !ctx->initialized) {
        ESP_LOGE(TAG, "Invalid MQTT client handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Take mutex
    if (xSemaphoreTake(ctx->queue_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take queue mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    // Free all queued messages
    queued_message_t* queued_msg;
    uint32_t cleared_count = 0;
    
    while (xQueueReceive(ctx->message_queue, &queued_msg, 0) == pdTRUE) {
        free(queued_msg->payload);
        free(queued_msg->topic);
        free(queued_msg);
        cleared_count++;
    }
    
    ctx->queued_message_count = 0;
    
    // Give mutex
    xSemaphoreGive(ctx->queue_mutex);
    
    ESP_LOGI(TAG, "Cleared %lu queued messages", cleared_count);
    return ESP_OK;
}

esp_err_t mqtt_client_register_callback(mqtt_client_handle_t handle, mqtt_event_callback_t callback, void* user_data)
{
    struct mqtt_client_context* ctx = (struct mqtt_client_context*)handle;
    
    if (ctx == NULL || !ctx->initialized || callback == NULL) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Find available callback slot
    for (int i = 0; i < MAX_MQTT_CALLBACKS; i++) {
        if (ctx->callbacks[i] == NULL) {
            ctx->callbacks[i] = callback;
            ctx->callback_user_data[i] = user_data;
            ESP_LOGI(TAG, "Registered MQTT callback at slot %d", i);
            return ESP_OK;
        }
    }
    
    ESP_LOGE(TAG, "No available callback slots");
    return ESP_ERR_NO_MEM;
}

esp_err_t mqtt_client_unregister_callback(mqtt_client_handle_t handle, mqtt_event_callback_t callback)
{
    struct mqtt_client_context* ctx = (struct mqtt_client_context*)handle;
    
    if (ctx == NULL || !ctx->initialized || callback == NULL) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Find and unregister callback
    for (int i = 0; i < MAX_MQTT_CALLBACKS; i++) {
        if (ctx->callbacks[i] == callback) {
            ctx->callbacks[i] = NULL;
            ctx->callback_user_data[i] = NULL;
            ESP_LOGI(TAG, "Unregistered MQTT callback at slot %d", i);
            return ESP_OK;
        }
    }
    
    ESP_LOGW(TAG, "Callback not found");
    return ESP_ERR_NOT_FOUND;
}

const char* mqtt_client_event_to_string(mqtt_event_t event)
{
    switch (event) {
        case MQTT_EVENT_CONNECTED:              return "CONNECTED";
        case MQTT_EVENT_DISCONNECTED:           return "DISCONNECTED";
        case MQTT_EVENT_CONNECTION_FAILED:      return "CONNECTION_FAILED";
        case MQTT_EVENT_DATA_RECEIVED:          return "DATA_RECEIVED";
        case MQTT_EVENT_PUBLISHED:              return "PUBLISHED";
        case MQTT_EVENT_OFFLINE_QUEUING_STARTED: return "OFFLINE_QUEUING_STARTED";
        case MQTT_EVENT_OFFLINE_SYNC_STARTED:   return "OFFLINE_SYNC_STARTED";
        case MQTT_EVENT_OFFLINE_SYNC_COMPLETED: return "OFFLINE_SYNC_COMPLETED";
        default:                                return "UNKNOWN";
    }
}

const char* mqtt_client_state_to_string(mqtt_connection_state_t state)
{
    switch (state) {
        case MQTT_STATE_DISCONNECTED:       return "DISCONNECTED";
        case MQTT_STATE_CONNECTING:         return "CONNECTING";
        case MQTT_STATE_CONNECTED:          return "CONNECTED";
        case MQTT_STATE_CONNECTION_LOST:    return "CONNECTION_LOST";
        case MQTT_STATE_OFFLINE_QUEUING:    return "OFFLINE_QUEUING";
        default:                            return "UNKNOWN";
    }
}