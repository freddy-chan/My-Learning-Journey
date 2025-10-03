# Offline-Ready IoT System Integration Guide

This guide explains how to integrate the WiFi Manager, MQTT Client, and OTA Update components into your existing ESP-IDF project.

## Overview

The offline-ready architecture implements the principles described in "Offline-Ready IoT: Turning Disconnection Into Strength" by providing:

1. **Data Modeling**: Local storage and caching of data
2. **Reading Data**: Access to cached information when offline
3. **Writing Data**: Queueing of operations for later synchronization
4. **Conflict Resolution**: Handling of data conflicts during synchronization

## Components Overview

### 1. WiFi Manager (`components/wifi_manager/`)
- Manages WiFi connections and offline mode transitions
- Provides event notifications for connectivity changes
- Handles automatic reconnection and retry logic

### 2. MQTT Client (`components/mqtt_client/`)
- Implements MQTT communication with offline queuing
- Queues messages when offline for later synchronization
- Supports Quality of Service (QoS) levels

### 3. OTA Update (`components/ota_update/`)
- Handles firmware updates with rollback capability
- Defers updates when in offline mode
- Provides progress reporting and status monitoring

## Integration Steps

### Step 1: Update Project CMakeLists.txt

Add the new components to your project's main CMakeLists.txt:

```cmake
# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(practice-project)
```

The ESP-IDF build system will automatically discover the new components in the `components/` directory.

### Step 2: Update Kconfig.projbuild (Optional)

Add configuration options for the new components in your `Kconfig.projbuild`:

```
menu "Network Configuration"
    
    # Existing WiFi and MQTT configuration options...
    
    config OTA_UPDATE_URL
        string "OTA Update Server URL"
        default "https://your-update-server.com/firmware.bin"
        help
            URL for firmware updates. Should point to a secure HTTPS endpoint.
    
    config OTA_AUTO_UPDATE
        bool "Automatically Apply Updates"
        default n
        help
            Automatically download and apply firmware updates when available.
            
endmenu
```

### Step 3: Update main.c

The main application has already been updated with integration code. The key integration points are:

1. **Component Initialization**: Initialize all three components with appropriate configuration
2. **Event Handling**: Register callbacks to handle events from each component
3. **State Management**: Coordinate between components based on connectivity state
4. **Data Synchronization**: Publish sensor data to MQTT when connected
5. **Offline Mode**: Handle transitions between online and offline modes

### Step 4: Build and Test

1. Run `idf.py menuconfig` to configure your WiFi and MQTT settings
2. Build the project with `idf.py build`
3. Flash to your device with `idf.py flash`
4. Monitor output with `idf.py monitor`

## Key Integration Points

### WiFi and MQTT Coordination

The WiFi manager and MQTT client work together:
- When WiFi connects, MQTT client automatically connects
- When WiFi disconnects, MQTT client disconnects
- Offline mode affects both components

### Offline Message Queuing

The MQTT client queues messages when:
- Device is in offline mode
- WiFi connection is lost
- MQTT broker is unreachable

When connectivity is restored, queued messages are synchronized automatically.

### OTA Update Deferral

OTA updates are deferred when:
- Device is in offline mode
- WiFi connection is unavailable
- User has manually enabled offline mode

Updates resume when the device returns to online mode.

## Usage Examples

### Publishing Sensor Data

```c
// Publish sensor data to MQTT when connected
if (g_mqtt_handle && mqtt_client_is_connected(g_mqtt_handle)) {
    char payload[256];
    snprintf(payload, sizeof(payload), 
            "{\"temperature\": %.1f, \"humidity\": %.1f, \"timestamp\": %lu}",
            g_last_sensor_reading.temperature,
            g_last_sensor_reading.humidity,
            (unsigned long)time(NULL));
    
    mqtt_client_publish(g_mqtt_handle, 
                      "smart_logger/sensor_data", 
                      payload, 
                      strlen(payload), 
                      MQTT_QOS_AT_LEAST_ONCE, 
                      false);
}
```

### Handling Offline Mode

```c
// Check if device is in offline mode
if (wifi_manager_is_offline_mode(g_wifi_handle)) {
    // Perform offline-only operations
    // Queue messages for later synchronization
} else {
    // Perform online operations
    // Send messages immediately
}
```

### Checking for Updates

```c
// Check for firmware updates periodically
if (g_ota_handle && !wifi_manager_is_offline_mode(g_wifi_handle)) {
    ota_check_for_updates(g_ota_handle, false);
}
```

## Error Handling

Each component provides comprehensive error handling:
- Detailed error codes for specific failure conditions
- Retry mechanisms for transient failures
- Graceful degradation when services are unavailable
- Logging for debugging and monitoring

## Testing Offline Mode

To test offline capabilities:

1. **Manual Offline Mode**: Call `wifi_manager_enter_offline_mode()`
2. **Network Disconnection**: Physically disconnect WiFi or block network access
3. **Verify Behavior**: 
   - Messages should be queued when offline
   - Queued messages should synchronize when connectivity returns
   - OTA updates should be deferred when offline

## Extending the Architecture

The offline-ready architecture can be extended to support:

1. **Additional Sensors**: Add new sensor components following the same pattern
2. **Data Storage**: Implement local database storage for complex data
3. **Conflict Resolution**: Add sophisticated conflict resolution strategies
4. **User Interface**: Implement UI indicators for offline status
5. **Power Management**: Add power-saving modes for offline operation

## Troubleshooting

### Common Issues

1. **Components Not Found**: Ensure the component directories are in the `components/` folder
2. **Build Errors**: Run `idf.py fullclean` and rebuild if components aren't recognized
3. **Connection Issues**: Verify WiFi credentials and MQTT broker settings
4. **Memory Issues**: Monitor heap usage, especially with message queuing

### Debugging Tips

1. **Enable Verbose Logging**: Set log level to DEBUG for detailed information
2. **Monitor Events**: Use event callbacks to track component state changes
3. **Check Configuration**: Verify all configuration parameters are correct
4. **Test Connectivity**: Ensure network access and broker availability

## Conclusion

The offline-ready IoT architecture provides robust operation in challenging network conditions while maintaining full functionality when connectivity is available. By implementing data modeling, reading/writing strategies, and conflict resolution, your IoT devices can provide reliable service regardless of network status.