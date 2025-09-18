/**
 * MQTT Feature Configuration
 *
 * This file controls which MQTT features are compiled into the firmware.
 * Disabling unused features can significantly reduce binary size.
 */

#ifndef MQTT_CONFIG_H
#define MQTT_CONFIG_H

// Core MQTT support (required for basic MQTT functionality)
// Set to 0 to completely disable MQTT and save ~40KB
#define CONFIG_MQTT_ENABLED 1

// MQTT OTA (Over-The-Air) update support
// Pass-through mode: ESP32 forwards OTA to Pico via AT commands
// No local flash storage needed on ESP32
#define CONFIG_MQTT_OTA_ENABLED 1
#define CONFIG_MQTT_OTA_PASSTHROUGH 1  // Use pass-through to Pico instead of local storage

// MQTT TLS/SSL support
// Disable this to save ~20KB if only using unencrypted MQTT connections
#define CONFIG_MQTT_TLS_ENABLED 1

// Advanced MQTT features
#define CONFIG_MQTT_RAW_PACKETS 0     // Raw packet publishing (not implemented)
#define CONFIG_MQTT_GPS_PUBLISH 0     // GPS position publishing (not implemented)

// MQTT Lite Mode - Ultra compact implementation
// Enable this to use simplified MQTT with static buffers (saves ~10KB)
#define CONFIG_MQTT_LITE_MODE 0

// Memory optimization settings
#define MQTT_MAX_TOPIC_LEN 96          // Reduced from 128
#define MQTT_MAX_PAYLOAD_LEN 8192      // Increased for OTA chunks (4096 + header)

// OTA Settings (pass-through mode)
#define MQTT_OTA_CHUNK_SIZE 512        // Small chunk size to avoid broker limits
#define MQTT_OTA_MAX_CHUNK_SIZE 1024   // Maximum chunk size to stay under 1KB limit
#define MQTT_OTA_COMMAND_TIMEOUT_MS 5000  // Timeout waiting for Pico ACK

// Buffer pool sizes (only used if not in LITE mode)
#define MQTT_JSON_BUFFER_SIZE 512      // For JSON serialization
#define MQTT_BINARY_BUFFER_SIZE 128    // For binary messages

#endif // MQTT_CONFIG_H