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

// Memory optimization settings
#define MQTT_MAX_TOPIC_LEN 96          // Reduced from 128
#define MQTT_MAX_PAYLOAD_LEN 8192      // Increased for OTA chunks (4096 + header)

// OTA Settings (pass-through mode)
#define MQTT_OTA_CHUNK_SIZE 2048       // Chunk size for OTA transfers
#define MQTT_OTA_MAX_CHUNK_SIZE 4096   // Maximum chunk size supported
#define MQTT_OTA_COMMAND_TIMEOUT_MS 5000  // Timeout waiting for Pico ACK

#endif // MQTT_CONFIG_H
