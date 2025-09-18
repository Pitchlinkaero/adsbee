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
// Disable this to save ~20KB of flash if OTA via MQTT is not needed
// CONFIG_MQTT_ENABLED=1
#define CONFIG_MQTT_OTA_ENABLED 0

// MQTT TLS/SSL support
// Disable this to save ~20KB if only using unencrypted MQTT connections
#define CONFIG_MQTT_TLS_ENABLED 1

// Advanced MQTT features
#define CONFIG_MQTT_RAW_PACKETS 0     // Raw packet publishing (not implemented)
#define CONFIG_MQTT_GPS_PUBLISH 0     // GPS position publishing (not implemented)

// Memory optimization settings
#define MQTT_MAX_TOPIC_LEN 128        // Maximum topic length
#define MQTT_MAX_PAYLOAD_LEN 4096     // Maximum payload size
#define MQTT_CHUNK_SIZE 1024           // Reduced from 4096 for OTA chunks

#endif // MQTT_CONFIG_H