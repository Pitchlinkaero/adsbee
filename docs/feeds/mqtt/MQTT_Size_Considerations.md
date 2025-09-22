# Firmware Size Optimization Guide

## Overview
The ADSBee firmware with full MQTT, TLS, and OTA support can be large for the ESP32-S3. This guide explains how to reduce the firmware size by disabling unused features.

## Feature Sizes (Approximate)
- Core MQTT Client: ~20KB
- MQTT OTA Support: ~20KB
- TLS/SSL Support: ~20KB
- JSON Parsing: ~10KB
- Total MQTT Full Stack: ~70KB

## Configuration Options

### 1. Disable MQTT OTA (Save ~20KB)
Edit `/firmware/esp/main/comms/mqtt_config.h`:
```c
#define CONFIG_MQTT_OTA_ENABLED 0  // Was 1
```

This removes:
- OTA manifest handling
- Chunk transfer protocol
- Flash partition management
- Progress reporting

### 2. Disable TLS Support (Save ~20KB)
Edit `/firmware/esp/main/comms/mqtt_config.h`:
```c
#define CONFIG_MQTT_TLS_ENABLED 0  // Was 1
```

This removes:
- Certificate verification
- Encrypted connections
- Forces all MQTT to use unencrypted connections

### 3. Disable MQTT Completely (Save ~70KB)
Edit `/firmware/esp/main/comms/mqtt_config.h`:
```c
#define CONFIG_MQTT_ENABLED 0  // Was 1
```

This removes all MQTT functionality but keeps traditional TCP feeds working.

### 4. Use Binary Protocol Instead of JSON
The binary protocol is already more compact:
- JSON aircraft status: ~150 bytes
- Binary aircraft status: 31 bytes

Set format to binary in feed configuration:
```
AT+FEED=0,broker.example.com,1883,1,MQTT
AT+MQTTFMT=0,BINARY
```

## Build Commands

### Full Build (All Features)
```bash
cd firmware/esp
idf.py build
```

### Minimal Build (No MQTT)
```bash
cd firmware/esp
# Edit mqtt_config.h first
idf.py fullclean
idf.py build
```

### Check Binary Size
```bash
idf.py size
idf.py size-components
```

## Memory Usage Tips

1. **Static vs Dynamic Allocation**: The optimized code uses static buffers where possible to reduce heap fragmentation.

2. **String Optimization**: Topic strings are built on-demand rather than stored.

3. **Conditional Compilation**: Features are completely removed from the binary when disabled.

## Recommended Configurations

### For Basic MQTT (No OTA, No TLS)
```c
#define CONFIG_MQTT_ENABLED 1
#define CONFIG_MQTT_OTA_ENABLED 0
#define CONFIG_MQTT_TLS_ENABLED 0
```
Saves ~40KB vs full build.

### For Secure MQTT (TLS but No OTA)
```c
#define CONFIG_MQTT_ENABLED 1
#define CONFIG_MQTT_OTA_ENABLED 0
#define CONFIG_MQTT_TLS_ENABLED 1
```
Saves ~20KB vs full build.

### For Traditional TCP Only
```c
#define CONFIG_MQTT_ENABLED 0
```
Saves ~70KB, keeps Beast/Raw/Basestation protocols.
