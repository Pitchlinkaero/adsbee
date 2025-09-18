# MQTT OTA (Over-The-Air) Update Guide

## Overview

The ADSBee MQTT OTA system enables remote firmware updates via MQTT protocol. The implementation uses a **pass-through architecture** where the ESP32 forwards OTA commands and data to the Pico (RP2040) processor, which handles the actual firmware storage and update process.

## Architecture

### Pass-Through Design
```
[MQTT Broker] <--MQTT--> [ESP32] <--SPI/AT Commands--> [Pico/RP2040]
                            |                              |
                         (Forwards)                   (Stores & Flashes)
```

**Benefits:**
- No ESP32 flash storage needed for OTA
- Leverages existing Pico dual-partition OTA system
- Simplified ESP32 code
- More reliable single-point flash management
- Saves ~12MB of ESP32 flash space

### Components

1. **MQTT Broker** - Distributes firmware chunks and commands
2. **ESP32** - Receives MQTT messages, forwards to Pico via AT commands
3. **Pico (RP2040)** - Manages dual-partition OTA, performs actual firmware update
4. **Python Publisher** - Sends firmware to device via MQTT

## Device Configuration

### Enable MQTT OTA

1. **Configure MQTT connection:**
```
AT+FEED=0,192.168.1.100,1883,1,MQTT
AT+MQTTDEVICE=bee0003a59b3356e
```

2. **Optional: Configure authentication:**
```
AT+MQTTAUTH=username,password
```

3. **Optional: Enable TLS:**
```
AT+MQTTTLS=VERIFY
```

4. **Save settings:**
```
AT+SAVE
```

## Firmware Preparation

The firmware must be in `.ota` format, which includes partition information for the dual-partition system.

### Building OTA Firmware
```bash
cd firmware/esp
idf.py build

cd ../pico
./build.sh --ota
# Creates: build/adsbee_1090.ota
```

## Python OTA Publisher

### Installation
```bash
pip install paho-mqtt
```

### Basic Usage
```bash
python3 mqtt_ota_publisher.py \
    --broker localhost \
    --device bee0003a59b3356e \
    firmware.ota
```

### Advanced Options
```bash
python3 mqtt_ota_publisher.py \
    --broker broker.hivemq.com \
    --port 1883 \
    --device bee0003a59b3356e \
    --username myuser \
    --password mypass \
    --chunk-size 4096 \
    --pre-reboot \
    --auto-boot \
    firmware.ota
```

#### Command-line Arguments:
- `--broker` - MQTT broker hostname (required)
- `--port` - MQTT broker port (default: 1883)
- `--device` - Target device ID (required)
- `--username` - MQTT username for authentication
- `--password` - MQTT password for authentication
- `--tls` - Enable TLS/SSL connection
- `--chunk-size` - Size of firmware chunks in bytes (default: 4096, max: 8192)
- `--pre-reboot` - Reboot device before OTA for clean state
- `--auto-boot` - Automatically reboot to new firmware after verification
- `--version` - Firmware version string (default: 0.8.3)

## MQTT Topics

### Control Topics (Publisher → Device)
- `{device_id}/ota/control/manifest` - Firmware manifest with metadata
- `{device_id}/ota/control/command` - Control commands (START, PAUSE, RESUME, ABORT, VERIFY, BOOT, REBOOT, GET_PARTITION)
- `{device_id}/ota/data/chunk/{index}` - Firmware data chunks

### Status Topics (Device → Publisher)
- `{device_id}/ota/status/state` - Current OTA state
- `{device_id}/ota/status/progress` - Download progress
- `{device_id}/ota/status/ack/{index}` - Chunk acknowledgments
- `{device_id}/ota/status/manifest_ack` - Manifest received acknowledgment
- `{device_id}/telemetry` - Device telemetry (used for online detection)

## OTA Process Flow

### 1. Pre-flight Checks
```
Publisher                 Device
    |                        |
    |--Check Connectivity--->|
    |<----Telemetry----------|
    |                        |
    |--Optional: REBOOT----->|
    |      (wait 60s)        |
    |<----Telemetry----------|
```

### 2. Manifest Exchange
```
Publisher                 Device
    |                        |
    |--Publish Manifest----->|
    |<--Manifest ACK---------|
    |                        |
    |--Send START command--->|
    |<--State: DOWNLOADING---|
```

### 3. Firmware Transfer
```
Publisher                 Device                 Pico
    |                        |                     |
    |--Chunk[0]------------->|--AT+OTA=WRITE------>|
    |<--ACK[0]---------------|<----OK--------------|
    |                        |                     |
    |--Chunk[1]------------->|--AT+OTA=WRITE------>|
    |<--ACK[1]---------------|<----OK--------------|
    |        ...             |       ...           |
    |--Chunk[N]------------->|--AT+OTA=WRITE------>|
    |<--ACK[N]---------------|<----OK--------------|
    |                        |                     |
    |<--State: VERIFYING-----|                     |
```

### 4. Verification & Boot
```
Publisher                 Device                 Pico
    |                        |                     |
    |                        |--AT+OTA=VERIFY----->|
    |                        |<--SHA256 Match------|
    |<--State: READY_TO_BOOT-|                     |
    |                        |                     |
    |--Send BOOT command---->|--AT+OTA=BOOT------->|
    |                        |    (Device Reboots) |
```

## OTA States

- **IDLE** - No OTA in progress
- **MANIFEST_RECEIVED** - Manifest received and validated
- **ERASING** - Erasing flash partition (Pico side)
- **DOWNLOADING** - Receiving firmware chunks
- **VERIFYING** - Verifying firmware integrity
- **READY_TO_BOOT** - Firmware verified, ready to boot
- **ERROR** - OTA failed

## AT Commands (Pico Side)

The Pico implements these AT commands for OTA:

- `AT+OTA?` - Query OTA status
- `AT+OTA=GET_PARTITION` - Get target partition number
- `AT+OTA=ERASE` - Erase complementary partition
- `AT+OTA=ERASE,offset,length` - Partial erase
- `AT+OTA=WRITE,offset,length,crc` - Write firmware chunk
- `AT+OTA=VERIFY,sha256` - Verify firmware SHA256
- `AT+OTA=BOOT` - Boot from new partition

## Troubleshooting

### Device Not Responding
1. Check device ID matches exactly (including leading zeros)
2. Verify MQTT connection: `AT+STATUS`
3. Check telemetry is being published
4. Ensure device has network connectivity

### OTA Fails Immediately
1. Check partition configuration on Pico
2. Verify firmware size fits in partition (< 6MB)
3. Check ESP32 console for error messages
4. Ensure MQTT_OTA_ENABLED=1 in firmware

### Chunks Not Acknowledged
1. Check chunk size (max 8192 bytes)
2. Verify SPI communication between ESP32 and Pico
3. Check network console queue isn't full
4. Monitor Pico serial output for AT command responses

### Verification Fails
1. Ensure complete firmware file
2. Check SHA256 calculation matches
3. Verify no data corruption during transfer
4. Check CRC32 on individual chunks

## Configuration Files

### `/firmware/esp/main/comms/mqtt_config.h`
```c
#define CONFIG_MQTT_OTA_ENABLED 1        // Enable OTA support
#define CONFIG_MQTT_OTA_PASSTHROUGH 1    // Use pass-through mode
#define MQTT_OTA_CHUNK_SIZE 4096         // Default chunk size
#define MQTT_OTA_MAX_CHUNK_SIZE 8192     // Maximum chunk size
```

### Python Publisher Defaults
```python
chunk_size = 4096        # Default chunk size
pre_reboot = False       # Don't reboot before OTA
auto_boot = False        # Don't auto-boot after verification
timeout = 60             # Seconds to wait for device
```

## Security Considerations

1. **Use TLS when possible** - Encrypt firmware in transit
2. **Implement authentication** - Use MQTT username/password
3. **Verify firmware signatures** - Implement signing (future enhancement)
4. **Network isolation** - Use separate MQTT topics/broker for OTA
5. **Rate limiting** - Implement chunk rate limiting to prevent DoS

## Example Session

```bash
$ python3 mqtt_ota_publisher.py --broker localhost --device bee0003a59b3356e \
    --pre-reboot --auto-boot firmware.ota

Connecting to localhost:1883...
Connected to localhost:1883
Subscribed to bee0003a59b3356e/ota/status/state
Subscribed to bee0003a59b3356e/ota/status/progress
Subscribed to bee0003a59b3356e/ota/status/ack/+
Subscribed to bee0003a59b3356e/telemetry
Checking device connectivity...
✓ Device is online
Rebooting device to clean state...
Waiting for device to come back online (up to 60s)...
✓ Device back online after reboot
Loaded firmware: firmware.ota
  Size: 4,915,580 bytes
  Chunks: 1201 x 4096 bytes
Publishing manifest for version 0.8.3
Waiting for device to process manifest...
Sending command: START
Waiting for device to start download...
Device state: DOWNLOADING
Publishing chunks...
Progress: [████████████████████] 100% (1201/1201)
All chunks published successfully
Waiting for device to verify firmware...
Device state: VERIFYING
Device state: READY_TO_BOOT
Firmware verified successfully!
Device rebooting with new firmware...

✓ OTA update completed successfully!
```
