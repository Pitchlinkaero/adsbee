# MQTT Configuration Guide for ADSBee

## Overview

ADSBee supports MQTT protocol for publishing ADS-B/UAT aircraft data and receiver telemetry to MQTT brokers. This guide covers configuration, setup, and usage.

## Quick Start

### Configure MQTT Feed

MQTT is configured using the existing feed system. Use one of the 10 available feed slots (0-9):

**Via AT Commands:**
```bash
# Configure feed 0 for MQTT (all parameters at once)
AT+FEED=0,mqtt://broker.hivemq.com,1883,1,MQTT

# Or configure parameters individually (use commas to skip):
# Just set URI
AT+FEED=0,mqtt://broker.hivemq.com

# Just set port (skip URI)
AT+FEED=0,,1883

# Just set protocol (skip URI, port, active)
AT+FEED=0,,,,MQTT

# Just enable/disable (skip URI, port)
AT+FEED=0,,,1

# Query feed configuration
AT+FEED?0

# Set message format (JSON or BINARY)
AT+MQTTFMT=0,JSON

# Save settings
AT+SETTINGS=SAVE

# Reboot to apply
AT+REBOOT
```

### Feed Command Format

`AT+FEED=<index>,<uri>,<port>,<active>,<protocol>`

- **index**: Feed slot number (0-9)
- **uri**: Broker address (mqtt:// or mqtts://)
- **port**: Broker port (1883 for MQTT, 8883 for MQTTS)
- **active**: 0=disabled, 1=enabled
- **protocol**: BEAST, BEAST_RAW, or MQTT

## Current Implementation Details

### Settings Structure

MQTT settings are stored in the device configuration:
- **Feed slots**: 10 available (0-9)
- **Per-feed settings**: URI, port, active status, protocol
- **MQTT-specific fields** (in code, AT commands pending):
  - Username (32 chars max)
  - Password (64 chars max)
  - Client ID (32 chars max)
  - Format (JSON/Binary)
  - Report mode (Status/Raw/Both)

### Global Settings (hardcoded defaults):
- **Device ID**: Auto-generated from receiver ID (16 hex chars)
- **Telemetry interval**: 60 seconds
- **GPS interval**: 60 seconds
- **Status rate**: 1 Hz per aircraft
- **MQTT enabled**: Via feed protocol selection

## MQTT Topics

ADSBee publishes to the following topics (where `{device_id}` is your 16-character hex device ID):

### JSON Format Topics

| Topic | Description | QoS | Retained |
|-------|-------------|-----|----------|
| `{device_id}/adsb/{ICAO}/status` | 1090 MHz aircraft status | 0 | No |
| `{device_id}/uat/{ICAO}/status` | 978 MHz UAT aircraft status | 0 | No |
| `{device_id}/system/telemetry` | Receiver telemetry | 1 | No |
| `{device_id}/system/gps` | GPS position | 0 | No |
| `{device_id}/system/online` | Online status (LWT) | 1 | Yes |

### Binary Format Topics (Compact)

| Topic | Description | Size |
|-------|-------------|------|
| `{device_id}/a/{ICAO}/s` | 1090 MHz status | 31 bytes |
| `{device_id}/u/{ICAO}/s` | UAT status | 31 bytes |
| `{device_id}/sys/t` | Telemetry | 28 bytes |
| `{device_id}/sys/g` | GPS | 15 bytes |

## Message Formats

### Aircraft Status (JSON)
```json
{
  "icao": "A12345",
  "band": "BAND_1090",
  "call": "UAL123",
  "lat": 37.7749,
  "lon": -122.4194,
  "alt_ft": 35000,
  "hdg_deg": 270,
  "spd_kts": 450,
  "vr_fpm": 0,
  "sqk": "1200",
  "cat": "A3",
  "on_ground": false,
  "rssi_dbm": -65,
  "ts": 1234567890
}
```

### Telemetry (JSON)
```json
{
  "uptime_sec": 3600,
  "msgs_rx": 150000,
  "msgs_tx": 145000,
  "cpu_temp_c": 45,
  "mem_free_kb": 32768,
  "noise_floor_dbm": -105,
  "rx_1090": true,
  "rx_978": false,
  "wifi": true,
  "mqtt": true,
  "fw_version": "0.8.3",
  "mps_total": 250,
  "demods_1090": 1000,
  "raw_squitter_frames": 800,
  "valid_squitter_frames": 750,
  "raw_extended_squitter": 600,
  "valid_extended_squitter": 580
}
```

## Complete Configuration Example

### Step-by-step MQTT Setup
```bash
# 1. Configure MQTT feed
AT+FEED=0,mqtt://192.168.7.65,1883,1,MQTT

# 2. Set format (optional, defaults to JSON)
AT+MQTTFMT=0,JSON      # For human-readable
# OR
AT+MQTTFMT=0,BINARY    # For compact messages

# 3. Verify configuration
AT+FEED?0              # Check feed settings
AT+MQTTFMT?0           # Check format setting

# 4. Save settings
AT+SETTINGS=SAVE

# 5. Reboot to apply
AT+REBOOT

# 6. Monitor console for connection status
# Look for: "Initialized MQTT client for feed 0"
```

## Example Broker Configurations

### Mosquitto Test Server (Public, No Auth)
```bash
# Configure feed 0 for public Mosquitto test server
AT+FEED=0,mqtt://test.mosquitto.org,1883,1,MQTT

# Query to verify
AT+FEED?0

# Save settings
AT+SETTINGS=SAVE

# Reboot to apply
AT+REBOOT
```

### Local Mosquitto Server
```bash
# Configure feed 2 for local broker
AT+FEED=2,mqtt://192.168.1.100,1883,1,MQTT


# Save settings
AT+SETTINGS=SAVE

# Reboot to apply
AT+REBOOT
```

## Authentication

### Configure Username/Password
```bash
# Set authentication for feed 0
AT+MQTTAUTH=0,myusername,mypassword

# Query authentication (password shown as ****)
AT+MQTTAUTH?0

# Clear authentication
AT+MQTTAUTH=0
```

## TLS/SSL Security

### Configure TLS Verification Mode
```bash
# Set TLS mode for feed 0
AT+MQTTTLS=0,STRICT    # Full verification (recommended)
AT+MQTTTLS=0,VERIFY    # CA verification only
AT+MQTTTLS=0,NONE      # No verification (testing only)

# Query TLS mode
AT+MQTTTLS?0
```

**TLS Modes:**
- `STRICT`: Full certificate and hostname verification (production)
- `VERIFY`: CA certificate verification without hostname check
- `NONE`: No verification (development/testing only, insecure)

### Secure Connection Example
```bash
# Configure secure MQTT with authentication
AT+FEED=0,mqtts://broker.hivemq.com,8883,1,MQTT
AT+MQTTTLS=0,STRICT
AT+MQTTAUTH=0,username,password
AT+MQTTFMT=0,JSON
AT+SETTINGS=SAVE
AT+REBOOT
```

## Testing Your Configuration

### 1. Using Mosquitto Client
Subscribe to all topics from your device:
```bash
# Subscribe to all topics from your device
mosquitto_sub -h test.mosquitto.org -t "0123456789abcdef/#" -v

# Subscribe to telemetry only
mosquitto_sub -h test.mosquitto.org -t "0123456789abcdef/system/telemetry" -v

# Subscribe to all aircraft
mosquitto_sub -h test.mosquitto.org -t "0123456789abcdef/+/+/status" -v
```

### 2. Using MQTT Explorer
1. Download [MQTT Explorer](http://mqtt-explorer.com/)
2. Connect to your broker
3. Navigate to your device ID in the topic tree
4. Watch real-time updates

### 3. Verify in ADSBee Console
Check MQTT status via serial console:
```
MQTT: Connected to mqtt://test.mosquitto.org:1883
MQTT: Published status for A12345 (1090 MHz)
MQTT: Published telemetry (28 bytes)
```

## Performance Tuning

### Rate Limiting
- Status updates: 1 Hz per aircraft by default
- Configurable 1-2 Hz via `AT+MQTT_STATUS_RATE`
- Automatic dropping of intermediate updates under load

### Binary vs JSON
- **JSON**: Human-readable, larger (200-300 bytes per message)
- **Binary**: Compact (15-31 bytes), better for bandwidth-limited connections
- Set via `AT+MQTTFMT=0,BINARY` or `AT+MQTTFMT=0,JSON`

### Connection Resilience
- Automatic reconnection with exponential backoff
- Base: 1 second, Max: 60 seconds
- ±20% jitter to prevent thundering herd

## Troubleshooting

### Not Connecting
1. Check network connectivity (ensure WiFi is connected)
2. Verify broker URI format: `mqtt://` not `http://`
3. Check firewall allows port 1883/8883
4. Currently only public brokers without authentication work

### No Messages Published
1. Check feed is active: `AT+FEED?<index>` should show active=1
2. Verify protocol is MQTT: `AT+FEED?<index>` should show protocol=MQTT
3. Check aircraft are being received
4. Monitor serial console for MQTT connection messages

### Settings Not Persisting
After configuration changes:
1. Save settings: `AT+SETTINGS=SAVE`
2. Reboot device: `AT+REBOOT`
3. Device will restart with new configuration
4. Verify with `AT+FEED?` after restart

## Security Considerations

### TLS/SSL (MQTTS)
- Use `mqtts://` URI scheme
- Port 8883 typically
- CA certificate validation enabled
- **Note**: Client certificates not yet supported

### Authentication
- Always use username/password with cloud brokers
- Passwords stored in device flash (not encrypted)
- Consider using dedicated MQTT user with limited permissions

### Topic Security
- Device ID provides basic namespace isolation
- Consider broker ACLs to restrict topic access
- Monitor for unauthorized subscriptions

## Integration Examples

### Node-RED
```javascript
// Flow to process ADSBee MQTT data
msg.topic = "0123456789abcdef/+/+/status";
// Parse aircraft data
let aircraft = JSON.parse(msg.payload);
// Process...
```

### Python
```python
import paho.mqtt.client as mqtt
import json

def on_message(client, userdata, msg):
    if "status" in msg.topic:
        aircraft = json.loads(msg.payload)
        print(f"Aircraft {aircraft['icao']} at {aircraft['alt_ft']} ft")

client = mqtt.Client()
client.on_message = on_message
client.connect("test.mosquitto.org", 1883)
client.subscribe("0123456789abcdef/#")
client.loop_forever()
```

## AT Command Reference

### Current AT Commands for MQTT

| Command | Description | Example |
|---------|-------------|---------|
| `AT+FEED?` | Query all feed configurations | `AT+FEED?` |
| `AT+FEED?<n>` | Query specific feed | `AT+FEED?0` |
| `AT+FEED=` | Set feed configuration | `AT+FEED=0,mqtt://broker,1883,1,MQTT` |
| `AT+MQTTFMT?` | Query all feed formats | `AT+MQTTFMT?` |
| `AT+MQTTFMT?<n>` | Query specific feed format | `AT+MQTTFMT?0` |
| `AT+MQTTFMT=` | Set MQTT message format | `AT+MQTTFMT=0,JSON` or `AT+MQTTFMT=0,BINARY` |
| `AT+SETTINGS=SAVE` | Save current settings to flash | `AT+SETTINGS=SAVE` |
| `AT+REBOOT` | Reboot the device | `AT+REBOOT` |

### Feed Configuration Parameters

When using `AT+FEED=<index>,<uri>,<port>,<active>,<protocol>`:
- Leave parameter empty to skip (e.g., `AT+FEED=0,,,,MQTT` only sets protocol)
- Parameters are positional, use commas to skip

### Supported Protocols
- `BEAST` - Beast binary protocol
- `BEAST_RAW` - Beast raw protocol
- `MQTT` - MQTT protocol
- Empty/not set - No reports

### Format Selection

The MQTT client supports both JSON and binary formats. Use the `AT+MQTTFMT` command to switch between formats:

**Commands**:
```bash
# Set format for feed 0
AT+MQTTFMT=0,JSON    # Human-readable JSON
AT+MQTTFMT=0,BINARY  # Compact binary format

# Query format
AT+MQTTFMT?0         # Check feed 0 format
AT+MQTTFMT?          # Check all feed formats
```

**Format comparison**:
- **JSON**: ~200-300 bytes, human-readable, standard topics
- **Binary**: 31 bytes (status), compact, requires decoder

## Complete AT Command Reference

### Feed Configuration
| Command | Description | Example |
|---------|-------------|---------|
| `AT+FEED?` | Query all feeds | `AT+FEED?` |
| `AT+FEED?<n>` | Query specific feed | `AT+FEED?0` |
| `AT+FEED=<n>,<uri>,<port>,<active>,<protocol>` | Configure feed | `AT+FEED=0,mqtt://broker.hivemq.com,1883,1,MQTT` |
| `AT+FEEDEN=<n>,<0\|1>` | Enable/disable feed | `AT+FEEDEN=0,1` |
| `AT+FEEDPROTOCOL=<n>,<protocol>` | Set protocol | `AT+FEEDPROTOCOL=0,MQTT` |

### MQTT-Specific Configuration
| Command | Description | Example |
|---------|-------------|---------|
| `AT+MQTTFMT?` | Query all format settings | `AT+MQTTFMT?` |
| `AT+MQTTFMT?<n>` | Query feed format | `AT+MQTTFMT?0` |
| `AT+MQTTFMT=<n>,<format>` | Set format (JSON/BINARY) | `AT+MQTTFMT=0,JSON` |
| `AT+MQTTAUTH?` | Query all auth settings | `AT+MQTTAUTH?` |
| `AT+MQTTAUTH?<n>` | Query feed auth | `AT+MQTTAUTH?0` |
| `AT+MQTTAUTH=<n>,<user>,<pass>` | Set authentication | `AT+MQTTAUTH=0,user,pass` |
| `AT+MQTTAUTH=<n>` | Clear authentication | `AT+MQTTAUTH=0` |
| `AT+MQTTTLS?` | Query all TLS settings | `AT+MQTTTLS?` |
| `AT+MQTTTLS?<n>` | Query feed TLS mode | `AT+MQTTTLS?0` |
| `AT+MQTTTLS=<n>,<mode>` | Set TLS mode | `AT+MQTTTLS=0,STRICT` |
| `AT+MQTTOTA?` | Query all OTA settings | `AT+MQTTOTA?` |
| `AT+MQTTOTA?<n>` | Query feed OTA status | `AT+MQTTOTA?0` |
| `AT+MQTTOTA=<n>,<0\|1>` | Enable/disable OTA | `AT+MQTTOTA=0,1` |

### System Commands
| Command | Description | Example |
|---------|-------------|---------|
| `AT+SETTINGS=SAVE` | Save configuration to flash | `AT+SETTINGS=SAVE` |
| `AT+REBOOT` | Reboot device | `AT+REBOOT` |
| `AT+SETTINGS?` | Show all settings | `AT+SETTINGS?` |

## MQTT OTA Updates

### Overview

ADSBee supports Over-The-Air (OTA) firmware updates via MQTT. This allows remote firmware updates without physical access to the device.

**WARNING**: OTA updates can brick your device if interrupted. Ensure:
- Stable power supply
- Reliable network connection
- Correct firmware file
- Backup of current configuration

### Enabling OTA

```bash
# Enable OTA for feed 0 (disabled by default for safety)
AT+MQTTOTA=0,1
AT+SETTINGS=SAVE
AT+REBOOT

# Check OTA status
AT+MQTTOTA?0
```

### OTA Process

1. **Device subscribes to OTA topics**:
   - `{device_id}/ota/control/manifest` - Firmware metadata
   - `{device_id}/ota/control/command` - Control commands
   - `{device_id}/ota/data/chunk/+` - Firmware chunks

2. **Server publishes manifest** containing:
   - Firmware version
   - Total size and number of chunks
   - CRC/SHA256 for verification

3. **Server sends START command**
   - Device erases flash partition
   - Prepares for chunk reception

4. **Server publishes firmware chunks**
   - Each chunk has CRC32 verification
   - Device ACKs successful chunks
   - Server retries failed chunks

5. **Verification and boot**
   - Device verifies complete firmware
   - Server sends BOOT command
   - Device reboots to new firmware

### Topic Structure

#### Subscribe (Device listens):
- `{device_id}/ota/control/manifest` - Firmware manifest
- `{device_id}/ota/control/command` - Control commands
- `{device_id}/ota/data/chunk/{n}` - Firmware chunks

#### Publish (Device sends):
- `{device_id}/ota/status/state` - Current OTA state
- `{device_id}/ota/status/progress` - Download progress
- `{device_id}/ota/status/ack/{n}` - Chunk acknowledgments

### Security Considerations

- **Disabled by default** - Must explicitly enable via AT command
- **TLS recommended** - Use MQTTS for encrypted transfer
- **Authentication required** - Use strong credentials
- **CRC verification** - Each chunk is verified
- **Session IDs** - Prevent replay attacks
- **Rollback protection** - Previous firmware preserved

### Example Configuration

```bash
# Secure OTA setup
AT+FEED=0,mqtts://broker.example.com,8883,1,MQTT
AT+MQTTTLS=0,STRICT
AT+MQTTAUTH=0,device-ota,secure-password
AT+MQTTOTA=0,1
AT+SETTINGS=SAVE
AT+REBOOT
```

## Pending Features

The following features are planned for future releases:
- Custom client ID configuration
- Raw packet publishing mode
- GPS position publishing
- Custom CA certificate upload
- Client certificate authentication
- Configurable QoS levels
- Custom topic prefixes
- OTA server tools and scripts

## Version History

### 0.8.2-RC13 - Size Optimizations
- Added conditional compilation for all MQTT features
- OTA support can be disabled to save ~20KB (CONFIG_MQTT_OTA_ENABLED)
- TLS support can be disabled to save ~20KB (CONFIG_MQTT_TLS_ENABLED)
- Entire MQTT stack can be disabled to save ~70KB (CONFIG_MQTT_ENABLED)
- Optimized string handling with static buffers
- Reduced default OTA chunk size from 4KB to 1KB
- See FIRMWARE_SIZE_OPTIMIZATION.md for configuration details
