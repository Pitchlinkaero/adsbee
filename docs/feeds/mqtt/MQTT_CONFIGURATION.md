# MQTT Configuration Guide for ADSBee

## Overview

ADSBee supports MQTT protocol for publishing ADS-B/UAT aircraft data and receiver telemetry to MQTT brokers. This guide covers configuration, setup, and usage.

## Current Status

**IMPORTANT**: MQTT implementation is in development. Core functionality is implemented but AT command configuration for authentication is pending.

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

**Note**: Authentication (username/password) configuration via AT commands is pending implementation. Currently, only public brokers without authentication can be used.

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
- Set via `AT+MQTT_FORMAT0=BINARY`

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

### Home Assistant
```yaml
# configuration.yaml
mqtt:
  sensor:
    - name: "ADSBee Aircraft Count"
      state_topic: "0123456789abcdef/system/telemetry"
      value_template: "{{ value_json.mps_total }}"
      unit_of_measurement: "aircraft/sec"
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

### Pending Implementation

The following AT commands are planned but not yet implemented:
- MQTT authentication (username/password)
- MQTT client ID configuration
- MQTT report mode (Status/Raw/Both)
- Global MQTT enable/disable
- Telemetry/GPS intervals
- Status update rate