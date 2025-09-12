# ADSBee MQTT Integration

Stream real-time ADS-B data to any MQTT broker with support for both JSON and binary formats.

## Quick Start

### Basic Setup (JSON Format)
```bash
# Configure MQTT feed
AT+FEEDPROTOCOL=0,MQTT
AT+FEEDURI=0,mqtt.broker.com
AT+FEEDPORT=0,1883
AT+FEEDEN=0,1
AT+WRITE
```

### Cellular/IoT Setup (Binary Format)
```bash
# Same as above, but use binary format for 90% bandwidth savings
AT+FEEDPROTOCOL=0,MQTT
AT+FEEDURI=0,mqtt.broker.com
AT+FEEDPORT=0,1883
AT+MQTTFORMAT=0,BINARY
AT+FEEDEN=0,1
AT+WRITE
```

## Message Formats

### JSON Format (Default)
Human-readable, ~250 bytes per message. Best for WiFi/Ethernet.

**Aircraft Status:**
```json
{
  "icao": "A12345",
  "call": "UAL123",
  "lat": 37.7749,
  "lon": -122.4194,
  "alt": 35000,
  "hdg": 270,
  "spd": 485,
  "vr": 0,
  "sqk": "1200",
  "gnd": 0
}
```

Topic: `adsb/{ICAO}/status`

### Binary Format
Compact 20-byte messages for bandwidth-limited connections.

- Same real-time delivery
- Ideal for cellular IoT, satellite, LoRaWAN

Topic: `a/{ICAO}/s` (shortened for additional savings)

## Configuration Commands

### Set Output Format
```bash
AT+MQTTFORMAT=<feed>,<format>

# Examples:
AT+MQTTFORMAT=0,JSON     # Human-readable
AT+MQTTFORMAT=0,BINARY   # Bandwidth-optimized
```

### Query Settings
```bash
AT+MQTTFORMAT?   # Show current format
AT+FEEDPROTOCOL? # Show protocol settings
AT+FEEDURI?      # Show broker address
```

## MQTT Broker Setup

### Mosquitto (Testing)
```bash
# Install
sudo apt install mosquitto mosquitto-clients

# Subscribe to all aircraft
mosquitto_sub -h localhost -t "adsb/+/status" -v

# Binary format (if using short topics)
mosquitto_sub -h localhost -t "a/+/s" -v
```

### Home Assistant
```yaml
mqtt:
  broker: localhost

sensor:
  - platform: mqtt
    name: "Aircraft Count"
    state_topic: "adsb/stats/count"
```

### Python Client
```python
import paho.mqtt.client as mqtt
import json
import struct

def on_message(client, userdata, message):
    if message.topic.startswith("adsb/"):
        # JSON format
        data = json.loads(message.payload)
        print(f"Aircraft {data['icao']} at {data['alt']} ft")
    elif message.topic.startswith("a/"):
        # Binary format - first byte is message type
        msg_type = message.payload[0]
        if msg_type == 0x02:  # Aircraft message
            # Unpack binary structure (example)
            icao = int.from_bytes(message.payload[1:4], 'big')
            print(f"Aircraft {icao:06X} (binary)")

client = mqtt.Client()
client.on_message = on_message
client.connect("localhost", 1883)
client.subscribe("adsb/+/status")  # JSON
client.subscribe("a/+/s")          # Binary
client.loop_forever()
```

## Multiple Feeds

Configure up to 4 simultaneous MQTT feeds:

```bash
# Feed 0: Local broker (JSON)
AT+FEEDPROTOCOL=0,MQTT
AT+FEEDURI=0,192.168.1.100
AT+MQTTFORMAT=0,JSON

# Feed 1: Cloud broker (Binary for cellular)
AT+FEEDPROTOCOL=1,MQTT  
AT+FEEDURI=1,cloud.mqtt.com
AT+MQTTFORMAT=1,BINARY

# Enable both
AT+FEEDEN=0,1
AT+FEEDEN=1,1
AT+WRITE
```

## Authentication

Include credentials in the URI:
```bash
AT+FEEDURI=0,username:password@mqtt.broker.com
```

## TLS/SSL

Use port 8883 for secure connections:
```bash
AT+FEEDPORT=0,8883
```

## Troubleshooting

**No messages received:**
- Check WiFi connection: `AT+WIFISTATUS`
- Verify feed enabled: `AT+FEEDEN?`
- Check broker logs for connection attempts

**High bandwidth usage:**
- Switch to binary format: `AT+MQTTFORMAT=0,BINARY`
- Verify with: `AT+MQTTFORMAT?`

## Support

- GitHub: https://github.com/CoolNamesAllTaken/adsbee
- Website: https://pantsforbirds.com/adsbee-1090