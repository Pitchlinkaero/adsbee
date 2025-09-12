# ADSBee MQTT Integration

Stream real-time ADS-B data from both 1090 MHz and 978 MHz (UAT) to any MQTT broker with support for both JSON and binary formats.

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
  "band": "1090",    // "1090" or "UAT" 
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

Topics:
- 1090 MHz: `adsb/{ICAO}/status`
- 978 MHz UAT: `uat/{ICAO}/status`

### Binary Format
Compact 21-byte messages for bandwidth-limited connections.

- Same real-time delivery
- Includes band identification
- Ideal for cellular IoT, satellite, LoRaWAN

Topics (shortened for additional savings):
- 1090 MHz: `a/{ICAO}/s` 
- 978 MHz UAT: `u/{ICAO}/s`

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

## Dual-Band Support

ADSBee supports both frequency bands used for ADS-B:

### 1090 MHz (Mode S / ADS-B)
- International standard
- Commercial aviation
- All aircraft above 18,000 ft
- Topics prefix: `adsb/` (JSON) or `a/` (binary)

### 978 MHz (UAT)
- US-specific below 18,000 ft
- General aviation
- Weather (FIS-B) and traffic (TIS-B) uplinks
- Topics prefix: `uat/` (JSON) or `u/` (binary)

Messages automatically include band identification so you can:
- Filter by frequency band
- Track coverage gaps
- Identify aircraft type (commercial vs GA)
- Separate traffic for different applications

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
    # Handle both 1090 MHz and UAT messages
    if message.topic.startswith(("adsb/", "uat/")):
        # JSON format
        data = json.loads(message.payload)
        band = data.get('band', '1090')
        print(f"[{band}] Aircraft {data['icao']} at {data['alt']} ft")
        
    elif message.topic.startswith(("a/", "u/")):
        # Binary format
        msg_type = message.payload[0]
        if msg_type == 0x02:  # Aircraft message
            band_bits = message.payload[1] >> 6  # Band in upper 2 bits
            band = "UAT" if band_bits == 1 else "1090"
            icao = int.from_bytes(message.payload[1:4], 'big') & 0xFFFFFF
            print(f"[{band}] Aircraft {icao:06X} (binary)")

client = mqtt.Client()
client.on_message = on_message
client.connect("localhost", 1883)

# Subscribe to both bands
client.subscribe("adsb/+/status")  # 1090 MHz JSON
client.subscribe("uat/+/status")   # UAT JSON
client.subscribe("a/+/s")          # 1090 MHz Binary
client.subscribe("u/+/s")          # UAT Binary

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