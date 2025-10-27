# ADSBee GNSS Configuration Guide

## Overview

The ADSBee supports multiple GNSS (GNSS) sources and protocols with advanced configuration options for high-precision positioning. This guide covers all available GNSS configuration commands and options.

## Quick Start

### Basic GNSS Configuration
```bash
# Check current GNSS status
AT+GNSS_STATUS?

# Set GNSS source to UART with auto-detection
AT+GNSS_CONFIG=UART,AUTO,1

# Enable PPP for high precision (if supported)
AT+GNSS_PPP=AUTO

# Set failover timeout
AT+GNSS_FAILOVER=10
```

## GNSS Sources

The ADSBee supports multiple GNSS data sources with automatic failover:

### Priority Order (Highest to Lowest)
1. **UART** - Direct connection to GNSS receiver
2. **Network** - GNSS over TCP/UDP (NTRIP, etc.)
3. **MAVLink** - GNSS data from autopilot/flight controller

### Source Configuration
```bash
AT+GNSS_CONFIG=<source>,<protocol>,<rate>
```

**Parameters:**
- `source`: `UART`, `NETWORK`, `MAVLINK`, `AUTO`
- `protocol`: `AUTO`, `NMEA`, `UBX`, `SBF`, `RTCM`
- `rate`: Update rate in Hz (1-10)

**Examples:**
```bash
# Auto-detect everything
AT+GNSS_CONFIG=AUTO,AUTO,1

# u-blox receiver on UART
AT+GNSS_CONFIG=UART,UBX,5

# Septentrio receiver
AT+GNSS_CONFIG=UART,SBF,1

# Network RTCM corrections
AT+GNSS_CONFIG=NETWORK,RTCM,1
```

## Protocol Support

### NMEA (Text Format)
- **Use Case**: Standard GNSS receivers, simple applications
- **Accuracy**: 3-5 meters typical
- **Supported Messages**: GGA, RMC, GSA, GSV, VTG, ZDA

### UBX (u-blox Binary)
- **Use Case**: u-blox receivers (F9P, F9H, M8n, M10, etc.)
- **Accuracy**: Sub-meter with RTK, centimeter with PPP
- **Features**: RTK, PPP (PointPerfect), multi-band

### SBF (Septentrio Binary Format)
- **Use Case**: Septentrio receivers (mosaic-X5, AsteRx, etc.)
- **Accuracy**: Centimeter-level with PPP
- **Features**: PPP (multiple services), RTK, multi-constellation

### RTCM (Correction Data)
- **Use Case**: Real-time corrections for RTK
- **Source**: Base stations, NTRIP casters
- **Versions**: RTCM 3.x supported

## High-Precision Positioning

### PPP (Precise Point Positioning)
PPP provides improved accuracy without base stations using free satellite-based correction services.

```bash
# Check PPP support
AT+GNSS_PPP?

# Enable auto PPP selection
AT+GNSS_PPP=AUTO

# Available free PPP services
AT+GNSS_PPP=SBAS           # WAAS/EGNOS/MSAS - 1-2m accuracy, instant
AT+GNSS_PPP=GALILEO_HAS    # Galileo HAS - 20cm accuracy, 10-15min convergence
AT+GNSS_PPP=IGS_RTS        # IGS Real-Time Service - 10-20cm accuracy, 15-20min convergence
AT+GNSS_PPP=BEIDOU_B2B     # BeiDou PPP-B2b - 10-30cm accuracy (Asia-Pacific region)
```

**Available PPP Services:**

| Service | Accuracy | Convergence | Coverage | Requirements |
|---------|----------|-------------|----------|--------------|
| **SBAS** | 1-2m | Instant | Regional (WAAS/EGNOS/MSAS) | Standard GNSS receiver |
| **Galileo HAS** | 20cm | 10-15min | Global | E6-capable receiver (mosaic-X5) |
| **IGS_RTS** | 10-20cm | 15-20min | Global | Multi-frequency receiver |
| **BEIDOU_B2B** | 10-30cm | 10-15min | Asia-Pacific | BeiDou B2b-capable receiver |
| **AUTO** | Best Available | Varies | Global | Auto-selects best service for receiver |

**Service Details:**
- **SBAS (WAAS/EGNOS/MSAS)**: Satellite-based augmentation system providing immediate 1-2m accuracy improvement
- **Galileo HAS**: High Accuracy Service providing free 20cm accuracy globally via Galileo E6 signal
- **IGS_RTS**: International GNSS Service Real-Time Service providing free 10-20cm accuracy via SSR corrections
- **BeiDou PPP-B2b**: Free PPP service for Asia-Pacific region via BeiDou B2b signal
- **AUTO**: Automatically selects the best available service for your receiver and location

### RTK (Real-Time Kinematic)
RTK provides centimeter accuracy with base station corrections.

```bash
# Enable RTK mode (if supported)
AT+GNSS_RTK=ENABLE

# Set NTRIP settings for RTK corrections
AT+GNSS_NETWORK=rtk.example.com,2101,MOUNT01,user,pass
```

## Failover Configuration

Configure automatic source switching when GNSS signal is lost:

```bash
# Set failover timeout (5-30 seconds)
AT+GNSS_FAILOVER=15

# Query current failover settings
AT+GNSS_FAILOVER?
# Returns: =15,ENABLED
```

**Failover Behavior:**
1. Primary source fails → Wait for timeout
2. Switch to next priority source
3. Continue monitoring all sources
4. Return to higher priority when available

## Network GNSS Configuration

Configure GNSS over network (NTRIP, TCP, UDP):

```bash
AT+GNSS_NETWORK=<host>,<port>,<mountpoint>,<user>,<pass>
```

**Examples:**
```bash
# Public NTRIP caster
AT+GNSS_NETWORK=rtk.example.com,2101,STATION01,user,password

# Raw TCP stream
AT+GNSS_NETWORK=GNSS.server.com,8080,,,,

# Query current network settings
AT+GNSS_NETWORK?
```

## GNSS Status and Diagnostics

### Real-time Status
```bash
AT+GNSS_STATUS?
```

**Example Output:**
```
GNSS Status Report:
  Source: UART (SBF parser)
  Fix: PPP Converged (6 satellites)
  Position: 48.117300, 11.516700 @ 515.2m
  Accuracy: H=0.08m, V=0.12m
  PPP: Galileo HAS (Converged 95%, ETA: 0s)
  HDOP: 1.2, PDOP: 1.8
  Last Update: 1.2s ago
  
Parser Statistics:
  Messages: 1,247 (Errors: 0)
  Uptime: 00:05:23
```

### Position Query
```bash
AT+GNSS_POSITION?
```

**Example Output:**
```
+GNSS_POSITION=48.117300,11.516700,515.2,PPP_CONVERGED,8,0.08
# Format: lat,lon,alt,fix_type,satellites,accuracy_m
```

## Advanced Configuration

### Static Mode
For stationary applications, enable static mode for better accuracy:

```bash
# Enable static hold with 2m threshold
AT+GNSS_STATIC=ENABLE,2.0

# Disable static mode
AT+GNSS_STATIC=DISABLE
```

### Update Rate Optimization
Higher rates consume more power and processing:

```bash
# Conservative: 1Hz for most applications
AT+GNSS_CONFIG=AUTO,AUTO,1

# Active tracking: 5Hz for moving applications
AT+GNSS_CONFIG=AUTO,AUTO,5

# High-rate: 10Hz for research/development
AT+GNSS_CONFIG=AUTO,AUTO,10
```

### SBAS Configuration
SBAS provides 1-2m accuracy improvement instantly:

```bash
# Enable SBAS (WAAS/EGNOS/MSAS)
AT+GNSS_SBAS=ENABLE

# Disable SBAS
AT+GNSS_SBAS=DISABLE
```

### Debug Output Configuration
Enable detailed GNSS message debugging with two levels of output:

```bash
# Query current debug settings
AT+GNSS_DEBUG?
# Returns: =0,0 (decoded_msgs,raw_hex)

# Enable decoded message output (WARNING level - always visible)
AT+GNSS_DEBUG=1,0

# Enable raw hex output (INFO level - requires AT+LOG_LEVEL=INFO)
AT+GNSS_DEBUG=0,1

# Enable both outputs
AT+GNSS_DEBUG=1,1

# Disable all debug output
AT+GNSS_DEBUG=0,0
```

**Debug Output Levels:**

1. **Decoded Messages** (Parameter 1: `0` or `1`)
   - **Log Level**: WARNING (visible at default log level)
   - **Content**: Parsed GNSS information from each message
   - **Use Case**: Monitor position updates without excessive data
   - **Example Output**:
     ```
     GNSS: GNGGA | Fix:3 Sats:12 Lat:48.117300 Lon:11.516700 Alt:515.2m HDOP:1.20
     GNSS: GNRMC | Fix:3 Sats:12 Lat:48.117301 Lon:11.516702 Alt:515.3m HDOP:1.19
     ```

2. **Raw Hex Output** (Parameter 2: `0` or `1`)
   - **Log Level**: INFO (requires `AT+LOG_LEVEL=INFO`)
   - **Content**: Raw bytes and ASCII interpretation of all GNSS data
   - **Use Case**: Protocol debugging, troubleshooting parser issues
   - **Example Output**:
     ```
     GNSS RAW [64 bytes]: 24 47 4E 47 47 41 2C 31 32 33 35 31 39 2E 30 30
                          2C 34 38 30 37 2E 30 33 38 2C 4E 2C 30 31 31 33
     GNSS ASCII: $GNGGA,123519.00,4807.038,N,01131.324,E,1,12,1.2,515.2,M...<CR><LF>
     ```

**Log Level Requirements:**
```bash
# Check current log level
AT+LOG_LEVEL?

# Set to WARNINGS (default) - shows decoded messages only
AT+LOG_LEVEL=WARNINGS

# Set to INFO - shows both decoded messages and raw hex
AT+LOG_LEVEL=INFO
```

**Typical Usage Scenarios:**

| Scenario | decoded_msgs | raw_hex | log_level | Purpose |
|----------|--------------|---------|-----------|---------|
| Normal Operation | 0 | 0 | WARNINGS | No debug output |
| Position Monitoring | 1 | 0 | WARNINGS | Track parsed positions |
| Protocol Debug | 1 | 1 | INFO | Full message inspection |
| Parser Development | 0 | 1 | INFO | Raw data analysis |

**Note:** The raw hex output only prints when the UART buffer starts with a sentence beginning (usually `$` for NMEA). Mid-sentence captures will only show hex dump without ASCII interpretation.

## Troubleshooting

### No GNSS Fix
1. Check antenna connection: `AT+GNSS_STATUS?`
2. Enable debug output to see raw data: `AT+GNSS_DEBUG=1,1` and `AT+LOG_LEVEL=INFO`
3. Verify protocol: Try `AT+GNSS_CONFIG=AUTO,AUTO,1`
4. Check signal environment (indoor/urban canyon)
5. Monitor satellites: Look for 4+ satellites in status
6. If no raw data appears, check UART connection and baud rate: `AT+BAUD_RATE?`

### Poor Accuracy
1. Enable SBAS: `AT+GNSS_SBAS=ENABLE`
2. Try PPP: `AT+GNSS_PPP=AUTO`
3. Check for multipath (reflective surfaces)
4. Increase static hold threshold: `AT+GNSS_STATIC=ENABLE,5.0`

### PPP Not Converging
1. Ensure clear sky view for 15+ minutes
2. Check constellation availability
3. Verify receiver supports selected PPP service:
   - SBAS: Any GNSS receiver
   - Galileo HAS: Requires E6-capable receiver (mosaic-X5, etc.)
   - IGS_RTS: Multi-frequency GNSS receiver
   - BeiDou B2b: BeiDou B2b-capable receiver (Asia-Pacific region)
4. Try different PPP service: `AT+GNSS_PPP=SBAS` or `AT+GNSS_PPP=IGS_RTS`
5. Check receiver model compatibility:
   ```bash
   AT+GNSS_STATUS?  # Shows receiver type and capabilities
   ```

### Frequent Failovers
1. Increase timeout: `AT+GNSS_FAILOVER=30`
2. Check primary source stability
3. Verify antenna/cable integrity
4. Monitor power supply stability

## AT Command Reference

### GNSS Configuration Commands
| Command | Description |
|---------|-------------|
| `AT+GNSS_CONFIG` | Set GNSS source, protocol, and rate |
| `AT+GNSS_PPP` | Configure PPP service |
| `AT+GNSS_RTK` | Enable/disable RTK mode |
| `AT+GNSS_FAILOVER` | Set failover timeout |
| `AT+GNSS_NETWORK` | Configure network GNSS settings |
| `AT+GNSS_STATIC` | Configure static mode |
| `AT+GNSS_SBAS` | Enable/disable SBAS |
| `AT+GNSS_DEBUG` | Enable/disable debug output (decoded and raw) |

### GNSS Status Commands
| Command | Description |
|---------|-------------|
| `AT+GNSS_STATUS` | Detailed GNSS status report |
| `AT+GNSS_POSITION` | Current position and fix info |

### Examples by Use Case

```bash
# High accuracy, low power
AT+GNSS_CONFIG=UART,SBF,1
AT+GNSS_PPP=GALILEO_HAS
AT+GNSS_STATIC=ENABLE,1.0
AT+GNSS_FAILOVER=30
```

```bash
# Balanced accuracy and responsiveness
AT+GNSS_CONFIG=AUTO,AUTO,5
AT+GNSS_PPP=AUTO
AT+GNSS_SBAS=ENABLE
AT+GNSS_FAILOVER=10
```

```bash
# High accuracy with free PPP service
AT+GNSS_CONFIG=UART,SBF,1
AT+GNSS_PPP=GALILEO_HAS
AT+GNSS_RTK=ENABLE
AT+GNSS_STATIC=ENABLE,0.5
```

```bash
# Reliable with SBAS
AT+GNSS_CONFIG=AUTO,AUTO,1
AT+GNSS_SBAS=ENABLE
AT+GNSS_FAILOVER=15
```

## Supported Hardware

### Tested GNSS Receivers
- **u-blox F9P/F9H/F9T/M10**: Full UBX support, RTK, PPP
- **Septentrio mosaic-X5**: Full SBF support, multi-band PPP
- **Generic NMEA**: Basic positioning, limited accuracy

