# ADSBee GNSS Developer's Guide

## Overview

This guide explains how the ADSBee GNSS system works internally and how to integrate it into your application code. The GNSS system provides a unified interface for position, navigation, and timing (PNT) data regardless of source (UART, Network, or MAVLink).

## Architecture

### System Components

```
┌─────────────────────────────────────────────────────────────┐
│                     ADSBee GNSS System                      │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌──────────────┐           ┌──────────────────────────┐    │
│  │   ESP32      │           │        RP2040            │    │
│  │              │   SPI     │                          │    │
│  │  - TCP/UDP   │ Object    │  ┌──────────────────┐    │    │
│  │  - NTRIP     │Dictionary │  │  GNSS Manager    │    │    │
│  │  - MAVLink   │═══════════>  │                  │    │    │
│  │  - Web UI    │           │  │  - UART Source   │    │    │
│  │              │<═══════════  │  - Network Source│    │    │
│  │  Receives    │           │  │  - Parser Select │    │    │
│  │  NetworkGNSS │  Syncs    │  │  - Failover      │    │    │
│  │  Messages    │  Status   │  │  - Position Data │    │    │
│  └──────────────┘           │  └──────────────────┘    │    │
│                             │           │              │    │
│                             │           ▼              │    │
│                             │  ┌──────────────────┐    │    │
│                             │  │ Application Code │    │    │
│                             │  │                  │    │    │
│                             │  │  GetPosition()   │    │    │
│                             │  │  IsValid()       │    │    │
│                             │  └──────────────────┘    │    │
│                             └──────────────────────────┘    │
└─────────────────────────────────────────────────────────────┘

       UART GNSS ─────────────────┐
       (Local GNSS Module)        │
                                  ▼
                          [RP2040 UART0]
```

### Key Design Principles

1. **Source Transparency**: Application code doesn't need to know if position data comes from UART, Network, or MAVLink
2. **Automatic Failover**: System automatically switches between sources based on availability and priority
3. **Unified Parsing**: All GNSS data (UART/Network) flows through the same parser and position structure
4. **Bidirectional Sync**: Position data is synchronized between RP2040 and ESP32 for web interface and network operations

## Core Components

### 1. GNSSManager (RP2040)

**Location**: `firmware/pico/application/gnss/gnss_manager.hh`

The central coordinator running on RP2040 that:
- Manages all GNSS data sources (UART, Network, MAVLink)
- Handles parser selection (NMEA, UBX, SBF, etc.)
- Coordinates failover between sources
- Maintains current position state
- Provides unified API for application code

**Global Instance**:
```cpp
extern GNSSManager gnss_manager;  // Defined in main.cc
```

### 2. GNSSInterface

**Location**: `firmware/common/gnss/gnss_interface.hh`

Base interface for all GNSS parsers, defining:
- Position structure with comprehensive PNT data
- Parser interface (ParseData, GetLastPosition, etc.)
- PPP service enumeration and configuration
- Fix type definitions

### 3. Object Dictionary (ESP32 ↔ RP2040)

**Location**: `firmware/common/coprocessor/object_dictionary.hh`

Shared memory interface for inter-processor communication:
- **`kAddrGPSNetworkMessage`** (0x11): Network GPS messages from ESP32 to RP2040
- **`kAddrGPSStatus`** (0x13): GPS status from RP2040 to ESP32 (for web interface)

## Data Flow

### UART GNSS Data Flow

```
GNSS Module (UART) → RP2040 UART0 → GNSSManager::ProcessUARTData()
                                             ↓
                                    GNSSManager::ProcessData()
                                             ↓
                                    Parser (NMEA/UBX/SBF)
                                             ↓
                                    current_position_ updated
                                             ↓
                                    Sync to ESP32 (Object Dictionary)
                                             ↓
                                    Available to application via GetCurrentPosition()
```

### Network GNSS Data Flow

```
Network (TCP/NTRIP) → ESP32 → Object Dictionary (kAddrGPSNetworkMessage)
                                             ↓
                      RP2040 polls in main.cc loop
                                             ↓
                      GNSSManager::ProcessNetworkGPSMessage()
                                             ↓
                      GNSSManager::ProcessData() [SAME PATH AS UART]
                                             ↓
                      Parser processes data
                                             ↓
                      current_position_ updated
                                             ↓
                      Sync back to ESP32 (Object Dictionary)
                                             ↓
                      Available to application via GetCurrentPosition()
```

### Position Synchronization

The system maintains position consistency across processors:

**RP2040 → ESP32 Sync** (for web interface):
```cpp
// In main.cc or periodic task
ObjectDictionary::GPSStatus gps_status;
gps_status.valid = gnss_manager.IsPositionValid();
gps_status.latitude_deg = pos.latitude_deg;
gps_status.longitude_deg = pos.longitude_deg;
// ... copy other fields ...
esp32.Write(ObjectDictionary::kAddrGPSStatus, gps_status);
```

**ESP32 → RP2040 Sync** (network GNSS):
```cpp
// On ESP32 when network GPS received
GPSNetworkMessage msg;
msg.type = 0;  // NMEA
msg.length = nmea_len;
memcpy(msg.data, nmea_data, nmea_len);
object_dictionary.gps_network_message_buffer_ = msg;

// On RP2040 (main.cc loop)
GPSNetworkMessage gps_msg;
if (esp32.Read(ObjectDictionary::Address::kAddrGPSNetworkMessage, gps_msg)) {
    gnss_manager.ProcessNetworkGPSMessage(gps_msg.type, gps_msg.data, gps_msg.length);
}
```

## Using GNSS in Your Application

### Basic Position Access

```cpp
#include "gnss_manager.hh"

extern GNSSManager gnss_manager;

void MyApplicationFunction() {
    // Get current position
    GNSSInterface::Position pos = gnss_manager.GetCurrentPosition();

    // Check validity
    if (!gnss_manager.IsPositionValid()) {
        // No valid GPS fix
        return;
    }

    // Use position data
    double lat = pos.latitude_deg;
    double lon = pos.longitude_deg;
    float alt_msl = pos.altitude_msl_m;  // MSL altitude (standard for aviation)
    float alt_hae = pos.altitude_m;       // Height Above Ellipsoid

    // Check fix quality
    if (pos.HasHighPrecision()) {
        // PPP converged - centimeter to decimeter accuracy
        printf("High precision: %.2f m accuracy\n", pos.GetAccuracyM());
    } else if (pos.Has3DFix()) {
        // Standard 3D fix - meter accuracy
        printf("Standard 3D fix: %d satellites\n", pos.satellites_used);
    }
}
```

### Position Structure Reference

```cpp
struct Position {
    // === Core Position Data ===
    double latitude_deg;              // WGS84 latitude
    double longitude_deg;             // WGS84 longitude
    float altitude_m;                 // Height Above Ellipsoid (HAE)
    float altitude_msl_m;             // Mean Sea Level altitude

    // === Velocity (for mobile platforms) ===
    float ground_speed_mps;           // Ground speed (m/s)
    float track_deg;                  // True track (degrees)
    float vertical_velocity_mps;      // Climb/descent rate (m/s)

    // === Fix Quality ===
    FixType fix_type;                 // 0=NoFix, 2=2D, 3=3D, 6=PPP, 8=RTK
    uint8_t satellites_used;          // Number of satellites
    float hdop, vdop, pdop;           // Dilution of Precision

    // === Accuracy Estimates ===
    float accuracy_horizontal_m;      // Horizontal accuracy (meters)
    float accuracy_vertical_m;        // Vertical accuracy (meters)
    float accuracy_speed_mps;         // Speed accuracy (m/s)
    float accuracy_heading_deg;       // Heading accuracy (degrees)

    // === PPP Information ===
    PPPService ppp_service;           // Active PPP service (if any)
    uint8_t ppp_status;               // 0=none, 1=converging, 2=converged
    float ppp_convergence_percent;    // 0-100%
    uint32_t ppp_convergence_time_s;  // Time to convergence

    // === Timing ===
    uint32_t timestamp_ms;            // System timestamp
    uint32_t gps_time_week;           // GPS week number
    uint32_t gps_time_ms;             // GPS time of week (ms)

    // UTC Date/Time
    uint16_t utc_year;                // e.g., 2025
    uint8_t utc_month, utc_day;       // 1-12, 1-31
    uint8_t utc_hour, utc_minute, utc_second;  // 0-23, 0-59, 0-59

    // === Validity ===
    bool valid;                       // Overall position validity

    // === Helper Methods ===
    bool HasFix() const;              // Any fix (≥2D)
    bool Has3DFix() const;            // 3D fix or better
    bool HasHighPrecision() const;    // PPP converged
    float GetAccuracyM() const;       // Best accuracy estimate
};
```

### Fix Type Enumeration

```cpp
enum FixType : uint8_t {
    kNoFix = 0,           // No position fix
    kDead = 1,            // Dead reckoning only
    k2DFix = 2,           // 2D fix (lat/lon only)
    k3DFix = 3,           // 3D fix (lat/lon/alt)
    kGNSSDGPS = 4,        // 3D + SBAS/DGPS (~1-2m)
    kPPPConverging = 5,   // PPP solution converging
    kPPPConverged = 6,    // PPP solution converged (~10-20cm)
    kRTKFloat = 7,        // RTK float solution (~20-50cm)
    kRTKFixed = 8,        // RTK fixed solution (~1-2cm)
    kRTKFixedDGPS = 9     // RTK + DGPS (highest accuracy)
};
```

## Code Examples

### Example 1

```cpp
#include "gnss_manager.hh"

extern GNSSManager gnss_manager;

void UpdateADSBOwnshipPosition(ADSBMessage& msg) {
    GNSSInterface::Position pos = gnss_manager.GetCurrentPosition();

    if (!pos.valid || !pos.Has3DFix()) {
        msg.position_valid = false;
        return;
    }

    // Convert to ADS-B format
    msg.position_valid = true;
    msg.latitude = pos.latitude_deg;
    msg.longitude = pos.longitude_deg;
    msg.altitude_ft = static_cast<int32_t>(pos.altitude_msl_m * 3.28084);  // MSL in feet

    // Set quality indicator based on fix type
    if (pos.fix_type >= GNSSInterface::kPPPConverged) {
        msg.nic = 11;  // High integrity
        msg.sil = 3;   // High confidence
    } else if (pos.fix_type >= GNSSInterface::k3DFix) {
        msg.nic = 8;   // Standard GPS
        msg.sil = 2;   // Medium confidence
    }

    // Velocity
    msg.ground_speed_kts = pos.ground_speed_mps * 1.94384;  // m/s to knots
    msg.track_deg = pos.track_deg;
    msg.vertical_rate_fpm = pos.vertical_velocity_mps * 196.85;  // m/s to ft/min
}
```

### Example 2
```cpp
#include <cmath>
#include "gnss_manager.hh"

float CalculateDistanceMeters(const GNSSInterface::Position& pos1,
                               const GNSSInterface::Position& pos2) {
    if (!pos1.valid || !pos2.valid) {
        return -1.0f;  // Invalid
    }

    // Haversine formula for great circle distance
    const float R = 6371000.0f;  // Earth radius in meters

    float lat1 = pos1.latitude_deg * M_PI / 180.0f;
    float lat2 = pos2.latitude_deg * M_PI / 180.0f;
    float dlat = (pos2.latitude_deg - pos1.latitude_deg) * M_PI / 180.0f;
    float dlon = (pos2.longitude_deg - pos1.longitude_deg) * M_PI / 180.0f;

    float a = sin(dlat/2) * sin(dlat/2) +
              cos(lat1) * cos(lat2) * sin(dlon/2) * sin(dlon/2);
    float c = 2 * atan2(sqrt(a), sqrt(1-a));

    return R * c;  // Distance in meters
}

// Usage
void CheckProximityToWaypoint() {
    GNSSInterface::Position current = gnss_manager.GetCurrentPosition();
    GNSSInterface::Position waypoint;
    waypoint.latitude_deg = 48.117300;
    waypoint.longitude_deg = 11.516700;
    waypoint.valid = true;

    float distance = CalculateDistanceMeters(current, waypoint);

    if (distance >= 0 && distance < 100.0f) {
        printf("Within 100m of waypoint!\n");
    }
}
```

### Example 3

```cpp
#include "gnss_manager.hh"

void CheckAltitudeThreshold() {
    GNSSInterface::Position pos = gnss_manager.GetCurrentPosition();

    if (!pos.valid || !pos.Has3DFix()) {
        return;
    }

    // Use MSL altitude for aviation applications
    float altitude_ft = pos.altitude_msl_m * 3.28084;

    if (altitude_ft > 18000.0f) {
        // Above FL180 - use Flight Level
        int flight_level = static_cast<int>(altitude_ft / 100);
        printf("Flight Level: FL%03d\n", flight_level);
    } else {
        // Below 18000 - use MSL altitude
        printf("Altitude: %d ft MSL\n", static_cast<int>(altitude_ft));
    }

    // Check accuracy for precision operations
    if (pos.accuracy_vertical_m < 10.0f) {
        // High vertical accuracy - safe for precision approaches
        printf("Vertical accuracy: %.1f m (precision capable)\n",
               pos.accuracy_vertical_m);
    }
}
```

### Example 4

```cpp
#include <time.h>
#include "gnss_manager.hh"

bool SyncSystemTimeFromGPS() {
    GNSSInterface::Position pos = gnss_manager.GetCurrentPosition();

    if (!pos.valid || pos.utc_year < 2020) {
        return false;  // Invalid time
    }

    // GPS provides UTC time
    struct tm gps_time = {};
    gps_time.tm_year = pos.utc_year - 1900;  // tm_year is years since 1900
    gps_time.tm_mon = pos.utc_month - 1;     // tm_mon is 0-11
    gps_time.tm_mday = pos.utc_day;
    gps_time.tm_hour = pos.utc_hour;
    gps_time.tm_min = pos.utc_minute;
    gps_time.tm_sec = pos.utc_second;

    // Convert to timestamp
    time_t utc_time = mktime(&gps_time);

    printf("GPS Time: %04d-%02d-%02d %02d:%02d:%02d UTC\n",
           pos.utc_year, pos.utc_month, pos.utc_day,
           pos.utc_hour, pos.utc_minute, pos.utc_second);

    // Sync system time (platform-specific)
    // ... implementation depends on platform ...

    return true;
}
```

## Debugging and Diagnostics

### Enable Debug Output

See the [GNSS Configuration Guide](GNSS_Configuration_Guide.md#debug-output-configuration) for details on debug output.

```bash
# Enable decoded message output (WARNING level - always visible)
AT+GNSS_DEBUG=1,0

# Enable raw hex output (requires INFO level)
AT+LOG_LEVEL=INFO
AT+GNSS_DEBUG=1,1
```

### Get Diagnostics Programmatically

```cpp
void PrintGNSSDiagnostics() {
    // Get statistics
    GNSSManager::Statistics stats = gnss_manager.GetStatistics();

    printf("GNSS Statistics:\n");
    printf("  Messages: %u\n", stats.messages_processed);
    printf("  Position Updates: %u\n", stats.position_updates);
    printf("  Parse Errors: %u\n", stats.parse_errors);
    printf("  Source Switches: %u\n", stats.source_switches);
    printf("  Failover Count: %u\n", stats.failover_count);
    printf("  Best Accuracy: %.2f m\n", stats.best_accuracy_m);
    printf("  Uptime: %u s\n", stats.uptime_s);

    // Get receiver info
    printf("  Receiver: %s\n", gnss_manager.GetReceiverType());
    printf("  High Precision: %s\n",
           gnss_manager.SupportsHighPrecision() ? "YES" : "NO");
}
```

### Check Position Health

```cpp
bool IsPositionHealthy() {
    GNSSInterface::Position pos = gnss_manager.GetCurrentPosition();

    // Basic validity
    if (!pos.valid || !pos.HasFix()) {
        printf("No GPS fix\n");
        return false;
    }

    // Check satellite count
    if (pos.satellites_used < 6) {
        printf("WARNING: Low satellite count (%d)\n", pos.satellites_used);
    }

    // Check HDOP
    if (pos.hdop > 5.0f) {
        printf("WARNING: Poor geometry (HDOP=%.2f)\n", pos.hdop);
    }

    // Check accuracy
    if (pos.accuracy_horizontal_m > 50.0f) {
        printf("WARNING: Low accuracy (%.1f m)\n", pos.accuracy_horizontal_m);
    }

    // Position age check
    uint32_t age_ms = GetTimeMs() - pos.timestamp_ms;
    if (age_ms > 5000) {
        printf("WARNING: Stale position (age=%u ms)\n", age_ms);
        return false;
    }

    return true;
}
```

## Thread Safety

The GNSS manager is thread-safe. All public methods are protected by an internal mutex and can be safely called from multiple threads, interrupt handlers, or callbacks without external synchronization.

```cpp
// ✅ Safe: Call from main loop
void MainLoopTask() {
    GNSSInterface::Position pos = gnss_manager.GetCurrentPosition();
    // Process position
}

// ✅ Safe: Call from interrupt or callback
void OnTimerInterrupt() {
    if (gnss_manager.IsPositionValid()) {
        UpdateDisplay();
    }
}

// ✅ Safe: Call from multiple threads
void Thread1() {
    gnss_manager.UpdatePosition();
}

void Thread2() {
    GNSSInterface::Position pos = gnss_manager.GetCurrentPosition();
    ProcessPosition(pos);
}
```

**Implementation Details:**
- Uses `std::mutex` for synchronization
- All public methods use `std::lock_guard` for RAII-style locking
- Const methods (getters) also lock to prevent reading during updates
- No risk of deadlocks as locks are never held while calling external code

## Performance Considerations

1. **GetCurrentPosition()** is a lightweight copy operation - safe to call frequently
2. **UpdatePosition()** should be called at configured rate (typically 1Hz)
3. **ProcessData()** has minimal overhead - real-time suitable
4. Avoid blocking operations in position callbacks


For configuration and setup, see the [GNSS Configuration Guide](GNSS_Configuration_Guide.md).

For AT command reference, use `AT+HELP` or see the configuration guide.
