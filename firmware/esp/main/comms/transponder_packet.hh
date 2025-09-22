#pragma once

#include <cstdint>
#include <cstring>

// Simple transponder packet structure for MQTT publishing
struct TransponderPacket {
    // Aircraft identification
    uint32_t address;  // ICAO address (24-bit)

    // Position data
    double latitude;
    double longitude;
    int32_t altitude;  // Altitude in feet

    // Velocity data
    float heading;  // Heading in degrees
    float velocity;  // Speed in knots
    int32_t vertical_rate;  // Vertical rate in feet per minute

    // Status data
    uint16_t squawk;
    uint8_t airborne;  // 0 = on ground, 1 = airborne
    uint8_t category;  // Aircraft category code
    char callsign[9];  // 8-character callsign + null terminator

    // Timestamp
    uint64_t timestamp_ms;

    // Validity flags
    uint32_t flags;

    // Flag bit definitions
    static constexpr uint32_t FLAG_POSITION_VALID = 0x0001;
    static constexpr uint32_t FLAG_ALTITUDE_VALID = 0x0002;
    static constexpr uint32_t FLAG_VELOCITY_VALID = 0x0004;
    static constexpr uint32_t FLAG_HEADING_VALID = 0x0008;
    static constexpr uint32_t FLAG_VERTICAL_RATE_VALID = 0x0010;
    static constexpr uint32_t FLAG_CALLSIGN_VALID = 0x0020;
    static constexpr uint32_t FLAG_SQUAWK_VALID = 0x0040;
    static constexpr uint32_t FLAG_CATEGORY_VALID = 0x0080;

    // Constructor
    TransponderPacket() {
        memset(this, 0, sizeof(TransponderPacket));
        callsign[8] = '\0';
    }

    // Validity check helpers
    bool IsValid() const { return address != 0; }
    bool HasPosition() const { return flags & FLAG_POSITION_VALID; }
    bool HasAltitude() const { return flags & FLAG_ALTITUDE_VALID; }
    bool HasVelocity() const { return flags & FLAG_VELOCITY_VALID; }
    bool HasHeading() const { return flags & FLAG_HEADING_VALID; }
    bool HasVerticalRate() const { return flags & FLAG_VERTICAL_RATE_VALID; }
    bool HasCallsign() const { return flags & FLAG_CALLSIGN_VALID; }
    bool HasSquawk() const { return flags & FLAG_SQUAWK_VALID; }
    bool HasCategory() const { return flags & FLAG_CATEGORY_VALID; }
};