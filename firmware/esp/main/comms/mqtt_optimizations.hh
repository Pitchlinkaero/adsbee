/**
 * MQTT Memory Optimizations
 *
 * Helper functions to reduce string allocations and memory usage
 */

#ifndef MQTT_OPTIMIZATIONS_HH
#define MQTT_OPTIMIZATIONS_HH

#include <cstdio>
#include <cstring>
#include "mqtt_config.h"

namespace MQTT {

// Inline topic formatting to avoid string allocations
inline size_t format_topic(char* buf, size_t buf_size, const char* device_id,
                           const char* suffix1, const char* suffix2 = nullptr) {
    if (suffix2) {
        return snprintf(buf, buf_size, "%s/%s/%s", device_id, suffix1, suffix2);
    } else {
        return snprintf(buf, buf_size, "%s/%s", device_id, suffix1);
    }
}

// Compact JSON builders using static buffers
class CompactJSON {
public:
    CompactJSON(char* buffer, size_t size)
        : buf_(buffer), size_(size), pos_(0) {
        append("{");
    }

    void addString(const char* key, const char* value) {
        if (pos_ > 1) append(",");
        append("\"");
        append(key);
        append("\":\"");
        append(value);
        append("\"");
    }

    void addInt(const char* key, int value) {
        if (pos_ > 1) append(",");
        char temp[32];
        snprintf(temp, sizeof(temp), "\"%s\":%d", key, value);
        append(temp);
    }

    void addFloat(const char* key, float value) {
        if (pos_ > 1) append(",");
        char temp[32];
        snprintf(temp, sizeof(temp), "\"%s\":%.2f", key, value);
        append(temp);
    }

    void close() {
        append("}");
    }

    const char* c_str() const { return buf_; }
    size_t length() const { return pos_; }

private:
    void append(const char* str) {
        size_t len = strlen(str);
        if (pos_ + len < size_ - 1) {
            strcpy(buf_ + pos_, str);
            pos_ += len;
        }
    }

    char* buf_;
    size_t size_;
    size_t pos_;
};

// Binary protocol helpers (more compact than JSON)
namespace Binary {
    // Pack aircraft status into minimal bytes
    inline size_t pack_status(uint8_t* buf, uint32_t icao, float lat, float lon,
                              int32_t alt, uint16_t hdg, uint16_t spd) {
        size_t pos = 0;
        // Message type (1 byte)
        buf[pos++] = 0x01;  // STATUS message

        // ICAO (3 bytes)
        buf[pos++] = (icao >> 16) & 0xFF;
        buf[pos++] = (icao >> 8) & 0xFF;
        buf[pos++] = icao & 0xFF;

        // Position (8 bytes: 4 lat, 4 lon as fixed-point)
        int32_t lat_fixed = (int32_t)(lat * 1e5);
        int32_t lon_fixed = (int32_t)(lon * 1e5);
        memcpy(buf + pos, &lat_fixed, 4); pos += 4;
        memcpy(buf + pos, &lon_fixed, 4); pos += 4;

        // Altitude (2 bytes, units of 25ft)
        uint16_t alt_25ft = (alt > 0) ? (alt / 25) : 0;
        buf[pos++] = (alt_25ft >> 8) & 0xFF;
        buf[pos++] = alt_25ft & 0xFF;

        // Heading (1 byte, units of 360/256)
        buf[pos++] = (hdg * 256) / 360;

        // Speed (1 byte, knots, capped at 255)
        buf[pos++] = (spd > 255) ? 255 : spd;

        return pos;  // Total: 16 bytes vs ~100+ for JSON
    }
}

}  // namespace MQTT

#endif // MQTT_OPTIMIZATIONS_HH