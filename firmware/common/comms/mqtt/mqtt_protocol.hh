#ifndef MQTT_PROTOCOL_HH_
#define MQTT_PROTOCOL_HH_

#include <stdint.h>
#include <string.h>
#include "transponder_packet.hh"
#include "data_structures.hh"

/**
 * MQTT protocol formatter for ADS-B data.
 * Supports both JSON (human-readable) and binary (bandwidth-optimized) formats.
 * Prioritizes real-time delivery - no batching delays.
 */

class MQTTProtocol {
public:
    // Output format selection
    enum Format : uint8_t {
        FORMAT_JSON = 0,    // Human-readable JSON (~250 bytes/msg)
        FORMAT_BINARY = 1   // Compact binary (~20 bytes/msg)
    };
    
    // Binary message types
    enum BinaryType : uint8_t {
        BINARY_POSITION = 0x01,   // Position update (13 bytes)
        BINARY_AIRCRAFT = 0x02,   // Full aircraft state (20 bytes)
        BINARY_RAW = 0x03        // Raw Mode-S packet (8-15 bytes)
    };
    
    // Compact binary structure for aircraft data (20 bytes total)
    struct __attribute__((packed)) BinaryAircraft {
        uint8_t type;             // Message type (1 byte)
        uint32_t icao24 : 24;     // ICAO address (3 bytes)
        uint16_t timestamp;       // Seconds since epoch % 65536 (2 bytes)
        int32_t lat : 24;         // Latitude * 1e5 (3 bytes)
        int32_t lon : 24;         // Longitude * 1e5 (3 bytes)
        uint16_t alt;             // Altitude in 25ft units (2 bytes)
        uint16_t hdg : 9;         // Heading 0-359° (9 bits)
        uint16_t spd : 10;        // Speed in knots (10 bits)
        int16_t vrate : 13;       // Vertical rate / 64 fpm (13 bits)
        uint8_t flags;            // Status flags (1 byte)
    };
    
    static const uint16_t kMaxMessageSize = 512;  // Max size for any message
    static const uint16_t kMaxTopicSize = 64;     // Max MQTT topic length
    
    /**
     * Format a decoded ADS-B packet for MQTT publishing
     * @param[in] packet The decoded packet
     * @param[out] buffer Output buffer
     * @param[in] buffer_size Buffer size
     * @param[in] format Output format (JSON or BINARY)
     * @return Number of bytes written, 0 on error
     */
    static uint16_t FormatPacket(const Decoded1090Packet& packet,
                                  uint8_t* buffer,
                                  uint16_t buffer_size,
                                  Format format);
    
    /**
     * Format aircraft data for MQTT publishing
     * @param[in] aircraft The aircraft data
     * @param[out] buffer Output buffer
     * @param[in] buffer_size Buffer size
     * @param[in] format Output format (JSON or BINARY)
     * @return Number of bytes written, 0 on error
     */
    static uint16_t FormatAircraft(const Aircraft& aircraft,
                                    uint8_t* buffer,
                                    uint16_t buffer_size,
                                    Format format);
    
    /**
     * Get MQTT topic for publishing
     * @param[in] icao24 Aircraft ICAO address
     * @param[in] msg_type Message type ("position", "status", "raw")
     * @param[out] topic_buf Buffer for topic string
     * @param[in] topic_size Buffer size
     * @param[in] use_short Use short topics for binary format
     * @return true on success
     */
    static bool GetTopic(uint32_t icao24,
                        const char* msg_type,
                        char* topic_buf,
                        uint16_t topic_size,
                        bool use_short = false);
    
    /**
     * Estimate bandwidth usage
     * @param[in] format Message format
     * @param[in] messages_per_hour Expected message rate
     * @return Estimated bytes per hour
     */
    static uint32_t EstimateBandwidth(Format format, uint16_t messages_per_hour) {
        uint16_t msg_size = (format == FORMAT_JSON) ? 250 : 20;
        return msg_size * messages_per_hour;
    }
    
    /**
     * Get format from string
     */
    static Format ParseFormat(const char* str) {
        if (str && strcasecmp(str, "BINARY") == 0) {
            return FORMAT_BINARY;
        }
        return FORMAT_JSON;  // Default
    }
    
    /**
     * Get format string
     */
    static const char* FormatToString(Format format) {
        return (format == FORMAT_BINARY) ? "BINARY" : "JSON";
    }

private:
    // JSON formatting
    static uint16_t FormatPacketJSON(const Decoded1090Packet& packet,
                                      char* buffer,
                                      uint16_t buffer_size);
    
    static uint16_t FormatAircraftJSON(const Aircraft& aircraft,
                                        char* buffer,
                                        uint16_t buffer_size);
    
    // Binary formatting
    static uint16_t FormatPacketBinary(const Decoded1090Packet& packet,
                                        uint8_t* buffer,
                                        uint16_t buffer_size);
    
    static uint16_t FormatAircraftBinary(const Aircraft& aircraft,
                                          uint8_t* buffer,
                                          uint16_t buffer_size);
};

#endif // MQTT_PROTOCOL_HH_