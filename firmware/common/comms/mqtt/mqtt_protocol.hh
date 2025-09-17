#ifndef MQTT_PROTOCOL_HH_
#define MQTT_PROTOCOL_HH_

#include <stdint.h>
#include <string.h>
#include "transponder_packet.hh"
#include "data_structures.hh"
#include "aircraft_dictionary.hh"

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
        BINARY_RAW = 0x03,        // Raw Mode-S packet (8-15 bytes)
        BINARY_TELEMETRY = 0x04,  // Device telemetry (variable)
        BINARY_GPS = 0x05         // GPS status (15 bytes)
    };
    
    // Frequency band source
    enum FrequencyBand : uint8_t {
        BAND_1090_MHZ = 0,    // Mode S / ADS-B (1090 MHz)
        BAND_978_MHZ = 1,     // UAT (978 MHz)
        BAND_UNKNOWN = 2      // Unknown source
    };
    
    // Compact binary structure for aircraft data (23 bytes total)
    struct __attribute__((packed)) BinaryAircraft {
        uint8_t type;             // Message type (1 byte)
        uint8_t band : 2;         // Frequency band (2 bits)
        uint32_t icao24 : 24;     // ICAO address (3 bytes)
        uint8_t rssi : 6;         // Signal strength (6 bits, -63 to 0 dBm)
        uint16_t timestamp;       // Seconds since epoch % 65536 (2 bytes)
        int32_t lat : 24;         // Latitude * 1e5 (3 bytes)
        int32_t lon : 24;         // Longitude * 1e5 (3 bytes)
        uint16_t alt;             // Altitude in 25ft units (2 bytes)
        uint16_t hdg : 9;         // Heading 0-359° (9 bits)
        uint16_t spd : 10;        // Speed in knots (10 bits)
        int16_t vrate : 13;       // Vertical rate / 64 fpm (13 bits)
        uint8_t flags;            // Status flags (1 byte)
        uint8_t category;         // Aircraft category (1 byte)
        uint8_t callsign[8];      // Callsign (8 bytes, null padded)
    };
    
    // Telemetry data structure
    struct TelemetryData {
        uint32_t uptime_sec;           // Uptime in seconds
        uint16_t messages_received;    // ADS-B messages per second (published/decoded)
        uint16_t messages_sent;        // MQTT messages sent (cumulative)
        int16_t cpu_temp_c;           // CPU temperature (Celsius)
        uint16_t memory_free_kb;      // Free memory in KB
        int16_t rssi_noise_floor_dbm; // Current noise floor
        uint8_t receiver_1090_enabled; // 1090 MHz receiver status
        uint8_t receiver_978_enabled;  // 978 MHz receiver status
        uint8_t wifi_connected;        // WiFi connection status
        uint8_t mqtt_connected;        // MQTT connection status
        // Firmware version info
        uint8_t fw_major = 0;          // Firmware major version
        uint8_t fw_minor = 0;          // Firmware minor version
        uint8_t fw_patch = 0;          // Firmware patch version
        // Optional message rate reporting (JSON only)
        uint16_t mps_total = 0;        // Total messages per second across all feeds
        uint8_t mps_feed_count = 0;    // Number of per-feed entries populated in mps_feeds
        static constexpr uint8_t kMaxFeedsForTelemetry = 10;
        uint16_t mps_feeds[kMaxFeedsForTelemetry] = {0}; // Per-feed messages per second
    };
    
    // GPS data structure
    struct GPSData {
        double latitude;       // Receiver latitude
        double longitude;      // Receiver longitude
        float altitude_m;      // Altitude in meters
        uint8_t fix_status;    // 0=no fix, 1=2D, 2=3D
        uint8_t num_satellites;// Number of satellites
        float hdop;           // Horizontal dilution of precision
        uint32_t timestamp;   // GPS timestamp
    };
    
    // Compact binary telemetry (16 bytes)
    struct __attribute__((packed)) BinaryTelemetry {
        uint8_t type;                   // Message type (BINARY_TELEMETRY)
        uint32_t uptime : 24;          // Uptime in minutes (3 bytes)
        uint16_t msgs_rx;              // Messages received (2 bytes)
        uint16_t msgs_tx;              // Messages transmitted (2 bytes)
        int8_t cpu_temp;               // CPU temp in Celsius (1 byte)
        uint16_t mem_free;             // Free memory in MB (2 bytes)
        int8_t noise_floor;            // Noise floor in dBm (1 byte)
        uint8_t status;                // Status bits (1 byte)
        uint8_t reserved[2];           // Reserved for future use
    };
    
    // Compact binary GPS (15 bytes)
    struct __attribute__((packed)) BinaryGPS {
        uint8_t type;                   // Message type (BINARY_GPS)
        int32_t lat : 24;              // Latitude * 1e5 (3 bytes)
        int32_t lon : 24;              // Longitude * 1e5 (3 bytes)
        uint16_t alt;                  // Altitude in meters (2 bytes)
        uint8_t fix : 2;               // Fix status (2 bits)
        uint8_t sats : 6;              // Number of satellites (6 bits)
        uint16_t hdop;                 // HDOP * 100 (2 bytes)
        uint16_t timestamp;            // Minutes since epoch % 65536 (2 bytes)
    };
    
    static const uint16_t kMaxMessageSize = 512;  // Max size for any message
    static const uint16_t kMaxTopicSize = 64;     // Max MQTT topic length
    
    /**
     * Format a decoded ADS-B packet for MQTT publishing
     * @param[in] packet The decoded packet
     * @param[out] buffer Output buffer
     * @param[in] buffer_size Buffer size
     * @param[in] format Output format (JSON or BINARY)
     * @param[in] band Frequency band source
     * @return Number of bytes written, 0 on error
     */
    static uint16_t FormatPacket(const Decoded1090Packet& packet,
                                  uint8_t* buffer,
                                  uint16_t buffer_size,
                                  Format format,
                                  FrequencyBand band = BAND_1090_MHZ);
    
    /**
     * Format aircraft data for MQTT publishing
     * @param[in] aircraft The aircraft data
     * @param[out] buffer Output buffer
     * @param[in] buffer_size Buffer size
     * @param[in] format Output format (JSON or BINARY)
     * @param[in] band Frequency band source
     * @return Number of bytes written, 0 on error
     */
    static uint16_t FormatAircraft(const Aircraft1090& aircraft,
                                    uint8_t* buffer,
                                    uint16_t buffer_size,
                                    Format format,
                                    FrequencyBand band = BAND_1090_MHZ);
    
    /**
     * Get MQTT topic for publishing
     * @param[in] icao24 Aircraft ICAO address
     * @param[in] msg_type Message type ("position", "status", "raw")
     * @param[out] topic_buf Buffer for topic string
     * @param[in] topic_size Buffer size
     * @param[in] band Frequency band source
     * @param[in] use_short Use short topics for binary format
     * @param[in] device_id Optional device ID to prepend to topic
     * @return true on success
     */
    static bool GetTopic(uint32_t icao24,
                        const char* msg_type,
                        char* topic_buf,
                        uint16_t topic_size,
                        FrequencyBand band = BAND_1090_MHZ,
                        bool use_short = false,
                        const char* device_id = nullptr);
    
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
    
    /**
     * Format telemetry data for MQTT publishing
     * @param[in] telemetry Device telemetry data
     * @param[out] buffer Output buffer
     * @param[in] buffer_size Buffer size
     * @param[in] format Output format (JSON or BINARY)
     * @return Number of bytes written, 0 on error
     */
    static uint16_t FormatTelemetry(const TelemetryData& telemetry,
                                     uint8_t* buffer,
                                     uint16_t buffer_size,
                                     Format format);
    
    /**
     * Format GPS data for MQTT publishing
     * @param[in] gps GPS position data
     * @param[out] buffer Output buffer
     * @param[in] buffer_size Buffer size
     * @param[in] format Output format (JSON or BINARY)
     * @return Number of bytes written, 0 on error
     */
    static uint16_t FormatGPS(const GPSData& gps,
                              uint8_t* buffer,
                              uint16_t buffer_size,
                              Format format);
    
    /**
     * Get telemetry topic
     * @param[out] topic_buf Buffer for topic string
     * @param[in] topic_size Buffer size
     * @param[in] msg_type "telemetry" or "gps"
     * @param[in] use_short Use short topics for binary format
     * @param[in] device_id Optional device ID to prepend to topic
     * @return true on success
     */
    static bool GetTelemetryTopic(char* topic_buf,
                                  uint16_t topic_size,
                                  const char* msg_type,
                                  bool use_short = false,
                                  const char* device_id = nullptr);

private:
    // JSON formatting
    static uint16_t FormatPacketJSON(const Decoded1090Packet& packet,
                                      char* buffer,
                                      uint16_t buffer_size,
                                      FrequencyBand band);
    
    static uint16_t FormatAircraftJSON(const Aircraft1090& aircraft,
                                        char* buffer,
                                        uint16_t buffer_size,
                                        FrequencyBand band);
    
    // Binary formatting
    static uint16_t FormatPacketBinary(const Decoded1090Packet& packet,
                                        uint8_t* buffer,
                                        uint16_t buffer_size,
                                        FrequencyBand band);
    
    static uint16_t FormatAircraftBinary(const Aircraft1090& aircraft,
                                          uint8_t* buffer,
                                          uint16_t buffer_size,
                                          FrequencyBand band);
    
    /**
     * Get category string from enum
     */
    static const char* GetCategoryString(Aircraft1090::Category category);
    
    /**
     * Get standard ADS-B category code (e.g., A3, C1, D7)
     * @param[in] category_raw Raw 8-bit category value from ADS-B
     * @return Category code string
     */
    static const char* GetCategoryCode(uint8_t category_raw);
    
    // Telemetry formatting
    static uint16_t FormatTelemetryJSON(const TelemetryData& telemetry,
                                         char* buffer,
                                         uint16_t buffer_size);
    
    static uint16_t FormatTelemetryBinary(const TelemetryData& telemetry,
                                           uint8_t* buffer,
                                           uint16_t buffer_size);
    
    // GPS formatting
    static uint16_t FormatGPSJSON(const GPSData& gps,
                                   char* buffer,
                                   uint16_t buffer_size);
    
    static uint16_t FormatGPSBinary(const GPSData& gps,
                                     uint8_t* buffer,
                                     uint16_t buffer_size);
};

#endif // MQTT_PROTOCOL_HH_