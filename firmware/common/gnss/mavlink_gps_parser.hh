#ifndef MAVLINK_GPS_PARSER_HH_
#define MAVLINK_GPS_PARSER_HH_

#include "gnss_interface.hh"
#include <cstdint>

/**
 * MAVLink GPS Parser - Extracts GPS data from MAVLink telemetry stream
 * 
 * This parser connects to autopilots (ArduPilot, PX4, etc.) via serial
 * and extracts GPS information from the telemetry stream.
 * 
 * Integration with existing MAVLink infrastructure:
 * - Works alongside existing MAVLink output for ADS-B
 * - Shares the same serial port when configured for bidirectional MAVLink
 * - Can be used on GNSS UART when autopilot is connected there
 * 
 * Supported MAVLink messages:
 * - GPS_RAW_INT (ID 24): Primary GPS data
 * - GPS2_RAW (ID 124): Secondary GPS data  
 * - GLOBAL_POSITION_INT (ID 33): Fused position estimate
 * - GPS_STATUS (ID 25): Satellite information
 * - HIGH_LATENCY2 (ID 235): Compressed telemetry with GPS
 */
class MAVLinkGPSParser : public GNSSInterface {
public:
    MAVLinkGPSParser();
    virtual ~MAVLinkGPSParser() = default;
    
    // GNSSInterface implementation
    bool ParseData(const uint8_t* buffer, size_t length) override;
    bool Configure(const Config& config) override;
    Position GetLastPosition() const override { return last_position_; }
    const char* GetReceiverType() const override { return "MAVLink"; }
    void Reset() override;
    
    // PPP not supported via MAVLink
    bool SupportsPPPService(PPPService service) const override { return false; }
    bool EnablePPP(PPPService service, const char* key) override { return false; }
    PPPService GetPPPStatus(float& convergence_percent, uint32_t& eta_seconds) const override {
        convergence_percent = 0;
        eta_seconds = 0;
        return kPPPNone;
    }
    
    size_t GetDiagnostics(char* buffer, size_t max_len) const override;
    
private:
    // MAVLink protocol constants
    static constexpr uint8_t MAVLINK_STX_V1 = 0xFE;
    static constexpr uint8_t MAVLINK_STX_V2 = 0xFD;
    static constexpr size_t MAVLINK_MAX_PACKET_LEN = 280;
    
    // MAVLink message IDs we care about
    enum MessageID : uint8_t {
        HEARTBEAT = 0,
        GPS_RAW_INT = 24,
        GPS_STATUS = 25, 
        GLOBAL_POSITION_INT = 33,
        GPS2_RAW = 124,
        HIGH_LATENCY2 = 235
    };
    
    // MAVLink parser state
    enum ParserState {
        MAVLINK_PARSE_STATE_IDLE,
        MAVLINK_PARSE_STATE_GOT_STX,
        MAVLINK_PARSE_STATE_GOT_LENGTH,
        MAVLINK_PARSE_STATE_GOT_SEQ,
        MAVLINK_PARSE_STATE_GOT_SYSID,
        MAVLINK_PARSE_STATE_GOT_COMPID,
        MAVLINK_PARSE_STATE_GOT_MSGID,
        MAVLINK_PARSE_STATE_GOT_PAYLOAD,
        MAVLINK_PARSE_STATE_GOT_CRC1
    };
    
    // Packet structure for MAVLink v1
    struct MAVLinkPacket {
        uint8_t stx;
        uint8_t len;
        uint8_t seq;
        uint8_t sysid;
        uint8_t compid;
        uint8_t msgid;
        uint8_t payload[255];
        uint16_t checksum;
    };
    
    // Parse MAVLink packet
    bool ParseByte(uint8_t byte);
    bool ParsePacket();
    
    // Message handlers
    void HandleGPSRawInt(const uint8_t* payload);
    void HandleGPS2Raw(const uint8_t* payload);
    void HandleGlobalPositionInt(const uint8_t* payload);
    void HandleGPSStatus(const uint8_t* payload);
    void HandleHighLatency2(const uint8_t* payload);
    void HandleHeartbeat(const uint8_t* payload);
    
    // CRC calculation for MAVLink
    uint16_t CalculateCRC(const uint8_t* buffer, size_t length, uint8_t crc_extra);
    
    // Parser state
    ParserState parser_state_ = MAVLINK_PARSE_STATE_IDLE;
    MAVLinkPacket current_packet_;
    size_t packet_idx_ = 0;
    uint16_t packet_drops_ = 0;
    
    // GPS data
    Position last_position_;
    
    // System tracking
    uint8_t autopilot_sysid_ = 0;
    uint8_t autopilot_compid_ = 0;
    uint8_t autopilot_type_ = 0;  // MAV_AUTOPILOT enum
    bool autopilot_detected_ = false;
    
    // Statistics
    uint32_t messages_received_ = 0;
    uint32_t gps_messages_received_ = 0;
    uint32_t parse_errors_ = 0;
    uint32_t last_heartbeat_ms_ = 0;
    
    // Configuration
    bool use_fused_position_ = false;  // Use GLOBAL_POSITION_INT instead of raw GPS
    uint8_t gps_instance_ = 0;  // 0 = GPS1, 1 = GPS2
};

#endif // MAVLINK_GPS_PARSER_HH_