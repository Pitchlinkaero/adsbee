#include "mavlink_gps_parser.hh"
#include <cstring>
#include <cstdio>
#include <cmath>

#ifdef ON_EMBEDDED_DEVICE
#include "pico/stdlib.h"
#define GET_TIME_MS() to_ms_since_boot(get_absolute_time())
#else
#include <chrono>
#define GET_TIME_MS() std::chrono::duration_cast<std::chrono::milliseconds>(\
    std::chrono::steady_clock::now().time_since_epoch()).count()
#endif

// MAVLink CRC extra bytes for each message ID (required for CRC calculation)
static const uint8_t MAVLINK_CRC_EXTRA[256] = {
    50, 124, 137, 0, 237, 217, 104, 119, 0, 0, 0, 89, 0, 0, 0, 0,
    0, 0, 0, 0, 214, 159, 220, 168, 24, 23, 170, 144, 67, 115, 39, 246,
    185, 104, 237, 244, 222, 212, 9, 254, 230, 28, 28, 132, 221, 232, 11, 153,
    41, 39, 78, 196, 0, 0, 15, 3, 0, 0, 0, 0, 0, 167, 183, 119,
    191, 118, 148, 21, 0, 243, 124, 0, 0, 38, 20, 158, 152, 143, 0, 0,
    0, 106, 49, 22, 143, 140, 5, 150, 0, 231, 183, 63, 54, 47, 0, 0,
    0, 0, 0, 0, 175, 102, 158, 208, 56, 93, 138, 108, 32, 185, 84, 34,
    174, 124, 237, 4, 76, 128, 56, 116, 134, 237, 203, 250, 87, 203, 220, 25,
    226, 46, 29, 223, 85, 6, 229, 203, 1, 195, 109, 168, 181, 47, 72, 131,
    127, 0, 103, 154, 178, 200, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    183, 0, 130, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 186, 224, 0, 0
};

MAVLinkGPSParser::MAVLinkGPSParser() {
    // Initialize to defaults
    memset(&last_position_, 0, sizeof(last_position_));
    memset(&satellite_info_, 0, sizeof(satellite_info_));
    memset(&current_packet_, 0, sizeof(current_packet_));
}

bool MAVLinkGPSParser::Configure(const Config& config) {
    // Store configuration
    use_fused_position_ = config.use_fused_position;
    return true;
}

bool MAVLinkGPSParser::ParseData(const uint8_t* buffer, size_t length) {
    if (!buffer || length == 0) return false;
    
    bool packet_found = false;
    for (size_t i = 0; i < length; i++) {
        if (ParseByte(buffer[i])) {
            packet_found = true;
        }
    }
    
    return packet_found;
}

bool MAVLinkGPSParser::ParseByte(uint8_t byte) {
    switch (parser_state_) {
        case MAVLINK_PARSE_STATE_IDLE:
            if (byte == MAVLINK_STX_V1) {
                current_packet_.stx = byte;
                parser_state_ = MAVLINK_PARSE_STATE_GOT_STX;
                packet_idx_ = 0;
            }
            break;
            
        case MAVLINK_PARSE_STATE_GOT_STX:
            current_packet_.len = byte;
            if (byte > 255) {
                // Invalid length
                parser_state_ = MAVLINK_PARSE_STATE_IDLE;
                parse_errors_++;
            } else {
                parser_state_ = MAVLINK_PARSE_STATE_GOT_LENGTH;
            }
            break;
            
        case MAVLINK_PARSE_STATE_GOT_LENGTH:
            current_packet_.seq = byte;
            parser_state_ = MAVLINK_PARSE_STATE_GOT_SEQ;
            break;
            
        case MAVLINK_PARSE_STATE_GOT_SEQ:
            current_packet_.sysid = byte;
            parser_state_ = MAVLINK_PARSE_STATE_GOT_SYSID;
            break;
            
        case MAVLINK_PARSE_STATE_GOT_SYSID:
            current_packet_.compid = byte;
            parser_state_ = MAVLINK_PARSE_STATE_GOT_COMPID;
            break;
            
        case MAVLINK_PARSE_STATE_GOT_COMPID:
            current_packet_.msgid = byte;
            packet_idx_ = 0;
            if (current_packet_.len == 0) {
                parser_state_ = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
            } else {
                parser_state_ = MAVLINK_PARSE_STATE_GOT_MSGID;
            }
            break;
            
        case MAVLINK_PARSE_STATE_GOT_MSGID:
            current_packet_.payload[packet_idx_++] = byte;
            if (packet_idx_ >= current_packet_.len) {
                parser_state_ = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
            }
            break;
            
        case MAVLINK_PARSE_STATE_GOT_PAYLOAD:
            current_packet_.checksum = byte;
            parser_state_ = MAVLINK_PARSE_STATE_GOT_CRC1;
            break;
            
        case MAVLINK_PARSE_STATE_GOT_CRC1:
            current_packet_.checksum |= (uint16_t)byte << 8;
            
            // Verify CRC
            uint8_t crc_buffer[263];  // Max MAVLink v1 packet
            crc_buffer[0] = current_packet_.len;
            crc_buffer[1] = current_packet_.seq;
            crc_buffer[2] = current_packet_.sysid;
            crc_buffer[3] = current_packet_.compid;
            crc_buffer[4] = current_packet_.msgid;
            memcpy(&crc_buffer[5], current_packet_.payload, current_packet_.len);
            
            uint16_t calculated_crc = CalculateCRC(crc_buffer, 5 + current_packet_.len, 
                                                   MAVLINK_CRC_EXTRA[current_packet_.msgid]);
            
            parser_state_ = MAVLINK_PARSE_STATE_IDLE;
            
            if (calculated_crc == current_packet_.checksum) {
                messages_received_++;
                return ParsePacket();
            } else {
                parse_errors_++;
                return false;
            }
            break;
    }
    
    return false;
}

bool MAVLinkGPSParser::ParsePacket() {
    // Handle different message types
    switch (current_packet_.msgid) {
        case HEARTBEAT:
            HandleHeartbeat(current_packet_.payload);
            break;
            
        case GPS_RAW_INT:
            HandleGPSRawInt(current_packet_.payload);
            gps_messages_received_++;
            return true;
            
        case GPS_STATUS:
            HandleGPSStatus(current_packet_.payload);
            return true;
            
        case GLOBAL_POSITION_INT:
            if (use_fused_position_) {
                HandleGlobalPositionInt(current_packet_.payload);
                gps_messages_received_++;
                return true;
            }
            break;
            
        case GPS2_RAW:
            if (gps_instance_ == 1) {
                HandleGPS2Raw(current_packet_.payload);
                gps_messages_received_++;
                return true;
            }
            break;
            
        case HIGH_LATENCY2:
            HandleHighLatency2(current_packet_.payload);
            gps_messages_received_++;
            return true;
    }
    
    return false;
}

void MAVLinkGPSParser::HandleGPSRawInt(const uint8_t* payload) {
    // GPS_RAW_INT message structure
    struct {
        uint64_t time_usec;
        int32_t lat;  // Latitude in 1E7 degrees
        int32_t lon;  // Longitude in 1E7 degrees
        int32_t alt;  // Altitude in mm
        uint16_t eph;  // GPS HDOP * 100
        uint16_t epv;  // GPS VDOP * 100
        uint16_t vel;  // Ground speed in cm/s
        uint16_t cog;  // Course over ground in cdeg
        uint8_t fix_type;
        uint8_t satellites_visible;
    } __attribute__((packed)) gps_raw;
    
    memcpy(&gps_raw, payload, sizeof(gps_raw));
    
    // Update position
    last_position_.timestamp_ms = GET_TIME_MS();
    last_position_.latitude_deg = gps_raw.lat / 1e7;
    last_position_.longitude_deg = gps_raw.lon / 1e7;
    last_position_.altitude_m = gps_raw.alt / 1000.0f;
    last_position_.hdop = gps_raw.eph / 100.0f;
    last_position_.vdop = gps_raw.epv / 100.0f;
    last_position_.ground_speed_mps = gps_raw.vel / 100.0f;
    last_position_.course_deg = gps_raw.cog / 100.0f;
    last_position_.fix_type = gps_raw.fix_type;
    last_position_.satellites_used = gps_raw.satellites_visible;
    last_position_.valid = (gps_raw.fix_type >= 2);  // 2D fix or better
    
    // Update satellite info
    satellite_info_.num_visible = gps_raw.satellites_visible;
    satellite_info_.num_used = gps_raw.satellites_visible;  // Assume all visible are used
}

void MAVLinkGPSParser::HandleGPS2Raw(const uint8_t* payload) {
    // GPS2_RAW has same structure as GPS_RAW_INT but for second GPS
    HandleGPSRawInt(payload);
}

void MAVLinkGPSParser::HandleGlobalPositionInt(const uint8_t* payload) {
    // GLOBAL_POSITION_INT message structure
    struct {
        uint32_t time_boot_ms;
        int32_t lat;  // Latitude in 1E7 degrees
        int32_t lon;  // Longitude in 1E7 degrees
        int32_t alt;  // Altitude in mm (AMSL)
        int32_t relative_alt;  // Altitude above home in mm
        int16_t vx;  // Ground X velocity in cm/s (NED)
        int16_t vy;  // Ground Y velocity in cm/s (NED)
        int16_t vz;  // Ground Z velocity in cm/s (NED)
        uint16_t hdg;  // Heading in cdeg
    } __attribute__((packed)) global_pos;
    
    memcpy(&global_pos, payload, sizeof(global_pos));
    
    // Update position (this is fused/filtered position)
    last_position_.timestamp_ms = GET_TIME_MS();
    last_position_.latitude_deg = global_pos.lat / 1e7;
    last_position_.longitude_deg = global_pos.lon / 1e7;
    last_position_.altitude_m = global_pos.alt / 1000.0f;
    
    // Calculate ground speed from velocity components
    float vx_mps = global_pos.vx / 100.0f;
    float vy_mps = global_pos.vy / 100.0f;
    last_position_.ground_speed_mps = sqrtf(vx_mps * vx_mps + vy_mps * vy_mps);
    last_position_.course_deg = global_pos.hdg / 100.0f;
    
    // Fused position is always valid if we're receiving it
    last_position_.valid = true;
    last_position_.fix_type = 3;  // Assume 3D fix for fused position
}

void MAVLinkGPSParser::HandleGPSStatus(const uint8_t* payload) {
    // GPS_STATUS message structure
    struct {
        uint8_t satellites_visible;
        uint8_t satellite_prn[20];
        uint8_t satellite_used[20];
        uint8_t satellite_elevation[20];
        uint8_t satellite_azimuth[20];
        uint8_t satellite_snr[20];
    } __attribute__((packed)) gps_status;
    
    memcpy(&gps_status, payload, sizeof(gps_status));
    
    // Update satellite info
    satellite_info_.num_visible = gps_status.satellites_visible;
    
    // Count how many satellites are actually used
    uint8_t num_used = 0;
    for (int i = 0; i < gps_status.satellites_visible && i < 20; i++) {
        if (gps_status.satellite_used[i]) {
            num_used++;
        }
        
        // Store individual satellite data if needed
        if (i < 12) {  // We only track up to 12 satellites
            satellite_info_.satellites[i].prn = gps_status.satellite_prn[i];
            satellite_info_.satellites[i].elevation = gps_status.satellite_elevation[i];
            satellite_info_.satellites[i].azimuth = gps_status.satellite_azimuth[i];
            satellite_info_.satellites[i].snr = gps_status.satellite_snr[i];
            satellite_info_.satellites[i].used = gps_status.satellite_used[i];
        }
    }
    satellite_info_.num_used = num_used;
}

void MAVLinkGPSParser::HandleHighLatency2(const uint8_t* payload) {
    // HIGH_LATENCY2 message (compressed telemetry)
    struct {
        uint32_t timestamp;
        uint8_t type;
        uint8_t autopilot;
        uint16_t custom_mode;
        int32_t latitude;   // Latitude in 1E7 degrees
        int32_t longitude;  // Longitude in 1E7 degrees
        int16_t altitude;   // Altitude in meters (AMSL)
        int16_t target_altitude;
        uint8_t heading;    // Heading in 2 degree units
        uint8_t throttle;
        uint16_t airspeed;
        int16_t airspeed_sp;
        uint8_t groundspeed;  // Ground speed in m/s * 5
        int8_t climb_rate;
        uint8_t gps_nsat;
        uint8_t gps_fix_type;
        uint8_t battery;
        int8_t temperature;
        int8_t temperature_air;
        uint8_t failsafe;
        uint8_t wp_num;
        uint16_t wp_distance;
    } __attribute__((packed)) hl2;
    
    memcpy(&hl2, payload, sizeof(hl2));
    
    // Update position from high latency message
    last_position_.timestamp_ms = GET_TIME_MS();
    last_position_.latitude_deg = hl2.latitude / 1e7;
    last_position_.longitude_deg = hl2.longitude / 1e7;
    last_position_.altitude_m = hl2.altitude;
    last_position_.ground_speed_mps = hl2.groundspeed / 5.0f;
    last_position_.course_deg = hl2.heading * 2.0f;
    last_position_.fix_type = hl2.gps_fix_type;
    last_position_.satellites_used = hl2.gps_nsat;
    last_position_.valid = (hl2.gps_fix_type >= 2);
}

void MAVLinkGPSParser::HandleHeartbeat(const uint8_t* payload) {
    // HEARTBEAT message structure
    struct {
        uint32_t custom_mode;
        uint8_t type;
        uint8_t autopilot;
        uint8_t base_mode;
        uint8_t system_status;
        uint8_t mavlink_version;
    } __attribute__((packed)) heartbeat;
    
    memcpy(&heartbeat, payload, sizeof(heartbeat));
    
    // Track autopilot info
    if (!autopilot_detected_) {
        autopilot_sysid_ = current_packet_.sysid;
        autopilot_compid_ = current_packet_.compid;
        autopilot_type_ = heartbeat.autopilot;
        autopilot_detected_ = true;
    }
    
    last_heartbeat_ms_ = GET_TIME_MS();
}

uint16_t MAVLinkGPSParser::CalculateCRC(const uint8_t* buffer, size_t length, uint8_t crc_extra) {
    uint16_t crc = 0xFFFF;
    
    while (length--) {
        uint8_t tmp = *buffer++ ^ (uint8_t)(crc & 0xFF);
        tmp ^= (tmp << 4);
        crc = (crc >> 8) ^ ((uint16_t)tmp << 8) ^ ((uint16_t)tmp << 3) ^ ((uint16_t)(tmp >> 4));
    }
    
    // Add CRC extra byte
    uint8_t tmp = crc_extra ^ (uint8_t)(crc & 0xFF);
    tmp ^= (tmp << 4);
    crc = (crc >> 8) ^ ((uint16_t)tmp << 8) ^ ((uint16_t)tmp << 3) ^ ((uint16_t)(tmp >> 4));
    
    return crc;
}

size_t MAVLinkGPSParser::GetDiagnostics(char* buffer, size_t max_len) const {
    if (!buffer || max_len == 0) return 0;
    
    const char* autopilot_names[] = {
        "Generic", "Reserved", "Slugs", "ArduPilot", "OpenPilot", 
        "Generic_WP", "Generic_WP_Simple", "PX4", "Pixhawk", "Autoquad",
        "UDB", "FP", "SmartAP", "AIRRAILS", "Invalid"
    };
    
    const char* ap_name = "Unknown";
    if (autopilot_type_ < 14) {
        ap_name = autopilot_names[autopilot_type_];
    }
    
    int written = snprintf(buffer, max_len,
        "MAVLink GPS Parser:\n"
        "  Autopilot: %s (ID %d:%d)\n"
        "  Messages: %u (GPS: %u)\n"
        "  Errors: %u\n"
        "  Last heartbeat: %u ms ago\n"
        "  Using: %s\n",
        autopilot_detected_ ? ap_name : "Not detected",
        autopilot_sysid_, autopilot_compid_,
        messages_received_, gps_messages_received_,
        parse_errors_,
        last_heartbeat_ms_ ? (GET_TIME_MS() - last_heartbeat_ms_) : 0,
        use_fused_position_ ? "Fused position" : "Raw GPS"
    );
    
    return (written > 0 && written < static_cast<int>(max_len)) ? written : 0;
}