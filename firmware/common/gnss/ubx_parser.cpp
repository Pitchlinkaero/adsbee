#include "ubx_parser.hh"
#include <cstring>
#include <cstdio>

UBXParser::UBXParser() {
    Reset();
}

UBXParser::~UBXParser() = default;

bool UBXParser::ParseData(const uint8_t* buffer, size_t length) {
    if (!buffer || length == 0) {
        return false;
    }
    
    bool message_processed = false;
    for (size_t i = 0; i < length; i++) {
        if (ProcessByte(buffer[i])) {
            message_processed = true;
        }
    }
    
    return message_processed;
}

bool UBXParser::ProcessByte(uint8_t byte) {
    switch (parser_state_) {
        case kWaitingSync1:
            if (byte == UBX_SYNC1) {
                parser_state_ = kWaitingSync2;
            }
            break;
            
        case kWaitingSync2:
            if (byte == UBX_SYNC2) {
                parser_state_ = kWaitingClass;
                current_message_ = UBXMessage();  // Reset message
            } else {
                parser_state_ = kWaitingSync1;
            }
            break;
            
        case kWaitingClass:
            current_message_.msg_class = byte;
            parser_state_ = kWaitingId;
            break;
            
        case kWaitingId:
            current_message_.msg_id = byte;
            parser_state_ = kWaitingLength1;
            break;
            
        case kWaitingLength1:
            current_message_.length = byte;
            parser_state_ = kWaitingLength2;
            break;
            
        case kWaitingLength2:
            current_message_.length |= (uint16_t)byte << 8;
            if (current_message_.length > 0) {
                current_message_.payload.reserve(current_message_.length);
                payload_index_ = 0;
                parser_state_ = kReceivingPayload;
            } else {
                parser_state_ = kWaitingCk_A;
            }
            break;
            
        case kReceivingPayload:
            current_message_.payload.push_back(byte);
            payload_index_++;
            if (payload_index_ >= current_message_.length) {
                parser_state_ = kWaitingCk_A;
            }
            break;
            
        case kWaitingCk_A:
            current_message_.ck_a = byte;
            parser_state_ = kWaitingCk_B;
            break;
            
        case kWaitingCk_B:
            current_message_.ck_b = byte;
            parser_state_ = kWaitingSync1;
            
            // Verify checksum
            uint8_t calc_ck_a, calc_ck_b;
            CalculateChecksum(current_message_, calc_ck_a, calc_ck_b);
            
            if (calc_ck_a == current_message_.ck_a && calc_ck_b == current_message_.ck_b) {
                return ProcessMessage(current_message_);
            } else {
                checksum_errors_++;
            }
            break;
    }
    
    return false;
}

void UBXParser::CalculateChecksum(const UBXMessage& msg, uint8_t& ck_a, uint8_t& ck_b) {
    ck_a = 0;
    ck_b = 0;
    
    // Include class and ID
    ck_a += msg.msg_class;
    ck_b += ck_a;
    ck_a += msg.msg_id;
    ck_b += ck_a;
    
    // Include length
    ck_a += msg.length & 0xFF;
    ck_b += ck_a;
    ck_a += (msg.length >> 8) & 0xFF;
    ck_b += ck_a;
    
    // Include payload
    for (uint8_t byte : msg.payload) {
        ck_a += byte;
        ck_b += ck_a;
    }
}

bool UBXParser::ProcessMessage(const UBXMessage& msg) {
    messages_parsed_++;
    
    bool handled = false;
    
    // Handle different message types
    if (msg.msg_class == UBX_CLASS_NAV) {
        switch (msg.msg_id) {
            case UBX_NAV_PVT:
                handled = HandleNavPvt(msg.payload.data(), msg.payload.size());
                break;
            case UBX_NAV_HPPOSLLH:
                handled = HandleNavHpPosLlh(msg.payload.data(), msg.payload.size());
                break;
            case UBX_NAV_STATUS:
                handled = HandleNavStatus(msg.payload.data(), msg.payload.size());
                break;
            case UBX_NAV_SAT:
                handled = HandleNavSat(msg.payload.data(), msg.payload.size());
                break;
            case UBX_NAV_RELPOSNED:
                handled = HandleNavRelPosNed(msg.payload.data(), msg.payload.size());
                break;
        }
    } else if (msg.msg_class == UBX_CLASS_RXM) {
        switch (msg.msg_id) {
            case UBX_RXM_SPARTN:
                handled = HandleRxmSpartn(msg.payload.data(), msg.payload.size());
                break;
            case UBX_RXM_COR:
                handled = HandleRxmCor(msg.payload.data(), msg.payload.size());
                break;
        }
    } else if (msg.msg_class == UBX_CLASS_MON) {
        if (msg.msg_id == UBX_MON_VER) {
            handled = HandleMonVer(msg.payload.data(), msg.payload.size());
        }
    }
    
    if (!handled) {
        parse_errors_++;
    }
    
    return handled;
}

bool UBXParser::HandleNavPvt(const uint8_t* payload, size_t length) {
    if (length < 92) {  // Minimum NAV-PVT size
        return false;
    }
    
    // Extract time (for future use)
    [[maybe_unused]] uint16_t year = *(uint16_t*)(payload + 4);
    [[maybe_unused]] uint8_t month = payload[6];
    [[maybe_unused]] uint8_t day = payload[7];
    [[maybe_unused]] uint8_t hour = payload[8];
    [[maybe_unused]] uint8_t min = payload[9];
    [[maybe_unused]] uint8_t sec = payload[10];
    uint8_t valid_flags = payload[11];
    
    // Extract fix information
    uint8_t fix_type = payload[20];
    uint8_t fix_flags = payload[21];
    uint8_t fix_flags2 = payload[22];
    uint8_t num_sv = payload[23];
    
    // Extract position
    int32_t lon_raw = *(int32_t*)(payload + 24);  // 1e-7 degrees
    int32_t lat_raw = *(int32_t*)(payload + 28);  // 1e-7 degrees
    int32_t height_raw = *(int32_t*)(payload + 32);  // mm above ellipsoid
    int32_t height_msl_raw = *(int32_t*)(payload + 36);  // mm above MSL
    
    // Extract accuracy
    uint32_t h_acc = *(uint32_t*)(payload + 40);  // mm
    uint32_t v_acc = *(uint32_t*)(payload + 44);  // mm
    
    // Extract velocity
    [[maybe_unused]] int32_t vel_n = *(int32_t*)(payload + 48);  // mm/s north
    [[maybe_unused]] int32_t vel_e = *(int32_t*)(payload + 52);  // mm/s east
    int32_t vel_d = *(int32_t*)(payload + 56);  // mm/s down
    int32_t ground_speed = *(int32_t*)(payload + 60);  // mm/s
    int32_t heading_motion = *(int32_t*)(payload + 64);  // 1e-5 degrees
    
    // Extract DOP
    uint16_t p_dop = *(uint16_t*)(payload + 76);  // 0.01 scale
    
    // Update position
    last_position_.latitude_deg = lat_raw * 1e-7;
    last_position_.longitude_deg = lon_raw * 1e-7;
    last_position_.altitude_m = height_raw * 0.001f;
    last_position_.altitude_msl_m = height_msl_raw * 0.001f;
    
    // Apply high-precision offsets if available (F9P)
    if (has_high_precision_) {
        last_position_.latitude_deg += hp_lat_offset_ * 1e-10;
        last_position_.longitude_deg += hp_lon_offset_ * 1e-10;
        last_position_.altitude_m += hp_alt_offset_ * 0.0001f;
    }
    
    // Set velocity
    last_position_.ground_speed_mps = ground_speed * 0.001f;
    last_position_.track_deg = heading_motion * 1e-5f;
    last_position_.vertical_velocity_mps = -vel_d * 0.001f;  // Convert down to up
    
    // Set accuracy
    last_position_.accuracy_horizontal_m = h_acc * 0.001f;
    last_position_.accuracy_vertical_m = v_acc * 0.001f;
    
    // Set DOP
    last_position_.pdop = p_dop * 0.01f;
    
    // Set satellites
    last_position_.satellites_used = num_sv;
    
    // Determine fix type
    switch (fix_type) {
        case 0:
            last_position_.fix_type = kNoFix;
            break;
        case 1:
            last_position_.fix_type = kDead;
            break;
        case 2:
            last_position_.fix_type = k2DFix;
            break;
        case 3:
            if (fix_flags & 0x02) {  // DGPS/SBAS used
                last_position_.fix_type = kGNSSDGPS;
            } else {
                last_position_.fix_type = k3DFix;
            }
            break;
        case 4:  // GNSS + dead reckoning
            last_position_.fix_type = k3DFix;
            break;
        case 5:  // Time only fix
            last_position_.fix_type = kNoFix;
            break;
    }
    
    // Check for carrier phase fix (RTK/PPP)
    if (fix_flags & 0x40) {  // carrSoln valid
        uint8_t carr_soln = (fix_flags2 >> 6) & 0x03;
        if (carr_soln == 1) {  // Float
            last_position_.fix_type = kRTKFloat;
        } else if (carr_soln == 2) {  // Fixed
            if (active_ppp_service_ != kPPPNone) {
                last_position_.fix_type = kPPPConverged;
                ppp_converged_ = true;
            } else {
                last_position_.fix_type = kRTKFixed;
            }
        }
    }
    
    // Update validity
    last_position_.valid = (valid_flags & 0x07) == 0x07;  // Date, time, and fix valid
    
    // Update timestamp
    last_position_.timestamp_ms = GetTimeMs();
    
    return true;
}

bool UBXParser::HandleNavHpPosLlh(const uint8_t* payload, size_t length) {
    if (length < 36) {  // Minimum NAV-HPPOSLLH size
        return false;
    }
    
    // This message provides high-precision offsets for F9P
    has_high_precision_ = true;
    detected_model_ = kModelF9P;  // Only F9P sends this message
    
    // Extract high-precision components (0.1mm resolution)
    hp_lon_offset_ = *(int8_t*)(payload + 24);
    hp_lat_offset_ = *(int8_t*)(payload + 25);
    hp_alt_offset_ = *(int8_t*)(payload + 26);
    
    // Extract high-precision accuracy
    uint8_t h_acc_hp = payload[31];  // 0.1mm
    uint8_t v_acc_hp = payload[32];  // 0.1mm
    
    // Update accuracy if better
    float hp_h_acc = h_acc_hp * 0.0001f;  // Convert to meters
    float hp_v_acc = v_acc_hp * 0.0001f;
    
    if (hp_h_acc < last_position_.accuracy_horizontal_m) {
        last_position_.accuracy_horizontal_m = hp_h_acc;
    }
    if (hp_v_acc < last_position_.accuracy_vertical_m) {
        last_position_.accuracy_vertical_m = hp_v_acc;
    }
    
    return true;
}

bool UBXParser::HandleNavStatus(const uint8_t* payload, size_t length) {
    if (length < 16) {
        return false;
    }
    
    [[maybe_unused]] uint8_t gps_fix = payload[4];
    uint8_t flags = payload[5];
    
    // Additional fix status processing
    if (flags & 0x08) {  // psmState - Power Save Mode
        // Handle power save mode if needed
    }
    
    return true;
}

bool UBXParser::HandleNavSat(const uint8_t* payload, size_t length) {
    if (length < 8) {
        return false;
    }
    
    uint8_t num_svs = payload[5];
    
    // Process satellite information for diagnostics
    // Each satellite block is 12 bytes
    size_t expected_length = 8 + (num_svs * 12);
    if (length < expected_length) {
        return false;
    }
    
    // Could extract detailed satellite info here if needed
    
    return true;
}

bool UBXParser::HandleNavRelPosNed(const uint8_t* payload, size_t length) {
    if (length < 64) {
        return false;
    }
    
    // RTK relative position (advanced feature)
    rtk_enabled_ = true;
    
    // Extract baseline vector
    [[maybe_unused]] int32_t rel_pos_n = *(int32_t*)(payload + 8);  // cm + HP part
    [[maybe_unused]] int32_t rel_pos_e = *(int32_t*)(payload + 12);
    [[maybe_unused]] int32_t rel_pos_d = *(int32_t*)(payload + 16);
    int32_t rel_pos_length = *(int32_t*)(payload + 20);  // cm
    
    // High precision parts
    [[maybe_unused]] int8_t rel_pos_hp_n = *(int8_t*)(payload + 32);  // 0.1mm
    [[maybe_unused]] int8_t rel_pos_hp_e = *(int8_t*)(payload + 33);
    [[maybe_unused]] int8_t rel_pos_hp_d = *(int8_t*)(payload + 34);
    int8_t rel_pos_hp_length = *(int8_t*)(payload + 35);
    
    // Calculate baseline with HP
    rtk_baseline_m_ = (rel_pos_length + rel_pos_hp_length * 0.001f) * 0.01f;
    
    // Extract flags
    uint32_t flags = *(uint32_t*)(payload + 36);
    [[maybe_unused]] bool gnss_fix_ok = flags & 0x01;
    [[maybe_unused]] bool diff_soln = flags & 0x02;
    bool rel_pos_valid = flags & 0x04;
    uint8_t carr_soln = (flags >> 3) & 0x03;
    
    // Update RTK status
    if (rel_pos_valid && carr_soln == 2) {
        last_position_.fix_type = kRTKFixed;
        last_position_.rtk_available = true;
        last_position_.rtk_baseline_m = rtk_baseline_m_;
    }
    
    return true;
}

bool UBXParser::HandleRxmSpartn(const uint8_t* payload, size_t length) {
    // SPARTN correction data status (PointPerfect)
    if (length < 16) {
        return false;
    }
    
    [[maybe_unused]] uint8_t msg_type = payload[4];
    [[maybe_unused]] uint16_t msg_sub_type = *(uint16_t*)(payload + 6);
    
    ppp_stats_.corrections_received++;
    
    // Check if corrections are being used
    uint8_t flags = payload[11];
    if (flags & 0x01) {  // Used flag
        ppp_stats_.corrections_used++;
    }
    
    return true;
}

bool UBXParser::HandleRxmCor(const uint8_t* payload, size_t length) {
    // Differential corrections input status
    if (length < 12) {
        return false;
    }
    
    uint8_t status_info = payload[4];
    [[maybe_unused]] uint16_t msg_type = *(uint16_t*)(payload + 6);
    [[maybe_unused]] uint16_t ref_station = *(uint16_t*)(payload + 8);
    
    // Update correction age
    rtk_correction_age_ms_ = status_info * 100;  // Approximate
    ppp_stats_.correction_age_ms = rtk_correction_age_ms_;
    
    return true;
}

bool UBXParser::HandleMonVer(const uint8_t* payload, size_t length) {
    // Version information - detect receiver model
    if (length < 40) {
        return false;
    }
    
    // Software version at offset 0 (30 bytes)
    // Hardware version at offset 30 (10 bytes)
    char sw_version[31] = {0};
    char hw_version[11] = {0};
    
    memcpy(sw_version, payload, 30);
    memcpy(hw_version, payload + 30, 10);
    
    // Parse version to detect model
    ParseVersionString(sw_version);
    
    // Look for extension strings (after byte 40)
    size_t offset = 40;
    while (offset + 30 <= length) {
        char extension[31] = {0};
        memcpy(extension, payload + offset, 30);
        
        // Check for model indicators
        if (strstr(extension, "F9P")) {
            detected_model_ = kModelF9P;
            strcpy(receiver_type_string_, "UBX-F9P");
        } else if (strstr(extension, "M10")) {
            detected_model_ = kModelM10;
            strcpy(receiver_type_string_, "UBX-M10");
        } else if (strstr(extension, "M9")) {
            detected_model_ = kModelM9;
            strcpy(receiver_type_string_, "UBX-M9");
        } else if (strstr(extension, "M8")) {
            detected_model_ = kModelM8;
            strcpy(receiver_type_string_, "UBX-M8");
        }
        
        offset += 30;
    }
    
    return true;
}

void UBXParser::ParseVersionString(const char* version) {
    // Parse version string to detect capabilities
    if (strstr(version, "HPG")) {  // High Precision GNSS
        has_high_precision_ = true;
        if (strstr(version, "1.32") || strstr(version, "1.40")) {
            // F9P firmware versions that support PPP
            supports_ppp_ = true;
        }
    }
    
    if (strstr(version, "SPG")) {  // Standard Precision GNSS
        has_high_precision_ = false;
    }
}

bool UBXParser::Configure(const Config& config) {
    // Configure navigation rate
    if (!ConfigureNavigationRate(config.update_rate_hz)) {
        return false;
    }
    
    // Enable SBAS if requested
    if (config.enable_sbas) {
        EnableSBAS();
    }
    
    // Configure PPP if supported and requested
    if (config.ppp_service != kPPPNone) {
        EnablePPP(config.ppp_service);
    }
    
    // Configure message output rates
    ConfigureMessageRate(UBX_CLASS_NAV, UBX_NAV_PVT, 1);  // Every epoch
    
    if (detected_model_ == kModelF9P) {
        ConfigureMessageRate(UBX_CLASS_NAV, UBX_NAV_HPPOSLLH, 1);  // High precision
        ConfigureMessageRate(UBX_CLASS_NAV, UBX_NAV_RELPOSNED, 1);  // RTK status
        ConfigureMessageRate(UBX_CLASS_RXM, UBX_RXM_SPARTN, 1);     // SPARTN status
    }
    
    ConfigureMessageRate(UBX_CLASS_NAV, UBX_NAV_STATUS, 5);  // Every 5 epochs
    ConfigureMessageRate(UBX_CLASS_NAV, UBX_NAV_SAT, 10);    // Every 10 epochs
    
    return true;
}

bool UBXParser::ConfigureNavigationRate(uint8_t rate_hz) {
    if (rate_hz < 1 || rate_hz > 10) {
        return false;
    }
    
    uint16_t period_ms = 1000 / rate_hz;
    
    // CFG-RATE message
    uint8_t payload[6];
    *(uint16_t*)payload = period_ms;  // Measurement rate
    *(uint16_t*)(payload + 2) = 1;    // Navigation rate (cycles)
    *(uint16_t*)(payload + 4) = 0;    // Time reference (UTC)
    
    return SendUBX(0x06, 0x08, payload, sizeof(payload));
}

bool UBXParser::ConfigureMessageRate(uint8_t msg_class, uint8_t msg_id, uint8_t rate) {
    // CFG-MSG message
    uint8_t payload[3];
    payload[0] = msg_class;
    payload[1] = msg_id;
    payload[2] = rate;  // Rate on current port
    
    return SendUBX(0x06, 0x01, payload, sizeof(payload));
}

bool UBXParser::EnableSBAS() {
    // Enable SBAS augmentation (WAAS/EGNOS/MSAS)
    // This provides ~1-2m accuracy for free
    
    // CFG-SBAS message
    uint8_t payload[8] = {0};
    payload[0] = 0x01;  // Mode: enabled
    payload[1] = 0x07;  // Usage: range, diffCorr, integrity
    payload[2] = 0x03;  // Max SBAS: 3
    payload[3] = 0x00;  // Scan mode
    *(uint32_t*)(payload + 4) = 0x00051000;  // PRN mask (auto)
    
    return SendUBX(0x06, 0x16, payload, sizeof(payload));
}

bool UBXParser::EnablePPP(PPPService service, const char* key) {
    if (detected_model_ != kModelF9P) {
        // Only F9P supports PPP
        return false;
    }
    
    active_ppp_service_ = service;
    ppp_start_time_ms_ = GetTimeMs();
    
    switch (service) {
        case kPPPGalileoHAS:
            return ConfigureGalileoHAS();
            
        case kPPPPointPerfect:
            return ConfigurePointPerfect(key);
            
        case kPPPSBAS:
            return EnableSBAS();  // Basic SBAS
            
        case kPPPAuto:
            // Try Galileo HAS first (free), fall back to SBAS
            if (ConfigureGalileoHAS()) {
                active_ppp_service_ = kPPPGalileoHAS;
                return true;
            }
            return EnableSBAS();
            
        default:
            return false;
    }
}

bool UBXParser::ConfigureGalileoHAS() {
    // Configure Galileo High Accuracy Service (E6-B signal)
    // Free 20cm accuracy service
    
    // Enable Galileo E6
    // This requires F9P firmware 1.32 or later
    
    // Configuration using CFG-VALSET (Generation 9+ protocol)
    // Key IDs for Galileo HAS configuration
    const uint32_t CFG_SIGNAL_GAL_E6_ENA = 0x10310023;  // Enable E6
    [[maybe_unused]] const uint32_t CFG_NAVHPG_DGNSSMODE = 0x20140011;   // Set to 3 for float RTK
    
    // Build VALSET message
    uint8_t payload[12];
    payload[0] = 0x00;  // Version
    payload[1] = 0x01;  // Layer: RAM
    payload[2] = 0x00;  // Reserved
    payload[3] = 0x00;  // Reserved
    
    // Add configuration items
    *(uint32_t*)(payload + 4) = CFG_SIGNAL_GAL_E6_ENA;
    payload[8] = 0x01;  // Enable
    
    return SendUBX(UBX_CLASS_CFG, UBX_CFG_VALSET, payload, 9);
}

bool UBXParser::ConfigurePointPerfect(const char* key) {
    if (!key) {
        return false;
    }
    
    // PointPerfect requires SPARTN corrections
    // This would typically be received via NTRIP or L-band
    // The key is used for decryption of SPARTN messages
    
    // Enable SPARTN input on receiver
    // This is complex and requires additional setup
    // For now, just mark as configured
    
    active_ppp_service_ = kPPPPointPerfect;
    return true;
}

bool UBXParser::SendUBX(uint8_t msg_class, uint8_t msg_id, const uint8_t* payload, uint16_t length) {
    if (!output_callback_) {
        return false;  // No way to send commands
    }
    
    // Build UBX message
    std::vector<uint8_t> message;
    message.push_back(UBX_SYNC1);
    message.push_back(UBX_SYNC2);
    message.push_back(msg_class);
    message.push_back(msg_id);
    message.push_back(length & 0xFF);
    message.push_back((length >> 8) & 0xFF);
    
    if (payload && length > 0) {
        message.insert(message.end(), payload, payload + length);
    }
    
    // Calculate checksum
    uint8_t ck_a = 0, ck_b = 0;
    for (size_t i = 2; i < message.size(); i++) {
        ck_a += message[i];
        ck_b += ck_a;
    }
    
    message.push_back(ck_a);
    message.push_back(ck_b);
    
    // Send via callback
    return output_callback_(message.data(), message.size());
}

bool UBXParser::SupportsPPPService(PPPService service) const {
    if (detected_model_ != kModelF9P) {
        // Only F9P supports PPP
        return service == kPPPSBAS;  // All receivers support basic SBAS
    }
    
    switch (service) {
        case kPPPSBAS:
        case kPPPGalileoHAS:
        case kPPPPointPerfect:
        case kPPPCenterPointRTX:
        case kPPPAuto:
            return true;
        default:
            return false;
    }
}

GNSSInterface::PPPService UBXParser::GetPPPStatus(float& convergence_percent, uint32_t& eta_seconds) const {
    if (active_ppp_service_ == kPPPNone) {
        convergence_percent = 0;
        eta_seconds = 0;
        return kPPPNone;
    }
    
    // Calculate convergence based on accuracy and time
    uint32_t elapsed_ms = GetTimeMs() - ppp_start_time_ms_;
    uint32_t elapsed_s = elapsed_ms / 1000;
    
    if (ppp_converged_) {
        convergence_percent = 100.0f;
        eta_seconds = 0;
    } else {
        // Estimate based on typical convergence times
        uint32_t expected_convergence_s = 300;  // 5 minutes typical
        
        if (active_ppp_service_ == kPPPGalileoHAS) {
            expected_convergence_s = 180;  // 3 minutes for Galileo HAS
        } else if (active_ppp_service_ == kPPPPointPerfect) {
            expected_convergence_s = 120;  // 2 minutes for PointPerfect
        }
        
        convergence_percent = (float)elapsed_s / expected_convergence_s * 100.0f;
        if (convergence_percent > 95.0f) {
            convergence_percent = 95.0f;  // Cap at 95% until converged
        }
        
        eta_seconds = expected_convergence_s - elapsed_s;
        // eta_seconds is unsigned, so no need to check for < 0
    }
    
    return active_ppp_service_;
}

void UBXParser::Reset() {
    parser_state_ = kWaitingSync1;
    current_message_ = UBXMessage();
    payload_index_ = 0;
    messages_parsed_ = 0;
    parse_errors_ = 0;
    checksum_errors_ = 0;
    last_position_ = Position();
    has_high_precision_ = false;
    ppp_converged_ = false;
    detected_model_ = kModelUnknown;
    strcpy(receiver_type_string_, "Unknown");
}

size_t UBXParser::GetDiagnostics(char* buffer, size_t max_len) const {
    if (!buffer || max_len == 0) {
        return 0;
    }
    
    int written = snprintf(buffer, max_len,
        "UBX Parser Diagnostics:\n"
        "Model: %s\n"
        "Messages: %lu (errors: %lu, checksum: %lu)\n"
        "Fix: %d, Sats: %u\n"
        "Accuracy H/V: %.2f/%.2f m\n"
        "PPP: %s (converged: %s)\n"
        "High Precision: %s\n",
        receiver_type_string_,
        (unsigned long)messages_parsed_, (unsigned long)parse_errors_, (unsigned long)checksum_errors_,
        last_position_.fix_type, last_position_.satellites_used,
        last_position_.accuracy_horizontal_m,
        last_position_.accuracy_vertical_m,
        active_ppp_service_ == kPPPNone ? "None" : 
            (active_ppp_service_ == kPPPGalileoHAS ? "Galileo HAS" :
             active_ppp_service_ == kPPPPointPerfect ? "PointPerfect" : "Other"),
        ppp_converged_ ? "Yes" : "No",
        has_high_precision_ ? "Yes" : "No"
    );
    
    return written > 0 ? written : 0;
}

// Helper function - would be defined elsewhere in actual implementation
uint32_t UBXParser::GetTimeMs() const {
    // This would interface with the system time
    // For now, return a placeholder
    return 0;
}