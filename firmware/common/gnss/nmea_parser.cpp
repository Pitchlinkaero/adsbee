#include "nmea_parser.hh"
#include <cmath>
#include <cstring>

#ifdef ON_EMBEDDED_DEVICE
#include "pico/stdlib.h"
#define GET_TIME_MS() to_ms_since_boot(get_absolute_time())
#else
#include <chrono>
#define GET_TIME_MS() std::chrono::duration_cast<std::chrono::milliseconds>(\
    std::chrono::steady_clock::now().time_since_epoch()).count()
#endif

NMEAParser::NMEAParser() {
    Reset();
}

bool NMEAParser::ParseData(const uint8_t* buffer, size_t length) {
    if (!buffer || length == 0) return false;
    
    bool parsed_any = false;
    
    for (size_t i = 0; i < length; i++) {
        char c = static_cast<char>(buffer[i]);
        
        // Start of sentence
        if (c == '$' || c == '!') {
            buffer_pos_ = 0;
            sentence_buffer_[buffer_pos_++] = c;
        }
        // Add to buffer if we're in a sentence
        else if (buffer_pos_ > 0 && buffer_pos_ < kNMEABufferSize - 1) {
            sentence_buffer_[buffer_pos_++] = c;
            
            // End of sentence (CR or LF)
            if (c == '\r' || c == '\n') {
                sentence_buffer_[buffer_pos_] = '\0';
                
                // Process complete sentence
                if (buffer_pos_ > 10 && sentence_buffer_[0] == '$') {
                    if (ProcessNMEASentence(sentence_buffer_)) {
                        parsed_any = true;
                        sentences_parsed_++;
                    }
                }
                buffer_pos_ = 0;
            }
        }
        // Buffer overflow protection
        else if (buffer_pos_ >= kNMEABufferSize - 1) {
            buffer_pos_ = 0;
            parse_errors_++;
        }
    }
    
    // Update position validity based on timeout
    uint32_t now_ms = GET_TIME_MS();
    if (last_position_.valid && (now_ms - last_fix_timestamp_ms_) > kFixTimeoutMs) {
        last_position_.valid = false;
        last_position_.fix_type = kNoFix;
    }
    
    return parsed_any;
}

bool NMEAParser::ProcessNMEASentence(const char* sentence) {
    if (!sentence || strlen(sentence) < 10) return false;
    
    // Validate checksum first
    if (!ValidateChecksum(sentence)) {
        checksum_errors_++;
        return false;
    }
    
    // Identify sentence type (skip $ and talker ID)
    const char* type = sentence + 3;  // Skip $GP, $GN, $GL, etc.
    
    bool success = false;
    
    if (strncmp(type, "GGA,", 4) == 0) {
        success = ParseGGA(sentence);
        if (success) received_.gga = true;
    }
    else if (strncmp(type, "RMC,", 4) == 0) {
        success = ParseRMC(sentence);
        if (success) received_.rmc = true;
    }
    else if (strncmp(type, "GSA,", 4) == 0) {
        success = ParseGSA(sentence);
        if (success) received_.gsa = true;
    }
    else if (strncmp(type, "GSV,", 4) == 0) {
        success = ParseGSV(sentence);
    }
    else if (strncmp(type, "VTG,", 4) == 0) {
        success = ParseVTG(sentence);
    }
    else if (strncmp(type, "GLL,", 4) == 0) {
        success = ParseGLL(sentence);
    }
    else if (strncmp(type, "ZDA,", 4) == 0) {
        success = ParseZDA(sentence);
    }
    
    // Update timestamp if we got valid data
    if (success && received_.HasMinimumForFix()) {
        last_fix_timestamp_ms_ = GET_TIME_MS();
        last_position_.timestamp_ms = last_fix_timestamp_ms_;
        last_position_.valid = true;
    }
    
    return success;
}

bool NMEAParser::ParseGGA(const char* sentence) {
    // $xxGGA,time,lat,NS,lon,EW,quality,numSV,HDOP,alt,M,sep,M,diffAge,diffStation*cs
    
    char temp[kNMEAMaxSentenceLen];
    strncpy(temp, sentence, sizeof(temp) - 1);
    temp[sizeof(temp) - 1] = '\0';
    
    // Remove checksum
    char* checksum_pos = strchr(temp, '*');
    if (checksum_pos) *checksum_pos = '\0';
    
    // Tokenize
    char* saveptr;
    char* token = strtok_r(temp, ",", &saveptr);
    if (!token) return false;
    
    int field = 0;
    uint8_t quality = 0;
    
    while (token != nullptr) {
        switch (field) {
            case 1: // Time (HHMMSS.sss)
                // Parse time if needed
                break;
                
            case 2: // Latitude (DDMM.mmmm)
                if (strlen(token) > 0) {
                    double lat = ParseDecimalDegrees(token);
                    token = strtok_r(nullptr, ",", &saveptr);
                    field++;
                    if (token && token[0] == 'S') lat = -lat;
                    last_position_.latitude_deg = lat;
                }
                break;
                
            case 4: // Longitude (DDDMM.mmmm)
                if (strlen(token) > 0) {
                    double lon = ParseDecimalDegrees(token);
                    token = strtok_r(nullptr, ",", &saveptr);
                    field++;
                    if (token && token[0] == 'W') lon = -lon;
                    last_position_.longitude_deg = lon;
                }
                break;
                
            case 6: // Fix quality
                quality = ParseInt(token, 0);
                switch (quality) {
                    case 0: last_position_.fix_type = kNoFix; break;
                    case 1: last_position_.fix_type = k3DFix; break;
                    case 2: last_position_.fix_type = kGNSSDGPS; break;
                    case 4: last_position_.fix_type = kRTKFixed; break;
                    case 5: last_position_.fix_type = kRTKFloat; break;
                    case 6: last_position_.fix_type = kDead; break;
                    default: last_position_.fix_type = k3DFix; break;
                }
                break;
                
            case 7: // Number of satellites
                last_position_.satellites_used = ParseInt(token, 0);
                break;
                
            case 8: // HDOP
                last_position_.hdop = ParseFloat(token, 99.99f);
                break;
                
            case 9: // Altitude (MSL)
                last_position_.altitude_msl_m = ParseFloat(token, 0.0f);
                last_position_.altitude_m = last_position_.altitude_msl_m; // Use MSL as primary
                break;
                
            case 11: // Geoid separation
                {
                    float sep = ParseFloat(token, 0.0f);
                    last_position_.altitude_m = last_position_.altitude_msl_m + sep; // HAE
                }
                break;
        }
        
        token = strtok_r(nullptr, ",", &saveptr);
        field++;
    }
    
    return quality > 0;
}

bool NMEAParser::ParseRMC(const char* sentence) {
    // $xxRMC,time,status,lat,NS,lon,EW,speed,course,date,magvar,EW,mode*cs
    
    char temp[kNMEAMaxSentenceLen];
    strncpy(temp, sentence, sizeof(temp) - 1);
    temp[sizeof(temp) - 1] = '\0';
    
    char* checksum_pos = strchr(temp, '*');
    if (checksum_pos) *checksum_pos = '\0';
    
    char* saveptr;
    char* token = strtok_r(temp, ",", &saveptr);
    if (!token) return false;
    
    int field = 0;
    bool valid = false;
    
    while (token != nullptr) {
        switch (field) {
            case 2: // Status (A=active, V=void)
                valid = (token[0] == 'A');
                if (!valid) {
                    last_position_.fix_type = kNoFix;
                    last_position_.valid = false;
                }
                break;
                
            case 3: // Latitude
                if (strlen(token) > 0 && valid) {
                    double lat = ParseDecimalDegrees(token);
                    token = strtok_r(nullptr, ",", &saveptr);
                    field++;
                    if (token && token[0] == 'S') lat = -lat;
                    last_position_.latitude_deg = lat;
                }
                break;
                
            case 5: // Longitude
                if (strlen(token) > 0 && valid) {
                    double lon = ParseDecimalDegrees(token);
                    token = strtok_r(nullptr, ",", &saveptr);
                    field++;
                    if (token && token[0] == 'W') lon = -lon;
                    last_position_.longitude_deg = lon;
                }
                break;
                
            case 7: // Speed over ground (knots)
                if (strlen(token) > 0) {
                    float knots = ParseFloat(token, 0.0f);
                    last_position_.ground_speed_mps = knots * 0.514444f; // knots to m/s
                }
                break;
                
            case 8: // Track angle
                if (strlen(token) > 0) {
                    last_position_.track_deg = ParseFloat(token, 0.0f);
                }
                break;
                
            case 9: // Date (DDMMYY)
                // Parse date if needed
                break;
        }
        
        token = strtok_r(nullptr, ",", &saveptr);
        field++;
    }
    
    if (valid && last_position_.fix_type == kNoFix) {
        last_position_.fix_type = k2DFix; // RMC typically indicates at least 2D fix
    }
    
    return valid;
}

bool NMEAParser::ParseGSA(const char* sentence) {
    // $xxGSA,mode,fixType,prn1,prn2,...,prn12,PDOP,HDOP,VDOP*cs
    
    char temp[kNMEAMaxSentenceLen];
    strncpy(temp, sentence, sizeof(temp) - 1);
    temp[sizeof(temp) - 1] = '\0';
    
    char* checksum_pos = strchr(temp, '*');
    if (checksum_pos) *checksum_pos = '\0';
    
    char* saveptr;
    char* token = strtok_r(temp, ",", &saveptr);
    if (!token) return false;
    
    int field = 0;
    uint8_t fix_type = 0;
    
    while (token != nullptr) {
        switch (field) {
            case 2: // Fix type (1=none, 2=2D, 3=3D)
                fix_type = ParseInt(token, 0);
                if (fix_type == 1) last_position_.fix_type = kNoFix;
                else if (fix_type == 2) last_position_.fix_type = k2DFix;
                else if (fix_type == 3 && last_position_.fix_type < k3DFix) {
                    last_position_.fix_type = k3DFix;
                }
                break;
                
            case 15: // PDOP
                last_position_.pdop = ParseFloat(token, 99.99f);
                break;
                
            case 16: // HDOP
                last_position_.hdop = ParseFloat(token, 99.99f);
                break;
                
            case 17: // VDOP
                last_position_.vdop = ParseFloat(token, 99.99f);
                break;
        }
        
        token = strtok_r(nullptr, ",", &saveptr);
        field++;
    }
    
    // Estimate accuracy from DOP values (rough approximation)
    if (last_position_.hdop < 99.0f) {
        last_position_.accuracy_horizontal_m = last_position_.hdop * 2.5f; // Rough estimate
    }
    if (last_position_.vdop < 99.0f) {
        last_position_.accuracy_vertical_m = last_position_.vdop * 2.5f;
    }
    
    return fix_type > 1;
}

bool NMEAParser::ParseGSV(const char* sentence) {
    // $xxGSV,numMsg,msgNum,numSV,prn,elev,azim,snr,...*cs
    // This is for satellite tracking info (not critical for position)
    
    // For now, just count satellites in view
    char temp[kNMEAMaxSentenceLen];
    strncpy(temp, sentence, sizeof(temp) - 1);
    temp[sizeof(temp) - 1] = '\0';
    
    char* checksum_pos = strchr(temp, '*');
    if (checksum_pos) *checksum_pos = '\0';
    
    char* saveptr;
    char* token = strtok_r(temp, ",", &saveptr);
    if (!token) return false;
    
    int field = 0;
    
    while (token != nullptr && field < 4) {
        if (field == 3) { // Number of satellites in view
            satellites_in_view_ = ParseInt(token, 0);
        }
        token = strtok_r(nullptr, ",", &saveptr);
        field++;
    }
    
    return true;
}

bool NMEAParser::ParseVTG(const char* sentence) {
    // $xxVTG,course,T,course,M,speed,N,speed,K,mode*cs
    // Track and ground speed
    
    char temp[kNMEAMaxSentenceLen];
    strncpy(temp, sentence, sizeof(temp) - 1);
    temp[sizeof(temp) - 1] = '\0';
    
    char* checksum_pos = strchr(temp, '*');
    if (checksum_pos) *checksum_pos = '\0';
    
    char* saveptr;
    char* token = strtok_r(temp, ",", &saveptr);
    if (!token) return false;
    
    int field = 0;
    
    while (token != nullptr) {
        switch (field) {
            case 1: // True track
                if (strlen(token) > 0) {
                    last_position_.track_deg = ParseFloat(token, 0.0f);
                }
                break;
                
            case 7: // Ground speed (km/h)
                if (strlen(token) > 0) {
                    float kmh = ParseFloat(token, 0.0f);
                    last_position_.ground_speed_mps = kmh / 3.6f; // km/h to m/s
                }
                break;
        }
        
        token = strtok_r(nullptr, ",", &saveptr);
        field++;
    }
    
    return true;
}

bool NMEAParser::ParseGLL(const char* sentence) {
    // Similar to RMC but with just position
    // Implementation similar to ParseRMC for position extraction
    return true; // Simplified for now
}

bool NMEAParser::ParseZDA(const char* sentence) {
    // Time and date - useful for GPS time sync
    return true; // Simplified for now
}

bool NMEAParser::ValidateChecksum(const char* sentence) const {
    if (!sentence || sentence[0] != '$') return false;
    
    const char* checksum_pos = strchr(sentence, '*');
    if (!checksum_pos || strlen(checksum_pos) < 3) return false;
    
    // Calculate checksum between $ and *
    uint8_t calculated = 0;
    for (const char* p = sentence + 1; p < checksum_pos; p++) {
        calculated ^= static_cast<uint8_t>(*p);
    }
    
    // Parse provided checksum
    char hex[3] = { checksum_pos[1], checksum_pos[2], '\0' };
    uint8_t provided = static_cast<uint8_t>(strtoul(hex, nullptr, 16));
    
    return calculated == provided;
}

uint8_t NMEAParser::CalculateChecksum(const char* sentence) const {
    uint8_t checksum = 0;
    
    // Skip initial $ if present
    const char* start = sentence;
    if (*start == '$') start++;
    
    // XOR all bytes until * or end
    while (*start && *start != '*') {
        checksum ^= static_cast<uint8_t>(*start);
        start++;
    }
    
    return checksum;
}

double NMEAParser::ParseDecimalDegrees(const char* dmm_field) const {
    if (!dmm_field || strlen(dmm_field) < 3) return 0.0;
    
    // NMEA format: DDMM.mmmm or DDDMM.mmmm
    double value = atof(dmm_field);
    int degrees = static_cast<int>(value / 100.0);
    double minutes = value - (degrees * 100.0);
    
    return degrees + (minutes / 60.0);
}

float NMEAParser::ParseFloat(const char* field, float default_val) const {
    if (!field || strlen(field) == 0) return default_val;
    char* endptr;
    float val = strtof(field, &endptr);
    return (endptr != field) ? val : default_val;
}

int NMEAParser::ParseInt(const char* field, int default_val) const {
    if (!field || strlen(field) == 0) return default_val;
    char* endptr;
    long val = strtol(field, &endptr, 10);
    return (endptr != field) ? static_cast<int>(val) : default_val;
}

bool NMEAParser::ParseTime(const char* time_field, uint8_t& hour, uint8_t& min, float& sec) const {
    if (!time_field || strlen(time_field) < 6) return false;
    
    // HHMMSS.sss format
    char temp[3] = {0};
    
    temp[0] = time_field[0]; temp[1] = time_field[1];
    hour = static_cast<uint8_t>(atoi(temp));
    
    temp[0] = time_field[2]; temp[1] = time_field[3];
    min = static_cast<uint8_t>(atoi(temp));
    
    sec = ParseFloat(time_field + 4, 0.0f);
    
    return true;
}

bool NMEAParser::Configure(const Config& config) {
    config_ = config;
    
    // NMEA is passive - we can't configure the receiver
    // But we can use config for our internal processing
    
    return true;
}

bool NMEAParser::SupportsPPPService(PPPService service) const {
    // Basic NMEA only supports SBAS
    return (service == kPPPSBAS || service == kPPPAuto);
}

void NMEAParser::Reset() {
    last_position_ = Position();
    buffer_pos_ = 0;
    memset(sentence_buffer_, 0, sizeof(sentence_buffer_));
    sentences_parsed_ = 0;
    checksum_errors_ = 0;
    parse_errors_ = 0;
    received_.Reset();
    satellites_in_view_ = 0;
    gsv_expected_messages_ = 0;
    gsv_received_messages_ = 0;
    last_fix_timestamp_ms_ = 0;
}

size_t NMEAParser::GetDiagnostics(char* buffer, size_t max_len) const {
    if (!buffer || max_len == 0) return 0;
    
    int written = snprintf(buffer, max_len,
        "NMEA Parser Diagnostics:\n"
        "  Sentences parsed: %lu\n"
        "  Checksum errors: %lu\n"
        "  Parse errors: %lu\n"
        "  Satellites in view: %u\n"
        "  Last fix: %s\n"
        "  Position: %.6f, %.6f @ %.1fm\n"
        "  HDOP: %.2f, Sats used: %u\n",
        sentences_parsed_,
        checksum_errors_,
        parse_errors_,
        satellites_in_view_,
        last_position_.valid ? "Valid" : "Invalid",
        last_position_.latitude_deg,
        last_position_.longitude_deg,
        last_position_.altitude_m,
        last_position_.hdop,
        last_position_.satellites_used
    );
    
    return (written > 0 && written < static_cast<int>(max_len)) ? written : 0;
}