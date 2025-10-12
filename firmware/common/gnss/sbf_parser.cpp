#include "sbf_parser.hh"
#include <cstring>
#include <cmath>
#include <cstdio>

// CRC-16-CCITT lookup table for SBF
const uint16_t SBFParser::crc16_table_[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

SBFParser::SBFParser() {
    Reset();
}

SBFParser::~SBFParser() = default;

void SBFParser::Reset() {
    parser_state_ = kWaitingSync1;
    current_block_ = {};
    header_buffer_.clear();
    data_index_ = 0;
    header_index_ = 0;
    
    last_position_ = {};
    blocks_parsed_ = 0;
    parse_errors_ = 0;
    crc_errors_ = 0;
}

bool SBFParser::ParseData(const uint8_t* buffer, size_t length) {
    for (size_t i = 0; i < length; i++) {
        if (!ProcessByte(buffer[i])) {
            parse_errors_++;
        }
    }
    return true;
}

bool SBFParser::ProcessByte(uint8_t byte) {
    switch (parser_state_) {
        case kWaitingSync1:
            if (byte == SBF_SYNC1) {
                header_buffer_.clear();
                header_buffer_.push_back(byte);
                parser_state_ = kWaitingSync2;
            }
            break;
            
        case kWaitingSync2:
            if (byte == SBF_SYNC2) {
                header_buffer_.push_back(byte);
                parser_state_ = kReadingHeader;
                header_index_ = 2;
            } else {
                parser_state_ = kWaitingSync1;
            }
            break;
            
        case kReadingHeader:
            header_buffer_.push_back(byte);
            header_index_++;
            
            if (header_index_ >= 8) {
                // Parse header
                memcpy(&current_block_.header, header_buffer_.data(), 8);
                
                // Validate header CRC
                uint16_t calc_crc = CalculateCRC(&header_buffer_[4], 4);
                uint16_t header_crc = (header_buffer_[2] | (header_buffer_[3] << 8));
                
                if (calc_crc != header_crc) {
                    crc_errors_++;
                    parser_state_ = kWaitingSync1;
                    return false;
                }
                
                // Setup for data reception
                uint16_t data_length = current_block_.header.length - 8;
                current_block_.data.clear();
                current_block_.data.reserve(data_length);
                data_index_ = 0;
                
                if (data_length > 0) {
                    parser_state_ = kReceivingData;
                } else {
                    // No data, process block
                    ProcessBlock(current_block_);
                    parser_state_ = kWaitingSync1;
                }
            }
            break;
            
        case kReceivingData:
            current_block_.data.push_back(byte);
            data_index_++;
            
            if (data_index_ >= static_cast<size_t>(current_block_.header.length - 8)) {
                parser_state_ = kCheckingCRC;
            }
            break;
            
        case kCheckingCRC:
            // Last 2 bytes are CRC
            current_block_.data.push_back(byte);
            data_index_++;
            
            if (data_index_ >= static_cast<size_t>(current_block_.header.length - 6)) {
                // Validate data CRC
                size_t data_len = current_block_.data.size() - 2;
                uint16_t calc_crc = CalculateCRC(current_block_.data.data(), data_len);
                uint16_t data_crc = current_block_.data[data_len] | 
                                   (current_block_.data[data_len + 1] << 8);
                
                if (calc_crc == data_crc) {
                    ProcessBlock(current_block_);
                    blocks_parsed_++;
                } else {
                    crc_errors_++;
                }
                
                parser_state_ = kWaitingSync1;
            }
            break;
    }
    
    return true;
}

bool SBFParser::ProcessBlock(const SBFBlock& block) {
    switch (block.header.id & 0x1FFF) {  // Mask out sub-block number
        case SBF_PVTGeodetic:
            return HandlePVTGeodetic(block.data.data(), block.data.size() - 2);
            
        case SBF_PVTCartesian:
            return HandlePVTCartesian(block.data.data(), block.data.size() - 2);
            
        case SBF_ReceiverStatus:
            return HandleReceiverStatus(block.data.data(), block.data.size() - 2);
            
        case SBF_QualityInd:
            return HandleQualityIndicators(block.data.data(), block.data.size() - 2);
            
        case SBF_BaseVectorCart:
        case SBF_BaseVectorGeod:
            return HandleBaseVector(block.data.data(), block.data.size() - 2);
            
        case SBF_DOP:
            return HandleDOP(block.data.data(), block.data.size() - 2);
            
        case SBF_EndOfPVT:
            // End of PVT epoch marker, update fix type
            UpdateFixType();
            return true;
            
        default:
            // Unknown or unhandled block
            return true;
    }
}

bool SBFParser::HandlePVTGeodetic(const uint8_t* data, size_t length) {
    if (length < 80) return false;  // Minimum size for PVTGeodetic
    
    // Extract time of week (ms) and GPS week
    uint32_t tow_ms = *reinterpret_cast<const uint32_t*>(&data[0]);
    uint16_t week = *reinterpret_cast<const uint16_t*>(&data[4]);
    
    // Set GPS time in Position struct
    last_position_.gps_time_ms = tow_ms;
    last_position_.gps_time_week = week;
    
    // Extract position (radians and meters)
    double lat_rad = *reinterpret_cast<const double*>(&data[8]);
    double lon_rad = *reinterpret_cast<const double*>(&data[16]);
    double height_m = *reinterpret_cast<const double*>(&data[24]);
    
    // Extract velocity (m/s)
    float vn = *reinterpret_cast<const float*>(&data[40]);
    float ve = *reinterpret_cast<const float*>(&data[44]);
    float vu = *reinterpret_cast<const float*>(&data[48]);
    
    // Extract quality indicators
    uint8_t mode = data[70];  // GNSS mode
    uint8_t error = data[71]; // Error code
    nr_sv_ = data[72];         // Number of satellites
    
    // Update position
    last_position_.latitude_deg = ConvertRadiansToDegrees(lat_rad);
    last_position_.longitude_deg = ConvertRadiansToDegrees(lon_rad);
    last_position_.altitude_msl_m = height_m;
    last_position_.altitude_m = height_m;  // HAE approximately same as MSL for Septentrio
    
    // Calculate ground speed and track
    float speed_ms = sqrt(vn * vn + ve * ve);
    last_position_.ground_speed_mps = speed_ms;
    last_position_.track_deg = atan2(ve, vn) * 180.0 / M_PI;
    if (last_position_.track_deg < 0) {
        last_position_.track_deg += 360.0;
    }
    
    last_position_.vertical_velocity_mps = vu;
    last_position_.satellites_used = nr_sv_;
    
    // Set solution type based on mode
    solution_type_ = mode & 0x0F;
    last_position_.valid = (error == 0) && (nr_sv_ >= 4);
    
    // Update timestamp
    last_position_.timestamp_ms = GetTimeMs();
    last_pvt_tow_ms_ = tow_ms;
    
    return true;
}

bool SBFParser::HandlePVTCartesian(const uint8_t* data, size_t length) {
    if (length < 72) return false;  // Minimum size for PVTCartesian
    
    // Extract Cartesian coordinates
    double x = *reinterpret_cast<const double*>(&data[8]);
    double y = *reinterpret_cast<const double*>(&data[16]);
    double z = *reinterpret_cast<const double*>(&data[24]);
    
    // Convert ECEF to geodetic (simplified)
    // This is approximate, proper conversion requires iterative method
    double p = sqrt(x * x + y * y);
    double theta = atan2(z, p * 0.99664719);
    
    double lat_rad = atan2(z + 42.69561 * sin(theta) * sin(theta) * sin(theta),
                           p - 42.84131 * cos(theta) * cos(theta) * cos(theta));
    double lon_rad = atan2(y, x);
    double height = p / cos(lat_rad) - 6378137.0;  // Approximate
    
    last_position_.latitude_deg = ConvertRadiansToDegrees(lat_rad);
    last_position_.longitude_deg = ConvertRadiansToDegrees(lon_rad);
    last_position_.altitude_msl_m = height;
    
    return true;
}

bool SBFParser::HandleReceiverStatus(const uint8_t* data, size_t length) {
    if (length < 16) return false;
    
    // Extract receiver status fields
    // uint8_t cpu_load = data[8];  // For future diagnostics
    // uint8_t ext_error = data[9];  // For future error reporting
    uint32_t rx_status = *reinterpret_cast<const uint32_t*>(&data[12]);
    
    // Check for RTK/PPP status bits
    rtk_enabled_ = (rx_status & 0x0100) != 0;
    ppp_enabled_ = (rx_status & 0x0200) != 0;
    
    return true;
}

bool SBFParser::HandleQualityIndicators(const uint8_t* data, size_t length) {
    if (length < 40) return false;
    
    // Extract quality metrics
    position_rms_m_ = *reinterpret_cast<const float*>(&data[8]);
    velocity_rms_ms_ = *reinterpret_cast<const float*>(&data[12]);
    hdop_ = *reinterpret_cast<const float*>(&data[16]);
    vdop_ = *reinterpret_cast<const float*>(&data[20]);
    pdop_ = *reinterpret_cast<const float*>(&data[24]);
    
    // Update accuracy estimate and DOP values
    last_position_.accuracy_horizontal_m = position_rms_m_;
    last_position_.accuracy_vertical_m = position_rms_m_ * 1.5;  // Approximate
    last_position_.hdop = hdop_;
    last_position_.vdop = vdop_;
    last_position_.pdop = pdop_;
    
    return true;
}

bool SBFParser::HandleBaseVector(const uint8_t* data, size_t length) {
    if (length < 60) return false;
    
    // Extract RTK baseline
    float delta_x = *reinterpret_cast<const float*>(&data[8]);
    float delta_y = *reinterpret_cast<const float*>(&data[12]);
    float delta_z = *reinterpret_cast<const float*>(&data[16]);
    
    baseline_length_m_ = sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);
    
    // Extract carrier phase status
    carrier_phase_status_ = data[48];  // 0=No RTK, 1=Float, 2=Fixed
    
    // Update RTK fields in Position struct
    last_position_.rtk_available = (carrier_phase_status_ > 0);
    last_position_.rtk_baseline_m = baseline_length_m_;
    
    return true;
}

bool SBFParser::HandleDOP(const uint8_t* data, size_t length) {
    if (length < 20) return false;
    
    pdop_ = *reinterpret_cast<const float*>(&data[8]);
    hdop_ = *reinterpret_cast<const float*>(&data[12]);
    vdop_ = *reinterpret_cast<const float*>(&data[16]);
    
    // Update Position struct with DOP values
    last_position_.hdop = hdop_;
    last_position_.vdop = vdop_;
    last_position_.pdop = pdop_;
    
    return true;
}

void SBFParser::UpdateFixType() {
    if (!last_position_.valid) {
        last_position_.fix_type = FixType::kNoFix;  // No fix
        last_position_.ppp_status = 0;
    } else if (carrier_phase_status_ == 2) {
        last_position_.fix_type = FixType::kRTKFixed;  // RTK Fixed
        last_position_.ppp_status = 0;  // RTK, not PPP
    } else if (carrier_phase_status_ == 1) {
        last_position_.fix_type = FixType::kRTKFloat;  // RTK Float
        last_position_.ppp_status = 0;  // RTK, not PPP
    } else if (solution_type_ >= 4 && ppp_enabled_) {
        // PPP solution
        if (position_rms_m_ < 0.5) {
            last_position_.fix_type = FixType::kPPPConverged;
            last_position_.ppp_status = 2;  // Converged
        } else {
            last_position_.fix_type = FixType::kPPPConverging;
            last_position_.ppp_status = 1;  // Converging
        }
        last_position_.ppp_service = PPPService::kPPPSBAS;  // Default for Septentrio
    } else if (solution_type_ >= 4) {
        last_position_.fix_type = FixType::kGNSSDGPS;  // 3D fix with SBAS (no PPP)
        last_position_.ppp_status = 0;
    } else {
        last_position_.fix_type = FixType::k3DFix;  // Standard 3D fix
        last_position_.ppp_status = 0;
    }
}

uint16_t SBFParser::CalculateCRC(const uint8_t* data, size_t length) {
    uint16_t crc = 0;
    
    for (size_t i = 0; i < length; i++) {
        crc = (crc << 8) ^ crc16_table_[((crc >> 8) ^ data[i]) & 0xFF];
    }
    
    return crc;
}

double SBFParser::ConvertRadiansToDegrees(double radians) {
    return radians * 180.0 / M_PI;
}

bool SBFParser::Configure(const Config& config) {
    // Configure output rate
    if (config.update_rate_hz > 0) {
        ConfigureOutput(config.update_rate_hz);
    }
    
    // Enable PPP if requested
    if (config.ppp_service != PPPService::kPPPNone) {
        EnablePPPInternal();
    }
    
    // Enable RTK if supported
    if (config.enable_rtk) {
        EnableRTK();
    }
    
    return true;
}

bool SBFParser::ConfigureOutput(uint8_t rate_hz) {
    // Send Septentrio command to set output rate
    // setSBFOutput, Stream1, PVTGeodetic+ReceiverStatus+QualityInd, <interval>
    char cmd[128];
    uint32_t interval_ms = 1000 / rate_hz;
    snprintf(cmd, sizeof(cmd), "setSBFOutput, Stream1, PVTGeodetic+ReceiverStatus+QualityInd, %lums\n",
             (unsigned long)interval_ms);
    return SendCommand(cmd);
}

bool SBFParser::EnablePPPInternal() {
    // Enable Septentrio's PPP mode
    ppp_enabled_ = true;
    return SendCommand("setPPPMode, on\n");
}

bool SBFParser::EnableRTK() {
    // Enable RTK mode
    rtk_enabled_ = true;
    return SendCommand("setRTKMode, on\n");
}

bool SBFParser::SendCommand(const char* command) {
    // Commands would be sent to receiver via UART
    // This is a placeholder - actual implementation would use output callback
    return true;
}

bool SBFParser::SupportsPPPService(GNSSInterface::PPPService service) const {
    switch (service) {
        case PPPService::kPPPSBAS:
            return true;  // All Septentrio receivers support SBAS
        case PPPService::kPPPGalileoHAS:
            return detected_model_ >= kModelMosaicX5;  // Newer models only
        case PPPService::kPPPAuto:
            return true;
        default:
            return false;
    }
}

GNSSInterface::PPPService SBFParser::GetPPPStatus(float& convergence_percent, uint32_t& eta_seconds) const {
    if (!ppp_enabled_) {
        convergence_percent = 0;
        eta_seconds = 0;
        return PPPService::kPPPNone;
    }
    
    // Estimate convergence based on accuracy
    if (position_rms_m_ < 0.5) {
        convergence_percent = 100;
        eta_seconds = 0;
    } else if (position_rms_m_ < 2.0) {
        convergence_percent = 50;
        eta_seconds = 300;  // 5 minutes
    } else {
        convergence_percent = 10;
        eta_seconds = 1200;  // 20 minutes
    }
    
    return PPPService::kPPPSBAS;  // Default to SBAS
}

bool SBFParser::EnablePPP(GNSSInterface::PPPService service, const char* key) {
    switch (service) {
        case PPPService::kPPPSBAS:
            return SendCommand("setSBASMode, on\n");
        case PPPService::kPPPGalileoHAS:
            if (detected_model_ >= kModelMosaicX5) {
                return SendCommand("setGalileoHAS, on\n");
            }
            return false;
        default:
            return false;
    }
}

uint32_t SBFParser::GetTimeMs() const {
    // Placeholder - would use actual system time
    return last_pvt_tow_ms_;
}

size_t SBFParser::GetDiagnostics(char* buffer, size_t max_len) const {
    return snprintf(buffer, max_len,
                   "SBF Parser Diagnostics:\n"
                   "Receiver: %s\n"
                   "Blocks parsed: %lu\n"
                   "Parse errors: %lu\n"
                   "CRC errors: %lu\n"
                   "Solution: %s\n"
                   "Satellites: %u\n"
                   "PDOP: %.1f\n"
                   "Position RMS: %.2fm\n"
                   "RTK: %s\n"
                   "Baseline: %.1fm\n",
                   receiver_type_string_,
                   (unsigned long)blocks_parsed_,
                   (unsigned long)parse_errors_,
                   (unsigned long)crc_errors_,
                   last_position_.valid ? "Valid" : "Invalid",
                   nr_sv_,
                   pdop_,
                   position_rms_m_,
                   carrier_phase_status_ == 2 ? "Fixed" :
                   carrier_phase_status_ == 1 ? "Float" : "None",
                   baseline_length_m_);
}