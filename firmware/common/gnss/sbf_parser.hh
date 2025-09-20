#ifndef SBF_PARSER_HH_
#define SBF_PARSER_HH_

#include "gnss_interface.hh"
#include <cstdint>
#include <vector>

/**
 * SBF (Septentrio Binary Format) parser for Septentrio GNSS receivers.
 * 
 * Supports Septentrio receivers like:
 * - mosaic-X5: Multi-frequency, multi-constellation GNSS module
 * - AsteRx: Professional grade receivers
 * - PolaRx: Reference station receivers
 * 
 * Note: Septentrio receivers also output NMEA, so this parser is optional.
 * Most users will use NMEA output instead of SBF.
 */
class SBFParser : public GNSSInterface {
public:
    SBFParser();
    ~SBFParser() override;
    
    // GNSSInterface implementation
    bool ParseData(const uint8_t* buffer, size_t length) override;
    Position GetLastPosition() const override { return last_position_; }
    bool Configure(const Config& config) override;
    const char* GetReceiverType() const override { return receiver_type_string_; }
    bool SupportsPPPService(PPPService service) const override;
    PPPService GetPPPStatus(float& convergence_percent, uint32_t& eta_seconds) const override;
    bool EnablePPP(PPPService service, const char* key = nullptr) override;
    void Reset() override;
    size_t GetDiagnostics(char* buffer, size_t max_len) const override;
    
    // SBF-specific methods
    enum ReceiverModel {
        kModelUnknown = 0,
        kModelMosaicX5,
        kModelAsteRx,
        kModelPolaRx,
        kModelAsteRxSB,
        kModelAsteRxU
    };
    
    ReceiverModel GetDetectedModel() const { return detected_model_; }
    
private:
    // SBF block IDs (most commonly used)
    static constexpr uint16_t SBF_SYNC1 = 0x24;  // '$'
    static constexpr uint16_t SBF_SYNC2 = 0x40;  // '@'
    
    // Common SBF Block IDs
    static constexpr uint16_t SBF_PVTCartesian = 4007;    // Position, velocity, time in Cartesian
    static constexpr uint16_t SBF_PVTGeodetic = 4006;     // Position, velocity, time in Geodetic
    static constexpr uint16_t SBF_PosCovCartesian = 5905; // Position covariance matrix
    static constexpr uint16_t SBF_PosCovGeodetic = 5906;  // Position covariance geodetic
    static constexpr uint16_t SBF_ReceiverStatus = 4014;  // Overall receiver status
    static constexpr uint16_t SBF_QualityInd = 4082;      // Quality indicators
    static constexpr uint16_t SBF_ReceiverTime = 5914;    // Receiver time information
    static constexpr uint16_t SBF_ExtEventPVT = 4037;     // External event PVT
    static constexpr uint16_t SBF_DOP = 4001;             // Dilution of precision
    static constexpr uint16_t SBF_EndOfPVT = 5921;        // End of PVT epoch marker
    
    // RTK/PPP specific blocks
    static constexpr uint16_t SBF_BaseVectorCart = 4043;  // RTK base vector
    static constexpr uint16_t SBF_BaseVectorGeod = 4028;  // RTK base vector geodetic
    static constexpr uint16_t SBF_PVTResiduals = 4009;    // PVT residuals
    
    // Parser state machine
    enum ParserState {
        kWaitingSync1,
        kWaitingSync2,
        kReadingHeader,
        kReceivingData,
        kCheckingCRC
    };
    
    // SBF Block Header (8 bytes)
    struct SBFHeader {
        uint8_t sync1;      // '$'
        uint8_t sync2;      // '@'
        uint16_t crc16;     // CRC-16 of header
        uint16_t id;        // Block ID
        uint8_t revision;   // Block revision
        uint8_t length;     // Block length = 8 + data_length (multiple of 4)
    };
    
    // SBF Block structure
    struct SBFBlock {
        SBFHeader header;
        std::vector<uint8_t> data;
        uint16_t data_crc;  // CRC at end of data
    };
    
    // Parsing methods
    bool ProcessByte(uint8_t byte);
    bool ProcessBlock(const SBFBlock& block);
    uint16_t CalculateCRC(const uint8_t* data, size_t length);
    bool ValidateHeader(const SBFHeader& header);
    uint32_t GetTimeMs() const;
    
    // Block handlers
    bool HandlePVTGeodetic(const uint8_t* data, size_t length);
    bool HandlePVTCartesian(const uint8_t* data, size_t length);
    bool HandleReceiverStatus(const uint8_t* data, size_t length);
    bool HandleQualityIndicators(const uint8_t* data, size_t length);
    bool HandleBaseVector(const uint8_t* data, size_t length);
    bool HandleDOP(const uint8_t* data, size_t length);
    
    // Configuration methods
    bool SendCommand(const char* command);
    bool ConfigureOutput(uint8_t rate_hz);
    bool EnablePPPInternal();
    bool EnableRTK();
    
    // Auto-detection
    bool DetectReceiverModel();
    
    // Helper methods
    double ConvertRadiansToDegrees(double radians);
    void UpdateFixType();
    
    // Member variables
    ParserState parser_state_ = kWaitingSync1;
    SBFBlock current_block_;
    std::vector<uint8_t> header_buffer_;
    size_t data_index_ = 0;
    size_t header_index_ = 0;
    
    Position last_position_;
    ReceiverModel detected_model_ = kModelUnknown;
    char receiver_type_string_[32] = "Septentrio";
    
    // Navigation solution details
    uint8_t nr_sv_ = 0;           // Number of satellites used
    uint8_t solution_type_ = 0;    // 0=No fix, 1=Standalone, 4=RTK Fixed, 5=RTK Float
    float pdop_ = 99.9f;           // Position dilution of precision
    float hdop_ = 99.9f;           // Horizontal dilution of precision
    float vdop_ = 99.9f;           // Vertical dilution of precision
    
    // Quality indicators
    float position_rms_m_ = 999.0f;
    float velocity_rms_ms_ = 999.0f;
    uint32_t last_pvt_tow_ms_ = 0;  // GPS time of week in ms
    
    // RTK/PPP data
    bool rtk_enabled_ = false;
    bool ppp_enabled_ = false;
    float baseline_length_m_ = 0.0f;
    uint32_t correction_age_ms_ = 0;
    uint8_t carrier_phase_status_ = 0;  // 0=No RTK, 1=Float, 2=Fixed
    
    // Statistics
    uint32_t blocks_parsed_ = 0;
    uint32_t parse_errors_ = 0;
    uint32_t crc_errors_ = 0;
    
    // CRC-16-CCITT lookup table
    static const uint16_t crc16_table_[256];
};

#endif // SBF_PARSER_HH_