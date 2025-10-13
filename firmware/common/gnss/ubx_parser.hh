#ifndef UBX_PARSER_HH_
#define UBX_PARSER_HH_

#include "gnss_interface.hh"
#include <cstdint>
#include <vector>
#include <map>
#include <functional>

/**
 * UBX Protocol parser for u-blox GPS receivers (M10, F9P, etc).
 * 
 * Handles binary UBX protocol for high-precision GNSS with PPP support.
 * Primary receiver types:
 * - M10: Basic positioning (1-2m accuracy with SBAS)
 * - F9P: High-precision with PPP support (20cm with Galileo HAS, 3-6cm with PointPerfect)
 * 
 * PPP Services supported on F9P:
 * - Galileo HAS (free, 20cm)
 * - PointPerfect/SPARTN (3-6cm, subscription)
 * - RTX (2-4cm, subscription)
 */
class UBXParser : public GNSSInterface {
public:
    UBXParser();
    ~UBXParser() override;
    
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
    
    // UBX-specific methods
    enum ReceiverModel {
        kModelUnknown = 0,
        kModelM8,     // Legacy
        kModelM9,     // Legacy
        kModelM10,    // Current basic
        kModelF9P,    // High precision with PPP
        kModelF9R,    // High precision + IMU
        kModelD9S     // Correction service receiver
    };
    
    ReceiverModel GetDetectedModel() const { return detected_model_; }
    
    // PPP correction statistics (F9P only)
    struct PPPStats {
        uint32_t corrections_received = 0;
        uint32_t corrections_used = 0;
        uint32_t correction_age_ms = 0;
        float baseline_accuracy_m = 999.0f;
        uint8_t convergence_status = 0; // 0=none, 1=converging, 2=converged
    };
    
    PPPStats GetPPPStats() const { return ppp_stats_; }
    
private:
    // UBX message IDs
    static constexpr uint8_t UBX_SYNC1 = 0xB5;
    static constexpr uint8_t UBX_SYNC2 = 0x62;
    
    // Message classes
    static constexpr uint8_t UBX_CLASS_NAV = 0x01;
    static constexpr uint8_t UBX_CLASS_RXM = 0x02;
    static constexpr uint8_t UBX_CLASS_CFG = 0x06;
    static constexpr uint8_t UBX_CLASS_MON = 0x0A;
    static constexpr uint8_t UBX_CLASS_ACK = 0x05;
    
    // NAV message IDs
    static constexpr uint8_t UBX_NAV_PVT = 0x07;      // Position Velocity Time
    static constexpr uint8_t UBX_NAV_HPPOSLLH = 0x14; // High Precision Position
    static constexpr uint8_t UBX_NAV_STATUS = 0x03;   // Receiver Navigation Status
    static constexpr uint8_t UBX_NAV_SAT = 0x35;      // Satellite Information
    static constexpr uint8_t UBX_NAV_SVIN = 0x3B;     // Survey-in data (for RTK base)
    static constexpr uint8_t UBX_NAV_RELPOSNED = 0x3C;// Relative position (RTK)
    
    // RXM message IDs (for PPP)
    static constexpr uint8_t UBX_RXM_SPARTN = 0x33;   // SPARTN input status
    static constexpr uint8_t UBX_RXM_COR = 0x34;      // Differential correction input
    
    // Configuration message IDs
    static constexpr uint8_t UBX_CFG_VALSET = 0x8A;   // Set configuration (Gen 9+)
    static constexpr uint8_t UBX_CFG_VALGET = 0x8B;   // Get configuration (Gen 9+)
    static constexpr uint8_t UBX_CFG_VALDEL = 0x8C;   // Delete configuration (Gen 9+)
    
    // MON message IDs
    static constexpr uint8_t UBX_MON_VER = 0x04;      // Version information
    static constexpr uint8_t UBX_MON_HW = 0x09;       // Hardware status
    
    // Parser state machine
    enum ParserState {
        kWaitingSync1,
        kWaitingSync2,
        kWaitingClass,
        kWaitingId,
        kWaitingLength1,
        kWaitingLength2,
        kReceivingPayload,
        kWaitingCk_A,
        kWaitingCk_B
    };
    
    // Message structure
    struct UBXMessage {
        uint8_t msg_class = 0;
        uint8_t msg_id = 0;
        uint16_t length = 0;
        std::vector<uint8_t> payload;
        uint8_t ck_a = 0;
        uint8_t ck_b = 0;
    };
    
    // Parsing methods
    bool ProcessByte(uint8_t byte);
    bool ProcessMessage(const UBXMessage& msg);
    void CalculateChecksum(const UBXMessage& msg, uint8_t& ck_a, uint8_t& ck_b);
    uint32_t GetTimeMs() const;
    
    // Message handlers
    bool HandleNavPvt(const uint8_t* payload, size_t length);
    bool HandleNavHpPosLlh(const uint8_t* payload, size_t length);
    bool HandleNavStatus(const uint8_t* payload, size_t length);
    bool HandleNavSat(const uint8_t* payload, size_t length);
    bool HandleNavRelPosNed(const uint8_t* payload, size_t length);
    bool HandleRxmSpartn(const uint8_t* payload, size_t length);
    bool HandleRxmCor(const uint8_t* payload, size_t length);
    bool HandleMonVer(const uint8_t* payload, size_t length);
    
    // Configuration methods
    bool SendUBX(uint8_t msg_class, uint8_t msg_id, const uint8_t* payload = nullptr, uint16_t length = 0);
    bool ConfigureMessageRate(uint8_t msg_class, uint8_t msg_id, uint8_t rate);
    bool ConfigureNavigationRate(uint8_t rate_hz);
    bool ConfigurePPP_F9P();
    bool ConfigureGalileoHAS();
    bool ConfigureIGSRTS();
    bool ConfigurePointPerfect(const char* key);
    bool ConfigureBeiDouB2b();
    bool EnableSBAS();
    
    // Auto-detection
    bool DetectReceiverModel();
    void ParseVersionString(const char* version);
    
    // Member variables
    ParserState parser_state_ = kWaitingSync1;
    UBXMessage current_message_;
    size_t payload_index_ = 0;
    
    Position last_position_;
    ReceiverModel detected_model_ = kModelUnknown;
    char receiver_type_string_[32] = "Unknown";
    
    // High precision extensions (F9P)
    bool has_high_precision_ = false;
    int32_t hp_lat_offset_ = 0;  // in 0.1mm
    int32_t hp_lon_offset_ = 0;  // in 0.1mm
    int32_t hp_alt_offset_ = 0;  // in 0.1mm
    
    // PPP management
    PPPService active_ppp_service_ = kPPPNone;
    PPPStats ppp_stats_;
    uint32_t ppp_start_time_ms_ = 0;
    bool ppp_converged_ = false;
    bool supports_ppp_ = false;
    
    // RTK data (optional/advanced)
    bool rtk_enabled_ = false;
    float rtk_baseline_m_ = 0.0f;
    uint32_t rtk_correction_age_ms_ = 0;
    
    // Statistics
    uint32_t messages_parsed_ = 0;
    uint32_t parse_errors_ = 0;
    uint32_t checksum_errors_ = 0;
    
    // Output callback for sending configuration
    using OutputCallback = std::function<bool(const uint8_t*, size_t)>;
    OutputCallback output_callback_ = nullptr;
    
public:
    // Set output callback for sending UBX commands
    void SetOutputCallback(OutputCallback callback) { output_callback_ = callback; }
};

#endif // UBX_PARSER_HH_