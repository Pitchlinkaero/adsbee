#ifndef NMEA_PARSER_HH_
#define NMEA_PARSER_HH_

#include "gnss_interface.hh"
#include <cstdio>
#include <cstdlib>
#include <cstring>

/**
 * NMEA 0183 sentence parser implementation.
 * Supports standard NMEA sentences from any GPS receiver.
 * 
 * This is the baseline parser that works with all GPS receivers.
 * Specific receivers (F9P, M10, Septentrio) may have extended parsers.
 */
class NMEAParser : public GNSSInterface {
public:
    static constexpr size_t kNMEAMaxSentenceLen = 82;  // NMEA standard max
    static constexpr size_t kNMEABufferSize = 256;     // Buffer for fragmented sentences
    
    NMEAParser();
    virtual ~NMEAParser() = default;
    
    // GNSSInterface implementation
    bool ParseData(const uint8_t* buffer, size_t length) override;
    Position GetLastPosition() const override { return last_position_; }
    bool Configure(const Config& config) override;
    const char* GetReceiverType() const override { return "NMEA"; }
    bool SupportsPPPService(PPPService service) const override;
    void Reset() override;
    size_t GetDiagnostics(char* buffer, size_t max_len) const override;
    
protected:
    // Sentence parsing functions
    bool ProcessNMEASentence(const char* sentence);
    bool ParseGGA(const char* sentence);  // Position fix
    bool ParseRMC(const char* sentence);  // Recommended minimum
    bool ParseGSA(const char* sentence);  // DOP and active satellites
    bool ParseGSV(const char* sentence);  // Satellites in view
    bool ParseVTG(const char* sentence);  // Track and ground speed
    bool ParseGLL(const char* sentence);  // Geographic position
    bool ParseZDA(const char* sentence);  // Time and date
    
    // Utility functions
    bool ValidateChecksum(const char* sentence) const;
    uint8_t CalculateChecksum(const char* sentence) const;
    double ParseLatLon(const char* field, const char* hemisphere) const;
    double ParseDecimalDegrees(const char* dmm_field) const;
    float ParseFloat(const char* field, float default_val = 0.0f) const;
    int ParseInt(const char* field, int default_val = 0) const;
    bool ParseTime(const char* time_field, uint8_t& hour, uint8_t& min, float& sec) const;
    
    // State management
    Position last_position_;
    Config config_;
    
    // Sentence assembly buffer for fragmented data
    char sentence_buffer_[kNMEABufferSize];
    size_t buffer_pos_ = 0;
    
    // Statistics
    uint32_t sentences_parsed_ = 0;
    uint32_t checksum_errors_ = 0;
    uint32_t parse_errors_ = 0;
    
    // Satellite tracking
    struct SatelliteInfo {
        uint8_t prn = 0;
        uint8_t elevation = 0;
        uint16_t azimuth = 0;
        uint8_t snr = 0;
        bool used = false;
    };
    static constexpr size_t kMaxSatellites = 32;
    SatelliteInfo satellites_[kMaxSatellites];
    uint8_t satellites_in_view_ = 0;
    
    // For handling multi-part GSV messages
    uint8_t gsv_expected_messages_ = 0;
    uint8_t gsv_received_messages_ = 0;
    
    // Track which sentences we've received for position validity
    struct ReceivedSentences {
        bool gga = false;
        bool rmc = false;
        bool gsa = false;
        uint32_t timestamp_ms = 0;
        
        void Reset() {
            gga = false;
            rmc = false;
            gsa = false;
            timestamp_ms = 0;
        }
        
        bool HasMinimumForFix() const {
            return gga || rmc;  // Either GGA or RMC provides position
        }
    } received_;
    
    // Time since last valid fix (for timeout detection)
    uint32_t last_fix_timestamp_ms_ = 0;
    static constexpr uint32_t kFixTimeoutMs = 5000;  // 5 second timeout
};

#endif // NMEA_PARSER_HH_