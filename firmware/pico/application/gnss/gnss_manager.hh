#ifndef GNSS_MANAGER_HH_
#define GNSS_MANAGER_HH_

#include "gnss_interface.hh"
#include "gps_settings.hh"
#include "nmea_parser.hh"
#include <memory>
#include <cstdint>

// Forward declarations for specific parsers (to be implemented)
class UBXParser;
class SBFParser;
class MAVLinkGPSParser;

/**
 * GNSS Manager - Central coordinator for all GPS/GNSS functionality.
 * 
 * Responsibilities:
 * - Manage GPS data sources (UART, Network, MAVLink)
 * - Handle parser selection and switching
 * - Coordinate PPP services
 * - Provide unified interface for position data
 * - Handle failover between sources
 * 
 * This runs on the RP2040 to balance CPU load with ESP32.
 */
class GNSSManager {
public:
    GNSSManager();
    ~GNSSManager();
    
    /**
     * Initialize the GNSS subsystem with settings
     * @param settings GPS configuration settings
     * @return true if initialization successful
     */
    bool Initialize(const GPSSettings& settings);
    
    /**
     * Process incoming data from current source
     * @param buffer Data buffer
     * @param length Data length in bytes
     * @return true if data was processed successfully
     */
    bool ProcessData(const uint8_t* buffer, size_t length);
    
    /**
     * Process GPS message from network (received via ESP32)
     * @param type Message type (NMEA, MAVLink, etc)
     * @param buffer Data buffer
     * @param length Data length in bytes
     * @return true if data was processed successfully
     */
    bool ProcessNetworkGPSMessage(uint8_t type, const uint8_t* buffer, size_t length);
    
    /**
     * Update position from current source
     * Should be called periodically (at configured rate)
     * @return true if position was updated
     */
    bool UpdatePosition();
    
    /**
     * Get current position
     * @return Current GPS position (may be invalid if no fix)
     */
    GNSSInterface::Position GetCurrentPosition() const;
    
    /**
     * Check if position is valid
     * @return true if we have a valid GPS fix
     */
    bool IsPositionValid() const;
    
    /**
     * Get current GPS source
     * @return Active GPS source
     */
    GPSSettings::GPSSource GetCurrentSource() const { return current_source_; }
    
    /**
     * Switch to a different GPS source
     * @param source New GPS source
     * @return true if switch successful
     */
    bool SetSource(GPSSettings::GPSSource source);
    
    /**
     * Enable PPP service
     * @param service PPP service to enable (or kPPPAuto)
     * @return true if PPP enabled
     */
    bool EnablePPP(GNSSInterface::PPPService service = GNSSInterface::kPPPAuto);
    
    /**
     * Get PPP status
     * @param convergence_percent Output: convergence percentage
     * @param eta_seconds Output: estimated time to convergence
     * @return Current PPP service
     */
    GNSSInterface::PPPService GetPPPStatus(float& convergence_percent, uint32_t& eta_seconds) const;
    
    /**
     * Configure update rate
     * @param hz Update rate in Hz (1-5)
     * @return true if rate set successfully
     */
    bool SetUpdateRate(uint8_t hz);
    
    /**
     * Get diagnostics string
     * @param buffer Output buffer
     * @param max_len Maximum buffer length
     * @return Bytes written
     */
    size_t GetDiagnostics(char* buffer, size_t max_len) const;
    
    /**
     * Check if receiver supports high precision
     * @return true if F9P, Septentrio, or other high-precision receiver detected
     */
    bool SupportsHighPrecision() const { return supports_high_precision_; }
    
    /**
     * Get receiver type string
     * @return String identifying detected receiver
     */
    const char* GetReceiverType() const;
    
    // Statistics
    struct Statistics {
        uint32_t messages_processed = 0;
        uint32_t position_updates = 0;
        uint32_t parse_errors = 0;
        uint32_t source_switches = 0;
        uint32_t ppp_convergence_time_s = 0;
        float best_accuracy_m = 999.0f;
        uint32_t uptime_s = 0;
    };
    
    Statistics GetStatistics() const { return stats_; }
    
private:
    // Internal methods
    bool InitializeUART();
    bool InitializeNetworkSource();
    bool InitializeMAVLinkSource();
    
    bool DetectUARTProtocol();
    std::unique_ptr<GNSSInterface> CreateParser(GPSSettings::UARTProtocol protocol);
    
    bool ProcessUARTData();
    bool ProcessNetworkData();
    bool ProcessMAVLinkData();
    
    void UpdateStatistics();
    bool CheckForFailover();
    
    // Rate limiting
    bool ShouldUpdate() const;
    uint32_t GetTimeMs() const;
    
    // Member variables
    GPSSettings settings_;
    GPSSettings::GPSSource current_source_ = GPSSettings::kGPSSourceNone;
    GPSSettings::GPSSource backup_source_ = GPSSettings::kGPSSourceNone;
    
    std::unique_ptr<GNSSInterface> parser_;
    GNSSInterface::Position current_position_;
    
    // UART handling
    GPSSettings::UARTProtocol detected_protocol_ = GPSSettings::kUARTProtoAuto;
    uint8_t uart_buffer_[256];
    size_t uart_buffer_pos_ = 0;
    
    // Network handling
    uint8_t network_buffer_[1024];
    size_t network_buffer_pos_ = 0;
    
    // Rate limiting
    uint8_t update_rate_hz_ = 1;
    uint32_t last_update_ms_ = 0;
    uint32_t update_interval_ms_ = 1000;  // 1Hz default
    
    // Auto-detection
    bool auto_detect_active_ = false;
    uint32_t auto_detect_start_ms_ = 0;
    static constexpr uint32_t kAutoDetectTimeoutMs = 5000;
    
    // PPP management
    bool ppp_enabled_ = false;
    GNSSInterface::PPPService active_ppp_service_ = GNSSInterface::kPPPNone;
    uint32_t ppp_start_time_ms_ = 0;
    
    // Receiver capabilities
    bool supports_high_precision_ = false;
    bool supports_rtk_ = false;
    bool supports_ppp_ = false;
    
    // Failover
    uint32_t last_valid_position_ms_ = 0;
    static constexpr uint32_t kFailoverTimeoutMs = 10000;  // 10 seconds
    
    // Statistics
    Statistics stats_;
    uint32_t start_time_ms_ = 0;
};

#endif // GNSS_MANAGER_HH_