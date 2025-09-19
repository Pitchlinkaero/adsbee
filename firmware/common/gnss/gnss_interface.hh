#ifndef GNSS_INTERFACE_HH_
#define GNSS_INTERFACE_HH_

#include <cstdint>
#include <cstring>

/**
 * Base interface for all GNSS parsers and receivers.
 * Provides a unified interface for GPS position data regardless of source.
 * 
 * Supports PPP (Precise Point Positioning) as primary high-precision method.
 * RTK Not supported yet.
 */
class GNSSInterface {
public:
    // Fix types following our PPP-first approach
    enum FixType : uint8_t {
        kNoFix = 0,
        kDead = 1,           // Dead reckoning only
        k2DFix = 2,
        k3DFix = 3,
        kGNSSDGPS = 4,       // 3D + SBAS/DGPS
        kPPPConverging = 5,  // PPP solution converging
        kPPPConverged = 6,   // PPP solution converged
        kRTKFloat = 7,       // RTK float solution (advanced)
        kRTKFixed = 8,       // RTK fixed solution (advanced)
        kRTKFixedDGPS = 9    // RTK + DGPS (advanced)
    };

    // PPP service types
    enum PPPService : uint8_t {
        kPPPNone = 0,
        kPPPSBAS = 1,        // WAAS/EGNOS/MSAS (1-2m)
        kPPPGalileoHAS = 2,  // FREE 20cm accuracy
        kPPPIGSRTS = 3,      // FREE 10-20cm
        kPPPPointPerfect = 4,// u-blox 3-6cm ($$)
        kPPPCenterPointRTX = 5, // Trimble RTX 2-4cm ($$)
        kPPPTerraStar = 6,   // TerraStar 2.5cm ($$)
        kPPPBeiDouB2b = 7,   // BeiDou PPP-B2b (Asia)
        kPPPAuto = 255       // Auto-select best available
    };
    
    static constexpr const char* kPPPServiceStrs[] = {
        "NONE",
        "SBAS",
        "GALILEO_HAS",
        "IGS_RTS",
        "POINTPERFECT",
        "RTX",
        "TERRASTAR",
        "BEIDOU_B2B"
    };
    
    static constexpr int kPPPServiceCount = 8;

    // Position structure optimized for ADSBEE use case
    struct Position {
        // Core position data
        double latitude_deg = 0.0;
        double longitude_deg = 0.0;
        float altitude_m = 0.0;        // HAE (Height Above Ellipsoid)
        float altitude_msl_m = 0.0;    // MSL (Mean Sea Level)
        
        // Velocity (mostly for mobile applications)
        float ground_speed_mps = 0.0;
        float track_deg = 0.0;         // True track
        float vertical_velocity_mps = 0.0;
        
        // Fix quality
        FixType fix_type = kNoFix;
        uint8_t satellites_used = 0;
        float hdop = 99.99;
        float vdop = 99.99;
        float pdop = 99.99;
        
        // Accuracy estimates
        float accuracy_horizontal_m = 999.0;
        float accuracy_vertical_m = 999.0;
        float accuracy_speed_mps = 999.0;
        float accuracy_heading_deg = 999.0;
        
        // PPP-specific fields (primary for high precision)
        PPPService ppp_service = kPPPNone;
        uint8_t ppp_status = 0;        // 0=none, 1=converging, 2=converged
        float ppp_convergence_percent = 0.0;
        uint32_t ppp_convergence_time_s = 0;
        
        // RTK fields (optional/advanced users only)
        bool rtk_available = false;
        float rtk_baseline_m = 0.0;    // Distance to base station
        uint32_t rtk_correction_age_ms = 0;
        
        // Timing
        uint32_t timestamp_ms = 0;     // System timestamp
        uint32_t gps_time_week = 0;    // GPS week number
        uint32_t gps_time_ms = 0;      // GPS time of week in ms
        
        // Validity flag
        bool valid = false;
        
        // Helper functions
        bool HasFix() const { return fix_type >= k2DFix; }
        bool Has3DFix() const { return fix_type >= k3DFix; }
        bool HasHighPrecision() const { return fix_type >= kPPPConverged; }
        float GetAccuracyM() const { 
            return (accuracy_horizontal_m < accuracy_vertical_m) ? 
                   accuracy_horizontal_m : accuracy_vertical_m;
        }
    };

    // Configuration structure for GPS settings
    struct Config {
        uint8_t update_rate_hz = 1;    // 1Hz default per our optimization
        PPPService ppp_service = kPPPAuto;
        bool enable_rtk = false;       // RTK off by default
        bool enable_sbas = true;       // SBAS on for instant fix
        uint8_t min_satellites = 4;    // Minimum satellites for fix
        float static_hold_threshold_m = 2.0; // Movement threshold for static mode
    };

    // Pure virtual interface methods
    virtual ~GNSSInterface() = default;
    
    /**
     * Parse incoming data from the GPS receiver
     * @param buffer Raw data buffer
     * @param length Length of data in bytes
     * @return true if valid data was parsed
     */
    virtual bool ParseData(const uint8_t* buffer, size_t length) = 0;
    
    /**
     * Get the last valid position
     * @return Position structure with latest GPS data
     */
    virtual Position GetLastPosition() const = 0;
    
    /**
     * Configure the GPS receiver
     * @param config Configuration parameters
     * @return true if configuration successful
     */
    virtual bool Configure(const Config& config) = 0;
    
    /**
     * Get receiver type string for identification
     * @return String identifying the receiver type (e.g., "NMEA", "UBX-F9P", "SBF")
     */
    virtual const char* GetReceiverType() const = 0;
    
    /**
     * Check if receiver supports a specific PPP service
     * @param service PPP service to check
     * @return true if supported
     */
    virtual bool SupportsPPPService(PPPService service) const = 0;
    
    /**
     * Get current PPP convergence status
     * @param convergence_percent Output: convergence percentage (0-100)
     * @param eta_seconds Output: estimated time to full convergence
     * @return Current PPP service being used
     */
    virtual PPPService GetPPPStatus(float& convergence_percent, uint32_t& eta_seconds) const {
        convergence_percent = 0;
        eta_seconds = 0;
        return kPPPNone;
    }
    
    /**
     * Enable/configure PPP service
     * @param service PPP service to enable
     * @param key Service key/credentials if needed (nullptr for free services)
     * @return true if PPP enabled successfully
     */
    virtual bool EnablePPP(PPPService service, const char* key = nullptr) {
        (void)service;
        (void)key;
        return false; // Default: no PPP support
    }
    
    /**
     * Reset the parser state (useful for error recovery)
     */
    virtual void Reset() = 0;
    
    /**
     * Get diagnostic information
     * @param buffer Output buffer for diagnostic string
     * @param max_len Maximum buffer length
     * @return Number of bytes written
     */
    virtual size_t GetDiagnostics(char* buffer, size_t max_len) const {
        if (buffer && max_len > 0) {
            strncpy(buffer, "No diagnostics available", max_len - 1);
            buffer[max_len - 1] = '\0';
            return strlen(buffer);
        }
        return 0;
    }
};

#endif // GNSS_INTERFACE_HH_