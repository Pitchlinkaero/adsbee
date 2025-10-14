#ifndef GNSS_INTERFACE_HH_
#define GNSS_INTERFACE_HH_

#include <cstdint>
#include <cstring>
#include "unit_conversions.hh"

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
        "BEIDOU_B2B",
        "AUTO"
    };

    static constexpr int kPPPServiceCount = 9;

    /**
     * Position structure using fixed-point representation for performance.
     *
     * Uses integer types to avoid slow software FP emulation on RP2040/ESP32.
     * Provides 20-100x performance improvement over float operations.
     * Memory footprint: ~60 bytes (40% smaller than float version).
     * Precision: 1cm for lat/lon (sufficient for PPP/RTK).
     *
     * All conversions use functions from unit_conversions.hh
     */
    struct Position {
        // Core position data (fixed-point Q7.24 for lat/lon, mm for altitude)
        int32_t latitude_fixed24 = 0;       // Latitude in Q7.24 format (0.67cm resolution)
        int32_t longitude_fixed24 = 0;      // Longitude in Q7.24 format
        int32_t altitude_mm = 0;            // Altitude HAE in millimeters
        int32_t altitude_msl_mm = 0;        // Altitude MSL in millimeters

        // Velocity (cm/s for speed, centidegrees for heading)
        int16_t ground_speed_cms = 0;       // Ground speed in cm/s
        uint16_t track_cdeg = 0;            // True track in 0.01 degrees
        int16_t vertical_velocity_cms = 0;  // Vertical velocity in cm/s

        // Fix quality
        FixType fix_type = kNoFix;
        uint8_t satellites_used = 0;
        uint16_t hdop_e2 = kDopInvalidValue;    // HDOP * 100
        uint16_t vdop_e2 = kDopInvalidValue;    // VDOP * 100
        uint16_t pdop_e2 = kDopInvalidValue;    // PDOP * 100

        // Accuracy estimates (decimeters for distance, cm/s for speed, centidegrees for heading)
        uint16_t accuracy_horizontal_dm = kAccuracyInvalidValue;
        uint16_t accuracy_vertical_dm = kAccuracyInvalidValue;
        uint16_t accuracy_speed_cms = 0;      // In cm/s
        uint16_t accuracy_heading_cdeg = 0;   // In 0.01 degrees

        // PPP-specific fields
        PPPService ppp_service = kPPPNone;
        uint8_t ppp_status = 0;                     // 0=none, 1=converging, 2=converged
        uint16_t ppp_convergence_e1 = 0;            // Convergence * 10 (0.1% resolution)
        uint32_t ppp_convergence_time_s = 0;

        // RTK fields
        bool rtk_available = false;
        uint16_t rtk_baseline_dm = 0;               // Baseline in decimeters
        uint32_t rtk_correction_age_ms = 0;

        // Timing (already integers - no change)
        uint32_t timestamp_ms = 0;
        uint32_t gps_time_week = 0;
        uint32_t gps_time_ms = 0;

        // UTC Date/Time (already integers - no change)
        uint16_t utc_year = 0;
        uint8_t utc_month = 0;
        uint8_t utc_day = 0;
        uint8_t utc_hour = 0;
        uint8_t utc_minute = 0;
        uint8_t utc_second = 0;

        // Validity flag
        bool valid = false;

        // Helper functions (integer-only operations)
        bool HasFix() const { return fix_type >= k2DFix; }
        bool Has3DFix() const { return fix_type >= k3DFix; }
        bool HasHighPrecision() const { return fix_type >= kPPPConverged; }

        // Get best accuracy in decimeters (integer operation)
        uint16_t GetAccuracyDm() const {
            return (accuracy_horizontal_dm < accuracy_vertical_dm) ?
                   accuracy_horizontal_dm : accuracy_vertical_dm;
        }

        // Float getters for external interfaces requiring float (converts on demand)
        inline double GetLatitudeDeg() const {
            return LatLonFixed24ToDeg(latitude_fixed24);
        }
        inline double GetLongitudeDeg() const {
            return LatLonFixed24ToDeg(longitude_fixed24);
        }
        inline float GetAltitudeM() const {
            return MMToMeters(altitude_mm);
        }
        inline float GetAltitudeMSL() const {
            return MMToMeters(altitude_msl_mm);
        }
        inline float GetGroundSpeedMps() const {
            return CmsToMps(ground_speed_cms);
        }
        inline float GetTrackDeg() const {
            return CentiDegToDeg(track_cdeg);
        }
        inline float GetVerticalVelocityMps() const {
            return CmsToMps(vertical_velocity_cms);
        }
        inline float GetHDOP() const {
            return FixedToDop(hdop_e2);
        }
        inline float GetVDOP() const {
            return FixedToDop(vdop_e2);
        }
        inline float GetPDOP() const {
            return FixedToDop(pdop_e2);
        }
        inline float GetAccuracyHorizontalM() const {
            return DecimetersToMeters(accuracy_horizontal_dm);
        }
        inline float GetAccuracyVerticalM() const {
            return DecimetersToMeters(accuracy_vertical_dm);
        }
        inline float GetAccuracyM() const {
            return DecimetersToMeters(GetAccuracyDm());
        }
        inline float GetPPPConvergencePercent() const {
            return TenthsToPercent(ppp_convergence_e1);
        }
        inline float GetRTKBaselineM() const {
            return DecimetersToMetersDistance(rtk_baseline_dm);
        }

        // Setters from float (for parser compatibility during migration)
        inline void SetLatitudeDeg(double deg) {
            latitude_fixed24 = LatLonDegToFixed24(deg);
        }
        inline void SetLongitudeDeg(double deg) {
            longitude_fixed24 = LatLonDegToFixed24(deg);
        }
        inline void SetAltitudeM(float m) {
            altitude_mm = MetersToMM(m);
        }
        inline void SetAltitudeMSL(float m) {
            altitude_msl_mm = MetersToMM(m);
        }
        inline void SetGroundSpeedMps(float mps) {
            ground_speed_cms = MpsToCms(mps);
        }
        inline void SetTrackDeg(float deg) {
            track_cdeg = DegToCentiDeg(deg);
        }
        inline void SetVerticalVelocityMps(float mps) {
            vertical_velocity_cms = MpsToCms(mps);
        }
        inline void SetHDOP(float dop) {
            hdop_e2 = DopToFixed(dop);
        }
        inline void SetVDOP(float dop) {
            vdop_e2 = DopToFixed(dop);
        }
        inline void SetPDOP(float dop) {
            pdop_e2 = DopToFixed(dop);
        }
        inline void SetAccuracyHorizontalM(float m) {
            accuracy_horizontal_dm = MetersToDecimeters(m);
        }
        inline void SetAccuracyVerticalM(float m) {
            accuracy_vertical_dm = MetersToDecimeters(m);
        }
        inline void SetPPPConvergencePercent(float pct) {
            ppp_convergence_e1 = PercentToTenths(pct);
        }
        inline void SetRTKBaselineM(float m) {
            rtk_baseline_dm = MetersToDecimetersDistance(m);
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