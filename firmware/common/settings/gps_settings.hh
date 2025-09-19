#ifndef GPS_SETTINGS_HH_
#define GPS_SETTINGS_HH_

#include <cstdint>
#include "gnss_interface.hh"

/**
 * GPS/GNSS settings structure to be integrated into main Settings struct.
 * Following the PPP-first approach from our specification.
 */
struct GPSSettings {
    // Constants
    static constexpr uint16_t kPPPKeyMaxLen = 64;      // For PointPerfect/RTX keys
    static constexpr uint16_t kNTRIPHostMaxLen = 32;   // NTRIP hostname
    static constexpr uint16_t kNTRIPMountMaxLen = 32;  // NTRIP mountpoint
    static constexpr uint16_t kNTRIPUserMaxLen = 32;   // NTRIP username
    static constexpr uint16_t kNTRIPPassMaxLen = 32;   // NTRIP password
    
    // GPS source configuration
    enum GPSSource : uint8_t {
        kGPSSourceNone = 0,
        kGPSSourceUART = 1,        // Physical GPS on GNSS UART
        kGPSSourceUDPNMEA = 2,     // Network NMEA over UDP
        kGPSSourceMAVLink = 3,     // Extract from MAVLink stream
        kGPSSourceAuto = 255       // Auto-detect best source
    };
    
    // UART protocol types
    enum UARTProtocol : uint8_t {
        kUARTProtoAuto = 0,        // Auto-detect protocol
        kUARTProtoNMEA = 1,        // Standard NMEA 0183
        kUARTProtoUBX = 2,         // u-blox binary
        kUARTProtoSBF = 3,         // Septentrio binary
        kUARTProtoRTCM = 4         // RTCM corrections (input)
    };
    
    // Main GPS settings
    GPSSource gps_source = kGPSSourceUART;           // Default to physical GPS
    UARTProtocol gps_uart_protocol = kUARTProtoAuto; // Auto-detect by default
    uint32_t gps_uart_baud = 9600;                   // Conservative default
    uint8_t gps_update_rate_hz = 1;                  // 1Hz default (optimal)
    bool gps_output_enabled = true;                  // Include in GDL90 output
    
    // PPP settings (PRIMARY high-precision method)
    GNSSInterface::PPPService ppp_service = GNSSInterface::kPPPAuto;  // Auto-select best
    bool ppp_auto_enable = true;                     // Auto-start PPP on boot
    char ppp_key[kPPPKeyMaxLen + 1] = {0};          // Service key if needed
    uint8_t ppp_convergence_threshold_cm = 20;       // Consider converged at 20cm
    
    // RTK settings (ADVANCED - rarely used)
    bool rtk_enabled = false;                        // RTK off by default
    char ntrip_host[kNTRIPHostMaxLen + 1] = {0};
    uint16_t ntrip_port = 2101;                      // Standard NTRIP port
    char ntrip_mountpoint[kNTRIPMountMaxLen + 1] = {0};
    char ntrip_username[kNTRIPUserMaxLen + 1] = {0};
    char ntrip_password[kNTRIPPassMaxLen + 1] = {0};
    
    // Network GPS settings
    bool network_gps_enabled = false;                // Network GPS sources
    uint16_t network_nmea_port = 10110;             // UDP port for NMEA
    uint16_t mavlink_gps_port = 14550;              // MAVLink GPS port
    
    // Position configuration
    bool static_mode = true;                         // Assume stationary ground station
    float static_threshold_m = 2.0f;                 // Movement threshold
    bool enable_sbas = true;                         // WAAS/EGNOS/MSAS for quick fix
    uint8_t min_satellites = 4;                      // Minimum for position
    
    // MAVLink relay settings (for drone integration)
    bool mavlink_relay_enabled = false;              // Relay MAVLink messages
    bool mavlink_inject_adsb = true;                 // Inject ADS-B into MAVLink
    bool mavlink_extract_gps = true;                 // Extract GPS from MAVLink
    uint16_t mavlink_relay_port = 14550;            // Standard MAVLink port
    uint8_t mavlink_relay_rate_hz = 10;             // Relay rate limit
    
    // Diagnostics/debug
    bool gps_debug_output = false;                   // Extra debug messages
    bool gps_raw_output = false;                     // Output raw NMEA/UBX
    
    /**
     * Default constructor - sets sensible defaults
     */
    GPSSettings() {
        // Most defaults set above with member initializers
        // Any additional initialization here
    }
    
    /**
     * Check if high precision is configured
     */
    bool IsHighPrecisionEnabled() const {
        return (ppp_service != GNSSInterface::kPPPNone && 
                ppp_service != GNSSInterface::kPPPSBAS) || 
                rtk_enabled;
    }
    
    /**
     * Get human-readable GPS source string
     */
    const char* GetSourceString() const {
        switch (gps_source) {
            case kGPSSourceNone: return "NONE";
            case kGPSSourceUART: return "UART";
            case kGPSSourceUDPNMEA: return "UDP_NMEA";
            case kGPSSourceMAVLink: return "MAVLINK";
            case kGPSSourceAuto: return "AUTO";
            default: return "UNKNOWN";
        }
    }
    
    /**
     * Get human-readable protocol string
     */
    const char* GetProtocolString() const {
        switch (gps_uart_protocol) {
            case kUARTProtoAuto: return "AUTO";
            case kUARTProtoNMEA: return "NMEA";
            case kUARTProtoUBX: return "UBX";
            case kUARTProtoSBF: return "SBF";
            case kUARTProtoRTCM: return "RTCM";
            default: return "UNKNOWN";
        }
    }
    
    /**
     * Get human-readable PPP service string
     */
    const char* GetPPPServiceString() const {
        switch (ppp_service) {
            case GNSSInterface::kPPPNone: return "NONE";
            case GNSSInterface::kPPPSBAS: return "SBAS";
            case GNSSInterface::kPPPGalileoHAS: return "GALILEO_HAS";
            case GNSSInterface::kPPPIGSRTS: return "IGS_RTS";
            case GNSSInterface::kPPPPointPerfect: return "POINTPERFECT";
            case GNSSInterface::kPPPCenterPointRTX: return "RTX";
            case GNSSInterface::kPPPTerraStar: return "TERRASTAR";
            case GNSSInterface::kPPPBeiDouB2b: return "BEIDOU_B2B";
            case GNSSInterface::kPPPAuto: return "AUTO";
            default: return "UNKNOWN";
        }
    }
};

#endif // GPS_SETTINGS_HH_