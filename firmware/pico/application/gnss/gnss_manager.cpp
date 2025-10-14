#include "gnss_manager.hh"
#include <cstring>
#include <algorithm>
#include "mavlink_gps_parser.hh"
#include "ubx_parser.hh"
#include "sbf_parser.hh"

#ifdef ON_EMBEDDED_DEVICE
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "bsp.hh"  // For UART definitions
#include "hardware/gpio.h"  // For gpio_set_function
#include "uart_switch.hh"  // For UART0 switching between ESP32 and GNSS
extern BSP bsp;  // External BSP instance
#define GET_TIME_MS() to_ms_since_boot(get_absolute_time())
#else
#include <chrono>
#define GET_TIME_MS() std::chrono::duration_cast<std::chrono::milliseconds>(\
    std::chrono::steady_clock::now().time_since_epoch()).count()
#endif

// For logging
#ifdef ON_EMBEDDED_DEVICE
#include "comms.hh"
extern CommsManager comms_manager;
#define LOG_INFO(...) comms_manager.console_level_printf(SettingsManager::LogLevel::kInfo, __VA_ARGS__)
#define LOG_WARNING(...) comms_manager.console_level_printf(SettingsManager::LogLevel::kWarnings, __VA_ARGS__)
#define LOG_ERROR(...) comms_manager.console_level_printf(SettingsManager::LogLevel::kErrors, __VA_ARGS__)
#else
#include <cstdio>
#define LOG_INFO(...) printf(__VA_ARGS())
#define LOG_WARNING(...) printf(__VA_ARGS())
#define LOG_ERROR(...) fprintf(stderr, __VA_ARGS__)
#endif

#ifdef ON_EMBEDDED_DEVICE
// RAII wrapper for Pico SDK critical section
class CriticalSectionLock {
public:
    explicit CriticalSectionLock(critical_section_t& cs) : cs_(cs) {
        critical_section_enter_blocking(&cs_);
    }
    ~CriticalSectionLock() {
        critical_section_exit(&cs_);
    }
    // Non-copyable
    CriticalSectionLock(const CriticalSectionLock&) = delete;
    CriticalSectionLock& operator=(const CriticalSectionLock&) = delete;
private:
    critical_section_t& cs_;
};
#define LOCK_GUARD() CriticalSectionLock lock(critical_section_)
#else
// No-op for non-embedded builds
#define LOCK_GUARD() (void)0
#endif

GNSSManager::GNSSManager() {
    start_time_ms_ = GetTimeMs();
#ifdef ON_EMBEDDED_DEVICE
    critical_section_init(&critical_section_);
#endif
}

GNSSManager::~GNSSManager() {
#ifdef ON_EMBEDDED_DEVICE
    critical_section_deinit(&critical_section_);
#endif
    // Cleanup handled by smart pointers
}

bool GNSSManager::Initialize(const GPSSettings& settings) {
    LOCK_GUARD();

    settings_ = settings;
    stats_ = Statistics(); // Reset stats

    // Set update rate (internal call, already locked)
    update_rate_hz_ = settings.gps_update_rate_hz;
    update_interval_ms_ = 1000 / settings.gps_update_rate_hz;
    
    // Configure failover timeout
    if (settings.failover_timeout_ms >= 5000 && settings.failover_timeout_ms <= 30000) {
        failover_timeout_ms_ = settings.failover_timeout_ms;
    } else {
        failover_timeout_ms_ = 10000;  // Default to 10 seconds if out of range
    }
    
    // Initialize based on configured source
    bool success = false;
    
    switch (settings.gps_source) {
        case GPSSettings::kGPSSourceUART:
            success = InitializeUART();
            break;
            
        case GPSSettings::kGPSSourceNetwork:
            success = InitializeNetworkSource();
            break;
            
        case GPSSettings::kGPSSourceMAVLink:
            success = InitializeMAVLinkSource();
            break;
            
        case GPSSettings::kGPSSourceAuto:
            // Try UART first, then network
            success = InitializeUART();
            if (!success) {
                success = InitializeNetworkSource();
            }
            break;
            
        default:
            LOG_INFO("GPS source set to NONE\n");
            return true;  // Not an error to have GPS disabled
    }
    
    if (success) {
        current_source_ = settings.gps_source;
        
        // Enable PPP if configured
        if (settings.ppp_auto_enable && parser_) {
            EnablePPP(settings.ppp_service);
        }
    }
    
    return success;
}

bool GNSSManager::AutodetectBaudRate(uint32_t& detected_baud) {
    // Try common u-blox baud rates in order of likelihood
    const uint32_t baud_rates[] = {38400, 9600, 115200, 230400, 460800};
    const size_t num_rates = sizeof(baud_rates) / sizeof(baud_rates[0]);

#ifdef ON_EMBEDDED_DEVICE
    uart_inst_t* uart = uart0;

    for (size_t i = 0; i < num_rates; i++) {
        uint32_t baud = baud_rates[i];
        LOG_INFO("Trying baud rate %u...\n", baud);

        // Set baud rate
        uart_set_baudrate(uart, baud);

        // Small delay for UART to settle
        sleep_ms(100);

        // Flush any old data
        while (uart_is_readable(uart)) {
            uart_getc(uart);
        }

        // Send UBX-CFG-PRT poll message (0xB5 0x62 0x06 0x00 0x00 0x00)
        // This requests port configuration and should generate an ACK
        const uint8_t poll_msg[] = {
            0xB5, 0x62,  // Sync
            0x06, 0x00,  // Class/ID (CFG-PRT)
            0x00, 0x00,  // Length = 0 (poll)
            0x06, 0x00   // Checksum
        };

        // Send poll message
        for (size_t j = 0; j < sizeof(poll_msg); j++) {
            uart_putc_raw(uart, poll_msg[j]);
        }

        // Wait for response (up to 500ms)
        uint32_t start_time = to_ms_since_boot(get_absolute_time());
        bool ubx_response = false;
        uint8_t response_buffer[64];
        size_t response_len = 0;

        while ((to_ms_since_boot(get_absolute_time()) - start_time) < 500) {
            if (uart_is_readable(uart)) {
                uint8_t byte = uart_getc(uart);
                response_buffer[response_len++] = byte;

                // Check for UBX sync bytes (0xB5 0x62)
                if (response_len >= 2 &&
                    response_buffer[response_len-2] == 0xB5 &&
                    response_buffer[response_len-1] == 0x62) {
                    ubx_response = true;
                    LOG_INFO("Got UBX sync bytes at %u baud!\n", baud);
                    break;
                }

                if (response_len >= sizeof(response_buffer)) {
                    break;  // Buffer full
                }
            }
            sleep_ms(10);
        }

        if (ubx_response) {
            detected_baud = baud;
            LOG_INFO("Autodetect successful: %u baud\n", baud);
            return true;
        }
    }

    // No valid response at any baud rate
    LOG_ERROR("Autobaud failed, no UBX response at any rate\n");
    detected_baud = 38400;  // Default to most common
    return false;
#else
    // Simulation mode - just return default
    detected_baud = 38400;
    return true;
#endif
}

bool GNSSManager::InitializeUART() {
#ifdef ON_EMBEDDED_DEVICE
    // Initialize UART0 for GNSS using the UART switch
    // UART0 is shared between ESP32 programming and GNSS, so we must use the switch

    uint32_t baud_rate = settings_.gps_uart_baud;

    // If baud rate is 0 or if we want to autobaud, run autodetection
    if (baud_rate == 0 || (settings_.gps_uart_protocol == GPSSettings::kUARTProtoAuto)) {
        LOG_INFO("Running baud rate autodetection...\n");
        uint32_t detected_baud = 0;

        if (UARTSwitch::InitForGNSS(38400)) {  // Start with most common default
            if (AutodetectBaudRate(detected_baud)) {
                LOG_INFO("Autodetected baud rate: %u\n", detected_baud);
                baud_rate = detected_baud;
                settings_.gps_uart_baud = detected_baud;  // Update settings
            } else {
                LOG_ERROR("Baud rate autodetection failed, using %u\n", baud_rate);
                baud_rate = (baud_rate == 0) ? 38400 : baud_rate;
            }
        }
    }

    // Initialize at detected or configured baud rate
    if (!UARTSwitch::InitForGNSS(baud_rate)) {
        LOG_ERROR("Failed to initialize UART0 for GNSS\n");
        return false;
    }

    LOG_INFO("GNSS UART0 initialized at %u baud\n", baud_rate);
#else
    LOG_INFO("UART initialization (simulated)\n");
#endif
    
    // Create appropriate parser
    if (settings_.gps_uart_protocol == GPSSettings::kUARTProtoAuto) {
        // Start with NMEA, will auto-detect later
        parser_ = std::make_unique<NMEAParser>();
        auto_detect_active_ = true;
        auto_detect_start_ms_ = GetTimeMs();
    } else {
        parser_ = CreateParser(settings_.gps_uart_protocol);
    }
    
    if (!parser_) {
        LOG_ERROR("Failed to create GPS parser\n");
        return false;
    }
    
    // Configure parser
    GNSSInterface::Config config;
    config.update_rate_hz = settings_.gps_update_rate_hz;
    config.ppp_service = settings_.ppp_service;
    config.enable_rtk = settings_.rtk_enabled;
    config.enable_sbas = settings_.enable_sbas;
    config.min_satellites = settings_.min_satellites;
    config.static_hold_threshold_m = settings_.static_threshold_m;
    
    parser_->Configure(config);
    
    return true;
}

bool GNSSManager::InitializeNetworkSource() {
    // Network GPS will be handled by ESP32
    // This would set up communication with ESP32 for network GPS data
    LOG_INFO("Network GPS source initialization\n");
    
    // Create NMEA parser for network data
    parser_ = std::make_unique<NMEAParser>();
    
    if (!parser_) {
        return false;
    }
    
    return true;
}

bool GNSSManager::InitializeMAVLinkSource() {
    // MAVLink GPS extraction from serial autopilot connection
    LOG_INFO("MAVLink GPS source initialization\n");
    
    // Create MAVLink GPS parser
    parser_ = std::make_unique<MAVLinkGPSParser>();
    
    if (!parser_) {
        LOG_ERROR("Failed to create MAVLink GPS parser\n");
        return false;
    }
    
    // Configure for autopilot connection
    GNSSInterface::Config config;
    config.update_rate_hz = settings_.gps_update_rate_hz;
    parser_->Configure(config);
    
    LOG_INFO("MAVLink GPS parser initialized\n");
    return true;
}

std::unique_ptr<GNSSInterface> GNSSManager::CreateParser(GPSSettings::UARTProtocol protocol) {
    switch (protocol) {
        case GPSSettings::kUARTProtoNMEA:
            return std::make_unique<NMEAParser>();
            
        case GPSSettings::kUARTProtoUBX:
            LOG_INFO("Using UBX parser for u-blox receiver\n");
            return std::make_unique<UBXParser>();
            
        case GPSSettings::kUARTProtoSBF:
            LOG_INFO("Using SBF parser for Septentrio receiver\n");
            return std::make_unique<SBFParser>();
            
        default:
            return std::make_unique<NMEAParser>();
    }
}

bool GNSSManager::ProcessData(const uint8_t* buffer, size_t length) {
    LOCK_GUARD();

    if (!parser_ || !buffer || length == 0) {
        return false;
    }

    // Print raw hex data if raw output is enabled (INFO level)
    if (settings_.gps_raw_output && length > 0) {
        LOG_INFO("GNSS RAW [%zu bytes]: ", length);
        for (size_t i = 0; i < length; i++) {
            LOG_INFO("%02X ", buffer[i]);
            // Print newline every 16 bytes for readability
            if ((i + 1) % 16 == 0 && i < length - 1) {
                LOG_INFO("\n                       ");
            }
        }
        LOG_INFO("\n");

        // Also print ASCII interpretation for NMEA sentences
        if (buffer[0] == '$' || buffer[0] == '!') {
            LOG_INFO("GNSS ASCII: ");
            for (size_t i = 0; i < length; i++) {
                char c = buffer[i];
                if (c >= 32 && c <= 126) {
                    LOG_INFO("%c", c);
                } else if (c == '\r') {
                    LOG_INFO("<CR>");
                } else if (c == '\n') {
                    LOG_INFO("<LF>");
                } else {
                    LOG_INFO(".");
                }
            }
            LOG_INFO("\n");
        }
    }

    bool success = parser_->ParseData(buffer, length);

    // Print decoded message information if debug output is enabled (WARNING level)
    if (settings_.gps_debug_output && success) {
        GNSSInterface::Position pos = parser_->GetLastPosition();

        // Print decoded message info - always visible at WARNING level
        if (buffer[0] == '$' || buffer[0] == '!') {
            // Extract sentence ID from NMEA message (e.g., "$GNGGA" -> "GNGGA")
            char sentence_id[8] = {0};
            size_t id_len = 0;
            for (size_t i = 1; i < length && i < 7 && buffer[i] != ','; i++) {
                sentence_id[id_len++] = buffer[i];
            }
            sentence_id[id_len] = '\0';

            LOG_WARNING("GNSS: %s | Fix:%d Sats:%d Lat:%.6f Lon:%.6f Alt:%.1fm HDOP:%.2f\n",
                       sentence_id,
                       pos.fix_type,
                       pos.satellites_used,
                       pos.latitude_deg,
                       pos.longitude_deg,
                       pos.altitude_msl_m,
                       pos.hdop);
        } else {
            // Binary protocol (UBX/SBF)
            LOG_WARNING("GNSS: Binary | Fix:%d Sats:%d Lat:%.6f Lon:%.6f Alt:%.1fm Acc:%.1fm\n",
                       pos.fix_type,
                       pos.satellites_used,
                       pos.latitude_deg,
                       pos.longitude_deg,
                       pos.altitude_msl_m,
                       pos.GetAccuracyM());
        }
    }

    if (success) {
        stats_.messages_processed++;

        // Check if we have a new valid position
        GNSSInterface::Position pos = parser_->GetLastPosition();
        if (pos.valid && pos.timestamp_ms != current_position_.timestamp_ms) {
            current_position_ = pos;
            stats_.position_updates++;
            last_valid_position_ms_ = GetTimeMs();

            // Track best accuracy
            float accuracy = pos.GetAccuracyM();
            if (accuracy < stats_.best_accuracy_m) {
                stats_.best_accuracy_m = accuracy;
            }
        }
    } else {
        stats_.parse_errors++;
    }

    // Auto-detect protocol if needed
    if (auto_detect_active_) {
        DetectUARTProtocol();
    }

    return success;
}

bool GNSSManager::UpdatePosition() {
    LOCK_GUARD();

    if (!parser_) {
        return false;
    }

    // Check rate limiting
    if (!ShouldUpdate()) {
        return false;
    }
    
    bool updated = false;
    
    switch (current_source_) {
        case GPSSettings::kGPSSourceUART:
            updated = ProcessUARTData();
            break;
            
        case GPSSettings::kGPSSourceNetwork:
            updated = ProcessNetworkData();
            break;
            
        case GPSSettings::kGPSSourceMAVLink:
            updated = ProcessMAVLinkData();
            break;
            
        default:
            break;
    }
    
    last_update_ms_ = GetTimeMs();
    
    // Check for failover if configured
    if (!updated || !current_position_.valid) {
        CheckForFailover();
    }
    
    UpdateStatistics();
    
    return updated;
}

bool GNSSManager::ProcessUARTData() {
#ifdef ON_EMBEDDED_DEVICE
    uart_inst_t* uart = uart0;  // GNSS uses UART0 (shared with ESP32)

    // Read available data from UART
    uart_buffer_pos_ = 0;
    while (uart_is_readable(uart) && uart_buffer_pos_ < sizeof(uart_buffer_)) {
        uart_buffer_[uart_buffer_pos_++] = uart_getc(uart);
    }

    if (uart_buffer_pos_ > 0) {
        return ProcessData(uart_buffer_, uart_buffer_pos_);
    }
#else
    // Simulation mode - no UART data
#endif
    return false;
}

bool GNSSManager::ProcessNetworkData() {
    // Network GPS data comes from ESP32 via SPI
    // The ESP32 queues GPS messages and we poll for them
    
    // Check if we have network GPS messages waiting
    // This would be implemented by checking the ESP32 object dictionary
    // for GPS network messages
    
    // For now, we rely on ProcessNetworkGPSMessage being called
    // from main.cc when ESP32 has data
    
    // Return true if we've received recent network GPS data
    uint32_t current_time = GetTimeMs();
    if (current_time - last_network_gps_ms_ < 2000) {  // Within 2 seconds
        return true;  // We have recent network GPS data
    }
    
    return false;
}

bool GNSSManager::ProcessNetworkGPSMessage(uint8_t type, const uint8_t* buffer, size_t length) {
    LOCK_GUARD();

    if (!buffer || length == 0) {
        return false;
    }

    // Update timestamp for network GPS activity
    last_network_gps_ms_ = GetTimeMs();
    
    // Ensure we're in network mode or auto mode
    if (current_source_ != GPSSettings::kGPSSourceNetwork && 
        current_source_ != GPSSettings::kGPSSourceAuto) {
        // If in auto mode, switch to network
        if (settings_.gps_source == GPSSettings::kGPSSourceAuto) {
            current_source_ = GPSSettings::kGPSSourceNetwork;
            LOG_INFO("Auto-switched to Network GPS source\n");
        } else {
            return false;  // Not configured for network GPS
        }
    }
    
    // Handle different message types
    switch (type) {
        case 0: // NMEA
            if (!parser_ || current_source_ != GPSSettings::kGPSSourceNetwork) {
                parser_ = std::make_unique<NMEAParser>();
                current_source_ = GPSSettings::kGPSSourceNetwork;
            }
            stats_.messages_processed++;
            return ProcessData(buffer, length);
            
        case 1: // MAVLink GPS
            if (!parser_ || current_source_ != GPSSettings::kGPSSourceNetwork) {
                parser_ = std::make_unique<MAVLinkGPSParser>();
                current_source_ = GPSSettings::kGPSSourceNetwork;
            }
            stats_.messages_processed++;
            return ProcessData(buffer, length);
            
        case 2: // UBX binary
            // For now, treat UBX as raw binary data that NMEA parser will ignore
            LOG_INFO("Received UBX GPS message (parsing as NMEA for now)\n");
            if (!parser_) {
                parser_ = std::make_unique<NMEAParser>();
            }
            return ProcessData(buffer, length);
            
        default:
            LOG_ERROR("Unknown GPS message type: %d\n", type);
            stats_.parse_errors++;
            return false;
    }
}

bool GNSSManager::ProcessMAVLinkData() {
#ifdef ON_EMBEDDED_DEVICE
    // MAVLink data comes from serial port configured for MAVLink
    // This could be either the GNSS UART or CommsUART depending on configuration

    // For MAVLink over GNSS UART (when autopilot is connected there)
    if (settings_.gps_source == GPSSettings::kGPSSourceMAVLink) {
        uart_inst_t* uart = uart0;  // GNSS uses UART0 (shared with ESP32)

        // Read available data from UART
        size_t available = 0;
        while (uart_is_readable(uart) && available < sizeof(uart_buffer_)) {
            uart_buffer_[available++] = uart_getc(uart);
        }

        if (available > 0 && parser_) {
            return parser_->ParseData(uart_buffer_, available);
        }
    }
#else
    // Simulation mode - no MAVLink data
#endif
    return false;
}

bool GNSSManager::DetectUARTProtocol() {
    // Simple auto-detection based on received data patterns
    // Priority: Check binary protocols first (UBX, SBF) before defaulting to NMEA

    if (!auto_detect_active_) {
        return false;
    }

    // Check for timeout
    uint32_t now = GetTimeMs();
    if (now - auto_detect_start_ms_ > kAutoDetectTimeoutMs) {
        LOG_INFO("Auto-detect timeout, defaulting to NMEA\n");
        auto_detect_active_ = false;
        detected_protocol_ = GPSSettings::kUARTProtoNMEA;
        return true;
    }

    // **PRIORITY 1: Check for UBX protocol FIRST (u-blox binary)**
    // UBX sync bytes: 0xB5 0x62
    // This is the most reliable detection since these bytes are unlikely in NMEA
    for (size_t i = 0; i < uart_buffer_pos_ - 1; i++) {
        if (uart_buffer_[i] == 0xB5 && uart_buffer_[i + 1] == 0x62) {
            LOG_INFO("Detected UBX protocol (u-blox binary) at byte %zu\n", i);

            // Switch to UBX parser
            parser_ = std::make_unique<UBXParser>();

            // Configure parser
            GNSSInterface::Config config;
            config.update_rate_hz = settings_.gps_update_rate_hz;
            config.enable_sbas = settings_.enable_sbas;
            config.ppp_service = settings_.ppp_service;
            config.enable_rtk = settings_.rtk_enabled;
            config.min_satellites = settings_.min_satellites;
            config.static_hold_threshold_m = settings_.static_threshold_m;
            parser_->Configure(config);

            detected_protocol_ = GPSSettings::kUARTProtoUBX;
            auto_detect_active_ = false;

            // Reset stats since we're switching parsers
            stats_.parse_errors = 0;
            stats_.messages_processed = 0;

            // Process the buffered data with new parser
            parser_->ParseData(uart_buffer_, uart_buffer_pos_);
            return true;
        }
    }
    
    // **PRIORITY 2: Check for SBF protocol (Septentrio binary)**
    // SBF sync bytes: '$@' (0x24 0x40)
    for (size_t i = 0; i < uart_buffer_pos_ - 1; i++) {
        if (uart_buffer_[i] == 0x24 && uart_buffer_[i + 1] == 0x40) {
            LOG_INFO("Detected SBF protocol (Septentrio binary) at byte %zu\n", i);

            // Switch to SBF parser
            parser_ = std::make_unique<SBFParser>();

            // Configure parser
            GNSSInterface::Config config;
            config.update_rate_hz = settings_.gps_update_rate_hz;
            config.enable_sbas = settings_.enable_sbas;
            config.ppp_service = settings_.ppp_service;
            config.enable_rtk = settings_.rtk_enabled;
            config.min_satellites = settings_.min_satellites;
            config.static_hold_threshold_m = settings_.static_threshold_m;
            parser_->Configure(config);

            detected_protocol_ = GPSSettings::kUARTProtoSBF;
            auto_detect_active_ = false;

            // Reset stats since we're switching parsers
            stats_.parse_errors = 0;
            stats_.messages_processed = 0;

            // Process the buffered data with new parser
            parser_->ParseData(uart_buffer_, uart_buffer_pos_);
            return true;
        }
    }

    // **PRIORITY 3: Check NMEA quality - high error rate means wrong parser**
    // If we've processed enough data and error rate is very high, keep looking for binary
    if (stats_.messages_processed + stats_.parse_errors > 10) {
        float error_rate = static_cast<float>(stats_.parse_errors) /
                          (stats_.messages_processed + stats_.parse_errors);

        // If error rate > 80%, likely receiving binary data (UBX/SBF)
        // Keep auto-detect active and keep searching for sync bytes
        if (error_rate > 0.8f) {
            LOG_INFO("High NMEA error rate (%.1f%%), continuing auto-detection\n", error_rate * 100);
            // Don't disable auto-detect yet - keep looking for binary protocols
            return false;
        }
    }

    // **PRIORITY 4: Accept NMEA if we have valid position with low error rate**
    if (parser_ && parser_->GetLastPosition().valid) {
        // Check error rate before accepting NMEA
        if (stats_.messages_processed + stats_.parse_errors > 10) {
            float error_rate = static_cast<float>(stats_.parse_errors) /
                              (stats_.messages_processed + stats_.parse_errors);

            // Only accept NMEA if error rate is reasonable (< 30%)
            if (error_rate < 0.3f) {
                LOG_INFO("Detected NMEA protocol (error rate: %.1f%%)\n", error_rate * 100);
                detected_protocol_ = GPSSettings::kUARTProtoNMEA;
                auto_detect_active_ = false;
                return true;
            } else {
                LOG_INFO("NMEA error rate too high (%.1f%%), waiting for binary protocol\n", error_rate * 100);
            }
        }
    }

    // **PRIORITY 5: Check for RTCM messages (for RTK base stations)**
    // RTCM3 preamble: 0xD3
    if (uart_buffer_pos_ > 0 && uart_buffer_[0] == 0xD3) {
        LOG_INFO("Detected RTCM protocol (RTK corrections)\n");
        detected_protocol_ = GPSSettings::kUARTProtoRTCM;
        auto_detect_active_ = false;

        // RTCM is usually input, not output
        // This might be a base station or correction service
        return false;  // Don't try to parse as position
    }

    return false;
}

GNSSInterface::Position GNSSManager::GetCurrentPosition() const {
    LOCK_GUARD();
    return current_position_;
}

bool GNSSManager::IsPositionValid() const {
    LOCK_GUARD();

    if (!current_position_.valid) {
        return false;
    }

    // Check for stale position
    uint32_t age_ms = GetTimeMs() - current_position_.timestamp_ms;
    if (age_ms > 5000) {  // 5 second timeout
        return false;
    }

    return true;
}

bool GNSSManager::SetSource(GPSSettings::GPSSource source) {
    LOCK_GUARD();

    if (source == current_source_) {
        return true;
    }

    // Save current as backup
    if (current_source_ != GPSSettings::kGPSSourceNone) {
        backup_source_ = current_source_;
    }
    
    // Switch to new source
    bool success = false;
    
    switch (source) {
        case GPSSettings::kGPSSourceUART:
            success = InitializeUART();
            break;
            
        case GPSSettings::kGPSSourceNetwork:
            success = InitializeNetworkSource();
            break;
            
        case GPSSettings::kGPSSourceMAVLink:
            success = InitializeMAVLinkSource();
            break;
            
        default:
            current_source_ = GPSSettings::kGPSSourceNone;
            return true;
    }
    
    if (success) {
        current_source_ = source;
        stats_.source_switches++;
    }
    
    return success;
}

bool GNSSManager::EnablePPP(GNSSInterface::PPPService service) {
    LOCK_GUARD();

    if (!parser_) {
        return false;
    }

    // Check if receiver supports this service
    if (service != GNSSInterface::kPPPAuto && !parser_->SupportsPPPService(service)) {
        LOG_ERROR("Receiver does not support PPP service %d\n", service);
        return false;
    }
    
    // Auto-select best available service
    if (service == GNSSInterface::kPPPAuto) {
        // Try services in order of preference
        const GNSSInterface::PPPService services[] = {
            GNSSInterface::kPPPGalileoHAS,    // Free 20cm
            GNSSInterface::kPPPIGSRTS,         // Free 10-20cm
            GNSSInterface::kPPPPointPerfect,   // Paid but fast convergence
            GNSSInterface::kPPPCenterPointRTX, // Paid
            GNSSInterface::kPPPSBAS           // Fallback
        };
        
        for (auto svc : services) {
            if (parser_->SupportsPPPService(svc)) {
                service = svc;
                LOG_INFO("Auto-selected PPP service: %d\n", service);
                break;
            }
        }
    }
    
    // Enable the service
    const char* key = (settings_.ppp_key[0] != '\0') ? settings_.ppp_key : nullptr;
    
    if (parser_->EnablePPP(service, key)) {
        ppp_enabled_ = true;
        active_ppp_service_ = service;
        ppp_start_time_ms_ = GetTimeMs();
        LOG_INFO("PPP enabled with service %d\n", service);
        return true;
    }
    
    return false;
}

GNSSInterface::PPPService GNSSManager::GetPPPStatus(float& convergence_percent, uint32_t& eta_seconds) const {
    LOCK_GUARD();

    if (!parser_ || !ppp_enabled_) {
        convergence_percent = 0;
        eta_seconds = 0;
        return GNSSInterface::kPPPNone;
    }

    return parser_->GetPPPStatus(convergence_percent, eta_seconds);
}

bool GNSSManager::SetUpdateRate(uint8_t hz) {
    LOCK_GUARD();

    if (hz < 1 || hz > 10) {
        LOG_ERROR("Invalid update rate %d Hz (must be 1-10)\n", hz);
        return false;
    }

    update_rate_hz_ = hz;
    update_interval_ms_ = 1000 / hz;

    // Update parser configuration if it exists
    if (parser_) {
        GNSSInterface::Config config;
        config.update_rate_hz = hz;
        parser_->Configure(config);
    }

    LOG_INFO("GPS update rate set to %d Hz\n", hz);
    return true;
}

bool GNSSManager::ShouldUpdate() const {
    uint32_t now = GetTimeMs();
    return (now - last_update_ms_) >= update_interval_ms_;
}

uint32_t GNSSManager::GetTimeMs() const {
    return GET_TIME_MS();
}

bool GNSSManager::CheckForFailover() {
    // Don't failover if we have no backup or we're in manual mode
    if (settings_.gps_source != GPSSettings::kGPSSourceAuto) {
        return false;
    }
    
    uint32_t now = GetTimeMs();
    uint32_t time_since_valid = now - last_valid_position_ms_;
    
    // Check if current source has failed
    bool should_failover = false;
    
    switch (current_source_) {
        case GPSSettings::kGPSSourceUART:
            // UART failed if no valid position for timeout period
            should_failover = (time_since_valid > failover_timeout_ms_);
            break;
            
        case GPSSettings::kGPSSourceNetwork:
            // Network failed if no data received recently
            should_failover = (now - last_network_gps_ms_ > failover_timeout_ms_);
            break;
            
        case GPSSettings::kGPSSourceMAVLink:
            // MAVLink failed if no messages recently
            should_failover = (time_since_valid > failover_timeout_ms_);
            break;
            
        default:
            break;
    }
    
    if (!should_failover) {
        return false;
    }
    
    LOG_INFO("GPS source %s failed, attempting failover\n", 
             settings_.GetSourceString());
    
    // Try next source in priority order
    const GPSSettings::GPSSource priority_order[] = {
        GPSSettings::kGPSSourceUART,     // Highest priority
        GPSSettings::kGPSSourceNetwork,  // Second priority
        GPSSettings::kGPSSourceMAVLink   // Lowest priority
    };
    
    for (auto source : priority_order) {
        // Skip current failed source
        if (source == current_source_) {
            continue;
        }
        
        LOG_INFO("Trying GPS source: %s\n", GPSSettings::kGPSSourceStrs[source]);
        
        if (SetSource(source)) {
            LOG_INFO("Failover successful to %s\n", GPSSettings::kGPSSourceStrs[source]);
            stats_.failover_count++;
            
            // Remember failed source
            if (current_source_ != GPSSettings::kGPSSourceNone) {
                backup_source_ = current_source_;
            }
            return true;
        }
    }
    
    LOG_ERROR("All GPS sources failed, no position available\n");
    current_source_ = GPSSettings::kGPSSourceNone;
    return false;
}

void GNSSManager::UpdateStatistics() {
    uint32_t now = GetTimeMs();
    stats_.uptime_s = (now - start_time_ms_) / 1000;
    
    if (ppp_enabled_ && current_position_.ppp_status == 2) {  // Converged
        if (stats_.ppp_convergence_time_s == 0) {
            stats_.ppp_convergence_time_s = (now - ppp_start_time_ms_) / 1000;
        }
    }
}

const char* GNSSManager::GetReceiverType() const {
    LOCK_GUARD();

    if (!parser_) {
        return "NONE";
    }

    const char* type = parser_->GetReceiverType();

    // Add high precision indicator if detected
    static char type_buffer[32];
    if (supports_high_precision_) {
        snprintf(type_buffer, sizeof(type_buffer), "%s-HP", type);
        return type_buffer;
    }

    return type;
}

size_t GNSSManager::GetDiagnostics(char* buffer, size_t max_len) const {
    LOCK_GUARD();

    if (!buffer || max_len == 0) return 0;

    int written = snprintf(buffer, max_len,
        "GNSS Manager Diagnostics:\n"
        "  Source: %s\n"
        "  Receiver: %s\n"
        "  Update rate: %d Hz\n"
        "  Position: %s\n"
        "    Lat: %.6f°\n"
        "    Lon: %.6f°\n"
        "    Alt: %.1f m\n"
        "    Fix: %d\n"
        "    Sats: %d\n"
        "    HDOP: %.2f\n"
        "    Accuracy: %.2f m\n"
        "  PPP: %s (Service: %d)\n"
        "  Statistics:\n"
        "    Messages: %u\n"
        "    Updates: %u\n"
        "    Errors: %u\n"
        "    Uptime: %u s\n"
        "    Best accuracy: %.2f m\n",
        settings_.GetSourceString(),
        GetReceiverType(),
        update_rate_hz_,
        current_position_.valid ? "Valid" : "Invalid",
        current_position_.latitude_deg,
        current_position_.longitude_deg,
        current_position_.altitude_m,
        current_position_.fix_type,
        current_position_.satellites_used,
        current_position_.hdop,
        current_position_.GetAccuracyM(),
        ppp_enabled_ ? "Enabled" : "Disabled",
        active_ppp_service_,
        stats_.messages_processed,
        stats_.position_updates,
        stats_.parse_errors,
        stats_.uptime_s,
        stats_.best_accuracy_m
    );
    
    // Add parser-specific diagnostics if available
    if (parser_ && written > 0 && written < static_cast<int>(max_len - 1)) {
        size_t remaining = max_len - written;
        written += parser_->GetDiagnostics(buffer + written, remaining);
    }
    
    return (written > 0 && written < static_cast<int>(max_len)) ? written : 0;
}