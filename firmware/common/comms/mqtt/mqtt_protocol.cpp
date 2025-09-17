#include "mqtt_protocol.hh"
#include <stdio.h>
#include <string.h>

// Main formatting functions
uint16_t MQTTProtocol::FormatPacket(const Decoded1090Packet& packet,
                                     uint8_t* buffer,
                                     uint16_t buffer_size,
                                     Format format,
                                     FrequencyBand band) {
    if (!buffer || buffer_size < 20) {
        return 0;
    }
    
    if (format == FORMAT_JSON) {
        return FormatPacketJSON(packet, (char*)buffer, buffer_size, band);
    } else {
        return FormatPacketBinary(packet, buffer, buffer_size, band);
    }
}

uint16_t MQTTProtocol::FormatAircraft(const Aircraft1090& aircraft,
                                       uint8_t* buffer,
                                       uint16_t buffer_size,
                                       Format format,
                                       FrequencyBand band) {
    if (!buffer || buffer_size < 20) {
        return 0;
    }
    
    if (format == FORMAT_JSON) {
        return FormatAircraftJSON(aircraft, (char*)buffer, buffer_size, band);
    } else {
        return FormatAircraftBinary(aircraft, buffer, buffer_size, band);
    }
}

bool MQTTProtocol::GetTopic(uint32_t icao24,
                            const char* msg_type,
                            char* topic_buf,
                            uint16_t topic_size,
                            FrequencyBand band,
                            bool use_short,
                            const char* device_id) {
    if (!topic_buf || !msg_type || topic_size < 16) {
        return false;
    }
    
    int written;
    const char* band_str = (band == BAND_978_MHZ) ? "uat" : "adsb";
    const char* band_short = (band == BAND_978_MHZ) ? "u" : "a";
    
    if (device_id && strlen(device_id) > 0) {
        // Include device ID in topic hierarchy
        if (use_short) {
            // Short topics for binary format
            const char* short_type = "x";
            if (strcmp(msg_type, "position") == 0) short_type = "p";
            else if (strcmp(msg_type, "status") == 0) short_type = "s";
            else if (strcmp(msg_type, "raw") == 0) short_type = "r";
            
            written = snprintf(topic_buf, topic_size, "%s/%s/%06X/%s", 
                             device_id, band_short, (unsigned int)icao24, short_type);
        } else {
            // Standard topics for JSON
            written = snprintf(topic_buf, topic_size, "%s/%s/%06X/%s", 
                             device_id, band_str, (unsigned int)icao24, msg_type);
        }
    } else {
        // No device ID - use original format for backward compatibility
        if (use_short) {
            const char* short_type = "x";
            if (strcmp(msg_type, "position") == 0) short_type = "p";
            else if (strcmp(msg_type, "status") == 0) short_type = "s";
            else if (strcmp(msg_type, "raw") == 0) short_type = "r";
            
            written = snprintf(topic_buf, topic_size, "%s/%06X/%s", 
                             band_short, (unsigned int)icao24, short_type);
        } else {
            written = snprintf(topic_buf, topic_size, "%s/%06X/%s", 
                             band_str, (unsigned int)icao24, msg_type);
        }
    }
    
    return (written > 0 && written < topic_size);
}

// JSON Implementation
uint16_t MQTTProtocol::FormatPacketJSON(const Decoded1090Packet& packet,
                                         char* buffer,
                                         uint16_t buffer_size,
                                         FrequencyBand band) {
    uint32_t icao24 = packet.GetICAOAddress();
    uint8_t df = packet.GetDownlinkFormat();
    const char* band_str = (band == BAND_978_MHZ) ? "UAT" : "1090";
    
    // Build JSON with essential fields including band
    int written = snprintf(buffer, buffer_size,
        "{\"t\":%llu,"           // timestamp (short key)
        "\"icao\":\"%06X\","
        "\"band\":\"%s\","       // Frequency band
        "\"df\":%d,"
        "\"rssi\":%d,"
        "\"hex\":\"",
        (unsigned long long)(packet.GetMLAT12MHzCounter() / 12000),  // Convert to ms
        (unsigned int)icao24,
        band_str,
        df,
        packet.GetRSSIdBm()
    );
    
    if (written < 0 || written >= buffer_size) {
        return 0;
    }
    
    // Add hex data
    char* ptr = buffer + written;
    int remaining = buffer_size - written;
    
    uint32_t packet_buf[Decoded1090Packet::kMaxPacketLenWords32];
    uint16_t packet_len_bytes = packet.DumpPacketBuffer(packet_buf);
    uint8_t* packet_bytes = (uint8_t*)packet_buf;
    
    for (int i = 0; i < packet_len_bytes && remaining > 3; i++) {
        int n = snprintf(ptr, remaining, "%02X", packet_bytes[i]);
        if (n < 0 || n >= remaining) break;
        ptr += n;
        remaining -= n;
        written += n;
    }
    
    // Close JSON
    int n = snprintf(ptr, remaining, "\"}");
    if (n < 0 || n >= remaining) {
        return 0;
    }
    
    return written + n;
}

uint16_t MQTTProtocol::FormatAircraftJSON(const Aircraft1090& aircraft,
                                           char* buffer,
                                           uint16_t buffer_size,
                                           FrequencyBand band) {
    const char* band_str = (band == BAND_978_MHZ) ? "UAT" : "1090";
    
    // Compact JSON with short keys including band and category code
    int written = snprintf(buffer, buffer_size,
        "{\"icao\":\"%06X\","
        "\"band\":\"%s\","   // Frequency band
        "\"call\":\"%s\","
        "\"cat\":\"%s\","    // Aircraft category code (e.g., A3, C1)
        "\"lat\":%.5f,"      // 5 decimal places sufficient
        "\"lon\":%.5f,"
        "\"alt\":%ld,"       // Use %ld for int32_t
        "\"hdg\":%.0f,"      // No decimals for heading
        "\"spd\":%.0f,"      // No decimals for speed
        "\"vr\":%d,"         // Vertical rate
        "\"sqk\":\"%04X\","
        "\"gnd\":%d}",       // 1/0 instead of true/false
        (unsigned int)aircraft.icao_address,
        band_str,
        aircraft.callsign,
        GetCategoryCode(aircraft.category_raw),  // Use standard category code
        aircraft.latitude_deg,
        aircraft.longitude_deg,
        aircraft.baro_altitude_ft,
        aircraft.direction_deg,
        aircraft.velocity_kts,
        aircraft.vertical_rate_fpm,
        aircraft.squawk,
        (aircraft.flags & (1 << Aircraft1090::kBitFlagIsAirborne)) ? 0 : 1
    );
    
    if (written < 0 || written >= buffer_size) {
        return 0;
    }
    
    return written;
}

// Binary Implementation
uint16_t MQTTProtocol::FormatPacketBinary(const Decoded1090Packet& packet,
                                           uint8_t* buffer,
                                           uint16_t buffer_size,
                                           FrequencyBand band) {
    uint8_t df = packet.GetDownlinkFormat();
    uint8_t len = (df >= 16) ? 14 : 7;  // Long or short format
    
    if (buffer_size < len + 2) {
        return 0;
    }
    
    // Format: [Type:1][Band:2bits|Reserved:6bits][Data:7-14]
    buffer[0] = BINARY_RAW;
    buffer[1] = (band & 0x03) << 6;  // Band in upper 2 bits
    uint32_t packet_buf[Decoded1090Packet::kMaxPacketLenWords32];
    packet.DumpPacketBuffer(packet_buf);
    memcpy(buffer + 2, packet_buf, len);
    
    return len + 2;
}

uint16_t MQTTProtocol::FormatAircraftBinary(const Aircraft1090& aircraft,
                                             uint8_t* buffer,
                                             uint16_t buffer_size,
                                             FrequencyBand band) {
    if (buffer_size < sizeof(BinaryAircraft)) {
        return 0;
    }
    
    BinaryAircraft* msg = (BinaryAircraft*)buffer;
    
    // Pack all data into compact structure
    msg->type = BINARY_AIRCRAFT;
    msg->band = band & 0x03;  // 2-bit band identifier
    msg->icao24 = aircraft.icao_address & 0xFFFFFF;
    msg->rssi = (aircraft.last_message_signal_strength_dbm + 63) & 0x3F;  // Convert to 6-bit value
    msg->timestamp = (aircraft.last_message_timestamp_ms / 1000) & 0xFFFF;
    
    // Fixed-point encoding for lat/lon (0.00001° precision)
    msg->lat = (int32_t)(aircraft.latitude_deg * 100000) & 0xFFFFFF;
    msg->lon = (int32_t)(aircraft.longitude_deg * 100000) & 0xFFFFFF;
    
    // Altitude in 25ft increments (up to 1.6M ft)
    msg->alt = aircraft.baro_altitude_ft / 25;
    
    // Pack other fields
    msg->hdg = (uint16_t)aircraft.direction_deg & 0x1FF;
    msg->spd = (uint16_t)aircraft.velocity_kts & 0x3FF;
    msg->vrate = aircraft.vertical_rate_fpm / 64;  // 64 fpm increments
    
    // Status flags
    msg->flags = 0;
    if (!(aircraft.flags & (1 << Aircraft1090::kBitFlagIsAirborne))) msg->flags |= 0x01;  // On ground
    if (aircraft.flags & (1 << Aircraft1090::kBitFlagAlert)) msg->flags |= 0x02;
    if (aircraft.flags & (1 << Aircraft1090::kBitFlagIdent)) msg->flags |= 0x04;  // Using Ident for emergency
    if (aircraft.flags & (1 << Aircraft1090::kBitFlagTCASRA)) msg->flags |= 0x08;  // TCAS RA
    
    // Add raw category code (includes both CA and TYPE)
    msg->category = aircraft.category_raw;
    
    // Add callsign (8 bytes, null padded)
    memset(msg->callsign, 0, 8);
    strncpy((char*)msg->callsign, aircraft.callsign, 8);
    
    return sizeof(BinaryAircraft);
}

const char* MQTTProtocol::GetCategoryString(Aircraft1090::Category category) {
    switch (category) {
        case Aircraft1090::kCategoryInvalid: return "Unknown";
        case Aircraft1090::kCategoryReserved: return "Reserved";
        case Aircraft1090::kCategoryNoCategoryInfo: return "None";
        case Aircraft1090::kCategorySurfaceEmergencyVehicle: return "EmergencyVehicle";
        case Aircraft1090::kCategorySurfaceServiceVehicle: return "ServiceVehicle";
        case Aircraft1090::kCategoryGroundObstruction: return "GroundObstruction";
        case Aircraft1090::kCategoryGliderSailplane: return "Glider";
        case Aircraft1090::kCategoryLighterThanAir: return "Balloon";
        case Aircraft1090::kCategoryParachutistSkydiver: return "Parachutist";
        case Aircraft1090::kCategoryUltralightHangGliderParaglider: return "Ultralight";
        case Aircraft1090::kCategoryUnmannedAerialVehicle: return "UAV";
        case Aircraft1090::kCategorySpaceTransatmosphericVehicle: return "Space";
        case Aircraft1090::kCategoryLight: return "Light";
        case Aircraft1090::kCategoryMedium1: return "Medium1";
        case Aircraft1090::kCategoryMedium2: return "Medium2";
        case Aircraft1090::kCategoryHighVortexAircraft: return "HighVortex";
        case Aircraft1090::kCategoryHeavy: return "Heavy";
        case Aircraft1090::kCategoryHighPerformance: return "HighPerformance";
        case Aircraft1090::kCategoryRotorcraft: return "Rotorcraft";
        default: return "Unknown";
    }
}

const char* MQTTProtocol::GetCategoryCode(uint8_t category_raw) {
    // ADS-B emitter category format: [CA:3bits][TYPE:5bits]
    // CA = Capability (0-7), TYPE = Type code (0-31)
    // Standard format is Letter + Number (e.g., A3, C1, D7)
    
    static char category_code[4];
    uint8_t ca = (category_raw >> 5) & 0x07;  // Upper 3 bits
    uint8_t type = category_raw & 0x1F;       // Lower 5 bits
    
    // Map capability to letter (A=0, B=1, C=2, D=3, etc.)
    // But ADS-B uses specific mappings:
    // Set A: No ADS-B emitter category info
    // Set B: Surface vehicles
    // Set C: Airborne, no specific category
    // Set D: Aircraft
    
    char letter;
    switch (ca) {
        case 0: letter = 'A'; break;  // No category info / reserved
        case 1: letter = 'B'; break;  // Surface
        case 2: letter = 'C'; break;  // Reserved
        case 3: letter = 'D'; break;  // Reserved
        case 4: letter = 'A'; break;  // Airborne
        case 5: letter = 'B'; break;  // Airborne
        case 6: letter = 'C'; break;  // Airborne
        case 7: letter = 'D'; break;  // Airborne
        default: letter = 'X'; break;
    }
    
    snprintf(category_code, sizeof(category_code), "%c%d", letter, type);
    return category_code;
}

// Telemetry formatting
uint16_t MQTTProtocol::FormatTelemetry(const TelemetryData& telemetry,
                                        uint8_t* buffer,
                                        uint16_t buffer_size,
                                        Format format) {
    if (!buffer || buffer_size < 20) {
        return 0;
    }
    
    if (format == FORMAT_JSON) {
        return FormatTelemetryJSON(telemetry, (char*)buffer, buffer_size);
    } else {
        return FormatTelemetryBinary(telemetry, buffer, buffer_size);
    }
}

uint16_t MQTTProtocol::FormatTelemetryJSON(const TelemetryData& telemetry,
                                            char* buffer,
                                            uint16_t buffer_size) {
    int written = snprintf(buffer, buffer_size,
        "{\"uptime\":%lu,"
        "\"msgs_adsb_ps\":%u,"
        "\"msgs_mqtt_tx\":%u,"
        "\"cpu_temp\":%d,"
        "\"mem_free\":%u,"
        "\"noise_floor\":%d,"
        "\"rx_1090\":%d,"
        "\"rx_978\":%d,"
        "\"wifi\":%d,"
        "\"mqtt\":%d",
        telemetry.uptime_sec,
        telemetry.messages_received,
        telemetry.messages_sent,
        telemetry.cpu_temp_c,
        telemetry.memory_free_kb,
        telemetry.rssi_noise_floor_dbm,
        telemetry.receiver_1090_enabled,
        telemetry.receiver_978_enabled,
        telemetry.wifi_connected,
        telemetry.mqtt_connected
    );
    
    if (written < 0 || written >= buffer_size) {
        return 0;
    }

    // Append message rate info if present
    int remaining = buffer_size - written;
    char* ptr = buffer + written;

    if (telemetry.mps_feed_count > 0 || telemetry.mps_total > 0) {
        int n = snprintf(ptr, remaining, ",\"mps_total\":%u,\"mps\":[",
                         telemetry.mps_total);
        if (n < 0 || n >= remaining) return 0;
        ptr += n; remaining -= n; written += n;

        for (uint8_t i = 0; i < telemetry.mps_feed_count && i < TelemetryData::kMaxFeedsForTelemetry; i++) {
            n = snprintf(ptr, remaining, "%s%u", (i == 0 ? "" : ","), telemetry.mps_feeds[i]);
            if (n < 0 || n >= remaining) return 0;
            ptr += n; remaining -= n; written += n;
        }
        n = snprintf(ptr, remaining, "]");
        if (n < 0 || n >= remaining) return 0;
        ptr += n; remaining -= n; written += n;
    }

    int n = snprintf(ptr, remaining, "}");
    if (n < 0 || n >= remaining) return 0;
    written += n;

    return written;
}

uint16_t MQTTProtocol::FormatTelemetryBinary(const TelemetryData& telemetry,
                                              uint8_t* buffer,
                                              uint16_t buffer_size) {
    if (buffer_size < sizeof(BinaryTelemetry)) {
        return 0;
    }
    
    BinaryTelemetry* msg = (BinaryTelemetry*)buffer;
    
    msg->type = BINARY_TELEMETRY;
    msg->uptime = (telemetry.uptime_sec / 60) & 0xFFFFFF;  // Minutes
    msg->msgs_rx = telemetry.messages_received;
    msg->msgs_tx = telemetry.messages_sent;
    msg->cpu_temp = telemetry.cpu_temp_c;
    msg->mem_free = telemetry.memory_free_kb / 1024;  // Convert to MB
    msg->noise_floor = telemetry.rssi_noise_floor_dbm;
    
    // Pack status bits
    msg->status = 0;
    if (telemetry.receiver_1090_enabled) msg->status |= 0x01;
    if (telemetry.receiver_978_enabled) msg->status |= 0x02;
    if (telemetry.wifi_connected) msg->status |= 0x04;
    if (telemetry.mqtt_connected) msg->status |= 0x08;
    
    msg->reserved[0] = 0;
    msg->reserved[1] = 0;
    
    return sizeof(BinaryTelemetry);
}

// GPS formatting
uint16_t MQTTProtocol::FormatGPS(const GPSData& gps,
                                  uint8_t* buffer,
                                  uint16_t buffer_size,
                                  Format format) {
    if (!buffer || buffer_size < 15) {
        return 0;
    }
    
    if (format == FORMAT_JSON) {
        return FormatGPSJSON(gps, (char*)buffer, buffer_size);
    } else {
        return FormatGPSBinary(gps, buffer, buffer_size);
    }
}

uint16_t MQTTProtocol::FormatGPSJSON(const GPSData& gps,
                                      char* buffer,
                                      uint16_t buffer_size) {
    const char* fix_str = (gps.fix_status == 2) ? "3D" : 
                         (gps.fix_status == 1) ? "2D" : "None";
    
    int written = snprintf(buffer, buffer_size,
        "{\"lat\":%.6f,"
        "\"lon\":%.6f,"
        "\"alt\":%.1f,"
        "\"fix\":\"%s\","
        "\"sats\":%u,"
        "\"hdop\":%.2f,"
        "\"ts\":%lu}",
        gps.latitude,
        gps.longitude,
        gps.altitude_m,
        fix_str,
        gps.num_satellites,
        gps.hdop,
        gps.timestamp
    );
    
    if (written < 0 || written >= buffer_size) {
        return 0;
    }
    
    return written;
}

uint16_t MQTTProtocol::FormatGPSBinary(const GPSData& gps,
                                        uint8_t* buffer,
                                        uint16_t buffer_size) {
    if (buffer_size < sizeof(BinaryGPS)) {
        return 0;
    }
    
    BinaryGPS* msg = (BinaryGPS*)buffer;
    
    msg->type = BINARY_GPS;
    msg->lat = (int32_t)(gps.latitude * 100000) & 0xFFFFFF;
    msg->lon = (int32_t)(gps.longitude * 100000) & 0xFFFFFF;
    msg->alt = (uint16_t)gps.altitude_m;
    msg->fix = gps.fix_status & 0x03;
    msg->sats = gps.num_satellites & 0x3F;
    msg->hdop = (uint16_t)(gps.hdop * 100);
    msg->timestamp = (gps.timestamp / 60) & 0xFFFF;  // Minutes since epoch
    
    return sizeof(BinaryGPS);
}

bool MQTTProtocol::GetTelemetryTopic(char* topic_buf,
                                     uint16_t topic_size,
                                     const char* msg_type,
                                     bool use_short,
                                     const char* device_id) {
    if (!topic_buf || !msg_type || topic_size < 16) {
        return false;
    }
    
    int written;
    
    if (device_id && strlen(device_id) > 0) {
        // Include device ID in topic hierarchy
        if (use_short) {
            // Short topics for binary format
            const char* short_type = "x";
            if (strcmp(msg_type, "telemetry") == 0) short_type = "t";
            else if (strcmp(msg_type, "gps") == 0) short_type = "g";
            
            written = snprintf(topic_buf, topic_size, "%s/sys/%s", device_id, short_type);
        } else {
            // Standard topics for JSON
            written = snprintf(topic_buf, topic_size, "%s/system/%s", device_id, msg_type);
        }
    } else {
        // No device ID - use original format for backward compatibility
        if (use_short) {
            const char* short_type = "x";
            if (strcmp(msg_type, "telemetry") == 0) short_type = "t";
            else if (strcmp(msg_type, "gps") == 0) short_type = "g";
            
            written = snprintf(topic_buf, topic_size, "sys/%s", short_type);
        } else {
            written = snprintf(topic_buf, topic_size, "system/%s", msg_type);
        }
    }
    
    return (written > 0 && written < topic_size);
}