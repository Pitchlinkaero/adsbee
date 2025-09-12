#include "mqtt_protocol.hh"
#include <stdio.h>

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

uint16_t MQTTProtocol::FormatAircraft(const Aircraft& aircraft,
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
                            bool use_short) {
    if (!topic_buf || !msg_type || topic_size < 16) {
        return false;
    }
    
    int written;
    const char* band_str = (band == BAND_978_MHZ) ? "uat" : "adsb";
    const char* band_short = (band == BAND_978_MHZ) ? "u" : "a";
    
    if (use_short) {
        // Short topics for binary format (saves ~20 bytes)
        const char* short_type = "x";
        if (strcmp(msg_type, "position") == 0) short_type = "p";
        else if (strcmp(msg_type, "status") == 0) short_type = "s";
        else if (strcmp(msg_type, "raw") == 0) short_type = "r";
        
        written = snprintf(topic_buf, topic_size, "%s/%06X/%s", band_short, icao24, short_type);
    } else {
        // Standard topics for JSON - include band in topic
        written = snprintf(topic_buf, topic_size, "%s/%06X/%s", band_str, icao24, msg_type);
    }
    
    return (written > 0 && written < topic_size);
}

// JSON Implementation
uint16_t MQTTProtocol::FormatPacketJSON(const Decoded1090Packet& packet,
                                         char* buffer,
                                         uint16_t buffer_size,
                                         FrequencyBand band) {
    uint32_t icao24 = packet.transponder_packet.aa_or_vs & 0xFFFFFF;
    uint8_t df = packet.transponder_packet.format;
    const char* band_str = (band == BAND_978_MHZ) ? "UAT" : "1090";
    
    // Build JSON with essential fields including band
    int written = snprintf(buffer, buffer_size,
        "{\"t\":%llu,"           // timestamp (short key)
        "\"icao\":\"%06X\","
        "\"band\":\"%s\","       // Frequency band
        "\"df\":%d,"
        "\"rssi\":%d,"
        "\"hex\":\"",
        packet.timestamp_12mhz / 12000,  // Convert to ms
        icao24,
        band_str,
        df,
        packet.signal_level
    );
    
    if (written < 0 || written >= buffer_size) {
        return 0;
    }
    
    // Add hex data
    char* ptr = buffer + written;
    int remaining = buffer_size - written;
    
    for (int i = 0; i < packet.transponder_packet.packet_len_bytes && remaining > 3; i++) {
        int n = snprintf(ptr, remaining, "%02X", packet.transponder_packet.packet_buf[i]);
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

uint16_t MQTTProtocol::FormatAircraftJSON(const Aircraft& aircraft,
                                           char* buffer,
                                           uint16_t buffer_size,
                                           FrequencyBand band) {
    const char* band_str = (band == BAND_978_MHZ) ? "UAT" : "1090";
    
    // Compact JSON with short keys including band
    int written = snprintf(buffer, buffer_size,
        "{\"icao\":\"%06X\","
        "\"band\":\"%s\","   // Frequency band
        "\"call\":\"%s\","
        "\"lat\":%.5f,"      // 5 decimal places sufficient
        "\"lon\":%.5f,"
        "\"alt\":%d,"
        "\"hdg\":%.0f,"      // No decimals for heading
        "\"spd\":%.0f,"      // No decimals for speed
        "\"vr\":%d,"
        "\"sqk\":\"%04X\","
        "\"gnd\":%d}",       // 1/0 instead of true/false
        aircraft.icao_address,
        band_str,
        aircraft.callsign,
        aircraft.latitude,
        aircraft.longitude,
        aircraft.altitude_ft,
        aircraft.heading_deg,
        aircraft.ground_speed_kt,
        aircraft.vertical_rate_fpm,
        aircraft.squawk,
        aircraft.on_ground ? 1 : 0
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
    uint8_t df = packet.transponder_packet.format;
    uint8_t len = (df >= 16) ? 14 : 7;  // Long or short format
    
    if (buffer_size < len + 2) {
        return 0;
    }
    
    // Format: [Type:1][Band:2bits|Reserved:6bits][Data:7-14]
    buffer[0] = BINARY_RAW;
    buffer[1] = (band & 0x03) << 6;  // Band in upper 2 bits
    memcpy(buffer + 2, packet.transponder_packet.packet_buf, len);
    
    return len + 2;
}

uint16_t MQTTProtocol::FormatAircraftBinary(const Aircraft& aircraft,
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
    msg->rssi = (aircraft.signal_level + 63) & 0x3F;  // Convert to 6-bit value
    msg->timestamp = (aircraft.last_message_timestamp_12mhz / 12000000) & 0xFFFF;
    
    // Fixed-point encoding for lat/lon (0.00001° precision)
    msg->lat = (int32_t)(aircraft.latitude * 100000) & 0xFFFFFF;
    msg->lon = (int32_t)(aircraft.longitude * 100000) & 0xFFFFFF;
    
    // Altitude in 25ft increments (up to 1.6M ft)
    msg->alt = aircraft.altitude_ft / 25;
    
    // Pack other fields
    msg->hdg = (uint16_t)aircraft.heading_deg & 0x1FF;
    msg->spd = (uint16_t)aircraft.ground_speed_kt & 0x3FF;
    msg->vrate = aircraft.vertical_rate_fpm / 64;  // 64 fpm increments
    
    // Status flags
    msg->flags = 0;
    if (aircraft.on_ground) msg->flags |= 0x01;
    if (aircraft.alert) msg->flags |= 0x02;
    if (aircraft.emergency) msg->flags |= 0x04;
    if (aircraft.spi) msg->flags |= 0x08;
    
    return sizeof(BinaryAircraft);
}