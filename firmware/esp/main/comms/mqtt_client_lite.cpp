/**
 * MQTT Client Lite - Optimized for minimal size
 *
 * This is a size-optimized version of the MQTT client that:
 * - Uses static buffers instead of dynamic strings
 * - Eliminates cJSON dependency for JSON formatting
 * - Removes redundant code
 * - Simplifies topic generation
 */

#include "mqtt_client.hh"
#include "mqtt_config.h"
#include "esp_log.h"
#include <cstring>
#include <cstdio>

#if CONFIG_MQTT_LITE_MODE

namespace MQTT {

// Static buffers to avoid allocations
static char g_topic_buffer[128];
static char g_json_buffer[512];
static uint8_t g_binary_buffer[256];

// Simplified JSON builder without cJSON
static size_t build_status_json(char* buf, size_t max_len, const AircraftStatus& status) {
    int written = snprintf(buf, max_len,
        "{\"icao\":\"%s\",\"band\":%d",
        status.icao.c_str(), status.band);

    if (!status.callsign.empty()) {
        written += snprintf(buf + written, max_len - written,
            ",\"call\":\"%s\"", status.callsign.c_str());
    }

    if (status.lat != 0 || status.lon != 0) {
        written += snprintf(buf + written, max_len - written,
            ",\"lat\":%.5f,\"lon\":%.5f", status.lat, status.lon);
    }

    if (status.alt_ft != 0) {
        written += snprintf(buf + written, max_len - written,
            ",\"alt_ft\":%d", status.alt_ft);
    }

    if (status.hdg_deg > 0) {
        written += snprintf(buf + written, max_len - written,
            ",\"hdg_deg\":%.1f", status.hdg_deg);
    }

    if (status.spd_kts > 0) {
        written += snprintf(buf + written, max_len - written,
            ",\"spd_kts\":%.1f", status.spd_kts);
    }

    if (status.vr_fpm != 0) {
        written += snprintf(buf + written, max_len - written,
            ",\"vr_fpm\":%d", status.vr_fpm);
    }

    if (!status.squawk.empty()) {
        written += snprintf(buf + written, max_len - written,
            ",\"squawk\":\"%s\"", status.squawk.c_str());
    }

    written += snprintf(buf + written, max_len - written,
        ",\"on_ground\":%s,\"t_ms\":%llu}",
        status.on_ground ? "true" : "false",
        (unsigned long long)status.t_ms);

    return written;
}

// Ultra-compact binary format (16 bytes total)
static size_t build_status_binary(uint8_t* buf, const AircraftStatus& status) {
    size_t pos = 0;

    // Message type (1 byte)
    buf[pos++] = 0x01;  // STATUS

    // ICAO (3 bytes)
    uint32_t icao = 0;
    sscanf(status.icao.c_str(), "%06X", &icao);
    buf[pos++] = (icao >> 16) & 0xFF;
    buf[pos++] = (icao >> 8) & 0xFF;
    buf[pos++] = icao & 0xFF;

    // Lat/Lon (8 bytes, 4 each as fixed-point)
    int32_t lat_fixed = (int32_t)(status.lat * 100000);
    int32_t lon_fixed = (int32_t)(status.lon * 100000);
    memcpy(buf + pos, &lat_fixed, 4); pos += 4;
    memcpy(buf + pos, &lon_fixed, 4); pos += 4;

    // Altitude (2 bytes, units of 25ft)
    uint16_t alt_25ft = (status.alt_ft > 0) ? (status.alt_ft / 25) : 0;
    buf[pos++] = (alt_25ft >> 8) & 0xFF;
    buf[pos++] = alt_25ft & 0xFF;

    // Heading (1 byte)
    buf[pos++] = (uint8_t)((status.hdg_deg * 256) / 360);

    // Speed (1 byte)
    buf[pos++] = (status.spd_kts > 255) ? 255 : (uint8_t)status.spd_kts;

    return pos;  // 16 bytes total
}

// Optimized publish function using static buffers
bool MQTTClient::PublishAircraftStatusLite(const TransponderPacket& packet, uint8_t band) {
    if (!connected_ || !packet.IsValid()) {
        return false;
    }

    // Format ICAO once
    char icao_str[8];
    snprintf(icao_str, sizeof(icao_str), "%06X", packet.address);

    // Rate limit check
    if (!rate_limiter_.ShouldPublish(icao_str)) {
        stats_.messages_dropped++;
        return false;
    }

    // Build topic in static buffer
    snprintf(g_topic_buffer, sizeof(g_topic_buffer),
             "%s/aircraft/%s/status", config_.device_id.c_str(), icao_str);

    // Convert packet to status
    AircraftStatus status = PacketToStatus(packet, band);

    // Publish based on format
    int msg_id = -1;
    if (config_.format == SettingsManager::MQTTFormat::kMQTTFormatBinary) {
        size_t len = build_status_binary(g_binary_buffer, status);
        msg_id = esp_mqtt_client_publish(client_, g_topic_buffer,
                                         (const char*)g_binary_buffer, len, 0, false);
        stats_.bytes_sent += len;
    } else {
        size_t len = build_status_json(g_json_buffer, sizeof(g_json_buffer), status);
        msg_id = esp_mqtt_client_publish(client_, g_topic_buffer,
                                         g_json_buffer, len, 0, false);
        stats_.bytes_sent += len;
    }

    if (msg_id >= 0) {
        stats_.messages_published++;
        return true;
    } else {
        stats_.messages_dropped++;
        return false;
    }
}

// Simplified telemetry with static buffer
bool MQTTClient::PublishTelemetryLite(const Telemetry& telemetry) {
    if (!connected_) return false;

    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        now - last_telemetry_publish_).count();

    if (elapsed < config_.telemetry_interval_sec) {
        return false;
    }

    last_telemetry_publish_ = now;

    // Build topic
    snprintf(g_topic_buffer, sizeof(g_topic_buffer),
             "%s/telemetry", config_.device_id.c_str());

    // Build minimal telemetry JSON
    int len = snprintf(g_json_buffer, sizeof(g_json_buffer),
        "{\"uptime\":%u,\"msgs_rx\":%u,\"msgs_tx\":%u,\"cpu_temp\":%d,"
        "\"mem_free\":%u,\"fw\":\"%d.%d.%d\"}",
        telemetry.uptime_sec, telemetry.msgs_rx, telemetry.msgs_tx,
        telemetry.cpu_temp_c, telemetry.mem_free_kb,
        ObjectDictionary::kFirmwareVersionMajor,
        ObjectDictionary::kFirmwareVersionMinor,
        ObjectDictionary::kFirmwareVersionPatch);

    int msg_id = esp_mqtt_client_publish(client_, g_topic_buffer,
                                         g_json_buffer, len, 1, false);
    if (msg_id >= 0) {
        stats_.bytes_sent += len;
    }

    return msg_id >= 0;
}

}  // namespace MQTT

#endif // CONFIG_MQTT_LITE_MODE