#include "mqtt_client.hh"
#include "mqtt_config.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "hal.hh"  // For get_time_since_boot_ms
#include "cJSON.h"  // ESP-IDF's cJSON component
#include "object_dictionary.hh"  // For firmware version
#include <cstring>
#include <algorithm>
#include <cstdlib>
#include "mqtt_ota.hh"
#include <memory>

static const char* TAG = "MQTT";

namespace MQTT {

// AircraftRateLimiter implementation
bool AircraftRateLimiter::ShouldPublish(const std::string& icao) {
    auto now = std::chrono::steady_clock::now();
    auto it = aircraft_states_.find(icao);

    if (it == aircraft_states_.end()) {
        // New aircraft, publish immediately
        aircraft_states_[icao].last_published = now;
        return true;
    }

    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - it->second.last_published);

    if (elapsed >= kMinPublishInterval) {
        it->second.last_published = now;
        return true;
    }

    return false;
}

void AircraftRateLimiter::Reset() {
    aircraft_states_.clear();
}

// MQTTClient implementation
MQTTClient::MQTTClient(const Config& config, uint16_t feed_index)
    : config_(config),
      feed_index_(feed_index),
      client_(nullptr),
      connected_(false),
      reconnect_delay_ms_(kInitialReconnectDelayMs) {

    // Initialize statistics
    stats_ = {};

    // Create OTA handler if enabled
    ESP_LOGI(TAG, "Creating MQTT client for feed %d, OTA enabled: %s",
             feed_index_, config_.ota_enabled ? "YES" : "NO");
    if (config_.ota_enabled) {
        ota_handler_ = std::make_unique<MQTTOTAHandler>(config_.device_id, feed_index_);
        ESP_LOGI(TAG, "OTA handler created successfully for feed %d", feed_index_);
    } else {
        ota_handler_ = nullptr;
        ESP_LOGI(TAG, "OTA handler NOT created for feed %d (disabled)", feed_index_);
    }

    // Configure MQTT client
    esp_mqtt_client_config_t mqtt_cfg = {};

    // Build full URI
    std::string full_uri;
    if (config_.use_tls) {
        full_uri = "mqtts://" + config_.broker_uri + ":" + std::to_string(config_.port);
    } else {
        full_uri = "mqtt://" + config_.broker_uri + ":" + std::to_string(config_.port);
    }

    mqtt_cfg.broker.address.uri = full_uri.c_str();

    // Authentication
    if (!config_.username.empty()) {
        mqtt_cfg.credentials.username = config_.username.c_str();
    }
    if (!config_.password.empty()) {
        mqtt_cfg.credentials.authentication.password = config_.password.c_str();
    }

    // Client ID
    if (!config_.client_id.empty()) {
        mqtt_cfg.credentials.client_id = config_.client_id.c_str();
    } else {
        // Generate client ID from device ID
        std::string generated_client_id = "adsbee_" + config_.device_id;
        mqtt_cfg.credentials.client_id = generated_client_id.c_str();
    }

#if CONFIG_MQTT_TLS_ENABLED
    // TLS/SSL Configuration
    if (config_.use_tls) {
        // Skip certificate verification for now (insecure but reduces size)
        mqtt_cfg.broker.verification.skip_cert_common_name_check = true;
        mqtt_cfg.broker.verification.certificate = nullptr;  // No CA cert verification
    }
#else
    if (config_.use_tls) {
        ESP_LOGW(TAG, "TLS requested but not compiled in - using unencrypted connection");
        // Force non-TLS URI
        full_uri = "mqtt://" + config_.broker_uri + ":" + std::to_string(config_.port);
        mqtt_cfg.broker.address.uri = full_uri.c_str();
    }
#endif

    // Buffer configuration for OTA chunks
    mqtt_cfg.buffer.size = 8192;  // Increase buffer size for OTA chunks (4096 data + header)
    mqtt_cfg.buffer.out_size = 8192;  // Output buffer for publishing

    // Last Will and Testament (LWT)
    std::string lwt_topic = GetOnlineTopic();
    mqtt_cfg.session.last_will.topic = lwt_topic.c_str();
    mqtt_cfg.session.last_will.msg = "0";
    mqtt_cfg.session.last_will.msg_len = 1;
    mqtt_cfg.session.last_will.qos = 1;
    mqtt_cfg.session.last_will.retain = true;

    // Create MQTT client
    client_ = esp_mqtt_client_init(&mqtt_cfg);
    if (client_ == nullptr) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client for feed %d", feed_index_);
        return;
    }

    // Register event handler
    esp_mqtt_client_register_event(client_, MQTT_EVENT_ANY, MQTTEventHandler, this);
}

MQTTClient::~MQTTClient() {
    if (client_) {
        Disconnect();
        esp_mqtt_client_destroy(client_);
    }
}

bool MQTTClient::Connect() {
    if (!client_) {
        return false;
    }

    if (connected_) {
        return true;
    }

    // Check if enough time has passed since last attempt
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_connect_attempt_).count();

    if (elapsed < reconnect_delay_ms_) {
        return false;  // Too soon to retry
    }

    last_connect_attempt_ = now;

    ESP_LOGI(TAG, "Connecting to MQTT broker %s for feed %d",
             config_.broker_uri.c_str(), feed_index_);

    esp_err_t err = esp_mqtt_client_start(client_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MQTT client: %s", esp_err_to_name(err));
        stats_.last_error = "Failed to start client: " + std::string(esp_err_to_name(err));
        ScheduleReconnect();
        return false;
    }

    return true;
}

void MQTTClient::Disconnect() {
    if (client_ && connected_) {
        esp_mqtt_client_stop(client_);
        connected_ = false;
    }
}

bool MQTTClient::IsConnected() const {
    return connected_;
}

void MQTTClient::MQTTEventHandler(void* handler_args, esp_event_base_t base,
                                   int32_t event_id, void* event_data) {
    MQTTClient* self = static_cast<MQTTClient*>(handler_args);
    esp_mqtt_event_handle_t event = static_cast<esp_mqtt_event_handle_t>(event_data);

    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "Connected to MQTT broker for feed %d", self->feed_index_);
            self->HandleConnect();
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "Disconnected from MQTT broker for feed %d", self->feed_index_);
            self->HandleDisconnect();
            break;

        case MQTT_EVENT_DATA:
            self->HandleMessage(event);
            break;

        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT error for feed %d: %d", self->feed_index_, event->error_handle->error_type);
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                self->stats_.last_error = "TCP transport error";
            } else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
                self->stats_.last_error = "Connection refused";
            }
            break;

        default:
            break;
    }
}

void MQTTClient::HandleConnect() {
    connected_ = true;
    reconnect_delay_ms_ = kInitialReconnectDelayMs;  // Reset backoff

    // Publish online status with retain
    std::string online_topic = GetOnlineTopic();
    esp_mqtt_client_publish(client_, online_topic.c_str(), "1", 1, 1, true);

    // Initialize and subscribe to OTA topics if enabled
    ESP_LOGI(TAG, "HandleConnect: OTA handler exists: %s", ota_handler_ ? "YES" : "NO");
    if (ota_handler_) {
        // Initialize the OTA handler with this client
        ota_handler_->Initialize(this);
        ESP_LOGI(TAG, "HandleConnect: OTA handler initialized");

        std::string ota_base = GetOTABaseTopic();
        ESP_LOGI(TAG, "HandleConnect: OTA base topic: %s", ota_base.c_str());

        // Subscribe to control topics
        std::string manifest_topic = ota_base + "/control/manifest";
        std::string command_topic = ota_base + "/control/command";
        std::string chunk_topic = ota_base + "/data/chunk/+";

        ESP_LOGI(TAG, "HandleConnect: Subscribing to: %s", manifest_topic.c_str());
        esp_mqtt_client_subscribe(client_, manifest_topic.c_str(), 1);
        ESP_LOGI(TAG, "HandleConnect: Subscribing to: %s", command_topic.c_str());
        esp_mqtt_client_subscribe(client_, command_topic.c_str(), 1);
        ESP_LOGI(TAG, "HandleConnect: Subscribing to: %s", chunk_topic.c_str());
        esp_mqtt_client_subscribe(client_, chunk_topic.c_str(), 1);

        ESP_LOGI(TAG, "Initialized and subscribed to OTA topics for device %s", config_.device_id.c_str());
    }

    // Clear rate limiter to start fresh
    rate_limiter_.Reset();
}

void MQTTClient::HandleDisconnect() {
    connected_ = false;
    stats_.reconnect_count++;

    // Reset OTA state on disconnect to prevent stuck state
    if (ota_handler_) {
        ESP_LOGI(TAG, "HandleDisconnect: Aborting OTA due to disconnect");
        ota_handler_->AbortOTA();
    }

    ScheduleReconnect();
}

void MQTTClient::HandleMessage(esp_mqtt_event_handle_t event) {
    // Extract topic first to check if it's an OTA message
    std::string topic(event->topic, event->topic_len);
    std::string ota_base = GetOTABaseTopic();

    ESP_LOGI(TAG, "HandleMessage: Received message on topic: %s", topic.c_str());
    ESP_LOGI(TAG, "HandleMessage: OTA base topic: %s", ota_base.c_str());
    ESP_LOGI(TAG, "HandleMessage: OTA handler exists: %s", ota_handler_ ? "YES" : "NO");

    // Check if it's an OTA message
    if (topic.find(ota_base) == 0) {
        ESP_LOGI(TAG, "HandleMessage: This IS an OTA message");
        // It's an OTA message - check if handler exists
        if (!ota_handler_) {
            ESP_LOGW(TAG, "OTA message received but OTA handler is NULL");

            // Publish error status to inform the publisher
            std::string status_topic = ota_base + "/status/state";
            std::string error_json = "{\"state\":\"DISABLED\",\"error\":\"OTA not enabled for this feed\"}";
            esp_mqtt_client_publish(client_, status_topic.c_str(),
                                   error_json.c_str(), error_json.length(), 1, false);
            return;
        }

        ESP_LOGI(TAG, "Received message on topic: %s (OTA base: %s)", topic.c_str(), ota_base.c_str());
    } else {
        return;  // Not an OTA topic
    }

    // Remove base from topic
    std::string ota_topic = topic.substr(ota_base.length());

    // Handle different OTA messages
    ESP_LOGI(TAG, "HandleMessage: OTA topic suffix: %s", ota_topic.c_str());
    if (ota_topic == "/control/manifest") {
        ESP_LOGI(TAG, "HandleMessage: Processing OTA manifest - data_len=%d", event->data_len);
        ESP_LOGI(TAG, "HandleMessage: Manifest data: %.*s", event->data_len, event->data);
        // Parse JSON manifest
        cJSON* root = cJSON_ParseWithLength(event->data, event->data_len);
        if (!root) {
            ESP_LOGE(TAG, "Failed to parse OTA manifest JSON");
            return;
        }

        MQTTOTAHandler::Manifest manifest;

        cJSON* version = cJSON_GetObjectItem(root, "version");
        if (version && cJSON_IsString(version)) {
            manifest.version = version->valuestring;
        }

        cJSON* size = cJSON_GetObjectItem(root, "size");
        if (size && cJSON_IsNumber(size)) {
            manifest.size = size->valueint;
        }

        cJSON* chunks = cJSON_GetObjectItem(root, "chunks");
        if (chunks && cJSON_IsNumber(chunks)) {
            manifest.total_chunks = chunks->valueint;
        }

        cJSON* chunk_size = cJSON_GetObjectItem(root, "chunk_size");
        if (chunk_size && cJSON_IsNumber(chunk_size)) {
            manifest.chunk_size = chunk_size->valueint;
        }

        cJSON* sha256 = cJSON_GetObjectItem(root, "sha256");
        if (sha256 && cJSON_IsString(sha256)) {
            manifest.sha256 = sha256->valuestring;
        }

        cJSON* session_id = cJSON_GetObjectItem(root, "session_id");
        if (session_id && cJSON_IsString(session_id)) {
            manifest.session_id = session_id->valuestring;
        }

        cJSON_Delete(root);

        ESP_LOGI(TAG, "HandleMessage: Manifest parsed - version=%s, size=%d, chunks=%d, session=%s",
                 manifest.version.c_str(), manifest.size, manifest.total_chunks, manifest.session_id.c_str());

        // Process manifest
        bool manifest_result = ota_handler_->HandleManifest(manifest);
        ESP_LOGI(TAG, "HandleMessage: HandleManifest returned: %s", manifest_result ? "TRUE" : "FALSE");

        if (manifest_result) {
            // Publish acknowledgment
            std::string ack_topic = ota_base + "/status/manifest_ack";
            esp_mqtt_client_publish(client_, ack_topic.c_str(), "1", 1, 1, false);
            ESP_LOGI(TAG, "HandleMessage: Published manifest ACK");
        } else {
            ESP_LOGE(TAG, "HandleMessage: Manifest rejected by handler");
        }

    } else if (ota_topic == "/control/command") {
        // Parse command
        cJSON* root = cJSON_ParseWithLength(event->data, event->data_len);
        if (!root) {
            ESP_LOGE(TAG, "Failed to parse OTA command JSON");
            return;
        }

        cJSON* cmd = cJSON_GetObjectItem(root, "command");
        cJSON* session_id = cJSON_GetObjectItem(root, "session_id");
        if (cmd && cJSON_IsString(cmd)) {
            std::string command = cmd->valuestring;
            std::string sid = (session_id && cJSON_IsString(session_id)) ? session_id->valuestring : std::string();
            ESP_LOGI(TAG, "HandleMessage: Processing OTA command: %s with session: %s",
                     command.c_str(), sid.c_str());
            bool result = ota_handler_->HandleCommand(command, sid);
            ESP_LOGI(TAG, "HandleMessage: Command result: %s", result ? "SUCCESS" : "FAILED");

            // Publish current state
            std::string state_topic = ota_base + "/status/state";
            std::string state_json = ota_handler_->GetStateJSON();
            ESP_LOGI(TAG, "HandleMessage: Publishing state: %s", state_json.c_str());
            esp_mqtt_client_publish(client_, state_topic.c_str(),
                                   state_json.c_str(), state_json.length(), 1, false);
        } else {
            ESP_LOGW(TAG, "HandleMessage: Command JSON missing 'command' field");
        }

        cJSON_Delete(root);

    } else if (ota_topic.find("/data/chunk/") == 0) {
        // Extract chunk index from topic
        std::string chunk_idx_str = ota_topic.substr(12);  // "/data/chunk/".length() = 12
        uint32_t chunk_index = std::stoul(chunk_idx_str);

        // Debug: Log chunk reception
        ESP_LOGI(TAG, "Received chunk %lu: data_len=%d, topic_len=%d, total_data_len=%d",
                 (unsigned long)chunk_index, event->data_len, event->topic_len, event->total_data_len);

        // Defensive reassembly for partial MQTT messages
        static std::unordered_map<uint32_t, std::string> chunk_bufs;  // keyed by chunk index
        if (event->current_data_offset == 0) {
            chunk_bufs[chunk_index].clear();
            chunk_bufs[chunk_index].reserve(event->total_data_len);
        }
        chunk_bufs[chunk_index].append(event->data, event->data_len);

        bool success = false;
        if ((int)chunk_bufs[chunk_index].size() >= event->total_data_len) {
            // Have full payload; process once
            success = ota_handler_->HandleChunk(chunk_index,
                                                (const uint8_t*)chunk_bufs[chunk_index].data(),
                                                chunk_bufs[chunk_index].size());
            chunk_bufs.erase(chunk_index);
        } else {
            // Not complete yet; wait for more fragments
            ESP_LOGI(TAG, "Chunk %lu partial: %d/%d bytes",
                     (unsigned long)chunk_index, (int)chunk_bufs[chunk_index].size(), event->total_data_len);
            // Defer ACK until full
            return;
        }

        // Publish ACK/NACK
        std::string ack_topic = ota_base + "/status/ack/" + chunk_idx_str;
        esp_mqtt_client_publish(client_, ack_topic.c_str(),
                               success ? "1" : "0", 1, 0, false);

        // Publish progress periodically (every 10 chunks)
        if (chunk_index % 10 == 0) {
            std::string progress_topic = ota_base + "/status/progress";
            std::string progress_json = ota_handler_->GetProgressJSON();
            esp_mqtt_client_publish(client_, progress_topic.c_str(),
                                   progress_json.c_str(), progress_json.length(), 0, false);
        }
    }
}

void MQTTClient::ScheduleReconnect() {
    // Apply exponential backoff with jitter
    reconnect_delay_ms_ = std::min(
        static_cast<uint32_t>(reconnect_delay_ms_ * kReconnectBackoffFactor),
        kMaxReconnectDelayMs
    );

    // Add jitter (±20%)
    uint32_t jitter = (esp_random() % (uint32_t)(reconnect_delay_ms_ * kReconnectJitterFactor * 2))
                      - (reconnect_delay_ms_ * kReconnectJitterFactor);
    reconnect_delay_ms_ += jitter;

    ESP_LOGI(TAG, "Will reconnect in %lu ms for feed %d", reconnect_delay_ms_, feed_index_);
}

std::string MQTTClient::GetStatusTopic(const std::string& icao, uint8_t band) const {
    std::string band_prefix = (band == 2) ? "uat" : "adsb";

    if (config_.format == SettingsManager::MQTTFormat::kMQTTFormatBinary) {
        // Short form for binary
        std::string short_band = (band == 2) ? "u" : "a";
        return config_.device_id + "/" + short_band + "/" + icao + "/s";
    } else {
        // Long form for JSON
        return config_.device_id + "/" + band_prefix + "/" + icao + "/status";
    }
}

std::string MQTTClient::GetRawTopic(const std::string& icao, uint8_t band) const {
    std::string band_prefix = (band == 2) ? "uat" : "adsb";
    return config_.device_id + "/" + band_prefix + "/" + icao + "/raw";
}

std::string MQTTClient::GetTelemetryTopic() const {
    if (config_.format == SettingsManager::MQTTFormat::kMQTTFormatBinary) {
        return config_.device_id + "/sys/t";
    } else {
        return config_.device_id + "/system/telemetry";
    }
}

std::string MQTTClient::GetGPSTopic() const {
    if (config_.format == SettingsManager::MQTTFormat::kMQTTFormatBinary) {
        return config_.device_id + "/sys/g";
    } else {
        return config_.device_id + "/system/gps";
    }
}

// (Removed duplicate topic helper definitions)

std::string MQTTClient::GetOnlineTopic() const {
    return config_.device_id + "/system/online";
}

std::string MQTTClient::GetOTABaseTopic() const {
    return config_.device_id + "/ota";
}

AircraftStatus MQTTClient::PacketToStatus(const TransponderPacket& packet, uint8_t band) const {
    AircraftStatus status = {};

    // Extract ICAO address (uppercase hex)
    char icao_str[7];
    snprintf(icao_str, sizeof(icao_str), "%06lX", (unsigned long)packet.address);
    status.icao = icao_str;

    status.band = band;

    // Extract data from packet
    if (packet.HasCallsign()) {
        status.callsign = std::string(packet.callsign, 8);
        // Trim trailing spaces
        status.callsign.erase(status.callsign.find_last_not_of(' ') + 1);
    }

    if (packet.HasCategory()) {
        // Convert category to string format (e.g., "A3")
        char cat_str[4];  // Increased buffer size to 4
        snprintf(cat_str, sizeof(cat_str), "%c%d",
                 'A' + ((packet.category >> 4) & 0x0F),
                 packet.category & 0x0F);
        status.category = cat_str;
    }

    if (packet.HasPosition()) {
        status.lat = packet.latitude;
        status.lon = packet.longitude;
    }

    if (packet.HasAltitude()) {
        status.alt_ft = packet.altitude;
    }

    if (packet.HasHeading()) {
        status.hdg_deg = packet.heading;
    }

    if (packet.HasVelocity()) {
        status.spd_kts = packet.velocity;
    }

    if (packet.HasVerticalRate()) {
        status.vr_fpm = packet.vertical_rate;
    }

    if (packet.HasSquawk()) {
        char sqk_str[8];  // Increased buffer size to handle worst case
        snprintf(sqk_str, sizeof(sqk_str), "%04o", (unsigned int)packet.squawk);
        status.squawk = sqk_str;
    }

    status.on_ground = packet.airborne == 0;
    status.t_ms = packet.timestamp_ms;

    return status;
}

bool MQTTClient::PublishAircraftStatus(const TransponderPacket& packet, uint8_t band) {
    if (!connected_ || !packet.IsValid()) {
        return false;
    }

    // Check rate limit (1Hz per aircraft)
    char icao_str[9];  // Increased buffer size to handle worst case
    snprintf(icao_str, sizeof(icao_str), "%06lX", (unsigned long)packet.address);

    if (!rate_limiter_.ShouldPublish(icao_str)) {
        stats_.messages_dropped++;
        return false;  // Drop to maintain 1Hz rate
    }

    // Convert packet to status
    AircraftStatus status = PacketToStatus(packet, band);

    // Generate topic using stack buffer
    char topic_buf[MQTT_MAX_TOPIC_LEN];
    snprintf(topic_buf, sizeof(topic_buf), "%s/aircraft/%s/status",
             config_.device_id.c_str(), status.icao.c_str());

    // Serialize based on format
    int msg_id = -1;
    if (config_.format == SettingsManager::MQTTFormat::kMQTTFormatBinary) {
        uint8_t binary_buf[MQTT_BINARY_BUFFER_SIZE];
        size_t binary_len = SerializeStatusBinaryToBuffer(status, binary_buf, sizeof(binary_buf));
        msg_id = esp_mqtt_client_publish(client_, topic_buf,
                                          (const char*)binary_buf,
                                          binary_len, 0, false);
        stats_.bytes_sent += binary_len;
    } else {
        char json_buf[MQTT_JSON_BUFFER_SIZE];
        size_t json_len = SerializeStatusJSONToBuffer(status, json_buf, sizeof(json_buf));
        msg_id = esp_mqtt_client_publish(client_, topic_buf,
                                          json_buf,
                                          json_len, 0, false);
        stats_.bytes_sent += json_len;
    }

    if (msg_id >= 0) {
        stats_.messages_published++;
        return true;
    } else {
        stats_.messages_dropped++;
        return false;
    }
}

bool MQTTClient::PublishTelemetry(const Telemetry& telemetry) {
    if (!connected_) {
        return false;
    }

    // Check if enough time has passed
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        now - last_telemetry_publish_).count();

    if (elapsed < config_.telemetry_interval_sec) {
        return false;
    }

    last_telemetry_publish_ = now;

    // Use stack buffer for topic
    char topic_buf[MQTT_MAX_TOPIC_LEN];
    snprintf(topic_buf, sizeof(topic_buf), "%s/telemetry", config_.device_id.c_str());

    int msg_id = -1;
    if (config_.format == SettingsManager::MQTTFormat::kMQTTFormatBinary) {
        uint8_t binary_buf[256];
        size_t binary_len = SerializeTelemetryBinaryToBuffer(telemetry, binary_buf, sizeof(binary_buf));
        msg_id = esp_mqtt_client_publish(client_, topic_buf,
                                          (const char*)binary_buf,
                                          binary_len, 1, false);  // QoS 1
        stats_.bytes_sent += binary_len;
    } else {
        char json_buf[MQTT_JSON_BUFFER_SIZE];
        size_t json_len = SerializeTelemetryJSONToBuffer(telemetry, json_buf, sizeof(json_buf));
        msg_id = esp_mqtt_client_publish(client_, topic_buf,
                                          json_buf,
                                          json_len, 1, false);  // QoS 1
        stats_.bytes_sent += json_len;
    }

    return msg_id >= 0;
}

bool MQTTClient::PublishGPS(const GPS& gps) {
    if (!connected_) {
        return false;
    }

    // Check if enough time has passed
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        now - last_gps_publish_).count();

    if (elapsed < config_.gps_interval_sec) {
        return false;
    }

    last_gps_publish_ = now;

    // Use stack buffer for topic
    char topic_buf[MQTT_MAX_TOPIC_LEN];
    if (config_.format == SettingsManager::MQTTFormat::kMQTTFormatBinary) {
        snprintf(topic_buf, sizeof(topic_buf), "%s/sys/g", config_.device_id.c_str());
    } else {
        snprintf(topic_buf, sizeof(topic_buf), "%s/system/gps", config_.device_id.c_str());
    }

    int msg_id = -1;
    if (config_.format == SettingsManager::MQTTFormat::kMQTTFormatBinary) {
        uint8_t binary_buf[32];
        size_t binary_len = SerializeGPSBinaryToBuffer(gps, binary_buf, sizeof(binary_buf));
        msg_id = esp_mqtt_client_publish(client_, topic_buf,
                                          (const char*)binary_buf,
                                          binary_len, 0, false);
        stats_.bytes_sent += binary_len;
    } else {
        char json_buf[256];
        size_t json_len = SerializeGPSJSONToBuffer(gps, json_buf, sizeof(json_buf));
        msg_id = esp_mqtt_client_publish(client_, topic_buf,
                                          json_buf,
                                          json_len, 0, false);
        stats_.bytes_sent += json_len;
    }

    return msg_id >= 0;
}

// JSON Serialization
std::string MQTTClient::SerializeStatusJSON(const AircraftStatus& status) const {
    cJSON* root = cJSON_CreateObject();

    cJSON_AddStringToObject(root, "icao", status.icao.c_str());
    cJSON_AddNumberToObject(root, "band", status.band);

    if (!status.callsign.empty()) {
        cJSON_AddStringToObject(root, "call", status.callsign.c_str());
    }

    if (!status.category.empty()) {
        cJSON_AddStringToObject(root, "cat", status.category.c_str());
    }

    if (status.lat != 0 || status.lon != 0) {
        cJSON_AddNumberToObject(root, "lat", status.lat);
        cJSON_AddNumberToObject(root, "lon", status.lon);
    }

    if (status.alt_ft != 0) {
        cJSON_AddNumberToObject(root, "alt_ft", status.alt_ft);
    }

    if (status.hdg_deg != 0) {
        cJSON_AddNumberToObject(root, "hdg_deg", status.hdg_deg);
    }

    if (status.spd_kts != 0) {
        cJSON_AddNumberToObject(root, "spd_kts", status.spd_kts);
    }

    if (status.vr_fpm != 0) {
        cJSON_AddNumberToObject(root, "vr_fpm", status.vr_fpm);
    }

    if (!status.squawk.empty()) {
        cJSON_AddStringToObject(root, "sqk", status.squawk.c_str());
    }

    cJSON_AddBoolToObject(root, "on_ground", status.on_ground);
    cJSON_AddNumberToObject(root, "t_ms", status.t_ms);

    char* json_str = cJSON_PrintUnformatted(root);
    std::string result(json_str);

    cJSON_free(json_str);
    cJSON_Delete(root);

    return result;
}

std::string MQTTClient::SerializeTelemetryJSON(const Telemetry& telemetry) const {
    cJSON* root = cJSON_CreateObject();

    cJSON_AddNumberToObject(root, "uptime_sec", telemetry.uptime_sec);
    cJSON_AddNumberToObject(root, "msgs_rx", telemetry.msgs_rx);
    cJSON_AddNumberToObject(root, "msgs_tx", telemetry.msgs_tx);
    cJSON_AddNumberToObject(root, "cpu_temp_c", telemetry.cpu_temp_c);
    cJSON_AddNumberToObject(root, "mem_free_kb", telemetry.mem_free_kb);
    cJSON_AddNumberToObject(root, "noise_floor_dbm", telemetry.noise_floor_dbm);
    cJSON_AddBoolToObject(root, "rx_1090", telemetry.rx_1090);
    cJSON_AddBoolToObject(root, "rx_978", telemetry.rx_978);
    cJSON_AddBoolToObject(root, "wifi", telemetry.wifi);
    cJSON_AddBoolToObject(root, "mqtt", telemetry.mqtt);
    cJSON_AddStringToObject(root, "fw_version", telemetry.fw_version.c_str());

    // Add OTA status to help diagnose configuration issues
    cJSON_AddBoolToObject(root, "ota_enabled", config_.ota_enabled);

    if (telemetry.mps_total > 0) {
        cJSON_AddNumberToObject(root, "mps_total", telemetry.mps_total);
    }

    if (!telemetry.mps_feeds.empty()) {
        cJSON* feeds_array = cJSON_CreateArray();
        for (uint32_t mps : telemetry.mps_feeds) {
            cJSON_AddItemToArray(feeds_array, cJSON_CreateNumber(mps));
        }
        cJSON_AddItemToObject(root, "mps_feeds", feeds_array);
    }

    // Add decoder statistics if available
    if (telemetry.demods_1090 > 0) {
        cJSON_AddNumberToObject(root, "demods_1090", telemetry.demods_1090);
    }
    if (telemetry.raw_squitter_frames > 0) {
        cJSON_AddNumberToObject(root, "raw_squitter_frames", telemetry.raw_squitter_frames);
    }
    if (telemetry.valid_squitter_frames > 0) {
        cJSON_AddNumberToObject(root, "valid_squitter_frames", telemetry.valid_squitter_frames);
    }
    if (telemetry.raw_extended_squitter > 0) {
        cJSON_AddNumberToObject(root, "raw_extended_squitter", telemetry.raw_extended_squitter);
    }
    if (telemetry.valid_extended_squitter > 0) {
        cJSON_AddNumberToObject(root, "valid_extended_squitter", telemetry.valid_extended_squitter);
    }

    char* json_str = cJSON_PrintUnformatted(root);
    std::string result(json_str);

    cJSON_free(json_str);
    cJSON_Delete(root);

    return result;
}

std::string MQTTClient::SerializeGPSJSON(const GPS& gps) const {
    cJSON* root = cJSON_CreateObject();

    cJSON_AddNumberToObject(root, "lat", gps.lat);
    cJSON_AddNumberToObject(root, "lon", gps.lon);
    cJSON_AddNumberToObject(root, "alt_m", gps.alt_m);
    cJSON_AddNumberToObject(root, "fix", gps.fix);
    cJSON_AddNumberToObject(root, "sats", gps.sats);
    cJSON_AddNumberToObject(root, "hdop", gps.hdop);
    cJSON_AddNumberToObject(root, "ts", gps.ts);

    char* json_str = cJSON_PrintUnformatted(root);
    std::string result(json_str);

    cJSON_free(json_str);
    cJSON_Delete(root);

    return result;
}

// Binary Serialization (compact format as specified)
std::vector<uint8_t> MQTTClient::SerializeStatusBinary(const AircraftStatus& status) const {
    // Fixed 31 bytes for status message
    std::vector<uint8_t> data(31, 0);

    size_t offset = 0;

    // Byte 0: Message type (0 = status)
    data[offset++] = 0;

    // Byte 1: Band (1=1090, 2=UAT)
    data[offset++] = status.band;

    // Bytes 2-4: ICAO address (24 bits)
    uint32_t icao = 0;
    const char* icao_cstr = status.icao.c_str();
    char* icao_end = nullptr;
    unsigned long icao_parsed = std::strtoul(icao_cstr, &icao_end, 16);
    if (icao_end != icao_cstr && *icao_end == '\0') {
        icao = static_cast<uint32_t>(icao_parsed);
    } else {
        icao = 0;  // Default if parsing fails
    }
    data[offset++] = (icao >> 16) & 0xFF;
    data[offset++] = (icao >> 8) & 0xFF;
    data[offset++] = icao & 0xFF;

    // Byte 5: RSSI (placeholder, -50 dBm as signed int8)
    data[offset++] = static_cast<int8_t>(-50);

    // Bytes 6-9: Timestamp (ms, 32-bit)
    uint32_t t_ms = status.t_ms & 0xFFFFFFFF;
    data[offset++] = (t_ms >> 24) & 0xFF;
    data[offset++] = (t_ms >> 16) & 0xFF;
    data[offset++] = (t_ms >> 8) & 0xFF;
    data[offset++] = t_ms & 0xFF;

    // Bytes 10-13: Latitude (1e-5 degrees, signed 32-bit)
    int32_t lat = static_cast<int32_t>(status.lat * 100000);
    data[offset++] = (lat >> 24) & 0xFF;
    data[offset++] = (lat >> 16) & 0xFF;
    data[offset++] = (lat >> 8) & 0xFF;
    data[offset++] = lat & 0xFF;

    // Bytes 14-17: Longitude (1e-5 degrees, signed 32-bit)
    int32_t lon = static_cast<int32_t>(status.lon * 100000);
    data[offset++] = (lon >> 24) & 0xFF;
    data[offset++] = (lon >> 16) & 0xFF;
    data[offset++] = (lon >> 8) & 0xFF;
    data[offset++] = lon & 0xFF;

    // Bytes 18-19: Altitude (units of 25ft, unsigned 16-bit)
    uint16_t alt_25ft = static_cast<uint16_t>(std::max<int32_t>(0, status.alt_ft) / 25);
    data[offset++] = (alt_25ft >> 8) & 0xFF;
    data[offset++] = alt_25ft & 0xFF;

    // Byte 20: Heading (units of 360/256 degrees)
    data[offset++] = static_cast<uint8_t>((status.hdg_deg * 256) / 360);

    // Byte 21: Speed (knots, capped at 255)
    data[offset++] = std::min(static_cast<uint8_t>(status.spd_kts), uint8_t(255));

    // Byte 22: Vertical rate (units of 64 fpm, signed)
    int8_t vr_64fpm = static_cast<int8_t>(status.vr_fpm / 64);
    data[offset++] = static_cast<uint8_t>(vr_64fpm);

    // Byte 23: Flags (bit 0 = on_ground) and Category
    uint8_t flags_cat = (status.on_ground ? 1 : 0);
    // Parse category if it's a string like "A3"
    if (!status.category.empty() && status.category.length() >= 2) {
        uint8_t cat_high = (status.category[0] - 'A') & 0x0F;
        uint8_t cat_low = (status.category[1] - '0') & 0x0F;
        flags_cat |= ((cat_high << 4) | cat_low) << 1;  // Shift left by 1 to leave bit 0 for on_ground
    }
    data[offset++] = flags_cat;

    // Bytes 24-30: Callsign (7 chars, space-padded)
    for (size_t i = 0; i < 7; i++) {
        if (i < status.callsign.length()) {
            data[offset + i] = status.callsign[i];
        } else {
            data[offset + i] = ' ';  // Space padding
        }
    }

    return data;
}

std::vector<uint8_t> MQTTClient::SerializeTelemetryBinary(const Telemetry& telemetry) const {
    // Extended to 28 bytes to include decoder statistics
    std::vector<uint8_t> data(28, 0);

    // Byte 0: Message type (1 = telemetry)
    data[0] = 1;

    // Bytes 1-4: Uptime (seconds, 32-bit)
    uint32_t uptime = telemetry.uptime_sec;
    data[1] = (uptime >> 24) & 0xFF;
    data[2] = (uptime >> 16) & 0xFF;
    data[3] = (uptime >> 8) & 0xFF;
    data[4] = uptime & 0xFF;

    // Bytes 5-6: Messages received (16-bit)
    uint16_t msgs_rx = std::min(telemetry.msgs_rx, uint32_t(0xFFFF));
    data[5] = (msgs_rx >> 8) & 0xFF;
    data[6] = msgs_rx & 0xFF;

    // Bytes 7-8: Messages transmitted (16-bit)
    uint16_t msgs_tx = std::min(telemetry.msgs_tx, uint32_t(0xFFFF));
    data[7] = (msgs_tx >> 8) & 0xFF;
    data[8] = msgs_tx & 0xFF;

    // Byte 9: CPU temperature (Celsius, signed)
    data[9] = static_cast<uint8_t>(telemetry.cpu_temp_c);

    // Bytes 10-11: Free memory (KB, 16-bit)
    uint16_t mem_kb = std::min(telemetry.mem_free_kb, uint32_t(0xFFFF));
    data[10] = (mem_kb >> 8) & 0xFF;
    data[11] = mem_kb & 0xFF;

    // Byte 12: Noise floor (dBm + 128)
    data[12] = telemetry.noise_floor_dbm + 128;

    // Byte 13: Status flags
    uint8_t flags = 0;
    if (telemetry.rx_1090) flags |= 0x01;
    if (telemetry.rx_978) flags |= 0x02;
    if (telemetry.wifi) flags |= 0x04;
    if (telemetry.mqtt) flags |= 0x08;
    data[13] = flags;

    // Bytes 14-15: Demodulations per second (16-bit)
    uint16_t demods = std::min(telemetry.demods_1090, uint32_t(0xFFFF));
    data[14] = (demods >> 8) & 0xFF;
    data[15] = demods & 0xFF;

    // Bytes 16-17: Raw squitter frames per second (16-bit)
    uint16_t raw_sq = std::min(telemetry.raw_squitter_frames, uint32_t(0xFFFF));
    data[16] = (raw_sq >> 8) & 0xFF;
    data[17] = raw_sq & 0xFF;

    // Bytes 18-19: Valid squitter frames per second (16-bit)
    uint16_t valid_sq = std::min(telemetry.valid_squitter_frames, uint32_t(0xFFFF));
    data[18] = (valid_sq >> 8) & 0xFF;
    data[19] = valid_sq & 0xFF;

    // Bytes 20-21: Raw extended squitter per second (16-bit)
    uint16_t raw_ext = std::min(telemetry.raw_extended_squitter, uint32_t(0xFFFF));
    data[20] = (raw_ext >> 8) & 0xFF;
    data[21] = raw_ext & 0xFF;

    // Bytes 22-23: Valid extended squitter per second (16-bit)
    uint16_t valid_ext = std::min(telemetry.valid_extended_squitter, uint32_t(0xFFFF));
    data[22] = (valid_ext >> 8) & 0xFF;
    data[23] = valid_ext & 0xFF;

    // Bytes 24-25: Total messages per second across all feeds (16-bit)
    uint16_t mps = std::min(telemetry.mps_total, uint32_t(0xFFFF));
    data[24] = (mps >> 8) & 0xFF;
    data[25] = mps & 0xFF;

    // Bytes 26-27: Reserved for future use
    data[26] = 0;
    data[27] = 0;

    return data;
}

std::vector<uint8_t> MQTTClient::SerializeGPSBinary(const GPS& gps) const {
    // Fixed 15 bytes for GPS
    std::vector<uint8_t> data(15, 0);

    // Byte 0: Message type (2 = GPS)
    data[0] = 2;

    // Bytes 1-4: Latitude (1e-5 degrees, signed 32-bit)
    int32_t lat = static_cast<int32_t>(gps.lat * 100000);
    data[1] = (lat >> 24) & 0xFF;
    data[2] = (lat >> 16) & 0xFF;
    data[3] = (lat >> 8) & 0xFF;
    data[4] = lat & 0xFF;

    // Bytes 5-8: Longitude (1e-5 degrees, signed 32-bit)
    int32_t lon = static_cast<int32_t>(gps.lon * 100000);
    data[5] = (lon >> 24) & 0xFF;
    data[6] = (lon >> 16) & 0xFF;
    data[7] = (lon >> 8) & 0xFF;
    data[8] = lon & 0xFF;

    // Bytes 9-10: Altitude (meters, signed 16-bit)
    int16_t alt_m = static_cast<int16_t>(gps.alt_m);
    data[9] = (alt_m >> 8) & 0xFF;
    data[10] = alt_m & 0xFF;

    // Byte 11: Fix type (0=none, 1=2D, 2=3D)
    data[11] = gps.fix;

    // Byte 12: Satellites
    data[12] = std::min(gps.sats, uint32_t(255));

    // Byte 13: HDOP (scaled by 10)
    data[13] = static_cast<uint8_t>(std::min(gps.hdop * 10, 255.0f));

    // Byte 14: Reserved
    data[14] = 0;

    return data;
}

// Optimized buffer-based serialization (no heap allocations)
size_t MQTTClient::SerializeStatusJSONToBuffer(const AircraftStatus& status, char* buf, size_t buf_size) const {
    int written = snprintf(buf, buf_size,
        "{\"icao\":\"%s\",\"band\":%d", status.icao.c_str(), status.band);

    if (!status.callsign.empty() && written < buf_size) {
        written += snprintf(buf + written, buf_size - written,
            ",\"call\":\"%s\"", status.callsign.c_str());
    }

    if ((status.lat != 0 || status.lon != 0) && written < buf_size) {
        written += snprintf(buf + written, buf_size - written,
            ",\"lat\":%.5f,\"lon\":%.5f", status.lat, status.lon);
    }

    if (status.alt_ft != 0 && written < buf_size) {
        written += snprintf(buf + written, buf_size - written,
            ",\"alt_ft\":%ld", (long)status.alt_ft);
    }

    if (status.hdg_deg > 0 && written < buf_size) {
        written += snprintf(buf + written, buf_size - written,
            ",\"hdg\":%.1f", status.hdg_deg);
    }

    if (status.spd_kts > 0 && written < buf_size) {
        written += snprintf(buf + written, buf_size - written,
            ",\"spd\":%.1f", status.spd_kts);
    }

    if (written < buf_size) {
        written += snprintf(buf + written, buf_size - written,
            ",\"gnd\":%d,\"t\":%llu}",
            status.on_ground ? 1 : 0, (unsigned long long)status.t_ms);
    }

    return written;
}

size_t MQTTClient::SerializeStatusBinaryToBuffer(const AircraftStatus& status, uint8_t* buf, size_t buf_size) const {
    if (buf_size < 31) return 0;  // Need at least 31 bytes

    size_t pos = 0;
    buf[pos++] = 0;  // Message type
    buf[pos++] = status.band;

    // ICAO (3 bytes)
    unsigned long icao_long = 0;
    sscanf(status.icao.c_str(), "%06lX", &icao_long);
    uint32_t icao = (uint32_t)icao_long;
    buf[pos++] = (icao >> 16) & 0xFF;
    buf[pos++] = (icao >> 8) & 0xFF;
    buf[pos++] = icao & 0xFF;

    // RSSI placeholder
    buf[pos++] = 0xC8;  // -50 dBm

    // Timestamp (4 bytes)
    uint32_t t_ms = status.t_ms & 0xFFFFFFFF;
    buf[pos++] = (t_ms >> 24) & 0xFF;
    buf[pos++] = (t_ms >> 16) & 0xFF;
    buf[pos++] = (t_ms >> 8) & 0xFF;
    buf[pos++] = t_ms & 0xFF;

    // Lat/Lon (8 bytes)
    int32_t lat = (int32_t)(status.lat * 100000);
    int32_t lon = (int32_t)(status.lon * 100000);
    memcpy(buf + pos, &lat, 4); pos += 4;
    memcpy(buf + pos, &lon, 4); pos += 4;

    // Altitude (2 bytes)
    uint16_t alt_25ft = (status.alt_ft > 0) ? (status.alt_ft / 25) : 0;
    buf[pos++] = (alt_25ft >> 8) & 0xFF;
    buf[pos++] = alt_25ft & 0xFF;

    // Heading, speed, vrate, flags
    buf[pos++] = (uint8_t)((status.hdg_deg * 256) / 360);
    buf[pos++] = (status.spd_kts > 255) ? 255 : (uint8_t)status.spd_kts;
    buf[pos++] = (uint8_t)((status.vr_fpm / 64) + 128);
    buf[pos++] = status.on_ground ? 1 : 0;

    // Callsign (7 bytes, space padded)
    for (size_t i = 0; i < 7; i++) {
        buf[pos++] = (i < status.callsign.length()) ? status.callsign[i] : ' ';
    }

    return pos;
}

size_t MQTTClient::SerializeTelemetryJSONToBuffer(const Telemetry& telemetry, char* buf, size_t buf_size) const {
    // Use shortened keys to save space but include all fields
    int written = snprintf(buf, buf_size,
        "{\"up\":%lu,\"rx\":%lu,\"tx\":%lu,\"cpu\":%ld,\"mem\":%lu,"
        "\"r1090\":%s,\"r978\":%s,\"wifi\":%s,\"mqtt\":%s,"
        "\"fw\":\"%d.%d.%d\","
        "\"ota\":%s",
        (unsigned long)telemetry.uptime_sec,
        (unsigned long)telemetry.msgs_rx,
        (unsigned long)telemetry.msgs_tx,
        (long)telemetry.cpu_temp_c,
        (unsigned long)telemetry.mem_free_kb,
        telemetry.rx_1090 ? "true" : "false",
        telemetry.rx_978 ? "true" : "false",
        telemetry.wifi ? "true" : "false",
        telemetry.mqtt ? "true" : "false",
        object_dictionary.kFirmwareVersionMajor,
        object_dictionary.kFirmwareVersionMinor,
        object_dictionary.kFirmwareVersionPatch,
        config_.ota_enabled ? "true" : "false"
        );

    // Add optional fields if present and buffer space allows
    if (telemetry.mps_total > 0 && written < buf_size - 20) {
        written += snprintf(buf + written, buf_size - written,
            ",\"mps\":%lu", (unsigned long)telemetry.mps_total);
    }

    // Add decoder stats if present
    if (telemetry.demods_1090 > 0 && written < buf_size - 30) {
        written += snprintf(buf + written, buf_size - written,
            ",\"d1090\":%lu", (unsigned long)telemetry.demods_1090);
    }
    if (telemetry.raw_squitter_frames > 0 && written < buf_size - 30) {
        written += snprintf(buf + written, buf_size - written,
            ",\"rsq\":%lu", (unsigned long)telemetry.raw_squitter_frames);
    }
    if (telemetry.valid_squitter_frames > 0 && written < buf_size - 30) {
        written += snprintf(buf + written, buf_size - written,
            ",\"vsq\":%lu", (unsigned long)telemetry.valid_squitter_frames);
    }
    if (telemetry.raw_extended_squitter > 0 && written < buf_size - 30) {
        written += snprintf(buf + written, buf_size - written,
            ",\"res\":%lu", (unsigned long)telemetry.raw_extended_squitter);
    }
    if (telemetry.valid_extended_squitter > 0 && written < buf_size - 30) {
        written += snprintf(buf + written, buf_size - written,
            ",\"ves\":%lu", (unsigned long)telemetry.valid_extended_squitter);
    }

    // Close JSON object
    if (written < buf_size - 1) {
        written += snprintf(buf + written, buf_size - written, "}");
    }

    return written;
}

size_t MQTTClient::SerializeTelemetryBinaryToBuffer(const Telemetry& telemetry, uint8_t* buf, size_t buf_size) const {
    if (buf_size < 20) return 0;

    size_t pos = 0;
    buf[pos++] = 0x02;  // TELEMETRY message type

    // Pack data efficiently
    memcpy(buf + pos, &telemetry.uptime_sec, 4); pos += 4;
    memcpy(buf + pos, &telemetry.msgs_rx, 4); pos += 4;
    memcpy(buf + pos, &telemetry.msgs_tx, 4); pos += 4;

    int16_t temp = telemetry.cpu_temp_c;
    memcpy(buf + pos, &temp, 2); pos += 2;

    uint16_t mem = telemetry.mem_free_kb / 1024;  // Convert to MB
    memcpy(buf + pos, &mem, 2); pos += 2;

    // Firmware version (3 bytes)
    buf[pos++] = object_dictionary.kFirmwareVersionMajor;
    buf[pos++] = object_dictionary.kFirmwareVersionMinor;
    buf[pos++] = object_dictionary.kFirmwareVersionPatch;

    return pos;
}

size_t MQTTClient::SerializeGPSJSONToBuffer(const GPS& gps, char* buf, size_t buf_size) const {
    return snprintf(buf, buf_size,
        "{\"lat\":%.6f,\"lon\":%.6f,\"alt\":%.1f,\"fix\":%d,\"sats\":%lu}",
        gps.lat, gps.lon, gps.alt_m, gps.fix, (unsigned long)gps.sats);
}

size_t MQTTClient::SerializeGPSBinaryToBuffer(const GPS& gps, uint8_t* buf, size_t buf_size) const {
    if (buf_size < 15) return 0;

    size_t pos = 0;
    buf[pos++] = 0x03;  // GPS message type

    // Lat/Lon (8 bytes)
    int32_t lat = (int32_t)(gps.lat * 1000000);
    int32_t lon = (int32_t)(gps.lon * 1000000);
    memcpy(buf + pos, &lat, 4); pos += 4;
    memcpy(buf + pos, &lon, 4); pos += 4;

    // Altitude (2 bytes)
    int16_t alt = (int16_t)gps.alt_m;
    memcpy(buf + pos, &alt, 2); pos += 2;

    // Fix, sats, HDOP
    buf[pos++] = gps.fix;
    buf[pos++] = (gps.sats > 255) ? 255 : gps.sats;
    buf[pos++] = (uint8_t)(gps.hdop * 10);

    return pos;
}

}  // namespace MQTT
