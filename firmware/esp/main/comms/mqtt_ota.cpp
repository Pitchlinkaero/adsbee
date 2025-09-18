#include "mqtt_ota.hh"
#include "comms.hh"
#include "coprocessor/spi_coprocessor.hh"
#include "esp_log.h"
#include "esp_system.h"
#include <cstring>
#include <sstream>

extern SPICoprocessor pico;
extern SettingsManager settings_manager;

MQTTOTAHandler::MQTTOTAHandler(uint16_t feed_index)
    : feed_index_(feed_index),
      mqtt_client_(nullptr),
      state_(OTAState::IDLE),
      chunks_received_(0),
      total_chunks_(0),
      bytes_received_(0),
      total_bytes_(0),
      last_chunk_timestamp_ms_(0),
      ota_start_timestamp_ms_(0),
      timeout_ms_(30000) {
    memset(chunk_buffer_, 0, kMaxChunkSize);
}

MQTTOTAHandler::~MQTTOTAHandler() {
    if (state_ != OTAState::IDLE) {
        AbortOTA();
    }
}

bool MQTTOTAHandler::Initialize(MQTTClient* mqtt_client) {
    if (!mqtt_client || !mqtt_client->IsConnected()) {
        CONSOLE_ERROR("MQTTOTAHandler::Initialize", "MQTT client not connected");
        return false;
    }

    mqtt_client_ = mqtt_client;

    // Subscribe to OTA control topics
    std::string device_id = settings_manager.settings.mqtt_device_id;

    // Subscribe to manifest topic
    std::string manifest_topic = device_id + "/ota/control/manifest";
    if (!mqtt_client_->Subscribe(manifest_topic, 1)) {
        CONSOLE_ERROR("MQTTOTAHandler::Initialize", "Failed to subscribe to manifest topic");
        return false;
    }

    // Subscribe to control commands
    std::string control_topic = device_id + "/ota/control/command";
    if (!mqtt_client_->Subscribe(control_topic, 1)) {
        CONSOLE_ERROR("MQTTOTAHandler::Initialize", "Failed to subscribe to control topic");
        return false;
    }

    CONSOLE_INFO("MQTTOTAHandler::Initialize", "OTA handler initialized for feed %d", feed_index_);

    // Publish initial status
    PublishStatus();

    return true;
}

void MQTTOTAHandler::HandleManifest(const std::string& payload) {
    CONSOLE_INFO("MQTTOTAHandler::HandleManifest", "Received manifest: %d bytes", payload.length());

    if (state_ != OTAState::IDLE) {
        CONSOLE_WARNING("MQTTOTAHandler::HandleManifest",
                        "Ignoring manifest - OTA already in progress");
        return;
    }

    if (!ParseManifest(payload)) {
        CONSOLE_ERROR("MQTTOTAHandler::HandleManifest", "Failed to parse manifest");
        state_ = OTAState::ERROR;
        PublishStatus();
        return;
    }

    // Validate manifest
    if (manifest_.size == 0 || manifest_.chunks == 0 || manifest_.chunk_size == 0) {
        CONSOLE_ERROR("MQTTOTAHandler::HandleManifest", "Invalid manifest parameters");
        state_ = OTAState::ERROR;
        PublishStatus();
        return;
    }

    // Check version (simple string comparison for now)
    // TODO: Implement proper semantic version comparison

    state_ = OTAState::MANIFEST_RECEIVED;
    PublishStatus();

    CONSOLE_INFO("MQTTOTAHandler::HandleManifest",
                 "Manifest received: version=%s, size=%lu, chunks=%lu",
                 manifest_.version.c_str(), manifest_.size, manifest_.chunks);
}

void MQTTOTAHandler::HandleControl(const std::string& payload) {
    CONSOLE_INFO("MQTTOTAHandler::HandleControl", "Received control command");

    // Parse control command JSON
    // Expected format: {"command": "START|PAUSE|RESUME|ABORT|VERIFY|BOOT", "session_id": "..."}

    // Simple parsing for command (TODO: Use proper JSON parser)
    std::string command;
    if (payload.find("\"START\"") != std::string::npos) {
        StartOTA();
    } else if (payload.find("\"PAUSE\"") != std::string::npos) {
        PauseOTA();
    } else if (payload.find("\"RESUME\"") != std::string::npos) {
        ResumeOTA();
    } else if (payload.find("\"ABORT\"") != std::string::npos) {
        AbortOTA();
    } else if (payload.find("\"VERIFY\"") != std::string::npos) {
        VerifyOTA();
    } else if (payload.find("\"BOOT\"") != std::string::npos) {
        BootNewFirmware();
    }
}

void MQTTOTAHandler::HandleChunk(uint32_t index, const uint8_t* data, size_t len) {
    if (state_ != OTAState::DOWNLOADING) {
        CONSOLE_WARNING("MQTTOTAHandler::HandleChunk",
                        "Ignoring chunk %lu - not in download state", index);
        return;
    }

    if (index >= total_chunks_) {
        CONSOLE_ERROR("MQTTOTAHandler::HandleChunk",
                      "Invalid chunk index %lu (max %lu)", index, total_chunks_ - 1);
        PublishAck(index, false);
        return;
    }

    if (received_chunks_[index]) {
        CONSOLE_INFO("MQTTOTAHandler::HandleChunk",
                     "Chunk %lu already received, ignoring", index);
        PublishAck(index, true);  // Still ACK it
        return;
    }

    // Parse chunk header
    if (len < sizeof(ChunkHeader)) {
        CONSOLE_ERROR("MQTTOTAHandler::HandleChunk",
                      "Chunk too small for header: %d bytes", len);
        PublishAck(index, false);
        return;
    }

    ChunkHeader header;
    memcpy(&header, data, sizeof(ChunkHeader));

    // Validate session ID
    // TODO: Check session ID matches current session

    // Extract chunk data
    const uint8_t* chunk_data = data + sizeof(ChunkHeader);
    size_t chunk_data_len = len - sizeof(ChunkHeader);

    if (chunk_data_len != header.chunk_size) {
        CONSOLE_ERROR("MQTTOTAHandler::HandleChunk",
                      "Chunk size mismatch: expected %d, got %d",
                      header.chunk_size, chunk_data_len);
        PublishAck(index, false);
        return;
    }

    // Verify CRC
    uint32_t calculated_crc = CalculateCRC32(chunk_data, chunk_data_len);
    if (calculated_crc != header.crc32) {
        CONSOLE_ERROR("MQTTOTAHandler::HandleChunk",
                      "CRC mismatch for chunk %lu: expected 0x%08x, got 0x%08x",
                      index, header.crc32, calculated_crc);
        PublishAck(index, false);
        return;
    }

    // Calculate flash offset
    uint32_t flash_offset = index * manifest_.chunk_size;

    // Write to flash via AT command
    if (!WriteChunkToFlash(flash_offset, chunk_data, chunk_data_len, header.crc32)) {
        CONSOLE_ERROR("MQTTOTAHandler::HandleChunk",
                      "Failed to write chunk %lu to flash", index);
        PublishAck(index, false);
        return;
    }

    // Mark chunk as received
    received_chunks_[index] = true;
    chunks_received_++;
    bytes_received_ += chunk_data_len;
    last_chunk_timestamp_ms_ = esp_log_timestamp();

    // Send ACK
    PublishAck(index, true);

    // Update progress
    PublishProgress();

    // Check if all chunks received
    if (chunks_received_ == total_chunks_) {
        CONSOLE_INFO("MQTTOTAHandler::HandleChunk",
                     "All chunks received, starting verification");
        state_ = OTAState::VERIFYING;
        PublishStatus();
        VerifyOTA();
    } else if (chunks_received_ % 10 == 0) {
        // Every 10 chunks, check for missing ones
        RequestMissingChunks();
    }
}

bool MQTTOTAHandler::StartOTA() {
    if (state_ != OTAState::MANIFEST_RECEIVED) {
        CONSOLE_ERROR("MQTTOTAHandler::StartOTA",
                      "Cannot start OTA - no manifest or already in progress");
        return false;
    }

    CONSOLE_INFO("MQTTOTAHandler::StartOTA", "Starting OTA update");

    // Initialize chunk tracking
    total_chunks_ = manifest_.chunks;
    total_bytes_ = manifest_.size;
    chunks_received_ = 0;
    bytes_received_ = 0;
    received_chunks_.clear();
    received_chunks_.resize(total_chunks_, false);

    ota_start_timestamp_ms_ = esp_log_timestamp();
    last_chunk_timestamp_ms_ = ota_start_timestamp_ms_;

    // Erase flash partition
    state_ = OTAState::ERASING;
    PublishStatus();

    if (!EraseFlashPartition()) {
        CONSOLE_ERROR("MQTTOTAHandler::StartOTA", "Failed to erase flash partition");
        state_ = OTAState::ERROR;
        PublishStatus();
        return false;
    }

    // Subscribe to chunk topics
    std::string device_id = settings_manager.settings.mqtt_device_id;
    for (uint32_t i = 0; i < total_chunks_; i++) {
        std::string chunk_topic = GetChunkTopic(i);
        // Note: This could be optimized with wildcard subscription
        // but for now we'll use a single wildcard for all chunks
    }

    // Subscribe to all chunk topics with wildcard
    std::string all_chunks_topic = device_id + "/ota/data/chunk/+";
    if (!mqtt_client_->Subscribe(all_chunks_topic, 1)) {
        CONSOLE_ERROR("MQTTOTAHandler::StartOTA", "Failed to subscribe to chunk topics");
        state_ = OTAState::ERROR;
        PublishStatus();
        return false;
    }

    state_ = OTAState::DOWNLOADING;
    PublishStatus();

    CONSOLE_INFO("MQTTOTAHandler::StartOTA", "Ready to receive %lu chunks", total_chunks_);
    return true;
}

bool MQTTOTAHandler::AbortOTA() {
    CONSOLE_INFO("MQTTOTAHandler::AbortOTA", "Aborting OTA update");

    state_ = OTAState::IDLE;

    // Clear session
    current_session_id_.clear();
    manifest_ = OTAManifest();
    received_chunks_.clear();
    chunks_received_ = 0;
    total_chunks_ = 0;
    bytes_received_ = 0;
    total_bytes_ = 0;

    PublishStatus();
    return true;
}

bool MQTTOTAHandler::VerifyOTA() {
    if (state_ != OTAState::VERIFYING && state_ != OTAState::DOWNLOADING) {
        CONSOLE_ERROR("MQTTOTAHandler::VerifyOTA", "Not in correct state for verification");
        return false;
    }

    state_ = OTAState::VERIFYING;
    PublishStatus();

    // Send AT+OTA=VERIFY command
    if (!VerifyFlashPartition()) {
        CONSOLE_ERROR("MQTTOTAHandler::VerifyOTA", "Flash partition verification failed");
        state_ = OTAState::ERROR;
        PublishStatus();
        return false;
    }

    state_ = OTAState::READY_TO_BOOT;
    PublishStatus();

    CONSOLE_INFO("MQTTOTAHandler::VerifyOTA", "OTA verification successful");
    return true;
}

bool MQTTOTAHandler::BootNewFirmware() {
    if (state_ != OTAState::READY_TO_BOOT) {
        CONSOLE_ERROR("MQTTOTAHandler::BootNewFirmware", "Not ready to boot");
        return false;
    }

    CONSOLE_INFO("MQTTOTAHandler::BootNewFirmware", "Booting new firmware");

    // Send final status before reboot
    PublishStatus();

    // Send AT+OTA=BOOT command
    // This will trigger a reboot to the new partition
    // TODO: Implement actual AT command sending

    return true;
}

void MQTTOTAHandler::PublishStatus() {
    if (!mqtt_client_ || !mqtt_client_->IsConnected()) {
        return;
    }

    std::string status_json = CreateStatusJson();
    std::string topic = GetStatusTopic();

    mqtt_client_->Publish(topic, status_json, 1, false);
}

void MQTTOTAHandler::PublishProgress() {
    if (!mqtt_client_ || !mqtt_client_->IsConnected()) {
        return;
    }

    std::string progress_json = CreateProgressJson();
    std::string topic = GetProgressTopic();

    mqtt_client_->Publish(topic, progress_json, 0, false);
}

void MQTTOTAHandler::PublishAck(uint32_t chunk_index, bool success) {
    if (!mqtt_client_ || !mqtt_client_->IsConnected()) {
        return;
    }

    std::string ack = success ? "1" : "0";
    std::string topic = GetAckTopic(chunk_index);

    mqtt_client_->Publish(topic, ack, 1, false);
}

float MQTTOTAHandler::GetProgress() const {
    if (total_chunks_ == 0) {
        return 0.0f;
    }
    return (float)chunks_received_ / (float)total_chunks_ * 100.0f;
}

// Topic generation helpers
std::string MQTTOTAHandler::GetControlTopic() const {
    return std::string(settings_manager.settings.mqtt_device_id) + "/ota/control";
}

std::string MQTTOTAHandler::GetStatusTopic() const {
    return std::string(settings_manager.settings.mqtt_device_id) + "/ota/status/state";
}

std::string MQTTOTAHandler::GetProgressTopic() const {
    return std::string(settings_manager.settings.mqtt_device_id) + "/ota/status/progress";
}

std::string MQTTOTAHandler::GetAckTopic(uint32_t index) const {
    return std::string(settings_manager.settings.mqtt_device_id) + "/ota/status/ack/" + std::to_string(index);
}

std::string MQTTOTAHandler::GetChunkTopic(uint32_t index) const {
    return std::string(settings_manager.settings.mqtt_device_id) + "/ota/data/chunk/" + std::to_string(index);
}

// TODO: Implement these functions
bool MQTTOTAHandler::EraseFlashPartition() {
    // Send AT+OTA=ERASE command via pico interface
    return true;  // Placeholder
}

bool MQTTOTAHandler::WriteChunkToFlash(uint32_t offset, const uint8_t* data, size_t len, uint32_t crc) {
    // Send AT+OTA=WRITE command via pico interface
    return true;  // Placeholder
}

bool MQTTOTAHandler::VerifyFlashPartition() {
    // Send AT+OTA=VERIFY command via pico interface
    return true;  // Placeholder
}

void MQTTOTAHandler::RequestMissingChunks() {
    // Publish request for missing chunks
    std::vector<uint32_t> missing = GetMissingChunks();
    if (missing.empty()) {
        return;
    }

    // TODO: Publish list of missing chunks
}

std::vector<uint32_t> MQTTOTAHandler::GetMissingChunks() const {
    std::vector<uint32_t> missing;
    for (uint32_t i = 0; i < total_chunks_; i++) {
        if (!received_chunks_[i]) {
            missing.push_back(i);
        }
    }
    return missing;
}

bool MQTTOTAHandler::ParseManifest(const std::string& json_str) {
    // Simple JSON parsing - TODO: Use proper JSON library
    // Expected format: {"version": "0.8.3", "size": 2097152, "chunks": 512, "chunk_size": 4096, ...}

    // Extract version
    size_t pos = json_str.find("\"version\"");
    if (pos != std::string::npos) {
        pos = json_str.find("\"", pos + 10);
        if (pos != std::string::npos) {
            size_t end = json_str.find("\"", pos + 1);
            manifest_.version = json_str.substr(pos + 1, end - pos - 1);
        }
    }

    // Extract size
    pos = json_str.find("\"size\"");
    if (pos != std::string::npos) {
        pos = json_str.find(":", pos);
        if (pos != std::string::npos) {
            manifest_.size = std::stoul(json_str.substr(pos + 1));
        }
    }

    // Extract chunks
    pos = json_str.find("\"chunks\"");
    if (pos != std::string::npos) {
        pos = json_str.find(":", pos);
        if (pos != std::string::npos) {
            manifest_.chunks = std::stoul(json_str.substr(pos + 1));
        }
    }

    // Extract chunk_size
    pos = json_str.find("\"chunk_size\"");
    if (pos != std::string::npos) {
        pos = json_str.find(":", pos);
        if (pos != std::string::npos) {
            manifest_.chunk_size = std::stoul(json_str.substr(pos + 1));
        }
    }

    return true;
}

std::string MQTTOTAHandler::CreateStatusJson() const {
    std::stringstream json;
    json << "{";
    json << "\"session_id\":\"" << current_session_id_ << "\",";
    json << "\"state\":\"";

    switch (state_) {
        case OTAState::IDLE: json << "IDLE"; break;
        case OTAState::MANIFEST_RECEIVED: json << "MANIFEST_RECEIVED"; break;
        case OTAState::ERASING: json << "ERASING"; break;
        case OTAState::DOWNLOADING: json << "DOWNLOADING"; break;
        case OTAState::VERIFYING: json << "VERIFYING"; break;
        case OTAState::READY_TO_BOOT: json << "READY_TO_BOOT"; break;
        case OTAState::ERROR: json << "ERROR"; break;
    }

    json << "\"";
    if (state_ == OTAState::ERROR) {
        json << ",\"error\":\"OTA failed\"";
    }
    json << "}";

    return json.str();
}

std::string MQTTOTAHandler::CreateProgressJson() const {
    std::stringstream json;
    json << "{";
    json << "\"session_id\":\"" << current_session_id_ << "\",";
    json << "\"chunks_received\":" << chunks_received_ << ",";
    json << "\"total_chunks\":" << total_chunks_ << ",";
    json << "\"bytes_received\":" << bytes_received_ << ",";
    json << "\"total_bytes\":" << total_bytes_ << ",";
    json << "\"percent\":" << GetProgress();
    json << "}";

    return json.str();
}

uint32_t MQTTOTAHandler::CalculateCRC32(const uint8_t* data, size_t len) {
    // TODO: Use actual CRC32 implementation
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
        }
    }
    return ~crc;
}