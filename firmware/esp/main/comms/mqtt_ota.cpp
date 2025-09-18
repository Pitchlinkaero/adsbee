#include "mqtt_ota.hh"
#include "comms.hh"
#include <inttypes.h>
#include "coprocessor/spi_coprocessor.hh"
#include "object_dictionary.hh"
#include "esp_log.h"
#include "esp_system.h"
#include <cstring>
#include <sstream>

extern SPICoprocessor pico;
extern SettingsManager settings_manager;

MQTTOTAHandler::MQTTOTAHandler(const std::string& device_id, uint16_t feed_index)
    : device_id_(device_id),
      feed_index_(feed_index),
      state_(OTAState::IDLE),
      chunks_received_(0),
      total_chunks_(0),
      bytes_received_(0),
      total_bytes_(0),
      last_chunk_timestamp_ms_(0),
      ota_start_timestamp_ms_(0),
      timeout_ms_(30000),
      ota_partition_(nullptr),
      ota_handle_(0) {
    memset(chunk_buffer_, 0, kMaxChunkSize);
}

MQTTOTAHandler::~MQTTOTAHandler() {
    if (state_ != OTAState::IDLE) {
        AbortOTA();
    }
}

bool MQTTOTAHandler::Initialize(MQTT::MQTTClient* mqtt_client) {
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

bool MQTTOTAHandler::HandleManifest(const Manifest& manifest) {
    if (state_ != OTAState::IDLE) {
        CONSOLE_WARNING("MQTTOTAHandler::HandleManifest",
                        "Ignoring manifest - OTA already in progress");
        return false;
    }

    manifest_ = manifest;

    // Validate manifest
    if (manifest_.size == 0 || manifest_.total_chunks == 0 || manifest_.chunk_size == 0) {
        CONSOLE_ERROR("MQTTOTAHandler::HandleManifest", "Invalid manifest parameters");
        state_ = OTAState::ERROR;
        PublishStatus();
        return false;
    }

    state_ = OTAState::MANIFEST_RECEIVED;
    PublishStatus();

    CONSOLE_INFO("MQTTOTAHandler::HandleManifest",
                 "Manifest received: version=%s, size=%" PRIu32 ", chunks=%" PRIu32,
                 manifest_.version.c_str(), manifest_.size, manifest_.total_chunks);
    return true;
}

bool MQTTOTAHandler::HandleCommand(const std::string& command) {
    if (command == "START") {
        return StartOTA();
    } else if (command == "PAUSE") {
        return PauseOTA();
    } else if (command == "RESUME") {
        return ResumeOTA();
    } else if (command == "ABORT") {
        return AbortOTA();
    } else if (command == "VERIFY") {
        return VerifyOTA();
    } else if (command == "BOOT") {
        return BootNewFirmware();
    }
    CONSOLE_WARNING("MQTTOTAHandler::HandleCommand", "Unknown command: %s", command.c_str());
    return false;
}

bool MQTTOTAHandler::HandleChunk(uint32_t index, const uint8_t* data, size_t len) {
    if (state_ != OTAState::DOWNLOADING) {
        CONSOLE_WARNING("MQTTOTAHandler::HandleChunk",
                        "Ignoring chunk %" PRIu32 " - not in download state", index);
        return false;
    }

    if (index >= total_chunks_) {
        CONSOLE_ERROR("MQTTOTAHandler::HandleChunk",
                      "Invalid chunk index %" PRIu32 " (max %" PRIu32 ")", index, total_chunks_ - 1);
        PublishAck(index, false);
        return false;
    }

    if (received_chunks_[index]) {
        CONSOLE_INFO("MQTTOTAHandler::HandleChunk",
                     "Chunk %" PRIu32 " already received, ignoring", index);
        PublishAck(index, true);  // Still ACK it
        return true;
    }

    // Parse chunk header
    if (len < sizeof(ChunkHeader)) {
        CONSOLE_ERROR("MQTTOTAHandler::HandleChunk",
                      "Chunk too small for header: %zu bytes", len);
        PublishAck(index, false);
        return false;
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
                      "Chunk size mismatch: expected %" PRIu16 ", got %zu",
                      header.chunk_size, chunk_data_len);
        PublishAck(index, false);
        return false;
    }

    // Verify CRC
    uint32_t calculated_crc = CalculateCRC32(chunk_data, chunk_data_len);
    if (calculated_crc != header.crc32) {
        CONSOLE_ERROR("MQTTOTAHandler::HandleChunk",
                      "CRC mismatch for chunk %" PRIu32 ": expected 0x%08" PRIx32 ", got 0x%08" PRIx32,
                      index, header.crc32, calculated_crc);
        PublishAck(index, false);
        return false;
    }

    // Calculate flash offset
    uint32_t flash_offset = index * manifest_.chunk_size;

    // Write to flash via AT command
    if (!WriteChunkToFlash(flash_offset, chunk_data, chunk_data_len, header.crc32)) {
        CONSOLE_ERROR("MQTTOTAHandler::HandleChunk",
                      "Failed to write chunk %" PRIu32 " to flash", index);
        PublishAck(index, false);
        return false;
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
    return true;
}

bool MQTTOTAHandler::StartOTA() {
    if (state_ != OTAState::MANIFEST_RECEIVED) {
        CONSOLE_ERROR("MQTTOTAHandler::StartOTA",
                      "Cannot start OTA - no manifest or already in progress");
        return false;
    }

    CONSOLE_INFO("MQTTOTAHandler::StartOTA", "Starting OTA update");

    // Initialize chunk tracking
    total_chunks_ = manifest_.total_chunks;
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

    CONSOLE_INFO("MQTTOTAHandler::StartOTA", "Ready to receive %" PRIu32 " chunks", total_chunks_);
    return true;
}

bool MQTTOTAHandler::AbortOTA() {
    CONSOLE_INFO("MQTTOTAHandler::AbortOTA", "Aborting OTA update");

    state_ = OTAState::IDLE;

    // Clear session
    current_session_id_.clear();
    manifest_ = Manifest();
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
    // For ESP32, we handle the flash directly using ESP-IDF OTA APIs
    // Get the next OTA partition
    const esp_partition_t* ota_partition = esp_ota_get_next_update_partition(NULL);
    if (!ota_partition) {
        CONSOLE_ERROR("MQTTOTAHandler::EraseFlashPartition",
                      "Failed to get OTA partition");
        return false;
    }

    // Erase the partition
    esp_err_t err = esp_partition_erase_range(ota_partition, 0, manifest_.size);
    if (err != ESP_OK) {
        CONSOLE_ERROR("MQTTOTAHandler::EraseFlashPartition",
                      "Failed to erase partition: %s", esp_err_to_name(err));
        return false;
    }

    // Store partition for later use
    ota_partition_ = ota_partition;

    CONSOLE_INFO("MQTTOTAHandler::EraseFlashPartition",
                 "Erased %" PRIu32 " bytes in partition %s",
                 manifest_.size, ota_partition->label);
    return true;
}

bool MQTTOTAHandler::WriteChunkToFlash(uint32_t offset, const uint8_t* data, size_t len, uint32_t crc) {
    // Validate we have an OTA partition
    if (!ota_partition_) {
        CONSOLE_ERROR("MQTTOTAHandler::WriteChunkToFlash",
                      "No OTA partition selected");
        return false;
    }

    // Write data to partition
    esp_err_t err = esp_partition_write(ota_partition_, offset, data, len);
    if (err != ESP_OK) {
        CONSOLE_ERROR("MQTTOTAHandler::WriteChunkToFlash",
                      "Failed to write to partition at offset 0x%08" PRIx32 ": %s",
                      offset, esp_err_to_name(err));
        return false;
    }

    return true;
}

bool MQTTOTAHandler::VerifyFlashPartition() {
    if (!ota_partition_) {
        CONSOLE_ERROR("MQTTOTAHandler::VerifyFlashPartition",
                      "No OTA partition selected");
        return false;
    }

    // Calculate SHA256 of written data
    uint8_t calculated_sha[32];
    esp_partition_get_sha256(ota_partition_, calculated_sha);

    // Convert to hex string for comparison
    char calculated_hex[65];
    for (int i = 0; i < 32; i++) {
        sprintf(&calculated_hex[i*2], "%02x", calculated_sha[i]);
    }
    calculated_hex[64] = '\0';

    // Compare with expected
    if (strcasecmp(calculated_hex, manifest_.sha256.c_str()) != 0) {
        CONSOLE_ERROR("MQTTOTAHandler::VerifyFlashPartition",
                      "SHA256 mismatch: expected %s, got %s",
                      manifest_.sha256.c_str(), calculated_hex);
        return false;
    }

    CONSOLE_INFO("MQTTOTAHandler::VerifyFlashPartition",
                 "Firmware verified successfully");
    return true;
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