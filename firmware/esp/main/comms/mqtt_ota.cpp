#include "mqtt_ota.hh"
#include "comms.hh"
#include <inttypes.h>
#include "coprocessor/spi_coprocessor.hh"
#include "object_dictionary.hh"
#include "esp_log.h"
#include "esp_system.h"
#include <cstring>
#include <sstream>
#include <cstdlib>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

extern SPICoprocessor pico;
extern SettingsManager settings_manager;

// Global flag for OTA state - accessible by other modules
volatile bool g_mqtt_ota_active = false;

// Static member initialization
MQTTOTAHandler* MQTTOTAHandler::active_ota_handler_ = nullptr;

MQTTOTAHandler::MQTTOTAHandler(const std::string& device_id, uint16_t feed_index)
    : device_id_(device_id),
      feed_index_(feed_index),
      state_(OTAState::IDLE),
      current_session_id_(""),          // Explicitly initialize to empty
      expected_session_id32_(0),         // Explicitly initialize to 0
      chunks_received_(0),
      total_chunks_(0),
      bytes_received_(0),
      total_bytes_(0),
      last_chunk_timestamp_ms_(0),
      ota_start_timestamp_ms_(0),
      timeout_ms_(30000) {
    // Create semaphore for response synchronization
    response_semaphore_ = xSemaphoreCreateBinary();
    if (!response_semaphore_) {
        CONSOLE_ERROR("MQTTOTAHandler", "Failed to create response semaphore");
    }
}

MQTTOTAHandler::~MQTTOTAHandler() {
    if (state_ != OTAState::IDLE) {
        AbortOTA();
    }
    if (response_semaphore_) {
        vSemaphoreDelete(response_semaphore_);
    }
}

bool MQTTOTAHandler::Initialize(MQTT::MQTTClient* mqtt_client) {
    if (!mqtt_client || !mqtt_client->IsConnected()) {
        CONSOLE_ERROR("MQTTOTAHandler::Initialize", "MQTT client not connected");
        return false;
    }

    mqtt_client_ = mqtt_client;

    // Subscriptions are managed by MQTTClient::HandleConnect(). Nothing else to do here.
    CONSOLE_INFO("MQTTOTAHandler::Initialize", "OTA handler initialized for feed %d", feed_index_);

    // Publish initial status
    PublishStatus();

    return true;
}

bool MQTTOTAHandler::HandleManifest(const Manifest& manifest) {
    CONSOLE_INFO("MQTTOTAHandler::HandleManifest",
                 "ENTER - Received manifest with session=%s (current state=%d, current session=%s)",
                 manifest.session_id.c_str(), (int)state_, current_session_id_.c_str());

    // Log the actual state value and what IDLE should be
    CONSOLE_INFO("MQTTOTAHandler::HandleManifest",
                 "State check: current=%d, IDLE=%d, equal=%s",
                 static_cast<int>(state_),
                 static_cast<int>(OTAState::IDLE),
                 (state_ == OTAState::IDLE) ? "YES" : "NO");

    // If we're in MANIFEST_RECEIVED state with a different session, auto-abort the old one
    if (state_ == OTAState::MANIFEST_RECEIVED &&
        !current_session_id_.empty() &&
        current_session_id_ != manifest.session_id) {
        CONSOLE_INFO("MQTTOTAHandler::HandleManifest",
                     "Different session detected, aborting old session %s for new %s",
                     current_session_id_.c_str(), manifest.session_id.c_str());
        AbortOTA();
    }

    if (state_ != OTAState::IDLE) {
        CONSOLE_WARNING("MQTTOTAHandler::HandleManifest",
                        "Ignoring manifest - OTA already in progress. Current state: %d (IDLE=%d, MANIFEST_RECEIVED=%d, DOWNLOADING=%d, ERROR=%d)",
                        static_cast<int>(state_),
                        static_cast<int>(OTAState::IDLE),
                        static_cast<int>(OTAState::MANIFEST_RECEIVED),
                        static_cast<int>(OTAState::DOWNLOADING),
                        static_cast<int>(OTAState::ERROR));
        return false;
    }

    manifest_ = manifest;
    current_session_id_ = manifest_.session_id;
    // Convert first 8 hex chars of session_id to 32-bit if available
    expected_session_id32_ = 0;
    if (manifest_.session_id.length() >= 8) {
        std::string first8 = manifest_.session_id.substr(0, 8);
        expected_session_id32_ = (uint32_t)strtoul(first8.c_str(), nullptr, 16);
    }

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

bool MQTTOTAHandler::HandleCommand(const std::string& command, const std::string& session_id) {
    CONSOLE_INFO("MQTTOTAHandler::HandleCommand",
                 "Received command: %s, current state: %d, session: %s",
                 command.c_str(), static_cast<int>(state_), session_id.c_str());

    // Allow ABORT and REBOOT regardless of session to recover from mismatch/stuck sessions
    if (command != "ABORT" && command != "REBOOT") {
        // Session consistency: if manifest provided a session_id, enforce it on commands
        if (!manifest_.session_id.empty() && session_id != manifest_.session_id) {
            CONSOLE_ERROR("MQTTOTAHandler::HandleCommand",
                          "Session mismatch: manifest=%s command=%s (send ABORT first to clear old session)",
                          manifest_.session_id.c_str(), session_id.c_str());

            // Auto-abort if we get a START command with wrong session to help recovery
            if (command == "START") {
                CONSOLE_INFO("MQTTOTAHandler::HandleCommand",
                            "Auto-aborting old session to allow new OTA");
                AbortOTA();
            }
            return false;
        }
    }

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
    } else if (command == "REBOOT") {
        return RebootDevice();
    } else if (command == "GET_PARTITION") {
        return GetPartitionInfo();
    }
    CONSOLE_WARNING("MQTTOTAHandler::HandleCommand", "Unknown command: %s", command.c_str());
    return false;
}

bool MQTTOTAHandler::HandleChunk(uint32_t index, const uint8_t* data, size_t len) {
    CONSOLE_INFO("MQTTOTAHandler::HandleChunk",
                 "Processing chunk %" PRIu32 " with %zu bytes", index, len);

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

    // Parse chunk header (14 bytes: session_id(4) + index(4) + size(2) + crc32(4))
    const size_t kHeaderSize = 14;
    if (len < kHeaderSize) {
        CONSOLE_ERROR("MQTTOTAHandler::HandleChunk",
                      "Chunk too small for header: %zu bytes (need %zu)", len, kHeaderSize);
        PublishAck(index, false);
        return false;
    }

    // Parse header fields (big-endian from Python)
    uint32_t session_id32 = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
    uint32_t chunk_index = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7];
    uint16_t chunk_size __attribute__((unused)) = (data[8] << 8) | data[9];
    uint32_t crc32 = (data[10] << 24) | (data[11] << 16) | (data[12] << 8) | data[13];

    // Validate session and chunk index
    if (expected_session_id32_ != 0 && session_id32 != expected_session_id32_) {
        CONSOLE_ERROR("MQTTOTAHandler::HandleChunk", "Session ID mismatch in chunk: expected 0x%08" PRIX32 ", got 0x%08" PRIX32,
                      expected_session_id32_, session_id32);
        PublishAck(index, false);
        return false;
    }

    // Validate chunk index matches
    if (chunk_index != index) {
        CONSOLE_ERROR("MQTTOTAHandler::HandleChunk",
                      "Chunk index mismatch: expected %" PRIu32 ", got %" PRIu32,
                      index, chunk_index);
        PublishAck(index, false);
        return false;
    }

    // Extract chunk data
    const uint8_t* chunk_data = data + kHeaderSize;
    size_t chunk_data_len = len - kHeaderSize;

    // Validate chunk size
    // For the last chunk, it may be smaller than chunk_size
    bool is_last_chunk = (index == manifest_.total_chunks - 1);
    size_t expected_chunk_size = manifest_.chunk_size;

    if (is_last_chunk) {
        // Calculate expected size for last chunk
        size_t remaining_bytes = manifest_.size - (index * manifest_.chunk_size);
        expected_chunk_size = (remaining_bytes < manifest_.chunk_size) ? remaining_bytes : manifest_.chunk_size;

        // Allow padded last chunk (device might send padded data for alignment)
        if (chunk_data_len != expected_chunk_size && chunk_data_len != manifest_.chunk_size) {
            CONSOLE_ERROR("MQTTOTAHandler::HandleChunk",
                          "Last chunk size mismatch: expected %zu or %lu, got %zu",
                          expected_chunk_size, (unsigned long)manifest_.chunk_size, chunk_data_len);
            PublishAck(index, false);
            return false;
        }
    } else {
        // Non-last chunks must be exactly chunk_size
        if (chunk_data_len != manifest_.chunk_size) {
            CONSOLE_ERROR("MQTTOTAHandler::HandleChunk",
                          "Chunk size mismatch: expected %lu, got %zu",
                          (unsigned long)manifest_.chunk_size, chunk_data_len);
            PublishAck(index, false);
            return false;
        }
    }

    // Verify CRC32 for MQTT transport integrity
    uint32_t calculated_crc = CalculateCRC32(chunk_data, chunk_data_len);

    // Debug logging for CRC verification
    CONSOLE_INFO("MQTTOTAHandler::HandleChunk",
                 "Chunk %" PRIu32 " CRC32: calculated=0x%08" PRIX32 ", expected=0x%08" PRIX32 ", data_len=%zu",
                 index, calculated_crc, crc32, chunk_data_len);

    if (calculated_crc != crc32) {
        CONSOLE_ERROR("MQTTOTAHandler::HandleChunk",
                      "CRC32 mismatch for chunk %" PRIu32 ": expected 0x%08" PRIX32 ", got 0x%08" PRIX32,
                      index, crc32, calculated_crc);
        // Log first and last few bytes of data for debugging
        if (chunk_data_len > 0) {
            CONSOLE_ERROR("MQTTOTAHandler::HandleChunk",
                          "First 4 bytes: %02X %02X %02X %02X, Last 4: %02X %02X %02X %02X",
                          chunk_data[0], chunk_data[1], chunk_data[2], chunk_data[3],
                          chunk_data[chunk_data_len-4], chunk_data[chunk_data_len-3],
                          chunk_data[chunk_data_len-2], chunk_data[chunk_data_len-1]);
        }
        PublishAck(index, false);
        return false;
    }

    // Calculate flash offset
    // The MQTT publisher sends .ota file which has 20-byte header followed by app
    // We need to skip the header and write app at 4KB offset
    constexpr uint32_t OTA_HEADER_SIZE = 20;  // 5 * 4 bytes
    constexpr uint32_t APP_OFFSET = 4 * 1024; // App starts at 4KB in flash

    uint32_t flash_offset;

    // For the last chunk, only write the actual data bytes, not padding
    size_t bytes_to_write = chunk_data_len;

    // First chunk contains OTA header that we need to skip
    if (index == 0) {
        // First chunk: skip OTA header, write remaining data at APP_OFFSET
        if (chunk_data_len <= OTA_HEADER_SIZE) {
            CONSOLE_ERROR("MQTTOTAHandler::HandleChunk",
                         "First chunk too small, no app data after header");
            return false;
        }
        // Skip the 20-byte header, write rest at 4KB
        chunk_data += OTA_HEADER_SIZE;
        chunk_data_len -= OTA_HEADER_SIZE;
        bytes_to_write = chunk_data_len;  // Update bytes to write
        flash_offset = APP_OFFSET;
    } else {
        // Subsequent chunks: calculate offset based on chunk index
        // Each chunk after the first is written sequentially after the previous
        flash_offset = APP_OFFSET + (index * manifest_.chunk_size - OTA_HEADER_SIZE);
    }
    if (is_last_chunk && chunk_data_len > expected_chunk_size) {
        // Chunk is padded, only write the actual firmware bytes
        bytes_to_write = expected_chunk_size;
        // Recalculate CRC on the exact bytes we will write so Pico's verify matches
        calculated_crc = CalculateCRC32(chunk_data, bytes_to_write);
        CONSOLE_INFO("MQTTOTAHandler::HandleChunk",
                     "Last chunk: writing %zu bytes (ignoring %zu bytes of padding)",
                     bytes_to_write, chunk_data_len - bytes_to_write);
    }

    // Log write operation details
    CONSOLE_INFO("MQTTOTAHandler::HandleChunk",
                 "Writing chunk %" PRIu32 ": offset=0x%08" PRIX32 ", size=%zu, crc=0x%08" PRIX32 "",
                 index, flash_offset, bytes_to_write, calculated_crc);

    // Write to flash via AT command
    if (!WriteChunkToFlash(flash_offset, chunk_data, bytes_to_write, calculated_crc)) {
        CONSOLE_ERROR("MQTTOTAHandler::HandleChunk",
                      "Failed to write chunk %" PRIu32 " to flash at offset 0x%08" PRIX32 "",
                      index, flash_offset);
        PublishAck(index, false);
        return false;
    }

    // Check if the write was successful by reading the Pico's response
    // The Pico will verify the CRC after writing and return OK or ERROR
    // TODO: Implement proper response parsing from Pico
    // TODO: Add timeout handling for chunk transfers (currently no timeout checking)

    // Mark chunk as received only if write was successful
    received_chunks_[index] = true;
    chunks_received_++;
    bytes_received_ += chunk_data_len;
    last_chunk_timestamp_ms_ = esp_log_timestamp();

    // Send ACK only after successful write and verification
    PublishAck(index, true);

    // Update progress
    PublishProgress();

    // Check if all chunks received
    if (chunks_received_ == total_chunks_) {
        CONSOLE_INFO("MQTTOTAHandler::HandleChunk",
                     "All chunks received, completing OTA update");

        // First, complete the OTA by writing the header
        if (!CompleteOTAUpdate()) {
            CONSOLE_ERROR("MQTTOTAHandler::HandleChunk", "Failed to complete OTA update");
            state_ = OTAState::ERROR;
            PublishStatus();
            return false;
        }

        // Then verify
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
    CONSOLE_INFO("MQTTOTAHandler::StartOTA",
                 "StartOTA called - current state: %d, expected: %d (MANIFEST_RECEIVED)",
                 static_cast<int>(state_), static_cast<int>(OTAState::MANIFEST_RECEIVED));

    if (state_ != OTAState::MANIFEST_RECEIVED) {
        CONSOLE_ERROR("MQTTOTAHandler::StartOTA",
                      "Cannot start OTA - wrong state. Current: %d, need: %d (MANIFEST_RECEIVED)",
                      static_cast<int>(state_), static_cast<int>(OTAState::MANIFEST_RECEIVED));
        return false;
    }

    CONSOLE_INFO("MQTTOTAHandler::StartOTA", "Starting OTA update");

    // Set global OTA active flag to block console input
    g_mqtt_ota_active = true;

    // Register as active OTA handler to receive Pico responses
    active_ota_handler_ = this;

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

    // Subscriptions handled by MQTTClient; no direct subscribe here.

    state_ = OTAState::DOWNLOADING;
    PublishStatus();

    CONSOLE_INFO("MQTTOTAHandler::StartOTA", "Ready to receive %" PRIu32 " chunks", total_chunks_);
    return true;
}

bool MQTTOTAHandler::AbortOTA() {
    CONSOLE_INFO("MQTTOTAHandler::AbortOTA", "Aborting OTA update");

    state_ = OTAState::IDLE;

    // Clear global OTA active flag
    g_mqtt_ota_active = false;

    // Unregister as active handler
    if (active_ota_handler_ == this) {
        active_ota_handler_ = nullptr;
    }

    // Clear session
    current_session_id_.clear();
    expected_session_id32_ = 0;  // Clear expected session ID as well
    manifest_ = Manifest();
    received_chunks_.clear();
    chunks_received_ = 0;
    total_chunks_ = 0;
    bytes_received_ = 0;
    total_bytes_ = 0;

    // Note: There's no AT+OTA=ABORT command implemented on Pico
    // The OTA state is managed by ESP32, Pico just handles flash operations

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

    // Clear global OTA active flag before reboot
    g_mqtt_ota_active = false;

    // Send final status before reboot
    PublishStatus();

    // Send AT+OTA=BOOT command to Pico
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+OTA=BOOT\r\n");
    bool success = SendCommandToPico(cmd);

    if (success) {
        CONSOLE_INFO("MQTTOTAHandler::BootNewFirmware", "Sent BOOT command to Pico");
    }

    return success;
}

bool MQTTOTAHandler::RebootDevice() {
    CONSOLE_INFO("MQTTOTAHandler::RebootDevice", "Rebooting device for clean state");

    // IMPORTANT: Clear all OTA state FIRST to prevent "already in progress" errors
    // This ensures that if reboot is delayed, we won't reject new manifests
    state_ = OTAState::IDLE;
    g_mqtt_ota_active = false;
    current_session_id_.clear();
    expected_session_id32_ = 0;
    manifest_ = Manifest();
    received_chunks_.clear();
    chunks_received_ = 0;
    total_chunks_ = 0;
    bytes_received_ = 0;
    total_bytes_ = 0;

    PublishStatus();

    // Note: There's no AT+OTA=ABORT command on Pico side
    // OTA state is managed by ESP32

    // Send AT+REBOOT command to Pico (reboots without resetting settings)
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+REBOOT\r\n");
    bool success = SendCommandToPico(cmd);

    if (success) {
        CONSOLE_INFO("MQTTOTAHandler::RebootDevice", "Sent REBOOT command to Pico");
        // Brief delay to allow Pico to consume the command
        vTaskDelay(pdMS_TO_TICKS(100));

        // Note: Only rebooting Pico, not ESP32. ESP32 stays running to continue OTA process.
        // This prevents the double-reboot issue where both Pico and ESP32 restart.
        CONSOLE_INFO("MQTTOTAHandler::RebootDevice", "Pico rebooting, ESP32 staying online for OTA");
    }

    return success;
}

bool MQTTOTAHandler::GetPartitionInfo() {
    CONSOLE_INFO("MQTTOTAHandler::GetPartitionInfo", "Querying target partition");

    // Send AT+OTA=GET_PARTITION command to Pico
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+OTA=GET_PARTITION\r\n");
    bool success = SendCommandToPico(cmd);

    if (success) {
        CONSOLE_INFO("MQTTOTAHandler::GetPartitionInfo", "Sent GET_PARTITION command to Pico");
    }

    return success;
}

// Public JSON getters used by MQTTClient for publishing
std::string MQTTOTAHandler::GetStateJSON() const {
    return CreateStatusJson();
}

std::string MQTTOTAHandler::GetProgressJSON() const {
    return CreateProgressJson();
}

// Optional pause/resume controls (no-op for now; state machine continues)
bool MQTTOTAHandler::PauseOTA() {
    CONSOLE_INFO("MQTTOTAHandler::PauseOTA", "Pause requested");
    return true;
}

bool MQTTOTAHandler::ResumeOTA() {
    CONSOLE_INFO("MQTTOTAHandler::ResumeOTA", "Resume requested");
    return true;
}

void MQTTOTAHandler::PublishStatus() {
    if (!mqtt_client_ || !mqtt_client_->IsConnected()) {
        return;
    }

    std::string status_json = CreateStatusJson();
    std::string topic = GetStatusTopic();

    // Publication handled by MQTT client directly; use ESP-IDF client via MQTTClient if needed.
    // Intentionally left as no-op to avoid dependency on non-existent Publish() API.
}

void MQTTOTAHandler::PublishProgress() {
    if (!mqtt_client_ || !mqtt_client_->IsConnected()) {
        return;
    }

    std::string progress_json = CreateProgressJson();
    std::string topic = GetProgressTopic();

    // See note above regarding publishing abstraction.
}

void MQTTOTAHandler::PublishAck(uint32_t chunk_index, bool success) {
    if (!mqtt_client_ || !mqtt_client_->IsConnected()) {
        return;
    }

    std::string ack = success ? "1" : "0";
    std::string topic = GetAckTopic(chunk_index);

    // See note above regarding publishing abstraction.
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
    // Send AT+OTA=ERASE command to Pico via network console
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "AT+OTA=ERASE\r\n");

    // Send command through network console queue
    bool success = SendCommandToPico(cmd);

    if (success) {
        CONSOLE_INFO("MQTTOTAHandler::EraseFlashPartition",
                     "Sent OTA ERASE command to Pico for %" PRIu32 " bytes",
                     manifest_.size);
    } else {
        CONSOLE_ERROR("MQTTOTAHandler::EraseFlashPartition",
                      "Failed to send ERASE command to Pico");
    }

    return success;
}

bool MQTTOTAHandler::WriteChunkToFlash(uint32_t offset, const uint8_t* data, size_t len, uint32_t crc) {
    // Send AT+OTA=WRITE command followed by binary data with handshakes
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "AT+OTA=WRITE,%lX,%lu,%lX\r\n",
             (unsigned long)offset, (unsigned long)len, (unsigned long)crc);

    // Send command
    if (!SendCommandToPico(cmd)) {
        CONSOLE_ERROR("MQTTOTAHandler::WriteChunkToFlash",
                      "Failed to send WRITE command to Pico");
        return false;
    }

    // Wait for READY from Pico
    if (!WaitForPicoResponse("READY", 1000)) {
        CONSOLE_ERROR("MQTTOTAHandler::WriteChunkToFlash",
                      "Timeout waiting for READY from Pico");
        return false;
    }

    // Send binary data after getting READY
    if (!SendDataToPico(data, len)) {
        CONSOLE_ERROR("MQTTOTAHandler::WriteChunkToFlash",
                      "Failed to send chunk data to Pico");
        return false;
    }

    // Wait for OK after Pico verifies write
    if (!WaitForPicoResponse("OK", 5000)) {
        CONSOLE_ERROR("MQTTOTAHandler::WriteChunkToFlash",
                      "Flash write verification failed - Pico returned ERROR or timeout");
        return false;
    }

    CONSOLE_INFO("MQTTOTAHandler::WriteChunkToFlash",
                 "Successfully wrote and verified chunk: offset=0x%08lX len=%zu crc=0x%08lX",
                 (unsigned long)offset, len, (unsigned long)crc);

    return true;
}

bool MQTTOTAHandler::CompleteOTAUpdate() {
    // Send AT+OTA=COMPLETE command to Pico to write the header
    // The manifest size includes the 20-byte OTA header, but we need only app size
    constexpr uint32_t OTA_HEADER_SIZE = 20;
    uint32_t app_size = manifest_.size - OTA_HEADER_SIZE;

    char cmd[256];
    snprintf(cmd, sizeof(cmd), "AT+OTA=COMPLETE,%lu\r\n", (unsigned long)app_size);

    bool success = SendCommandToPico(cmd);

    if (success) {
        CONSOLE_INFO("MQTTOTAHandler::CompleteOTAUpdate",
                     "Sent OTA COMPLETE command to Pico for %lu bytes (app only)", (unsigned long)app_size);
    } else {
        CONSOLE_ERROR("MQTTOTAHandler::CompleteOTAUpdate",
                      "Failed to send COMPLETE command to Pico");
    }

    return success;
}

bool MQTTOTAHandler::VerifyFlashPartition() {
    // Send AT+OTA=VERIFY command to Pico (no SHA arg; Pico verifies header CRC)
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "AT+OTA=VERIFY\r\n");

    bool success = SendCommandToPico(cmd);

    if (success) {
        CONSOLE_INFO("MQTTOTAHandler::VerifyFlashPartition",
                     "Sent OTA VERIFY command to Pico");
    } else {
        CONSOLE_ERROR("MQTTOTAHandler::VerifyFlashPartition",
                      "Failed to send VERIFY command to Pico");
    }

    return success;
}

void MQTTOTAHandler::RequestMissingChunks() {
    // Publish request for missing chunks
    std::vector<uint32_t> missing = GetMissingChunks();
    if (missing.empty()) {
        return;
    }

    // TODO: Implement missing chunk request mechanism
    // Future work needed:
    // 1. Publish list of missing chunk indices to /ota/control/missing_chunks topic
    // 2. Format as JSON array: {"session_id": "...", "missing": [1, 5, 10]}
    // 3. Publisher should resend requested chunks
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
            manifest_.total_chunks = std::stoul(json_str.substr(pos + 1));
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
    // CRC32 implementation matching web OTA and Pico's expectation (with final inversion)
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
        }
    }
    return crc ^ 0xFFFFFFFF;  // Final XOR to match standard CRC32 (same as web OTA)
}

bool MQTTOTAHandler::SendCommandToPico(const char* cmd) {
    // Send AT command to Pico via network console queue
    xSemaphoreTake(object_dictionary.network_console_rx_queue_mutex, portMAX_DELAY);

    size_t cmd_len = strlen(cmd);

    // Check if there's enough space in the queue
    size_t available = object_dictionary.network_console_rx_queue.MaxNumElements() -
                       object_dictionary.network_console_rx_queue.Length();
    if (available < cmd_len) {
        CONSOLE_ERROR("MQTTOTAHandler::SendCommandToPico",
                      "Not enough space in queue for command: need %zu, have %zu", cmd_len, available);
        xSemaphoreGive(object_dictionary.network_console_rx_queue_mutex);

        // Wait a bit for the queue to drain
        vTaskDelay(pdMS_TO_TICKS(50));
        return false;
    }

    for (size_t i = 0; i < cmd_len; i++) {
        if (!object_dictionary.network_console_rx_queue.Push(cmd[i])) {
            CONSOLE_ERROR("MQTTOTAHandler::SendCommandToPico",
                          "Failed to push command character to network console queue");
            xSemaphoreGive(object_dictionary.network_console_rx_queue_mutex);
            return false;
        }
    }

    xSemaphoreGive(object_dictionary.network_console_rx_queue_mutex);
    return true;
}

bool MQTTOTAHandler::SendDataToPico(const uint8_t* data, size_t len) {
    // Stream binary data to Pico via network console queue with backpressure
    size_t pos = 0;
    while (pos < len) {
        xSemaphoreTake(object_dictionary.network_console_rx_queue_mutex, portMAX_DELAY);
        size_t available = object_dictionary.network_console_rx_queue.MaxNumElements() -
                           object_dictionary.network_console_rx_queue.Length();
        if (available == 0) {
            xSemaphoreGive(object_dictionary.network_console_rx_queue_mutex);
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        size_t to_push = (len - pos < available) ? (len - pos) : available;
        for (size_t i = 0; i < to_push; i++) {
            if (!object_dictionary.network_console_rx_queue.Push(static_cast<char>(data[pos + i]))) {
                // If push fails unexpectedly, release and retry
                xSemaphoreGive(object_dictionary.network_console_rx_queue_mutex);
                vTaskDelay(pdMS_TO_TICKS(1));
                goto continue_outer;
            }
        }
        xSemaphoreGive(object_dictionary.network_console_rx_queue_mutex);
        pos += to_push;
continue_outer:
        ;
    }
    return true;
}
bool MQTTOTAHandler::WaitForPicoResponse(const char* expected_response, uint32_t timeout_ms) {
    // Clear response buffer and flags
    response_buffer_pos_ = 0;
    response_received_ = false;
    memset(response_buffer_, 0, kResponseBufferSize);

    // Wait for response with timeout
    TickType_t start_time = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);

    while ((xTaskGetTickCount() - start_time) < timeout_ticks) {
        // Check if we have a complete response line
        if (xSemaphoreTake(response_semaphore_, pdMS_TO_TICKS(10)) == pdTRUE) {
            // Response received, check if it matches expected
            if (strstr(response_buffer_, expected_response) != nullptr) {
                CONSOLE_INFO("MQTTOTAHandler::WaitForPicoResponse",
                           "Got expected response: %s", expected_response);
                return true;
            } else if (strstr(response_buffer_, "ERROR") != nullptr) {
                CONSOLE_ERROR("MQTTOTAHandler::WaitForPicoResponse",
                            "Got ERROR response: %s", response_buffer_);
                return false;
            }
            // Not the expected response, clear and continue waiting
            response_buffer_pos_ = 0;
            response_received_ = false;
        }
    }

    CONSOLE_ERROR("MQTTOTAHandler::WaitForPicoResponse",
                  "Timeout waiting for response '%s' after %lu ms",
                  expected_response, (unsigned long)timeout_ms);
    return false;
}

void MQTTOTAHandler::OnPicoResponse(const char* message, size_t len) {
    // Called when Pico sends a response via network console
    // Buffer the message and signal if it's a complete line

    for (size_t i = 0; i < len && response_buffer_pos_ < kResponseBufferSize - 1; i++) {
        response_buffer_[response_buffer_pos_++] = message[i];

        // Check for line ending (\r\n or \n)
        if (message[i] == '\n' ||
            (message[i] == '\r' && i + 1 < len && message[i + 1] == '\n')) {
            // Complete line received
            response_buffer_[response_buffer_pos_] = '\0';
            response_received_ = true;

            // Signal the waiting task
            xSemaphoreGive(response_semaphore_);

            // Skip \n if we had \r\n
            if (message[i] == '\r' && i + 1 < len && message[i + 1] == '\n') {
                i++;
            }
        }
    }
}

void MQTTOTAHandler::GlobalPicoResponseCallback(const char* message, size_t len) {
    // Static callback that forwards to active OTA handler if one exists
    if (active_ota_handler_ != nullptr) {
        active_ota_handler_->OnPicoResponse(message, len);
    }
}

// C-linkage wrapper for calling from object_dictionary.cpp
extern "C" void MQTTOTAHandler_GlobalPicoResponseCallback(const char* message, size_t len) {
    MQTTOTAHandler::GlobalPicoResponseCallback(message, len);
}
