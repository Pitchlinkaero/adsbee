#ifndef MQTT_OTA_HH
#define MQTT_OTA_HH

#include "mqtt_config.h"
#include <string>
#include <vector>
#include <map>
#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
// Pass-through OTA - no ESP flash operations needed

// Forward declaration to avoid circular include with mqtt_client.hh
namespace MQTT {
class MQTTClient;
}

class MQTTOTAHandler {
public:
    enum class OTAState {
        IDLE,
        MANIFEST_RECEIVED,
        ERASING,
        DOWNLOADING,
        VERIFYING,
        READY_TO_BOOT,
        ERROR
    };

    struct Manifest {
        std::string version;
        uint32_t size;
        uint32_t total_chunks;
        uint32_t chunk_size;
        std::string sha256;
        std::string session_id;
        std::string build_date;
    };

    struct ChunkHeader {
        uint32_t session_id;
        uint32_t chunk_index;
        uint16_t chunk_size;
        uint32_t crc32;
        uint16_t flags;
    };

    MQTTOTAHandler(const std::string& device_id, uint16_t feed_index);
    ~MQTTOTAHandler();

    // Initialization with MQTT client
    bool Initialize(MQTT::MQTTClient* mqtt_client);

    // Handle incoming MQTT messages
    bool HandleManifest(const Manifest& manifest);
    bool HandleCommand(const std::string& command, const std::string& session_id);
    bool HandleChunk(uint32_t index, const uint8_t* data, size_t len);

    // Get status/progress as JSON
    std::string GetStateJSON() const;
    std::string GetProgressJSON() const;

    // State management
    OTAState GetState() const { return state_; }
    float GetProgress() const;
    std::string GetSessionId() const { return current_session_id_; }

    // Control functions
    bool StartOTA();
    bool PauseOTA();
    bool ResumeOTA();
    bool AbortOTA();
    bool VerifyOTA();
    bool BootNewFirmware();
    bool RebootDevice();
    bool GetPartitionInfo();

    // Check if OTA is active (for blocking other operations)
    bool IsOTAActive() const {
        return state_ != OTAState::IDLE && state_ != OTAState::ERROR;
    }

    // Status reporting
    void PublishStatus();
    void PublishProgress();
    void PublishAck(uint32_t chunk_index, bool success);

private:
    // Configuration
    std::string device_id_;
    uint16_t feed_index_;

    // MQTT client
    MQTT::MQTTClient* mqtt_client_ = nullptr;

    // State
    OTAState state_;
    Manifest manifest_;
    std::string current_session_id_;
    uint32_t expected_session_id32_ = 0;  // First 4 bytes of session (hex) from manifest
    int target_partition_ = -1;  // Which partition to write to (0 or 1, -1 = unknown)

    // Chunk management
    std::vector<bool> received_chunks_;
    uint32_t chunks_received_;
    uint32_t total_chunks_;
    uint32_t bytes_received_;
    uint32_t total_bytes_;

    // Timing
    uint32_t last_chunk_timestamp_ms_;
    uint32_t ota_start_timestamp_ms_;
    uint32_t timeout_ms_;

    // No local buffering required in pass-through mode

    // Pass-through mode - no local storage needed
    // Pico handles all flash operations

    // Helper functions
    bool EraseFlashPartition();
    bool WriteChunkToFlash(uint32_t offset, const uint8_t* data, size_t len, uint32_t crc);
    bool WriteChunkToFlashWithTimeout(uint32_t offset, const uint8_t* data, size_t len, uint32_t crc, uint32_t timeout_ms);
    bool CompleteOTAUpdate();
    bool VerifyFlashPartition();
    std::vector<uint32_t> GetMissingChunks() const;
    void RequestMissingChunks();

    // Topic generation
    std::string GetControlTopic() const;
    std::string GetManifestTopic() const;
    std::string GetChunkTopic(uint32_t index) const;
    std::string GetStatusTopic() const;
    std::string GetProgressTopic() const;
    std::string GetAckTopic(uint32_t index) const;

    // JSON parsing helpers
    bool ParseManifest(const std::string& json_str);
    bool ParseControl(const std::string& json_str);
    std::string CreateStatusJson() const;
    std::string CreateProgressJson() const;

    // CRC calculation
    uint32_t CalculateCRC32(const uint8_t* data, size_t len);

    // Pico communication helpers
    bool SendCommandToPico(const char* cmd);
    bool SendDataToPico(const uint8_t* data, size_t len);
    bool WaitForPicoResponse(const char* expected_response, uint32_t timeout_ms);

    // Response buffer for capturing Pico messages
    static constexpr size_t kResponseBufferSize = 256;
    char response_buffer_[kResponseBufferSize];
    size_t response_buffer_pos_ = 0;
    SemaphoreHandle_t response_semaphore_ = nullptr;
    bool response_received_ = false;

    // Called by network console to feed responses
    void OnPicoResponse(const char* message, size_t len);

public:
    // Static callback for global access
    static MQTTOTAHandler* active_ota_handler_;
    static void GlobalPicoResponseCallback(const char* message, size_t len);
};

#endif // MQTT_OTA_HH