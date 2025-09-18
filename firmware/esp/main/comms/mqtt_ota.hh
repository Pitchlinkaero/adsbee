#ifndef MQTT_OTA_HH
#define MQTT_OTA_HH

#include <string>
#include <vector>
#include <map>
#include <cstdint>
#include "mqtt_client.hh"

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

    struct OTAManifest {
        std::string version;
        uint32_t size;
        uint32_t chunks;
        uint32_t chunk_size;
        std::string sha256;
        uint32_t partition_crc;
        std::string build_date;
        std::string session_id;
    };

    struct ChunkHeader {
        uint32_t session_id;
        uint32_t chunk_index;
        uint16_t chunk_size;
        uint32_t crc32;
        uint16_t flags;
    };

    MQTTOTAHandler(uint16_t feed_index);
    ~MQTTOTAHandler();

    // Initialize OTA handler and subscribe to topics
    bool Initialize(MQTTClient* mqtt_client);

    // Handle incoming MQTT messages
    void HandleManifest(const std::string& payload);
    void HandleControl(const std::string& payload);
    void HandleChunk(uint32_t index, const uint8_t* data, size_t len);

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

    // Status reporting
    void PublishStatus();
    void PublishProgress();
    void PublishAck(uint32_t chunk_index, bool success);

private:
    // Configuration
    uint16_t feed_index_;
    MQTTClient* mqtt_client_;

    // State
    OTAState state_;
    OTAManifest manifest_;
    std::string current_session_id_;

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

    // Buffer for accumulating chunks
    static constexpr size_t kMaxChunkSize = 4096;
    uint8_t chunk_buffer_[kMaxChunkSize];

    // Helper functions
    bool EraseFlashPartition();
    bool WriteChunkToFlash(uint32_t offset, const uint8_t* data, size_t len, uint32_t crc);
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
};

#endif // MQTT_OTA_HH