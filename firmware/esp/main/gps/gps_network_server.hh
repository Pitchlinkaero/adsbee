#ifndef GPS_NETWORK_SERVER_HH_
#define GPS_NETWORK_SERVER_HH_

#include <cstdint>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "lwip/sockets.h"

/**
 * GPS Network Server - Receives GPS data from network sources.
 * 
 * Supports:
 * - UDP NMEA sentences on configurable port (default 10110)
 * - MAVLink GPS messages on configurable port (default 14550)
 * - Multiple simultaneous connections
 * - Rate limiting to prevent overload
 * 
 * Data is forwarded to RP2040 via SPI for processing by GNSS Manager.
 */
class GPSNetworkServer {
public:
    // Configuration
    static constexpr uint16_t kDefaultNMEAPort = 10110;
    static constexpr uint16_t kDefaultMAVLinkPort = 14550;
    static constexpr size_t kMaxNMEASentenceLen = 256;
    static constexpr size_t kMaxMAVLinkMessageLen = 280;
    static constexpr size_t kGPSMessageQueueDepth = 20;
    static constexpr uint32_t kRateLimitIntervalMs = 100;  // Min time between GPS updates
    static constexpr size_t kMaxClientsPerPort = 4;
    
    // GPS message types for SPI transfer
    enum GPSMessageType : uint8_t {
        kGPSMessageTypeNMEA = 0,
        kGPSMessageTypeMAVLink = 1,
        kGPSMessageTypeUBX = 2,
        kGPSMessageTypeSBF = 3
    };
    
    // Structure for GPS messages to send to RP2040
    struct __attribute__((__packed__)) GPSNetworkMessage {
        GPSMessageType type;
        uint8_t source_id;  // Client ID
        uint16_t length;
        uint8_t data[kMaxNMEASentenceLen];
        uint32_t timestamp_ms;
    };
    
    GPSNetworkServer();
    ~GPSNetworkServer();
    
    /**
     * Initialize the GPS network server.
     * @param nmea_port UDP port for NMEA sentences
     * @param mavlink_port UDP port for MAVLink messages
     * @return true if initialization successful
     */
    bool Init(uint16_t nmea_port = kDefaultNMEAPort, 
              uint16_t mavlink_port = kDefaultMAVLinkPort);
    
    /**
     * Start the server tasks.
     * Creates FreeRTOS tasks for UDP listeners.
     */
    bool Start();
    
    /**
     * Stop the server tasks.
     */
    void Stop();
    
    /**
     * Check if server is running.
     */
    bool IsRunning() const { return running_; }
    
    /**
     * Get the next GPS message from the queue.
     * @param message Output message structure
     * @param timeout_ms Timeout in milliseconds
     * @return true if message retrieved
     */
    bool GetNextMessage(GPSNetworkMessage& message, uint32_t timeout_ms = 0);
    
    /**
     * Get number of messages in queue.
     */
    size_t GetQueuedMessageCount() const;
    
    /**
     * Get statistics.
     */
    struct Statistics {
        uint32_t nmea_sentences_received = 0;
        uint32_t mavlink_messages_received = 0;
        uint32_t messages_dropped = 0;
        uint32_t parse_errors = 0;
        uint32_t last_nmea_timestamp_ms = 0;
        uint32_t last_mavlink_timestamp_ms = 0;
        uint8_t active_nmea_clients = 0;
        uint8_t active_mavlink_clients = 0;
    };
    
    Statistics GetStatistics() const { return stats_; }
    
    /**
     * Enable/disable NMEA reception.
     */
    void SetNMEAEnabled(bool enabled) { nmea_enabled_ = enabled; }
    
    /**
     * Enable/disable MAVLink reception.
     */
    void SetMAVLinkEnabled(bool enabled) { mavlink_enabled_ = enabled; }
    
private:
    // Task functions (static for FreeRTOS)
    static void NMEAListenerTask(void* param);
    static void MAVLinkListenerTask(void* param);
    
    // Internal methods
    void NMEAListener();
    void MAVLinkListener();
    bool ProcessNMEASentence(const uint8_t* data, size_t length, uint8_t source_id);
    bool ProcessMAVLinkMessage(const uint8_t* data, size_t length, uint8_t source_id);
    bool QueueMessage(const GPSNetworkMessage& message);
    bool IsRateLimited();
    
    // Client tracking
    struct ClientInfo {
        struct sockaddr_in addr;
        uint32_t last_message_ms;
        bool active;
    };
    
    // Configuration
    uint16_t nmea_port_;
    uint16_t mavlink_port_;
    bool nmea_enabled_;
    bool mavlink_enabled_;
    
    // Sockets
    int nmea_socket_;
    int mavlink_socket_;
    
    // Client tracking
    ClientInfo nmea_clients_[kMaxClientsPerPort];
    ClientInfo mavlink_clients_[kMaxClientsPerPort];
    uint8_t FindOrAddClient(ClientInfo* clients, const struct sockaddr_in& addr);
    void UpdateClientActivity(ClientInfo* clients, uint8_t client_id);
    void CleanupInactiveClients(ClientInfo* clients);
    
    // Message queue
    QueueHandle_t message_queue_;
    
    // Tasks
    TaskHandle_t nmea_task_handle_;
    TaskHandle_t mavlink_task_handle_;
    bool running_;
    bool should_stop_;
    
    // Rate limiting
    uint32_t last_message_time_ms_;
    
    // Statistics
    Statistics stats_;
    
    // Buffers
    uint8_t nmea_buffer_[kMaxNMEASentenceLen];
    uint8_t mavlink_buffer_[kMaxMAVLinkMessageLen];
};

// Global instance
extern GPSNetworkServer gps_network_server;

#endif // GPS_NETWORK_SERVER_HH_