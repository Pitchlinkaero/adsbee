#include "gps_network_server.hh"
#include "esp_log.h"
#include "esp_timer.h"
#include <algorithm>
#include <cstring>

static const char* TAG = "GPS_NET";

// Global instance
GPSNetworkServer gps_network_server;

GPSNetworkServer::GPSNetworkServer() 
    : nmea_port_(kDefaultNMEAPort),
      mavlink_port_(kDefaultMAVLinkPort),
      nmea_enabled_(true),
      mavlink_enabled_(true),
      nmea_socket_(-1),
      mavlink_socket_(-1),
      message_queue_(nullptr),
      nmea_task_handle_(nullptr),
      mavlink_task_handle_(nullptr),
      running_(false),
      should_stop_(false),
      last_message_time_ms_(0) {
    
    memset(nmea_clients_, 0, sizeof(nmea_clients_));
    memset(mavlink_clients_, 0, sizeof(mavlink_clients_));
    memset(&stats_, 0, sizeof(stats_));
}

GPSNetworkServer::~GPSNetworkServer() {
    Stop();
    if (message_queue_) {
        vQueueDelete(message_queue_);
    }
}

bool GPSNetworkServer::Init(uint16_t nmea_port, uint16_t mavlink_port) {
    nmea_port_ = nmea_port;
    mavlink_port_ = mavlink_port;
    
    // Create message queue
    message_queue_ = xQueueCreate(kGPSMessageQueueDepth, sizeof(GPSNetworkMessage));
    if (!message_queue_) {
        ESP_LOGE(TAG, "Failed to create message queue");
        return false;
    }
    
    // Create UDP socket for NMEA
    nmea_socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (nmea_socket_ < 0) {
        ESP_LOGE(TAG, "Failed to create NMEA socket");
        return false;
    }
    
    // Set socket to non-blocking
    int flags = fcntl(nmea_socket_, F_GETFL, 0);
    fcntl(nmea_socket_, F_SETFL, flags | O_NONBLOCK);
    
    // Bind NMEA socket
    struct sockaddr_in nmea_addr;
    memset(&nmea_addr, 0, sizeof(nmea_addr));
    nmea_addr.sin_family = AF_INET;
    nmea_addr.sin_addr.s_addr = INADDR_ANY;
    nmea_addr.sin_port = htons(nmea_port_);
    
    if (bind(nmea_socket_, (struct sockaddr*)&nmea_addr, sizeof(nmea_addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind NMEA socket to port %d", nmea_port_);
        close(nmea_socket_);
        nmea_socket_ = -1;
        return false;
    }
    
    // Create UDP socket for MAVLink
    mavlink_socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (mavlink_socket_ < 0) {
        ESP_LOGE(TAG, "Failed to create MAVLink socket");
        close(nmea_socket_);
        nmea_socket_ = -1;
        return false;
    }
    
    // Set socket to non-blocking
    flags = fcntl(mavlink_socket_, F_GETFL, 0);
    fcntl(mavlink_socket_, F_SETFL, flags | O_NONBLOCK);
    
    // Bind MAVLink socket
    struct sockaddr_in mavlink_addr;
    memset(&mavlink_addr, 0, sizeof(mavlink_addr));
    mavlink_addr.sin_family = AF_INET;
    mavlink_addr.sin_addr.s_addr = INADDR_ANY;
    mavlink_addr.sin_port = htons(mavlink_port_);
    
    if (bind(mavlink_socket_, (struct sockaddr*)&mavlink_addr, sizeof(mavlink_addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind MAVLink socket to port %d", mavlink_port_);
        close(nmea_socket_);
        close(mavlink_socket_);
        nmea_socket_ = -1;
        mavlink_socket_ = -1;
        return false;
    }
    
    ESP_LOGI(TAG, "GPS Network Server initialized - NMEA:%d, MAVLink:%d", 
             nmea_port_, mavlink_port_);
    
    return true;
}

bool GPSNetworkServer::Start() {
    if (running_) {
        return true;
    }
    
    should_stop_ = false;
    
    // Create NMEA listener task
    if (nmea_enabled_ && nmea_socket_ >= 0) {
        xTaskCreate(NMEAListenerTask, "GPS_NMEA", 4096, this, 5, &nmea_task_handle_);
    }
    
    // Create MAVLink listener task
    if (mavlink_enabled_ && mavlink_socket_ >= 0) {
        xTaskCreate(MAVLinkListenerTask, "GPS_MAV", 4096, this, 5, &mavlink_task_handle_);
    }
    
    running_ = true;
    ESP_LOGI(TAG, "GPS Network Server started");
    
    return true;
}

void GPSNetworkServer::Stop() {
    if (!running_) {
        return;
    }
    
    should_stop_ = true;
    
    // Wait for tasks to stop
    if (nmea_task_handle_) {
        vTaskDelete(nmea_task_handle_);
        nmea_task_handle_ = nullptr;
    }
    
    if (mavlink_task_handle_) {
        vTaskDelete(mavlink_task_handle_);
        mavlink_task_handle_ = nullptr;
    }
    
    // Close sockets
    if (nmea_socket_ >= 0) {
        close(nmea_socket_);
        nmea_socket_ = -1;
    }
    
    if (mavlink_socket_ >= 0) {
        close(mavlink_socket_);
        mavlink_socket_ = -1;
    }
    
    running_ = false;
    ESP_LOGI(TAG, "GPS Network Server stopped");
}

void GPSNetworkServer::NMEAListenerTask(void* param) {
    GPSNetworkServer* server = static_cast<GPSNetworkServer*>(param);
    server->NMEAListener();
    vTaskDelete(nullptr);
}

void GPSNetworkServer::MAVLinkListenerTask(void* param) {
    GPSNetworkServer* server = static_cast<GPSNetworkServer*>(param);
    server->MAVLinkListener();
    vTaskDelete(nullptr);
}

void GPSNetworkServer::NMEAListener() {
    struct sockaddr_in client_addr;
    socklen_t client_addr_len = sizeof(client_addr);
    
    ESP_LOGI(TAG, "NMEA listener started on port %d", nmea_port_);
    
    while (!should_stop_) {
        // Receive data
        ssize_t len = recvfrom(nmea_socket_, nmea_buffer_, sizeof(nmea_buffer_) - 1,
                              0, (struct sockaddr*)&client_addr, &client_addr_len);
        
        if (len > 0) {
            nmea_buffer_[len] = '\0';  // Null terminate
            
            // Find or add client
            uint8_t client_id = FindOrAddClient(nmea_clients_, client_addr);
            UpdateClientActivity(nmea_clients_, client_id);
            
            // Process NMEA sentence
            if (ProcessNMEASentence(nmea_buffer_, len, client_id)) {
                stats_.nmea_sentences_received++;
                stats_.last_nmea_timestamp_ms = esp_timer_get_time() / 1000;
            } else {
                stats_.parse_errors++;
            }
        } else if (len < 0 && errno != EWOULDBLOCK && errno != EAGAIN) {
            ESP_LOGE(TAG, "NMEA recv error: %d", errno);
        }
        
        // Cleanup inactive clients periodically
        static uint32_t last_cleanup = 0;
        uint32_t now = esp_timer_get_time() / 1000;
        if (now - last_cleanup > 10000) {  // Every 10 seconds
            CleanupInactiveClients(nmea_clients_);
            last_cleanup = now;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));  // Small delay to prevent CPU hogging
    }
    
    ESP_LOGI(TAG, "NMEA listener stopped");
}

void GPSNetworkServer::MAVLinkListener() {
    struct sockaddr_in client_addr;
    socklen_t client_addr_len = sizeof(client_addr);
    
    ESP_LOGI(TAG, "MAVLink listener started on port %d", mavlink_port_);
    
    while (!should_stop_) {
        // Receive data
        ssize_t len = recvfrom(mavlink_socket_, mavlink_buffer_, sizeof(mavlink_buffer_),
                              0, (struct sockaddr*)&client_addr, &client_addr_len);
        
        if (len > 0) {
            // Find or add client
            uint8_t client_id = FindOrAddClient(mavlink_clients_, client_addr);
            UpdateClientActivity(mavlink_clients_, client_id);
            
            // Process MAVLink message
            if (ProcessMAVLinkMessage(mavlink_buffer_, len, client_id)) {
                stats_.mavlink_messages_received++;
                stats_.last_mavlink_timestamp_ms = esp_timer_get_time() / 1000;
            } else {
                stats_.parse_errors++;
            }
        } else if (len < 0 && errno != EWOULDBLOCK && errno != EAGAIN) {
            ESP_LOGE(TAG, "MAVLink recv error: %d", errno);
        }
        
        // Cleanup inactive clients periodically
        static uint32_t last_cleanup = 0;
        uint32_t now = esp_timer_get_time() / 1000;
        if (now - last_cleanup > 10000) {  // Every 10 seconds
            CleanupInactiveClients(mavlink_clients_);
            last_cleanup = now;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));  // Small delay to prevent CPU hogging
    }
    
    ESP_LOGI(TAG, "MAVLink listener stopped");
}

bool GPSNetworkServer::ProcessNMEASentence(const uint8_t* data, size_t length, uint8_t source_id) {
    // Basic validation - check for $ start and checksum
    if (length < 10 || data[0] != '$') {
        return false;
    }
    
    // Find checksum
    const uint8_t* checksum_pos = nullptr;
    for (size_t i = length - 5; i < length; i++) {
        if (data[i] == '*') {
            checksum_pos = &data[i];
            break;
        }
    }
    
    if (!checksum_pos) {
        return false;  // No checksum found
    }
    
    // Calculate checksum
    uint8_t calc_checksum = 0;
    for (const uint8_t* p = data + 1; p < checksum_pos; p++) {
        calc_checksum ^= *p;
    }
    
    // Parse provided checksum
    char checksum_str[3] = {0};
    memcpy(checksum_str, checksum_pos + 1, 2);
    uint8_t provided_checksum = strtol(checksum_str, nullptr, 16);
    
    if (calc_checksum != provided_checksum) {
        ESP_LOGW(TAG, "NMEA checksum mismatch: calc=%02X, provided=%02X", 
                 calc_checksum, provided_checksum);
        return false;
    }
    
    // Rate limit check
    if (IsRateLimited()) {
        return true;  // Valid but dropped due to rate limit
    }
    
    // Queue message
    GPSNetworkMessage msg;
    msg.type = kGPSMessageTypeNMEA;
    msg.source_id = source_id;
    msg.length = std::min(length, sizeof(msg.data));
    memcpy(msg.data, data, msg.length);
    msg.timestamp_ms = esp_timer_get_time() / 1000;
    
    return QueueMessage(msg);
}

bool GPSNetworkServer::ProcessMAVLinkMessage(const uint8_t* data, size_t length, uint8_t source_id) {
    // Basic MAVLink validation
    if (length < 8) {
        return false;
    }
    
    // Check for MAVLink v1 (0xFE) or v2 (0xFD) start byte
    if (data[0] != 0xFE && data[0] != 0xFD) {
        return false;
    }
    
    // For now, just forward the entire MAVLink message
    // The RP2040 will handle parsing and extracting GPS data
    
    // Rate limit check
    if (IsRateLimited()) {
        return true;  // Valid but dropped due to rate limit
    }
    
    // Queue message
    GPSNetworkMessage msg;
    msg.type = kGPSMessageTypeMAVLink;
    msg.source_id = source_id;
    msg.length = std::min(length, sizeof(msg.data));
    memcpy(msg.data, data, msg.length);
    msg.timestamp_ms = esp_timer_get_time() / 1000;
    
    return QueueMessage(msg);
}

bool GPSNetworkServer::QueueMessage(const GPSNetworkMessage& message) {
    if (!message_queue_) {
        return false;
    }
    
    if (xQueueSend(message_queue_, &message, 0) != pdTRUE) {
        stats_.messages_dropped++;
        return false;
    }
    
    last_message_time_ms_ = esp_timer_get_time() / 1000;
    return true;
}

bool GPSNetworkServer::IsRateLimited() {
    uint32_t now = esp_timer_get_time() / 1000;
    return (now - last_message_time_ms_) < kRateLimitIntervalMs;
}

bool GPSNetworkServer::GetNextMessage(GPSNetworkMessage& message, uint32_t timeout_ms) {
    if (!message_queue_) {
        return false;
    }
    
    TickType_t timeout = timeout_ms == 0 ? 0 : pdMS_TO_TICKS(timeout_ms);
    return xQueueReceive(message_queue_, &message, timeout) == pdTRUE;
}

size_t GPSNetworkServer::GetQueuedMessageCount() const {
    if (!message_queue_) {
        return 0;
    }
    return uxQueueMessagesWaiting(message_queue_);
}

uint8_t GPSNetworkServer::FindOrAddClient(ClientInfo* clients, const struct sockaddr_in& addr) {
    // Look for existing client
    for (uint8_t i = 0; i < kMaxClientsPerPort; i++) {
        if (clients[i].active && 
            clients[i].addr.sin_addr.s_addr == addr.sin_addr.s_addr &&
            clients[i].addr.sin_port == addr.sin_port) {
            return i;
        }
    }
    
    // Find free slot
    for (uint8_t i = 0; i < kMaxClientsPerPort; i++) {
        if (!clients[i].active) {
            clients[i].addr = addr;
            clients[i].active = true;
            clients[i].last_message_ms = esp_timer_get_time() / 1000;
            
            ESP_LOGI(TAG, "New GPS client from %s:%d (slot %d)",
                     inet_ntoa(addr.sin_addr), ntohs(addr.sin_port), i);
            
            // Update stats
            if (clients == nmea_clients_) {
                stats_.active_nmea_clients++;
            } else {
                stats_.active_mavlink_clients++;
            }
            
            return i;
        }
    }
    
    // No free slots - use oldest
    uint8_t oldest = 0;
    uint32_t oldest_time = clients[0].last_message_ms;
    for (uint8_t i = 1; i < kMaxClientsPerPort; i++) {
        if (clients[i].last_message_ms < oldest_time) {
            oldest = i;
            oldest_time = clients[i].last_message_ms;
        }
    }
    
    ESP_LOGW(TAG, "GPS client slots full, replacing oldest");
    clients[oldest].addr = addr;
    clients[oldest].last_message_ms = esp_timer_get_time() / 1000;
    
    return oldest;
}

void GPSNetworkServer::UpdateClientActivity(ClientInfo* clients, uint8_t client_id) {
    if (client_id < kMaxClientsPerPort) {
        clients[client_id].last_message_ms = esp_timer_get_time() / 1000;
    }
}

void GPSNetworkServer::CleanupInactiveClients(ClientInfo* clients) {
    uint32_t now = esp_timer_get_time() / 1000;
    const uint32_t timeout_ms = 30000;  // 30 seconds
    
    for (uint8_t i = 0; i < kMaxClientsPerPort; i++) {
        if (clients[i].active && (now - clients[i].last_message_ms) > timeout_ms) {
            ESP_LOGI(TAG, "GPS client timeout (slot %d)", i);
            clients[i].active = false;
            
            // Update stats
            if (clients == nmea_clients_) {
                if (stats_.active_nmea_clients > 0) {
                    stats_.active_nmea_clients--;
                }
            } else {
                if (stats_.active_mavlink_clients > 0) {
                    stats_.active_mavlink_clients--;
                }
            }
        }
    }
}