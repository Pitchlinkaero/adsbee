#ifndef MQTT_CLIENT_HH_
#define MQTT_CLIENT_HH_

#include "esp_log.h"
#include "mqtt_client.h"  // ESP-IDF MQTT client
#include "data_structures.hh"
#include "mqtt/mqtt_protocol.hh"
#include "settings.hh"

/**
 * MQTT client for real-time ADS-B data streaming.
 * Supports both JSON and binary formats with immediate delivery.
 */
class ADSBeeMQTTClient {
public:
    struct Config {
        uint8_t feed_index;           // Which feed (0-3) this client is for
        const char* broker_uri;       // From settings feed_uris
        uint16_t broker_port;         // From settings feed_ports
        const char* client_id;        // Unique client ID
        const char* base_topic;       // From feed_receiver_ids (optional)
        MQTTProtocol::Format format;  // JSON or BINARY
    };
    
    ADSBeeMQTTClient();
    ~ADSBeeMQTTClient();
    
    /**
     * Initialize MQTT client for a specific feed
     * @param[in] config Configuration from settings
     * @return true on success
     */
    bool Init(const Config& config);
    
    /**
     * Start connection to broker
     * @return true if connection initiated
     */
    bool Connect();
    
    /**
     * Disconnect from broker
     */
    void Disconnect();
    
    /**
     * Check connection status
     * @return true if connected
     */
    bool IsConnected() const { return connected_; }
    
    /**
     * Publish decoded packet immediately (no buffering)
     * @param[in] packet Decoded ADS-B packet
     * @param[in] band Frequency band source (1090 MHz or 978 MHz UAT)
     * @return true on success
     */
    bool PublishPacket(const Decoded1090Packet& packet,
                      MQTTProtocol::FrequencyBand band = MQTTProtocol::BAND_1090_MHZ);
    
    /**
     * Publish aircraft status immediately
     * @param[in] aircraft Aircraft data
     * @param[in] band Frequency band source (1090 MHz or 978 MHz UAT)
     * @return true on success
     */
    bool PublishAircraft(const Aircraft& aircraft,
                        MQTTProtocol::FrequencyBand band = MQTTProtocol::BAND_1090_MHZ);
    
    /**
     * Get current format
     */
    MQTTProtocol::Format GetFormat() const { return config_.format; }
    
    /**
     * Get bandwidth estimate
     */
    uint32_t GetBandwidthPerHour(uint16_t messages_per_hour) const {
        return MQTTProtocol::EstimateBandwidth(config_.format, messages_per_hour);
    }
    
private:
    esp_mqtt_client_handle_t client_;
    Config config_;
    bool connected_;
    bool initialized_;
    
    // Statistics
    uint32_t messages_sent_;
    uint32_t bytes_sent_;
    
    // MQTT event handler
    static void mqtt_event_handler(void* handler_args, esp_event_base_t base,
                                   int32_t event_id, void* event_data);
    void HandleEvent(esp_mqtt_event_handle_t event);
};

#endif // MQTT_CLIENT_HH_