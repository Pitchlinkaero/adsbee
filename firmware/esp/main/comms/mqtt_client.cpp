#include "mqtt_client.hh"
#include "hal.hh"

static const char* TAG = "MQTT";

ADSBeeMQTTClient::ADSBeeMQTTClient() 
    : client_(nullptr), connected_(false), initialized_(false),
      messages_sent_(0), bytes_sent_(0) {
}

ADSBeeMQTTClient::~ADSBeeMQTTClient() {
    if (client_) {
        Disconnect();
        esp_mqtt_client_destroy(client_);
    }
}

void ADSBeeMQTTClient::mqtt_event_handler(void* handler_args, esp_event_base_t base,
                                           int32_t event_id, void* event_data) {
    ADSBeeMQTTClient* client = static_cast<ADSBeeMQTTClient*>(handler_args);
    esp_mqtt_event_handle_t event = static_cast<esp_mqtt_event_handle_t>(event_data);
    client->HandleEvent(event);
}

void ADSBeeMQTTClient::HandleEvent(esp_mqtt_event_handle_t event) {
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            CONSOLE_INFO(TAG, "Connected to MQTT broker (feed %d)", config_.feed_index);
            connected_ = true;
            break;
            
        case MQTT_EVENT_DISCONNECTED:
            CONSOLE_WARNING(TAG, "Disconnected from MQTT broker (feed %d)", config_.feed_index);
            connected_ = false;
            break;
            
        case MQTT_EVENT_ERROR:
            CONSOLE_ERROR(TAG, "MQTT error on feed %d", config_.feed_index);
            break;
            
        default:
            break;
    }
}

bool ADSBeeMQTTClient::Init(const Config& config) {
    if (initialized_) {
        return true;
    }
    
    config_ = config;
    
    // Build broker URI with port
    char full_uri[256];
    snprintf(full_uri, sizeof(full_uri), "mqtt://%s:%d", 
             config.broker_uri, config.broker_port);
    
    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.uri = full_uri;
    mqtt_cfg.credentials.client_id = config.client_id;
    mqtt_cfg.session.keepalive = 60;
    mqtt_cfg.network.disable_auto_reconnect = false;
    mqtt_cfg.network.reconnect_timeout_ms = 5000;
    
    client_ = esp_mqtt_client_init(&mqtt_cfg);
    if (!client_) {
        CONSOLE_ERROR(TAG, "Failed to initialize MQTT client for feed %d", config.feed_index);
        return false;
    }
    
    esp_err_t err = esp_mqtt_client_register_event(client_, ESP_EVENT_ANY_ID,
                                                    mqtt_event_handler, this);
    if (err != ESP_OK) {
        CONSOLE_ERROR(TAG, "Failed to register MQTT event handler: %s", esp_err_to_name(err));
        esp_mqtt_client_destroy(client_);
        client_ = nullptr;
        return false;
    }
    
    initialized_ = true;
    CONSOLE_INFO(TAG, "MQTT client initialized for %s:%d (format=%s)",
                 config.broker_uri, config.broker_port,
                 MQTTProtocol::FormatToString(config.format));
    return true;
}

bool ADSBeeMQTTClient::Connect() {
    if (!initialized_) {
        return false;
    }
    
    esp_err_t err = esp_mqtt_client_start(client_);
    if (err != ESP_OK) {
        CONSOLE_ERROR(TAG, "Failed to start MQTT client: %s", esp_err_to_name(err));
        return false;
    }
    
    return true;
}

void ADSBeeMQTTClient::Disconnect() {
    if (initialized_ && client_) {
        esp_mqtt_client_stop(client_);
        connected_ = false;
    }
}

bool ADSBeeMQTTClient::PublishPacket(const Decoded1090Packet& packet) {
    if (!connected_) {
        return false;
    }
    
    // Format message
    uint8_t buffer[MQTTProtocol::kMaxMessageSize];
    uint16_t len = MQTTProtocol::FormatPacket(packet, buffer, sizeof(buffer), config_.format);
    
    if (len == 0) {
        return false;
    }
    
    // Get topic
    uint32_t icao24 = packet.transponder_packet.aa_or_vs & 0xFFFFFF;
    char topic[MQTTProtocol::kMaxTopicSize];
    bool use_short = (config_.format == MQTTProtocol::FORMAT_BINARY);
    
    if (!MQTTProtocol::GetTopic(icao24, "raw", topic, sizeof(topic), use_short)) {
        return false;
    }
    
    // Add base topic if configured
    char full_topic[128];
    if (config_.base_topic && strlen(config_.base_topic) > 0) {
        snprintf(full_topic, sizeof(full_topic), "%s/%s", config_.base_topic, topic);
    } else {
        strncpy(full_topic, topic, sizeof(full_topic));
    }
    
    // Publish immediately with QoS 0 for lowest latency
    int msg_id = esp_mqtt_client_publish(client_, full_topic, (const char*)buffer, len, 0, false);
    
    if (msg_id >= 0) {
        messages_sent_++;
        bytes_sent_ += len;
        return true;
    }
    
    return false;
}

bool ADSBeeMQTTClient::PublishAircraft(const Aircraft& aircraft) {
    if (!connected_) {
        return false;
    }
    
    // Format message
    uint8_t buffer[MQTTProtocol::kMaxMessageSize];
    uint16_t len = MQTTProtocol::FormatAircraft(aircraft, buffer, sizeof(buffer), config_.format);
    
    if (len == 0) {
        return false;
    }
    
    // Get topic
    char topic[MQTTProtocol::kMaxTopicSize];
    bool use_short = (config_.format == MQTTProtocol::FORMAT_BINARY);
    
    if (!MQTTProtocol::GetTopic(aircraft.icao_address, "status", topic, sizeof(topic), use_short)) {
        return false;
    }
    
    // Add base topic if configured
    char full_topic[128];
    if (config_.base_topic && strlen(config_.base_topic) > 0) {
        snprintf(full_topic, sizeof(full_topic), "%s/%s", config_.base_topic, topic);
    } else {
        strncpy(full_topic, topic, sizeof(full_topic));
    }
    
    // Publish immediately
    int msg_id = esp_mqtt_client_publish(client_, full_topic, (const char*)buffer, len, 0, false);
    
    if (msg_id >= 0) {
        messages_sent_++;
        bytes_sent_ += len;
        return true;
    }
    
    return false;
}