#include "mqtt_client.hh"
#include <string.h>
#include "comms.hh"

static const char* TAG = "MQTT";

ADSBeeMQTTClient::ADSBeeMQTTClient() {}
ADSBeeMQTTClient::~ADSBeeMQTTClient() {
    if (client_) {
        esp_mqtt_client_stop(client_);
        esp_mqtt_client_destroy(client_);
    }
}

bool ADSBeeMQTTClient::Init(const Config& cfg) {
    if (initialized_) return true;
    cfg_ = cfg;
    snprintf(uri_, sizeof(uri_), "mqtt://%s:%u", cfg_.broker_host, (unsigned)cfg_.broker_port);
    strncpy(client_id_, cfg_.client_id ? cfg_.client_id : "ADSBee", sizeof(client_id_) - 1);
    client_id_[sizeof(client_id_) - 1] = '\0';
    strncpy(device_id_, cfg_.device_id ? cfg_.device_id : "", sizeof(device_id_) - 1);
    device_id_[sizeof(device_id_) - 1] = '\0';

    esp_mqtt_client_config_t mc = {};
    mc.broker.address.uri = uri_;
    mc.credentials.client_id = client_id_;
    mc.session.keepalive = 60;
    mc.network.disable_auto_reconnect = false;

    client_ = esp_mqtt_client_init(&mc);
    if (!client_) {
        CONSOLE_ERROR(TAG, "esp_mqtt_client_init failed");
        return false;
    }
    esp_mqtt_client_register_event(client_, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, ADSBeeMQTTClient::EventHandler, this);
    initialized_ = true;
    return true;
}

bool ADSBeeMQTTClient::Connect() {
    if (!initialized_ || !client_) return false;
    if (connected_) return true;
    if (esp_mqtt_client_start(client_) != ESP_OK) {
        CONSOLE_ERROR(TAG, "esp_mqtt_client_start failed");
        return false;
    }
    return true;
}

void ADSBeeMQTTClient::Disconnect() {
    if (client_) {
        esp_mqtt_client_stop(client_);
        connected_ = false;
    }
}

void ADSBeeMQTTClient::EventHandler(void* handler_args, esp_event_base_t base, int32_t event_id, void* event_data) {
    ADSBeeMQTTClient* self = static_cast<ADSBeeMQTTClient*>(handler_args);
    self->HandleEvent((esp_mqtt_event_handle_t)event_data);
}

void ADSBeeMQTTClient::HandleEvent(esp_mqtt_event_handle_t event) {
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            connected_ = true;
            CONSOLE_INFO(TAG, "connected");
            break;
        case MQTT_EVENT_DISCONNECTED:
            connected_ = false;
            CONSOLE_WARNING(TAG, "disconnected");
            break;
        case MQTT_EVENT_ERROR:
            CONSOLE_ERROR(TAG, "error event");
            break;
        default:
            break;
    }
}

bool ADSBeeMQTTClient::PublishAircraftJSON(const Aircraft1090& ac, MQTTProtocol::FrequencyBand band) {
    if (!connected_) return false;
    char topic[MQTTProtocol::kMaxTopicLen];
    if (!MQTTProtocol::GetTopic(ac.icao_address, "status", topic, sizeof(topic), band, device_id_, /*short*/ false)) {
        return false;
    }
    char json[MQTTProtocol::kMaxJSONLen];
    uint16_t len = MQTTProtocol::FormatAircraftStatusJSON(ac, band, json, sizeof(json));
    if (len == 0) return false;
    int mid = esp_mqtt_client_publish(client_, topic, json, len, /*qos*/0, /*retain*/0);
    return (mid >= 0);
}


