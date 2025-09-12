/**
 * MQTT Integration for CommsManager::IPWANTask
 * 
 * Add this code to comms_ip.cpp to enable MQTT support.
 * This implementation prioritizes real-time delivery with no buffering delays.
 */

#include "mqtt_client.hh"

// Add these to the IPWANTask function:

void CommsManager::IPWANTask(void* pvParameters) {
    // ... existing code ...
    
    // MQTT clients for each feed (add at beginning of function)
    ADSBeeMQTTClient* mqtt_clients[SettingsManager::Settings::kMaxNumFeeds] = {nullptr};
    
    // Initialize MQTT clients for feeds configured with MQTT protocol
    for (uint16_t i = 0; i < SettingsManager::Settings::kMaxNumFeeds; i++) {
        if (settings_manager.settings.feed_protocols[i] == SettingsManager::ReportingProtocol::kMQTT) {
            // Create MQTT client for this feed
            mqtt_clients[i] = new ADSBeeMQTTClient();
            
            // Generate unique client ID
            char client_id[32];
            snprintf(client_id, sizeof(client_id), "ADSBee-%d-%06X", 
                     i, esp_random() & 0xFFFFFF);
            
            // Configure client
            ADSBeeMQTTClient::Config mqtt_config;
            mqtt_config.feed_index = i;
            mqtt_config.broker_uri = settings_manager.settings.feed_uris[i];
            mqtt_config.broker_port = settings_manager.settings.feed_ports[i];
            mqtt_config.client_id = client_id;
            mqtt_config.base_topic = (char*)settings_manager.settings.feed_receiver_ids[i];
            mqtt_config.format = static_cast<MQTTProtocol::Format>(
                settings_manager.settings.feed_mqtt_formats[i]);
            
            if (mqtt_clients[i]->Init(mqtt_config)) {
                CONSOLE_INFO("CommsManager::IPWANTask", 
                            "MQTT client initialized for feed %d (%s format)",
                            i, MQTTProtocol::FormatToString(mqtt_config.format));
            } else {
                delete mqtt_clients[i];
                mqtt_clients[i] = nullptr;
                CONSOLE_ERROR("CommsManager::IPWANTask", 
                             "Failed to initialize MQTT for feed %d", i);
            }
        }
    }
    
    // In the main loop where packets are sent:
    while (true) {
        // ... existing code to receive packets ...
        
        // Send to each feed
        for (uint16_t i = 0; i < SettingsManager::Settings::kMaxNumFeeds; i++) {
            if (!settings_manager.settings.feed_is_active[i]) {
                continue;
            }
            
            switch (settings_manager.settings.feed_protocols[i]) {
                // ... existing cases for Beast, etc ...
                
                case SettingsManager::ReportingProtocol::kMQTT: {
                    if (!mqtt_clients[i]) {
                        break;  // Client not initialized
                    }
                    
                    // Check if valid packet
                    if (!decoded_packet.IsValid()) {
                        break;
                    }
                    
                    // Connect if not connected
                    if (!mqtt_clients[i]->IsConnected()) {
                        // Only try to connect if we have network
                        if (wifi_sta_has_ip_ || ethernet_has_ip_) {
                            static uint32_t last_connect_attempt[SettingsManager::Settings::kMaxNumFeeds] = {0};
                            uint32_t now = get_time_since_boot_ms();
                            
                            // Retry every 5 seconds
                            if (now - last_connect_attempt[i] > 5000) {
                                CONSOLE_INFO("CommsManager::IPWANTask", 
                                            "Connecting MQTT feed %d to %s",
                                            i, settings_manager.settings.feed_uris[i]);
                                mqtt_clients[i]->Connect();
                                last_connect_attempt[i] = now;
                            }
                        }
                        break;  // Not connected yet
                    }
                    
                    // Publish immediately - no buffering!
                    if (mqtt_clients[i]->PublishPacket(decoded_packet)) {
                        feed_mps_counter_[i]++;  // Update statistics
                    } else {
                        CONSOLE_WARNING("CommsManager::IPWANTask",
                                       "Failed to publish to MQTT feed %d", i);
                    }
                    
                    break;
                }
                
                default:
                    break;
            }
        }
    }
    
    // Cleanup on exit (add at end of function)
    for (uint16_t i = 0; i < SettingsManager::Settings::kMaxNumFeeds; i++) {
        if (mqtt_clients[i]) {
            mqtt_clients[i]->Disconnect();
            delete mqtt_clients[i];
        }
    }
}

// AT Command Handler for MQTT Format
bool HandleMQTTFormat(const char* args, char* response, size_t response_size) {
    if (!args || strlen(args) == 0) {
        // Query current format
        snprintf(response, response_size, "+MQTTFORMAT:");
        for (int i = 0; i < SettingsManager::Settings::kMaxNumFeeds; i++) {
            char info[64];
            const char* format_str = 
                (settings_manager.settings.feed_mqtt_formats[i] == 
                 SettingsManager::Settings::MQTT_FORMAT_BINARY) ? "BINARY" : "JSON";
            snprintf(info, sizeof(info), "\r\n  Feed %d: %s", i, format_str);
            strncat(response, info, response_size - strlen(response) - 1);
        }
        return true;
    }
    
    // Parse arguments
    int feed_index;
    char format_str[32];
    
    if (sscanf(args, "%d,%31s", &feed_index, format_str) != 2) {
        snprintf(response, response_size, "ERROR: Use AT+MQTTFORMAT=<feed>,<JSON|BINARY>");
        return false;
    }
    
    if (feed_index < 0 || feed_index >= SettingsManager::Settings::kMaxNumFeeds) {
        snprintf(response, response_size, "ERROR: Invalid feed index (0-3)");
        return false;
    }
    
    // Set format
    if (strcasecmp(format_str, "BINARY") == 0) {
        settings_manager.settings.feed_mqtt_formats[feed_index] = 
            SettingsManager::Settings::MQTT_FORMAT_BINARY;
    } else {
        settings_manager.settings.feed_mqtt_formats[feed_index] = 
            SettingsManager::Settings::MQTT_FORMAT_JSON;
    }
    
    snprintf(response, response_size, "OK: Feed %d format set to %s",
            feed_index, format_str);
    return true;
}