#include "beast/beast_utils.hh"  // For beast reporting.
#include "comms.hh"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_mac.h"
#include "hal.hh"
#include "lwip/dns.h"
#include "lwip/err.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "mdns.h"
#include "mqtt_client.hh"              // For MQTT support.
#include "adsbee_server.hh"            // For aircraft_dictionary access.
#include "object_dictionary.hh"        // For device_status, firmware version constants.
#include "task_priorities.hh"

// Compile-time check that Settings::MQTTFormat and MQTTProtocol::Format enum values match.
static_assert(static_cast<uint8_t>(SettingsManager::Settings::MQTT_FORMAT_JSON) ==
              static_cast<uint8_t>(MQTTProtocol::FORMAT_JSON),
              "Settings::MQTT_FORMAT_JSON must match MQTTProtocol::FORMAT_JSON");
static_assert(static_cast<uint8_t>(SettingsManager::Settings::MQTT_FORMAT_BINARY) ==
              static_cast<uint8_t>(MQTTProtocol::FORMAT_BINARY),
              "Settings::MQTT_FORMAT_BINARY must match MQTTProtocol::FORMAT_BINARY");

static const uint32_t kTCPSocketReconnectIntervalMs = 10000;

// Heap threshold for back-pressure. If free heap drops below this, safe_send will block.
// Set high enough to leave room for WebSocket allocations (~2KB) and other system needs.
static const uint32_t kHeapBackPressureThresholdBytes = 20480;
// DMA-capable memory threshold. WiFi TX buffers require internal DMA memory which is more constrained.
static const uint32_t kDMABackPressureThresholdBytes = 16384;
static const uint32_t kHeapBackPressureCheckIntervalMs = 50;
static const uint32_t kHeapBackPressureTimeoutMs = 5000;

// #define ENABLE_TCP_SOCKET_RATE_LIMITING
#ifdef ENABLE_TCP_SOCKET_RATE_LIMITING
static const uint32_t kSendBufRateLimitBytesPerSecond = 100000;  // 100 kB/s
static const uint32_t kSendBufRateLimitIntervalMs = 10;
static const uint32_t kSendBufRateLimitBytesPerInterval =
    kSendBufRateLimitBytesPerSecond / (1000 / kSendBufRateLimitIntervalMs);
static const uint32_t kRateLimitDelayDurationMs = 1;
#endif

static const uint32_t kTCPKeepAliveEnable = 1;
static const uint32_t kTCPKeepAliveIdleSecondsBeforeStartingProbe = 120;
static const uint32_t kTCPKeepAliveIntervalSecondsBetweenProbes = 30;
static const uint32_t kTCPKeepAliveMaxFailedProbesBeforeDisconnect = 3;
static const uint32_t kTCPReuseAddrEnable =
    1;  // Allow reuse of local addresses and sockets that are in the TIME_WAIT state.

/** "Pass-Through" functions used to access member functions in callbacks. **/
void ip_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    comms_manager.IPEventHandler(arg, event_base, event_id, event_data);
}
void ip_wan_task(void* pvParameters) { comms_manager.IPWANTask(pvParameters); }
/** End "Pass-Through" functions. **/

bool IsNotIPAddress(const char* uri) {
    // Check if the URI contains any letters
    for (const char* p = uri; *p != '\0'; p++) {
        if (isalpha(*p)) {
            return true;
        }
    }
    return false;
}

bool ResolveURIToIP(const char* url, char* ip) {
    struct addrinfo hints;
    struct addrinfo* res;
    struct in_addr addr;
    int err;

    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;

    err = getaddrinfo(url, NULL, &hints, &res);
    if (err != 0 || res == NULL) {
        CONSOLE_ERROR("ResolveURLToIP", "DNS lookup failed for %s: %d", url, err);
        freeaddrinfo(res);
        return false;
    }

    addr.s_addr = ((struct sockaddr_in*)res->ai_addr)->sin_addr.s_addr;
    inet_ntop(AF_INET, &addr, ip, 16);
    CONSOLE_INFO("ResolveURLToIP", "DNS lookup succeeded. IP=%s", ip);

    freeaddrinfo(res);
    return true;
}

esp_err_t safe_send(int sock, const void* data, size_t total_len) {
    // 0. Back-pressure: block if heap or DMA memory is low to prevent OOM crashes
    uint32_t backpressure_start_ms = get_time_since_boot_ms();
    while (heap_caps_get_free_size(MALLOC_CAP_8BIT) < kHeapBackPressureThresholdBytes ||
           heap_caps_get_free_size(MALLOC_CAP_DMA) < kDMABackPressureThresholdBytes) {
        if (get_time_since_boot_ms() - backpressure_start_ms > kHeapBackPressureTimeoutMs) {
            CONSOLE_WARNING("safe_send", "Heap back-pressure timeout after %lu ms (heap=%u, dma=%u)",
                            kHeapBackPressureTimeoutMs, heap_caps_get_free_size(MALLOC_CAP_8BIT),
                            heap_caps_get_free_size(MALLOC_CAP_DMA));
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(kHeapBackPressureCheckIntervalMs));
    }

    // 1. Set socket to non-blocking mode
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    size_t sent_total = 0;
    while (sent_total < total_len) {
        fd_set write_set;
        FD_ZERO(&write_set);
        FD_SET(sock, &write_set);

        // 2. Set a timeout for select()
        struct timeval timeout = {.tv_sec = 5,  // 5 second timeout
                                  .tv_usec = 0};

        // Wait until the socket is ready to accept data
        int res = select(sock + 1, NULL, &write_set, NULL, &timeout);

        if (res < 0) {
            CONSOLE_ERROR("safe_send", "Select error: %d", errno);
            return ESP_FAIL;
        } else if (res == 0) {
            CONSOLE_WARNING("safe_send", "Send timeout - Network congested");
            return ESP_ERR_TIMEOUT;
        }

        // 3. Socket is ready, try to send the remaining chunk
        int sent_now = send(sock, (const char*)data + sent_total, total_len - sent_total, 0);

        if (sent_now > 0) {
            sent_total += sent_now;
            // CONSOLE_INFO("safe_send", "Sent %d bytes (%zu/%zu)", sent_now, sent_total, total_len);
        } else if (sent_now < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // This shouldn't really happen after select() says it's ready,
                // but it's good practice to handle it.
                continue;
            } else {
                CONSOLE_ERROR("safe_send", "Socket error during send: %d", errno);
                return ESP_FAIL;
            }
        }
    }

    return ESP_OK;
}

bool CommsManager::IPInit() {
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, &ip_event_handler, NULL));
    ip_event_handler_was_initialized_ = true;

    // IP WAN task stack size reduced since raw_packets_buf is now allocated on heap instead of stack.
    xTaskCreate(ip_wan_task, "ip_wan_task", 2 * 4096, &ip_wan_task_handle, kIPWANTaskPriority, NULL);

    // Initialize mDNS service.
    esp_err_t err = mdns_init();
    if (err) {
        CONSOLE_ERROR("CommsManager::IPInit", "MDNS Init failed: %d\n", err);
        return false;
    }

    // Set hostname.
    err = mdns_hostname_set(settings_manager.settings.core_network_settings.hostname);
    if (err) {
        CONSOLE_ERROR("CommsManager::IPInit", "Failed setting MDNS hostname to %s: %d\n",
                      settings_manager.settings.core_network_settings.hostname, err);
        return false;
    }
    // Set default instance.
    mdns_instance_name_set("ADSBee 1090");

    CONSOLE_INFO("CommsManager::IPInit", "MDNS initialized with hostname %s.\n",
                 settings_manager.settings.core_network_settings.hostname);
    return true;
}

void CommsManager::IPEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    switch (event_id) {
        case IP_EVENT_AP_STAIPASSIGNED: {
            // A new station has connected to the ADSBee's softAP network.
            ip_event_ap_staipassigned_t* event = (ip_event_ap_staipassigned_t*)event_data;

            char client_mac_str[SettingsManager::Settings::kMACAddrStrLen + 1] = {0};
            char client_ip_str[SettingsManager::Settings::kIPAddrStrLen + 1] = {0};
            snprintf(client_mac_str, SettingsManager::Settings::kMACAddrStrLen, MACSTR, MAC2STR(event->mac));
            client_mac_str[SettingsManager::Settings::kMACAddrStrLen] = '\0';
            snprintf(client_ip_str, SettingsManager::Settings::kIPAddrStrLen, IPSTR, IP2STR(&event->ip));
            client_ip_str[SettingsManager::Settings::kIPAddrStrLen] = '\0';

            CONSOLE_INFO("CommsManager::IPEventHandler",
                         "WiFi Access Point assigned IP address to client. IP: %s, MAC: %s", client_ip_str,
                         client_mac_str);

            WiFiAddClient(event->ip, event->mac);
            break;
        }
        case IP_EVENT_STA_GOT_IP: {
            // The ADSBee has connected to an external network as a WiFi station.
            ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
            const esp_netif_ip_info_t* ip_info = &event->ip_info;

            wifi_sta_has_ip_ = true;
            snprintf(wifi_sta_ip, SettingsManager::Settings::kIPAddrStrLen, IPSTR, IP2STR(&ip_info->ip));
            wifi_sta_ip[SettingsManager::Settings::kIPAddrStrLen] = '\0';
            snprintf(wifi_sta_netmask, SettingsManager::Settings::kIPAddrStrLen, IPSTR, IP2STR(&ip_info->netmask));
            wifi_sta_netmask[SettingsManager::Settings::kIPAddrStrLen] = '\0';
            snprintf(wifi_sta_gateway, SettingsManager::Settings::kIPAddrStrLen, IPSTR, IP2STR(&ip_info->gw));
            wifi_sta_gateway[SettingsManager::Settings::kIPAddrStrLen] = '\0';

            CONSOLE_INFO("CommsManager::IPEventHandler",
                         "WiFi Station got IP Address. IP: %s, Netmask: %s, Gateway: %s", wifi_sta_ip, wifi_sta_netmask,
                         wifi_sta_gateway);
            break;
        }
        case IP_EVENT_ETH_GOT_IP: {
            // The ADSBee's Ethernet interface has connected to an external network.
            ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
            const esp_netif_ip_info_t* ip_info = &event->ip_info;

            ethernet_has_ip_ = true;
            snprintf(ethernet_ip, SettingsManager::Settings::kIPAddrStrLen, IPSTR, IP2STR(&ip_info->ip));
            ethernet_ip[SettingsManager::Settings::kIPAddrStrLen] = '\0';
            snprintf(ethernet_netmask, SettingsManager::Settings::kIPAddrStrLen, IPSTR, IP2STR(&ip_info->netmask));
            ethernet_netmask[SettingsManager::Settings::kIPAddrStrLen] = '\0';
            snprintf(ethernet_gateway, SettingsManager::Settings::kIPAddrStrLen, IPSTR, IP2STR(&ip_info->gw));
            ethernet_gateway[SettingsManager::Settings::kIPAddrStrLen] = '\0';

            CONSOLE_INFO("CommsManager::IPEventHandler", "Ethernet got IP address. IP: %s, Netmask: %s, Gateway: %s",
                         ethernet_ip, ethernet_netmask, ethernet_gateway);
            break;
        }
        case IP_EVENT_ETH_LOST_IP: {
            // The ADSBee's Ethernet interface has disconnected from an external network.
            ethernet_has_ip_ = false;
            CONSOLE_INFO("CommsManager::IPEventHandler", "Ethernet lost IP address.");
            break;
        }
    }
}

void CommsManager::IPWANTask(void* pvParameters) {
    // MQTT clients for each feed (local to task, dynamically allocated).
    ADSBeeMQTTClient* mqtt_clients[SettingsManager::Settings::kMaxNumFeeds] = {nullptr};
    uint32_t mqtt_last_connect_attempt[SettingsManager::Settings::kMaxNumFeeds] = {0};

    CONSOLE_INFO("CommsManager::IPWANTask", "IP WAN Task started.");
    
    // Wait a bit for system to stabilize before initializing MQTT
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Initialize MQTT clients for feeds configured with MQTT protocol
    for (uint16_t i = 0; i < SettingsManager::Settings::kMaxNumFeeds; i++) {
        // Only initialize MQTT if the feed is active AND configured for MQTT
        if (settings_manager.settings.feed_is_active[i] &&
            settings_manager.settings.feed_protocols[i] == SettingsManager::ReportingProtocol::kMQTT) {
            
            // Validate feed configuration
            if (strlen(settings_manager.settings.feed_uris[i]) == 0) {
                CONSOLE_WARNING("CommsManager::IPWANTask", 
                               "Feed %d configured for MQTT but no URI set", i);
                continue;
            }
            
            mqtt_clients[i] = new ADSBeeMQTTClient();
            if (!mqtt_clients[i]) {
                CONSOLE_ERROR("CommsManager::IPWANTask", 
                             "Failed to allocate MQTT client for feed %d", i);
                continue;
            }
            
            // Generate unique client ID
            char client_id[32];
            snprintf(client_id, sizeof(client_id), "ADSBee-%d-%06X", 
                     i, (unsigned int)(esp_random() & 0xFFFFFF));
            
            // Convert receiver ID to hex string for device ID
            char device_id[17];  // 8 bytes * 2 + null terminator
            snprintf(device_id, sizeof(device_id), "%02x%02x%02x%02x%02x%02x%02x%02x",
                    settings_manager.settings.feed_receiver_ids[i][0],
                    settings_manager.settings.feed_receiver_ids[i][1],
                    settings_manager.settings.feed_receiver_ids[i][2],
                    settings_manager.settings.feed_receiver_ids[i][3],
                    settings_manager.settings.feed_receiver_ids[i][4],
                    settings_manager.settings.feed_receiver_ids[i][5],
                    settings_manager.settings.feed_receiver_ids[i][6],
                    settings_manager.settings.feed_receiver_ids[i][7]);
            
            // Configure client
            ADSBeeMQTTClient::Config mqtt_config;
            mqtt_config.feed_index = i;
            mqtt_config.broker_uri = settings_manager.settings.feed_uris[i];
            mqtt_config.broker_port = settings_manager.settings.feed_ports[i];
            mqtt_config.client_id = client_id;
            mqtt_config.device_id = device_id;
            mqtt_config.format = static_cast<MQTTProtocol::Format>(
                settings_manager.settings.feed_mqtt_formats[i]);
            mqtt_config.mqtt_content = settings_manager.settings.feed_mqtt_content[i];
            mqtt_config.ota_enabled = settings_manager.settings.mqtt_ota_enabled[i];

            if (mqtt_clients[i]->Init(mqtt_config)) {
                CONSOLE_INFO("CommsManager::IPWANTask", 
                            "MQTT client initialized for feed %d", i);
            } else {
                delete mqtt_clients[i];
                mqtt_clients[i] = nullptr;
                CONSOLE_ERROR("CommsManager::IPWANTask", 
                             "Failed to initialize MQTT for feed %d", i);
            }
        }
    }

    // Telemetry publishing interval (ms)
    static const uint32_t kTelemetryIntervalMs = 60000;
    uint32_t last_telemetry_publish_ms[SettingsManager::Settings::kMaxNumFeeds] = {0};

    // Aircraft status publishing interval (ms)
    static const uint32_t kAircraftStatusIntervalMs = 5000;
    uint32_t last_aircraft_publish_ms[SettingsManager::Settings::kMaxNumFeeds] = {0};

    uint8_t* raw_packets_buf = (uint8_t*)heap_caps_malloc(CompositeArray::RawPackets::kMaxLenBytes, MALLOC_CAP_8BIT);
    if (raw_packets_buf == nullptr) {
        CONSOLE_ERROR("CommsManager::IPWANTask", "Failed to allocate raw packets buffer on heap. Task exiting.");
        vTaskDelete(NULL);
        return;
    }

    while (true) {
        // Don't try establishing socket connections until the ESP32 has been assigned an IP address.
        while (!HasIP()) {
            vTaskDelay(1);  // Delay for 1 tick.
        }

        UpdateFeedMetrics();

        uint16_t num_active_report_sinks = 0;
        ReportSink active_report_sinks[SettingsManager::Settings::kMaxNumFeeds] = {0};
        SettingsManager::ReportingProtocol active_reporting_protocols[SettingsManager::Settings::kMaxNumFeeds] = {
            SettingsManager::ReportingProtocol::kNoReports};

        // Maintain socket connections and build list of acive report sinks.
        for (uint16_t i = 0; i < SettingsManager::Settings::kMaxNumFeeds; i++) {
            // Iterate through feeds, open/close and send message as required.
            if (!settings_manager.settings.feed_is_active[i]) {
                // Feed is not active, ensure socket is closed.
                if (feed_sock_is_connected_[i]) {
                    // Need to close the socket connection.
                    CloseFeedSocket(i);
                }
                continue;
            } else {
                // MQTT feeds manage their own connection via the ESP-IDF MQTT library,
                // so skip raw TCP socket management for them.
                if (settings_manager.settings.feed_protocols[i] == SettingsManager::ReportingProtocol::kMQTT) {
                    continue;
                }
                // Feed is active, ensure socket is open and add to active sinks if connected.
                if (!feed_sock_is_connected_[i] && !ConnectFeedSocket(i)) {
                    // Need to open the socket connection, but failed to do so.
                    continue;  // Failed to connect, try again later.
                }
                // Socket is connected, add to active sinks.
                active_report_sinks[num_active_report_sinks] = i;
                active_reporting_protocols[num_active_report_sinks] = settings_manager.settings.feed_protocols[i];
                num_active_report_sinks++;
            }
        }

        // Proactively attempt MQTT connection for configured feeds, independent of packet flow.
        for (uint16_t i = 0; i < SettingsManager::Settings::kMaxNumFeeds; i++) {
            if (!mqtt_clients[i]) {
                continue;
            }
            if (settings_manager.settings.feed_protocols[i] != SettingsManager::ReportingProtocol::kMQTT) {
                continue;
            }
            if (!settings_manager.settings.feed_is_active[i]) {
                continue;
            }
            if (!mqtt_clients[i]->IsConnected()) {
                uint32_t now = get_time_since_boot_ms();
                if (now - mqtt_last_connect_attempt[i] > 5000) {
                    CONSOLE_INFO("CommsManager::IPWANTask",
                                 "MQTT feed %d: connecting to %s (heap=%u)",
                                 i, settings_manager.settings.feed_uris[i],
                                 heap_caps_get_free_size(MALLOC_CAP_8BIT));
                    mqtt_clients[i]->Connect();
                    mqtt_last_connect_attempt[i] = now;
                }
            }
        }

        // Drain all queued raw packets first, before slow periodic MQTT work.
        // This prevents the 3-element queue from overflowing during aircraft status bursts.
        while (xQueueReceive(ip_wan_reporting_composite_array_queue_, raw_packets_buf, 0) == pdTRUE) {
            CompositeArray::RawPackets reporting_composite_array;
            if (!CompositeArray::UnpackRawPacketsBuffer(reporting_composite_array, raw_packets_buf,
                                                        CompositeArray::RawPackets::kMaxLenBytes)) {
                CONSOLE_ERROR("CommsManager::IPWANTask", "Failed to unpack CompositeArray from buffer.");
                continue;
            }

            // Update feeds with raw and digested reports (handles TCP-based protocols).
            if (!UpdateReporting(active_report_sinks, active_reporting_protocols, num_active_report_sinks,
                                 &reporting_composite_array)) {
                CONSOLE_ERROR("CommsManager::IPWANTask", "Error during UpdateReporting for feeds.");
            }

            // Publish raw packets to MQTT feeds (separate from TCP-based UpdateReporting flow).
            for (uint16_t i = 0; i < SettingsManager::Settings::kMaxNumFeeds; i++) {
                if (!mqtt_clients[i] || !mqtt_clients[i]->IsConnected()) {
                    continue;
                }
                if (settings_manager.settings.feed_protocols[i] != SettingsManager::ReportingProtocol::kMQTT) {
                    continue;
                }
                if (!settings_manager.settings.feed_is_active[i]) {
                    continue;
                }
                // Only publish raw packets if content mode is ALL or RAW.
                uint8_t content = mqtt_clients[i]->GetConfig().mqtt_content;
                if (content != SettingsManager::Settings::MQTT_CONTENT_ALL &&
                    content != SettingsManager::Settings::MQTT_CONTENT_RAW) {
                    continue;
                }
                // Iterate through Mode S packets in the composite array and publish each one.
                for (uint16_t p = 0; p < reporting_composite_array.header->num_mode_s_packets; p++) {
                    DecodedModeSPacket decoded_packet(reporting_composite_array.mode_s_packets[p]);
                    if (!decoded_packet.is_valid) {
                        continue;
                    }
                    MQTTProtocol::FrequencyBand band = MQTTProtocol::BAND_1090_MHZ;
                    if (mqtt_clients[i]->PublishPacket(decoded_packet, band)) {
                        feed_mps_counter_[i]++;
                    }
                }
                // Iterate through UAT ADS-B packets and publish each one.
                for (uint16_t p = 0; p < reporting_composite_array.header->num_uat_adsb_packets; p++) {
                    DecodedUATADSBPacket decoded_packet(reporting_composite_array.uat_adsb_packets[p]);
                    if (!decoded_packet.is_valid) {
                        continue;
                    }
                    if (mqtt_clients[i]->PublishUATPacket(decoded_packet)) {
                        feed_mps_counter_[i]++;
                    }
                }
            }
        }

        // Periodic telemetry publish on connected MQTT feeds
        for (uint16_t i = 0; i < SettingsManager::Settings::kMaxNumFeeds; i++) {
            if (!mqtt_clients[i]) {
                continue;
            }
            if (settings_manager.settings.feed_protocols[i] != SettingsManager::ReportingProtocol::kMQTT) {
                continue;
            }
            if (!settings_manager.settings.feed_is_active[i]) {
                continue;
            }
            if (!mqtt_clients[i]->IsConnected()) {
                continue;
            }
            uint32_t now = get_time_since_boot_ms();
            if (now - last_telemetry_publish_ms[i] >= kTelemetryIntervalMs) {
                MQTTProtocol::TelemetryData t = {};
                t.uptime_sec = now / 1000;
                uint32_t total_mps = 0;
                uint8_t mps_count = 0;
                for (uint16_t j = 0; j < SettingsManager::Settings::kMaxNumFeeds; j++) {
                    total_mps += feed_mps[j];
                    if (mps_count < MQTTProtocol::TelemetryData::kMaxFeedsForTelemetry) {
                        t.mps_feeds[mps_count++] = feed_mps[j];
                    }
                }
                t.messages_received = (uint16_t)MIN(total_mps, (uint32_t)0xFFFF);
                t.mps_total = (uint16_t)MIN(total_mps, (uint32_t)0xFFFF);
                t.mps_feed_count = mps_count;
                // Use per-client cumulative MQTT messages sent
                t.messages_sent = (uint16_t)MIN(mqtt_clients[i]->GetMessagesSent(), (uint32_t)0xFFFF);
                // ESP32 temperature from device_status_update_task (already running in app_main.cpp)
                t.cpu_temp_c = object_dictionary.device_status.temperature_deg_c;
                // RP2040 metrics from composite device status (sent via SPI)
                t.pico_temp_c = object_dictionary.composite_device_status.rp2040.temperature_deg_c;
                t.pico_core0_pct = object_dictionary.composite_device_status.rp2040.core_0_usage_percent;
                t.pico_core1_pct = object_dictionary.composite_device_status.rp2040.core_1_usage_percent;
                // Aircraft count
                t.aircraft_count = (uint16_t)adsbee_server.aircraft_dictionary.dict.size();
                // Free heap (8-bit accessible)
                t.memory_free_kb = (uint16_t)(heap_caps_get_free_size(MALLOC_CAP_8BIT) / 1024);
                // Noise floor not measured on ESP32 side
                t.rssi_noise_floor_dbm = 0;
                t.receiver_1090_enabled = 1;
                t.receiver_978_enabled = 0;
                t.wifi_connected = wifi_sta_has_ip_ ? 1 : 0;
                t.mqtt_connected = 1;
                // Firmware version from ObjectDictionary constants
                t.fw_major = ObjectDictionary::kFirmwareVersionMajor;
                t.fw_minor = ObjectDictionary::kFirmwareVersionMinor;
                t.fw_patch = ObjectDictionary::kFirmwareVersionPatch;

                if (mqtt_clients[i]->PublishTelemetry(t)) {
                    // Published OK; no counter increment (feed_mps is packet traffic)
                } else {
                    CONSOLE_WARNING("CommsManager::IPWANTask",
                                    "Failed to publish telemetry on MQTT feed %d", i);
                }
                last_telemetry_publish_ms[i] = now;
            }
        }

        // Periodically publish decoded aircraft status from the aircraft dictionary.
        for (uint16_t i = 0; i < SettingsManager::Settings::kMaxNumFeeds; i++) {
            if (!mqtt_clients[i] || !mqtt_clients[i]->IsConnected()) continue;
            if (settings_manager.settings.feed_protocols[i] != SettingsManager::ReportingProtocol::kMQTT) continue;
            if (!settings_manager.settings.feed_is_active[i]) continue;

            // Only publish aircraft status if content mode is ALL or STATUS.
            uint8_t content = mqtt_clients[i]->GetConfig().mqtt_content;
            if (content != SettingsManager::Settings::MQTT_CONTENT_ALL &&
                content != SettingsManager::Settings::MQTT_CONTENT_STATUS) {
                continue;
            }

            uint32_t now = get_time_since_boot_ms();
            if (now - last_aircraft_publish_ms[i] < kAircraftStatusIntervalMs) continue;

            uint32_t cutoff = last_aircraft_publish_ms[i];  // Previous publish time = cutoff for dedup.
            last_aircraft_publish_ms[i] = now;              // Update for next round.
            for (auto& itr : adsbee_server.aircraft_dictionary.dict) {
                if (ModeSAircraft* ac = get_if<ModeSAircraft>(&(itr.second)); ac) {
                    if (!ac->HasBitFlag(ModeSAircraft::kBitFlagPositionValid)) continue;
                    if (ac->last_message_timestamp_ms <= cutoff) continue;
                    mqtt_clients[i]->PublishAircraft(*ac, MQTTProtocol::BAND_1090_MHZ);
                } else if (UATAircraft* uat = get_if<UATAircraft>(&(itr.second)); uat) {
                    if (!uat->HasBitFlag(UATAircraft::kBitFlagPositionValid)) continue;
                    if (uat->last_message_timestamp_ms <= cutoff) continue;
                    // Populate a temporary ModeSAircraft with UAT data for formatting.
                    ModeSAircraft tmp;
                    tmp.icao_address = uat->icao_address;
                    strncpy(tmp.callsign, uat->callsign, sizeof(tmp.callsign));
                    tmp.latitude_deg = uat->latitude_deg;
                    tmp.longitude_deg = uat->longitude_deg;
                    tmp.baro_altitude_ft = uat->baro_altitude_ft;
                    tmp.direction_deg = uat->direction_deg;
                    tmp.speed_kts = uat->speed_kts;
                    tmp.baro_vertical_rate_fpm = uat->baro_vertical_rate_fpm;
                    tmp.squawk = uat->squawk;
                    tmp.emitter_category_raw = uat->emitter_category_raw;
                    tmp.last_message_signal_strength_dbm = uat->last_message_signal_strength_dbm;
                    tmp.last_message_timestamp_ms = uat->last_message_timestamp_ms;
                    // Map UAT flags to ModeSAircraft flags (same bit positions for shared flags).
                    if (uat->HasBitFlag(UATAircraft::kBitFlagIsAirborne))
                        tmp.flags |= (1 << ModeSAircraft::kBitFlagIsAirborne);
                    if (uat->HasBitFlag(UATAircraft::kBitFlagIdent))
                        tmp.flags |= (1 << ModeSAircraft::kBitFlagIdent);
                    if (uat->HasBitFlag(UATAircraft::kBitFlagTCASRA))
                        tmp.flags |= (1 << ModeSAircraft::kBitFlagTCASRA);
                    mqtt_clients[i]->PublishAircraft(tmp, MQTTProtocol::BAND_978_MHZ);
                }
            }
        }

        // Wait for next iteration. This is where the task sleeps when the queue is empty.
        vTaskDelay(kWiFiSTATaskUpdateIntervalTicks);
    }

    // Close all sockets while exiting.
    for (uint16_t i = 0; i < SettingsManager::Settings::kMaxNumFeeds; i++) {
        CloseFeedSocket(i);
    }

    // Free heap buffer.
    heap_caps_free(raw_packets_buf);

    CONSOLE_INFO("CommsManager::IPWANTask", "IP WAN Task exiting.");
}

void CommsManager::CloseFeedSocket(uint16_t feed_index) {
    // Need to close the socket connection.
    close(feed_sock_[feed_index]);
    feed_sock_is_connected_[feed_index] = false;
    CONSOLE_INFO("CommsManager::IPWANTask", "Closed socket for feed %d.", feed_index);
}

bool CommsManager::ConnectFeedSocket(uint16_t feed_index) {
    // Meter reconnect attempt interval.
    uint32_t timestamp_ms = get_time_since_boot_ms();
    if (timestamp_ms - feed_sock_last_connect_timestamp_ms_[feed_index] <= kTCPSocketReconnectIntervalMs) {
        return false;
    }
    feed_sock_last_connect_timestamp_ms_[feed_index] = timestamp_ms;

    // Create socket.
    // IPv4, TCP
    feed_sock_[feed_index] = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (feed_sock_[feed_index] <= 0) {
        CONSOLE_ERROR("CommsManager::IPWANTask", "Unable to create socket for feed %d: errno %d (%s)", feed_index,
                      errno, strerror(errno));
        CloseFeedSocket(feed_index);
        return false;
    }
    CONSOLE_INFO("CommsManager::IPWANTask", "Socket for feed %d created, connecting to %s:%d", feed_index,
                 settings_manager.settings.feed_uris[feed_index], settings_manager.settings.feed_ports[feed_index]);

    // Enable TCP keepalive
    setsockopt(feed_sock_[feed_index], SOL_SOCKET, SO_KEEPALIVE, &kTCPKeepAliveEnable, sizeof(kTCPKeepAliveEnable));
    setsockopt(feed_sock_[feed_index], IPPROTO_TCP, TCP_KEEPIDLE, &kTCPKeepAliveIdleSecondsBeforeStartingProbe,
               sizeof(kTCPKeepAliveIdleSecondsBeforeStartingProbe));
    setsockopt(feed_sock_[feed_index], IPPROTO_TCP, TCP_KEEPINTVL, &kTCPKeepAliveIntervalSecondsBetweenProbes,
               sizeof(kTCPKeepAliveIntervalSecondsBetweenProbes));
    setsockopt(feed_sock_[feed_index], IPPROTO_TCP, TCP_KEEPCNT, &kTCPKeepAliveMaxFailedProbesBeforeDisconnect,
               sizeof(kTCPKeepAliveMaxFailedProbesBeforeDisconnect));
    // Allow reuse of local addresses.
    setsockopt(feed_sock_[feed_index], SOL_SOCKET, SO_REUSEADDR, &kTCPReuseAddrEnable, sizeof(kTCPReuseAddrEnable));

    struct sockaddr_in dest_addr;
    // If the URI contains letters, resolve it to an IP address
    if (IsNotIPAddress(settings_manager.settings.feed_uris[feed_index])) {
        // Is not an IP address, try DNS resolution.
        char resolved_ip[16];
        if (!ResolveURIToIP(settings_manager.settings.feed_uris[feed_index], resolved_ip)) {
            CONSOLE_ERROR("CommsManager::IPWANTask", "Failed to resolve URL %s for feed %d",
                          settings_manager.settings.feed_uris[feed_index], feed_index);
            CloseFeedSocket(feed_index);
            return false;
        }
        inet_pton(AF_INET, resolved_ip, &dest_addr.sin_addr);
    } else {
        // Is an IP address, use it directly.
        inet_pton(AF_INET, settings_manager.settings.feed_uris[feed_index], &dest_addr.sin_addr);
    }

    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(settings_manager.settings.feed_ports[feed_index]);

    int err = connect(feed_sock_[feed_index], (struct sockaddr*)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        CONSOLE_ERROR("CommsManager::IPWANTask", "Socket unable to connect to URI %s:%d for feed %d: errno %d (%s)",
                      settings_manager.settings.feed_uris[feed_index], settings_manager.settings.feed_ports[feed_index],
                      feed_index, errno, strerror(errno));
        CloseFeedSocket(feed_index);
        return false;
    }
    CONSOLE_INFO("CommsManager::IPWANTask", "Successfully connected to %s",
                 settings_manager.settings.feed_uris[feed_index]);
    feed_sock_is_connected_[feed_index] = true;

    // Perform beginning-of-connection actions here.
    switch (settings_manager.settings.feed_protocols[feed_index]) {
        case SettingsManager::ReportingProtocol::kBeast:
        case SettingsManager::ReportingProtocol::kBeastNoUAT:
        case SettingsManager::ReportingProtocol::kBeastNoUATUplink: {
            uint8_t beast_message_buf[2 * SettingsManager::Settings::kFeedReceiverIDNumBytes +
                                      BeastReporter::kModeSBeastFrameMaxLenBytes];
            uint16_t beast_message_len_bytes = BeastReporter::BuildFeedStartFrame(
                beast_message_buf, settings_manager.settings.feed_receiver_ids[feed_index]);
            int err = safe_send(feed_sock_[feed_index], beast_message_buf, beast_message_len_bytes);
            if (err < 0) {
                CONSOLE_ERROR("CommsManager::IPWANTask",
                              "Error occurred while sending %d Byte Beast start of feed message to feed %d "
                              "with URI %s "
                              "on port %d: "
                              "errno %d.",
                              beast_message_len_bytes, feed_index, settings_manager.settings.feed_uris[feed_index],
                              settings_manager.settings.feed_ports[feed_index], errno);
                // Mark socket as disconnected and try reconnecting in next reporting interval.
                CloseFeedSocket(feed_index);
                return false;
            }
            break;
        }
        default:
            // No start of connections actions required for other protocols.
            break;
    }
    return true;
}

// Rate limit the SendBuf function by keeping a running count of bytes sent in every 10ms interval.
#ifdef ENABLE_TCP_SOCKET_RATE_LIMITING
static uint32_t last_send_buf_counter_reset_timestamp_ms = 0;
static uint32_t send_buf_counter_bytes = 0;
#endif
bool CommsManager::SendBuf(uint16_t iface, const char* buf, uint16_t buf_len) {
    if (iface >= SettingsManager::Settings::kMaxNumFeeds) {
        CONSOLE_ERROR("CommsManager::SendBuf", "Invalid feed index %d.", iface);
        return false;
    }
    if (!feed_sock_is_connected_[iface]) {
        CONSOLE_ERROR("CommsManager::SendBuf", "Can't send to feed %d, socket not connected.", iface);
        return false;
    }

#ifdef ENABLE_TCP_SOCKET_RATE_LIMITING
    uint32_t timestamp_ms = get_time_since_boot_ms();
    if (timestamp_ms - last_send_buf_counter_reset_timestamp_ms > kSendBufRateLimitIntervalMs) {
        // Reset the counter every 10ms.
        last_send_buf_counter_reset_timestamp_ms = timestamp_ms;
        send_buf_counter_bytes = 0;
    }
    if (send_buf_counter_bytes + buf_len > kSendBufRateLimitBytesPerInterval) {
        vTaskDelay(
            pdTICKS_TO_MS(kRateLimitDelayDurationMs));  // Delay to yield to other tasks and allow buffers to clear.
    }
    send_buf_counter_bytes += buf_len;
#endif

    int err = safe_send(feed_sock_[iface], buf, buf_len);

    if (err < 0) {
        CONSOLE_ERROR("CommsManager::SendBuf",
                      "Error occurred during sending %d byte message to feed %d with URI %s "
                      "on port %d: "
                      "errno %d (%s).",
                      buf_len, iface, settings_manager.settings.feed_uris[iface],
                      settings_manager.settings.feed_ports[iface], errno, strerror(errno));
        CloseFeedSocket(iface);
        return false;
    } else {
        // CONSOLE_INFO("CommsManager::IPWANTask", "Message sent to feed %d.", i);
        feed_mps_counter_[iface]++;  // Log that a message was sent in statistics.
    }
    return true;
}

void CommsManager::UpdateFeedMetrics() {
    // Update feed statistics once per second and print them. Put this before the queue receive so that it runs even
    // if no packets are received.
    static const uint16_t kStatsMessageMaxLen = 500;
    uint32_t timestamp_ms = get_time_since_boot_ms();
    if (timestamp_ms - feed_mps_last_update_timestamp_ms_ > kMsPerSec) {
        for (uint16_t i = 0; i < SettingsManager::Settings::kMaxNumFeeds; i++) {
            feed_mps[i] = feed_mps_counter_[i];
            feed_mps_counter_[i] = 0;
        }
        feed_mps_last_update_timestamp_ms_ = timestamp_ms;

        char feeds_metrics_message[kStatsMessageMaxLen] = {'\0'};

        for (uint16_t i = 0; i < SettingsManager::Settings::kMaxNumFeeds; i++) {
            char single_feed_metrics_message[kStatsMessageMaxLen / SettingsManager::Settings::kMaxNumFeeds] = {'\0'};
            snprintf(single_feed_metrics_message, kStatsMessageMaxLen / SettingsManager::Settings::kMaxNumFeeds,
                     "%d:[%d] ", i, feed_mps[i]);
            strcat(feeds_metrics_message, single_feed_metrics_message);
        }
        CONSOLE_INFO("CommsManager::IPWANTask", "Feed msgs/s: %s", feeds_metrics_message);
    }
}
