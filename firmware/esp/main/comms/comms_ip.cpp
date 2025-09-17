#include "beast/beast_utils.hh"  // For beast reporting.
#include "comms.hh"
#include "esp_event.h"
#include "esp_mac.h"
#include "hal.hh"
#include "lwip/dns.h"
#include "lwip/err.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "mdns.h"
#include "task_priorities.hh"
#include "driver/temperature_sensor.h"  // For CPU temperature
#include "esp_app_desc.h"  // For firmware version
#include "server/adsbee_server.hh"  // For aircraft_dictionary metrics

static const uint32_t kWiFiTCPSocketReconnectIntervalMs = 5000;

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
        return false;
    }

    addr.s_addr = ((struct sockaddr_in*)res->ai_addr)->sin_addr.s_addr;
    inet_ntop(AF_INET, &addr, ip, 16);
    CONSOLE_INFO("ResolveURLToIP", "DNS lookup succeeded. IP=%s", ip);

    freeaddrinfo(res);
    return true;
}

bool CommsManager::IPInit() {
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, &ip_event_handler, NULL));
    ip_event_handler_was_initialized_ = true;

    xTaskCreatePinnedToCore(ip_wan_task, "ip_wan_task", 4096, &ip_wan_task_handle, kIPWANTaskPriority, NULL,
                            kIPWANTaskCore);

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
    Decoded1090Packet decoded_packet;

    int feed_sock[SettingsManager::Settings::kMaxNumFeeds] = {0};
    bool feed_sock_is_connected[SettingsManager::Settings::kMaxNumFeeds] = {false};
    uint32_t feed_sock_last_connect_timestamp_ms[SettingsManager::Settings::kMaxNumFeeds] = {0};

    CONSOLE_INFO("CommsManager::IPWANTask", "IP WAN Task started.");

    // Initialize MQTT clients for feeds configured with MQTT protocol
    for (uint16_t i = 0; i < SettingsManager::Settings::kMaxNumFeeds; i++) {
        if (settings_manager.settings.feed_protocols[i] == SettingsManager::ReportingProtocol::kMQTT &&
            settings_manager.settings.feed_is_active[i]) {

            MQTT::MQTTClient::Config mqtt_config;

            // Parse URI to extract protocol and hostname
            std::string full_uri = settings_manager.settings.feed_uris[i];
            std::string broker_uri = full_uri;
            bool use_tls = false;

            // Determine TLS based on protocol prefix and extract hostname
            if (full_uri.find("mqtt://") == 0) {
                broker_uri = full_uri.substr(7);  // Remove "mqtt://"
                use_tls = false;
            } else if (full_uri.find("mqtts://") == 0) {
                broker_uri = full_uri.substr(8);  // Remove "mqtts://"
                use_tls = true;
            }

            mqtt_config.broker_uri = broker_uri;
            mqtt_config.port = settings_manager.settings.feed_ports[i];
            mqtt_config.username = settings_manager.settings.mqtt_usernames[i];
            mqtt_config.password = settings_manager.settings.mqtt_passwords[i];
            mqtt_config.client_id = settings_manager.settings.mqtt_client_ids[i];
            mqtt_config.device_id = settings_manager.settings.mqtt_device_id;
            mqtt_config.use_tls = use_tls;
            mqtt_config.format = settings_manager.settings.mqtt_formats[i];
            mqtt_config.report_mode = settings_manager.settings.mqtt_report_modes[i];
            mqtt_config.telemetry_interval_sec = settings_manager.settings.mqtt_telemetry_interval_sec;
            mqtt_config.gps_interval_sec = settings_manager.settings.mqtt_gps_interval_sec;
            mqtt_config.status_rate_hz = settings_manager.settings.mqtt_status_rate_hz;

            mqtt_clients_[i] = std::make_unique<MQTT::MQTTClient>(mqtt_config, i);
            CONSOLE_INFO("CommsManager::IPWANTask", "Initialized MQTT client for feed %d", i);
        }
    }

    while (true) {
        // Don't try establishing socket connections until the ESP32 has been assigned an IP address.
        while (!wifi_sta_has_ip_ && !ethernet_has_ip_) {
            vTaskDelay(1);  // Delay for 1 tick.
        }

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

            char feeds_stats_message[kStatsMessageMaxLen] = {'\0'};

            for (uint16_t i = 0; i < SettingsManager::Settings::kMaxNumFeeds; i++) {
                char single_feed_stats_message[kStatsMessageMaxLen / SettingsManager::Settings::kMaxNumFeeds] = {'\0'};
                snprintf(single_feed_stats_message, kStatsMessageMaxLen / SettingsManager::Settings::kMaxNumFeeds,
                         "%d:[%d] ", i, feed_mps[i]);
                strcat(feeds_stats_message, single_feed_stats_message);
            }
            CONSOLE_INFO("CommsManager::IPWANTask", "Feed msgs/s: %s", feeds_stats_message);
        }

        // Handle MQTT connections and periodic telemetry
        for (uint16_t i = 0; i < SettingsManager::Settings::kMaxNumFeeds; i++) {
            if (!mqtt_clients_[i]) {
                continue;  // No MQTT client for this feed
            }

            if (settings_manager.settings.feed_protocols[i] != SettingsManager::ReportingProtocol::kMQTT ||
                !settings_manager.settings.feed_is_active[i]) {
                // Feed is not MQTT or not active, disconnect if connected
                if (mqtt_clients_[i]->IsConnected()) {
                    mqtt_clients_[i]->Disconnect();
                }
                continue;
            }

            // Try to connect if not connected
            if (!mqtt_clients_[i]->IsConnected()) {
                uint32_t now = get_time_since_boot_ms();
                if (now - mqtt_last_connect_attempt_[i] > 5000) {  // Try every 5 seconds
                    CONSOLE_INFO("CommsManager::IPWANTask",
                                 "Connecting MQTT feed %d to %s:%d",
                                 i, settings_manager.settings.feed_uris[i],
                                 settings_manager.settings.feed_ports[i]);
                    mqtt_clients_[i]->Connect();
                    mqtt_last_connect_attempt_[i] = now;
                }
            }

            // Publish telemetry if connected
            if (mqtt_clients_[i]->IsConnected()) {
                MQTT::Telemetry telemetry = {};
                telemetry.uptime_sec = get_time_since_boot_ms() / 1000;

                // msgs_rx should be total messages received per second across all feeds
                uint32_t total_rx_mps = 0;
                for (uint16_t j = 0; j < SettingsManager::Settings::kMaxNumFeeds; j++) {
                    total_rx_mps += feed_mps[j];
                }
                telemetry.msgs_rx = total_rx_mps;

                // msgs_tx is messages sent by this MQTT feed per second
                telemetry.msgs_tx = feed_mps[i];

                // Get CPU temperature (ESP32-S3 specific)
                float temp_celsius = 0.0f;
                temperature_sensor_handle_t temp_sensor = NULL;
                temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
                if (temperature_sensor_install(&temp_sensor_config, &temp_sensor) == ESP_OK) {
                    temperature_sensor_enable(temp_sensor);
                    temperature_sensor_get_celsius(temp_sensor, &temp_celsius);
                    temperature_sensor_disable(temp_sensor);
                    temperature_sensor_uninstall(temp_sensor);
                }
                telemetry.cpu_temp_c = (int32_t)temp_celsius;

                telemetry.mem_free_kb = esp_get_free_heap_size() / 1024;
                telemetry.noise_floor_dbm = -100;  // TODO: Get actual noise floor from decoder
                telemetry.rx_1090 = true;  // Always true for now
                telemetry.rx_978 =
                    (settings_manager.settings.subg_enabled == SettingsManager::EnableState::kEnableStateEnabled);
                telemetry.wifi = wifi_sta_has_ip_;
                telemetry.mqtt = true;

                // Get firmware version from app descriptor
                const esp_app_desc_t* app_desc = esp_app_get_description();
                telemetry.fw_version = app_desc ? app_desc->version : "unknown";

                telemetry.mps_total = 0;
                for (uint16_t j = 0; j < SettingsManager::Settings::kMaxNumFeeds; j++) {
                    telemetry.mps_total += feed_mps[j];
                }

                // Add per-feed message rates
                telemetry.mps_feeds.clear();
                for (uint16_t j = 0; j < SettingsManager::Settings::kMaxNumFeeds; j++) {
                    if (settings_manager.settings.feed_is_active[j]) {
                        telemetry.mps_feeds.push_back(feed_mps[j]);
                    }
                }

                // Add decoder statistics from aircraft dictionary
                // Note: ESP32 uses combined metrics (ESP32 + RP2040)
                AircraftDictionary::Metrics combined_metrics = adsbee_server.aircraft_dictionary.metrics;

                // Add RP2040 metrics for demodulations and raw frames
                combined_metrics.demods_1090 = adsbee_server.rp2040_aircraft_dictionary_metrics.demods_1090;
                combined_metrics.raw_squitter_frames = adsbee_server.rp2040_aircraft_dictionary_metrics.raw_squitter_frames;
                combined_metrics.raw_extended_squitter_frames =
                    adsbee_server.rp2040_aircraft_dictionary_metrics.raw_extended_squitter_frames;

                telemetry.demods_1090 = combined_metrics.demods_1090;
                telemetry.raw_squitter_frames = combined_metrics.raw_squitter_frames;
                telemetry.valid_squitter_frames = combined_metrics.valid_squitter_frames;
                telemetry.raw_extended_squitter = combined_metrics.raw_extended_squitter_frames;
                telemetry.valid_extended_squitter = combined_metrics.valid_extended_squitter_frames;

                mqtt_clients_[i]->PublishTelemetry(telemetry);
            }
        }

        // Gather packet(s) to send.
        if (xQueueReceive(ip_wan_decoded_transponder_packet_queue_, &decoded_packet, kWiFiSTATaskUpdateIntervalTicks) !=
            pdTRUE) {
            // No packets available to send, wait and try again.
            continue;
        }

        // Debug: Log that we received a packet for processing
        CONSOLE_INFO("CommsManager::IPWANTask", "Processing packet for ICAO 0x%06x, valid=%d",
                     decoded_packet.GetICAOAddress(), decoded_packet.IsValid());

        // NOTE: Construct packets that are shared between feeds here!

        for (uint16_t i = 0; i < SettingsManager::Settings::kMaxNumFeeds; i++) {
            // Iterate through feeds, open/close and send message as required.
            if (!settings_manager.settings.feed_is_active[i]) {
                // Socket should not be fed.
                // Only close raw sockets, not MQTT connections (handled by MQTT client)
                if (feed_sock_is_connected[i] &&
                    settings_manager.settings.feed_protocols[i] != SettingsManager::ReportingProtocol::kMQTT) {
                    // Need to close the socket connection.
                    close(feed_sock[i]);
                    feed_sock_is_connected[i] = false;
                    CONSOLE_INFO("CommsManager::IPWANTask", "Closed socket for feed %d.", i);
                }
                continue;  // Don't need to do anything else if socket should be closed and is closed.
            }

            // Skip MQTT feeds - they are handled by MQTT client
            if (settings_manager.settings.feed_protocols[i] == SettingsManager::ReportingProtocol::kMQTT) {
                continue;  // MQTT feeds don't use raw sockets
            }

            // Socket should be open.
            if (!feed_sock_is_connected[i]) {
                // Need to open the socket connection.

                // Meter reconnect attempt interval.
                uint32_t timestamp_ms = get_time_since_boot_ms();
                if (timestamp_ms - feed_sock_last_connect_timestamp_ms[i] <= kWiFiTCPSocketReconnectIntervalMs) {
                    continue;
                }
                feed_sock_last_connect_timestamp_ms[i] = timestamp_ms;

                // Create socket.
                // IPv4, TCP
                feed_sock[i] = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
                if (feed_sock[i] <= 0) {
                    CONSOLE_ERROR("CommsManager::IPWANTask", "Unable to create socket for feed %d: errno %d", i, errno);
                    continue;
                }
                CONSOLE_INFO("CommsManager::IPWANTask", "Socket for feed %d created, connecting to %s:%d", i,
                             settings_manager.settings.feed_uris[i], settings_manager.settings.feed_ports[i]);

                // Enable TCP keepalive
                setsockopt(feed_sock[i], SOL_SOCKET, SO_KEEPALIVE, &kTCPKeepAliveEnable, sizeof(kTCPKeepAliveEnable));
                setsockopt(feed_sock[i], IPPROTO_TCP, TCP_KEEPIDLE, &kTCPKeepAliveIdleSecondsBeforeStartingProbe,
                           sizeof(kTCPKeepAliveIdleSecondsBeforeStartingProbe));
                setsockopt(feed_sock[i], IPPROTO_TCP, TCP_KEEPINTVL, &kTCPKeepAliveIntervalSecondsBetweenProbes,
                           sizeof(kTCPKeepAliveIntervalSecondsBetweenProbes));
                setsockopt(feed_sock[i], IPPROTO_TCP, TCP_KEEPCNT, &kTCPKeepAliveMaxFailedProbesBeforeDisconnect,
                           sizeof(kTCPKeepAliveMaxFailedProbesBeforeDisconnect));
                // Allow reuse of local addresses.
                setsockopt(feed_sock[i], SOL_SOCKET, SO_REUSEADDR, &kTCPReuseAddrEnable, sizeof(kTCPReuseAddrEnable));

                struct sockaddr_in dest_addr;
                // If the URI contains letters, resolve it to an IP address
                if (IsNotIPAddress(settings_manager.settings.feed_uris[i])) {
                    // Is not an IP address, try DNS resolution.
                    char resolved_ip[16];
                    if (!ResolveURIToIP(settings_manager.settings.feed_uris[i], resolved_ip)) {
                        CONSOLE_ERROR("CommsManager::IPWANTask", "Failed to resolve URL %s for feed %d",
                                      settings_manager.settings.feed_uris[i], i);
                        close(feed_sock[i]);
                        feed_sock_is_connected[i] = false;
                        continue;
                    }
                    inet_pton(AF_INET, resolved_ip, &dest_addr.sin_addr);
                } else {
                    // Is an IP address, use it directly.
                    inet_pton(AF_INET, settings_manager.settings.feed_uris[i], &dest_addr.sin_addr);
                }

                dest_addr.sin_family = AF_INET;
                dest_addr.sin_port = htons(settings_manager.settings.feed_ports[i]);

                int err = connect(feed_sock[i], (struct sockaddr*)&dest_addr, sizeof(dest_addr));
                if (err != 0) {
                    CONSOLE_ERROR(
                        "CommsManager::IPWANTask", "Socket unable to connect to URI %s:%d for feed %d: errno %d",
                        settings_manager.settings.feed_uris[i], settings_manager.settings.feed_ports[i], i, errno);
                    close(feed_sock[i]);
                    feed_sock_is_connected[i] = false;
                    continue;
                }
                CONSOLE_INFO("CommsManager::IPWANTask", "Successfully connected to %s",
                             settings_manager.settings.feed_uris[i]);
                feed_sock_is_connected[i] = true;

                // Perform beginning-of-connection actions here.
                switch (settings_manager.settings.feed_protocols[i]) {
                    case SettingsManager::ReportingProtocol::kBeast:
                        [[fallthrough]];
                    case SettingsManager::ReportingProtocol::kBeastRaw: {
                        uint8_t beast_message_buf[2 * SettingsManager::Settings::kFeedReceiverIDNumBytes +
                                                  kBeastFrameMaxLenBytes];
                        uint16_t beast_message_len_bytes =
                            BuildFeedStartFrame(beast_message_buf, settings_manager.settings.feed_receiver_ids[i]);
                        int err = send(feed_sock[i], beast_message_buf, beast_message_len_bytes, 0);
                        if (err < 0) {
                            CONSOLE_ERROR("CommsManager::IPWANTask",
                                          "Error occurred while sending %d Byte Beast start of feed message to feed %d "
                                          "with URI %s "
                                          "on port %d: "
                                          "errno %d.",
                                          beast_message_len_bytes, i, settings_manager.settings.feed_uris[i],
                                          settings_manager.settings.feed_ports[i], errno);
                            // Mark socket as disconnected and try reconnecting in next reporting interval.
                            close(feed_sock[i]);
                            feed_sock_is_connected[i] = false;
                            continue;
                        }
                        break;
                    }
                    default:
                        // No start of connections actions required for other protocols.
                        break;
                }
            }

            // Send packet!
            // NOTE: Construct packets that are specific to a feed in case statements here!
            switch (settings_manager.settings.feed_protocols[i]) {
                case SettingsManager::ReportingProtocol::kBeast:
                    if (!decoded_packet.IsValid()) {
                        // Packet is invalid, don't send.
                        break;
                    }
                    [[fallthrough]];  // Intentional cascade into BEAST_RAW, since reporting code is shared.
                case SettingsManager::ReportingProtocol::kBeastRaw: {
                    // Send Beast packet.
                    // Double the length as a hack to make room for the escaped UUID.
                    uint8_t beast_message_buf[2 * SettingsManager::Settings::kFeedReceiverIDNumBytes +
                                              kBeastFrameMaxLenBytes];
                    uint16_t beast_message_len_bytes = Build1090BeastFrame(decoded_packet, beast_message_buf);

                    int err = send(feed_sock[i], beast_message_buf, beast_message_len_bytes, 0);
                    if (err < 0) {
                        CONSOLE_ERROR("CommsManager::IPWANTask",
                                      "Error occurred during sending %d Byte beast message to feed %d with URI %s "
                                      "on port %d: "
                                      "errno %d.",
                                      beast_message_len_bytes, i, settings_manager.settings.feed_uris[i],
                                      settings_manager.settings.feed_ports[i], errno);
                        // Mark socket as disconnected and try reconnecting in next reporting interval.
                        close(feed_sock[i]);
                        feed_sock_is_connected[i] = false;
                    } else {
                        // CONSOLE_INFO("CommsManager::IPWANTask", "Message sent to feed %d.", i);
                        feed_mps_counter_[i]++;  // Log that a message was sent in statistics.
                    }
                    break;
                }
                case SettingsManager::ReportingProtocol::kMQTT: {
                    // Debug: Log that we're in MQTT case
                    CONSOLE_INFO("CommsManager::IPWANTask", "Feed %d is MQTT, checking connection", i);

                    // Publish via MQTT
                    if (!mqtt_clients_[i] || !mqtt_clients_[i]->IsConnected()) {
                        // No client or not connected
                        CONSOLE_INFO("CommsManager::IPWANTask", "Feed %d MQTT client not connected", i);
                        break;
                    }

                    // Determine band (1090 MHz or UAT)
                    // TODO: Properly detect band from source
                    uint8_t band = 1;  // Default to 1090 MHz

                    // Publish based on report mode
                    if (settings_manager.settings.mqtt_report_modes[i] == SettingsManager::MQTTReportMode::kMQTTReportModeStatus ||
                        settings_manager.settings.mqtt_report_modes[i] == SettingsManager::MQTTReportMode::kMQTTReportModeBoth) {

                        // For MQTT, we publish aircraft status even if the packet itself is invalid,
                        // as long as we have aircraft data in the dictionary
                        uint32_t icao_address = decoded_packet.GetICAOAddress();

                        // Only proceed if we have a valid ICAO address
                        if (icao_address != 0) {
                            // Get aircraft data from dictionary for complete information
                            Aircraft1090* aircraft = adsbee_server.aircraft_dictionary.GetAircraftPtr(icao_address);

                            // Convert to TransponderPacket for publishing
                            TransponderPacket packet;
                            packet.address = icao_address;
                            packet.timestamp_ms = decoded_packet.GetTimestampMs();

                            if (aircraft != nullptr) {
                                // We have aircraft data from the dictionary
                                packet.latitude = aircraft->latitude_deg;
                                packet.longitude = aircraft->longitude_deg;
                                packet.altitude = aircraft->baro_altitude_ft;
                                packet.heading = aircraft->direction_deg;
                                packet.velocity = aircraft->velocity_kts;
                                packet.vertical_rate = aircraft->vertical_rate_fpm;
                                packet.squawk = aircraft->squawk;
                                packet.airborne = (aircraft->baro_altitude_ft > 100) ? 1 : 0;  // Simple ground detection
                                packet.category = aircraft->category_raw;

                                // Copy callsign
                                strncpy(packet.callsign, aircraft->callsign, 8);
                                packet.callsign[8] = '\0';

                                // Set validity flags based on available data
                                packet.flags = 0;
                                if (aircraft->latitude_deg != 0.0 || aircraft->longitude_deg != 0.0) {
                                    packet.flags |= TransponderPacket::FLAG_POSITION_VALID;
                                }
                                if (aircraft->baro_altitude_ft != 0) {
                                    packet.flags |= TransponderPacket::FLAG_ALTITUDE_VALID;
                                }
                                if (aircraft->velocity_kts > 0) {
                                    packet.flags |= TransponderPacket::FLAG_VELOCITY_VALID;
                                }
                                if (aircraft->direction_deg != 0.0) {
                                    packet.flags |= TransponderPacket::FLAG_HEADING_VALID;
                                }
                                if (aircraft->vertical_rate_fpm != 0) {
                                    packet.flags |= TransponderPacket::FLAG_VERTICAL_RATE_VALID;
                                }
                                if (strlen(aircraft->callsign) > 1 && strcmp(aircraft->callsign, "?") != 0) {
                                    packet.flags |= TransponderPacket::FLAG_CALLSIGN_VALID;
                                }
                                if (aircraft->squawk != 0) {
                                    packet.flags |= TransponderPacket::FLAG_SQUAWK_VALID;
                                }
                                if (aircraft->category_raw != 0) {
                                    packet.flags |= TransponderPacket::FLAG_CATEGORY_VALID;
                                }
                            } else {
                                // No aircraft data in dictionary, use minimal data
                                packet.altitude = 0;
                                packet.latitude = 0.0;
                                packet.longitude = 0.0;
                                packet.heading = 0.0;
                                packet.velocity = 0.0;
                                packet.vertical_rate = 0;
                                packet.squawk = 0;
                                packet.airborne = 1;
                                packet.category = 0;
                                memset(packet.callsign, 0, 9);
                                packet.flags = 0;  // Only address is valid
                            }

                            if (mqtt_clients_[i]->PublishAircraftStatus(packet, band)) {
                                feed_mps_counter_[i]++;  // Update statistics
                            }
                        }
                    }

                    // TODO: Add raw packet publishing if report_mode includes RAW
                    break;
                }
                // TODO: add other protocols here
                default:
                    // No reporting protocol or unsupported protocol: do nothing.
                    break;
            }
        }
    }

    // Close all sockets while exiting (skip MQTT feeds).
    for (uint16_t i = 0; i < SettingsManager::Settings::kMaxNumFeeds; i++) {
        if (feed_sock_is_connected[i] &&
            settings_manager.settings.feed_protocols[i] != SettingsManager::ReportingProtocol::kMQTT) {
            // Need to close the socket connection.
            close(feed_sock[i]);
            feed_sock_is_connected[i] = false;  // Not necessary but leaving this here in case of refactor.
        }
    }

    CONSOLE_INFO("CommsManager::IPWANTask", "IP WAN Task exiting.");
}