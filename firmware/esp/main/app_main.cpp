/* SPI Slave example, receiver (uses SPI Slave driver to communicate with sender)

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "adsbee_server.hh"
#include "bsp.hh"
#include "comms.hh"
#include "driver/gpio.h"
#include "driver/spi_slave.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hardware_unit_tests.hh"
#include "pico.hh"
#include "settings.hh"
#include "spi_coprocessor.hh"

#define HARDWARE_UNIT_TESTS

BSP bsp = BSP();
ObjectDictionary object_dictionary;
Pico pico_ll = Pico({});
SPICoprocessor pico = SPICoprocessor({.interface = pico_ll});
ADSBeeServer adsbee_server = ADSBeeServer();
SettingsManager settings_manager = SettingsManager();
CommsManager comms_manager = CommsManager({});

// Main application
extern "C" void app_main(void) {
    ESP_LOGI("app_main", "Beginning ADSBee Server Application.");
    ESP_LOGI("app_main", "=== FIRMWARE BUILD: 2025-01-17 with TEMP FIX ===");
    ESP_LOGI("app_main", "Firmware Version: %d.%d.%d",
             ObjectDictionary::kFirmwareVersionMajor,
             ObjectDictionary::kFirmwareVersionMinor,
             ObjectDictionary::kFirmwareVersionPatch);
    ESP_LOGI("app_main", "Default task priority: %d", uxTaskPriorityGet(NULL));

    // Print build version on startup
    ESP_LOGW("app_main", "ESP32 BUILD VERSION: %d.%d.%d-RC%d (DEBUG from 9b7ac304)",
             ObjectDictionary::kFirmwareVersionMajor,
             ObjectDictionary::kFirmwareVersionMinor,
             ObjectDictionary::kFirmwareVersionPatch,
             ObjectDictionary::kFirmwareVersionReleaseCandidate);

    adsbee_server.Init();

#ifdef HARDWARE_UNIT_TESTS
    RunHardwareUnitTests();
#endif

    uint32_t last_version_print_ms = 0;
    const uint32_t version_print_interval_ms = 60000; // 60 seconds

    while (1) {
        adsbee_server.Update();

        // Print version every 60 seconds
        uint32_t now_ms = esp_timer_get_time() / 1000;
        if (now_ms - last_version_print_ms >= version_print_interval_ms) {
            ESP_LOGW("app_main", "ESP32 VERSION CHECK: %d.%d.%d-RC%d (DEBUG from 9b7ac304) - Uptime: %lu sec",
                     ObjectDictionary::kFirmwareVersionMajor,
                     ObjectDictionary::kFirmwareVersionMinor,
                     ObjectDictionary::kFirmwareVersionPatch,
                     ObjectDictionary::kFirmwareVersionReleaseCandidate,
                     now_ms / 1000);
            last_version_print_ms = now_ms;
        }

        // Yield to the idle task to avoid a watchdog trigger. Note: Delay must be >= 10ms since 100Hz tick is typical.
        vTaskDelay(1);  // Delay 1 tick (10ms).
    }
}
