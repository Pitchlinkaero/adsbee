#include <mutex>

#include "adsbee.hh"
#include "comms.hh"
#include "core1.hh"  // Functions for runningon core1.
#include "cpu_utils.hh"
#include "eeprom.hh"
#include "esp32.hh"
#include "esp32_flasher.hh"
#include "firmware_update.hh"  // For figuring out which flash partition we're in.
#include "gnss/gnss_manager.hh"
#include "hal.hh"
#include "hardware_unit_tests.hh"  // For testing only!
#include "mode_s_packet.hh"
#include "mode_s_packet_decoder.hh"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "spi_coprocessor.hh"
#include "unit_conversions.hh"
#include "uart_switch.hh"

// #define DEBUG_DISABLE_ESP32_FLASH  // Uncomment this to stop the RP2040 from flashing the ESP32.

// For testing only
#include "hardware/gpio.h"

const uint16_t kStatusLEDBootupBlinkPeriodMs = 200;
const uint32_t kESP32BootupTimeoutMs = 10000;
const uint32_t kESP32BootupCommsRetryMs = 500;

const uint32_t kRP2040IdleTicksPerUpdateInterval =
    125e3;  // Arbitrary, assume 1000 instructions per idle loop at 125MHz.
const uint32_t kRP2040CPUMonitorUpdateIntervalMs = 1000;

// Override default config params here.
EEPROM eeprom = EEPROM({});
// BSP gets configured differently if there is or isn't an EEPROM attached. Attempt to initialize the EEPROM to figure
// out which board configuration we should load (settings in flash, or settings in EEPROM).
BSP bsp = BSP(eeprom.Init());

CPUMonitor core_0_monitor = CPUMonitor({.idle_ticks_per_update_interval = kRP2040IdleTicksPerUpdateInterval,
                                        .update_interval_ms = kRP2040CPUMonitorUpdateIntervalMs});
CPUMonitor core_1_monitor = CPUMonitor({.idle_ticks_per_update_interval = kRP2040IdleTicksPerUpdateInterval,
                                        .update_interval_ms = kRP2040CPUMonitorUpdateIntervalMs});

ADSBee adsbee = ADSBee({});
CommsManager comms_manager = CommsManager({});
ESP32SerialFlasher esp32_flasher = ESP32SerialFlasher({});

SettingsManager settings_manager;
ObjectDictionary object_dictionary;
GNSSManager gnss_manager;

// Define low-level coprocessor devices with overrides for things like GPIO and init functions.
ESP32 esp32_ll = ESP32({});

// Provide high-level coprocessor objects for interacting with coprocessor devices via low level class definitions.
SPICoprocessor esp32 = SPICoprocessor(
    {.interface = esp32_ll, .tag_str = "ESP32"});  // Use the low-level ESP32 interface to communicate with the ESP32.
ModeSPacketDecoder decoder = ModeSPacketDecoder({.enable_1090_error_correction = true});

int main() {
    bi_decl(bi_program_description("ADSBee 1090 ADSB Receiver"));

    // Initialize the temperature sensor.
    CPUMonitor::Init();

    // Initialize coprocessor SPI bus.
    // ESP32 SPI pins.
    gpio_set_function(bsp.copro_spi_clk_pin, GPIO_FUNC_SPI);
    gpio_set_function(bsp.copro_spi_mosi_pin, GPIO_FUNC_SPI);
    gpio_set_function(bsp.copro_spi_miso_pin, GPIO_FUNC_SPI);
    gpio_set_drive_strength(bsp.copro_spi_clk_pin, bsp.copro_spi_drive_strength);
    gpio_set_drive_strength(bsp.copro_spi_mosi_pin, bsp.copro_spi_drive_strength);
    gpio_set_pulls(bsp.copro_spi_clk_pin, bsp.copro_spi_pullup, bsp.copro_spi_pulldown);   // Clock pin pulls.
    gpio_set_pulls(bsp.copro_spi_mosi_pin, bsp.copro_spi_pullup, bsp.copro_spi_pulldown);  // MOSI pin pulls.
    gpio_set_pulls(bsp.copro_spi_miso_pin, bsp.copro_spi_pullup, bsp.copro_spi_pulldown);  // MISO pin pulls.
    // Initialize SPI Peripheral.
    spi_init(bsp.copro_spi_handle, bsp.copro_spi_clk_freq_hz);
    // The CC1312 straight up does not work with CPOL = 0 and CPHA = 0 (only sends one Byte per transaction then
    // explodes). The ESP32 doesn't seem to care either way (in fact it interprets CPOL = 1 CPHA = 1 as CPOL = 0 CPHA =
    // 0 just fine), so we stick with CPOL = 1 CPHA = 1.
    // I briefly tried switching the SPI format back and forth continutously in SPIBeginTransaction(), but this was
    // causing crashes.
    spi_set_format(bsp.copro_spi_handle,
                   8,           // Bits per transfer.
                   SPI_CPOL_1,  // Polarity (CPOL).
                   SPI_CPHA_1,  // Phase (CPHA).
                   SPI_MSB_FIRST);

    adsbee.Init();
    comms_manager.Init();  // Note: This does NOT initialize GNSS UART yet
    comms_manager.console_printf("ADSBee 1090\r\nSoftware Version %d.%d.%d\r\n",
                                 object_dictionary.kFirmwareVersionMajor, object_dictionary.kFirmwareVersionMinor,
                                 object_dictionary.kFirmwareVersionPatch);

    settings_manager.Load();

    uint16_t num_status_led_blinks = FirmwareUpdateManager::AmWithinFlashPartition(0) ? 1 : 2;
    // Blink the LED a few times to indicate a successful startup.
    for (uint16_t i = 0; i < num_status_led_blinks; i++) {
        adsbee.SetStatusLED(true);
        sleep_ms(kStatusLEDBootupBlinkPeriodMs / 2);
        adsbee.SetStatusLED(false);
        sleep_ms(kStatusLEDBootupBlinkPeriodMs / 2);
    }

    // If WiFi is enabled, check ESP32 firmware version via SPI and update if needed
    if (esp32.IsEnabled()) {
        adsbee.DisableWatchdog();  // Disable watchdog while setting up ESP32, in case kESP32BootupTimeoutMs >=
                                   // watchdog timeout, and to avoid watchdog reboot during ESP32 programming.

        // Initialize ESP32 SPI interface so we can communicate with it
        comms_manager.console_printf("Initializing ESP32 SPI interface...\r\n");
        if (!esp32.Init()) {
            CONSOLE_ERROR("main", "Failed to initialize ESP32 SPI interface");
        } else {
            comms_manager.console_printf("ESP32 SPI interface initialized successfully\r\n");
        }

        // Give ESP32 time to boot up
        comms_manager.console_printf("Waiting for ESP32 to boot (bootup delay = %d ms)...\r\n", ESP32::kBootupDelayMs);
        sleep_ms(ESP32::kBootupDelayMs);

        // Try reading firmware version from ESP32 via SPI
        uint32_t esp32_firmware_version = 0x0;
        bool flash_esp32 = false;
        bool version_read_successful = false;
        uint32_t esp32_comms_start_timestamp_ms = get_time_since_boot_ms();
        uint32_t esp32_comms_last_try_timestamp_ms = 0;
        uint32_t read_attempts = 0;

        comms_manager.console_printf("Checking ESP32 firmware version via SPI (timeout = %d ms)...\r\n",
                                    kESP32BootupTimeoutMs);

        while (get_time_since_boot_ms() - esp32_comms_start_timestamp_ms < kESP32BootupTimeoutMs) {
            // Wait until the next retry interval to avoid spamming the ESP32 continuously.
            if (get_time_since_boot_ms() - esp32_comms_last_try_timestamp_ms < kESP32BootupCommsRetryMs) {
                continue;
            }
            esp32_comms_last_try_timestamp_ms = get_time_since_boot_ms();
            read_attempts++;

            // Try reading the firmware version from the ESP32 via SPI
            comms_manager.console_printf("Attempt %d: Reading ESP32 firmware version...\r\n", read_attempts);
            if (!esp32.Read(ObjectDictionary::Address::kAddrFirmwareVersion, esp32_firmware_version)) {
                // Couldn't read firmware version from ESP32. Try again later.
                CONSOLE_ERROR("main", "Attempt %d: Unable to read firmware version from ESP32 via SPI.", read_attempts);
            } else {
                version_read_successful = true;
                comms_manager.console_printf("Successfully read ESP32 firmware version: 0x%08X\r\n", esp32_firmware_version);

                if (esp32_firmware_version != object_dictionary.kFirmwareVersion) {
                    // ESP32 firmware version doesn't match ours. Need to flash the ESP32.
                    CONSOLE_ERROR("main",
                                  "Firmware version MISMATCH! Pico: v%d.%d.%d (0x%08X), ESP32: v%d.%d.%d (0x%08X)",
                                  object_dictionary.kFirmwareVersionMajor, object_dictionary.kFirmwareVersionMinor,
                                  object_dictionary.kFirmwareVersionPatch, object_dictionary.kFirmwareVersion,
                                  esp32_firmware_version >> 16,
                                  (esp32_firmware_version >> 8) & 0xFF,
                                  esp32_firmware_version & 0xFF,
                                  esp32_firmware_version);
                    flash_esp32 = true;
                    break;
                } else {
                    // Firmware version matches! No need to flash.
                    comms_manager.console_printf("ESP32 firmware version MATCHES (v%d.%d.%d), no update needed\r\n",
                                                object_dictionary.kFirmwareVersionMajor,
                                                object_dictionary.kFirmwareVersionMinor,
                                                object_dictionary.kFirmwareVersionPatch);
                    flash_esp32 = false;
                    break;
                }
            }
        }

        // If we couldn't read firmware version after timeout, force flash
        if (!version_read_successful) {
            CONSOLE_ERROR("main", "Failed to read ESP32 firmware version after %d attempts over %d ms, FORCING FLASH",
                         read_attempts, kESP32BootupTimeoutMs);
            flash_esp32 = true;
        }

        comms_manager.console_printf("Firmware check complete: flash_esp32=%d\r\n", flash_esp32);

        adsbee.EnableWatchdog();  // Restore watchdog.

#ifndef DEBUG_DISABLE_ESP32_FLASH
        // Only flash if firmware version check indicated it's needed
        if (flash_esp32) {
            comms_manager.console_printf("ESP32 firmware update required, preparing for flash...\r\n");
            adsbee.DisableWatchdog();  // Disable watchdog while flashing.

            // Deinitialize ESP32 SPI before flashing (powers down ESP32)
            comms_manager.console_printf("Powering down ESP32...\r\n");
            if (!esp32.DeInit()) {
                CONSOLE_ERROR("main", "Error while de-initializing ESP32 SPI before flashing.");
            }

            // Give ESP32 time to fully power down before starting flash procedure
            sleep_ms(500);  // Increased from 200ms to give more time for power to fully drain

            // Ensure GPIO 0 and 1 are in a clean state before flashing
            // Reset them to default GPIO input mode to clear any previous UART/SPI configuration
            gpio_init(0);
            gpio_set_dir(0, GPIO_IN);
            gpio_init(1);
            gpio_set_dir(1, GPIO_IN);
            sleep_ms(50);  // Brief delay to ensure pins stabilize

            // FlashESP32() handles its own Init/DeInit cycle for UART0
            comms_manager.console_printf("Starting ESP32 firmware flash...\r\n");
            if (!esp32_flasher.FlashESP32()) {
                CONSOLE_ERROR("main", "Error while flashing ESP32. Disabling.");
                esp32.SetEnable(false);  // Disable ESP32 if flashing failed.
            } else {
                comms_manager.console_printf("ESP32 firmware updated successfully\r\n");
                // Re-initialize ESP32 SPI after flashing
                if (!esp32.Init()) {
                    CONSOLE_ERROR("main", "Error while re-initializing ESP32 SPI after flashing.");
                }
            }

            adsbee.EnableWatchdog();  // Restore watchdog after flashing.
        }
#endif
    }

    // Initialize UART0 for GNSS now that all ESP32 operations are complete
    comms_manager.console_printf("Initializing UART0 for GNSS...\r\n");
    if (!comms_manager.InitGNSSUART()) {
        comms_manager.console_printf("Warning: Failed to initialize GNSS UART\r\n");
    }

    // Initialize GPS manager with current settings
    if (gnss_manager.Initialize(settings_manager.settings.gps_settings)) {
        comms_manager.console_printf("GPS initialized: %s source\r\n", 
                                    settings_manager.settings.gps_settings.GetSourceString());
    }

    multicore_reset_core1();
    multicore_launch_core1(main_core1);

    uint32_t esp32_last_heartbeat_timestamp_ms = 0;

    while (true) {
        // Loop forever.
        core_0_monitor.Tick();
        core_0_monitor.Update();
        core_1_monitor.Update();

        decoder.UpdateLogLoop();
        comms_manager.Update();
        adsbee.Update();

        esp32.Update();
        
        // Update GPS position
        gnss_manager.UpdatePosition();
        
        // Check for GPS network messages from ESP32
        // Only read if GPS source is configured for network or auto mode
        if (settings_manager.settings.gps_settings.gps_source == GPSSettings::kGPSSourceNetwork ||
            settings_manager.settings.gps_settings.gps_source == GPSSettings::kGPSSourceAuto) {
            // This struct should match GPSNetworkServer::GPSNetworkMessage on ESP32
            struct __attribute__((__packed__)) GPSNetworkMessage {
                uint8_t type;
                uint8_t source_id;  // Client ID
                uint16_t length;
                uint8_t data[256];
                uint32_t timestamp_ms;
            } gps_msg;

            if (esp32.Read(ObjectDictionary::Address::kAddrGPSNetworkMessage, gps_msg)) {
                // Forward GPS message to GNSS manager
                gnss_manager.ProcessNetworkGPSMessage(gps_msg.type, gps_msg.data, gps_msg.length);
            }
        }

        // Poke the watchdog to keep things alive if the ESP32 is responding or if it's disabled.
        uint32_t old_esp32_last_heartbeat_timestamp_ms = esp32_last_heartbeat_timestamp_ms;
        esp32_last_heartbeat_timestamp_ms = esp32.GetLastHeartbeatTimestampMs();
        if (esp32_last_heartbeat_timestamp_ms != old_esp32_last_heartbeat_timestamp_ms || !esp32.IsEnabled()) {
            // Don't need to talk to the ESP32, or it acknowledged a heartbeat just now: poke the watchdog since nothing
            // seems amiss.
            adsbee.PokeWatchdog();
        }
    }
}