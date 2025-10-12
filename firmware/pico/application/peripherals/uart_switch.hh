#pragma once

#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <cstdint>

/**
 * UART Switching Manager
 * 
 * Manages UART0 switching between ESP32 programming interface and GNSS module.
 * Since UART0 pins (GPIO 0/1) are physically shared between both devices, 
 * only one can be active at a time.
 * 
 * Pin mapping:
 * - UART0 TX (GPIO 0) -> ESP32 RX (programming) AND GNSS RX (shared line)
 * - UART0 RX (GPIO 1) -> ESP32 TX (programming) AND GNSS TX (shared line)
 * 
 * CRITICAL: The RP2040 must check for ESP32 firmware updates BEFORE 
 * initializing UART0 for GNSS to avoid blocking ESP32 bootloader access.
 */
class UARTSwitch {
public:
    enum UARTMode {
        kModeNone,      // UART0 not initialized
        kModeESP32,     // UART0 configured for ESP32 programming
        kModeGNSS       // UART0 configured for GNSS communication
    };

    /**
     * Initialize UART0 for ESP32 programming mode
     * This should be called early in boot to allow ESP32 firmware checks
     *
     * @param baudrate Baudrate for ESP32 programming (typically 115200)
     * @return true if initialization successful
     */
    static bool InitForESP32(uint32_t baudrate = 115200) {
        if (current_mode_ == kModeESP32) {
            // Already in ESP32 mode, just update baudrate if needed
            uart_set_baudrate(uart0, baudrate);
            return true;
        }

        // Deinitialize if currently in another mode
        if (current_mode_ != kModeNone) {
            Deinit();
        }

        // Configure pins for UART0
        gpio_set_function(0, GPIO_FUNC_UART);  // TX
        gpio_set_function(1, GPIO_FUNC_UART);  // RX

        // Initialize UART0 for ESP32 communication
        uart_init(uart0, baudrate);
        uart_set_translate_crlf(uart0, false);
        uart_set_fifo_enabled(uart0, true);

        current_mode_ = kModeESP32;
        return true;
    }

    /**
     * Initialize UART0 for GNSS mode
     * This should be called after ESP32 firmware checks are complete
     *
     * @param baudrate Baudrate for GNSS (typically 115200 or configured value)
     * @return true if initialization successful
     */
    static bool InitForGNSS(uint32_t baudrate = 115200) {
        if (current_mode_ == kModeGNSS) {
            // Already in GNSS mode, just update baudrate if needed
            uart_set_baudrate(uart0, baudrate);
            return true;
        }

        // Deinitialize if currently in another mode
        if (current_mode_ != kModeNone) {
            Deinit();
        }

        // Configure pins for UART0
        gpio_set_function(0, GPIO_FUNC_UART);  // TX
        gpio_set_function(1, GPIO_FUNC_UART);  // RX

        // Initialize UART0 for GNSS communication
        uart_init(uart0, baudrate);
        uart_set_translate_crlf(uart0, false);
        uart_set_fifo_enabled(uart0, true);

        current_mode_ = kModeGNSS;
        return true;
    }

    /**
     * Deinitialize UART0, releasing the pins
     * This allows switching between modes
     */
    static void Deinit() {
        if (current_mode_ == kModeNone) {
            return; // Already deinitialized
        }

        // Wait for any pending transmissions to complete
        uart_tx_wait_blocking(uart0);
        
        // Deinitialize UART0
        uart_deinit(uart0);
        
        // Reset pins to GPIO mode (high impedance)
        gpio_init(0);
        gpio_init(1);
        gpio_set_dir(0, GPIO_IN);
        gpio_set_dir(1, GPIO_IN);
        
        current_mode_ = kModeNone;
    }

    /**
     * Get current UART0 mode
     * @return Current operating mode
     */
    static UARTMode GetCurrentMode() {
        return current_mode_;
    }

    /**
     * Check if UART0 is available for a specific mode
     * @param mode Mode to check
     * @return true if UART0 is either in the requested mode or not initialized
     */
    static bool IsAvailableFor(UARTMode mode) {
        return (current_mode_ == kModeNone || current_mode_ == mode);
    }

private:
    static UARTMode current_mode_;
};