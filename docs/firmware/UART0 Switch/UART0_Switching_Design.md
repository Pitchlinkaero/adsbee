# UART0 Switching Design Documentation

## Overview

The ADSBee RP2040 uses UART0 for communication with two different components:
- **ESP32-S3 UART Programming Interface** - For firmware updates and bootloader access
- **GNSS Module UART Interface** - For GPS/GNSS data communication

While both devices use UART0, they connect to **different GPIO pins**. The RP2040 can route UART0 to different GPIO pins as needed. This document describes the UART switching mechanism implemented to manage UART0 configuration.

## Hardware Configuration

### Pin Mapping
- **ESP32 UART (Programming)**:
  - UART0 TX on GPIO 16 → ESP32 RX
  - UART0 RX on GPIO 17 ← ESP32 TX
- **GNSS Module UART**:
  - UART0 TX on GPIO 0 → GNSS RX
  - UART0 RX on GPIO 1 ← GNSS TX

### Key Architecture Point
**The devices do NOT share physical pins.** Instead, the RP2040's UART0 peripheral can be routed to different GPIO pins:
- When flashing ESP32: UART0 is routed to GPIO 16/17
- When reading GNSS: UART0 is routed to GPIO 0/1

## Switch Architecture

### UART Switch Manager (`uart_switch.hh/cc`)

A dedicated manager class that handles UART0 mode switching:

```cpp
class UARTSwitch {
    enum UARTMode {
        kModeNone,      // UART0 not initialized
        kModeESP32,     // UART0 configured for ESP32 programming
        kModeGNSS       // UART0 configured for GNSS communication
    };
}
```

#### Key Functions

1. **`InitForESP32(baudrate)`** - **NOTE: Currently NOT used by ESP32 flasher**
   - Would configure UART0 for ESP32 programming mode on GPIO 16/17
   - The ESP32SerialFlasher directly configures UART0 instead (using `bsp.esp32_uart_tx/rx_pin`)
   - Kept for potential future use

2. **`InitForGNSS(baudrate)`**
   - Configures UART0 for GNSS communication on GPIO 0/1
   - Sets up GPIO 0/1 as UART function pins
   - Initializes UART0 with GNSS baudrate (typically 9600 or configured value)
   - Called after ESP32 firmware verification completes

3. **`Deinit()`**
   - Waits for pending transmissions to complete
   - Deinitializes UART0 peripheral
   - Resets GPIO pins to high-impedance input mode
   - Allows clean switching between different UART0 configurations

4. **`GetCurrentMode()`**
   - Returns current UART0 operating mode
   - Used for state tracking and debugging

### Modified Initialization Sequence

The new boot sequence follows these steps:

```
1. System Boot
   ├─> ADSBee Init
   ├─> CommsManager Init (UART1 only, NOT UART0)
   ├─> Load Settings
   │
2. ESP32 Firmware Check Phase (if ESP32 enabled)
   ├─> ESP32::Init() via SPI
   ├─> Read ESP32 firmware version via SPI
   ├─> If version mismatch:
   │   ├─> ESP32::DeInit() (power down)
   │   ├─> ESP32SerialFlasher::Init()
   │   │   └─> Configure UART0 on GPIO 16/17 for ESP32
   │   ├─> Flash ESP32 via UART0 (GPIO 16/17)
   │   ├─> ESP32SerialFlasher::DeInit()
   │   │   └─> uart_deinit(uart0)
   │   └─> ESP32::Init() (re-initialize SPI)
   │
3. GNSS Initialization Phase
   ├─> CommsManager::InitGNSSUART()
   │   └─> UARTSwitch::InitForGNSS()
   │       └─> Configure UART0 on GPIO 0/1 for GNSS
   └─> GNSSManager::Initialize()

4. Normal Operation
   └─> UART0 remains configured for GNSS (GPIO 0/1)
```

## Implementation Details

### CommsManager Modifications

The CommsManager now has split initialization:

- **`Init()`** - Initializes UART1 and other peripherals, but NOT UART0
- **`InitGNSSUART()`** - Specifically initializes UART0 for GNSS (called after ESP32 check)

### ESP32 Flasher Updates

The ESP32SerialFlasher directly manages UART0 configuration for ESP32 programming:
- **Does NOT use UARTSwitch** - manages UART0 directly for maximum control
- Uses `bsp.esp32_uart_tx_pin` (GPIO 16) and `bsp.esp32_uart_rx_pin` (GPIO 17)
- Configures UART0 with explicit format: 8 data bits, no parity, 1 stop bit (8N1)
- Calls `uart_deinit(uart0)` in `DeInit()` to release UART0 when done
- Disables pull-ups on GPIO 13 (ESP32 GPIO0/boot pin) to ensure clean bootloader entry

### Main.cc Sequencing

The main initialization sequence:
1. Check ESP32 firmware version via SPI (UART0 not used yet)
2. If firmware mismatch detected:
   - Power down ESP32 via SPI
   - Call ESP32SerialFlasher::FlashESP32()
     - Configures UART0 on GPIO 16/17
     - Flashes ESP32
     - Deinitializes UART0
3. Initialize UART0 for GNSS on GPIO 0/1
4. Continue with normal operation

## State Management

The UARTSwitch maintains a static state variable to track the current mode:

```cpp
static UARTMode current_mode_ = kModeNone;
```

This ensures:
- No double initialization
- Proper deinitialization before mode switches
- Clear understanding of current UART0 state

## Error Handling

### Safeguards
1. **Mode Check**: Functions check if already in requested mode before switching
2. **TX Buffer Drain**: `uart_tx_wait_blocking()` ensures no data loss during switches
3. **GPIO Reset**: Pins are reset to high-impedance to prevent interference
4. **Console Logging**: Mode switches are logged for debugging

### Failure Scenarios
- If ESP32 firmware update fails, UART0 still switches to GNSS mode
- If GNSS initialization fails, system continues but without GPS functionality
- Console warnings are printed for any initialization failures

## Testing Considerations

### Verification Steps
1. **Power-on Reset**: Verify ESP32 firmware check occurs before GNSS init
2. **Firmware Update**: Confirm ESP32 can be updated when version mismatches
3. **Normal Boot**: Ensure GNSS receives data after ESP32 check completes
4. **Mode Switching**: Verify clean transitions between ESP32 and GNSS modes

### Debug Points
- Console messages indicate each mode transition
- UART0 state can be queried via `GetCurrentMode()`
- Both ESP32 and GNSS initialization report success/failure

### Known Limitations
- Cannot use ESP32 programming (GPIO 16/17) and GNSS (GPIO 0/1) simultaneously (UART0 constraint)
- UART0 reconfiguration requires deinitialization between different pin sets
- ESP32 firmware updates only possible at boot (not runtime)

## Important Hardware Notes

### ESP32 Communication Paths
The ESP32 has **two communication interfaces** with the RP2040:
1. **SPI Interface** (primary):
   - Used for normal operation (settings, data transfer, status)
   - GPIO 13 used as SPI handshake pin
2. **UART Interface** (programming only):
   - Used ONLY for firmware updates
   - GPIO 16/17 for UART communication
   - GPIO 13 doubles as ESP32's GPIO0 (bootloader entry pin)

### GPIO 13 Dual Purpose
**Critical:** GPIO 13 serves two functions:
- **During normal operation**: SPI handshake pin (has internal pull-up enabled)
- **During ESP32 flashing**: ESP32's GPIO0 boot pin (must be pulled LOW for bootloader entry)

The ESP32 flasher must explicitly disable pull-ups on GPIO 13 before attempting bootloader entry, otherwise the pull-up prevents the ESP32 from entering bootloader mode.

