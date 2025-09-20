# UART0 Switching Design Documentation

## Overview

The ADSBee RP2040 has a hardware constraint where UART0 (GPIO pins 0/1) is physically shared between two components:
- **ESP32-S3 UART Programming Interface** - For firmware updates and bootloader access
- **GNSS Module UART Interface** - For GPS/GNSS data communication

Since both devices share the same physical UART0 TX/RX lines, they cannot be active simultaneously. This document describes the UART switching mechanism implemented to manage this shared resource.

## Hardware Configuration

### Pin Mapping
- **UART0 TX (GPIO 0)**: Connected to both ESP32 RX (programming) AND GNSS RX
- **UART0 RX (GPIO 1)**: Connected to both ESP32 TX (programming) AND GNSS TX

### Physical Constraint
The UART0 pins are electrically connected to both devices on the PCB. When UART0 is configured and transmitting, both devices receive the signal. This creates a conflict if both devices try to communicate simultaneously.

## The Problem

### Original Implementation Issue
The original firmware initialization sequence was:
1. Initialize all UARTs (including UART0 for GNSS) in `CommsManager::Init()`
2. Check ESP32 firmware version
3. Update ESP32 if needed

**Problem**: Once UART0 was initialized for GNSS in step 1, the ESP32 bootloader could no longer receive firmware update commands on those same lines, making ESP32 updates impossible.

### Critical
The RP2040 must check for ESP32 firmware updates **BEFORE** initializing UART0 for GNSS communication.

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

1. **`InitForESP32(baudrate)`**
   - Configures UART0 for ESP32 programming mode
   - Sets up GPIO 0/1 as UART pins
   - Initializes UART0 with ESP32 bootloader baudrate (typically 115200)
   - Called early in boot sequence

2. **`InitForGNSS(baudrate)`**
   - Configures UART0 for GNSS communication
   - Sets up GPIO 0/1 as UART pins (same pins, different purpose)
   - Initializes UART0 with GNSS baudrate
   - Called after ESP32 firmware verification

3. **`Deinit()`**
   - Waits for pending transmissions to complete
   - Deinitializes UART0 peripheral
   - Resets GPIO 0/1 to high-impedance input mode
   - Allows clean switching between modes

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
2. ESP32 Firmware Check Phase
   ├─> UARTSwitch::InitForESP32()  [UART0 -> ESP32 Mode]
   ├─> Attempt ESP32 communication
   ├─> Read ESP32 firmware version
   ├─> If version mismatch:
   │   ├─> Flash ESP32 via UART0
   │   └─> Verify update
   │
3. GNSS Initialization Phase
   ├─> UARTSwitch::Deinit()  [Release UART0]
   ├─> CommsManager::InitGNSSUART()
   │   └─> UARTSwitch::InitForGNSS()  [UART0 -> GNSS Mode]
   └─> GNSSManager::Initialize()
   
4. Normal Operation
   └─> UART0 remains in GNSS mode for duration of operation
```

## Implementation Details

### CommsManager Modifications

The CommsManager now has split initialization:

- **`Init()`** - Initializes UART1 and other peripherals, but NOT UART0
- **`InitGNSSUART()`** - Specifically initializes UART0 for GNSS (called after ESP32 check)

### ESP32 Flasher Updates

The ESP32SerialFlasher now:
- Uses `UARTSwitch::InitForESP32()` instead of direct UART initialization
- Properly releases UART0 with `UARTSwitch::Deinit()` when done
- Correctly uses GPIO 0/1 (not the incorrectly documented 16/17)

### Main.cc Sequencing

The main initialization now:
1. Initializes UART0 for ESP32 first
2. Performs ESP32 firmware check/update
3. Switches UART0 to GNSS mode
4. Continues with normal operation

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
- Cannot use ESP32 programming and GNSS simultaneously
- Mode switches require brief UART0 downtime
- ESP32 updates only possible at boot (not runtime)

