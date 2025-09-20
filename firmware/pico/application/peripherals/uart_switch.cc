#include "uart_switch.hh"

// Initialize static member
UARTSwitch::UARTMode UARTSwitch::current_mode_ = UARTSwitch::kModeNone;