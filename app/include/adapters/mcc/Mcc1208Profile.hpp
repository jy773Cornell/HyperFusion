#pragma once

namespace mcc
{
// Measurement Computing device ID (Universal Library board type).
inline constexpr int kUsb1208FsPlusBoardType = 232;

inline constexpr int kMaxConfiguredBoards = 32;
inline constexpr float kAnalogOutputVoltsMax = 5.0f;

// MCC UL range codes (cbw.h). UNI5VOLTS is valid for analog output only on this board.
inline constexpr int kAnalogOutputRangeUni5Volts = 101;
inline constexpr int kAnalogInputRangeSingleEnded10V = 1; // BIP10VOLTS, ±10 V SE

// cbAInputMode: SINGLE_ENDED (not RSE — RSE is PCI-6000 only).
inline constexpr int kAnalogInputModeSingleEnded = 1;
} // namespace mcc
