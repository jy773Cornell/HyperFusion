// Adapter: USB-1208FS-Plus UL constants (board type, analog ranges).
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

// USB-1208FS-Plus analog SE vs DIFF is an InstaCal setting, not cbAInputMode
// (that UL call returns BADFUNCTION until the USB device is opened, then is
// still the wrong API for this board). Keep InstaCal on single-ended for AI CH0–3.
} // namespace mcc
