// Shared Mono12 saturation detection and RGB marking for detector and waterfall views.
#pragma once

#include "backend/CameraTypes.hpp"

#include <cstdint>
#include <vector>

namespace ui
{
/// Mono12 full-scale DN stored in 16-bit containers (FX10e / SWIR3).
constexpr std::uint16_t kMono12FullScaleDn = 4095;
constexpr double kDnAxisMax = 4096.0;

inline bool isOverexposedDn(const std::uint16_t dn) noexcept
{
    return dn >= kMono12FullScaleDn;
}

std::uint8_t dnToDisplayGray(const std::uint16_t dn);

/// Per spatial column: true when any band at that column is saturated.
std::vector<bool> overexposedSpatialColumns(const FramePacket &frame);

/// Forces saturated spatial columns to red in an RGB888 row (width × 3 bytes).
void markOverexposedColumnsRed(std::vector<std::uint8_t> &rgbRow,
                               const int width,
                               const FramePacket &frame);

} // namespace ui
