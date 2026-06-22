// Saturation detection and DN→display scaling for detector and waterfall views.
// FX10e (Camera1): Mono12 in 16-bit containers. SWIR3 (Camera2): native 16-bit DN.
// Red saturation overlay is applied on the detector view only (not waterfall).
#pragma once

#include "backend/CameraTypes.hpp"

#include <cstdint>
#include <vector>

namespace ui
{
/// Mono12 full-scale DN (FX10e and similar).
constexpr std::uint16_t kMono12FullScaleDn = 4095;
constexpr double kMono12DnAxisMax = 4096.0;

/// Native 16-bit DN full scale (SWIR3).
constexpr std::uint16_t kMono16FullScaleDn = 65535;
constexpr double kMono16DnAxisMax = 65536.0;

/// Default profile/detector scaling axis (Mono12).
constexpr double kDnAxisMax = kMono12DnAxisMax;

inline std::uint16_t dnFullScaleForSource(const CameraBackendId source) noexcept
{
    return source == CameraBackendId::Camera2 ? kMono16FullScaleDn : kMono12FullScaleDn;
}

inline double dnAxisMaxForSource(const CameraBackendId source) noexcept
{
    return source == CameraBackendId::Camera2 ? kMono16DnAxisMax : kMono12DnAxisMax;
}

inline bool isOverexposedDn(const std::uint16_t dn, const CameraBackendId source) noexcept
{
    return dn >= dnFullScaleForSource(source);
}

std::uint8_t dnToDisplayGray(std::uint16_t dn, CameraBackendId source);

/// Per spatial column: true when any band at that column is saturated.
std::vector<bool> overexposedSpatialColumns(const FramePacket &frame);

/// Forces saturated spatial columns to red in an RGB888 row (width × 3 bytes).
void markOverexposedColumnsRed(std::vector<std::uint8_t> &rgbRow,
                               const int width,
                               const FramePacket &frame);

} // namespace ui
