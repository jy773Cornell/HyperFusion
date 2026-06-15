// Mono12 saturation helpers for detector and waterfall visualization.
#include "frontend/processing/Overexposure.hpp"

#include <algorithm>
#include <cmath>

namespace ui
{
namespace
{
constexpr double kByteScale = 255.0 / kDnAxisMax;
} // namespace

std::uint8_t dnToDisplayGray(const std::uint16_t dn)
{
    const double scaled = static_cast<double>(dn) * kByteScale;
    return static_cast<std::uint8_t>(std::min(255.0, std::max(0.0, scaled)));
}

std::vector<bool> overexposedSpatialColumns(const FramePacket &frame)
{
    const int width = frame.width;
    const int height = frame.height;
    if (width <= 0 || height <= 0 || frame.pixels.empty())
        return {};

    const std::size_t rowPixels = static_cast<std::size_t>(width);
    const std::size_t required = rowPixels * static_cast<std::size_t>(height);
    if (frame.pixels.size() < required)
        return {};

    std::vector<bool> mask(static_cast<std::size_t>(width), false);
    for (int band = 0; band < height; ++band)
    {
        const std::uint16_t *row = frame.pixels.data() + band * rowPixels;
        for (int x = 0; x < width; ++x)
        {
            if (isOverexposedDn(row[x]))
                mask[static_cast<std::size_t>(x)] = true;
        }
    }

    return mask;
}

void markOverexposedColumnsRed(std::vector<std::uint8_t> &rgbRow,
                               const int width,
                               const FramePacket &frame)
{
    if (width <= 0 || rgbRow.size() < static_cast<std::size_t>(width) * 3)
        return;

    const std::vector<bool> columnMask = overexposedSpatialColumns(frame);
    if (columnMask.size() != static_cast<std::size_t>(width))
        return;

    for (int x = 0; x < width; ++x)
    {
        if (!columnMask[static_cast<std::size_t>(x)])
            continue;

        const std::size_t base = static_cast<std::size_t>(x) * 3;
        rgbRow[base + 0] = 255;
        rgbRow[base + 1] = 0;
        rgbRow[base + 2] = 0;
    }
}

} // namespace ui
