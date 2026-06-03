// BIL profile extraction (spectral column and spatial row).
#include "ui/BilProfileExtractor.hpp"

#include <algorithm>

namespace ui
{
bool extractProfilesFromBilFrame(const FramePacket &frame,
                                 const ProfileCursor &cursor,
                                 ProfileExtraction &out)
{
    out = ProfileExtraction{};
    if (frame.width <= 0 || frame.height <= 0 || frame.pixels.empty())
        return false;

    const int width = frame.width;
    const int bands = frame.height;
    const std::size_t required = static_cast<std::size_t>(width) * static_cast<std::size_t>(bands);
    if (frame.pixels.size() < required)
        return false;

    const int spatialX = std::clamp(cursor.spatialX, 0, width - 1);
    const int bandY = std::clamp(cursor.bandY, 0, bands - 1);

    out.wavelengthDn.resize(static_cast<std::size_t>(bands));
    for (int band = 0; band < bands; ++band)
    {
        out.wavelengthDn[static_cast<std::size_t>(band)] =
            frame.pixels[static_cast<std::size_t>(band) * static_cast<std::size_t>(width)
                         + static_cast<std::size_t>(spatialX)];
    }

    out.spatialDn.resize(static_cast<std::size_t>(width));
    const std::size_t rowOffset =
        static_cast<std::size_t>(bandY) * static_cast<std::size_t>(width);
    for (int x = 0; x < width; ++x)
        out.spatialDn[static_cast<std::size_t>(x)] = frame.pixels[rowOffset + static_cast<std::size_t>(x)];

    out.cursor.spatialX = spatialX;
    out.cursor.bandY = bandY;
    out.frameWidth = width;
    out.frameBands = bands;
    out.valid = true;
    return true;
}
} // namespace ui
