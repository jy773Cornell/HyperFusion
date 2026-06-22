// RGB line extraction from BIL 16-bit hyperspectral frames.
#include "frontend/processing/RgbBandExtractor.hpp"

#include "frontend/processing/Overexposure.hpp"

#include <algorithm>

namespace ui
{
int mapCalpackBandToBilRow(const int calpackBandIndex,
                           const int frameBandCount,
                           const int /*spectralBinning*/)
{
    if (frameBandCount <= 0)
        return std::max(0, calpackBandIndex);

    // Combo indices come from wlcal{b}b.wls matching current spectral binning (row = band index).
    return std::clamp(calpackBandIndex, 0, frameBandCount - 1);
}

namespace
{
void scaleBandRowToBytes(const std::uint16_t *row,
                         const int width,
                         const CameraBackendId source,
                         std::vector<std::uint8_t> &channelOut)
{
    channelOut.resize(static_cast<std::size_t>(width));

    if (width <= 0)
        return;

    for (int x = 0; x < width; ++x)
        channelOut[static_cast<std::size_t>(x)] = dnToDisplayGray(row[x], source);
}
} // namespace

bool extractRgbLineFromBilFrame(const FramePacket &frame,
                                const RgbBandIndices &bands,
                                std::vector<std::uint8_t> &rgbRowOut)
{
    if (frame.width <= 0 || frame.height <= 0 || frame.pixels.empty())
        return false;

    const int width = frame.width;
    const int height = frame.height;
    if (bands.red < 0 || bands.red >= height || bands.green < 0 || bands.green >= height
        || bands.blue < 0 || bands.blue >= height)
        return false;

    const std::size_t rowPixels = static_cast<std::size_t>(width);
    const std::size_t required = rowPixels * static_cast<std::size_t>(height);
    if (frame.pixels.size() < required)
        return false;

    std::vector<std::uint8_t> redChannel;
    std::vector<std::uint8_t> greenChannel;
    std::vector<std::uint8_t> blueChannel;

    scaleBandRowToBytes(frame.pixels.data() + bands.red * rowPixels, width, frame.source, redChannel);
    scaleBandRowToBytes(frame.pixels.data() + bands.green * rowPixels, width, frame.source, greenChannel);
    scaleBandRowToBytes(frame.pixels.data() + bands.blue * rowPixels, width, frame.source, blueChannel);

    rgbRowOut.resize(rowPixels * 3);
    for (int x = 0; x < width; ++x)
    {
        const std::size_t index = static_cast<std::size_t>(x);
        rgbRowOut[index * 3 + 0] = redChannel[index];
        rgbRowOut[index * 3 + 1] = greenChannel[index];
        rgbRowOut[index * 3 + 2] = blueChannel[index];
    }

    return true;
}
} // namespace ui
