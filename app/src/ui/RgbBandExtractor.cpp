// RGB line extraction from BIL 16-bit hyperspectral frames.
#include "ui/RgbBandExtractor.hpp"

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
constexpr double kDnAxisMax = 4096.0;
constexpr double kByteScale = 255.0 / kDnAxisMax;

void scaleBandRowToBytes(const std::uint16_t *row,
                         int width,
                         std::vector<std::uint8_t> &channelOut)
{
    channelOut.resize(static_cast<std::size_t>(width));

    if (width <= 0)
        return;

    for (int x = 0; x < width; ++x)
    {
        const double dn = static_cast<double>(row[x]);
        channelOut[static_cast<std::size_t>(x)] = static_cast<std::uint8_t>(
            std::min(255.0, std::max(0.0, dn * kByteScale)));
    }
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

    scaleBandRowToBytes(frame.pixels.data() + bands.red * rowPixels, width, redChannel);
    scaleBandRowToBytes(frame.pixels.data() + bands.green * rowPixels, width, greenChannel);
    scaleBandRowToBytes(frame.pixels.data() + bands.blue * rowPixels, width, blueChannel);

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
