// In-place BIL bad-pixel replacement for SWIR3 frames (backend/processing).
#include "backend/processing/SwirBprApply.hpp"

#include <algorithm>
#include <optional>

namespace hf::processing
{
void applySwirBilBprInPlace(FramePacket &frame,
                            const int spatialBinning,
                            const int spectralBinning,
                            const std::vector<std::uint8_t> &badMaskBinned)
{
    if (frame.width <= 0 || frame.height <= 0 || frame.pixels.empty() || badMaskBinned.empty())
        return;

    const int width = frame.width;
    const int bands = frame.height;
    const int spatBin = std::max(1, spatialBinning);
    const int specBin = std::max(1, spectralBinning);

    const std::size_t planeSize = static_cast<std::size_t>(width) * static_cast<std::size_t>(bands);
    if (frame.pixels.size() < planeSize || badMaskBinned.size() < planeSize)
        return;

    const auto indexOf = [width](const int band, const int sample) {
        return static_cast<std::size_t>(band) * static_cast<std::size_t>(width)
               + static_cast<std::size_t>(sample);
    };

    const auto isBad = [&](const int band, const int sample) {
        if (band < 0 || band >= bands || sample < 0 || sample >= width)
            return true;
        return badMaskBinned[indexOf(band, sample)] != 0;
    };

    const auto nearestGoodSpatialNeighbor = [&](const int band, const int sample, const int direction)
        -> std::optional<std::uint16_t> {
        for (int delta = 1; delta < width; ++delta)
        {
            const int neighborSample = sample + direction * delta;
            if (neighborSample < 0 || neighborSample >= width)
                return std::nullopt;
            if (isBad(band, neighborSample))
                continue;
            return frame.pixels[indexOf(band, neighborSample)];
        }
        return std::nullopt;
    };

    const auto nearestGoodSpectralNeighbor = [&](const int band, const int sample, const int direction)
        -> std::optional<std::uint16_t> {
        for (int delta = 1; delta < bands; ++delta)
        {
            const int neighborBand = band + direction * delta;
            if (neighborBand < 0 || neighborBand >= bands)
                return std::nullopt;
            if (isBad(neighborBand, sample))
                continue;
            return frame.pixels[indexOf(neighborBand, sample)];
        }
        return std::nullopt;
    };

    for (int band = 0; band < bands; ++band)
    {
        for (int sample = 0; sample < width; ++sample)
        {
            if (!isBad(band, sample))
                continue;

            const std::optional<std::uint16_t> left = nearestGoodSpatialNeighbor(band, sample, -1);
            const std::optional<std::uint16_t> right = nearestGoodSpatialNeighbor(band, sample, 1);
            const std::optional<std::uint16_t> up = nearestGoodSpectralNeighbor(band, sample, -1);
            const std::optional<std::uint16_t> down = nearestGoodSpectralNeighbor(band, sample, 1);

            std::uint32_t sum = 0;
            int count = 0;
            for (const std::optional<std::uint16_t> neighbor : {left, right, up, down})
            {
                if (!neighbor)
                    continue;
                sum += *neighbor;
                ++count;
            }

            if (count > 0)
                frame.pixels[indexOf(band, sample)] = static_cast<std::uint16_t>(sum / count);
        }
    }

    (void)spatBin;
    (void)specBin;
}

} // namespace hf::processing
