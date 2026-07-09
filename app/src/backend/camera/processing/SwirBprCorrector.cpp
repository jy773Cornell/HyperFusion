// SWIR3 static calpack BPR (backend/processing layer).
#include "backend/camera/processing/SwirBprCorrector.hpp"

#include "backend/camera/processing/SwirBprApply.hpp"

#include <algorithm>

namespace hf::processing
{
bool SwirBprCorrector::loadFromCalpack(const QString &calpackPath, QString *errorMessage)
{
    binnedMaskScratch_.clear();
    return map_.loadFromCalpack(calpackPath, errorMessage);
}

void SwirBprCorrector::apply(FramePacket &frame, const int spatialBinning, const int spectralBinning) const
{
    if (!map_.isLoaded() || frame.width <= 0 || frame.height <= 0 || frame.pixels.empty())
        return;

    const int width = frame.width;
    const int bands = frame.height;
    const int spatBin = std::max(1, spatialBinning);
    const int specBin = std::max(1, spectralBinning);

    const std::size_t planeSize = static_cast<std::size_t>(width) * static_cast<std::size_t>(bands);
    if (frame.pixels.size() < planeSize)
        return;

    binnedMaskScratch_.assign(planeSize, 0);
    for (int fullBand = 0; fullBand < map_.bands; ++fullBand)
    {
        for (int fullSample = 0; fullSample < map_.samples; ++fullSample)
        {
            const std::size_t fullIndex =
                static_cast<std::size_t>(fullBand) * static_cast<std::size_t>(map_.samples)
                + static_cast<std::size_t>(fullSample);
            if (map_.mask[fullIndex] == 0)
                continue;

            const int band = fullBand / specBin;
            const int sample = fullSample / spatBin;
            if (band < 0 || band >= bands || sample < 0 || sample >= width)
                continue;

            binnedMaskScratch_[static_cast<std::size_t>(band) * static_cast<std::size_t>(width)
                               + static_cast<std::size_t>(sample)] = 1;
        }
    }

    applySwirBilBprInPlace(frame, spatialBinning, spectralBinning, binnedMaskScratch_);
}

} // namespace hf::processing
