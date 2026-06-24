// SWIR3 stream-adaptive BPR implementation (backend/processing layer).
#include "backend/processing/SwirAdaptiveBprCorrector.hpp"

#include "backend/processing/SwirBprApply.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace hf::processing
{
namespace
{
inline std::size_t fullResIndex(const int bands, const int samples, const int band, const int sample)
{
    return static_cast<std::size_t>(band) * static_cast<std::size_t>(samples)
           + static_cast<std::size_t>(sample);
}
} // namespace

bool SwirAdaptiveBprCorrector::loadFromCalpack(const QString &calpackPath, QString *errorMessage)
{
    resetAdaptiveState();
    if (!baseline_.loadFromCalpack(calpackPath, errorMessage))
        return false;

    adaptiveMask_.assign(baseline_.mask.size(), 0);
    hitCounts_.assign(baseline_.mask.size(), 0);
    return true;
}

void SwirAdaptiveBprCorrector::resetAdaptiveState()
{
    adaptiveMask_.clear();
    hitCounts_.clear();
    adaptiveBadPixelCount_ = 0;
    activeMaskBinned_.clear();
}

bool SwirAdaptiveBprCorrector::isBaselineBadFullRes(const int fullBand, const int fullSample) const
{
    if (!baseline_.isLoaded() || fullBand < 0 || fullBand >= baseline_.bands || fullSample < 0
        || fullSample >= baseline_.samples)
        return true;

    return baseline_.mask[fullResIndex(baseline_.bands, baseline_.samples, fullBand, fullSample)] != 0;
}

bool SwirAdaptiveBprCorrector::isAdaptiveBadFullRes(const int fullBand, const int fullSample) const
{
    if (adaptiveMask_.empty() || fullBand < 0 || fullBand >= baseline_.bands || fullSample < 0
        || fullSample >= baseline_.samples)
        return false;

    return adaptiveMask_[fullResIndex(baseline_.bands, baseline_.samples, fullBand, fullSample)] != 0;
}

void SwirAdaptiveBprCorrector::markAdaptiveFullRes(const int fullBand, const int fullSample)
{
    if (!baseline_.isLoaded() || isBaselineBadFullRes(fullBand, fullSample)
        || isAdaptiveBadFullRes(fullBand, fullSample))
        return;

    if (settings_.maxAdaptivePixels > 0
        && static_cast<int>(adaptiveBadPixelCount_) >= settings_.maxAdaptivePixels)
        return;

    const std::size_t index = fullResIndex(baseline_.bands, baseline_.samples, fullBand, fullSample);
    adaptiveMask_[index] = 1;
    ++adaptiveBadPixelCount_;
}

void SwirAdaptiveBprCorrector::markAdaptiveBinCell(const int band,
                                                   const int sample,
                                                   const int spatialBinning,
                                                   const int spectralBinning)
{
    const int spatBin = std::max(1, spatialBinning);
    const int specBin = std::max(1, spectralBinning);
    const int fullBandStart = band * specBin;
    const int fullSampleStart = sample * spatBin;

    for (int bandOffset = 0; bandOffset < specBin; ++bandOffset)
    {
        const int fullBand = fullBandStart + bandOffset;
        if (fullBand >= baseline_.bands)
            break;

        for (int sampleOffset = 0; sampleOffset < spatBin; ++sampleOffset)
        {
            const int fullSample = fullSampleStart + sampleOffset;
            if (fullSample >= baseline_.samples)
                break;

            markAdaptiveFullRes(fullBand, fullSample);
        }
    }
}

void SwirAdaptiveBprCorrector::detectAndUpdate(const FramePacket &frame,
                                              const int spatialBinning,
                                              const int spectralBinning)
{
    if (!baseline_.isLoaded() || frame.width <= 0 || frame.height <= 0 || frame.pixels.empty())
        return;

    const int width = frame.width;
    const int bands = frame.height;
    const int spatBin = std::max(1, spatialBinning);
    const int specBin = std::max(1, spectralBinning);

    const std::size_t planeSize = static_cast<std::size_t>(width) * static_cast<std::size_t>(bands);
    if (frame.pixels.size() < planeSize)
        return;

    const auto indexOf = [width](const int band, const int sample) {
        return static_cast<std::size_t>(band) * static_cast<std::size_t>(width)
               + static_cast<std::size_t>(sample);
    };

    const auto baselineGoodSpatialNeighbor = [&](const int band, const int sample, const int direction)
        -> std::optional<std::uint16_t> {
        for (int delta = 1; delta < width; ++delta)
        {
            const int neighborSample = sample + direction * delta;
            if (neighborSample < 0 || neighborSample >= width)
                return std::nullopt;

            const int fullBand = band * specBin;
            const int fullSample = neighborSample * spatBin;
            if (isBaselineBadFullRes(fullBand, fullSample))
                continue;

            return frame.pixels[indexOf(band, neighborSample)];
        }
        return std::nullopt;
    };

    for (int band = 0; band < bands; ++band)
    {
        for (int sample = 0; sample < width; ++sample)
        {
            const int fullBand = band * specBin;
            const int fullSample = sample * spatBin;
            if (isBaselineBadFullRes(fullBand, fullSample) || isAdaptiveBadFullRes(fullBand, fullSample))
                continue;

            const std::optional<std::uint16_t> left = baselineGoodSpatialNeighbor(band, sample, -1);
            const std::optional<std::uint16_t> right = baselineGoodSpatialNeighbor(band, sample, 1);
            if (!left || !right)
                continue;

            const double neighborMean = (static_cast<double>(*left) + static_cast<double>(*right)) * 0.5;
            if (neighborMean < settings_.minNeighborMeanDn)
                continue;

            const double value = static_cast<double>(frame.pixels[indexOf(band, sample)]);
            const double ratio = value / neighborMean;

            const std::size_t hitIndex =
                fullResIndex(baseline_.bands, baseline_.samples, fullBand, fullSample);

            if (ratio < settings_.gainMin || ratio > settings_.gainMax)
            {
                const int nextHits =
                    std::min(255, static_cast<int>(hitCounts_[hitIndex]) + 1);
                hitCounts_[hitIndex] = static_cast<std::uint8_t>(nextHits);

                if (nextHits >= settings_.minConsecutiveHits)
                    markAdaptiveBinCell(band, sample, spatBin, specBin);
            }
            else if (hitCounts_[hitIndex] > 0)
            {
                hitCounts_[hitIndex] = static_cast<std::uint8_t>(hitCounts_[hitIndex] - 1);
            }
        }
    }
}

void SwirAdaptiveBprCorrector::buildActiveMaskBinned(const int width,
                                                     const int bands,
                                                     const int spatialBinning,
                                                     const int spectralBinning)
{
    const int spatBin = std::max(1, spatialBinning);
    const int specBin = std::max(1, spectralBinning);

    activeMaskBinned_.assign(static_cast<std::size_t>(width) * static_cast<std::size_t>(bands), 0);

    for (int fullBand = 0; fullBand < baseline_.bands; ++fullBand)
    {
        for (int fullSample = 0; fullSample < baseline_.samples; ++fullSample)
        {
            if (!isBaselineBadFullRes(fullBand, fullSample)
                && !isAdaptiveBadFullRes(fullBand, fullSample))
                continue;

            const int band = fullBand / specBin;
            const int sample = fullSample / spatBin;
            if (band < 0 || band >= bands || sample < 0 || sample >= width)
                continue;

            activeMaskBinned_[static_cast<std::size_t>(band) * static_cast<std::size_t>(width)
                              + static_cast<std::size_t>(sample)] = 1;
        }
    }
}

void SwirAdaptiveBprCorrector::processFrame(FramePacket &frame,
                                            const int spatialBinning,
                                            const int spectralBinning)
{
    if (!isLoaded())
        return;

    detectAndUpdate(frame, spatialBinning, spectralBinning);
    buildActiveMaskBinned(frame.width, frame.height, spatialBinning, spectralBinning);
    applySwirBilBprInPlace(frame, spatialBinning, spectralBinning, activeMaskBinned_);
}

} // namespace hf::processing
