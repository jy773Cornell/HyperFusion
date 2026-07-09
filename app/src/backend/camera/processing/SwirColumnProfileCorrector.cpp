// SWIR3 column profile destripe implementation (backend/processing layer).
#include "backend/processing/SwirColumnProfileCorrector.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace hf::processing
{
void SwirColumnProfileCorrector::resetState()
{
    profile_.clear();
    columnBad_.clear();
    columnHits_.clear();
    badColumnCount_ = 0;
}

void SwirColumnProfileCorrector::buildSpatialProfile(const FramePacket &frame)
{
    const int width = frame.width;
    const int bands = frame.height;
    if (width <= 0 || bands <= 0 || frame.pixels.empty())
        return;

    const std::size_t planeSize = static_cast<std::size_t>(width) * static_cast<std::size_t>(bands);
    if (frame.pixels.size() < planeSize)
        return;

    profile_.assign(static_cast<std::size_t>(width), 0.0);

    const auto indexOf = [width](const int band, const int sample) {
        return static_cast<std::size_t>(band) * static_cast<std::size_t>(width)
               + static_cast<std::size_t>(sample);
    };

    const int minBands = std::max(1, settings_.minBandsPerColumn);
    std::vector<double> bandValues;
    bandValues.reserve(static_cast<std::size_t>(bands));

    for (int sample = 0; sample < width; ++sample)
    {
        bandValues.clear();
        for (int band = 0; band < bands; ++band)
        {
            const double value = static_cast<double>(frame.pixels[indexOf(band, sample)]);
            if (value >= settings_.minBandDn)
                bandValues.push_back(value);
        }

        if (static_cast<int>(bandValues.size()) < minBands)
        {
            profile_[static_cast<std::size_t>(sample)] = 0.0;
            continue;
        }

        const std::size_t mid = bandValues.size() / 2;
        std::nth_element(bandValues.begin(),
                         bandValues.begin() + static_cast<std::ptrdiff_t>(mid),
                         bandValues.end());
        if (bandValues.size() % 2 == 1)
            profile_[static_cast<std::size_t>(sample)] = bandValues[mid];
        else
        {
            const double upper = bandValues[mid];
            const double lower = *std::max_element(bandValues.begin(),
                                                   bandValues.begin()
                                                       + static_cast<std::ptrdiff_t>(mid));
            profile_[static_cast<std::size_t>(sample)] = (lower + upper) * 0.5;
        }
    }
}

std::optional<double> SwirColumnProfileCorrector::spatialBaselineMedian(const int sample,
                                                                        const int width) const
{
    if (profile_.empty() || sample < 0 || sample >= width)
        return std::nullopt;

    const int radius = std::max(1, settings_.baselineRadius);
    std::vector<double> values;
    values.reserve(static_cast<std::size_t>(2 * radius));

    for (int neighbor = sample - radius; neighbor <= sample + radius; ++neighbor)
    {
        if (neighbor == sample || neighbor < 0 || neighbor >= width)
            continue;

        const double value = profile_[static_cast<std::size_t>(neighbor)];
        if (value <= 0.0)
            continue;

        values.push_back(value);
    }

    if (values.size() < 2)
        return std::nullopt;

    const std::size_t mid = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(mid), values.end());
    if (values.size() % 2 == 1)
        return values[mid];

    const double upper = values[mid];
    const double lower =
        *std::max_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(mid));
    return (lower + upper) * 0.5;
}

void SwirColumnProfileCorrector::detectValleyColumns()
{
    const int width = static_cast<int>(profile_.size());
    if (width <= 0)
        return;

    if (columnBad_.size() != static_cast<std::size_t>(width))
    {
        columnBad_.assign(static_cast<std::size_t>(width), 0);
        columnHits_.assign(static_cast<std::size_t>(width), 0);
        badColumnCount_ = 0;
    }

    const int requiredHits =
        settings_.minConsecutiveHits < 1 ? 1 : settings_.minConsecutiveHits;
    const double valleyGainMin = std::clamp(settings_.valleyGainMin, 0.1, 1.0);

    for (int sample = 0; sample < width; ++sample)
    {
        const double profileValue = profile_[static_cast<std::size_t>(sample)];
        if (profileValue <= 0.0)
            continue;

        const std::optional<double> baseline = spatialBaselineMedian(sample, width);
        if (!baseline || *baseline < settings_.minBandDn)
            continue;

        const double ratio = profileValue / *baseline;
        const bool isValley = ratio < valleyGainMin
                              && (settings_.minValleyDn <= 0.0
                                  || (*baseline - profileValue) >= settings_.minValleyDn);

        const std::size_t index = static_cast<std::size_t>(sample);
        if (!isValley)
        {
            if (columnHits_[index] > 0)
                columnHits_[index] = static_cast<std::uint8_t>(columnHits_[index] - 1);
            continue;
        }

        const int nextHits =
            std::min(255, static_cast<int>(columnHits_[index]) + 1);
        columnHits_[index] = static_cast<std::uint8_t>(nextHits);
        if (nextHits < requiredHits)
            continue;

        if (columnBad_[index] == 0)
        {
            columnBad_[index] = 1;
            ++badColumnCount_;
        }
    }
}

void SwirColumnProfileCorrector::correctBadColumns(FramePacket &frame) const
{
    const int width = frame.width;
    const int bands = frame.height;
    if (width <= 0 || bands <= 0 || frame.pixels.empty() || columnBad_.empty())
        return;

    const std::size_t planeSize = static_cast<std::size_t>(width) * static_cast<std::size_t>(bands);
    if (frame.pixels.size() < planeSize
        || columnBad_.size() != static_cast<std::size_t>(width))
        return;

    const auto indexOf = [width](const int band, const int sample) {
        return static_cast<std::size_t>(band) * static_cast<std::size_t>(width)
               + static_cast<std::size_t>(sample);
    };

    const auto nearestGoodValue = [&](const int band, const int sample, const int direction)
        -> std::optional<std::uint16_t> {
        for (int delta = 1; delta < width; ++delta)
        {
            const int neighborSample = sample + direction * delta;
            if (neighborSample < 0 || neighborSample >= width)
                return std::nullopt;
            if (columnBad_[static_cast<std::size_t>(neighborSample)] != 0)
                continue;
            return frame.pixels[indexOf(band, neighborSample)];
        }
        return std::nullopt;
    };

    for (int band = 0; band < bands; ++band)
    {
        for (int sample = 0; sample < width; ++sample)
        {
            if (columnBad_[static_cast<std::size_t>(sample)] == 0)
                continue;

            const std::optional<std::uint16_t> left = nearestGoodValue(band, sample, -1);
            const std::optional<std::uint16_t> right = nearestGoodValue(band, sample, 1);

            std::uint32_t sum = 0;
            int count = 0;
            for (const std::optional<std::uint16_t> neighbor : {left, right})
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
}

void SwirColumnProfileCorrector::processFrame(FramePacket &frame)
{
    if (frame.width <= 0 || frame.height <= 0 || frame.pixels.empty())
        return;

    buildSpatialProfile(frame);
    detectValleyColumns();
    correctBadColumns(frame);
}

} // namespace hf::processing
