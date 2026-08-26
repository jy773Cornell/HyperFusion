// SWIR3 residual comb BPR from white/dark refs (backend/processing layer).
#include "backend/camera/processing/SwirRefBprCorrector.hpp"

#include "backend/camera/processing/EnviBilReader.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <type_traits>
#include <vector>

namespace hf::processing
{
namespace
{
double medianOf(std::vector<double> values)
{
    if (values.empty())
        return 0.0;

    const std::size_t mid = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(mid), values.end());
    if (values.size() % 2 == 1)
        return values[mid];

    const double upper = values[mid];
    const double lower =
        *std::max_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(mid));
    return (lower + upper) * 0.5;
}

std::optional<double> neighborMedian(const BilRowReference &row,
                                     const int bands,
                                     const int samples,
                                     const int band,
                                     const int sample,
                                     const int radius)
{
    if (band < 0 || band >= bands || sample < 0 || sample >= samples || radius < 1)
        return std::nullopt;

    std::vector<double> values;
    values.reserve(static_cast<std::size_t>(2 * radius));
    for (int neighbor = sample - radius; neighbor <= sample + radius; ++neighbor)
    {
        if (neighbor == sample || neighbor < 0 || neighbor >= samples)
            continue;
        values.push_back(static_cast<double>(row[bilLinePixelIndex(neighbor, band, samples)]));
    }

    if (values.size() < 2)
        return std::nullopt;
    return medianOf(std::move(values));
}

template <typename Pixel>
void interpolateBandMajorLine(Pixel *line,
                              const int samples,
                              const int bands,
                              const std::vector<std::uint8_t> &badMask)
{
    if (line == nullptr || samples <= 0 || bands <= 0 || badMask.empty())
        return;

    std::vector<int> goodSamples;
    goodSamples.reserve(static_cast<std::size_t>(samples));

    for (int band = 0; band < bands; ++band)
    {
        goodSamples.clear();
        for (int sample = 0; sample < samples; ++sample)
        {
            const std::size_t maskIndex =
                static_cast<std::size_t>(band) * static_cast<std::size_t>(samples)
                + static_cast<std::size_t>(sample);
            if (badMask[maskIndex] == 0)
                goodSamples.push_back(sample);
        }

        if (goodSamples.empty() || static_cast<int>(goodSamples.size()) == samples)
            continue;

        const int goodCount = static_cast<int>(goodSamples.size());
        for (int sample = 0; sample < samples; ++sample)
        {
            const std::size_t maskIndex =
                static_cast<std::size_t>(band) * static_cast<std::size_t>(samples)
                + static_cast<std::size_t>(sample);
            if (badMask[maskIndex] == 0)
                continue;

            const auto found =
                std::lower_bound(goodSamples.begin(), goodSamples.end(), sample);
            int pos = static_cast<int>(found - goodSamples.begin());
            const int leftIndex = std::clamp(pos - 1, 0, goodCount - 1);
            const int rightIndex = std::clamp(pos, 0, goodCount - 1);
            const int leftSample = goodSamples[static_cast<std::size_t>(leftIndex)];
            const int rightSample = goodSamples[static_cast<std::size_t>(rightIndex)];
            const int span = std::max(1, rightSample - leftSample);
            const double wRight =
                static_cast<double>(sample - leftSample) / static_cast<double>(span);
            const double wLeft = 1.0 - wRight;

            const double leftValue =
                static_cast<double>(line[bilLinePixelIndex(leftSample, band, samples)]);
            const double rightValue =
                static_cast<double>(line[bilLinePixelIndex(rightSample, band, samples)]);
            const double blended = leftValue * wLeft + rightValue * wRight;

            if constexpr (std::is_same_v<Pixel, std::uint16_t>)
            {
                const long rounded = std::lround(blended);
                line[bilLinePixelIndex(sample, band, samples)] =
                    static_cast<std::uint16_t>(std::clamp(rounded, 0L, 65535L));
            }
            else
            {
                line[bilLinePixelIndex(sample, band, samples)] = static_cast<Pixel>(blended);
            }
        }
    }
}
} // namespace

bool SwirRefBprCorrector::buildFromReferences(const BilRowReference &whiteRow,
                                              const BilRowReference &darkRow,
                                              const int samples,
                                              const int bands,
                                              QString *errorMessage)
{
    ready_ = false;
    badMask_.clear();
    badPixelCount_ = 0;
    badColumnCount_ = 0;
    samples_ = 0;
    bands_ = 0;

    if (samples <= 1 || bands <= 0)
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("SWIR ref BPR needs samples > 1 and bands > 0.");
        return false;
    }

    const std::size_t expected =
        static_cast<std::size_t>(samples) * static_cast<std::size_t>(bands);
    if (whiteRow.size() != expected || darkRow.size() != expected)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage =
                QStringLiteral("SWIR ref BPR row size mismatch (expected %1, white %2, dark %3).")
                    .arg(expected)
                    .arg(whiteRow.size())
                    .arg(darkRow.size());
        }
        return false;
    }

    const int radius = std::max(1, settings_.baselineRadius);
    const double whiteLo = std::clamp(settings_.whiteRatioMin, 0.1, 1.0);
    const double whiteHi = std::max(settings_.whiteRatioMax, 1.0);
    const double promoteFrac = std::clamp(settings_.columnPromoteFrac, 0.01, 1.0);

    std::vector<double> darkResiduals;
    darkResiduals.reserve(expected);
    std::vector<std::uint8_t> mask(expected, 0);

    for (int band = 0; band < bands; ++band)
    {
        for (int sample = 0; sample < samples; ++sample)
        {
            const std::optional<double> darkBaseline =
                neighborMedian(darkRow, bands, samples, band, sample, radius);
            if (!darkBaseline)
                continue;
            const double darkValue =
                static_cast<double>(darkRow[bilLinePixelIndex(sample, band, samples)]);
            darkResiduals.push_back(std::abs(darkValue - *darkBaseline));
        }
    }

    const double darkMedian = medianOf(darkResiduals);
    const double darkThresh =
        std::max(settings_.darkAbsMinDn, settings_.darkAbsScale * darkMedian);

    for (int band = 0; band < bands; ++band)
    {
        for (int sample = 0; sample < samples; ++sample)
        {
            const std::optional<double> whiteBaseline =
                neighborMedian(whiteRow, bands, samples, band, sample, radius);
            const std::optional<double> darkBaseline =
                neighborMedian(darkRow, bands, samples, band, sample, radius);
            if (!whiteBaseline || !darkBaseline)
                continue;

            const std::size_t index = static_cast<std::size_t>(band)
                                          * static_cast<std::size_t>(samples)
                                      + static_cast<std::size_t>(sample);
            const double whiteValue =
                static_cast<double>(whiteRow[bilLinePixelIndex(sample, band, samples)]);
            const double darkValue =
                static_cast<double>(darkRow[bilLinePixelIndex(sample, band, samples)]);
            const double whiteRatio = (whiteValue + 1.0) / (*whiteBaseline + 1.0);
            const bool whiteOutlier = whiteRatio < whiteLo || whiteRatio > whiteHi;
            const bool darkOutlier = std::abs(darkValue - *darkBaseline) > darkThresh;
            if (whiteOutlier || darkOutlier)
                mask[index] = 1;
        }
    }

    for (int sample = 0; sample < samples; ++sample)
    {
        int hits = 0;
        for (int band = 0; band < bands; ++band)
        {
            const std::size_t index = static_cast<std::size_t>(band)
                                          * static_cast<std::size_t>(samples)
                                      + static_cast<std::size_t>(sample);
            if (mask[index] != 0)
                ++hits;
        }

        if (static_cast<double>(hits) / static_cast<double>(bands) < promoteFrac)
            continue;

        ++badColumnCount_;
        for (int band = 0; band < bands; ++band)
        {
            const std::size_t index = static_cast<std::size_t>(band)
                                          * static_cast<std::size_t>(samples)
                                      + static_cast<std::size_t>(sample);
            mask[index] = 1;
        }
    }

    for (const std::uint8_t flag : mask)
    {
        if (flag != 0)
            ++badPixelCount_;
    }

    samples_ = samples;
    bands_ = bands;
    badMask_ = std::move(mask);
    ready_ = true;
    return true;
}

void SwirRefBprCorrector::interpolateSpatial(float *bandMajorLine) const
{
    interpolateBandMajorLine(bandMajorLine, samples_, bands_, badMask_);
}

void SwirRefBprCorrector::interpolateSpatial(std::uint16_t *bandMajorLine) const
{
    interpolateBandMajorLine(bandMajorLine, samples_, bands_, badMask_);
}

void SwirRefBprCorrector::applyToFloatRow(BilRowReference &row) const
{
    if (!ready_ || row.size() != badMask_.size())
        return;
    interpolateSpatial(row.data());
}

void SwirRefBprCorrector::applyToUint16Line(std::uint16_t *linePixels) const
{
    if (!ready_ || linePixels == nullptr)
        return;
    interpolateSpatial(linePixels);
}

} // namespace hf::processing
