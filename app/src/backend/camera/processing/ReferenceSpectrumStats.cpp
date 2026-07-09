#include "backend/camera/processing/ReferenceSpectrumStats.hpp"

#include <cmath>

namespace hf::processing
{
namespace
{
bool finalizeBandStats(const int bands,
                       const std::vector<double> &sum,
                       const std::vector<double> &sumSq,
                       const std::uint64_t count,
                       BandMeanStd &statsOut,
                       QString *errorMessage)
{
    if (count == 0 || bands <= 0)
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("No pixels found in ENVI cube.");
        return false;
    }

    const double n = static_cast<double>(count);
    statsOut.meanDn.resize(static_cast<std::size_t>(bands));
    statsOut.stdDn.resize(static_cast<std::size_t>(bands));

    for (int band = 0; band < bands; ++band)
    {
        const std::size_t index = static_cast<std::size_t>(band);
        const double mean = sum[index] / n;
        const double variance = std::max(0.0, (sumSq[index] / n) - (mean * mean));
        statsOut.meanDn[index] = mean;
        statsOut.stdDn[index] = std::sqrt(variance);
    }

    return true;
}
} // namespace

bool computeBandMeanStd(const EnviBilMetadata &metadata, BandMeanStd &statsOut, QString *errorMessage)
{
    statsOut = BandMeanStd{};
    statsOut.wavelengthsNm = metadata.wavelengthsNm;

    const int bands = metadata.bands;
    const int samples = metadata.samples;
    if (bands <= 0 || samples <= 0)
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Cannot compute statistics for empty ENVI cube.");
        return false;
    }

    std::vector<double> sum(static_cast<std::size_t>(bands), 0.0);
    std::vector<double> sumSq(static_cast<std::size_t>(bands), 0.0);
    std::uint64_t pixelCount = 0;

    const bool ok = readEnviBilLines(
        metadata,
        [&](const std::uint16_t *linePixels) {
            for (int sample = 0; sample < samples; ++sample)
            {
                for (int band = 0; band < bands; ++band)
                {
                    const std::size_t index = bilLinePixelIndex(sample, band, samples);
                    const double value = static_cast<double>(linePixels[index]);
                    sum[static_cast<std::size_t>(band)] += value;
                    sumSq[static_cast<std::size_t>(band)] += value * value;
                }
                ++pixelCount;
            }
            return true;
        },
        errorMessage);

    if (!ok || pixelCount == 0)
    {
        if (errorMessage != nullptr && errorMessage->isEmpty())
            *errorMessage = QStringLiteral("No pixels found in ENVI cube.");
        return false;
    }

    return finalizeBandStats(bands, sum, sumSq, pixelCount, statsOut, errorMessage);
}

bool computeBandMeanStdAcrossFrames(const EnviBilMetadata &metadata,
                                    BandMeanStd &statsOut,
                                    QString *errorMessage)
{
    statsOut = BandMeanStd{};
    statsOut.wavelengthsNm = metadata.wavelengthsNm;

    const int bands = metadata.bands;
    const int samples = metadata.samples;
    if (bands <= 0 || samples <= 0)
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Cannot compute statistics for empty ENVI cube.");
        return false;
    }

    std::vector<double> sum(static_cast<std::size_t>(bands), 0.0);
    std::vector<double> sumSq(static_cast<std::size_t>(bands), 0.0);
    std::uint64_t frameCount = 0;

    const bool ok = readEnviBilLines(
        metadata,
        [&](const std::uint16_t *linePixels) {
            for (int band = 0; band < bands; ++band)
            {
                double bandMean = 0.0;
                for (int sample = 0; sample < samples; ++sample)
                {
                    const std::size_t index = bilLinePixelIndex(sample, band, samples);
                    bandMean += static_cast<double>(linePixels[index]);
                }
                bandMean /= static_cast<double>(samples);

                const std::size_t bandIndex = static_cast<std::size_t>(band);
                sum[bandIndex] += bandMean;
                sumSq[bandIndex] += bandMean * bandMean;
            }

            ++frameCount;
            return true;
        },
        errorMessage);

    if (!ok)
        return false;

    return finalizeBandStats(bands, sum, sumSq, frameCount, statsOut, errorMessage);
}

bool computeBandMeanStdFromHdr(const QString &hdrPath, BandMeanStd &statsOut, QString *errorMessage)
{
    EnviBilMetadata metadata;
    if (!parseEnviHdr(hdrPath, metadata, errorMessage))
        return false;

    return computeBandMeanStd(metadata, statsOut, errorMessage);
}

bool computeBandMeanStdAcrossFramesFromHdr(const QString &hdrPath,
                                           BandMeanStd &statsOut,
                                           QString *errorMessage)
{
    EnviBilMetadata metadata;
    if (!parseEnviHdr(hdrPath, metadata, errorMessage))
        return false;

    return computeBandMeanStdAcrossFrames(metadata, statsOut, errorMessage);
}

bool computeBandMeanStdFromRowReference(const BilRowReference &row,
                                        const EnviBilMetadata &metadata,
                                        BandMeanStd &statsOut,
                                        QString *errorMessage)
{
    statsOut = BandMeanStd{};
    statsOut.wavelengthsNm = metadata.wavelengthsNm;

    const int bands = metadata.bands;
    const int samples = metadata.samples;
    if (bands <= 0 || samples <= 0)
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Cannot compute statistics for empty row reference.");
        return false;
    }

    const std::size_t expected =
        static_cast<std::size_t>(samples) * static_cast<std::size_t>(bands);
    if (row.size() != expected)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("Row reference size mismatch (expected %1, got %2).")
                                 .arg(expected)
                                 .arg(row.size());
        }
        return false;
    }

    std::vector<double> sum(static_cast<std::size_t>(bands), 0.0);
    std::vector<double> sumSq(static_cast<std::size_t>(bands), 0.0);

    for (int sample = 0; sample < samples; ++sample)
    {
        for (int band = 0; band < bands; ++band)
        {
            const std::size_t index = bilLinePixelIndex(sample, band, samples);
            const double value = static_cast<double>(row[index]);
            sum[static_cast<std::size_t>(band)] += value;
            sumSq[static_cast<std::size_t>(band)] += value * value;
        }
    }

    const std::uint64_t sampleCount = static_cast<std::uint64_t>(samples);
    return finalizeBandStats(bands, sum, sumSq, sampleCount, statsOut, errorMessage);
}

} // namespace hf::processing
