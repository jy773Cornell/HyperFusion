#include "backend/camera/processing/ReferenceBuilder.hpp"

#include "backend/camera/processing/ReferenceSpectrumStats.hpp"

#include <cmath>
namespace hf::processing
{
namespace
{
bool accumulateImageBandStats(const int bands,
                              const int samples,
                              const std::uint16_t *linePixels,
                              std::vector<double> &bandSum,
                              std::vector<double> &bandSumSq)
{
    for (int band = 0; band < bands; ++band)
    {
        const std::size_t bandIndex = static_cast<std::size_t>(band);
        for (int sample = 0; sample < samples; ++sample)
        {
            const double value =
                static_cast<double>(linePixels[bilLinePixelIndex(sample, band, samples)]);
            bandSum[bandIndex] += value;
            bandSumSq[bandIndex] += value * value;
        }
    }

    return true;
}

bool finalizeBandStats(const int bands,
                            const std::vector<double> &bandSum,
                            const std::vector<double> &bandSumSq,
                            const std::uint64_t pixelCount,
                            const std::vector<double> &wavelengthsNm,
                            BandMeanStd &statsOut,
                            QString *errorMessage)
{
    statsOut = BandMeanStd{};
    statsOut.wavelengthsNm = wavelengthsNm;

    if (pixelCount == 0 || bands <= 0)
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Reference cube has no pixels.");
        return false;
    }

    const double n = static_cast<double>(pixelCount);
    statsOut.meanDn.resize(static_cast<std::size_t>(bands));
    statsOut.stdDn.resize(static_cast<std::size_t>(bands));

    for (int band = 0; band < bands; ++band)
    {
        const std::size_t index = static_cast<std::size_t>(band);
        const double mean = bandSum[index] / n;
        const double variance = std::max(0.0, (bandSumSq[index] / n) - (mean * mean));
        statsOut.meanDn[index] = mean;
        statsOut.stdDn[index] = std::sqrt(variance);
    }

    return true;
}
} // namespace

bool buildRowMeanReference(const QString &hdrPath,
                           BilRowReference &rowOut,
                           EnviBilMetadata &metadataOut,
                           QString *errorMessage)
{
    rowOut.clear();
    if (!parseEnviHdr(hdrPath, metadataOut, errorMessage))
        return false;

    const int samples = metadataOut.samples;
    const int bands = metadataOut.bands;
    const std::size_t rowPixels =
        static_cast<std::size_t>(samples) * static_cast<std::size_t>(bands);

    std::vector<double> sum(rowPixels, 0.0);
    int lineCount = 0;

    const bool ok = readEnviBilLines(
        metadataOut,
        [&](const std::uint16_t *linePixels) {
            for (std::size_t i = 0; i < rowPixels; ++i)
                sum[i] += static_cast<double>(linePixels[i]);
            ++lineCount;
            return true;
        },
        errorMessage);

    if (!ok || lineCount <= 0)
    {
        if (errorMessage != nullptr && errorMessage->isEmpty())
            *errorMessage = QStringLiteral("Reference cube has no lines: %1").arg(hdrPath);
        return false;
    }

    rowOut.resize(rowPixels);
    const double invLines = 1.0 / static_cast<double>(lineCount);
    for (std::size_t i = 0; i < rowPixels; ++i)
        rowOut[i] = static_cast<float>(sum[i] * invLines);

    metadataOut.lines = 1;
    return true;
}

bool buildRowMeanReferenceAndFrameStats(const QString &hdrPath,
                                          BilRowReference &rowOut,
                                          EnviBilMetadata &metadataOut,
                                          BandMeanStd &frameStatsOut,
                                          QString *errorMessage)
{
    rowOut.clear();
    frameStatsOut = BandMeanStd{};

    if (!parseEnviHdr(hdrPath, metadataOut, errorMessage))
        return false;

    const int samples = metadataOut.samples;
    const int bands = metadataOut.bands;
    const std::size_t rowPixels =
        static_cast<std::size_t>(samples) * static_cast<std::size_t>(bands);

    std::vector<double> rowSum(rowPixels, 0.0);
    std::vector<double> bandSum(static_cast<std::size_t>(bands), 0.0);
    std::vector<double> bandSumSq(static_cast<std::size_t>(bands), 0.0);
    std::uint64_t lineCount = 0;

    const bool ok = readEnviBilLines(
        metadataOut,
        [&](const std::uint16_t *linePixels) {
            for (std::size_t i = 0; i < rowPixels; ++i)
                rowSum[i] += static_cast<double>(linePixels[i]);

            accumulateImageBandStats(bands, samples, linePixels, bandSum, bandSumSq);
            ++lineCount;
            return true;
        },
        errorMessage);

    if (!ok || lineCount <= 0)
    {
        if (errorMessage != nullptr && errorMessage->isEmpty())
            *errorMessage = QStringLiteral("Reference cube has no lines: %1").arg(hdrPath);
        return false;
    }

    rowOut.resize(rowPixels);
    const double invLines = 1.0 / static_cast<double>(lineCount);
    for (std::size_t i = 0; i < rowPixels; ++i)
        rowOut[i] = static_cast<float>(rowSum[i] * invLines);

    const std::uint64_t pixelCount =
        lineCount * static_cast<std::uint64_t>(samples);
    if (!finalizeBandStats(bands,
                           bandSum,
                           bandSumSq,
                           pixelCount,
                           metadataOut.wavelengthsNm,
                           frameStatsOut,
                           errorMessage))
    {
        return false;
    }

    metadataOut.lines = 1;
    return true;
}

} // namespace hf::processing
