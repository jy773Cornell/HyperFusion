#include "backend/camera/processing/FlatFieldCorrector.hpp"

#include "backend/camera/processing/EnviBilWriter.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace hf::processing
{
namespace
{
bool validateRowReference(const EnviBilMetadata &metadata,
                          const BilRowReference &row,
                          QString *errorMessage)
{
    const std::size_t expected =
        static_cast<std::size_t>(metadata.samples) * static_cast<std::size_t>(metadata.bands);
    if (row.size() != expected)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("Reference row size mismatch (expected %1, got %2).")
                                 .arg(expected)
                                 .arg(row.size());
        }
        return false;
    }
    return true;
}
} // namespace

bool applyFlatFieldCorrection(const EnviBilMetadata &sampleMetadata,
                              const BilRowReference &darkRow,
                              const BilRowReference &whiteRow,
                              const FlatFieldParams &params,
                              const FlatFieldLineCallback &onCorrectedLine,
                              QString *errorMessage)
{
    if (!validateRowReference(sampleMetadata, darkRow, errorMessage)
        || !validateRowReference(sampleMetadata, whiteRow, errorMessage))
        return false;

    const int samples = sampleMetadata.samples;
    const int bands = sampleMetadata.bands;
    std::vector<float> corrected(static_cast<std::size_t>(samples * bands));

    int lineIndex = 0;
    const bool ok = readEnviBilLines(
        sampleMetadata,
        [&](const std::uint16_t *linePixels) {
            for (int sample = 0; sample < samples; ++sample)
            {
                for (int band = 0; band < bands; ++band)
                {
                    const std::size_t index = bilLinePixelIndex(sample, band, samples);
                    const double raw = static_cast<double>(linePixels[index]);
                    const double dark = static_cast<double>(darkRow[index]);
                    const double white = static_cast<double>(whiteRow[index]);
                    double denom = white - dark;
                    if (!std::isfinite(denom) || denom < params.epsilon)
                        denom = params.epsilon;
                    double value = (raw - dark) / denom;
                    value = std::clamp(value, params.clampMin, params.clampMax);
                    corrected[index] = static_cast<float>(value);
                }
            }

            if (!onCorrectedLine(corrected.data(), lineIndex))
                return false;
            ++lineIndex;
            return true;
        },
        errorMessage);

    return ok;
}

bool writeFlatFieldCorrectedEnvi(const QString &sampleHdrPath,
                                 const BilRowReference &darkRow,
                                 const BilRowReference &whiteRow,
                                 const QString &outputHdrPath,
                                 const QString &sensorTypeLabel,
                                 const QString &enviDescription,
                                 const FlatFieldParams &params,
                                 QString *errorMessage)
{
    EnviBilMetadata sampleMetadata;
    if (!parseEnviHdr(sampleHdrPath, sampleMetadata, errorMessage))
        return false;

    EnviFloatWriter writer;
    if (!beginEnviFloatWriter(writer,
                              outputHdrPath,
                              sampleMetadata,
                              sensorTypeLabel,
                              enviDescription,
                              errorMessage))
        return false;

    const bool ok = applyFlatFieldCorrection(
        sampleMetadata,
        darkRow,
        whiteRow,
        params,
        [&](const float *linePixels, int /*lineIndex*/) {
            return appendEnviFloatLine(writer, linePixels, errorMessage);
        },
        errorMessage);

    if (!ok)
        return false;

    return finalizeEnviFloatWriter(writer, errorMessage);
}

} // namespace hf::processing
