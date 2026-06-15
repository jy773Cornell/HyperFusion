// ENVI BIL hyperspectral cube reader (uint16) for offline capture post-processing.
#pragma once

#include <QString>

#include <cstdint>
#include <functional>
#include <vector>

namespace hf::processing
{
struct EnviBilMetadata
{
    int samples = 0;
    int bands = 0;
    int lines = 0;
    int dataType = 12;
    QString interleave = QStringLiteral("bil");
    QString rawPath;
    std::vector<double> wavelengthsNm;
};

/// One ENVI BIL line: band 0 (all samples), band 1 (all samples), … — matches Lumo `FramePacket` layout.
inline std::size_t bilLinePixelIndex(const int sample, const int band, const int samplesPerLine)
{
    return static_cast<std::size_t>(band) * static_cast<std::size_t>(samplesPerLine)
           + static_cast<std::size_t>(sample);
}

bool parseEnviHdr(const QString &hdrPath, EnviBilMetadata &metadataOut, QString *errorMessage = nullptr);

/// Invokes @p onEachLine with one BIL line (`bands × samples` uint16 values, band-major).
bool readEnviBilLines(const EnviBilMetadata &metadata,
                    const std::function<bool(const std::uint16_t *linePixels)> &onEachLine,
                    QString *errorMessage = nullptr);

bool readEnviFloatBilLines(const EnviBilMetadata &metadata,
                           const QString &rawPathOverride,
                           const std::function<bool(const float *linePixels)> &onEachLine,
                           QString *errorMessage = nullptr);

} // namespace hf::processing
