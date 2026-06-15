#include "backend/processing/HsiToColor.hpp"

#include <QDir>
#include <QFileInfo>
#include <QImage>
#include <QSaveFile>

#include <algorithm>
#include <cmath>
#include <vector>

namespace hf::processing
{
namespace
{
constexpr float kSrgbGammaThreshold = 0.0031308f;
constexpr float kSrgbLinearScale = 12.92f;
constexpr float kSrgbGammaScale = 1.055f;
constexpr float kSrgbGammaOffset = 0.055f;
constexpr float kSrgbGammaExponent = 1.0f / 2.4f;

float applySrgbGamma(const float linear)
{
    if (linear <= kSrgbGammaThreshold)
        return kSrgbLinearScale * linear;
    return kSrgbGammaScale * std::pow(linear, kSrgbGammaExponent) - kSrgbGammaOffset;
}

std::vector<double> truncateWavelengths(const std::vector<double> &wavelengthsNm, const double truncateNm)
{
    std::vector<double> truncated;
    for (const double wavelength : wavelengthsNm)
    {
        if (wavelength > truncateNm)
            break;
        truncated.push_back(wavelength);
    }
    return truncated;
}

double trapezoidWeights(const std::vector<double> &x, std::vector<double> &weightsOut)
{
    weightsOut.assign(x.size(), 0.0);
    if (x.size() < 2)
        return 0.0;

    double integral = 0.0;
    for (std::size_t i = 0; i + 1 < x.size(); ++i)
    {
        const double dx = x[i + 1] - x[i];
        weightsOut[i] += 0.5 * dx;
        weightsOut[i + 1] += 0.5 * dx;
        integral += dx;
    }
    return integral;
}

void spectrumToXyz(const std::vector<double> &weights,
                   const IlluminantSpectra &spectra,
                   const float *reflectance,
                   float &xOut,
                   float &yOut,
                   float &zOut)
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    const int bandCount = static_cast<int>(weights.size());

    for (int band = 0; band < bandCount; ++band)
    {
        const double weight = weights[static_cast<std::size_t>(band)];
        const double value = static_cast<double>(reflectance[band]);
        x += value * spectra.xBar[static_cast<std::size_t>(band)] * spectra.illuminant[static_cast<std::size_t>(band)]
             * weight;
        y += value * spectra.yBar[static_cast<std::size_t>(band)] * spectra.illuminant[static_cast<std::size_t>(band)]
             * weight;
        z += value * spectra.zBar[static_cast<std::size_t>(band)] * spectra.illuminant[static_cast<std::size_t>(band)]
             * weight;
    }

    xOut = static_cast<float>(x);
    yOut = static_cast<float>(y);
    zOut = static_cast<float>(z);
}

void xyzToSrgb(const float x, const float y, const float z, float &rOut, float &gOut, float &bOut)
{
    const float rLin = 3.2404542f * x - 1.5371385f * y - 0.4985314f * z;
    const float gLin = -0.9692660f * x + 1.8760108f * y + 0.0415560f * z;
    const float bLin = 0.0556434f * x - 0.2040259f * y + 1.0572252f * z;

    rOut = std::clamp(applySrgbGamma(std::max(0.0f, rLin)), 0.0f, 1.0f);
    gOut = std::clamp(applySrgbGamma(std::max(0.0f, gLin)), 0.0f, 1.0f);
    bOut = std::clamp(applySrgbGamma(std::max(0.0f, bLin)), 0.0f, 1.0f);
}
} // namespace

bool writeReflectanceRgbPngFromLines(const EnviBilMetadata &metadata,
                                     const QString &floatRawPath,
                                     const int illuminantD,
                                     const double truncateNm,
                                     const QString &illuminantsJsonPath,
                                     const QString &outputPngPath,
                                     QString *errorMessage)
{
    const std::vector<double> truncatedWavelengths = truncateWavelengths(metadata.wavelengthsNm, truncateNm);
    const int bandCount = static_cast<int>(truncatedWavelengths.size());
    if (bandCount <= 1)
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("No spectral bands remain after truncation.");
        return false;
    }

    IlluminantSpectra spectra;
    if (!loadIlluminantSpectra(illuminantsJsonPath, illuminantD, truncatedWavelengths, spectra, errorMessage))
        return false;

    std::vector<double> integrationWeights;
    trapezoidWeights(truncatedWavelengths, integrationWeights);
    if (integrationWeights.empty())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Invalid wavelength axis for integration.");
        return false;
    }

    double yIntegral = 0.0;
    for (int band = 0; band < bandCount; ++band)
    {
        yIntegral += spectra.yBar[static_cast<std::size_t>(band)]
                     * spectra.illuminant[static_cast<std::size_t>(band)]
                     * integrationWeights[static_cast<std::size_t>(band)];
    }
    if (yIntegral <= 0.0)
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Illuminant normalization integral is zero.");
        return false;
    }

    const float normalization = static_cast<float>(1.0 / yIntegral);

    EnviBilMetadata floatMetadata = metadata;
    const int fullBands = metadata.bands;

    QImage image(metadata.samples, metadata.lines, QImage::Format_RGB888);
    if (image.isNull())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Could not allocate RGB image.");
        return false;
    }

    int lineIndex = 0;
    const bool ok = readEnviFloatBilLines(
        floatMetadata,
        floatRawPath,
        [&](const float *linePixels) {
            auto *scanLine = reinterpret_cast<std::uint8_t *>(image.scanLine(lineIndex));
            std::vector<float> spectrum(static_cast<std::size_t>(fullBands));
            for (int sample = 0; sample < metadata.samples; ++sample)
            {
                for (int band = 0; band < fullBands; ++band)
                {
                    spectrum[static_cast<std::size_t>(band)] =
                        linePixels[bilLinePixelIndex(sample, band, metadata.samples)];
                }

                float x = 0.0f;
                float y = 0.0f;
                float z = 0.0f;
                spectrumToXyz(integrationWeights, spectra, spectrum.data(), x, y, z);
                x *= normalization;
                y *= normalization;
                z *= normalization;

                float r = 0.0f;
                float g = 0.0f;
                float b = 0.0f;
                xyzToSrgb(x, y, z, r, g, b);

                const std::size_t base = static_cast<std::size_t>(sample) * 3;
                scanLine[base + 0] = static_cast<std::uint8_t>(std::lround(r * 255.0f));
                scanLine[base + 1] = static_cast<std::uint8_t>(std::lround(g * 255.0f));
                scanLine[base + 2] = static_cast<std::uint8_t>(std::lround(b * 255.0f));
            }
            ++lineIndex;
            return true;
        },
        errorMessage);

    if (!ok)
        return false;

    QDir().mkpath(QFileInfo(outputPngPath).absolutePath());
    if (!image.save(outputPngPath, "PNG"))
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Could not write RGB PNG: %1").arg(outputPngPath);
        return false;
    }

    return true;
}

} // namespace hf::processing
