// Reflectance hyperspectral cube to display RGB: VNIR sRGB integration or SWIR false-color ranges.
#pragma once

#include "backend/HyperFusionConfig.hpp"
#include "backend/camera/processing/EnviBilReader.hpp"

#include <QString>

#include <functional>
#include <vector>

namespace hf::processing
{
enum class ReflectanceRgbExportMode
{
    SpectralToSrgb,
    SwirFalseColorRanges,
};

struct SwirFalseColorConfig
{
    hf::HardwareConfig::WavelengthRangeNm red;
    hf::HardwareConfig::WavelengthRangeNm green;
    hf::HardwareConfig::WavelengthRangeNm blue;
};

/// VNIR path: integrate reflectance (λ ≤ truncateNm) to sRGB via D-illuminant + CIE CMFs.
bool writeReflectanceRgbPngFromLines(const EnviBilMetadata &metadata,
                                     const QString &floatRawPath,
                                     int illuminantD,
                                     double truncateNm,
                                     const QString &illuminantsJsonPath,
                                     const QString &outputPngPath,
                                     QString *errorMessage = nullptr);

/// SWIR path: mean reflectance in three wavelength ranges → per-channel min/max stretch → PNG.
bool writeSwirFalseColorRgbPngFromLines(const EnviBilMetadata &metadata,
                                        const QString &floatRawPath,
                                        const SwirFalseColorConfig &falseColor,
                                        const QString &outputPngPath,
                                        QString *errorMessage = nullptr);

/// Dispatches to sRGB (FX10e / VNIR) or SWIR false-color export from a float32 FFC cube.
bool writeFfcCubeRgbPng(const EnviBilMetadata &ffcMetadata,
                        const QString &ffcFloatRawPath,
                        ReflectanceRgbExportMode mode,
                        const SwirFalseColorConfig &swirFalseColor,
                        int illuminantD,
                        double truncateNm,
                        const QString &illuminantsJsonPath,
                        const QString &outputPngPath,
                        QString *errorMessage = nullptr);

/// Build sRGB PNG from per-pixel reflectance spectra (FX wavelength axis).
bool writeReflectanceRgbPngFromProvider(
    const std::vector<double> &wavelengthsNm,
    int samples,
    int lines,
    const std::function<void(int line, int sample, float *reflectanceSpectrum)> &provideReflectance,
    int illuminantD,
    double truncateNm,
    const QString &illuminantsJsonPath,
    const QString &outputPngPath,
    QString *errorMessage = nullptr);

} // namespace hf::processing
