// Color-matching functions and D-illuminant spectra (from D_illuminants.mat / JSON).
#pragma once

#include <QString>

#include <vector>

namespace hf::processing
{
struct IlluminantSpectra
{
    std::vector<double> wavelengthNm;
    std::vector<double> illuminant;
    std::vector<double> xBar;
    std::vector<double> yBar;
    std::vector<double> zBar;
};

QString defaultIlluminantsJsonPath();

bool loadIlluminantSpectra(const QString &jsonPath,
                           int illuminantD,
                           const std::vector<double> &targetWavelengthsNm,
                           IlluminantSpectra &spectraOut,
                           QString *errorMessage = nullptr);

} // namespace hf::processing
