// Multi-ROI reflectance spectrum plot export (mean ± 1σ vs wavelength).
#pragma once

#include <QString>

#include <vector>

namespace hf::processing
{
struct RoiSpectrumSeries
{
    QString label;
    std::vector<double> mean;
    std::vector<double> std;
};

bool saveRoiSpectrumMeanStdPlotPng(const std::vector<double> &wavelengthsNm,
                                 const std::vector<RoiSpectrumSeries> &series,
                                 const QString &title,
                                 const QString &yAxisLabel,
                                 const QString &outputPath,
                                 QString *errorMessage = nullptr);

} // namespace hf::processing
