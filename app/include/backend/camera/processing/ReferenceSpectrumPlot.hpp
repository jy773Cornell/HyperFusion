// Mean ± 1σ spectrum plot export (intensity DN vs wavelength).
#pragma once

#include <QString>

#include <vector>

namespace hf::processing
{
bool saveReferenceMeanStdPlotPng(const std::vector<double> &wavelengthsNm,
                                 const std::vector<double> &meanDn,
                                 const std::vector<double> &stdDn,
                                 const QString &title,
                                 const QString &outputPath,
                                 double yAxisMax = 4096.0,
                                 QString *errorMessage = nullptr);

} // namespace hf::processing
