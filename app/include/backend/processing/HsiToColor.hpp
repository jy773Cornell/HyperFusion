// Hyperspectral reflectance cube to sRGB (D-illuminant, trapezoidal integration).
#pragma once

#include "backend/processing/EnviBilReader.hpp"
#include "backend/processing/IlluminantTables.hpp"

#include <QString>

namespace hf::processing
{
bool writeReflectanceRgbPngFromLines(const EnviBilMetadata &metadata,
                                     const QString &floatRawPath,
                                     int illuminantD,
                                     double truncateNm,
                                     const QString &illuminantsJsonPath,
                                     const QString &outputPngPath,
                                     QString *errorMessage = nullptr);

} // namespace hf::processing
