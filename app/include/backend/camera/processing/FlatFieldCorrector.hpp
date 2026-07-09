// Streaming flat-field correction: per sample line, (raw - dark) / max(white - dark, eps)
// using one averaged reference row (mean over reference frames, spatial x preserved).
#pragma once

#include "backend/camera/processing/EnviBilReader.hpp"
#include "backend/camera/processing/ReferenceBuilder.hpp"

#include <QString>
#include <functional>

namespace hf::processing
{
struct FlatFieldParams
{
    double epsilon = 1e-6;
    double clampMin = 0.0;
    double clampMax = 1.0;
};

using FlatFieldLineCallback = std::function<bool(const float *correctedLine, int lineIndex)>;

bool applyFlatFieldCorrection(const EnviBilMetadata &sampleMetadata,
                              const BilRowReference &darkRow,
                              const BilRowReference &whiteRow,
                              const FlatFieldParams &params,
                              const FlatFieldLineCallback &onCorrectedLine,
                              QString *errorMessage = nullptr);

bool writeFlatFieldCorrectedEnvi(const QString &sampleHdrPath,
                                 const BilRowReference &darkRow,
                                 const BilRowReference &whiteRow,
                                 const QString &outputHdrPath,
                                 const QString &sensorTypeLabel,
                                 const QString &enviDescription,
                                 const FlatFieldParams &params,
                                 QString *errorMessage = nullptr);

} // namespace hf::processing
