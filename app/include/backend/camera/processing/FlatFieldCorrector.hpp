// Streaming flat-field correction: per sample line, (raw - dark) / max(white - dark, eps)
// using one averaged reference row (mean over reference frames, spatial x preserved).
#pragma once

#include "backend/camera/processing/EnviBilReader.hpp"
#include "backend/camera/processing/ReferenceBuilder.hpp"

#include <QString>

#include <cstdint>
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
/// Optional in-place mutator on each uint16 BIL sample line before FFC (e.g. SWIR ref BPR).
using SampleLineMutator = std::function<void(std::uint16_t *linePixels)>;

bool applyFlatFieldCorrection(const EnviBilMetadata &sampleMetadata,
                              const BilRowReference &darkRow,
                              const BilRowReference &whiteRow,
                              const FlatFieldParams &params,
                              const FlatFieldLineCallback &onCorrectedLine,
                              QString *errorMessage = nullptr,
                              const SampleLineMutator &preprocessSampleLine = {});

bool writeFlatFieldCorrectedEnvi(const QString &sampleHdrPath,
                                 const BilRowReference &darkRow,
                                 const BilRowReference &whiteRow,
                                 const QString &outputHdrPath,
                                 const QString &sensorTypeLabel,
                                 const QString &enviDescription,
                                 const FlatFieldParams &params,
                                 QString *errorMessage = nullptr,
                                 const SampleLineMutator &preprocessSampleLine = {});

} // namespace hf::processing
