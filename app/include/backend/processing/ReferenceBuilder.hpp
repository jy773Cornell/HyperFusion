// Builds one BIL row reference (mean over scan lines) from ENVI cubes.
#pragma once

#include "backend/processing/EnviBilReader.hpp"

#include <QString>
#include <vector>

namespace hf::processing
{
struct BandMeanStd;

/// One spatial line: `bands × samples` values in ENVI BIL order (band-major within the line).
/// Built by averaging reference cube lines (frames) per spatial column; x dimension retained.
using BilRowReference = std::vector<float>;

bool buildRowMeanReference(const QString &hdrPath,
                           BilRowReference &rowOut,
                           EnviBilMetadata &metadataOut,
                           QString *errorMessage = nullptr);

/// One pass over the reference cube: row mean (for FFC, keeps spatial x) and per-band
/// mean/std over all pixels (lines × samples) for reference plots.
bool buildRowMeanReferenceAndFrameStats(const QString &hdrPath,
                                        BilRowReference &rowOut,
                                        EnviBilMetadata &metadataOut,
                                        BandMeanStd &frameStatsOut,
                                        QString *errorMessage = nullptr);

} // namespace hf::processing
