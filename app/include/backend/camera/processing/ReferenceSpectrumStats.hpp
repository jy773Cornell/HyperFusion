// Per-band statistics for reference spectra plots and validation.
#pragma once

#include "backend/camera/processing/EnviBilReader.hpp"
#include "backend/camera/processing/ReferenceBuilder.hpp"

#include <QString>

#include <vector>

namespace hf::processing
{
struct BandMeanStd
{
    std::vector<double> meanDn;
    std::vector<double> stdDn;
    std::vector<double> wavelengthsNm;
};

/// Per-band mean/std over all pixels in the cube (all lines × spatial samples).
bool computeBandMeanStd(const EnviBilMetadata &metadata, BandMeanStd &statsOut, QString *errorMessage = nullptr);

/// Per-band mean/std of spatially averaged frame spectra (mean over samples per line,
/// then mean/std across lines). Alternate to full-image stats.
bool computeBandMeanStdAcrossFrames(const EnviBilMetadata &metadata,
                                    BandMeanStd &statsOut,
                                    QString *errorMessage = nullptr);

bool computeBandMeanStdFromHdr(const QString &hdrPath, BandMeanStd &statsOut, QString *errorMessage = nullptr);

bool computeBandMeanStdAcrossFramesFromHdr(const QString &hdrPath,
                                           BandMeanStd &statsOut,
                                           QString *errorMessage = nullptr);

/// Per-band mean/std over spatial samples on a single row reference (samples x bands).
bool computeBandMeanStdFromRowReference(const BilRowReference &row,
                                        const EnviBilMetadata &metadata,
                                        BandMeanStd &statsOut,
                                        QString *errorMessage = nullptr);

} // namespace hf::processing
