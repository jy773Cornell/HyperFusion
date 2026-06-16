// ROI mean reflectance extraction from FFC ENVI cubes and GSAM2 mask outputs.
#pragma once

#include <QString>

namespace hf::processing
{
struct Gsam2RoiAnalysisResult
{
    bool success = false;
    QString csvPath;
    QString spectrumPlotPath;
    QString errorMessage;
};

Gsam2RoiAnalysisResult analyzeGsam2SegmentationRois(const QString &ffcHdrPath,
                                                     const QString &segmentationDirectory,
                                                     const QString &imageName,
                                                     const QString &manifestJsonPath,
                                                     QString *errorMessage = nullptr);

} // namespace hf::processing
