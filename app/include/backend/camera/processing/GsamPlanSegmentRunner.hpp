// One-shot subprocess runner for plan-driven GSAM (backend/offline).
#pragma once

#include <QString>
#include <QStringList>

namespace hf::processing
{
struct GsamPlanSegmentRunRequest
{
    QString sessionDirectory;
    QString planPath;
    QString stream;
    QString stem;
    QString gsamServerUrl = QStringLiteral("http://127.0.0.1:8765");
};

struct GsamPlanSegmentRunResult
{
    bool success = false;
    int detectionCount = 0;
    bool twoStage = false;
    QString prompt;
    QString segmentationDir;
    QString errorMessage;
    QStringList logLines;
};

GsamPlanSegmentRunResult runGsamPlanSegment(const GsamPlanSegmentRunRequest &request);

/// Write {session}/preview QA collages from existing preprocessed outputs.
/// Always creates preview/ when the session directory exists (single- or multi-mode).
/// Includes rgb_all (collage of all streams) and reference_intensity when those sources exist;
/// GSAM adds mask_overlaps and roi_spectra_plots when those outputs exist.
/// Per-stream ROI spectra stay in preprocessed/segmentation/. Does not fail the session.
QStringList writeSessionPreviewSheets(const QString &sessionDirectory, QStringList *logLines = nullptr);
} // namespace hf::processing
