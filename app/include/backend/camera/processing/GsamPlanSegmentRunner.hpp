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

/// Write {session}/mask_overlaps.png and {session}/roi_spectra_plots.png from existing
/// GSAM outputs. Does not fail the session; returns the mask sheet path when present.
QString writeSessionMaskOverlapsSheet(const QString &sessionDirectory, QStringList *logLines = nullptr);
} // namespace hf::processing
