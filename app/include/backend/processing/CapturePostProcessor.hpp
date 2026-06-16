// Offline post-processing for completed stage-scan capture sessions.
#pragma once

#include "backend/CaptureWriterTypes.hpp"

#include <QString>
#include <QStringList>

namespace hf::processing
{
struct CapturePostProcessResult
{
    bool success = false;
    QStringList logLines;
};

struct CapturePostProcessOptions
{
    bool saveFfcImage = true;
    bool runGsamSegmentation = false;
    QString gsamPrompt;
    int gsamSampleCount = 5;
    QString gsamServerUrl = QStringLiteral("http://127.0.0.1:8765");
};

CapturePostProcessResult processCaptureSession(const CaptureWriterSessionSummary &summary,
                                               const CapturePostProcessOptions &options = {});

} // namespace hf::processing
