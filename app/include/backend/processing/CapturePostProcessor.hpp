// Offline post-processing for completed stage-scan capture sessions.
#pragma once

#include "backend/CaptureWriterTypes.hpp"

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
};

CapturePostProcessResult processCaptureSession(const CaptureWriterSessionSummary &summary,
                                               const CapturePostProcessOptions &options = {});

} // namespace hf::processing
