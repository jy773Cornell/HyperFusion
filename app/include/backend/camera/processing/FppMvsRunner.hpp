// One-shot subprocess runner for FPP MVS decode→fusion (backend/offline).
#pragma once

#include <QString>
#include <QStringList>

namespace hf::processing
{
struct FppMvsRunRequest
{
    /// Dataset/cluster root (parent of multiview/), or the multiview/ folder itself.
    QString inputPath;
    QString handEyeYaml;
    QString stereoYaml;
    QString mode = QStringLiteral("sweep");
    int timeoutMs = 3600000;
};

struct FppMvsRunResult
{
    bool success = false;
    bool skipped = false;
    QString reason;
    QString datafolder;
    QString processedDir;
    QString metadataJsonPath;
    QString primaryCloudPath;
    double elapsedS = 0.0;
    QString errorMessage;
    QStringList logLines;
};

QString resolveFppSidecarDirectory();
QString resolveFppPythonExecutable();

/// True if multiview folder has pose JSON with fpp_pattern / fpp_step_index.
bool multiviewHasFppBurst(const QString &multiviewOrDatafolder);

FppMvsRunResult runFppMvsPipeline(const FppMvsRunRequest &request);

} // namespace hf::processing
