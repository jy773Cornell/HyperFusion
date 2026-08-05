// One-shot subprocess runner for the hf_fusion Python pipeline (backend/offline).
#pragma once

#include <QString>
#include <QStringList>

namespace hf::processing
{
struct HfFusionRunRequest
{
    QString sessionDirectory;
    QString mode = QStringLiteral("reflectance");
    QString cfgPath;
    double marginMm = 5.0;
    bool processHsi = true;
};

struct HfFusionRunResult
{
    bool success = false;
    QString alignmentJsonPath;
    int roiCount = 0;
    bool pipelineComplete = false;
    QString errorMessage;
    QStringList logLines;
};

struct HfFusionSessionResult
{
    bool success = false;
    QStringList modesProcessed;
    QStringList logLines;
};

/// Illumination folders under session that contain both fx10e and swir3 (e.g. reflectance, transmittance).
QStringList fusionIlluminationModesForSession(const QString &sessionDirectory);

/// Both camera trees exist under {session}/{mode}/.
bool sessionHasDualCameraForMode(const QString &sessionDirectory, const QString &mode);

/// Run fusion for each listed mode (empty = all modes found under the session).
HfFusionSessionResult runSessionFusion(const QString &sessionDirectory,
                                       const QStringList &modes = {});
QString resolveHfFusionDirectory();

/// resources/hf_fusion/.venv/Scripts/python.exe (run setup_venv.ps1 once there).
QString resolveHfFusionPythonExecutable();

/// Both cameras have FFC + GSAM segmentation under {session}/{mode}/.
bool fusionPrerequisitesMet(const QString &sessionDirectory,
                            const QString &mode,
                            QString *detail = nullptr);

HfFusionRunResult runHfFusion(const HfFusionRunRequest &request);

} // namespace hf::processing
