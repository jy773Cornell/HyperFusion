// One-shot subprocess runner for the hf_fusion Python pipeline (backend/offline).
#include "backend/processing/HfFusionRunner.hpp"

#include "backend/HyperFusionConfig.hpp"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QProcess>
#include <QRegularExpression>

#include <algorithm>

namespace hf::processing
{
namespace
{
QString normalizedNativePath(const QString &path)
{
    return QDir::fromNativeSeparators(path.trimmed());
}

bool fileExists(const QString &path)
{
    return !path.isEmpty() && QFileInfo::exists(path);
}

QString preprocessedDirForCamera(const QString &sessionDirectory, const QString &mode, const QString &camera)
{
    return QDir(sessionDirectory).filePath(mode + QLatin1Char('/') + camera + QStringLiteral("/preprocessed"));
}

bool hasFfcHdr(const QString &preprocessedDir)
{
    const QDir dir(preprocessedDir);
    if (!dir.exists())
        return false;

    const QStringList matches =
        dir.entryList({QStringLiteral("*_ffc.hdr")}, QDir::Files, QDir::Name);
    return !matches.isEmpty();
}

bool hasSegmentationManifest(const QString &preprocessedDir)
{
    return fileExists(QDir(preprocessedDir).filePath(QStringLiteral("segmentation/segmentation_results.json")));
}

bool hasRgbPng(const QString &preprocessedDir)
{
    const QDir dir(preprocessedDir);
    if (!dir.exists())
        return false;

    const QStringList matches =
        dir.entryList({QStringLiteral("*_rgb.png")}, QDir::Files, QDir::Name);
    return !matches.isEmpty();
}

QString findHyperFusionCfgPath()
{
    const hf::HardwareConfig &config = hf::hardwareConfig();
    if (!config.filePath.isEmpty() && QFileInfo::exists(config.filePath))
        return QFileInfo(config.filePath).absoluteFilePath();

    for (const QString &candidate : hf::hyperFusionConfigSearchPaths())
    {
        if (QFileInfo::exists(candidate))
            return QFileInfo(candidate).absoluteFilePath();
    }

    return {};
}

QString resolveDevTreeHfFusionDirectory()
{
    if (QCoreApplication::instance() == nullptr)
        return {};

    QDir dir(QCoreApplication::applicationDirPath());
    for (int depth = 0; depth < 8; ++depth)
    {
        const QString direct = dir.filePath(QStringLiteral("hf_fusion"));
        if (QFileInfo::exists(QDir(direct).filePath(QStringLiteral("fusion_cli.py"))))
            return QFileInfo(direct).absoluteFilePath();

        const QString resources = dir.filePath(QStringLiteral("resources/hf_fusion"));
        if (QFileInfo::exists(QDir(resources).filePath(QStringLiteral("fusion_cli.py"))))
            return QFileInfo(resources).absoluteFilePath();

        const QString sibling = dir.filePath(QStringLiteral("../resources/hf_fusion"));
        if (QFileInfo::exists(QDir(sibling).filePath(QStringLiteral("fusion_cli.py"))))
            return QFileInfo(sibling).absoluteFilePath();

        if (!dir.cdUp())
            break;
    }

    return {};
}

QString resolveBundledHfFusionDirectory()
{
    if (QCoreApplication::instance() == nullptr)
        return {};

    const QString bundled =
        QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("hf_fusion"));
    if (QFileInfo::exists(QDir(bundled).filePath(QStringLiteral("fusion_cli.py"))))
        return QFileInfo(bundled).absoluteFilePath();

    return {};
}

bool parseFusionCliJson(const QByteArray &stdoutPayload, HfFusionRunResult *result)
{
    if (result == nullptr)
        return false;

    const QList<QByteArray> lines = stdoutPayload.split('\n');
    for (auto it = lines.crbegin(); it != lines.crend(); ++it)
    {
        const QByteArray trimmed = it->trimmed();
        if (trimmed.isEmpty() || trimmed.at(0) != '{')
            continue;

        QJsonParseError parseError;
        const QJsonDocument document = QJsonDocument::fromJson(trimmed, &parseError);
        if (parseError.error != QJsonParseError::NoError || !document.isObject())
            continue;

        const QJsonObject object = document.object();
        if (!object.value(QStringLiteral("ok")).toBool(false))
            continue;

        result->alignmentJsonPath = object.value(QStringLiteral("alignment_json")).toString();
        result->roiCount = object.value(QStringLiteral("roi_count")).toInt();
        result->pipelineComplete = object.value(QStringLiteral("pipeline_complete")).toBool();
        return !result->alignmentJsonPath.isEmpty();
    }

    return false;
}
} // namespace

QString resolveHfFusionDirectory()
{
    const QString bundled = resolveBundledHfFusionDirectory();
    if (!bundled.isEmpty())
        return bundled;

    return resolveDevTreeHfFusionDirectory();
}

QString resolveHfFusionPythonExecutable()
{
    const QString fusionDir = resolveHfFusionDirectory();
    if (fusionDir.isEmpty())
        return {};

    const QString venvPython =
        QDir(fusionDir).filePath(QStringLiteral(".venv/Scripts/python.exe"));
    if (QFileInfo::exists(venvPython))
        return QFileInfo(venvPython).absoluteFilePath();

    return {};
}

bool fusionPrerequisitesMet(const QString &sessionDirectory,
                            const QString &mode,
                            QString *detail)
{
    const QString session = normalizedNativePath(sessionDirectory);
    if (session.isEmpty() || !QFileInfo(session).isDir())
    {
        if (detail != nullptr)
            *detail = QStringLiteral("session directory is missing");
        return false;
    }

    const QString fxPre = preprocessedDirForCamera(session, mode, QStringLiteral("fx10e"));
    const QString swPre = preprocessedDirForCamera(session, mode, QStringLiteral("swir3"));

    if (!hasRgbPng(fxPre))
    {
        if (detail != nullptr)
            *detail = QStringLiteral("FX10e RGB PNG not found under %1").arg(fxPre);
        return false;
    }

    if (!hasRgbPng(swPre))
    {
        if (detail != nullptr)
            *detail = QStringLiteral("SWIR3 RGB PNG not found under %1").arg(swPre);
        return false;
    }

    if (!hasFfcHdr(fxPre))
    {
        if (detail != nullptr)
            *detail = QStringLiteral("FX10e FFC HDR not found under %1").arg(fxPre);
        return false;
    }

    if (!hasFfcHdr(swPre))
    {
        if (detail != nullptr)
            *detail = QStringLiteral("SWIR3 FFC HDR not found under %1").arg(swPre);
        return false;
    }

    if (!hasSegmentationManifest(fxPre))
    {
        if (detail != nullptr)
            *detail = QStringLiteral("FX10e segmentation_results.json not found under %1/segmentation")
                           .arg(fxPre);
        return false;
    }

    if (!hasSegmentationManifest(swPre))
    {
        if (detail != nullptr)
            *detail = QStringLiteral("SWIR3 segmentation_results.json not found under %1/segmentation")
                           .arg(swPre);
        return false;
    }

    if (detail != nullptr)
        detail->clear();
    return true;
}

bool sessionHasDualCameraForMode(const QString &sessionDirectory, const QString &mode)
{
    const QString session = normalizedNativePath(sessionDirectory);
    if (session.isEmpty() || mode.trimmed().isEmpty())
        return false;

    const QDir fxDir(QDir(session).filePath(mode + QLatin1Char('/') + QStringLiteral("fx10e")));
    const QDir swDir(QDir(session).filePath(mode + QLatin1Char('/') + QStringLiteral("swir3")));
    return fxDir.exists() && swDir.exists();
}

QStringList fusionIlluminationModesForSession(const QString &sessionDirectory)
{
    const QString session = normalizedNativePath(sessionDirectory);
    QStringList modes;
    if (session.isEmpty() || !QFileInfo(session).isDir())
        return modes;

    const QStringList entries =
        QDir(session).entryList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name);
    for (const QString &mode : entries)
    {
        if (sessionHasDualCameraForMode(session, mode))
            modes.push_back(mode);
    }
    return modes;
}

HfFusionSessionResult runSessionFusion(const QString &sessionDirectory, const QStringList &modes)
{
    HfFusionSessionResult sessionResult;
    const QString session = normalizedNativePath(sessionDirectory);
    if (session.isEmpty() || !QFileInfo(session).isDir())
    {
        sessionResult.success = false;
        sessionResult.logLines.push_back(
            QStringLiteral("Capture fusion: session directory not found."));
        return sessionResult;
    }

    QStringList targetModes = modes;
    if (targetModes.isEmpty())
        targetModes = fusionIlluminationModesForSession(session);

    if (targetModes.isEmpty())
    {
        sessionResult.success = false;
        sessionResult.logLines.push_back(
            QStringLiteral("Capture fusion: no illumination mode with fx10e and swir3 under %1")
                .arg(session));
        return sessionResult;
    }

    sessionResult.logLines.push_back(
        QStringLiteral("Capture fusion: running for mode(s): %1").arg(targetModes.join(QStringLiteral(", "))));

    bool allOk = true;
    for (const QString &mode : targetModes)
    {
        HfFusionRunRequest request;
        request.sessionDirectory = session;
        request.mode = mode;
        request.marginMm = hf::hardwareConfig().fusion.defaultMarginMm;
        request.processHsi = true;

        const HfFusionRunResult modeResult = runHfFusion(request);
        sessionResult.logLines.append(modeResult.logLines);
        if (modeResult.success)
            sessionResult.modesProcessed.push_back(mode);
        else
            allOk = false;
    }

    sessionResult.success = allOk;
    if (allOk)
    {
        sessionResult.logLines.push_back(
            QStringLiteral("Capture fusion: all mode(s) completed (%1).")
                .arg(sessionResult.modesProcessed.join(QStringLiteral(", "))));
    }
    else
    {
        sessionResult.logLines.push_back(QStringLiteral("Capture fusion: completed with errors."));
    }

    return sessionResult;
}

HfFusionRunResult runHfFusion(const HfFusionRunRequest &request)
{
    HfFusionRunResult result;

    const QString session = normalizedNativePath(request.sessionDirectory);
    const QString mode = request.mode.trimmed().isEmpty() ? QStringLiteral("reflectance") : request.mode.trimmed();

    if (session.isEmpty() || !QFileInfo(session).isDir())
    {
        result.errorMessage = QStringLiteral("Fusion session directory not found.");
        result.logLines.push_back(result.errorMessage);
        return result;
    }

    QString prerequisiteDetail;
    if (!fusionPrerequisitesMet(session, mode, &prerequisiteDetail))
    {
        result.errorMessage = prerequisiteDetail.isEmpty()
                                  ? QStringLiteral("Fusion prerequisites not met.")
                                  : prerequisiteDetail;
        result.logLines.push_back(QStringLiteral("Capture fusion: %1").arg(result.errorMessage));
        return result;
    }

    const QString fusionDir = resolveHfFusionDirectory();
    const QString cliScript = fusionDir.isEmpty()
                                  ? QString()
                                  : QDir(fusionDir).filePath(QStringLiteral("fusion_cli.py"));
    if (cliScript.isEmpty() || !QFileInfo::exists(cliScript))
    {
        result.errorMessage = QStringLiteral("fusion_cli.py not found (hf_fusion directory missing).");
        result.logLines.push_back(QStringLiteral("Capture fusion: %1").arg(result.errorMessage));
        return result;
    }

    const QString pythonExecutable = resolveHfFusionPythonExecutable();
    if (pythonExecutable.isEmpty())
    {
        result.errorMessage =
            QStringLiteral("hf_fusion Python venv not found at %1/.venv. "
                           "Run hf_fusion/setup_venv.ps1 beside app.exe.")
                .arg(fusionDir.isEmpty() ? QStringLiteral("hf_fusion") : fusionDir);
        result.logLines.push_back(QStringLiteral("Capture fusion: %1").arg(result.errorMessage));
        return result;
    }
    const QString cfgPath = request.cfgPath.trimmed().isEmpty() ? findHyperFusionCfgPath() : request.cfgPath.trimmed();

    QStringList arguments;
    arguments << QFileInfo(cliScript).absoluteFilePath();
    arguments << QStringLiteral("--session") << session;
    arguments << QStringLiteral("--mode") << mode;
    arguments << QStringLiteral("--margin-mm") << QString::number(request.marginMm, 'f', 3);
    if (!cfgPath.isEmpty())
        arguments << QStringLiteral("--cfg") << cfgPath;
    if (!request.processHsi)
        arguments << QStringLiteral("--no-hsi");

    result.logLines.push_back(QStringLiteral("Capture fusion: starting (%1)").arg(session));
    result.logLines.push_back(QStringLiteral("Capture fusion: python=%1").arg(pythonExecutable));
    result.logLines.push_back(QStringLiteral("Capture fusion: script=%1").arg(cliScript));

    QProcess process;
    process.setProgram(pythonExecutable);
    process.setArguments(arguments);
    process.setWorkingDirectory(fusionDir);
    process.setProcessChannelMode(QProcess::SeparateChannels);
    process.start();

    if (!process.waitForStarted(15000))
    {
        result.errorMessage =
            QStringLiteral("Failed to start fusion Python process: %1").arg(process.errorString());
        result.logLines.push_back(QStringLiteral("Capture fusion: %1").arg(result.errorMessage));
        return result;
    }

    const int timeoutMs = std::max(60000, hf::hardwareConfig().fusion.subprocessTimeoutMs);
    if (!process.waitForFinished(timeoutMs))
    {
        process.kill();
        process.waitForFinished(5000);
        result.errorMessage = QStringLiteral("Fusion subprocess timed out after %1 ms.").arg(timeoutMs);
        result.logLines.push_back(QStringLiteral("Capture fusion: %1").arg(result.errorMessage));
        return result;
    }

    const QByteArray stdoutPayload = process.readAllStandardOutput();
    const QByteArray stderrPayload = process.readAllStandardError();

    const auto appendLines = [&result](const QByteArray &payload, const QString &prefix) {
        const QString text = QString::fromUtf8(payload).trimmed();
        if (text.isEmpty())
            return;
        const QStringList lines = text.split(QRegularExpression(QStringLiteral("[\r\n]+")),
                                             Qt::SkipEmptyParts);
        for (const QString &line : lines)
            result.logLines.push_back(prefix + line);
    };

    appendLines(stdoutPayload, QStringLiteral("Capture fusion: "));
    appendLines(stderrPayload, QStringLiteral("Capture fusion: "));

    if (process.exitStatus() != QProcess::NormalExit || process.exitCode() != 0)
    {
        result.errorMessage = stderrPayload.trimmed().isEmpty()
                                  ? QStringLiteral("Fusion subprocess failed (exit %1).")
                                        .arg(process.exitCode())
                                  : QString::fromUtf8(stderrPayload.trimmed());
        if (stderrPayload.contains("No module named 'cv2'")
            || stderrPayload.contains("No module named \"cv2\""))
        {
            result.logLines.push_back(
                QStringLiteral("Capture fusion: OpenCV (cv2) not found. "
                               "Re-run hf_fusion/setup_venv.ps1 beside app.exe."));
        }
        result.logLines.push_back(QStringLiteral("Capture fusion: failed."));
        return result;
    }

    if (!parseFusionCliJson(stdoutPayload, &result))
    {
        result.errorMessage = QStringLiteral("Fusion subprocess succeeded but returned no JSON result.");
        result.logLines.push_back(QStringLiteral("Capture fusion: %1").arg(result.errorMessage));
        return result;
    }

    result.success = true;
    result.logLines.push_back(
        QStringLiteral("Capture fusion: completed (%1 ROI(s), alignment=%2)")
            .arg(result.roiCount)
            .arg(QFileInfo(result.alignmentJsonPath).fileName()));
    return result;
}

} // namespace hf::processing
