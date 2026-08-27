// One-shot subprocess runner for plan-driven GSAM (backend/offline).
#include "backend/camera/processing/GsamPlanSegmentRunner.hpp"

#include "backend/HyperFusionConfig.hpp"
#include "backend/camera/processing/HfFusionRunner.hpp"

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

bool parsePlanSegmentJson(const QByteArray &stdoutPayload, GsamPlanSegmentRunResult *result)
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

        result->detectionCount = object.value(QStringLiteral("detection_count")).toInt();
        result->twoStage = object.value(QStringLiteral("two_stage")).toBool(false);
        result->prompt = object.value(QStringLiteral("prompt")).toString();
        result->segmentationDir = object.value(QStringLiteral("segmentation_dir")).toString();
        return true;
    }

    return false;
}
} // namespace

GsamPlanSegmentRunResult runGsamPlanSegment(const GsamPlanSegmentRunRequest &request)
{
    GsamPlanSegmentRunResult result;

    const QString session = normalizedNativePath(request.sessionDirectory);
    const QString planPath = normalizedNativePath(request.planPath);
    const QString stream = request.stream.trimmed().toLower();

    if (session.isEmpty() || !QFileInfo(session).isDir())
    {
        result.errorMessage = QStringLiteral("GSAM plan-segment session directory not found.");
        result.logLines.push_back(result.errorMessage);
        return result;
    }
    if (planPath.isEmpty() || !QFileInfo::exists(planPath))
    {
        result.errorMessage = QStringLiteral("GSAM plan not found: %1").arg(request.planPath);
        result.logLines.push_back(result.errorMessage);
        return result;
    }
    if (stream.isEmpty() || !stream.contains(QLatin1Char('/')))
    {
        result.errorMessage = QStringLiteral("GSAM plan-segment stream must be mode/camera.");
        result.logLines.push_back(result.errorMessage);
        return result;
    }

    const QString fusionDir = resolveHfFusionDirectory();
    const QString cliScript = fusionDir.isEmpty()
                                  ? QString()
                                  : QDir(fusionDir).filePath(QStringLiteral("gsam_plan_segment_cli.py"));
    if (cliScript.isEmpty() || !QFileInfo::exists(cliScript))
    {
        result.errorMessage = QStringLiteral("gsam_plan_segment_cli.py not found (hf_fusion directory missing).");
        result.logLines.push_back(QStringLiteral("GSAM plan-segment: %1").arg(result.errorMessage));
        return result;
    }

    const QString pythonExecutable = resolveHfFusionPythonExecutable();
    if (pythonExecutable.isEmpty())
    {
        result.errorMessage =
            QStringLiteral("hf_fusion Python venv not found at %1/.venv. "
                           "Run: cd resources\\hf_fusion ; .\\setup_venv.ps1")
                .arg(fusionDir.isEmpty() ? QStringLiteral("resources/hf_fusion") : fusionDir);
        result.logLines.push_back(QStringLiteral("GSAM plan-segment: %1").arg(result.errorMessage));
        return result;
    }

    QStringList arguments;
    arguments << QFileInfo(cliScript).absoluteFilePath();
    arguments << QStringLiteral("--session") << session;
    arguments << QStringLiteral("--plan") << planPath;
    arguments << QStringLiteral("--stream") << stream;
    if (!request.stem.trimmed().isEmpty())
        arguments << QStringLiteral("--stem") << request.stem.trimmed();
    if (!request.gsamServerUrl.trimmed().isEmpty())
        arguments << QStringLiteral("--gsam-url") << request.gsamServerUrl.trimmed();

    result.logLines.push_back(QStringLiteral("GSAM plan-segment: starting (%1 %2)").arg(session, stream));
    result.logLines.push_back(QStringLiteral("GSAM plan-segment: python=%1").arg(pythonExecutable));
    result.logLines.push_back(QStringLiteral("GSAM plan-segment: script=%1").arg(cliScript));

    QProcess process;
    process.setProgram(pythonExecutable);
    process.setArguments(arguments);
    process.setWorkingDirectory(fusionDir);
    process.setProcessChannelMode(QProcess::SeparateChannels);
    process.start();

    if (!process.waitForStarted(15000))
    {
        result.errorMessage =
            QStringLiteral("Failed to start GSAM plan-segment Python process: %1").arg(process.errorString());
        result.logLines.push_back(QStringLiteral("GSAM plan-segment: %1").arg(result.errorMessage));
        return result;
    }

    const int timeoutMs = std::max(60000, hf::hardwareConfig().fusion.subprocessTimeoutMs);
    if (!process.waitForFinished(timeoutMs))
    {
        process.kill();
        process.waitForFinished(5000);
        result.errorMessage =
            QStringLiteral("GSAM plan-segment subprocess timed out after %1 ms.").arg(timeoutMs);
        result.logLines.push_back(QStringLiteral("GSAM plan-segment: %1").arg(result.errorMessage));
        return result;
    }

    const QByteArray stdoutPayload = process.readAllStandardOutput();
    const QByteArray stderrPayload = process.readAllStandardError();

    const auto appendLines = [&result](const QByteArray &payload, const QString &prefix) {
        const QString text = QString::fromUtf8(payload).trimmed();
        if (text.isEmpty())
            return;
        const QStringList lines =
            text.split(QRegularExpression(QStringLiteral("[\r\n]+")), Qt::SkipEmptyParts);
        for (const QString &line : lines)
            result.logLines.push_back(prefix + line);
    };

    appendLines(stdoutPayload, QStringLiteral("GSAM plan-segment: "));
    appendLines(stderrPayload, QStringLiteral("GSAM plan-segment: "));

    if (process.exitStatus() != QProcess::NormalExit || process.exitCode() != 0)
    {
        result.errorMessage = stderrPayload.trimmed().isEmpty()
                                  ? QStringLiteral("GSAM plan-segment subprocess failed (exit %1).")
                                        .arg(process.exitCode())
                                  : QString::fromUtf8(stderrPayload.trimmed());
        result.logLines.push_back(QStringLiteral("GSAM plan-segment: failed."));
        return result;
    }

    if (!parsePlanSegmentJson(stdoutPayload, &result))
    {
        result.errorMessage = QStringLiteral("GSAM plan-segment succeeded but returned no JSON result.");
        result.logLines.push_back(QStringLiteral("GSAM plan-segment: %1").arg(result.errorMessage));
        return result;
    }

    result.success = true;
    result.logLines.push_back(
        QStringLiteral("GSAM plan-segment: completed (%1 ROI(s)%2)")
            .arg(result.detectionCount)
            .arg(result.twoStage ? QStringLiteral(", two-stage") : QString()));
    return result;
}

QStringList writeSessionPreviewSheets(const QString &sessionDirectory, QStringList *logLines)
{
    const auto appendLog = [logLines](const QString &line) {
        if (logLines != nullptr)
            logLines->push_back(line);
    };

    const QString session = normalizedNativePath(sessionDirectory);
    if (session.isEmpty() || !QFileInfo(session).isDir())
        return {};

    const QString fusionDir = resolveHfFusionDirectory();
    const QString cliScript = fusionDir.isEmpty()
                                  ? QString()
                                  : QDir(fusionDir).filePath(QStringLiteral("gsam_plan_segment_cli.py"));
    const QString pythonExecutable = resolveHfFusionPythonExecutable();
    if (cliScript.isEmpty() || !QFileInfo::exists(cliScript) || pythonExecutable.isEmpty())
    {
        appendLog(QStringLiteral("Session preview: hf_fusion Python CLI not found."));
        return {};
    }

    QProcess process;
    process.setProgram(pythonExecutable);
    process.setArguments({QFileInfo(cliScript).absoluteFilePath(), QStringLiteral("--session"), session,
                          QStringLiteral("--write-sheet")});
    process.setWorkingDirectory(fusionDir);
    process.setProcessChannelMode(QProcess::SeparateChannels);
    process.start();
    if (!process.waitForStarted(15000) || !process.waitForFinished(60000))
    {
        process.kill();
        process.waitForFinished(5000);
        appendLog(QStringLiteral("Session preview: subprocess failed to finish."));
        return {};
    }

    const QByteArray stdoutPayload = process.readAllStandardOutput();
    if (process.exitStatus() != QProcess::NormalExit || process.exitCode() != 0)
    {
        const QString err = QString::fromUtf8(process.readAllStandardError().trimmed());
        appendLog(err.isEmpty() ? QStringLiteral("Session preview: write failed.") : err);
        return {};
    }

    QString rgbPath;
    QString refPath;
    QString maskSheetPath;
    QString spectraSheetPath;
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
        rgbPath = object.value(QStringLiteral("rgb_all")).toString();
        refPath = object.value(QStringLiteral("reference_intensity")).toString();
        maskSheetPath = object.value(QStringLiteral("mask_overlaps")).toString();
        spectraSheetPath = object.value(QStringLiteral("roi_spectra_plots")).toString();
        break;
    }

    const QString previewDir = QDir(session).filePath(QStringLiteral("preview"));
    const auto fallbackIfMissing = [&previewDir](QString path, const QString &fileName) {
        if (!path.isEmpty())
            return path;
        return QDir(previewDir).filePath(fileName);
    };
    rgbPath = fallbackIfMissing(rgbPath, QStringLiteral("rgb_all.png"));
    refPath = fallbackIfMissing(refPath, QStringLiteral("reference_intensity.png"));
    maskSheetPath = fallbackIfMissing(maskSheetPath, QStringLiteral("mask_overlaps.png"));
    spectraSheetPath = fallbackIfMissing(spectraSheetPath, QStringLiteral("roi_spectra_plots.png"));

    QStringList written;
    QStringList names;
    const QStringList candidates{rgbPath, refPath, maskSheetPath, spectraSheetPath};
    for (const QString &path : candidates)
    {
        if (path.isEmpty() || !QFileInfo::exists(path) || written.contains(path))
            continue;
        written.push_back(path);
        names.push_back(QFileInfo(path).fileName());
    }
    if (!names.isEmpty())
        appendLog(QStringLiteral("Session preview: wrote preview/%1").arg(names.join(QStringLiteral(", "))));
    return written;
}

} // namespace hf::processing
