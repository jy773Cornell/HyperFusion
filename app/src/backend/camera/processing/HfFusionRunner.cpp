// One-shot subprocess runner for the hf_fusion Python pipeline (backend/offline).
#include "backend/camera/processing/HfFusionRunner.hpp"

#include "backend/HyperFusionConfig.hpp"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QImage>
#include <QProcess>
#include <QRegularExpression>

#include <algorithm>
#include <cmath>

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

QString resolveRepoHfFusionDirectory()
{
#ifdef HF_APP_SOURCE_DIR
    {
        const QString fromSource =
            QDir(QString::fromUtf8(HF_APP_SOURCE_DIR))
                .filePath(QStringLiteral("../resources/hf_fusion"));
        if (QFileInfo::exists(QDir(fromSource).filePath(QStringLiteral("fusion_cli.py"))))
            return QFileInfo(fromSource).absoluteFilePath();
    }
#endif

    if (QCoreApplication::instance() == nullptr)
        return {};

    // Walk up from app.exe (e.g. app/build/Release → repo root).
    QDir dir(QCoreApplication::applicationDirPath());
    for (int depth = 0; depth < 8; ++depth)
    {
        const QString resources = dir.filePath(QStringLiteral("resources/hf_fusion"));
        if (QFileInfo::exists(QDir(resources).filePath(QStringLiteral("fusion_cli.py"))))
            return QFileInfo(resources).absoluteFilePath();

        if (!dir.cdUp())
            break;
    }

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
    // Always use repo resources/hf_fusion (code + .venv live there).
    return resolveRepoHfFusionDirectory();
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

    // Compatibility check: per-chip fusion requires the same number of detected ROIs
    // on both cameras; additionally validate rough spatial ordering via mask upper-left.
    auto readSegmentationDetectionCount =
        [](const QString &preprocessedDir, QString *errorMessage) -> int {
            const QDir dir(preprocessedDir);
            const QString manifestPath = dir.filePath(QStringLiteral("segmentation/segmentation_results.json"));
            QFile file(manifestPath);
            if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
            {
                if (errorMessage != nullptr)
                    *errorMessage = QStringLiteral("Could not open segmentation manifest: %1").arg(manifestPath);
                return 0;
            }

            const QByteArray payload = file.readAll();
            QJsonParseError parseError;
            const QJsonDocument doc = QJsonDocument::fromJson(payload, &parseError);
            if (parseError.error != QJsonParseError::NoError || !doc.isObject())
            {
                if (errorMessage != nullptr)
                    *errorMessage = QStringLiteral("Invalid segmentation manifest JSON in %1").arg(manifestPath);
                return 0;
            }

            const QJsonObject obj = doc.object();
            const QJsonArray detections = obj.value(QStringLiteral("detections")).toArray();
            const int count = detections.size();
            if (count <= 0 && errorMessage != nullptr)
                *errorMessage = QStringLiteral("Segmentation manifest has 0 detections: %1").arg(manifestPath);
            return count;
        };

    if (detail != nullptr)
        detail->clear();

    QString fxSegErr;
    QString swSegErr;
    const int fxCount = readSegmentationDetectionCount(fxPre, &fxSegErr);
    if (fxCount <= 0)
    {
        if (detail != nullptr)
            *detail = fxSegErr.isEmpty() ? QStringLiteral("FX10e segmentation has no detections.") : fxSegErr;
        return false;
    }

    const int swCount = readSegmentationDetectionCount(swPre, &swSegErr);
    if (swCount <= 0)
    {
        if (detail != nullptr)
            *detail = swSegErr.isEmpty() ? QStringLiteral("SWIR3 segmentation has no detections.") : swSegErr;
        return false;
    }

    if (fxCount != swCount)
    {
        if (detail != nullptr)
            *detail = QStringLiteral("GSAM ROI count mismatch: FX10e=%1 SWIR3=%2 (fusion requires equal counts)")
                           .arg(fxCount)
                           .arg(swCount);
        return false;
    }

    // Rough positional alignment QA: compare mask centroids after sorting by mask upper-left
    // (row-major: top-to-bottom, then left-to-right).
    struct MaskQaPoint
    {
        int roi = 0;
        double upperLeftXmm = 0.0;
        double upperLeftYmm = 0.0;
        double centroidXmm = 0.0;
        double centroidYmm = 0.0;
    };

    const hf::HardwareConfig &cfg = hf::hardwareConfig();
    const double fxMmPerPixel = cfg.spatialMmPerPixel[0];
    const double swMmPerPixel = cfg.spatialMmPerPixel[1];

    auto computeMaskQaPoints = [](const QString &preprocessedDir,
                                   const double mmPerPixel,
                                   QString *errorMessage) -> QVector<MaskQaPoint> {
        const QDir dir(preprocessedDir);
        const QString manifestPath = dir.filePath(QStringLiteral("segmentation/segmentation_results.json"));
        QFile file(manifestPath);
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            if (errorMessage != nullptr)
                *errorMessage = QStringLiteral("Could not open segmentation manifest: %1").arg(manifestPath);
            return {};
        }

        const QByteArray payload = file.readAll();
        QJsonParseError parseError;
        const QJsonDocument doc = QJsonDocument::fromJson(payload, &parseError);
        if (parseError.error != QJsonParseError::NoError || !doc.isObject())
        {
            if (errorMessage != nullptr)
                *errorMessage = QStringLiteral("Invalid segmentation manifest JSON in %1").arg(manifestPath);
            return {};
        }

        const QJsonObject obj = doc.object();
        const QJsonArray detections = obj.value(QStringLiteral("detections")).toArray();
        if (detections.isEmpty())
        {
            if (errorMessage != nullptr)
                *errorMessage = QStringLiteral("Segmentation manifest has 0 detections: %1").arg(manifestPath);
            return {};
        }

        QVector<MaskQaPoint> points;
        points.reserve(detections.size());

        const QDir masksDir(dir.filePath(QStringLiteral("segmentation/masks")));
        for (const QJsonValue &v : detections)
        {
            const QJsonObject det = v.toObject();
            const int roi = det.value(QStringLiteral("roi")).toInt(-1);
            if (roi <= 0)
            {
                if (errorMessage != nullptr)
                    *errorMessage = QStringLiteral("Segmentation manifest has invalid roi entry in %1").arg(manifestPath);
                return {};
            }

            const QString maskPath = masksDir.filePath(
                QStringLiteral("mask_%1.png").arg(roi, 3, 10, QChar(u'0')));
            QImage maskImg(maskPath);
            if (maskImg.isNull())
            {
                if (errorMessage != nullptr)
                    *errorMessage = QStringLiteral("Could not load mask PNG for roi %1: %2")
                                        .arg(roi)
                                        .arg(maskPath);
                return {};
            }

            const QImage gray = maskImg.convertToFormat(QImage::Format_Grayscale8);
            const int w = gray.width();
            const int h = gray.height();
            if (w <= 0 || h <= 0)
            {
                if (errorMessage != nullptr)
                    *errorMessage = QStringLiteral("Mask image invalid for roi %1: %2")
                                        .arg(roi)
                                        .arg(maskPath);
                return {};
            }

            // Scan mask pixels to compute centroid + upper-left (min x/y) in pixel space.
            double sumX = 0.0;
            double sumY = 0.0;
            int nonZero = 0;
            int minX = w;
            int minY = h;

            for (int y = 0; y < h; ++y)
            {
                const uchar *row = gray.constScanLine(y);
                for (int x = 0; x < w; ++x)
                {
                    const uchar px = row[x];
                    if (px > 0)
                    {
                        ++nonZero;
                        sumX += static_cast<double>(x);
                        sumY += static_cast<double>(y);
                        if (x < minX)
                            minX = x;
                        if (y < minY)
                            minY = y;
                    }
                }
            }

            if (nonZero <= 0)
            {
                if (errorMessage != nullptr)
                    *errorMessage = QStringLiteral("Mask has no non-zero pixels for roi %1: %2")
                                        .arg(roi)
                                        .arg(maskPath);
                return {};
            }

            MaskQaPoint p;
            p.roi = roi;
            p.upperLeftXmm = static_cast<double>(minX) * mmPerPixel;
            p.upperLeftYmm = static_cast<double>(minY) * mmPerPixel;
            p.centroidXmm = (sumX / static_cast<double>(nonZero)) * mmPerPixel;
            p.centroidYmm = (sumY / static_cast<double>(nonZero)) * mmPerPixel;
            points.push_back(p);
        }

        if (points.size() != static_cast<int>(detections.size()))
        {
            // Some manifests might contain invalid roi entries; treat mismatch as a QA failure.
            if (errorMessage != nullptr)
                *errorMessage = QStringLiteral("Segmentation manifest roi entries mismatch in %1").arg(manifestPath);
            return {};
        }

        std::sort(points.begin(), points.end(), [](const MaskQaPoint &a, const MaskQaPoint &b) {
            if (a.upperLeftYmm != b.upperLeftYmm)
                return a.upperLeftYmm < b.upperLeftYmm;
            return a.upperLeftXmm < b.upperLeftXmm;
        });
        return points;
    };

    QString fxQaErr;
    QString swQaErr;
    const QVector<MaskQaPoint> fxPoints = computeMaskQaPoints(fxPre, fxMmPerPixel, &fxQaErr);
    if (fxPoints.isEmpty())
    {
        if (detail != nullptr)
            *detail = fxQaErr.isEmpty() ? QStringLiteral("FX10e mask QA failed.") : fxQaErr;
        return false;
    }

    const QVector<MaskQaPoint> swPoints = computeMaskQaPoints(swPre, swMmPerPixel, &swQaErr);
    if (swPoints.isEmpty())
    {
        if (detail != nullptr)
            *detail = swQaErr.isEmpty() ? QStringLiteral("SWIR3 mask QA failed.") : swQaErr;
        return false;
    }

    if (fxPoints.size() != swPoints.size())
    {
        if (detail != nullptr)
            *detail = QStringLiteral("GSAM mask QA point count mismatch: FX10e=%1 SWIR3=%2")
                           .arg(fxPoints.size())
                           .arg(swPoints.size());
        return false;
    }

    // Pair ROIs the same way as hf_fusion (left-to-right by centroid x, then y) and
    // reject only if a matched pair is implausibly far apart. GSAM roi ids are
    // detection-order labels (1..N), not spatial indices.
    auto sortByCentroidX = [](QVector<MaskQaPoint> &points) {
        std::sort(points.begin(), points.end(), [](const MaskQaPoint &a, const MaskQaPoint &b) {
            if (a.centroidXmm != b.centroidXmm)
                return a.centroidXmm < b.centroidXmm;
            return a.centroidYmm < b.centroidYmm;
        });
    };

    QVector<MaskQaPoint> fxSorted = fxPoints;
    QVector<MaskQaPoint> swSorted = swPoints;
    sortByCentroidX(fxSorted);
    sortByCentroidX(swSorted);

    constexpr double kMaxPairDistanceMm = 50.0;
    for (int i = 0; i < fxSorted.size(); ++i)
    {
        const double dx = fxSorted[i].centroidXmm - swSorted[i].centroidXmm;
        const double dy = fxSorted[i].centroidYmm - swSorted[i].centroidYmm;
        const double distance = std::sqrt(dx * dx + dy * dy);
        if (distance > kMaxPairDistanceMm)
        {
            if (detail != nullptr)
            {
                *detail = QStringLiteral(
                               "GSAM centroid pairing distance too large for pair %1: %2 mm "
                               "(fx roi #%3 vs sw roi #%4, max %5 mm)")
                               .arg(i + 1)
                               .arg(distance, 0, 'f', 1)
                               .arg(fxSorted[i].roi)
                               .arg(swSorted[i].roi)
                               .arg(kMaxPairDistanceMm, 0, 'f', 1);
            }
            return false;
        }
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
                           "Run: cd resources\\hf_fusion ; .\\setup_venv.ps1")
                .arg(fusionDir.isEmpty() ? QStringLiteral("resources/hf_fusion") : fusionDir);
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
                               "Re-run: cd resources\\hf_fusion ; .\\setup_venv.ps1"));
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
