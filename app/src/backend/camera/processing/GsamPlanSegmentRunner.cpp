// One-shot subprocess runner for plan-driven GSAM (backend/offline).
#include "backend/camera/processing/GsamPlanSegmentRunner.hpp"

#include "backend/HyperFusionConfig.hpp"
#include "backend/camera/processing/HfFusionRunner.hpp"

#include <QColor>
#include <QDir>
#include <QFileInfo>
#include <QFont>
#include <QImage>
#include <QJsonDocument>
#include <QJsonObject>
#include <QPainter>
#include <QProcess>
#include <QRegularExpression>

#include <algorithm>
#include <utility>
#include <vector>

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

struct PreviewPanel
{
    QString label;
    QString path;
};

int preferredStreamRank(const QString &mode, const QString &camera)
{
    const QString key = mode.toLower() + QLatin1Char('/') + camera.toLower();
    if (key == QLatin1String("reflectance/fx10e"))
        return 0;
    if (key == QLatin1String("reflectance/swir3"))
        return 1;
    if (key == QLatin1String("transmittance/fx10e"))
        return 2;
    if (key == QLatin1String("transmittance/swir3"))
        return 3;
    return 100;
}

void collectPreprocessedPngs(const QString &session,
                             const QStringList &nameFilters,
                             const bool firstPerStreamOnly,
                             std::vector<PreviewPanel> &out)
{
    QDir root(session);
    const QStringList topEntries =
        root.entryList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name);

    struct Candidate
    {
        int rank = 100;
        QString mode;
        QString camera;
        QString path;
    };
    std::vector<Candidate> candidates;

    const auto considerDir = [&](const QString &mode, const QString &camera, const QDir &pre) {
        if (!pre.exists())
            return;
        const QStringList names = pre.entryList(nameFilters, QDir::Files, QDir::Name);
        for (const QString &name : names)
        {
            Candidate item;
            item.rank = preferredStreamRank(mode, camera);
            item.mode = mode;
            item.camera = camera;
            item.path = pre.filePath(name);
            candidates.push_back(std::move(item));
            if (firstPerStreamOnly)
                break;
        }
    };

    for (const QString &mode : topEntries)
    {
        if (mode.compare(QLatin1String("preview"), Qt::CaseInsensitive) == 0)
            continue;

        QDir modeDir(root.filePath(mode));
        if (mode.compare(QLatin1String("preprocessed"), Qt::CaseInsensitive) == 0)
        {
            considerDir(QStringLiteral("session"), QStringLiteral("capture"), modeDir);
            continue;
        }

        const QStringList cameras =
            modeDir.entryList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name);
        for (const QString &camera : cameras)
            considerDir(mode, camera, QDir(modeDir.filePath(camera + QStringLiteral("/preprocessed"))));
    }

    std::sort(candidates.begin(), candidates.end(), [](const Candidate &a, const Candidate &b) {
        if (a.rank != b.rank)
            return a.rank < b.rank;
        const int modeCmp = a.mode.compare(b.mode, Qt::CaseInsensitive);
        if (modeCmp != 0)
            return modeCmp < 0;
        const int camCmp = a.camera.compare(b.camera, Qt::CaseInsensitive);
        if (camCmp != 0)
            return camCmp < 0;
        return a.path < b.path;
    });

    for (const Candidate &item : candidates)
    {
        PreviewPanel panel;
        panel.label = item.mode + QStringLiteral(" / ") + item.camera;
        if (!firstPerStreamOnly)
        {
            const QString fileName = QFileInfo(item.path).fileName();
            if (fileName.startsWith(QLatin1String("DARKREF"), Qt::CaseInsensitive))
                panel.label += QStringLiteral("  black");
            else if (fileName.startsWith(QLatin1String("WHITEREF"), Qt::CaseInsensitive))
                panel.label += QStringLiteral("  white");
        }
        out.push_back(std::move(panel));
    }
}

QString writeLabeledPngSheet(const QString &previewDir,
                             const std::vector<PreviewPanel> &panels,
                             const QString &fileName,
                             const QString &title)
{
    if (panels.empty())
        return {};

    std::vector<std::pair<QString, QImage>> images;
    int cellW = 0;
    int cellH = 0;
    for (const PreviewPanel &panel : panels)
    {
        QImage image(panel.path);
        if (image.isNull())
            continue;
        image = image.convertToFormat(QImage::Format_RGB32);
        cellW = std::max(cellW, image.width());
        cellH = std::max(cellH, image.height());
        images.push_back({panel.label, std::move(image)});
    }
    if (images.empty() || cellW < 1 || cellH < 1)
        return {};

    const int count = static_cast<int>(images.size());
    const int cols = count == 1 ? 1 : 2;
    const int rows = (count + cols - 1) / cols;
    constexpr int kPad = 12;
    constexpr int kLabelH = 36;
    constexpr int kHeaderH = 48;
    const int canvasW = kPad + cols * (cellW + kPad);
    const int canvasH = kHeaderH + kPad + rows * (kLabelH + cellH + kPad);

    QImage canvas(canvasW, canvasH, QImage::Format_RGB32);
    canvas.fill(QColor(18, 18, 18));
    QPainter painter(&canvas);
    painter.setRenderHint(QPainter::SmoothPixmapTransform, true);
    painter.setPen(QColor(255, 255, 255));
    QFont headerFont = painter.font();
    headerFont.setPointSize(16);
    headerFont.setBold(true);
    painter.setFont(headerFont);
    painter.drawText(QRect(kPad, 8, canvasW - 2 * kPad, 32),
                     Qt::AlignLeft | Qt::AlignVCenter,
                     title);

    QFont labelFont = painter.font();
    labelFont.setPointSize(12);
    labelFont.setBold(false);
    painter.setFont(labelFont);
    painter.setPen(QColor(230, 230, 230));

    for (int i = 0; i < count; ++i)
    {
        const int row = i / cols;
        const int col = i % cols;
        const int x0 = kPad + col * (cellW + kPad);
        const int y0 = kHeaderH + kPad + row * (kLabelH + cellH + kPad);
        const QImage &im = images[static_cast<std::size_t>(i)].second;
        const double scale =
            std::min(static_cast<double>(cellW) / std::max(1, im.width()),
                     static_cast<double>(cellH) / std::max(1, im.height()));
        const int nw = std::max(1, static_cast<int>(im.width() * scale));
        const int nh = std::max(1, static_cast<int>(im.height() * scale));
        const int ox = x0 + (cellW - nw) / 2;
        const int oy = y0 + kLabelH + (cellH - nh) / 2;
        painter.drawText(QRect(x0, y0, cellW, kLabelH),
                         Qt::AlignLeft | Qt::AlignVCenter,
                         images[static_cast<std::size_t>(i)].first);
        painter.drawImage(QRect(ox, oy, nw, nh), im);
    }
    painter.end();

    const QString outPath = QDir(previewDir).filePath(fileName);
    if (!canvas.save(outPath, "PNG"))
        return {};
    return outPath;
}

QStringList writeNativePreviewFallback(const QString &session, const QString &previewDir)
{
    QStringList written;
    std::vector<PreviewPanel> rgbPanels;
    collectPreprocessedPngs(session, {QStringLiteral("*_rgb.png")}, true, rgbPanels);
    const QString rgbPath =
        writeLabeledPngSheet(previewDir, rgbPanels, QStringLiteral("rgb_all.png"),
                             QFileInfo(session).fileName() + QStringLiteral("  RGB previews"));
    if (!rgbPath.isEmpty())
        written.push_back(rgbPath);

    std::vector<PreviewPanel> refPanels;
    collectPreprocessedPngs(session,
                            {QStringLiteral("DARKREF_*_ref_plot.png"),
                             QStringLiteral("WHITEREF_*_ref_plot.png")},
                            false, refPanels);
    const QString refPath =
        writeLabeledPngSheet(previewDir, refPanels, QStringLiteral("reference_intensity.png"),
                             QFileInfo(session).fileName()
                                 + QStringLiteral("  black / white reference intensity"));
    if (!refPath.isEmpty())
        written.push_back(refPath);
    return written;
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

    const QString previewDir = QDir(session).filePath(QStringLiteral("preview"));
    if (!QDir().mkpath(previewDir))
    {
        appendLog(QStringLiteral("Session preview: could not create preview/."));
        return {};
    }

    const QString fusionDir = resolveHfFusionDirectory();
    const QString cliScript = fusionDir.isEmpty()
                                  ? QString()
                                  : QDir(fusionDir).filePath(QStringLiteral("gsam_plan_segment_cli.py"));
    const QString pythonExecutable = resolveHfFusionPythonExecutable();
    if (cliScript.isEmpty() || !QFileInfo::exists(cliScript) || pythonExecutable.isEmpty())
    {
        appendLog(QStringLiteral("Session preview: hf_fusion Python CLI not found — writing native preview sheets."));
        const QStringList native = writeNativePreviewFallback(session, previewDir);
        if (native.isEmpty())
            appendLog(QStringLiteral("Session preview: created preview/."));
        return native;
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
        appendLog(QStringLiteral("Session preview: subprocess failed to finish — writing native preview sheets."));
        const QStringList native = writeNativePreviewFallback(session, previewDir);
        if (native.isEmpty())
            appendLog(QStringLiteral("Session preview: created preview/."));
        return native;
    }

    const QByteArray stdoutPayload = process.readAllStandardOutput();
    if (process.exitStatus() != QProcess::NormalExit || process.exitCode() != 0)
    {
        const QString err = QString::fromUtf8(process.readAllStandardError().trimmed());
        appendLog(err.isEmpty() ? QStringLiteral("Session preview: write failed — writing native preview sheets.")
                               : err);
        const QStringList native = writeNativePreviewFallback(session, previewDir);
        if (native.isEmpty())
            appendLog(QStringLiteral("Session preview: created preview/."));
        return native;
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
    if (written.isEmpty()
        || !QFileInfo::exists(QDir(previewDir).filePath(QStringLiteral("rgb_all.png"))))
    {
        const QStringList native = writeNativePreviewFallback(session, previewDir);
        for (const QString &path : native)
        {
            if (path.isEmpty() || written.contains(path))
                continue;
            written.push_back(path);
            names.push_back(QFileInfo(path).fileName());
        }
    }
    if (!names.isEmpty())
        appendLog(QStringLiteral("Session preview: wrote preview/%1").arg(names.join(QStringLiteral(", "))));
    else
        appendLog(QStringLiteral("Session preview: created preview/."));
    return written;
}

} // namespace hf::processing
