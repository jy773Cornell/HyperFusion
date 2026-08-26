// GSAM preprocessing plan JSON loader (backend). Plans live beside app.exe under gsam_plans/
// (copied from app/preset/gsam_plans on build).
#include "backend/camera/processing/GsamPlan.hpp"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QHash>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>
#include <QIODevice>

#include <algorithm>
#include <utility>

namespace hf::processing
{
QString defaultGsamPlansDirectory()
{
    const QString besideExe =
        QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("gsam_plans"));
    if (QDir(besideExe).exists())
        return besideExe;

#ifdef HF_APP_SOURCE_DIR
    const QString fromPreset =
        QDir(QString::fromUtf8(HF_APP_SOURCE_DIR)).filePath(QStringLiteral("preset/gsam_plans"));
    if (QDir(fromPreset).exists())
        return fromPreset;
#endif

    return besideExe;
}

QStringList listGsamPlanJsonFiles(const QString &plansDirectory)
{
    const QString dirPath = plansDirectory.isEmpty() ? defaultGsamPlansDirectory() : plansDirectory;
    QDir dir(dirPath);
    if (!dir.exists())
        return {};

    QStringList names = dir.entryList({QStringLiteral("*.json")}, QDir::Files, QDir::Name);
    names.sort(Qt::CaseInsensitive);
    return names;
}

QString gsamPlanStreamKey(const bool transmittance, const bool swir3)
{
    return QStringLiteral("%1/%2")
        .arg(transmittance ? QStringLiteral("transmittance") : QStringLiteral("reflectance"),
             swir3 ? QStringLiteral("swir3") : QStringLiteral("fx10e"));
}

namespace
{
GsamPlanStreamSettings parseStreamObject(const QJsonObject &obj)
{
    GsamPlanStreamSettings settings;
    if (obj.contains(QStringLiteral("prompt")))
    {
        settings.prompt = obj.value(QStringLiteral("prompt")).toString().trimmed();
        settings.hasPrompt = true;
    }
    if (obj.contains(QStringLiteral("invert_rgb")))
    {
        settings.invertRgb = obj.value(QStringLiteral("invert_rgb")).toBool(false);
        settings.hasInvertRgb = true;
    }
    if (obj.contains(QStringLiteral("box_threshold")))
    {
        settings.boxThreshold = obj.value(QStringLiteral("box_threshold")).toDouble(0.30);
        settings.hasBoxThreshold = true;
    }
    if (obj.contains(QStringLiteral("max_box_area_frac")))
    {
        settings.maxBoxAreaFrac = obj.value(QStringLiteral("max_box_area_frac")).toDouble(0.0);
        settings.hasMaxBoxAreaFrac = true;
    }
    if (obj.contains(QStringLiteral("nms_iou")))
    {
        settings.nmsIou = obj.value(QStringLiteral("nms_iou")).toDouble(0.40);
        settings.hasNmsIou = true;
    }

    const QString reuseFrom = obj.value(QStringLiteral("reuse_masks_from")).toString().trimmed();
    if (!reuseFrom.isEmpty())
    {
        settings.reuseMasksFrom = reuseFrom.toLower().replace(QLatin1Char('\\'), QLatin1Char('/'));
        settings.hasReuseMasksFrom = true;
    }
    else if (obj.value(QStringLiteral("reuse_reflectance_masks")).toBool(false))
    {
        // Resolved later when stream key is known; store sentinel for tooltip/parse.
        settings.reuseMasksFrom = QStringLiteral("reflectance/*");
        settings.hasReuseMasksFrom = true;
    }

    const QJsonValue twoStageValue = obj.value(QStringLiteral("two_stage"));
    if (twoStageValue.isBool())
    {
        settings.twoStage.enabled = twoStageValue.toBool(false);
    }
    else if (twoStageValue.isObject())
    {
        const QJsonObject twoStage = twoStageValue.toObject();
        settings.twoStage.enabled = twoStage.value(QStringLiteral("enabled")).toBool(true);
        const QString stage1 = twoStage.value(QStringLiteral("stage1_prompt")).toString().trimmed();
        if (!stage1.isEmpty())
            settings.twoStage.stage1Prompt = stage1;
        const QString stage2 = twoStage.value(QStringLiteral("stage2_prompt")).toString().trimmed();
        if (!stage2.isEmpty())
            settings.twoStage.stage2Prompt = stage2;
        if (twoStage.contains(QStringLiteral("stage2_box_threshold")))
            settings.twoStage.stage2BoxThreshold =
                twoStage.value(QStringLiteral("stage2_box_threshold")).toDouble(0.20);
        if (twoStage.contains(QStringLiteral("crop_margin_px")))
            settings.twoStage.cropMarginPx =
                std::max(0, twoStage.value(QStringLiteral("crop_margin_px")).toInt(0));
        if (twoStage.contains(QStringLiteral("nms_iou")))
            settings.twoStage.nmsIou = twoStage.value(QStringLiteral("nms_iou")).toDouble(0.40);
        if (twoStage.contains(QStringLiteral("max_dets")))
            settings.twoStage.maxDets = std::max(1, twoStage.value(QStringLiteral("max_dets")).toInt(50));
    }
    return settings;
}
} // namespace

bool loadGsamPlan(const QString &planPath, GsamPlan *outPlan, QString *errorMessage)
{
    if (outPlan == nullptr)
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("loadGsamPlan: outPlan is null");
        return false;
    }

    QFile file(planPath);
    if (!file.open(QIODevice::ReadOnly))
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Could not open GSAM plan: %1").arg(planPath);
        return false;
    }

    QJsonParseError parseError;
    const QJsonDocument doc = QJsonDocument::fromJson(file.readAll(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !doc.isObject())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("Invalid GSAM plan JSON (%1): %2")
                                .arg(planPath, parseError.errorString());
        }
        return false;
    }

    const QJsonObject root = doc.object();
    GsamPlan plan;
    plan.name = root.value(QStringLiteral("name")).toString(QFileInfo(planPath).completeBaseName());
    plan.version = root.value(QStringLiteral("version")).toInt(1);
    if (root.contains(QStringLiteral("sample_count")))
    {
        plan.sampleCount = std::max(1, root.value(QStringLiteral("sample_count")).toInt(5));
        plan.hasSampleCount = true;
    }

    const QJsonObject streams = root.value(QStringLiteral("streams")).toObject();
    for (auto it = streams.begin(); it != streams.end(); ++it)
    {
        if (!it.value().isObject())
            continue;
        const QString key = it.key().trimmed().toLower();
        if (key.isEmpty())
            continue;
        GsamPlanStreamSettings streamSettings = parseStreamObject(it.value().toObject());
        if (streamSettings.hasReuseMasksFrom &&
            streamSettings.reuseMasksFrom == QStringLiteral("reflectance/*"))
        {
            const int slash = key.indexOf(QLatin1Char('/'));
            const QString camera = slash >= 0 ? key.mid(slash + 1) : QString();
            streamSettings.reuseMasksFrom =
                camera.isEmpty() ? QStringLiteral("reflectance")
                                 : QStringLiteral("reflectance/%1").arg(camera);
        }
        plan.streams.insert(key, streamSettings);
    }

    *outPlan = std::move(plan);
    return true;
}

GsamPlanStreamSettings resolveGsamPlanStream(const GsamPlan &plan,
                                             const QString &relativeRoot,
                                             const bool transmittance,
                                             const bool swir3)
{
    const QString normalizedRoot = relativeRoot.trimmed().toLower().replace(QLatin1Char('\\'), QLatin1Char('/'));
    if (!normalizedRoot.isEmpty() && plan.streams.contains(normalizedRoot))
        return plan.streams.value(normalizedRoot);

    const QString key = gsamPlanStreamKey(transmittance, swir3).toLower();
    if (plan.streams.contains(key))
        return plan.streams.value(key);

    return {};
}

bool gsamPlanHasTwoStage(const GsamPlan &plan)
{
    for (auto it = plan.streams.cbegin(); it != plan.streams.cend(); ++it)
    {
        if (it.value().twoStage.enabled)
            return true;
    }
    return false;
}

QString gsamPlanComboLabel(const GsamPlan &plan, const QString &fallbackId)
{
    const QString name = plan.name.trimmed().isEmpty() ? fallbackId : plan.name.trimmed();
    if (gsamPlanHasTwoStage(plan))
        return QStringLiteral("%1 (two-stage)").arg(name);
    return name;
}

QString gsamPlanTooltip(const GsamPlan &plan)
{
    QStringList lines;
    lines.push_back(plan.name.trimmed().isEmpty() ? QStringLiteral("GSAM plan") : plan.name.trimmed());
    if (plan.hasSampleCount)
        lines.push_back(QStringLiteral("sample_count %1").arg(plan.sampleCount));
    QStringList keys = plan.streams.keys();
    keys.sort(Qt::CaseInsensitive);
    for (const QString &key : keys)
    {
        const GsamPlanStreamSettings stream = plan.streams.value(key);
        QString line = key;
        if (stream.hasReuseMasksFrom && !stream.reuseMasksFrom.isEmpty())
        {
            line += QStringLiteral(": reuse masks from %1").arg(stream.reuseMasksFrom);
            if (stream.twoStage.enabled)
            {
                const QString stage2 = stream.twoStage.stage2Prompt.isEmpty() ? stream.prompt
                                                                              : stream.twoStage.stage2Prompt;
                line += QStringLiteral(" (else %1 box → %2)")
                            .arg(stream.twoStage.stage1Prompt, stage2);
            }
        }
        else if (stream.twoStage.enabled)
        {
            const QString stage2 = stream.twoStage.stage2Prompt.isEmpty() ? stream.prompt
                                                                          : stream.twoStage.stage2Prompt;
            line += QStringLiteral(": %1 box → %2")
                        .arg(stream.twoStage.stage1Prompt, stage2);
        }
        else if (stream.hasPrompt && !stream.prompt.isEmpty())
        {
            line += QStringLiteral(": %1").arg(stream.prompt);
        }
        if (stream.hasInvertRgb && stream.invertRgb)
            line += QStringLiteral(" (invert RGB)");
        lines.push_back(line);
    }
    return lines.join(QLatin1Char('\n'));
}

} // namespace hf::processing
