// GSAM preprocessing plan JSON (beside app.exe under gsam_plans/). Backend/offline.
// Per-stream: prompt, invert_rgb, thresholds, nms_iou, optional two_stage, optional
// reuse_reflectance_masks / reuse_masks_from (copy masks from another stream).
#pragma once

#include <QHash>
#include <QString>
#include <QStringList>

namespace hf::processing
{
struct GsamTwoStageSettings
{
    /// Stage 1 Grounding DINO boxes (typically wells/holes), then stage 2 inside each box crop.
    bool enabled = false;
    QString stage1Prompt = QStringLiteral("hole");
    QString stage2Prompt;
    double stage2BoxThreshold = 0.20;
    int cropMarginPx = 0;
    double nmsIou = 0.40;
    int maxDets = 50;
};

struct GsamPlanStreamSettings
{
    QString prompt;
    bool invertRgb = false;
    double boxThreshold = 0.30;
    double maxBoxAreaFrac = 0.0;
    double nmsIou = 0.40;
    bool hasPrompt = false;
    bool hasBoxThreshold = false;
    bool hasMaxBoxAreaFrac = false;
    bool hasInvertRgb = false;
    bool hasNmsIou = false;
    GsamTwoStageSettings twoStage;
    /// Non-empty: copy GSAM masks from this stream (e.g. reflectance/fx10e) instead of running GSAM.
    QString reuseMasksFrom;
    bool hasReuseMasksFrom = false;
};

struct GsamPlan
{
    QString name;
    int version = 1;
    int sampleCount = 5;
    bool hasSampleCount = false;
    /// Keys like "reflectance/fx10e". Empty map is still a valid plan.
    QHash<QString, GsamPlanStreamSettings> streams;
};

[[nodiscard]] QString defaultGsamPlansDirectory();

[[nodiscard]] QStringList listGsamPlanJsonFiles(const QString &plansDirectory = {});

[[nodiscard]] bool loadGsamPlan(const QString &planPath, GsamPlan *outPlan, QString *errorMessage = nullptr);

[[nodiscard]] QString gsamPlanStreamKey(bool transmittance, bool swir3);

[[nodiscard]] GsamPlanStreamSettings resolveGsamPlanStream(const GsamPlan &plan,
                                                           const QString &relativeRoot,
                                                           bool transmittance,
                                                           bool swir3);

[[nodiscard]] bool gsamPlanHasTwoStage(const GsamPlan &plan);

[[nodiscard]] QString gsamPlanComboLabel(const GsamPlan &plan, const QString &fallbackId);

[[nodiscard]] QString gsamPlanTooltip(const GsamPlan &plan);

} // namespace hf::processing
