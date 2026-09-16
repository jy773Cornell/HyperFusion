// Hemisphere / semi-fixed / FPP scan parameters for UR3e (frontend/ui layer).
#pragma once

#include "backend/multiview/Ur3eAutoHemisphereScanExecute.hpp"
#include "backend/multiview/Ur3eHemisphereScan.hpp"
#include "backend/multiview/Ur3eScanPlanCache.hpp"
#include "backend/multiview/Ur3eSemiFixedScan.hpp"
#include "backend/multiview/Ur3eWorkspaceBoundary.hpp"
#include "frontend/settings/AppSettingsStore.hpp"

#include <QWidget>

class QComboBox;
class QDoubleSpinBox;
class QPushButton;
class QSpinBox;
class QCheckBox;
class QLabel;
class QListWidget;
class QWidget;

namespace ui
{
class Ur3eHemisphereScanSettingsWidget : public QWidget
{
    Q_OBJECT

public:
    explicit Ur3eHemisphereScanSettingsWidget(QWidget *parent = nullptr);

    [[nodiscard]] hf::ur3e::Ur3eScanExecuteMode scanExecuteMode() const;
    [[nodiscard]] hf::ur3e::Ur3eHemisphereScanParams params() const;
    /// Semi Plan grid: Layer spin = ring count; imaging interval separate; φ candidates fixed.
    [[nodiscard]] hf::ur3e::Ur3eHemisphereScanParams semiPlanParams() const;
    [[nodiscard]] hf::ur3e::Ur3eWristSweepParams wristSweepParams() const;
    [[nodiscard]] hf::ur3e::Ur3eSemiFixedRoute semiFixedRoute() const;
    /// Multiview stage: pos1 = home/apex, pos2 = rings/DLP spin (mm).
    [[nodiscard]] double stagePosition1Mm() const;
    [[nodiscard]] double stagePosition2Mm() const;
    void setSemiFixedRoute(const hf::ur3e::Ur3eSemiFixedRoute &route);
    [[nodiscard]] bool semiFixedRouteReady() const;
    [[nodiscard]] bool rememberLastPlan() const;
    void setParams(const hf::ur3e::Ur3eHemisphereScanParams &params);
    /// Semi / FPP Load: apply radius / Layer / θ / interval / pan without emitting paramsChanged
    /// (paramsChanged would clear the rings about to be installed).
    void applyLoadedSemiPlanSettings(const hf::ur3e::Ur3eHemisphereScanParams &params,
                                     double intervalDeg,
                                     int panDirection,
                                     double panRangeDeg = 360.0);
    void applyBoundaryLimits(const hf::ur3e::Ur3eWorkspaceBoundary &boundary);
    /// Force Camera lens MoveIt tip (Scan tip UI removed; DLP TCP stays in cfg).
    void forceCameraScanTcp();
    void setPlanEnabled(bool enabled);
    void setExecuteEnabled(bool enabled);
    void setParamsEnabled(bool enabled);
    void setLoadRouteEnabled(bool enabled);
    /// Refresh Auto + Semi/FPP route combos from disk (cfg-matching only).
    void refreshAvailableRoutes();
    /// Persist selected Auto / Semi-FPP plan paths (and optional semi route file).
    void rememberAutoRoutePath(const QString &path);
    void rememberSemiFixedPlanPath(const QString &path);
    void rememberFppPlanPath(const QString &path);
    void rememberSemiFixedRoutePath(const QString &path);
    [[nodiscard]] QString rememberedAutoRoutePath() const;
    [[nodiscard]] QString rememberedSemiFixedPlanPath() const;
    [[nodiscard]] QString rememberedFppPlanPath() const;
    [[nodiscard]] QString rememberedSemiFixedRoutePath() const;
    /// After Plan: use reachable pin count for total-image estimate (−1 = grid estimate).
    void setPlannedReachablePins(int reachablePins);
    [[nodiscard]] int plannedReachablePins() const { return plannedReachablePins_; }

    /// Append a ring from live joints/TCP (called by panel after reading robot state).
    void appendSemiFixedRing(const std::vector<double> &jointsRad,
                             const hf::ur3e::Ur3eScanTcpPose &tcp,
                             bool hasTcp);

signals:
    void planScanRequested();
    void executeScanRequested();
    void loadScanRouteRequested(const QString &routePath);
    /// Load an Auto planned hemisphere route and convert latitudes → semi-fixed rings.
    void loadPlannedRouteAsSemiFixedRequested(const QString &routePath);
    void paramsChanged();
    void scanTcpChanged();
    void scanModeChanged();
    void semiFixedRouteChanged();
    void addSemiFixedRingRequested();

private:
    void onParameterChanged();
    void onWristSweepChanged();
    void updateImageEstimateLabel();
    void loadFromSettings();
    void saveToSettings() const;
    void applyModePanelToUi(const PersistedUr3eScanModePanelSettings &panel, bool ringMode);
    [[nodiscard]] PersistedUr3eScanModePanelSettings captureModePanelFromUi() const;
    void syncWristSweepEnabledState();
    void onLoadRouteClicked();
    void onScanModeChanged();
    void syncModeUi();
    void syncSemiFixedRouteFromUi();
    void refreshSemiFixedRingList();
    void onAddSemiFixedRingClicked();
    void onRemoveSemiFixedRingClicked();
    void onSaveSemiFixedRouteClicked();
    void onLoadSemiFixedRouteFileClicked();
    void onLoadPlannedAsSemiFixedClicked();
    void setFormFieldVisible(QWidget *field, bool visible);

    hf::ur3e::Ur3eWorkspaceBoundary boundaryLimits_;
    hf::ur3e::Ur3eSemiFixedRoute semiFixedRoute_;
    int plannedReachablePins_ = -1;

    QComboBox *modeCombo_ = nullptr;
    QWidget *autoSection_ = nullptr;
    QWidget *semiFixedSection_ = nullptr;

    QLabel *gridFormLabel_ = nullptr;
    QLabel *radiusFormLabel_ = nullptr;
    QLabel *rangeIntervalFormLabel_ = nullptr;
    QLabel *stageFormLabel_ = nullptr;
    QLabel *thetaFormLabel_ = nullptr;
    QLabel *sweepFormLabel_ = nullptr;
    QLabel *wristStepFormLabel_ = nullptr;
    QLabel *wristAxesFormLabel_ = nullptr;

    QComboBox *routeCombo_ = nullptr;
    QPushButton *loadRouteBtn_ = nullptr;
    QDoubleSpinBox *sphereRadiusSpin_ = nullptr;
    QWidget *rangeIntervalRow_ = nullptr;
    QDoubleSpinBox *fppRangeSpin_ = nullptr;
    QSpinBox *horizontalPointsSpin_ = nullptr;
    QSpinBox *verticalPointsSpin_ = nullptr;
    QLabel *gridTimesLabel_ = nullptr;
    QWidget *gridRow_ = nullptr;
    QDoubleSpinBox *thetaMinSpin_ = nullptr;
    QDoubleSpinBox *thetaMaxSpin_ = nullptr;
    QWidget *thetaRow_ = nullptr;
    QCheckBox *wristSweepEnabledCheck_ = nullptr;
    QDoubleSpinBox *wristSweepStepSpin_ = nullptr;
    QSpinBox *wristSweepStepsSpin_ = nullptr;
    QWidget *wristStepRow_ = nullptr;
    QCheckBox *wrist1Check_ = nullptr;
    QCheckBox *wrist2Check_ = nullptr;
    QCheckBox *wrist3Check_ = nullptr;
    QWidget *wristAxesRow_ = nullptr;
    QWidget *stageRow_ = nullptr;
    QDoubleSpinBox *stagePosition1Spin_ = nullptr;
    QDoubleSpinBox *stagePosition2Spin_ = nullptr;
    QLabel *imageEstimateLabel_ = nullptr;

    QComboBox *semiFixedPlanRouteCombo_ = nullptr;
    QPushButton *semiFixedLoadPlanBtn_ = nullptr;
    QDoubleSpinBox *semiFixedIntervalSpin_ = nullptr;
    QComboBox *semiFixedDirectionCombo_ = nullptr;
    QListWidget *semiFixedRingList_ = nullptr;
    QPushButton *semiFixedAddBtn_ = nullptr;
    QPushButton *semiFixedRemoveBtn_ = nullptr;
    QPushButton *semiFixedSaveBtn_ = nullptr;
    QPushButton *semiFixedLoadFileBtn_ = nullptr;

    QPushButton *planBtn_ = nullptr;
    QPushButton *executeBtn_ = nullptr;
};
} // namespace ui
