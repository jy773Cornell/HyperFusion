// Hemisphere scan parameters for UR3e sample-tray dome (frontend/ui layer).
#pragma once

#include "backend/3dscanning/Ur3eHemisphereScan.hpp"
#include "backend/3dscanning/Ur3eScanPlanCache.hpp"
#include "backend/3dscanning/Ur3eWorkspaceBoundary.hpp"

#include <QWidget>

class QComboBox;
class QDoubleSpinBox;
class QPushButton;
class QSpinBox;
class QCheckBox;
class QLabel;

namespace ui
{
class Ur3eHemisphereScanSettingsWidget : public QWidget
{
    Q_OBJECT

public:
    explicit Ur3eHemisphereScanSettingsWidget(QWidget *parent = nullptr);

    [[nodiscard]] hf::ur3e::Ur3eHemisphereScanParams params() const;
    [[nodiscard]] hf::ur3e::Ur3eWristSweepParams wristSweepParams() const;
    [[nodiscard]] bool rememberLastPlan() const;
    void setParams(const hf::ur3e::Ur3eHemisphereScanParams &params);
    void applyBoundaryLimits(const hf::ur3e::Ur3eWorkspaceBoundary &boundary);
    void setPlanEnabled(bool enabled);
    void setExecuteEnabled(bool enabled);
    void setParamsEnabled(bool enabled);
    void setLoadRouteEnabled(bool enabled);
    /// Refresh combo from disk (routes matching current robot cfg only).
    void refreshAvailableRoutes();
    /// After Plan: use reachable pin count for total-image estimate (−1 = grid estimate).
    void setPlannedReachablePins(int reachablePins);
    [[nodiscard]] int plannedReachablePins() const { return plannedReachablePins_; }

signals:
    void planScanRequested();
    void executeScanRequested();
    void loadScanRouteRequested(const QString &routePath);
    void paramsChanged();

private:
    void onParameterChanged();
    void onWristSweepChanged();
    void updateImageEstimateLabel();
    void loadFromSettings();
    void saveToSettings() const;
    void syncWristSweepEnabledState();
    void onLoadRouteClicked();

    hf::ur3e::Ur3eWorkspaceBoundary boundaryLimits_;
    int plannedReachablePins_ = -1;
    QComboBox *routeCombo_ = nullptr;
    QPushButton *loadRouteBtn_ = nullptr;
    QDoubleSpinBox *sphereRadiusSpin_ = nullptr;
    QSpinBox *horizontalPointsSpin_ = nullptr;
    QSpinBox *verticalPointsSpin_ = nullptr;
    QDoubleSpinBox *thetaMinSpin_ = nullptr;
    QDoubleSpinBox *thetaMaxSpin_ = nullptr;
    QCheckBox *wristSweepEnabledCheck_ = nullptr;
    QDoubleSpinBox *wristSweepStepSpin_ = nullptr;
    QSpinBox *wristSweepStepsSpin_ = nullptr;
    QCheckBox *wrist1Check_ = nullptr;
    QCheckBox *wrist2Check_ = nullptr;
    QCheckBox *wrist3Check_ = nullptr;
    QLabel *imageEstimateLabel_ = nullptr;
    QPushButton *planBtn_ = nullptr;
    QPushButton *executeBtn_ = nullptr;
};
} // namespace ui
