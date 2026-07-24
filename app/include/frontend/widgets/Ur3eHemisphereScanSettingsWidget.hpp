// Hemisphere scan parameters for UR3e sample-tray dome (frontend/ui layer).
#pragma once

#include "backend/3dscanning/Ur3eHemisphereScan.hpp"
#include "backend/3dscanning/Ur3eWorkspaceBoundary.hpp"

#include <QWidget>

class QDoubleSpinBox;
class QPushButton;
class QSpinBox;
class QCheckBox;

namespace ui
{
class Ur3eHemisphereScanSettingsWidget : public QWidget
{
    Q_OBJECT

public:
    explicit Ur3eHemisphereScanSettingsWidget(QWidget *parent = nullptr);

    [[nodiscard]] hf::ur3e::Ur3eHemisphereScanParams params() const;
    [[nodiscard]] bool rememberLastPlan() const;
    void applyBoundaryLimits(const hf::ur3e::Ur3eWorkspaceBoundary &boundary);
    void setPlanEnabled(bool enabled);
    void setExecuteEnabled(bool enabled);
    void setParamsEnabled(bool enabled);

signals:
    void planScanRequested();
    void executeScanRequested();
    void paramsChanged();

private:
    void onParameterChanged();
    void loadFromSettings();
    void saveToSettings() const;

    hf::ur3e::Ur3eWorkspaceBoundary boundaryLimits_;
    QDoubleSpinBox *sphereRadiusSpin_ = nullptr;
    QSpinBox *horizontalPointsSpin_ = nullptr;
    QSpinBox *verticalPointsSpin_ = nullptr;
    QDoubleSpinBox *thetaMinSpin_ = nullptr;
    QDoubleSpinBox *thetaMaxSpin_ = nullptr;
    QCheckBox *rememberLastPlanCheck_ = nullptr;
    QPushButton *planBtn_ = nullptr;
    QPushButton *executeBtn_ = nullptr;
};
} // namespace ui
