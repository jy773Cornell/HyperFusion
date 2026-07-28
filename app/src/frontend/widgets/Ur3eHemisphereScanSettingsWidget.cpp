// Hemisphere scan parameters for UR3e sample-tray dome (frontend/ui layer).
#include "frontend/widgets/Ur3eHemisphereScanSettingsWidget.hpp"

#include "backend/HyperFusionConfig.hpp"
#include "backend/3dscanning/Ur3eHemisphereScan.hpp"
#include "backend/3dscanning/Ur3eWorkspaceBoundary.hpp"

#include "frontend/settings/AppSettingsStore.hpp"

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QVBoxLayout>

namespace ui {
Ur3eHemisphereScanSettingsWidget::Ur3eHemisphereScanSettingsWidget(
    QWidget *parent)
    : QWidget(parent) {
  auto *layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(6);

  auto *group = new QGroupBox(QStringLiteral("Scanning"), this);
  auto *form = new QFormLayout(group);

  sphereRadiusSpin_ = new QDoubleSpinBox(group);
  sphereRadiusSpin_->setRange(10.0, 5000.0);
  sphereRadiusSpin_->setDecimals(0);
  sphereRadiusSpin_->setSingleStep(50.0);
  sphereRadiusSpin_->setSuffix(QStringLiteral(" mm"));
  sphereRadiusSpin_->setValue(500.0);
  form->addRow(QStringLiteral("Sphere radius"), sphereRadiusSpin_);

  horizontalPointsSpin_ = new QSpinBox(group);
  horizontalPointsSpin_->setRange(1, 360);
  horizontalPointsSpin_->setValue(12);

  verticalPointsSpin_ = new QSpinBox(group);
  verticalPointsSpin_->setRange(1, 180);
  verticalPointsSpin_->setValue(5);

  auto *gridRow = new QWidget(group);
  auto *gridLayout = new QHBoxLayout(gridRow);
  gridLayout->setContentsMargins(0, 0, 0, 0);
  gridLayout->setSpacing(6);
  gridLayout->addWidget(horizontalPointsSpin_);
  auto *gridTimes = new QLabel(QStringLiteral("×"), gridRow);
  gridTimes->setAlignment(Qt::AlignCenter);
  gridLayout->addWidget(gridTimes);
  gridLayout->addWidget(verticalPointsSpin_);
  form->addRow(QStringLiteral("Grid points"), gridRow);

  thetaMinSpin_ = new QDoubleSpinBox(group);
  thetaMinSpin_->setRange(0.0, 90.0);
  thetaMinSpin_->setDecimals(2);
  thetaMinSpin_->setSingleStep(1.0);
  thetaMinSpin_->setSuffix(QStringLiteral(" °"));
  thetaMinSpin_->setValue(30.0);
  thetaMinSpin_->setToolTip(QStringLiteral(
      "Polar angle from dome apex (0°) toward the tray rim (90°)."));

  thetaMaxSpin_ = new QDoubleSpinBox(group);
  thetaMaxSpin_->setRange(0.0, 90.0);
  thetaMaxSpin_->setDecimals(2);
  thetaMaxSpin_->setSingleStep(1.0);
  thetaMaxSpin_->setSuffix(QStringLiteral(" °"));
  thetaMaxSpin_->setValue(90.0);
  thetaMaxSpin_->setToolTip(QStringLiteral(
      "Upper hemisphere only — maximum 90° (equator of the dome)."));

  auto *thetaRow = new QWidget(group);
  auto *thetaLayout = new QHBoxLayout(thetaRow);
  thetaLayout->setContentsMargins(0, 0, 0, 0);
  thetaLayout->setSpacing(6);
  thetaLayout->addWidget(thetaMinSpin_);
  auto *thetaDash = new QLabel(QStringLiteral("–"), thetaRow);
  thetaDash->setAlignment(Qt::AlignCenter);
  thetaLayout->addWidget(thetaDash);
  thetaLayout->addWidget(thetaMaxSpin_);
  form->addRow(QStringLiteral("Theta range"), thetaRow);

  wristSweepEnabledCheck_ = new QCheckBox(QStringLiteral("Enabled"), group);
  wristSweepEnabledCheck_->setToolTip(
      QStringLiteral("After each pin, permute selected wrists for multiview stills "
                     "(collision skips). Center pose is always included."));
  form->addRow(QStringLiteral("Wrist permutation"), wristSweepEnabledCheck_);

  wristSweepStepSpin_ = new QDoubleSpinBox(group);
  wristSweepStepSpin_->setRange(0.5, 45.0);
  wristSweepStepSpin_->setDecimals(1);
  wristSweepStepSpin_->setSingleStep(1.0);
  wristSweepStepSpin_->setSuffix(QStringLiteral(" °"));
  wristSweepStepSpin_->setValue(3.0);
  form->addRow(QStringLiteral("Wrist step"), wristSweepStepSpin_);

  wristSweepStepsSpin_ = new QSpinBox(group);
  wristSweepStepsSpin_->setRange(1, 12);
  wristSweepStepsSpin_->setValue(4);
  wristSweepStepsSpin_->setToolTip(
      QStringLiteral("Steps each way from center (±N×step). Offsets exclude 0 "
                     "(center is captured separately)."));
  form->addRow(QStringLiteral("Steps each way"), wristSweepStepsSpin_);

  wrist1Check_ = new QCheckBox(QStringLiteral("wrist_1"), group);
  wrist2Check_ = new QCheckBox(QStringLiteral("wrist_2"), group);
  wrist3Check_ = new QCheckBox(QStringLiteral("wrist_3"), group);
  wrist2Check_->setChecked(true);
  wrist3Check_->setChecked(true);
  auto *wristAxesRow = new QWidget(group);
  auto *wristAxesLayout = new QHBoxLayout(wristAxesRow);
  wristAxesLayout->setContentsMargins(0, 0, 0, 0);
  wristAxesLayout->setSpacing(8);
  wristAxesLayout->addWidget(wrist1Check_);
  wristAxesLayout->addWidget(wrist2Check_);
  wristAxesLayout->addWidget(wrist3Check_);
  wristAxesLayout->addStretch(1);
  form->addRow(QStringLiteral("Permute"), wristAxesRow);

  imageEstimateLabel_ = new QLabel(group);
  imageEstimateLabel_->setWordWrap(true);
  imageEstimateLabel_->setStyleSheet(QStringLiteral("color: #444;"));
  form->addRow(QStringLiteral("Images"), imageEstimateLabel_);

  planBtn_ = new QPushButton(QStringLiteral("Plan"), group);
  planBtn_->setToolTip(
      QStringLiteral("Generate the scan grid and check each pose with MoveIt IK + collision."));

  executeBtn_ = new QPushButton(QStringLiteral("Execute"), group);
  executeBtn_->setEnabled(false);
  executeBtn_->setToolTip(
      QStringLiteral("Execute the planned scan using collision-aware MoveIt joint motions."));

  auto *actionRow = new QWidget(group);
  auto *actionLayout = new QHBoxLayout(actionRow);
  actionLayout->setContentsMargins(0, 0, 0, 0);
  actionLayout->setSpacing(8);
  actionLayout->addWidget(planBtn_, 1);
  actionLayout->addWidget(executeBtn_, 1);
  form->addRow(QStringLiteral(""), actionRow);

  layout->addWidget(group);

  const auto hook = [this]() { onParameterChanged(); };
  connect(sphereRadiusSpin_, qOverload<double>(&QDoubleSpinBox::valueChanged),
          this, hook);
  connect(horizontalPointsSpin_, qOverload<int>(&QSpinBox::valueChanged), this,
          hook);
  connect(verticalPointsSpin_, qOverload<int>(&QSpinBox::valueChanged), this,
          hook);
  connect(thetaMinSpin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
          hook);
  connect(thetaMaxSpin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
          hook);

  const auto wristHook = [this]() { onWristSweepChanged(); };
  connect(wristSweepEnabledCheck_, &QCheckBox::toggled, this, wristHook);
  connect(wristSweepStepSpin_, qOverload<double>(&QDoubleSpinBox::valueChanged),
          this, wristHook);
  connect(wristSweepStepsSpin_, qOverload<int>(&QSpinBox::valueChanged), this,
          wristHook);
  connect(wrist1Check_, &QCheckBox::toggled, this, wristHook);
  connect(wrist2Check_, &QCheckBox::toggled, this, wristHook);
  connect(wrist3Check_, &QCheckBox::toggled, this, wristHook);

  connect(planBtn_, &QPushButton::clicked, this,
          [this]() { emit planScanRequested(); });
  connect(executeBtn_, &QPushButton::clicked, this,
          [this]() { emit executeScanRequested(); });

  loadFromSettings();
  applyBoundaryLimits(
      hf::ur3e::workspaceBoundaryFromConfig(hf::hardwareConfig().ur3e));
  syncWristSweepEnabledState();
  updateImageEstimateLabel();
  saveToSettings();
}

void Ur3eHemisphereScanSettingsWidget::loadFromSettings()
{
    const PersistedUr3eHemisphereScanSettings saved = AppSettingsStore::loadUr3eHemisphereScan();

    const QSignalBlocker blockRadius(sphereRadiusSpin_);
    const QSignalBlocker blockHorizontal(horizontalPointsSpin_);
    const QSignalBlocker blockVertical(verticalPointsSpin_);
    const QSignalBlocker blockThetaMin(thetaMinSpin_);
    const QSignalBlocker blockThetaMax(thetaMaxSpin_);
    const QSignalBlocker blockWristEn(wristSweepEnabledCheck_);
    const QSignalBlocker blockWristStep(wristSweepStepSpin_);
    const QSignalBlocker blockWristSteps(wristSweepStepsSpin_);
    const QSignalBlocker blockW1(wrist1Check_);
    const QSignalBlocker blockW2(wrist2Check_);
    const QSignalBlocker blockW3(wrist3Check_);

    sphereRadiusSpin_->setValue(saved.sphereRadiusMm);
    horizontalPointsSpin_->setValue(saved.horizontalPoints);
    verticalPointsSpin_->setValue(saved.verticalPoints);
    thetaMinSpin_->setValue(saved.thetaMinDeg);
    thetaMaxSpin_->setValue(saved.thetaMaxDeg);
    wristSweepEnabledCheck_->setChecked(saved.wristSweepEnabled);
    wristSweepStepSpin_->setValue(saved.wristSweepStepDeg);
    wristSweepStepsSpin_->setValue(saved.wristSweepStepsEachWay);
    wrist1Check_->setChecked(saved.wristSweepWrist1);
    wrist2Check_->setChecked(saved.wristSweepWrist2);
    wrist3Check_->setChecked(saved.wristSweepWrist3);
}

void Ur3eHemisphereScanSettingsWidget::saveToSettings() const
{
    PersistedUr3eHemisphereScanSettings saved;
    saved.sphereRadiusMm = sphereRadiusSpin_->value();
    saved.horizontalPoints = horizontalPointsSpin_->value();
    saved.verticalPoints = verticalPointsSpin_->value();
    saved.thetaMinDeg = thetaMinSpin_->value();
    saved.thetaMaxDeg = thetaMaxSpin_->value();
    const hf::ur3e::Ur3eWristSweepParams wrist = wristSweepParams();
    saved.wristSweepEnabled = wrist.enabled;
    saved.wristSweepStepDeg = wrist.stepDeg;
    saved.wristSweepStepsEachWay = wrist.stepsEachWay;
    saved.wristSweepWrist1 = wrist.wrist1;
    saved.wristSweepWrist2 = wrist.wrist2;
    saved.wristSweepWrist3 = wrist.wrist3;
    AppSettingsStore::saveUr3eHemisphereScan(saved);
}

void Ur3eHemisphereScanSettingsWidget::applyBoundaryLimits(
    const hf::ur3e::Ur3eWorkspaceBoundary &boundary) {
  boundaryLimits_ = boundary;
  boundaryLimits_.normalize();

  const double maxRadiusM = hf::ur3e::maxHemisphereRadiusM(boundaryLimits_);
  const double maxRadiusMm = maxRadiusM * 1000.0;
  sphereRadiusSpin_->setMaximum(maxRadiusMm);
  if (sphereRadiusSpin_->value() > maxRadiusMm)
    sphereRadiusSpin_->setValue(maxRadiusMm);

  if (boundaryLimits_.enabled) {
    sphereRadiusSpin_->setToolTip(
        QStringLiteral(
            "Max %1 mm for workspace %2×%3×%4 mm (depth below mount at %5 mm) in hyperfusion.cfg.")
            .arg(maxRadiusMm, 0, 'f', 0)
            .arg(static_cast<int>(boundaryLimits_.lengthMm))
            .arg(static_cast<int>(boundaryLimits_.widthMm))
            .arg(static_cast<int>(boundaryLimits_.heightMm))
            .arg(static_cast<int>(boundaryLimits_.mountHeightMm)));
  } else {
    sphereRadiusSpin_->setToolTip(
        QStringLiteral("Workspace limits disabled in hyperfusion.cfg."));
  }
}

hf::ur3e::Ur3eHemisphereScanParams
Ur3eHemisphereScanSettingsWidget::params() const {
  hf::ur3e::Ur3eHemisphereScanParams scanParams;
  scanParams.sphereRadiusM = sphereRadiusSpin_->value() * 0.001;
  scanParams.horizontalPoints = horizontalPointsSpin_->value();
  scanParams.verticalPoints = verticalPointsSpin_->value();
  scanParams.thetaMinDeg = thetaMinSpin_->value();
  scanParams.thetaMaxDeg = thetaMaxSpin_->value();
  hf::ur3e::clampHemisphereScanParamsToBoundary(scanParams, boundaryLimits_);
  return scanParams;
}

hf::ur3e::Ur3eWristSweepParams
Ur3eHemisphereScanSettingsWidget::wristSweepParams() const
{
  hf::ur3e::Ur3eWristSweepParams wrist;
  wrist.enabled = wristSweepEnabledCheck_ != nullptr && wristSweepEnabledCheck_->isChecked();
  wrist.stepDeg = wristSweepStepSpin_ != nullptr ? wristSweepStepSpin_->value() : 3.0;
  wrist.stepsEachWay = wristSweepStepsSpin_ != nullptr ? wristSweepStepsSpin_->value() : 4;
  wrist.wrist1 = wrist1Check_ != nullptr && wrist1Check_->isChecked();
  wrist.wrist2 = wrist2Check_ != nullptr && wrist2Check_->isChecked();
  wrist.wrist3 = wrist3Check_ != nullptr && wrist3Check_->isChecked();
  return wrist;
}

bool Ur3eHemisphereScanSettingsWidget::rememberLastPlan() const
{
    return hf::hardwareConfig().ur3e.rememberLastScanPlan;
}

void Ur3eHemisphereScanSettingsWidget::setPlanEnabled(const bool enabled) {
  if (planBtn_ != nullptr)
    planBtn_->setEnabled(enabled);
}

void Ur3eHemisphereScanSettingsWidget::setExecuteEnabled(const bool enabled) {
  if (executeBtn_ != nullptr)
    executeBtn_->setEnabled(enabled);
}

void Ur3eHemisphereScanSettingsWidget::setParamsEnabled(const bool enabled) {
  if (sphereRadiusSpin_ != nullptr)
    sphereRadiusSpin_->setEnabled(enabled);
  if (horizontalPointsSpin_ != nullptr)
    horizontalPointsSpin_->setEnabled(enabled);
  if (verticalPointsSpin_ != nullptr)
    verticalPointsSpin_->setEnabled(enabled);
  if (thetaMinSpin_ != nullptr)
    thetaMinSpin_->setEnabled(enabled);
  if (thetaMaxSpin_ != nullptr)
    thetaMaxSpin_->setEnabled(enabled);
  if (wristSweepEnabledCheck_ != nullptr)
    wristSweepEnabledCheck_->setEnabled(enabled);
  syncWristSweepEnabledState();
  if (!enabled) {
    if (wristSweepStepSpin_ != nullptr)
      wristSweepStepSpin_->setEnabled(false);
    if (wristSweepStepsSpin_ != nullptr)
      wristSweepStepsSpin_->setEnabled(false);
    if (wrist1Check_ != nullptr)
      wrist1Check_->setEnabled(false);
    if (wrist2Check_ != nullptr)
      wrist2Check_->setEnabled(false);
    if (wrist3Check_ != nullptr)
      wrist3Check_->setEnabled(false);
  }
}

void Ur3eHemisphereScanSettingsWidget::setPlannedReachablePins(const int reachablePins)
{
  plannedReachablePins_ = reachablePins;
  updateImageEstimateLabel();
}

void Ur3eHemisphereScanSettingsWidget::syncWristSweepEnabledState()
{
  const bool on = wristSweepEnabledCheck_ != nullptr && wristSweepEnabledCheck_->isChecked()
                  && wristSweepEnabledCheck_->isEnabled();
  if (wristSweepStepSpin_ != nullptr)
    wristSweepStepSpin_->setEnabled(on);
  if (wristSweepStepsSpin_ != nullptr)
    wristSweepStepsSpin_->setEnabled(on);
  if (wrist1Check_ != nullptr)
    wrist1Check_->setEnabled(on);
  if (wrist2Check_ != nullptr)
    wrist2Check_->setEnabled(on);
  if (wrist3Check_ != nullptr)
    wrist3Check_->setEnabled(on);
}

void Ur3eHemisphereScanSettingsWidget::updateImageEstimateLabel()
{
  if (imageEstimateLabel_ == nullptr)
    return;

  const hf::ur3e::Ur3eWristSweepParams wrist = wristSweepParams();
  const int perPin = wrist.imagesPerPin();
  const int gridPins = hf::ur3e::hemisphereScanPointCount(params());
  const int pinCount = plannedReachablePins_ >= 0 ? plannedReachablePins_ : gridPins;
  const qint64 total = static_cast<qint64>(perPin) * static_cast<qint64>(pinCount);

  const QString pinSource = plannedReachablePins_ >= 0
                                ? QStringLiteral("%1 reachable").arg(pinCount)
                                : QStringLiteral("%1 grid").arg(pinCount);

  imageEstimateLabel_->setText(
      QStringLiteral("%1 / pin × %2 ≈ %3 total")
          .arg(perPin)
          .arg(pinSource)
          .arg(total));
}

void Ur3eHemisphereScanSettingsWidget::onWristSweepChanged()
{
  syncWristSweepEnabledState();
  saveToSettings();
  updateImageEstimateLabel();
}

void Ur3eHemisphereScanSettingsWidget::onParameterChanged() {
  if (thetaMinSpin_->value() > thetaMaxSpin_->value())
    thetaMaxSpin_->setValue(thetaMinSpin_->value());

  const double maxRadiusMm =
      hf::ur3e::maxHemisphereRadiusM(boundaryLimits_) * 1000.0;
  if (sphereRadiusSpin_->value() > maxRadiusMm)
    sphereRadiusSpin_->setValue(maxRadiusMm);

  plannedReachablePins_ = -1;
  saveToSettings();
  updateImageEstimateLabel();
  emit paramsChanged();
}
} // namespace ui
