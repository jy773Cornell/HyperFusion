// Hemisphere scan parameters for UR3e sample-tray dome (frontend/ui layer).
#include "frontend/widgets/Ur3eHemisphereScanSettingsWidget.hpp"

#include "backend/HyperFusionConfig.hpp"
#include "backend/ur3e/Ur3eHemisphereScan.hpp"
#include "backend/ur3e/Ur3eWorkspaceBoundary.hpp"

#include "frontend/settings/AppSettingsStore.hpp"

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

  auto *group = new QGroupBox(QStringLiteral("Scanning route"), this);
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
  connect(planBtn_, &QPushButton::clicked, this,
          [this]() { emit planScanRequested(); });
  connect(executeBtn_, &QPushButton::clicked, this,
          [this]() { emit executeScanRequested(); });

  loadFromSettings();
  applyBoundaryLimits(
      hf::ur3e::workspaceBoundaryFromConfig(hf::hardwareConfig().ur3e));
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

    sphereRadiusSpin_->setValue(saved.sphereRadiusMm);
    horizontalPointsSpin_->setValue(saved.horizontalPoints);
    verticalPointsSpin_->setValue(saved.verticalPoints);
    thetaMinSpin_->setValue(saved.thetaMinDeg);
    thetaMaxSpin_->setValue(saved.thetaMaxDeg);
}

void Ur3eHemisphereScanSettingsWidget::saveToSettings() const
{
    PersistedUr3eHemisphereScanSettings saved;
    saved.sphereRadiusMm = sphereRadiusSpin_->value();
    saved.horizontalPoints = horizontalPointsSpin_->value();
    saved.verticalPoints = verticalPointsSpin_->value();
    saved.thetaMinDeg = thetaMinSpin_->value();
    saved.thetaMaxDeg = thetaMaxSpin_->value();
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
}

void Ur3eHemisphereScanSettingsWidget::onParameterChanged() {
  if (thetaMinSpin_->value() > thetaMaxSpin_->value())
    thetaMaxSpin_->setValue(thetaMinSpin_->value());

  const double maxRadiusMm =
      hf::ur3e::maxHemisphereRadiusM(boundaryLimits_) * 1000.0;
  if (sphereRadiusSpin_->value() > maxRadiusMm)
    sphereRadiusSpin_->setValue(maxRadiusMm);

  saveToSettings();
  emit paramsChanged();
}
} // namespace ui
