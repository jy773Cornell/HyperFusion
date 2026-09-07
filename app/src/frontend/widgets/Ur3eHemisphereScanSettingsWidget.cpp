// Hemisphere / semi-fixed scan parameters for UR3e (frontend/ui layer).
#include "frontend/widgets/Ur3eHemisphereScanSettingsWidget.hpp"

#include "backend/HyperFusionConfig.hpp"
#include "backend/multiview/Ur3eHemisphereScan.hpp"
#include "backend/multiview/Ur3eScanPlanCache.hpp"
#include "backend/multiview/Ur3eSemiFixedScan.hpp"
#include "backend/multiview/Ur3eWorkspaceBoundary.hpp"

#include "frontend/settings/AppSettingsStore.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QSpinBox>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

namespace
{
/// GUI Radius spin cap (mm). Workspace half-width is 300 mm; rings at θ≲35° still fit at 400.
constexpr double kGuiSphereRadiusMaxMm = 400.0;
} // namespace

namespace ui
{
Ur3eHemisphereScanSettingsWidget::Ur3eHemisphereScanSettingsWidget(QWidget *parent)
    : QWidget(parent)
{
    auto *layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(6);

    auto *group = new QGroupBox(QStringLiteral("Multiview"), this);
    auto *form = new QFormLayout(group);
    form->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    form->setFormAlignment(Qt::AlignLeft | Qt::AlignTop);
    form->setFieldGrowthPolicy(QFormLayout::ExpandingFieldsGrow);
    form->setRowWrapPolicy(QFormLayout::DontWrapRows);
    form->setHorizontalSpacing(8);
    form->setVerticalSpacing(4);

    modeCombo_ = new QComboBox(group);
    modeCombo_->addItem(QStringLiteral("Auto planning"),
                        static_cast<int>(hf::ur3e::Ur3eScanExecuteMode::AutoHemisphere));
    modeCombo_->addItem(QStringLiteral("Semi-fixed"),
                        static_cast<int>(hf::ur3e::Ur3eScanExecuteMode::SemiFixed));
    modeCombo_->setToolTip(
        QStringLiteral("Auto: MoveIt hemisphere grid + wrist sweep.\n"
                       "Semi-fixed: Plan finds base-sweep OK ring entries (own folder); "
                       "Execute pans each ring (no collision on spin) + wrist."));
    form->addRow(QStringLiteral("Mode"), modeCombo_);

    // --- Auto section ---
    autoSection_ = new QWidget(group);
    auto *autoForm = new QFormLayout(autoSection_);
    autoForm->setContentsMargins(0, 0, 0, 0);
    autoForm->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    autoForm->setFieldGrowthPolicy(QFormLayout::ExpandingFieldsGrow);
    autoForm->setHorizontalSpacing(8);
    autoForm->setVerticalSpacing(4);

    routeCombo_ = new QComboBox(autoSection_);
    routeCombo_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    routeCombo_->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    routeCombo_->setMinimumContentsLength(8);
    routeCombo_->setToolTip(
        QStringLiteral("Saved scan routes that match the current robot cfg "
                       "(hyperfusion.cfg). Changing sphere/grid does not hide them — "
                       "Load applies that route's grid + planned joints."));
    loadRouteBtn_ = new QPushButton(QStringLiteral("Load"), autoSection_);
    loadRouteBtn_->setEnabled(false);
    loadRouteBtn_->setToolTip(QStringLiteral("Load the selected route (cfg must match)."));
    auto *routeRow = new QWidget(autoSection_);
    auto *routeLayout = new QHBoxLayout(routeRow);
    routeLayout->setContentsMargins(0, 0, 0, 0);
    routeLayout->setSpacing(6);
    routeLayout->addWidget(routeCombo_, 1);
    routeLayout->addWidget(loadRouteBtn_, 0);
    autoForm->addRow(QStringLiteral("Route"), routeRow);

    sphereRadiusSpin_ = new QDoubleSpinBox(autoSection_);
    sphereRadiusSpin_->setRange(10.0, kGuiSphereRadiusMaxMm);
    sphereRadiusSpin_->setDecimals(0);
    sphereRadiusSpin_->setSingleStep(10.0);
    sphereRadiusSpin_->setSuffix(QStringLiteral(" mm"));
    sphereRadiusSpin_->setValue(500.0);
    autoForm->addRow(QStringLiteral("Radius"), sphereRadiusSpin_);

    horizontalPointsSpin_ = new QSpinBox(autoSection_);
    horizontalPointsSpin_->setRange(1, 360);
    horizontalPointsSpin_->setValue(12);

    verticalPointsSpin_ = new QSpinBox(autoSection_);
    verticalPointsSpin_->setRange(1, 180);
    verticalPointsSpin_->setValue(5);

    auto *gridRow = new QWidget(autoSection_);
    auto *gridLayout = new QHBoxLayout(gridRow);
    gridLayout->setContentsMargins(0, 0, 0, 0);
    gridLayout->setSpacing(6);
    gridLayout->addWidget(horizontalPointsSpin_);
    auto *gridTimes = new QLabel(QStringLiteral("×"), gridRow);
    gridTimes->setAlignment(Qt::AlignCenter);
    gridLayout->addWidget(gridTimes);
    gridLayout->addWidget(verticalPointsSpin_);
    // Semi mode: imaging interval replaces vertical pin count in the same slot.
    semiFixedIntervalSpin_ = new QDoubleSpinBox(gridRow);
    semiFixedIntervalSpin_->setRange(1.0, 90.0);
    semiFixedIntervalSpin_->setDecimals(1);
    semiFixedIntervalSpin_->setSingleStep(1.0);
    semiFixedIntervalSpin_->setSuffix(QStringLiteral(" °"));
    semiFixedIntervalSpin_->setValue(10.0);
    semiFixedIntervalSpin_->setToolTip(
        QStringLiteral("Photo every this many degrees of shoulder_pan around each ring."));
    semiFixedIntervalSpin_->setVisible(false);
    gridLayout->addWidget(semiFixedIntervalSpin_);
    autoForm->addRow(QStringLiteral("Grid"), gridRow);
    gridFormLabel_ = qobject_cast<QLabel *>(autoForm->labelForField(gridRow));

    thetaMinSpin_ = new QDoubleSpinBox(autoSection_);
    thetaMinSpin_->setRange(0.0, 90.0);
    thetaMinSpin_->setDecimals(2);
    thetaMinSpin_->setSingleStep(1.0);
    thetaMinSpin_->setSuffix(QStringLiteral(" °"));
    thetaMinSpin_->setValue(30.0);

    thetaMaxSpin_ = new QDoubleSpinBox(autoSection_);
    thetaMaxSpin_->setRange(0.0, 90.0);
    thetaMaxSpin_->setDecimals(2);
    thetaMaxSpin_->setSingleStep(1.0);
    thetaMaxSpin_->setSuffix(QStringLiteral(" °"));
    thetaMaxSpin_->setValue(90.0);

    auto *thetaRow = new QWidget(autoSection_);
    auto *thetaLayout = new QHBoxLayout(thetaRow);
    thetaLayout->setContentsMargins(0, 0, 0, 0);
    thetaLayout->setSpacing(6);
    thetaLayout->addWidget(thetaMinSpin_);
    auto *thetaDash = new QLabel(QStringLiteral("–"), thetaRow);
    thetaDash->setAlignment(Qt::AlignCenter);
    thetaLayout->addWidget(thetaDash);
    thetaLayout->addWidget(thetaMaxSpin_);
    autoForm->addRow(QStringLiteral("Theta"), thetaRow);

    wristSweepEnabledCheck_ = new QCheckBox(QStringLiteral("On"), autoSection_);
    wristSweepEnabledCheck_->setToolTip(
        QStringLiteral("After each pin, permute selected wrists for multiview stills "
                       "(collision skips). Center pose is always included."));
    autoForm->addRow(QStringLiteral("Sweep"), wristSweepEnabledCheck_);

    wristSweepStepSpin_ = new QDoubleSpinBox(autoSection_);
    wristSweepStepSpin_->setRange(0.5, 45.0);
    wristSweepStepSpin_->setDecimals(1);
    wristSweepStepSpin_->setSingleStep(1.0);
    wristSweepStepSpin_->setSuffix(QStringLiteral(" °"));
    wristSweepStepSpin_->setValue(3.0);

    wristSweepStepsSpin_ = new QSpinBox(autoSection_);
    wristSweepStepsSpin_->setRange(1, 12);
    wristSweepStepsSpin_->setValue(1);

    auto *wristStepRow = new QWidget(autoSection_);
    auto *wristStepLayout = new QHBoxLayout(wristStepRow);
    wristStepLayout->setContentsMargins(0, 0, 0, 0);
    wristStepLayout->setSpacing(6);
    wristStepLayout->addWidget(wristSweepStepSpin_, 1);
    auto *wristStepsLabel = new QLabel(QStringLiteral("±"), wristStepRow);
    wristStepsLabel->setAlignment(Qt::AlignCenter);
    wristStepLayout->addWidget(wristStepsLabel);
    wristStepLayout->addWidget(wristSweepStepsSpin_, 1);
    autoForm->addRow(QStringLiteral("Step / ±"), wristStepRow);

    wrist1Check_ = new QCheckBox(QStringLiteral("w1"), autoSection_);
    wrist2Check_ = new QCheckBox(QStringLiteral("w2"), autoSection_);
    wrist3Check_ = new QCheckBox(QStringLiteral("w3"), autoSection_);
    wrist2Check_->setChecked(true);
    wrist3Check_->setChecked(false);
    auto *wristAxesRow = new QWidget(autoSection_);
    auto *wristAxesLayout = new QHBoxLayout(wristAxesRow);
    wristAxesLayout->setContentsMargins(0, 0, 0, 0);
    wristAxesLayout->setSpacing(8);
    wristAxesLayout->addWidget(wrist1Check_);
    wristAxesLayout->addWidget(wrist2Check_);
    wristAxesLayout->addWidget(wrist3Check_);
    wristAxesLayout->addStretch(1);
    autoForm->addRow(QStringLiteral("Axes"), wristAxesRow);

    imageEstimateLabel_ = new QLabel(autoSection_);
    imageEstimateLabel_->setWordWrap(false);
    imageEstimateLabel_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    imageEstimateLabel_->setStyleSheet(QStringLiteral("color: #444;"));
    autoForm->addRow(QStringLiteral("Images"), imageEstimateLabel_);

    form->addRow(autoSection_);

    // --- Semi-fixed section ---
    semiFixedSection_ = new QWidget(group);
    auto *semiForm = new QFormLayout(semiFixedSection_);
    semiForm->setContentsMargins(0, 0, 0, 0);
    semiForm->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    semiForm->setFieldGrowthPolicy(QFormLayout::ExpandingFieldsGrow);
    semiForm->setHorizontalSpacing(8);
    semiForm->setVerticalSpacing(4);

    semiFixedDirectionCombo_ = new QComboBox(semiFixedSection_);
    semiFixedDirectionCombo_->addItem(QStringLiteral("+360° (positive pan)"), 1);
    semiFixedDirectionCombo_->addItem(QStringLiteral("−360° (negative pan)"), -1);
    semiForm->addRow(QStringLiteral("Direction"), semiFixedDirectionCombo_);

    // Legacy widgets kept for API compatibility; not shown (top Route row loads Semi plans).
    semiFixedPlanRouteCombo_ = new QComboBox(semiFixedSection_);
    semiFixedPlanRouteCombo_->setVisible(false);
    semiFixedLoadPlanBtn_ = new QPushButton(semiFixedSection_);
    semiFixedLoadPlanBtn_->setVisible(false);
    semiFixedRingList_ = new QListWidget(semiFixedSection_);
    semiFixedRingList_->setVisible(false);
    semiFixedAddBtn_ = new QPushButton(semiFixedSection_);
    semiFixedAddBtn_->setVisible(false);
    semiFixedRemoveBtn_ = new QPushButton(semiFixedSection_);
    semiFixedRemoveBtn_->setVisible(false);
    semiFixedSaveBtn_ = new QPushButton(semiFixedSection_);
    semiFixedSaveBtn_->setVisible(false);
    semiFixedLoadFileBtn_ = new QPushButton(semiFixedSection_);
    semiFixedLoadFileBtn_->setVisible(false);

    semiFixedEstimateLabel_ = new QLabel(semiFixedSection_);
    semiFixedEstimateLabel_->setStyleSheet(QStringLiteral("color: #444;"));
    semiForm->addRow(QStringLiteral("Images"), semiFixedEstimateLabel_);

    form->addRow(semiFixedSection_);

    planBtn_ = new QPushButton(QStringLiteral("Plan"), group);
    planBtn_->setToolTip(
        QStringLiteral("Generate the scan grid and check each pose with MoveIt IK + collision."));
    executeBtn_ = new QPushButton(QStringLiteral("Execute"), group);
    executeBtn_->setEnabled(false);

    auto *actionRow = new QWidget(group);
    auto *actionLayout = new QHBoxLayout(actionRow);
    actionLayout->setContentsMargins(0, 0, 0, 0);
    actionLayout->setSpacing(8);
    actionLayout->addWidget(planBtn_, 1);
    actionLayout->addWidget(executeBtn_, 1);
    form->addRow(QStringLiteral(""), actionRow);

    layout->addWidget(group);

    const auto hook = [this]() { onParameterChanged(); };
    connect(sphereRadiusSpin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, hook);
    connect(horizontalPointsSpin_, qOverload<int>(&QSpinBox::valueChanged), this, hook);
    connect(verticalPointsSpin_, qOverload<int>(&QSpinBox::valueChanged), this, hook);
    connect(thetaMinSpin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, hook);
    connect(thetaMaxSpin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, hook);

    const auto wristHook = [this]() { onWristSweepChanged(); };
    connect(wristSweepEnabledCheck_, &QCheckBox::toggled, this, wristHook);
    connect(wristSweepStepSpin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
            wristHook);
    connect(wristSweepStepsSpin_, qOverload<int>(&QSpinBox::valueChanged), this, wristHook);
    connect(wrist1Check_, &QCheckBox::toggled, this, wristHook);
    connect(wrist2Check_, &QCheckBox::toggled, this, wristHook);
    connect(wrist3Check_, &QCheckBox::toggled, this, wristHook);

    connect(planBtn_, &QPushButton::clicked, this, [this]() { emit planScanRequested(); });
    connect(executeBtn_, &QPushButton::clicked, this, [this]() { emit executeScanRequested(); });
    connect(loadRouteBtn_, &QPushButton::clicked, this,
            &Ur3eHemisphereScanSettingsWidget::onLoadRouteClicked);
    connect(routeCombo_, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [this](const int index) {
                loadRouteBtn_->setEnabled(index >= 0
                                          && !routeCombo_->currentData().toString().isEmpty());
                const QString path = routeCombo_->currentData().toString();
                if (path.isEmpty())
                    return;
                if (scanExecuteMode() == hf::ur3e::Ur3eScanExecuteMode::SemiFixed)
                    rememberSemiFixedPlanPath(path);
                else
                    rememberAutoRoutePath(path);
            });

    connect(modeCombo_, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [this](int) { onScanModeChanged(); });
    connect(semiFixedIntervalSpin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
            [this](double) {
                syncSemiFixedRouteFromUi();
                saveToSettings();
                updateImageEstimateLabel();
                emit semiFixedRouteChanged();
            });
    connect(semiFixedDirectionCombo_, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [this](int) {
                syncSemiFixedRouteFromUi();
                saveToSettings();
                emit semiFixedRouteChanged();
            });
    connect(semiFixedAddBtn_, &QPushButton::clicked, this,
            &Ur3eHemisphereScanSettingsWidget::onAddSemiFixedRingClicked);
    connect(semiFixedRemoveBtn_, &QPushButton::clicked, this,
            &Ur3eHemisphereScanSettingsWidget::onRemoveSemiFixedRingClicked);
    connect(semiFixedSaveBtn_, &QPushButton::clicked, this,
            &Ur3eHemisphereScanSettingsWidget::onSaveSemiFixedRouteClicked);
    connect(semiFixedLoadFileBtn_, &QPushButton::clicked, this,
            &Ur3eHemisphereScanSettingsWidget::onLoadSemiFixedRouteFileClicked);
    // Top Route row loads Semi plans; hidden semiFixedPlanRouteCombo_ is unused.

    semiFixedRoute_.id = QStringLiteral("semi_fixed");
    semiFixedRoute_.displayName = QStringLiteral("Semi-fixed");
    semiFixedRoute_.robotCfgFingerprint =
        hf::ur3e::ur3eScanRobotCfgFingerprint(hf::hardwareConfig().ur3e);

    loadFromSettings();
    applyBoundaryLimits(hf::ur3e::workspaceBoundaryFromConfig(hf::hardwareConfig().ur3e));
    syncWristSweepEnabledState();
    syncModeUi();
    updateImageEstimateLabel();
    refreshAvailableRoutes();
    saveToSettings();
}

hf::ur3e::Ur3eScanExecuteMode Ur3eHemisphereScanSettingsWidget::scanExecuteMode() const
{
    if (modeCombo_ == nullptr)
        return hf::ur3e::Ur3eScanExecuteMode::AutoHemisphere;
    const int v = modeCombo_->currentData().toInt();
    return v == static_cast<int>(hf::ur3e::Ur3eScanExecuteMode::SemiFixed)
               ? hf::ur3e::Ur3eScanExecuteMode::SemiFixed
               : hf::ur3e::Ur3eScanExecuteMode::AutoHemisphere;
}

hf::ur3e::Ur3eSemiFixedRoute Ur3eHemisphereScanSettingsWidget::semiFixedRoute() const
{
    hf::ur3e::Ur3eSemiFixedRoute route = semiFixedRoute_;
    if (semiFixedIntervalSpin_ != nullptr)
        route.intervalDeg = semiFixedIntervalSpin_->value();
    if (semiFixedDirectionCombo_ != nullptr)
    {
        const int dir = semiFixedDirectionCombo_->currentData().toInt();
        route.panDirection = dir >= 0 ? 1 : -1;
    }
    if (route.robotCfgFingerprint.isEmpty())
        route.robotCfgFingerprint =
            hf::ur3e::ur3eScanRobotCfgFingerprint(hf::hardwareConfig().ur3e);
    hf::ur3e::ensureSemiFixedTopPose(route);
    return route;
}

void Ur3eHemisphereScanSettingsWidget::setSemiFixedRoute(const hf::ur3e::Ur3eSemiFixedRoute &route)
{
    semiFixedRoute_ = route;
    hf::ur3e::ensureSemiFixedTopPose(semiFixedRoute_);
    if (semiFixedIntervalSpin_ != nullptr)
    {
        const QSignalBlocker b(semiFixedIntervalSpin_);
        semiFixedIntervalSpin_->setValue(route.intervalDeg > 0.0 ? route.intervalDeg : 10.0);
    }
    if (semiFixedDirectionCombo_ != nullptr)
    {
        const QSignalBlocker b(semiFixedDirectionCombo_);
        const int idx = semiFixedDirectionCombo_->findData(route.panDirection >= 0 ? 1 : -1);
        if (idx >= 0)
            semiFixedDirectionCombo_->setCurrentIndex(idx);
    }
    refreshSemiFixedRingList();
    updateImageEstimateLabel();
    saveToSettings();
    emit semiFixedRouteChanged();
}

bool Ur3eHemisphereScanSettingsWidget::semiFixedRouteReady() const
{
    return !semiFixedRoute_.rings.isEmpty();
}

void Ur3eHemisphereScanSettingsWidget::appendSemiFixedRing(
    const std::vector<double> &jointsRad,
    const hf::ur3e::Ur3eScanTcpPose &tcp,
    const bool hasTcp)
{
    if (jointsRad.size() != 6)
        return;
    hf::ur3e::Ur3eSemiFixedRing ring;
    ring.id = QStringLiteral("ring_%1").arg(semiFixedRoute_.rings.size() + 1);
    ring.displayName = QStringLiteral("Ring %1").arg(semiFixedRoute_.rings.size() + 1);
    ring.entryJointsRad = jointsRad;
    ring.entryTcp = tcp;
    ring.hasEntryTcp = hasTcp;
    semiFixedRoute_.rings.push_back(ring);
    refreshSemiFixedRingList();
    updateImageEstimateLabel();
    emit semiFixedRouteChanged();
}

void Ur3eHemisphereScanSettingsWidget::onScanModeChanged()
{
    // Combo already shows the new mode; spins still hold the mode we are leaving.
    PersistedUr3eHemisphereScanSettings saved = AppSettingsStore::loadUr3eHemisphereScan();
    const bool leavingSemi = saved.scanExecuteMode == 1;
    if (leavingSemi)
        saved.semiPanel = captureModePanelFromUi(true);
    else
        saved.autoPanel = captureModePanelFromUi(false);

    saved.scanExecuteMode = static_cast<int>(scanExecuteMode());
    AppSettingsStore::saveUr3eHemisphereScan(saved);

    const bool enteringSemi = scanExecuteMode() == hf::ur3e::Ur3eScanExecuteMode::SemiFixed;
    applyModePanelToUi(enteringSemi ? saved.semiPanel : saved.autoPanel, enteringSemi);

    syncModeUi();
    saveToSettings();
    emit scanModeChanged();
    emit paramsChanged();
}

void Ur3eHemisphereScanSettingsWidget::syncModeUi()
{
    const bool autoMode = scanExecuteMode() == hf::ur3e::Ur3eScanExecuteMode::AutoHemisphere;
    // Shared grid + wrist UI for both modes (Semi remaps V → imaging interval).
    if (autoSection_ != nullptr)
        autoSection_->setVisible(true);
    if (semiFixedSection_ != nullptr)
        semiFixedSection_->setVisible(!autoMode);
    if (verticalPointsSpin_ != nullptr)
        verticalPointsSpin_->setVisible(autoMode);
    if (semiFixedIntervalSpin_ != nullptr)
        semiFixedIntervalSpin_->setVisible(!autoMode);
    if (gridFormLabel_ != nullptr)
    {
        gridFormLabel_->setText(autoMode ? QStringLiteral("Grid")
                                         : QStringLiteral("Layer × interval"));
    }
    if (horizontalPointsSpin_ != nullptr)
    {
        horizontalPointsSpin_->setToolTip(
            autoMode ? QStringLiteral("Horizontal (φ) pin count per latitude.")
                     : QStringLiteral("Number of latitude rings (layers) between theta min–max."));
        if (!autoMode)
            horizontalPointsSpin_->setRange(1, 180);
        else
            horizontalPointsSpin_->setRange(1, 360);
    }
    if (imageEstimateLabel_ != nullptr)
        imageEstimateLabel_->setVisible(autoMode);
    if (semiFixedEstimateLabel_ != nullptr)
        semiFixedEstimateLabel_->setVisible(!autoMode);
    if (routeCombo_ != nullptr)
    {
        routeCombo_->setToolTip(
            autoMode
                ? QStringLiteral("Saved Auto routes in ur3e_scan_routes (cfg must match).")
                : QStringLiteral("Saved Semi plans in mvs_semi_scan_plans (cfg must match)."));
    }
    if (loadRouteBtn_ != nullptr)
    {
        loadRouteBtn_->setToolTip(
            autoMode ? QStringLiteral("Load the selected Auto route.")
                     : QStringLiteral("Load the selected Semi plan → ring entries."));
    }
    if (planBtn_ != nullptr)
    {
        planBtn_->setVisible(true);
        planBtn_->setText(QStringLiteral("Plan"));
        planBtn_->setToolTip(
            autoMode
                ? QStringLiteral(
                      "Generate the scan grid and check each pose with MoveIt IK + collision.")
                : QStringLiteral(
                      "Semi Plan: MoveIt IK + base-link pan-circle check; keep first 3 "
                      "sweep-OK pins per ring. Saves to mvs_semi_scan_plans."));
    }
    refreshAvailableRoutes();
    updateImageEstimateLabel();
}

void Ur3eHemisphereScanSettingsWidget::syncSemiFixedRouteFromUi()
{
    if (semiFixedIntervalSpin_ != nullptr)
        semiFixedRoute_.intervalDeg = semiFixedIntervalSpin_->value();
    if (semiFixedDirectionCombo_ != nullptr)
    {
        const int dir = semiFixedDirectionCombo_->currentData().toInt();
        semiFixedRoute_.panDirection = dir >= 0 ? 1 : -1;
    }
    if (semiFixedRoute_.robotCfgFingerprint.isEmpty())
    {
        semiFixedRoute_.robotCfgFingerprint =
            hf::ur3e::ur3eScanRobotCfgFingerprint(hf::hardwareConfig().ur3e);
    }
}

void Ur3eHemisphereScanSettingsWidget::refreshSemiFixedRingList()
{
    if (semiFixedRingList_ == nullptr)
        return;
    const QSignalBlocker b(semiFixedRingList_);
    semiFixedRingList_->clear();
    for (const hf::ur3e::Ur3eSemiFixedRing &ring : semiFixedRoute_.rings)
    {
        QString line = ring.displayName;
        if (ring.hasEntryTcp)
        {
            line += QStringLiteral("  z=%1 mm r≈%2 mm")
                        .arg(ring.entryTcp.zM * 1000.0, 0, 'f', 0)
                        .arg(hf::ur3e::inferSemiFixedPreviewRing(ring).radiusM * 1000.0, 0, 'f',
                             0);
        }
        semiFixedRingList_->addItem(line);
    }
}

void Ur3eHemisphereScanSettingsWidget::onAddSemiFixedRingClicked()
{
    emit addSemiFixedRingRequested();
}

void Ur3eHemisphereScanSettingsWidget::onRemoveSemiFixedRingClicked()
{
    if (semiFixedRingList_ == nullptr)
        return;
    const int row = semiFixedRingList_->currentRow();
    if (row < 0 || row >= semiFixedRoute_.rings.size())
        return;
    semiFixedRoute_.rings.removeAt(row);
    refreshSemiFixedRingList();
    updateImageEstimateLabel();
    emit semiFixedRouteChanged();
}

void Ur3eHemisphereScanSettingsWidget::onSaveSemiFixedRouteClicked()
{
    syncSemiFixedRouteFromUi();
    if (semiFixedRoute_.rings.isEmpty())
        return;

    bool ok = false;
    const QString name = QInputDialog::getText(
        this,
        QStringLiteral("Save semi-fixed route"),
        QStringLiteral("Route name:"),
        QLineEdit::Normal,
        semiFixedRoute_.displayName,
        &ok);
    if (!ok || name.trimmed().isEmpty())
        return;

    semiFixedRoute_.displayName = name.trimmed();
    QString id = name.trimmed();
    id.replace(QLatin1Char(' '), QLatin1Char('_'));
    semiFixedRoute_.id = id;
    semiFixedRoute_.robotCfgFingerprint =
        hf::ur3e::ur3eScanRobotCfgFingerprint(hf::hardwareConfig().ur3e);

    const QString dir = hf::ur3e::defaultUr3eSemiScanRoutesDir();
    QDir().mkpath(dir);
    const QString path = QDir(dir).filePath(id + QStringLiteral(".json"));
    QString err;
    if (!hf::ur3e::saveUr3eSemiFixedRoute(path, semiFixedRoute_, &err))
        return;
    rememberSemiFixedRoutePath(path);
}

void Ur3eHemisphereScanSettingsWidget::onLoadSemiFixedRouteFileClicked()
{
    const QString dir = hf::ur3e::defaultUr3eSemiScanRoutesDir();
    QDir().mkpath(dir);
    const QString path = QFileDialog::getOpenFileName(
        this,
        QStringLiteral("Load semi-fixed route"),
        dir,
        QStringLiteral("JSON (*.json)"));
    if (path.isEmpty())
        return;

    hf::ur3e::Ur3eSemiFixedRoute route;
    QString err;
    const QString fp = hf::ur3e::ur3eScanRobotCfgFingerprint(hf::hardwareConfig().ur3e);
    if (!hf::ur3e::loadUr3eSemiFixedRoute(path, fp, route, &err))
        return;
    setSemiFixedRoute(route);
    rememberSemiFixedRoutePath(path);
}

void Ur3eHemisphereScanSettingsWidget::onLoadPlannedAsSemiFixedClicked()
{
    if (semiFixedPlanRouteCombo_ == nullptr)
        return;
    const QString path = semiFixedPlanRouteCombo_->currentData().toString();
    if (path.isEmpty())
        return;
    rememberSemiFixedPlanPath(path);
    emit loadPlannedRouteAsSemiFixedRequested(path);
}

void Ur3eHemisphereScanSettingsWidget::applyModePanelToUi(
    const PersistedUr3eScanModePanelSettings &panel,
    const bool semiMode)
{
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
    const QSignalBlocker blockInterval(semiFixedIntervalSpin_);
    const QSignalBlocker blockDir(semiFixedDirectionCombo_);

    if (sphereRadiusSpin_ != nullptr)
        sphereRadiusSpin_->setValue(panel.sphereRadiusMm);
    if (horizontalPointsSpin_ != nullptr)
        horizontalPointsSpin_->setValue(panel.horizontalPoints);
    if (verticalPointsSpin_ != nullptr)
        verticalPointsSpin_->setValue(panel.verticalPoints);
    if (thetaMinSpin_ != nullptr)
        thetaMinSpin_->setValue(panel.thetaMinDeg);
    if (thetaMaxSpin_ != nullptr)
        thetaMaxSpin_->setValue(panel.thetaMaxDeg);
    if (wristSweepEnabledCheck_ != nullptr)
        wristSweepEnabledCheck_->setChecked(panel.wristSweepEnabled);
    if (wristSweepStepSpin_ != nullptr)
        wristSweepStepSpin_->setValue(panel.wristSweepStepDeg);
    if (wristSweepStepsSpin_ != nullptr)
        wristSweepStepsSpin_->setValue(panel.wristSweepStepsEachWay);
    if (wrist1Check_ != nullptr)
        wrist1Check_->setChecked(panel.wristSweepWrist1);
    if (wrist2Check_ != nullptr)
        wrist2Check_->setChecked(panel.wristSweepWrist2);
    if (wrist3Check_ != nullptr)
        wrist3Check_->setChecked(panel.wristSweepWrist3);
    if (semiMode)
    {
        if (semiFixedIntervalSpin_ != nullptr)
            semiFixedIntervalSpin_->setValue(panel.imagingIntervalDeg);
        if (semiFixedDirectionCombo_ != nullptr)
        {
            const int dirIdx = semiFixedDirectionCombo_->findData(panel.panDirection);
            if (dirIdx >= 0)
                semiFixedDirectionCombo_->setCurrentIndex(dirIdx);
        }
        syncSemiFixedRouteFromUi();
    }
    syncWristSweepEnabledState();
}

PersistedUr3eScanModePanelSettings
Ur3eHemisphereScanSettingsWidget::captureModePanelFromUi(const bool /*semiMode*/) const
{
    PersistedUr3eScanModePanelSettings panel;
    panel.sphereRadiusMm = sphereRadiusSpin_ != nullptr ? sphereRadiusSpin_->value() : 500.0;
    panel.horizontalPoints = horizontalPointsSpin_ != nullptr ? horizontalPointsSpin_->value() : 12;
    panel.verticalPoints = verticalPointsSpin_ != nullptr ? verticalPointsSpin_->value() : 5;
    panel.thetaMinDeg = thetaMinSpin_ != nullptr ? thetaMinSpin_->value() : 30.0;
    panel.thetaMaxDeg = thetaMaxSpin_ != nullptr ? thetaMaxSpin_->value() : 90.0;
    const hf::ur3e::Ur3eWristSweepParams wrist = wristSweepParams();
    panel.wristSweepEnabled = wrist.enabled;
    panel.wristSweepStepDeg = wrist.stepDeg;
    panel.wristSweepStepsEachWay = wrist.stepsEachWay;
    panel.wristSweepWrist1 = wrist.wrist1;
    panel.wristSweepWrist2 = wrist.wrist2;
    panel.wristSweepWrist3 = wrist.wrist3;
    panel.imagingIntervalDeg =
        semiFixedIntervalSpin_ != nullptr ? semiFixedIntervalSpin_->value() : 10.0;
    int pan = semiFixedDirectionCombo_ != nullptr ? semiFixedDirectionCombo_->currentData().toInt()
                                                  : 1;
    panel.panDirection = pan >= 0 ? 1 : -1;
    return panel;
}

void Ur3eHemisphereScanSettingsWidget::loadFromSettings()
{
    const PersistedUr3eHemisphereScanSettings saved = AppSettingsStore::loadUr3eHemisphereScan();

    const QSignalBlocker blockMode(modeCombo_);
    const int modeIdx = modeCombo_ != nullptr ? modeCombo_->findData(saved.scanExecuteMode) : -1;
    if (modeIdx >= 0)
        modeCombo_->setCurrentIndex(modeIdx);

    const bool semiMode = scanExecuteMode() == hf::ur3e::Ur3eScanExecuteMode::SemiFixed;
    applyModePanelToUi(semiMode ? saved.semiPanel : saved.autoPanel, semiMode);
}

void Ur3eHemisphereScanSettingsWidget::saveToSettings() const
{
    PersistedUr3eHemisphereScanSettings saved = AppSettingsStore::loadUr3eHemisphereScan();
    saved.scanExecuteMode = static_cast<int>(scanExecuteMode());
    const bool semiMode = scanExecuteMode() == hf::ur3e::Ur3eScanExecuteMode::SemiFixed;
    const PersistedUr3eScanModePanelSettings panel = captureModePanelFromUi(semiMode);
    if (semiMode)
    {
        saved.semiPanel = panel;
    }
    else
    {
        // Keep Semi interval/pan when Auto is active (interval spin may be hidden).
        const double keepInterval = saved.semiPanel.imagingIntervalDeg;
        const int keepPan = saved.semiPanel.panDirection;
        saved.autoPanel = panel;
        saved.autoPanel.imagingIntervalDeg = keepInterval;
        saved.autoPanel.panDirection = keepPan;
        saved.semiPanel.imagingIntervalDeg = keepInterval;
        saved.semiPanel.panDirection = keepPan;
    }
    if (routeCombo_ != nullptr)
    {
        const QString path = routeCombo_->currentData().toString();
        if (!path.isEmpty())
        {
            if (semiMode)
                saved.lastSemiFixedPlanPath = path;
            else
                saved.lastAutoRoutePath = path;
        }
    }
    AppSettingsStore::saveUr3eHemisphereScan(saved);
}

void Ur3eHemisphereScanSettingsWidget::rememberAutoRoutePath(const QString &path)
{
    if (path.trimmed().isEmpty())
        return;
    PersistedUr3eHemisphereScanSettings saved = AppSettingsStore::loadUr3eHemisphereScan();
    saved.lastAutoRoutePath = path;
    AppSettingsStore::saveUr3eHemisphereScan(saved);
}

void Ur3eHemisphereScanSettingsWidget::rememberSemiFixedPlanPath(const QString &path)
{
    if (path.trimmed().isEmpty())
        return;
    PersistedUr3eHemisphereScanSettings saved = AppSettingsStore::loadUr3eHemisphereScan();
    saved.lastSemiFixedPlanPath = path;
    AppSettingsStore::saveUr3eHemisphereScan(saved);
}

void Ur3eHemisphereScanSettingsWidget::rememberSemiFixedRoutePath(const QString &path)
{
    if (path.trimmed().isEmpty())
        return;
    PersistedUr3eHemisphereScanSettings saved = AppSettingsStore::loadUr3eHemisphereScan();
    saved.lastSemiFixedRoutePath = path;
    AppSettingsStore::saveUr3eHemisphereScan(saved);
}

QString Ur3eHemisphereScanSettingsWidget::rememberedAutoRoutePath() const
{
    return AppSettingsStore::loadUr3eHemisphereScan().lastAutoRoutePath;
}

QString Ur3eHemisphereScanSettingsWidget::rememberedSemiFixedPlanPath() const
{
    return AppSettingsStore::loadUr3eHemisphereScan().lastSemiFixedPlanPath;
}

QString Ur3eHemisphereScanSettingsWidget::rememberedSemiFixedRoutePath() const
{
    return AppSettingsStore::loadUr3eHemisphereScan().lastSemiFixedRoutePath;
}

void Ur3eHemisphereScanSettingsWidget::applyBoundaryLimits(
    const hf::ur3e::Ur3eWorkspaceBoundary &boundary)
{
    boundaryLimits_ = boundary;
    boundaryLimits_.normalize();

    sphereRadiusSpin_->setMaximum(kGuiSphereRadiusMaxMm);
    if (sphereRadiusSpin_->value() > kGuiSphereRadiusMaxMm)
        sphereRadiusSpin_->setValue(kGuiSphereRadiusMaxMm);

    if (boundaryLimits_.enabled)
    {
        const double fitMm = hf::ur3e::maxHemisphereRadiusM(boundaryLimits_) * 1000.0;
        double centerXM = 0.0;
        double centerYM = 0.0;
        hf::ur3e::scanCenterOffsetM(centerXM, centerYM);
        sphereRadiusSpin_->setToolTip(
            QStringLiteral(
                "Allowed up to %1 mm. Equator-fit workspace limit is %2 mm "
                "(%3×%4 mm box, scan center %5, %6 mm).")
                .arg(kGuiSphereRadiusMaxMm, 0, 'f', 0)
                .arg(fitMm, 0, 'f', 0)
                .arg(static_cast<int>(boundaryLimits_.lengthMm))
                .arg(static_cast<int>(boundaryLimits_.widthMm))
                .arg(centerXM * 1000.0, 0, 'f', 0)
                .arg(centerYM * 1000.0, 0, 'f', 0));
    }
    else
    {
        sphereRadiusSpin_->setToolTip(
            QStringLiteral("Workspace limits disabled in hyperfusion.cfg. Max %1 mm.")
                .arg(kGuiSphereRadiusMaxMm, 0, 'f', 0));
    }
}

hf::ur3e::Ur3eHemisphereScanParams Ur3eHemisphereScanSettingsWidget::params() const
{
    hf::ur3e::Ur3eHemisphereScanParams scanParams;
    scanParams.sphereRadiusM = sphereRadiusSpin_->value() * 0.001;
    scanParams.horizontalPoints = horizontalPointsSpin_->value();
    scanParams.verticalPoints = verticalPointsSpin_->value();
    scanParams.thetaMinDeg = thetaMinSpin_->value();
    scanParams.thetaMaxDeg = thetaMaxSpin_->value();
    hf::ur3e::normalizeHemisphereScanParams(scanParams);
    scanParams.sphereRadiusM =
        std::min(scanParams.sphereRadiusM, kGuiSphereRadiusMaxMm * 0.001);
    return scanParams;
}

hf::ur3e::Ur3eHemisphereScanParams Ur3eHemisphereScanSettingsWidget::semiPlanParams() const
{
    hf::ur3e::Ur3eHemisphereScanParams scanParams = params();
    // Semi UI: first spin = latitude layers (rings); interval is imaging step only.
    // φ candidates for base-sweep search stay dense enough to find ≤3 OK pins.
    scanParams.verticalPoints = std::max(1, horizontalPointsSpin_->value());
    scanParams.horizontalPoints = 12;
    hf::ur3e::normalizeHemisphereScanParams(scanParams);
    scanParams.sphereRadiusM =
        std::min(scanParams.sphereRadiusM, kGuiSphereRadiusMaxMm * 0.001);
    return scanParams;
}

hf::ur3e::Ur3eWristSweepParams Ur3eHemisphereScanSettingsWidget::wristSweepParams() const
{
    hf::ur3e::Ur3eWristSweepParams wrist;
    wrist.enabled = wristSweepEnabledCheck_ != nullptr && wristSweepEnabledCheck_->isChecked();
    wrist.stepDeg = wristSweepStepSpin_ != nullptr ? wristSweepStepSpin_->value() : 3.0;
    wrist.stepsEachWay = wristSweepStepsSpin_ != nullptr ? wristSweepStepsSpin_->value() : 1;
    wrist.wrist1 = wrist1Check_ != nullptr && wrist1Check_->isChecked();
    wrist.wrist2 = wrist2Check_ != nullptr && wrist2Check_->isChecked();
    wrist.wrist3 = wrist3Check_ != nullptr && wrist3Check_->isChecked();
    return wrist;
}

bool Ur3eHemisphereScanSettingsWidget::rememberLastPlan() const
{
    return hf::hardwareConfig().ur3e.rememberLastScanPlan;
}

void Ur3eHemisphereScanSettingsWidget::setParams(const hf::ur3e::Ur3eHemisphereScanParams &params)
{
    hf::ur3e::Ur3eHemisphereScanParams normalized = params;
    hf::ur3e::normalizeHemisphereScanParams(normalized);
    normalized.sphereRadiusM =
        std::min(normalized.sphereRadiusM, kGuiSphereRadiusMaxMm * 0.001);

    const QSignalBlocker blockRadius(sphereRadiusSpin_);
    const QSignalBlocker blockHorizontal(horizontalPointsSpin_);
    const QSignalBlocker blockVertical(verticalPointsSpin_);
    const QSignalBlocker blockThetaMin(thetaMinSpin_);
    const QSignalBlocker blockThetaMax(thetaMaxSpin_);

    const bool semiMode = scanExecuteMode() == hf::ur3e::Ur3eScanExecuteMode::SemiFixed;

    if (sphereRadiusSpin_ != nullptr)
        sphereRadiusSpin_->setValue(normalized.sphereRadiusM * 1000.0);
    if (horizontalPointsSpin_ != nullptr)
    {
        // Semi Layer spin maps to verticalPoints in the saved plan (φ candidates stay 12).
        if (semiMode)
            horizontalPointsSpin_->setValue(std::max(1, normalized.verticalPoints));
        else
            horizontalPointsSpin_->setValue(normalized.horizontalPoints);
    }
    if (verticalPointsSpin_ != nullptr)
        verticalPointsSpin_->setValue(normalized.verticalPoints);
    if (thetaMinSpin_ != nullptr)
        thetaMinSpin_->setValue(normalized.thetaMinDeg);
    if (thetaMaxSpin_ != nullptr)
        thetaMaxSpin_->setValue(normalized.thetaMaxDeg);

    plannedReachablePins_ = -1;
    updateImageEstimateLabel();
    saveToSettings();
}

void Ur3eHemisphereScanSettingsWidget::applyLoadedSemiPlanSettings(
    const hf::ur3e::Ur3eHemisphereScanParams &params,
    const double intervalDeg,
    const int panDirection)
{
    // setParams maps Semi Layer ← verticalPoints and writes θ / radius.
    setParams(params);

    const QSignalBlocker blockInterval(semiFixedIntervalSpin_);
    const QSignalBlocker blockDir(semiFixedDirectionCombo_);
    if (semiFixedIntervalSpin_ != nullptr)
        semiFixedIntervalSpin_->setValue(intervalDeg > 0.0 ? intervalDeg : 10.0);
    if (semiFixedDirectionCombo_ != nullptr)
    {
        const int idx = semiFixedDirectionCombo_->findData(panDirection >= 0 ? 1 : -1);
        if (idx >= 0)
            semiFixedDirectionCombo_->setCurrentIndex(idx);
    }
    semiFixedRoute_.intervalDeg =
        semiFixedIntervalSpin_ != nullptr ? semiFixedIntervalSpin_->value() : 10.0;
    semiFixedRoute_.panDirection = panDirection >= 0 ? 1 : -1;
    updateImageEstimateLabel();
    saveToSettings();
}

void Ur3eHemisphereScanSettingsWidget::setPlanEnabled(const bool enabled)
{
    if (planBtn_ != nullptr)
        planBtn_->setEnabled(enabled);
}

void Ur3eHemisphereScanSettingsWidget::setExecuteEnabled(const bool enabled)
{
    if (executeBtn_ != nullptr)
        executeBtn_->setEnabled(enabled);
}

void Ur3eHemisphereScanSettingsWidget::setLoadRouteEnabled(const bool enabled)
{
    if (routeCombo_ != nullptr)
        routeCombo_->setEnabled(enabled);
    if (loadRouteBtn_ != nullptr)
    {
        const bool hasRoute = routeCombo_ != nullptr && routeCombo_->currentIndex() >= 0
                              && !routeCombo_->currentData().toString().isEmpty();
        loadRouteBtn_->setEnabled(enabled && hasRoute);
    }
}

void Ur3eHemisphereScanSettingsWidget::refreshAvailableRoutes()
{
    if (routeCombo_ == nullptr)
        return;

    const bool semiMode = scanExecuteMode() == hf::ur3e::Ur3eScanExecuteMode::SemiFixed;
    const PersistedUr3eHemisphereScanSettings saved = AppSettingsStore::loadUr3eHemisphereScan();
    const QString preferred =
        semiMode
            ? (!saved.lastSemiFixedPlanPath.isEmpty() ? saved.lastSemiFixedPlanPath
                                                      : routeCombo_->currentData().toString())
            : (!saved.lastAutoRoutePath.isEmpty() ? saved.lastAutoRoutePath
                                                  : routeCombo_->currentData().toString());

    const QSignalBlocker block(routeCombo_);
    routeCombo_->clear();

    const QString robotFp = hf::ur3e::ur3eScanRobotCfgFingerprint(hf::hardwareConfig().ur3e);
    const QString dir = semiMode ? hf::ur3e::defaultUr3eSemiScanRoutesDir()
                                 : hf::ur3e::defaultUr3eScanRoutesDir();
    const auto routes = hf::ur3e::listUr3eScanRoutesMatchingCfg(dir, robotFp);

    if (routes.isEmpty())
    {
        routeCombo_->addItem(semiMode ? QStringLiteral("(no matching Semi plans)")
                                      : QStringLiteral("(no matching routes)"),
                             QString());
        if (loadRouteBtn_ != nullptr)
            loadRouteBtn_->setEnabled(false);
        return;
    }

    int selectIndex = 0;
    for (int i = 0; i < routes.size(); ++i)
    {
        const hf::ur3e::Ur3eScanRouteInfo &route = routes[i];
        const QString label = route.displayName;
        const QString tip = QStringLiteral("%1\n%2").arg(route.displayName).arg(route.path);
        routeCombo_->addItem(label, route.path);
        routeCombo_->setItemData(routeCombo_->count() - 1, tip, Qt::ToolTipRole);
        if (!preferred.isEmpty() && route.path == preferred)
            selectIndex = i;
    }
    routeCombo_->setCurrentIndex(selectIndex);
    if (loadRouteBtn_ != nullptr)
        loadRouteBtn_->setEnabled(routeCombo_->isEnabled()
                                  && !routeCombo_->currentData().toString().isEmpty());
}

void Ur3eHemisphereScanSettingsWidget::onLoadRouteClicked()
{
    if (routeCombo_ == nullptr)
        return;
    const QString path = routeCombo_->currentData().toString();
    if (path.isEmpty())
        return;
    if (scanExecuteMode() == hf::ur3e::Ur3eScanExecuteMode::SemiFixed)
    {
        rememberSemiFixedPlanPath(path);
        emit loadPlannedRouteAsSemiFixedRequested(path);
    }
    else
    {
        rememberAutoRoutePath(path);
        emit loadScanRouteRequested(path);
    }
}

void Ur3eHemisphereScanSettingsWidget::setParamsEnabled(const bool enabled)
{
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
    if (modeCombo_ != nullptr)
        modeCombo_->setEnabled(enabled);
    if (semiFixedIntervalSpin_ != nullptr)
        semiFixedIntervalSpin_->setEnabled(enabled);
    if (semiFixedDirectionCombo_ != nullptr)
        semiFixedDirectionCombo_->setEnabled(enabled);
    if (semiFixedAddBtn_ != nullptr)
        semiFixedAddBtn_->setEnabled(enabled);
    if (semiFixedRemoveBtn_ != nullptr)
        semiFixedRemoveBtn_->setEnabled(enabled);
    if (semiFixedSaveBtn_ != nullptr)
        semiFixedSaveBtn_->setEnabled(enabled);
    if (semiFixedLoadFileBtn_ != nullptr)
        semiFixedLoadFileBtn_->setEnabled(enabled);
    if (semiFixedPlanRouteCombo_ != nullptr)
        semiFixedPlanRouteCombo_->setEnabled(enabled);
    if (semiFixedLoadPlanBtn_ != nullptr)
    {
        const bool hasPlan = semiFixedPlanRouteCombo_ != nullptr
                             && !semiFixedPlanRouteCombo_->currentData().toString().isEmpty();
        semiFixedLoadPlanBtn_->setEnabled(enabled && hasPlan);
    }
    setLoadRouteEnabled(enabled);
    syncWristSweepEnabledState();
    if (!enabled)
    {
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
    if (scanExecuteMode() == hf::ur3e::Ur3eScanExecuteMode::SemiFixed)
    {
        if (semiFixedEstimateLabel_ == nullptr)
            return;
        syncSemiFixedRouteFromUi();
        const hf::ur3e::Ur3eWristSweepParams wrist = wristSweepParams();
        const int perSample = wrist.imagesPerPin();
        const int layers = horizontalPointsSpin_ != nullptr ? horizontalPointsSpin_->value() : 1;
        const int rings =
            semiFixedRoute_.rings.isEmpty() ? std::max(1, layers) : semiFixedRoute_.rings.size();
        const int perRing = hf::ur3e::semiFixedSampleCount(semiFixedRoute_.intervalDeg);
        const qint64 total =
            static_cast<qint64>(perSample)
            * (1 + static_cast<qint64>(rings) * static_cast<qint64>(perRing));
        const QString ringSource = semiFixedRoute_.rings.isEmpty()
                                       ? QStringLiteral("%1 layers").arg(rings)
                                       : QStringLiteral("%1 planned").arg(rings);
        semiFixedEstimateLabel_->setText(
            QStringLiteral("%1/pose × (1 top + %2/ring × %3) = %4")
                .arg(perSample)
                .arg(perRing)
                .arg(ringSource)
                .arg(total));
        return;
    }

    if (imageEstimateLabel_ == nullptr)
        return;

    const hf::ur3e::Ur3eWristSweepParams wrist = wristSweepParams();
    const int perPin = wrist.imagesPerPin();
    const int gridPins = hf::ur3e::hemisphereScanPointCount(params());
    const int pinCount = plannedReachablePins_ >= 0 ? plannedReachablePins_ : gridPins;
    const qint64 total = static_cast<qint64>(perPin) * static_cast<qint64>(pinCount);
    const QString pinSource = plannedReachablePins_ >= 0
                                  ? QStringLiteral("%1 ok").arg(pinCount)
                                  : QStringLiteral("%1 grid").arg(pinCount);
    imageEstimateLabel_->setText(
        QStringLiteral("%1/pin x %2 = %3").arg(perPin).arg(pinSource).arg(total));
}

void Ur3eHemisphereScanSettingsWidget::onWristSweepChanged()
{
    syncWristSweepEnabledState();
    saveToSettings();
    updateImageEstimateLabel();
}

void Ur3eHemisphereScanSettingsWidget::onParameterChanged()
{
    if (thetaMinSpin_->value() > thetaMaxSpin_->value())
        thetaMaxSpin_->setValue(thetaMinSpin_->value());

    if (sphereRadiusSpin_->value() > kGuiSphereRadiusMaxMm)
        sphereRadiusSpin_->setValue(kGuiSphereRadiusMaxMm);

    plannedReachablePins_ = -1;
    saveToSettings();
    updateImageEstimateLabel();
    emit paramsChanged();
}
} // namespace ui
