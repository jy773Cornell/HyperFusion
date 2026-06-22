// Stage settings tab (Zaber connection, device info, motion controls).
// MainWindow method definitions extracted from MainWindow.cpp for clarity.
#include "frontend/controllers/StagePanelController.hpp"
#include "frontend/widgets/MainWindow.hpp"
#include "adapters/zaber/ZaberStageProfile.hpp"
#include "backend/HyperFusionConfig.hpp"
#include "backend/StageWorker.hpp"
#include "frontend/widgets/MainWindowTabHelpers.hpp"
#include "frontend/widgets/OperationWaitDialog.hpp"
#include "frontend/widgets/StageAxisWidget.hpp"
#include "frontend/utils/SerialPortEnumerator.hpp"

#include <QApplication>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QEventLoop>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QTimer>
#include <QStyle>
#include <QToolButton>
#include <QVBoxLayout>
QWidget *MainWindow::createStageSettingsTab()
{
    auto *page = new QWidget(this);
    auto *layout = new QVBoxLayout(page);

    auto *connBox = new QGroupBox("Connection", page);
    auto *connForm = new QFormLayout(connBox);
    stagePortCombo_ = new QComboBox(connBox);
    stagePortCombo_->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    stagePortCombo_->setMinimumContentsLength(18);

    auto *refreshPortsBtn = new QPushButton("Refresh", connBox);
    refreshPortsBtn->setToolTip("Rescan available COM ports");
    auto *portRow = new QWidget(connBox);
    auto *portRowLayout = new QHBoxLayout(portRow);
    portRowLayout->setContentsMargins(0, 0, 0, 0);
    portRowLayout->addWidget(stagePortCombo_, 1);
    portRowLayout->addWidget(refreshPortsBtn);

    stageBaudCombo_ = new QComboBox(connBox);
    stageBaudCombo_->addItems({"9600", "19200", "38400", "57600", "115200"});
    stageBaudCombo_->setCurrentText("115200");
    stageConnectBtn_ = new QPushButton("Connect", connBox);
    stageDisconnectBtn_ = new QPushButton("Disconnect", connBox);
    stageDisconnectBtn_->setEnabled(false);
    connForm->addRow("Port", portRow);
    connForm->addRow("Baud", stageBaudCombo_);
    connForm->addRow("", stageConnectBtn_);
    connForm->addRow("", stageDisconnectBtn_);

    stagePanel()->refreshComPortList();

    connect(refreshPortsBtn, &QPushButton::clicked, this, [this]() {
        stagePanel()->refreshComPortList();
    });
    connect(stageConnectBtn_, &QPushButton::clicked, this, [this]() {
        if (stageWorker() == nullptr)
            return;

        const QString portName = stagePanel()->selectedPortName();
        if (portName.isEmpty() || portName.startsWith('('))
        {
            appendLog("Stage: connect requested but no COM port is selected");
            return;
        }

        StageConnectSettings settings;
        settings.portName = portName.toStdString();
        settings.baudRate = stageBaudCombo_->currentText().toInt();
        settings.motionAccelerationMmPerSec2 = hf::hardwareConfig().stageMotionAccelerationMmPerSec2;

        appendLog(QString("Stage: connecting to %1 @ %2...")
                      .arg(portName, stageBaudCombo_->currentText()));
        stageHomingKind_ = StageHomingKind::Localization;
        stageWorker()->requestConnect(settings);
    });
    connect(stageDisconnectBtn_, &QPushButton::clicked, this, [this]() {
        if (stageWorker() == nullptr)
            return;
        if (!isStageSessionActive())
            return;

        appendLog("Stage: homing before disconnect\u2026");
        OperationWaitDialog waitDialog(this);
        waitDialog.setStatusText(tr("Homing stage before disconnect\u2026"));
        waitDialog.show();
        QApplication::processEvents();

        stageHomingKind_ = StageHomingKind::BeforeDisconnect;
        std::atomic<bool> done{false};
        stageWorker()->requestDisconnectWithHoming([&done]() { done.store(true, std::memory_order_release); });
        while (!done.load(std::memory_order_acquire))
        {
            waitDialog.raise();
            QApplication::processEvents(QEventLoop::AllEvents, 40);
        }
        stageHomingKind_ = StageHomingKind::None;
        appendLog("Stage: disconnected");
    });

    auto *deviceBox = new QGroupBox("Device", page);
    auto *deviceLayout = new QVBoxLayout(deviceBox);
    auto *deviceHint = new QLabel(
        QStringLiteral("Use Zaber Launcher to initialize or reconfigure the motors."),
        deviceBox);
    deviceHint->setWordWrap(true);
    stageDeviceDisplay_ = new QPlainTextEdit(deviceBox);
    stageDeviceDisplay_->setReadOnly(true);
    stageDeviceDisplay_->setPlaceholderText("Connect to discover the controller and axes.");
    stageDeviceDisplay_->setMinimumHeight(55);
    stageDeviceDisplay_->setMaximumHeight(75);
    deviceLayout->addWidget(deviceHint);
    deviceLayout->addWidget(stageDeviceDisplay_);

    stageControlBox_ = new QGroupBox("Control", page);
    stageControlBox_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Maximum);
    auto *controlLayout = new QVBoxLayout(stageControlBox_);
    controlLayout->setContentsMargins(6, 2, 6, 6);
    controlLayout->setSpacing(2);

    stageAxisWidget_ = new ui::StageAxisWidget(stageControlBox_);
    stageAxisWidget_->setTravelRangeMm(zaber_stage::kTravelMinimumMm, zaber_stage::kTravelLengthMm);

    auto *toolbarRow = new QHBoxLayout();
    toolbarRow->setSpacing(10);
    toolbarRow->setContentsMargins(0, 0, 0, 0);
    stageHomeBtn_ = new QToolButton(stageControlBox_);
    stageHomeBtn_->setIcon(ui::makeHomeIcon());
    stageHomeBtn_->setIconSize(QSize(28, 28));
    stageHomeBtn_->setMinimumSize(44, 44);
    stageHomeBtn_->setToolTip(tr("Perform homing"));
    stageHomeBtn_->setAutoRaise(true);
    stageToStartBtn_ =
        ui::makeStageToolButton(stageControlBox_, QStyle::SP_MediaSkipBackward, tr("Move to beginning (0 mm)"));
    stageBackBtn_ = ui::makeStageToolButton(stageControlBox_, QStyle::SP_MediaSeekBackward, tr("Move back (hold)"));
    stageStopBtn_ = ui::makeStageToolButton(stageControlBox_, QStyle::SP_MediaStop, tr("Stop"));
    stageForwardBtn_ =
        ui::makeStageToolButton(stageControlBox_, QStyle::SP_MediaSeekForward, tr("Move forward (hold)"));
    stageToEndBtn_ = ui::makeStageToolButton(stageControlBox_,
                                         QStyle::SP_MediaSkipForward,
                                         tr("Move to end (%1 mm)").arg(zaber_stage::kTravelLengthMm, 0, 'f', 0));

    toolbarRow->addWidget(stageHomeBtn_);
    toolbarRow->addWidget(stageToStartBtn_);
    toolbarRow->addSpacing(8);
    toolbarRow->addWidget(stageBackBtn_);
    toolbarRow->addWidget(stageStopBtn_);
    toolbarRow->addWidget(stageForwardBtn_);
    toolbarRow->addSpacing(8);
    toolbarRow->addWidget(stageToEndBtn_);
    toolbarRow->addStretch(1);

    auto *absoluteRow = new QHBoxLayout();
    auto *absoluteLabel = new QLabel(QStringLiteral("Move to absolute position"), stageControlBox_);
    stageAbsolutePositionSpin_ = new QDoubleSpinBox(stageControlBox_);
    stageAbsolutePositionSpin_->setRange(zaber_stage::kTravelMinimumMm, zaber_stage::kTravelLengthMm);
    stageAbsolutePositionSpin_->setDecimals(1);
    stageAbsolutePositionSpin_->setSingleStep(1.0);
    stageAbsolutePositionSpin_->setValue(0.0);
    stageAbsolutePositionSpin_->setEnabled(false);
    stageAbsoluteMoveBtn_ =
        ui::makeStageToolButton(stageControlBox_, QStyle::SP_MediaPlay, tr("Move to absolute position"));
    absoluteRow->addWidget(absoluteLabel);
    absoluteRow->addWidget(stageAbsolutePositionSpin_);
    absoluteRow->addWidget(stageAbsoluteMoveBtn_);

    controlLayout->addLayout(toolbarRow);
    controlLayout->addWidget(stageAxisWidget_);
    stagePanel()->updatePositionDisplay(0.0);
    controlLayout->addLayout(absoluteRow);

    stagePositionTimer_ = new QTimer(this);
    stagePositionTimer_->setInterval(100);

    connect(stageHomeBtn_, &QToolButton::clicked, this, [this]() {
        if (stageWorker() == nullptr)
            return;
        stageHomingKind_ = StageHomingKind::Simple;
        stageWorker()->requestHome();
    });
    connect(stageToStartBtn_, &QToolButton::clicked, this, [this]() {
        if (stageWorker() == nullptr)
            return;
        stageWorker()->requestMoveAbsoluteMm(zaber_stage::kTravelMinimumMm, zaber_stage::kMaxSpeedMmPerSec);
    });
    connect(stageBackBtn_, &QToolButton::pressed, this, [this]() {
        if (stageWorker() == nullptr)
            return;
        if (stagePanel() != nullptr)
            stagePanel()->onManualMotionStarted();
        stageWorker()->requestMoveVelocityMm(-zaber_stage::kMaxSpeedMmPerSec);
    });
    connect(stageBackBtn_, &QToolButton::released, this, [this]() {
        if (stageWorker() == nullptr)
            return;
        stageWorker()->requestStopMotion();
        if (stagePanel() != nullptr)
            stagePanel()->onManualMotionStopped();
    });
    connect(stageStopBtn_, &QToolButton::clicked, this, [this]() {
        if (stageWorker() == nullptr)
            return;
        appendLog("Stage: stop requested");
        stageWorker()->requestStopMotion();
    });
    connect(stageForwardBtn_, &QToolButton::pressed, this, [this]() {
        if (stageWorker() == nullptr)
            return;
        if (stagePanel() != nullptr)
            stagePanel()->onManualMotionStarted();
        stageWorker()->requestMoveVelocityMm(zaber_stage::kMaxSpeedMmPerSec);
    });
    connect(stageForwardBtn_, &QToolButton::released, this, [this]() {
        if (stageWorker() == nullptr)
            return;
        stageWorker()->requestStopMotion();
        if (stagePanel() != nullptr)
            stagePanel()->onManualMotionStopped();
    });
    connect(stageToEndBtn_, &QToolButton::clicked, this, [this]() {
        if (stageWorker() == nullptr)
            return;
        stageWorker()->requestMoveAbsoluteMm(zaber_stage::kTravelLengthMm, zaber_stage::kMaxSpeedMmPerSec);
    });
    connect(stageAbsoluteMoveBtn_, &QToolButton::clicked, this, [this]() {
        if (stageWorker() == nullptr || stageAbsolutePositionSpin_ == nullptr)
            return;

        const double targetMm = stageAbsolutePositionSpin_->value();
        appendLog(QString("Stage: move to absolute position %1 mm @ %2 mm/s")
                      .arg(targetMm, 0, 'f', 1)
                      .arg(zaber_stage::kMaxSpeedMmPerSec, 0, 'f', 0));
        stageWorker()->requestMoveAbsoluteMm(targetMm, zaber_stage::kMaxSpeedMmPerSec);
    });

    layout->addWidget(connBox);
    layout->addWidget(deviceBox);
    layout->addWidget(stageControlBox_);
    layout->addStretch(1);
    return page;
}
