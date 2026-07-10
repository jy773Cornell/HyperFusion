// Stage tab orchestration implementation.
#include "frontend/controllers/StagePanelController.hpp"

#include "adapters/zaber/ZaberStageController.hpp"
#include "backend/camera/CameraTypes.hpp"
#include "backend/stage/StageWorker.hpp"
#include "frontend/widgets/MainWindow.hpp"
#include "frontend/utils/SerialPortEnumerator.hpp"
#include "frontend/widgets/StageAxisWidget.hpp"

#include <QComboBox>
#include <QMetaObject>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSignalBlocker>

#include <cmath>
#include <QTimer>
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QToolButton>

#include <cmath>
#include <memory>
#include <algorithm>

namespace {

QString formatStageTopology(const StageTopology &topology)
{
    if (topology.devices.empty())
        return QStringLiteral("Not connected.");

    QString text;
    if (!topology.portName.empty())
    {
        text += QStringLiteral("Port: %1 @ %2\n\n")
                    .arg(QString::fromStdString(topology.portName))
                    .arg(topology.baudRate);
    }

    for (const StageDeviceInfo &device : topology.devices)
    {
        text += QStringLiteral("Device %1: %2\n")
                    .arg(device.deviceAddress)
                    .arg(QString::fromStdString(device.name));
        text += QStringLiteral("  Serial: %1\n").arg(device.serialNumber);
        text += QStringLiteral("  Firmware: %1\n")
                    .arg(QString::fromStdString(device.firmwareVersion));
        text += QStringLiteral("  Axes: %1\n").arg(device.axisCount);

        for (const StageAxisInfo &axis : device.axes)
        {
            text += QStringLiteral("    Axis %1: %2")
                        .arg(axis.axisNumber)
                        .arg(QString::fromStdString(axis.peripheralName));
            if (axis.peripheralSerialNumber != 0)
                text += QStringLiteral(" (SN %1)").arg(axis.peripheralSerialNumber);
            if (axis.axisNumber == 1)
                text += QStringLiteral(" \u2014 right");
            else if (axis.axisNumber == 2)
                text += QStringLiteral(" \u2014 left");
            text += QLatin1Char('\n');
        }

        text += QLatin1Char('\n');
    }

    if (!topology.stageType.empty() || topology.travelLengthMm > 0.0 || topology.lockstepEnabled)
    {
        text += QStringLiteral("Configuration\n");
        if (!topology.stageType.empty())
            text += QStringLiteral("  Stage: %1\n").arg(QString::fromStdString(topology.stageType));
        if (topology.travelLengthMm > 0.0)
            text += QStringLiteral("  Travel limit: %1 mm\n").arg(topology.travelLengthMm, 0, 'f', 0);
        if (topology.maxSpeedMmPerSec > 0.0)
            text += QStringLiteral("  Max speed: %1 mm/s\n").arg(topology.maxSpeedMmPerSec, 0, 'f', 0);
        if (topology.lockstepEnabled)
        {
            text += QStringLiteral("  Lockstep group %1: axis %2 (primary), axis %3\n")
                        .arg(topology.lockstepGroupId)
                        .arg(topology.lockstepPrimaryAxis)
                        .arg(topology.lockstepSecondaryAxis);
            text += QStringLiteral("  Lockstep offset (axis %1): %2 mm\n")
                        .arg(topology.lockstepSecondaryAxis)
                        .arg(topology.lockstepSecondaryOffsetMm, 0, 'f', 0);
            text += QStringLiteral("  Motion: command primary axis %1 only\n")
                        .arg(topology.lockstepPrimaryAxis);
        }
        if (topology.axesHomed)
            text += QStringLiteral("  Homing: lockstep home sensor\n");
    }

    return text.trimmed();
}

} // namespace

namespace hf::stage {

StagePanelController::StagePanelController(MainWindow *host, QObject *parent)
    : QObject(parent)
    , host_(host)
{
}

StageWorker *StagePanelController::worker() const { return stageWorker_.get(); }

bool StagePanelController::isSessionActive() const
{
    return stageWorker_ != nullptr && stageWorker_->currentState() == StageState::Connected;
}

void StagePanelController::shutdownSync(bool homeBeforeDisconnect)
{
    if (stageWorker_ == nullptr)
        return;
    if (homeBeforeDisconnect)
        host_->stageHomingKind_ = MainWindow::StageHomingKind::BeforeDisconnect;
    stageWorker_->shutdownSync(homeBeforeDisconnect);
    host_->stageHomingKind_ = MainWindow::StageHomingKind::None;
}

void StagePanelController::wireSettingsTabConnections()
{
    if (host_->stagePositionTimer_ == nullptr)
        return;
    connect(host_->stagePositionTimer_, &QTimer::timeout, this, [this]() { pollPosition(); });
}

void StagePanelController::refreshComPortList()
{
    if (host_->stagePortCombo_ == nullptr)
        return;

    QString previousPort = host_->stagePortCombo_->currentData().toString();
    if (previousPort.isEmpty())
        previousPort = host_->stagePortCombo_->currentText();
    if (previousPort.isEmpty() || previousPort.startsWith(QLatin1Char('(')))
        previousPort = host_->persistedStagePort_;

    QSignalBlocker blocker(host_->stagePortCombo_);
    host_->stagePortCombo_->clear();

    const QStringList ports = ui::enumerateSerialPortNames();
    const QString preferredPort = QStringLiteral("COM4");
    int selectIndex = -1;

    for (const QString &portName : ports)
    {
        host_->stagePortCombo_->addItem(portName, portName);

        if (portName.compare(previousPort, Qt::CaseInsensitive) == 0)
            selectIndex = host_->stagePortCombo_->count() - 1;
        else if (selectIndex < 0 && portName.compare(preferredPort, Qt::CaseInsensitive) == 0)
            selectIndex = host_->stagePortCombo_->count() - 1;
    }

    if (host_->stagePortCombo_->count() == 0)
    {
        host_->stagePortCombo_->addItem(QStringLiteral("(no serial ports found)"), QString());
        host_->stagePortCombo_->setEnabled(false);
        return;
    }

    host_->stagePortCombo_->setEnabled(true);
    if (selectIndex >= 0)
        host_->stagePortCombo_->setCurrentIndex(selectIndex);
    else
        host_->stagePortCombo_->setCurrentIndex(0);
}

QString StagePanelController::selectedPortName() const
{
    if (host_->stagePortCombo_ == nullptr)
        return QStringLiteral("COM4");

    const QString portName = host_->stagePortCombo_->currentData().toString();
    return portName.isEmpty() ? host_->stagePortCombo_->currentText() : portName;
}

void StagePanelController::initializeWorker()
{
    auto controller = std::make_shared<ZaberStageController>();
    stageWorker_ = std::make_unique<StageWorker>(controller);
    stageWorker_->setStateCallback([this](const StageState state) {
        QMetaObject::invokeMethod(
            this,
            [this, state]() { onStateChanged(state); },
            Qt::QueuedConnection);
    });
    stageWorker_->setTopologyCallback([this](const StageTopology &topology) {
        QMetaObject::invokeMethod(
            this,
            [this, topology]() { onTopologyChanged(topology); },
            Qt::QueuedConnection);
    });
    stageWorker_->setErrorCallback([this](const StageError &error) {
        QMetaObject::invokeMethod(
            this,
            [this, error]() { onError(error); },
            Qt::QueuedConnection);
    });
    stageWorker_->start();
}
void StagePanelController::updateConnectionControls(const StageState state, const bool refreshCaptureControls)
{
    const bool connected = state == StageState::Connected;
    const bool homing = state == StageState::Homing;
    const bool busy = state == StageState::Connecting || homing;
    const bool motionActive = connected || busy;
    const bool captureScanActive = host_->isCaptureSessionActive();

    if (host_->stagePortCombo_ != nullptr)
        host_->stagePortCombo_->setEnabled(!connected && !busy && !captureScanActive);
    if (host_->stageBaudCombo_ != nullptr)
        host_->stageBaudCombo_->setEnabled(!connected && !busy && !captureScanActive);
    if (host_->stageConnectBtn_ != nullptr)
        host_->stageConnectBtn_->setEnabled(!connected && !busy && !captureScanActive);
    if (host_->stageDisconnectBtn_ != nullptr)
        host_->stageDisconnectBtn_->setEnabled(motionActive && !captureScanActive);
    if (host_->stageControlBox_ != nullptr)
        host_->stageControlBox_->setEnabled(connected);

    updateMotionControls(state);

    if (host_->stagePositionTimer_ != nullptr)
    {
        if (connected || homing)
        {
            if (homing)
                host_->stagePositionTimer_->setInterval(kManualPositionPollIntervalMs);
            else
                syncPositionPollInterval();
            host_->stagePositionTimer_->start();
            pollPosition();
        }
        else
        {
            host_->stagePositionTimer_->stop();
        }
    }

    if (state == StageState::Disconnected || state == StageState::Fault)
    {
        manualMotionDepth_ = 0;
        manualMotionCoastActive_ = false;
        manualMotionCoastStableCount_ = 0;
        updatePositionDisplay(0.0);
    }

    if (refreshCaptureControls && host_->capturePanel() != nullptr)
        host_->capturePanel()->updatePositionControls(state);
}

void StagePanelController::updateMotionControls(const StageState state)
{
    const bool connected = state == StageState::Connected;
    const bool captureScanActive = host_->isCaptureSessionActive();
    const bool manualMotion = connected && !captureScanActive;

    if (host_->stageHomeBtn_ != nullptr)
        host_->stageHomeBtn_->setEnabled(manualMotion);
    if (host_->stageToStartBtn_ != nullptr)
        host_->stageToStartBtn_->setEnabled(manualMotion);
    if (host_->stageBackBtn_ != nullptr)
        host_->stageBackBtn_->setEnabled(manualMotion);
    if (host_->stageForwardBtn_ != nullptr)
        host_->stageForwardBtn_->setEnabled(manualMotion);
    if (host_->stageToEndBtn_ != nullptr)
        host_->stageToEndBtn_->setEnabled(manualMotion);
    if (host_->stageAbsolutePositionSpin_ != nullptr)
        host_->stageAbsolutePositionSpin_->setEnabled(manualMotion);
    if (host_->stageAbsoluteMoveBtn_ != nullptr)
        host_->stageAbsoluteMoveBtn_->setEnabled(manualMotion);
    if (host_->stageStopBtn_ != nullptr)
        host_->stageStopBtn_->setEnabled(connected);
}

void StagePanelController::updatePositionDisplay(const double positionMm)
{
    if (host_->stageAxisWidget_ != nullptr)
        host_->stageAxisWidget_->setPositionMm(positionMm);

    if (host_->capturePanel() != nullptr && host_->isCaptureSessionActive())
        host_->capturePanel()->onStagePosition(positionMm);
}

void StagePanelController::syncPositionPollInterval()
{
    if (host_->stagePositionTimer_ == nullptr)
        return;

    const bool fastPoll = manualMotionDepth_ > 0 || manualMotionCoastActive_;
    const int intervalMs =
        fastPoll ? kManualPositionPollIntervalMs : kIdlePositionPollIntervalMs;
    host_->stagePositionTimer_->setInterval(intervalMs);
}

void StagePanelController::onManualMotionStarted()
{
    manualMotionCoastActive_ = false;
    manualMotionCoastStableCount_ = 0;
    ++manualMotionDepth_;
    syncPositionPollInterval();
    pollPosition();
}

void StagePanelController::onStopMotionRequested()
{
    if (stageWorker_ == nullptr)
        return;

    manualMotionCoastActive_ = false;
    manualMotionCoastStableCount_ = 0;

    stageWorker_->requestStopMotion(
        [this]() {
            QMetaObject::invokeMethod(
                this, [this]() { beginManualMotionCoastWatch(); }, Qt::QueuedConnection);
        },
        false);
}

void StagePanelController::onManualMotionStopped()
{
    if (manualMotionDepth_ <= 0 || stageWorker_ == nullptr)
        return;

    stageWorker_->requestStopMotion(
        [this]() {
            QMetaObject::invokeMethod(
                this, [this]() { beginManualMotionCoastWatch(); }, Qt::QueuedConnection);
        },
        false);
}

void StagePanelController::beginManualMotionCoastWatch()
{
    manualMotionCoastActive_ = true;
    manualMotionCoastStableCount_ = 0;
    syncPositionPollInterval();
    pollPosition();
}

void StagePanelController::onManualMotionCoastPositionSample(const double positionMm)
{
    if (!manualMotionCoastActive_)
        return;

    if (std::abs(positionMm - manualMotionCoastLastPositionMm_) <= kManualMotionCoastStableToleranceMm)
        ++manualMotionCoastStableCount_;
    else
        manualMotionCoastStableCount_ = 0;

    manualMotionCoastLastPositionMm_ = positionMm;

    if (manualMotionCoastStableCount_ >= kManualMotionCoastStablePollsRequired)
        finishManualMotionCoast();
}

void StagePanelController::finishManualMotionCoast()
{
    if (!manualMotionCoastActive_)
        return;

    manualMotionCoastActive_ = false;
    manualMotionCoastStableCount_ = 0;
    if (manualMotionDepth_ > 0)
        manualMotionDepth_ = std::max(0, manualMotionDepth_ - 1);
    syncPositionPollInterval();
    pollPosition();
}

void StagePanelController::pollPosition()
{
    if (stageWorker_ == nullptr)
        return;

    const StageState state = stageWorker_->currentState();
    if (state != StageState::Connected && state != StageState::Homing)
        return;

    stageWorker_->requestPrimaryPosition([this](const double positionMm, const bool ok) {
        if (!ok)
            return;

        QMetaObject::invokeMethod(
            this,
            [this, positionMm]() {
                updatePositionDisplay(positionMm);
                if (manualMotionCoastActive_)
                    onManualMotionCoastPositionSample(positionMm);
            },
            Qt::QueuedConnection);
    });
}

void StagePanelController::updateDeviceDisplay(const StageTopology &topology)
{
    if (host_->stageDeviceDisplay_ == nullptr)
        return;

    host_->stageDeviceDisplay_->setPlainText(formatStageTopology(topology));
}

void StagePanelController::clearDeviceDisplay()
{
    updateDeviceDisplay({});
}

void StagePanelController::onStateChanged(const StageState state)
{
    updateConnectionControls(state);

    if (state == StageState::Homing)
    {
        if (host_->stageHomingKind_ == MainWindow::StageHomingKind::Localization)
            host_->appendLog(QStringLiteral("Stage: initial homing (home sensor)\u2026"));
        else if (host_->stageHomingKind_ == MainWindow::StageHomingKind::BeforeDisconnect)
            host_->appendLog("Stage: homing before disconnect\u2026");
        else if (host_->stageHomingKind_ == MainWindow::StageHomingKind::Simple)
            host_->appendLog("Stage: homing lockstep group (home sensor)\u2026");
        else if (host_->stageHomingKind_ == MainWindow::StageHomingKind::BeforeCapture)
            host_->appendLog(QStringLiteral("Capture: homing stage before scan\u2026"));
        else if (host_->stageHomingKind_ == MainWindow::StageHomingKind::AfterCapture)
            host_->appendLog(QStringLiteral("Capture: homing stage (home sensor)\u2026"));
    }
    else if (state == StageState::Connected)
        host_->appendLog("Stage: homed and ready");

    if (state == StageState::Disconnected || state == StageState::Fault)
        clearDeviceDisplay();
}

void StagePanelController::onTopologyChanged(const StageTopology &topology)
{
    updateDeviceDisplay(topology);

    if (topology.devices.empty())
        return;

    host_->appendLog(QString("Stage: detected %1 device(s) on %2")
                  .arg(topology.devices.size())
                  .arg(QString::fromStdString(topology.portName)));

    for (const StageDeviceInfo &device : topology.devices)
    {
        host_->appendLog(QString("Stage: device %1 = %2 (%3 axis(es), FW %4)")
                      .arg(device.deviceAddress)
                      .arg(QString::fromStdString(device.name))
                      .arg(device.axisCount)
                      .arg(QString::fromStdString(device.firmwareVersion)));
    }

    if (topology.lockstepEnabled)
    {
        host_->appendLog(QString("Stage: lockstep group %1 enabled (primary axis %2, secondary axis %3, travel %4 mm)")
                      .arg(topology.lockstepGroupId)
                      .arg(topology.lockstepPrimaryAxis)
                      .arg(topology.lockstepSecondaryAxis)
                      .arg(topology.travelLengthMm, 0, 'f', 0));
    }

    if (topology.motionAccelerationMmPerSec2 > 0.0)
    {
        host_->appendLog(QString("Stage: motion acceleration %1 mm/s\u00B2")
                      .arg(topology.motionAccelerationMmPerSec2, 0, 'f', 1));
        if (topology.motionAccelerationRequestedMmPerSec2 > 0.0
            && std::abs(topology.motionAccelerationRequestedMmPerSec2 - topology.motionAccelerationMmPerSec2)
                   > 0.05)
        {
            host_->appendLog(QString("Stage: requested acceleration %1 mm/s\u00B2 rounds below device minimum \u2014 using %2 mm/s\u00B2")
                          .arg(topology.motionAccelerationRequestedMmPerSec2, 0, 'f', 1)
                          .arg(topology.motionAccelerationMmPerSec2, 0, 'f', 1));
        }
    }

    if (topology.axesHomed && host_->stageHomingKind_ != MainWindow::StageHomingKind::None)
    {
        if (host_->stageHomingKind_ == MainWindow::StageHomingKind::Localization)
            host_->appendLog(QStringLiteral("Stage: initial homing complete"));
        else if (host_->stageHomingKind_ == MainWindow::StageHomingKind::BeforeCapture)
        {
            host_->appendLog(QStringLiteral("Capture: homing complete \u2014 starting scan sequence\u2026"));
            host_->stageHomingKind_ = MainWindow::StageHomingKind::None;
            if (host_->capturePanel() != nullptr)
                host_->capturePanel()->onStageHomedForCapture();
            return;
        }
        else if (host_->stageHomingKind_ == MainWindow::StageHomingKind::AfterCapture)
        {
            host_->appendLog(QStringLiteral("Capture: homing complete"));
            host_->stageHomingKind_ = MainWindow::StageHomingKind::None;
            if (host_->capturePanel() != nullptr)
                host_->capturePanel()->onStageHomedAfterCapture();
        }
        else
            host_->appendLog("Stage: homing complete");

        host_->stageHomingKind_ = MainWindow::StageHomingKind::None;
    }
}

void StagePanelController::onError(const StageError &error)
{
    if (error.message.empty())
        return;

    host_->appendLog(QString("Stage error: %1").arg(QString::fromStdString(error.message)));

    if (host_->stageHomingKind_ == MainWindow::StageHomingKind::BeforeCapture
        && host_->isCaptureSessionActive())
    {
        host_->stageHomingKind_ = MainWindow::StageHomingKind::None;
        if (host_->capturePanel() != nullptr)
            host_->capturePanel()->onCaptureStageHomingFailed(
                QStringLiteral("Capture: homing failed."));
    }
    else if (host_->stageHomingKind_ == MainWindow::StageHomingKind::AfterCapture)
    {
        host_->stageHomingKind_ = MainWindow::StageHomingKind::None;
        if (host_->capturePanel() != nullptr)
            host_->capturePanel()->onStageHomedAfterCapture();
    }
}

} // namespace hf::stage
