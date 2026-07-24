// UR3e tab orchestration implementation.
#include "frontend/controllers/Ur3ePanelController.hpp"

#include "backend/HyperFusionConfig.hpp"
#include "backend/3dscanning/Ur3eClient.hpp"
#include "backend/3dscanning/Ur3eHemisphereScan.hpp"
#include "backend/3dscanning/Ur3eHemisphereScanReachability.hpp"
#include "backend/3dscanning/Ur3eMountTransform.hpp"
#include "backend/3dscanning/Ur3eWorkspaceBoundary.hpp"
#include "backend/3dscanning/Ur3eMoveItManager.hpp"
#include "backend/3dscanning/Ur3eRvizManager.hpp"
#include "backend/3dscanning/Ur3eServerManager.hpp"
#include "backend/3dscanning/Ur3eWslSetup.hpp"
#include "backend/3dscanning/Ur3eCameraTransforms.hpp"
#include "backend/3dscanning/Ur3eScanPlanCache.hpp"
#include "backend/3dscanning/BfsTiffIo.hpp"
#include "frontend/controllers/BfsPanelController.hpp"
#include "frontend/widgets/MainWindow.hpp"
#include "frontend/widgets/Ur3eExternalControlWaitDialog.hpp"
#include "frontend/widgets/Ur3eHemisphereScanSettingsWidget.hpp"
#include "frontend/widgets/Ur3eJointBarWidget.hpp"
#include "frontend/widgets/Ur3eScanRoutePlanWidget.hpp"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QFileDialog>
#include <QLineEdit>
#include <QDateTime>
#include <QMessageBox>
#include <QAbstractButton>
#include <QMetaObject>
#include <QPushButton>
#include <QThread>
#include <QTimer>
#include <QHash>

#include <QMetaObject>
#include <QVariant>

#include <cmath>
#include <chrono>
#include <mutex>
#include <thread>

#include <vector>

namespace hf::ur3e
{
namespace
{
constexpr int kUr3eJointCount = 6;
constexpr const char *kUr3eJointNames[kUr3eJointCount] = {
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
};

/// Wrist_2 / wrist_3 offsets in radians: −N…−1, +1…+N × step (center is separate).
std::vector<double> wristSweepOffsetsRad(const int stepsEachWay, const double stepDeg)
{
    std::vector<double> offsets;
    if (stepsEachWay <= 0 || !(stepDeg > 0.0))
        return offsets;
    const double stepRad = stepDeg * (3.14159265358979323846 / 180.0);
    offsets.reserve(static_cast<std::size_t>(2 * stepsEachWay));
    for (int i = stepsEachWay; i >= 1; --i)
        offsets.push_back(-static_cast<double>(i) * stepRad);
    for (int i = 1; i <= stepsEachWay; ++i)
        offsets.push_back(static_cast<double>(i) * stepRad);
    return offsets;
}

Ur3eScanTcpPose scanTcpFromLivePose(const Ur3eTcpPose &live, const Ur3eScanTcpPose &fallback)
{
    Ur3eScanTcpPose tcp = fallback;
    tcp.xM = live.x;
    tcp.yM = live.y;
    tcp.zM = live.z;
    tcp.rxRad = live.rx;
    tcp.ryRad = live.ry;
    tcp.rzRad = live.rz;
    return tcp;
}

void appendScanPlanFailureReport(MainWindow *host, const Ur3eHemisphereScanPlan &plan)
{
    if (host == nullptr || plan.unreachableCount <= 0)
        return;

    QHash<QString, QStringList> grouped;
    for (int pointIndex = 0; pointIndex < static_cast<int>(plan.points.size()); ++pointIndex)
    {
        const Ur3ePlannedScanPoint &point = plan.points[static_cast<std::size_t>(pointIndex)];
        if (point.reachable)
            continue;

        QString reason = point.planningError.trimmed();
        if (reason.isEmpty())
            reason = QStringLiteral("unknown");
        grouped[reason].append(QString::number(pointIndex));
    }

    host->appendLog(QStringLiteral("UR3e scan plan failure breakdown:"));
    for (auto it = grouped.constBegin(); it != grouped.constEnd(); ++it)
    {
        host->appendLog(QStringLiteral("UR3e scan plan failure: %1 — %2 point(s): #%3")
                            .arg(it.key())
                            .arg(it.value().size())
                            .arg(it.value().join(QLatin1Char(','))));
    }

    for (int pointIndex = 0; pointIndex < static_cast<int>(plan.points.size()); ++pointIndex)
    {
        const Ur3ePlannedScanPoint &point = plan.points[static_cast<std::size_t>(pointIndex)];
        if (point.reachable)
            continue;

        host->appendLog(
            QStringLiteral("UR3e scan plan failure pt %1 tcp=(%2, %3, %4) m: %5")
                .arg(pointIndex)
                .arg(point.tcp.xM, 0, 'f', 3)
                .arg(point.tcp.yM, 0, 'f', 3)
                .arg(point.tcp.zM, 0, 'f', 3)
                .arg(point.planningError.trimmed().isEmpty() ? QStringLiteral("unknown")
                                                               : point.planningError.trimmed()));
    }
}

QString formatJointTargetsDeg(const std::vector<double> &targetRad)
{
    QStringList parts;
    const int count = qMin(static_cast<int>(targetRad.size()), kUr3eJointCount);
    for (int jointIndex = 0; jointIndex < count; ++jointIndex)
    {
        double rad = targetRad[static_cast<std::size_t>(jointIndex)];
        while (rad > M_PI)
            rad -= 2.0 * M_PI;
        while (rad <= -M_PI)
            rad += 2.0 * M_PI;
        const int degrees = static_cast<int>(std::lround(rad * 180.0 / M_PI));
        parts << QStringLiteral("%1=%2°")
                     .arg(QString::fromUtf8(kUr3eJointNames[jointIndex]))
                     .arg(degrees);
    }
    return parts.join(QStringLiteral(", "));
}

QString formatTcpPose(const Ur3eTcpPose &pose)
{
    return QStringLiteral("[%1, %2, %3, %4, %5, %6]")
        .arg(pose.x, 0, 'f', 3)
        .arg(pose.y, 0, 'f', 3)
        .arg(pose.z, 0, 'f', 3)
        .arg(pose.rx, 0, 'f', 3)
        .arg(pose.ry, 0, 'f', 3)
        .arg(pose.rz, 0, 'f', 3);
}

QString formatConfiguredHomeJointsDeg()
{
    const std::array<double, 6> &homeDeg = hf::hardwareConfig().ur3e.homeJointsDeg;
    QStringList parts;
    for (int jointIndex = 0; jointIndex < kUr3eJointCount; ++jointIndex)
    {
        parts << QStringLiteral("%1=%2°")
                     .arg(QString::fromUtf8(kUr3eJointNames[jointIndex]))
                     .arg(static_cast<int>(std::lround(homeDeg[static_cast<std::size_t>(jointIndex)])));
    }
    return parts.join(QStringLiteral(", "));
}
} // namespace

Ur3ePanelController::Ur3ePanelController(MainWindow *host, QObject *parent)
    : QObject(parent)
    , host_(host)
    , serverManager_(std::make_unique<Ur3eServerManager>(host))
    , moveItManager_(std::make_unique<Ur3eMoveItManager>(host))
    , rvizManager_(std::make_unique<Ur3eRvizManager>(host))
{
    connect(serverManager_.get(),
            &Ur3eServerManager::stateChanged,
            this,
            &Ur3ePanelController::onSidecarStateChanged);
    connect(moveItManager_.get(),
            &Ur3eMoveItManager::stateChanged,
            this,
            &Ur3ePanelController::onMoveItStateChanged);
    connect(rvizManager_.get(),
            &Ur3eRvizManager::stateChanged,
            this,
            &Ur3ePanelController::onRvizStateChanged);

    connectPollTimer_ = new QTimer(this);
    connectPollTimer_->setInterval(kConnectPollIntervalMs);
    connect(connectPollTimer_, &QTimer::timeout, this, &Ur3ePanelController::onConnectPollTick);

    connectCountdownTimer_ = new QTimer(this);
    connectCountdownTimer_->setInterval(1000);
    connect(connectCountdownTimer_, &QTimer::timeout, this, &Ur3ePanelController::onConnectCountdownTick);

    driverReadyPollTimer_ = new QTimer(this);
    driverReadyPollTimer_->setInterval(kDriverReadyPollIntervalMs);
    connect(driverReadyPollTimer_, &QTimer::timeout, this, &Ur3ePanelController::pollDriverPrestartReady);

    boundarySyncTimer_ = new QTimer(this);
    boundarySyncTimer_->setInterval(kBoundarySyncIntervalMs);
    connect(boundarySyncTimer_, &QTimer::timeout, this, &Ur3ePanelController::onBoundarySyncTick);

    driverPrestartReady_ = !hf::hardwareConfig().ur3e.prestartDriver;
}

Ur3ePanelController::~Ur3ePanelController()
{
    dismissConnectWaitDialog();
    (void)shutdownSync();
}

bool Ur3ePanelController::isSidecarRunning() const
{
    return serverManager_ != nullptr && serverManager_->isServerConnected();
}

bool Ur3ePanelController::isScanPlanReady() const
{
    return scanPlanReady_ && plannedScanPlan_.reachableCount > 0;
}

void Ur3ePanelController::applyHardwareConfigToUi()
{
    const hf::HardwareConfig::Ur3eConfig &cfg = hf::hardwareConfig().ur3e;

    if (host_->ur3eRobotIpEdit_ != nullptr)
        host_->ur3eRobotIpEdit_->setText(cfg.robotIp);

    applyConfiguredInitialJointTargets();
    syncWorkspaceBoundaryPreview();
    if (host_->ur3eScanRoutePlanWidget_ != nullptr && host_->ur3eHemisphereScanSettings_ != nullptr)
        host_->ur3eScanRoutePlanWidget_->setScanParams(host_->ur3eHemisphereScanSettings_->params());
    updateRobotUi();
}

void Ur3ePanelController::startSidecarOnLaunch()
{
    if (serverManager_ == nullptr)
        return;

    const hf::HardwareConfig::Ur3eConfig &cfg = hf::hardwareConfig().ur3e;
    if (!cfg.use3dScanning)
        return;

    host_->appendLog(QStringLiteral(
        "UR3e: preparing WSL (%1, prestart_driver=%2)\u2026")
                        .arg(cfg.useMockHardware ? QStringLiteral("simulation startup")
                                                 : QStringLiteral("network setup + cleanup"))
                        .arg(cfg.prestartDriver ? QStringLiteral("true") : QStringLiteral("false")));

    std::thread([this]() {
        QString setupDetail;
        (void)runUr3eStartupSetupOnce(&setupDetail);
        QMetaObject::invokeMethod(
            this,
            [this, setupDetail]() {
                if (!setupDetail.isEmpty())
                    host_->appendLog(QStringLiteral("UR3e: %1").arg(setupDetail));
                host_->appendLog(QStringLiteral("UR3e: starting sidecar\u2026"));
                serverManager_->tryAutoStart();
            },
            Qt::QueuedConnection);
    }).detach();
}

void Ur3ePanelController::refreshUi()
{
    updateRobotUi();
}

bool Ur3ePanelController::shutdownSync()
{
    if (!hf::hardwareConfig().ur3e.use3dScanning)
        return true;

    shutdownRequested_.store(true, std::memory_order_release);
    stopRequested_.store(true, std::memory_order_release);
    ++scanExecuteSessionId_;

    if (connectInProgress_)
        onConnectDialogCancelled();

    scanExecuting_ = false;
    scanPlanning_ = false;
    motionInProgress_ = false;

    if (robotConnected_ && serverManager_ != nullptr && serverManager_->isServerConnected())
        ur3eStopMotion(serverManager_->serverUrl());

    {
        std::lock_guard<std::mutex> lock(scanExecuteThreadMutex_);
        if (scanExecuteThread_.joinable())
            scanExecuteThread_.join();
    }

    joinJointPollThread();

    if (host_->ur3ePosePollTimer_ != nullptr)
        host_->ur3ePosePollTimer_->stop();

    if (robotConnected_ && serverManager_ != nullptr && serverManager_->isServerConnected())
    {
        beginHomeMotionUi();
        const HomeEnsureOutcome homeOutcome =
            ensureRobotAtHomeSync(HomeEnsureContext::BeforeShutdown);
        endHomeMotionUi();
        if (homeOutcome.cancelled)
        {
            shutdownRequested_.store(false, std::memory_order_release);
            stopRequested_.store(false, std::memory_order_release);
            if (host_->ur3ePosePollTimer_ != nullptr)
                host_->ur3ePosePollTimer_->start(kPosePollIntervalMs);
            pollJointsSync();
            syncTargetsFromCurrent();
            return false;
        }
        pollJointsSync();
        if (homeOutcome.atHomeVerified)
            applyScanHomeJointTargets();
    }

    if (robotConnected_ && serverManager_ != nullptr && serverManager_->isServerConnected())
        ur3eDisconnectRobot(serverManager_->serverUrl());

    if (serverManager_ != nullptr)
        serverManager_->stopServer();

    if (moveItManager_ != nullptr)
        moveItManager_->stop();

    if (rvizManager_ != nullptr)
        rvizManager_->stop();

    robotConnected_ = false;
    return true;
}

void Ur3ePanelController::wireSettingsTabConnections()
{
    if (host_->ur3eConnectBtn_ != nullptr)
    {
        connect(host_->ur3eConnectBtn_, &QPushButton::clicked, this, [this]() {
            onConnectRequested();
        });
    }
    if (host_->ur3eDisconnectBtn_ != nullptr)
    {
        connect(host_->ur3eDisconnectBtn_, &QPushButton::clicked, this, [this]() {
            onDisconnectRequested();
        });
    }
    if (host_->ur3eMoveBtn_ != nullptr)
    {
        connect(host_->ur3eMoveBtn_, &QPushButton::clicked, this, [this]() {
            onMoveRequested();
        });
    }
    for (int jointIndex = 0; jointIndex < MainWindow::kUr3eJointCount; ++jointIndex)
    {
        ui::Ur3eJointBarWidget *bar = host_->ur3eJointBars_[jointIndex];
        if (bar == nullptr)
            continue;
        connect(bar,
                &ui::Ur3eJointBarWidget::targetChanged,
                this,
                [this](const double) { scheduleManualTargetPreview(); });
    }
    if (host_->ur3eStopMotionBtn_ != nullptr)
    {
        connect(host_->ur3eStopMotionBtn_, &QPushButton::clicked, this, [this]() {
            onStopMotionRequested();
        });
    }
    if (host_->ur3eSyncJointsBtn_ != nullptr)
    {
        connect(host_->ur3eSyncJointsBtn_, &QPushButton::clicked, this, [this]() {
            onSyncJointsRequested();
        });
    }
    if (host_->ur3eStartRvizBtn_ != nullptr)
    {
        connect(host_->ur3eStartRvizBtn_, &QPushButton::clicked, this, [this]() {
            onStartRvizRequested();
        });
    }
    if (host_->ur3eStartMoveItBtn_ != nullptr)
    {
        connect(host_->ur3eStartMoveItBtn_, &QPushButton::clicked, this, [this]() {
            onStartMoveItRequested();
        });
    }
    if (host_->ur3eHemisphereScanSettings_ != nullptr)
    {
        connect(host_->ur3eHemisphereScanSettings_,
                &ui::Ur3eHemisphereScanSettingsWidget::planScanRequested,
                this,
                [this]() { onPlanHemisphereScanRequested(); });
        connect(host_->ur3eHemisphereScanSettings_,
                &ui::Ur3eHemisphereScanSettingsWidget::executeScanRequested,
                this,
                [this]() { onExecuteHemisphereScanRequested(); });
        connect(host_->ur3eHemisphereScanSettings_,
                &ui::Ur3eHemisphereScanSettingsWidget::paramsChanged,
                this,
                [this]() {
                    scanPlanReady_ = false;
                    if (host_->ur3eScanRoutePlanWidget_ != nullptr)
                        host_->ur3eScanRoutePlanWidget_->clearScanPlan();
                    if (host_->ur3eScanRoutePlanWidget_ != nullptr
                        && host_->ur3eHemisphereScanSettings_ != nullptr)
                    {
                        host_->ur3eScanRoutePlanWidget_->setScanParams(
                            host_->ur3eHemisphereScanSettings_->params());
                    }
                    scheduleManualTargetPreview();
                    updateRobotUi();
                    if (host_->capturePanel() != nullptr)
                        host_->capturePanel()->syncBfsAnd3dRgbCaptureControls();
                });
    }

    if (host_->ur3ePosePollTimer_ != nullptr)
    {
        connect(host_->ur3ePosePollTimer_, &QTimer::timeout, this, [this]() { pollJoints(); });
    }

    tryLoadCachedScanPlan();
}

void Ur3ePanelController::onSidecarStateChanged(const Ur3eServerManager::State state,
                                                  const QString &detail)
{
    bool sidecarStateChanged = false;
    if (serverManager_ != nullptr)
    {
        const bool stateChanged = state != lastLoggedSidecarState_;
        sidecarStateChanged = stateChanged;
        if (stateChanged)
        {
            lastLoggedSidecarState_ = state;
            if (state == Ur3eServerManager::State::Starting)
            {
                host_->appendLog(QStringLiteral("UR3e sidecar: starting (driver may take up to 2 min)\u2026"));
            }
            else if (state == Ur3eServerManager::State::Running
                || state == Ur3eServerManager::State::Failed
                || state == Ur3eServerManager::State::Unavailable)
            {
                host_->appendLog(
                    QStringLiteral("UR3e sidecar: %1").arg(serverManager_->statusText()));
            }
        }
        else if (!detail.isEmpty())
        {
            const QStringList lines = detail.split(QLatin1Char('\n'));
            for (const QString &line : lines)
            {
                const QString trimmed = line.trimmed();
                if (trimmed.isEmpty() || trimmed.contains(QStringLiteral("GET /health"), Qt::CaseInsensitive))
                    continue;
                if (connectInProgress_
                    && (trimmed.contains(QStringLiteral("connect phase="), Qt::CaseInsensitive)
                        || trimmed.contains(QStringLiteral("GET /connect/status"), Qt::CaseInsensitive)))
                {
                    updateConnectDialogFromSidecarLine(trimmed);
                    continue;
                }
                host_->appendLog(trimmed);
                updateConnectDialogFromSidecarLine(trimmed);
            }
        }
    }

    if (!isSidecarRunning())
    {
        robotConnected_ = false;
        driverPrestartReady_ = false;
        if (driverReadyPollTimer_ != nullptr)
            driverReadyPollTimer_->stop();
        if (boundarySyncTimer_ != nullptr)
            boundarySyncTimer_->stop();
        if (host_->ur3ePosePollTimer_ != nullptr)
            host_->ur3ePosePollTimer_->stop();
        updateRobotUi();
    }
    else
    {
        if (boundarySyncTimer_ != nullptr && !boundarySyncTimer_->isActive())
            boundarySyncTimer_->start();
        // Only (re)start the driver-ready poll on an actual transition into Running.
        if (state == Ur3eServerManager::State::Running && sidecarStateChanged)
        {
            pushWorkspaceBoundaryToMoveIt();
            const hf::HardwareConfig::Ur3eConfig &cfg = hf::hardwareConfig().ur3e;
            if (cfg.prestartDriver)
            {
                driverPrestartReady_ = false;
                if (driverReadyPollTimer_ != nullptr)
                    driverReadyPollTimer_->start();
                pollDriverPrestartReady();
            }
            else
            {
                driverPrestartReady_ = true;
            }
        }
        updateRobotUi();
    }
}

void Ur3ePanelController::setBusy(const bool busy)
{
    busy_ = busy;
    updateRobotUi();
}

void Ur3ePanelController::pollDriverPrestartReady()
{
    if (!hf::hardwareConfig().ur3e.prestartDriver || serverManager_ == nullptr
        || !serverManager_->isServerConnected())
    {
        if (driverReadyPollTimer_ != nullptr)
            driverReadyPollTimer_->stop();
        return;
    }

    if (driverPrestartReady_
        || driverReadyPollInFlight_.exchange(true, std::memory_order_acq_rel))
    {
        return;
    }

    const QString serverUrl = serverManager_->serverUrl();
    std::thread([this, serverUrl]() {
        Ur3eHealthStatus health;
        const bool ok = ur3eServerHealthCheck(serverUrl, &health);
        QMetaObject::invokeMethod(
            this,
            [this, ok, health]() {
                driverReadyPollInFlight_.store(false, std::memory_order_release);
                if (!ok || !isSidecarRunning())
                    return;

                if (health.driverReady)
                {
                    if (!driverPrestartReady_)
                    {
                        host_->appendLog(
                            hf::hardwareConfig().ur3e.useMockHardware
                                ? QStringLiteral("UR3e: simulation driver ready.")
                                : QStringLiteral("UR3e: robot driver ready."));
                    }
                    driverPrestartReady_ = true;
                    if (driverReadyPollTimer_ != nullptr)
                        driverReadyPollTimer_->stop();
                    updateRobotUi();
                }
            },
            Qt::QueuedConnection);
    }).detach();
}

void Ur3ePanelController::updateRobotUi()
{
    const bool sidecarRunning = isSidecarRunning();
    const bool captureActive = host_->isCaptureSessionActive();

    if (host_->ur3eRobotIpEdit_ != nullptr)
        host_->ur3eRobotIpEdit_->setEnabled(!robotConnected_ && !busy_ && !captureActive);

    const bool driverReady =
        !hf::hardwareConfig().ur3e.prestartDriver || driverPrestartReady_;

    if (host_->ur3eConnectBtn_ != nullptr)
    {
        host_->ur3eConnectBtn_->setEnabled(sidecarRunning && driverReady && !robotConnected_ && !busy_
                                            && !connectInProgress_ && !captureActive);
        if (!sidecarRunning)
        {
            host_->ur3eConnectBtn_->setToolTip(
                QStringLiteral("Waiting for UR3e sidecar (WSL). Check the Log tab for status."));
        }
        else if (!driverReady)
        {
            host_->ur3eConnectBtn_->setToolTip(
                hf::hardwareConfig().ur3e.useMockHardware
                    ? QStringLiteral(
                          "Waiting for simulation driver warmup in WSL (up to ~2 min after sidecar starts).")
                    : QStringLiteral("Waiting for UR robot driver warmup in WSL."));
        }
        else
        {
            host_->ur3eConnectBtn_->setToolTip(QString());
        }
    }
    if (host_->ur3eDisconnectBtn_ != nullptr)
    {
        host_->ur3eDisconnectBtn_->setEnabled(sidecarRunning && robotConnected_ && !busy_
                                              && !captureActive);
    }

    const bool motionReady = sidecarRunning && robotConnected_ && !captureActive;
    const bool canStartMotion = motionReady && !busy_;

    if (host_->ur3eMoveBtn_ != nullptr)
        host_->ur3eMoveBtn_->setEnabled(canStartMotion);
    if (host_->ur3eStopMotionBtn_ != nullptr)
        host_->ur3eStopMotionBtn_->setEnabled(motionInProgress_ || scanExecuting_);
    if (host_->ur3eSyncJointsBtn_ != nullptr)
        host_->ur3eSyncJointsBtn_->setEnabled(canStartMotion);

    const bool vizUiEnabled =
        sidecarRunning && robotConnected_ && !busy_ && !captureActive;
    const bool rvizRunning = rvizManager_ != nullptr && rvizManager_->isRunning();
    if (host_->ur3eStartRvizBtn_ != nullptr)
    {
        host_->ur3eStartRvizBtn_->setEnabled(vizUiEnabled);
        host_->ur3eStartRvizBtn_->setText(rvizRunning ? QStringLiteral("Stop RViz")
                                                      : QStringLiteral("Start RViz"));
    }

    const bool moveItRunning =
        moveItManager_ != nullptr && moveItManager_->isRunning();
    if (host_->ur3eStartMoveItBtn_ != nullptr)
    {
        host_->ur3eStartMoveItBtn_->setEnabled(vizUiEnabled);
        host_->ur3eStartMoveItBtn_->setText(moveItRunning ? QStringLiteral("Stop MoveIt")
                                                          : QStringLiteral("Start MoveIt"));
    }

    for (int jointIndex = 0; jointIndex < MainWindow::kUr3eJointCount; ++jointIndex)
    {
        ui::Ur3eJointBarWidget *bar = host_->ur3eJointBars_[jointIndex];
        if (bar != nullptr)
            bar->setEnabled(canStartMotion);
    }

    if (host_->ur3eHemisphereScanSettings_ != nullptr)
    {
        host_->ur3eHemisphereScanSettings_->setPlanEnabled(canStartMotion && !busy_);
        host_->ur3eHemisphereScanSettings_->setExecuteEnabled(canStartMotion && scanPlanReady_
                                                               && plannedScanPlan_.reachableCount > 0
                                                               && !scanExecuting_);
        host_->ur3eHemisphereScanSettings_->setParamsEnabled(!scanPlanning_ && !scanExecuting_);
    }
}

void Ur3ePanelController::applyJointTargets(const std::vector<double> &positionsRad,
                                            const QStringList &names)
{
    for (int uiIndex = 0; uiIndex < MainWindow::kUr3eJointCount; ++uiIndex)
    {
        ui::Ur3eJointBarWidget *bar = host_->ur3eJointBars_[uiIndex];
        if (bar == nullptr)
            continue;

        int sourceIndex = uiIndex;
        if (names.size() >= MainWindow::kUr3eJointCount)
        {
            const int namedIndex =
                names.indexOf(QString::fromUtf8(kUr3eJointNames[uiIndex]));
            if (namedIndex >= 0)
                sourceIndex = namedIndex;
        }

        if (sourceIndex < 0 || sourceIndex >= static_cast<int>(positionsRad.size()))
            continue;

        bar->setValueRadians(positionsRad[static_cast<std::size_t>(sourceIndex)]);
    }
}

void Ur3ePanelController::setJointPollIntervalMs(const int intervalMs)
{
    if (host_->ur3ePosePollTimer_ != nullptr)
        host_->ur3ePosePollTimer_->setInterval(qMax(50, intervalMs));
}

void Ur3ePanelController::applyJointPositions(const std::vector<double> &positionsRad,
                                              const QStringList &names,
                                              const bool syncTargets)
{
    for (int uiIndex = 0; uiIndex < MainWindow::kUr3eJointCount; ++uiIndex)
    {
        ui::Ur3eJointBarWidget *bar = host_->ur3eJointBars_[uiIndex];
        if (bar == nullptr)
            continue;

        int sourceIndex = uiIndex;
        if (names.size() >= MainWindow::kUr3eJointCount)
        {
            const int namedIndex =
                names.indexOf(QString::fromUtf8(kUr3eJointNames[uiIndex]));
            if (namedIndex >= 0)
                sourceIndex = namedIndex;
        }

        if (sourceIndex < 0 || sourceIndex >= static_cast<int>(positionsRad.size()))
            continue;

        const double radians = positionsRad[static_cast<std::size_t>(sourceIndex)];
        bar->setCurrentRadians(radians);
        if (syncTargets)
            bar->setValueRadians(radians);
    }
}

void Ur3ePanelController::syncWorkspaceBoundaryPreview()
{
    const Ur3eWorkspaceBoundary boundary =
        workspaceBoundaryFromConfig(hf::hardwareConfig().ur3e);

    if (host_->ur3eHemisphereScanSettings_ != nullptr)
        host_->ur3eHemisphereScanSettings_->applyBoundaryLimits(boundary);

    if (host_->ur3eScanRoutePlanWidget_ != nullptr)
    {
        host_->ur3eScanRoutePlanWidget_->setWorkspaceBoundary(boundary);
        host_->ur3eScanRoutePlanWidget_->setSceneMount(
            Ur3eMountTransform::sceneAlignFromConfig(hf::hardwareConfig().ur3e));
    }

    pushWorkspaceBoundaryToMoveIt();
    scheduleManualTargetPreview();
}

void Ur3ePanelController::pushWorkspaceBoundaryToMoveIt()
{
    if (serverManager_ == nullptr || !serverManager_->isServerConnected())
        return;

    if (boundarySyncInFlight_.exchange(true))
        return;

    const QString serverUrl = serverManager_->serverUrl();
    std::thread([this, serverUrl]() {
        QString error;
        (void)ur3eSyncWorkspaceBoundary(serverUrl, &error);
        boundarySyncInFlight_.store(false);
        Q_UNUSED(error);
    }).detach();
}

void Ur3ePanelController::onBoundarySyncTick()
{
    if (!isSidecarRunning())
        return;

    const bool moveItRunning = moveItManager_ != nullptr && moveItManager_->isRunning();
    if (robotConnected_ || moveItRunning)
        pushWorkspaceBoundaryToMoveIt();
}

void Ur3ePanelController::applyScanHomeJointTargets()
{
    const hf::HardwareConfig::Ur3eConfig &cfg = hf::hardwareConfig().ur3e;
    for (int jointIndex = 0; jointIndex < MainWindow::kUr3eJointCount; ++jointIndex)
    {
        ui::Ur3eJointBarWidget *bar = host_->ur3eJointBars_[jointIndex];
        if (bar == nullptr)
            continue;
        const double radians = cfg.homeJointsDeg[static_cast<std::size_t>(jointIndex)] * M_PI / 180.0;
        bar->setValueRadians(radians);
    }

    scheduleManualTargetPreview();
}

void Ur3ePanelController::syncHomeJointTargetSliders()
{
    applyScanHomeJointTargets();
}

void Ur3ePanelController::applyConfiguredInitialJointTargets()
{
    if (!hf::hardwareConfig().ur3e.useMockHardware)
        return;

    applyScanHomeJointTargets();
}

void Ur3ePanelController::beginHomeMotionUi()
{
    motionInProgress_ = true;
    setJointPollIntervalMs(kMotionPollIntervalMs);
    applyScanHomeJointTargets();
}

void Ur3ePanelController::endHomeMotionUi()
{
    motionInProgress_ = false;
    setJointPollIntervalMs(kPosePollIntervalMs);
}

void Ur3ePanelController::scheduleManualTargetPreview()
{
    if (!robotConnected_ || serverManager_ == nullptr || !serverManager_->isServerConnected()
        || shutdownRequested_.load(std::memory_order_acquire))
    {
        return;
    }

    if (manualTargetPreviewInFlight_.load(std::memory_order_acquire))
    {
        manualTargetPreviewPending_.store(true, std::memory_order_release);
        return;
    }

    pushManualTargetPreview();
}

void Ur3ePanelController::pushManualTargetPreview()
{
    if (!robotConnected_ || serverManager_ == nullptr || !serverManager_->isServerConnected()
        || shutdownRequested_.load(std::memory_order_acquire))
    {
        manualTargetPreviewPending_.store(false, std::memory_order_release);
        return;
    }

    if (manualTargetPreviewInFlight_.exchange(true, std::memory_order_acq_rel))
    {
        manualTargetPreviewPending_.store(true, std::memory_order_release);
        return;
    }

    manualTargetPreviewPending_.store(false, std::memory_order_release);

    std::vector<double> target;
    target.reserve(MainWindow::kUr3eJointCount);
    for (int jointIndex = 0; jointIndex < MainWindow::kUr3eJointCount; ++jointIndex)
    {
        ui::Ur3eJointBarWidget *bar = host_->ur3eJointBars_[jointIndex];
        target.push_back(bar != nullptr ? bar->valueRadians() : 0.0);
    }

    const QString serverUrl = serverManager_->serverUrl();
    std::thread([this, serverUrl, target]() {
        QString error;
        (void)ur3ePreviewManualTarget(serverUrl, target, &error);
        QMetaObject::invokeMethod(
            this,
            [this]() {
                manualTargetPreviewInFlight_.store(false, std::memory_order_release);
                if (manualTargetPreviewPending_.exchange(false, std::memory_order_acq_rel))
                    pushManualTargetPreview();
            },
            Qt::QueuedConnection);
    }).detach();
}

void Ur3ePanelController::syncTargetsFromCurrent()
{
    for (int jointIndex = 0; jointIndex < MainWindow::kUr3eJointCount; ++jointIndex)
    {
        ui::Ur3eJointBarWidget *bar = host_->ur3eJointBars_[jointIndex];
        if (bar != nullptr)
            bar->syncTargetFromCurrent();
    }
}

void Ur3ePanelController::joinJointPollThread()
{
    std::lock_guard<std::mutex> lock(jointPollThreadMutex_);
    if (jointPollThread_.joinable())
        jointPollThread_.join();
}

void Ur3ePanelController::applyPolledJoints(const QVariantList &positionsRad, const QStringList &names)
{
    if (positionsRad.size() < MainWindow::kUr3eJointCount)
        return;

    std::vector<double> positions;
    positions.reserve(static_cast<std::size_t>(positionsRad.size()));
    for (const QVariant &value : positionsRad)
        positions.push_back(value.toDouble());

    applyJointPositions(positions, names, false);
}

void Ur3ePanelController::pollJointsSync()
{
    if (!robotConnected_ || serverManager_ == nullptr || !serverManager_->isServerConnected()
        || shutdownRequested_.load(std::memory_order_acquire))
    {
        return;
    }

    const Ur3eJointsState result = ur3eGetJoints(serverManager_->serverUrl());
    if (!result.ok || result.positionsRad.size() < MainWindow::kUr3eJointCount)
        return;

    applyJointPositions(result.positionsRad, result.names, false);
}

void Ur3ePanelController::pollJoints()
{
    if (!robotConnected_ || serverManager_ == nullptr || !serverManager_->isServerConnected()
        || shutdownRequested_.load(std::memory_order_acquire) || connectInProgress_)
    {
        return;
    }

    if (busy_ && !scanExecuting_ && !motionInProgress_)
        return;

    // Skip this tick if the previous poll is still running so we never queue up
    // blocking wsl.exe/curl launches on the UI thread.
    if (jointPollInFlight_.exchange(true, std::memory_order_acq_rel))
        return;

    const QString serverUrl = serverManager_->serverUrl();

    std::lock_guard<std::mutex> lock(jointPollThreadMutex_);
    if (jointPollThread_.joinable())
        jointPollThread_.join();

    jointPollThread_ = std::thread([this, serverUrl]() {
        const Ur3eJointsState result = ur3eGetJoints(serverUrl);
        if (!shutdownRequested_.load(std::memory_order_acquire) && result.ok
            && result.positionsRad.size() >= static_cast<std::size_t>(MainWindow::kUr3eJointCount))
        {
            QVariantList positions;
            positions.reserve(static_cast<int>(result.positionsRad.size()));
            for (const double value : result.positionsRad)
                positions.append(value);

            QMetaObject::invokeMethod(
                this,
                "applyPolledJoints",
                Qt::QueuedConnection,
                Q_ARG(QVariantList, positions),
                Q_ARG(QStringList, result.names));
        }
        jointPollInFlight_.store(false, std::memory_order_release);
    });
}

void Ur3ePanelController::onSyncJointsRequested()
{
    if (serverManager_ == nullptr || busy_ || !robotConnected_)
        return;

    pollJointsSync();
    syncTargetsFromCurrent();
    host_->appendLog(QStringLiteral("UR3e: joint targets synced from current pose."));
}

void Ur3ePanelController::onMoveRequested()
{
    if (serverManager_ == nullptr || busy_ || !robotConnected_)
        return;

    const QString serverUrl = serverManager_->serverUrl();

    std::vector<double> target;
    target.reserve(MainWindow::kUr3eJointCount);
    for (int jointIndex = 0; jointIndex < MainWindow::kUr3eJointCount; ++jointIndex)
    {
        ui::Ur3eJointBarWidget *bar = host_->ur3eJointBars_[jointIndex];
        target.push_back(bar != nullptr ? bar->valueRadians() : 0.0);
    }

    const QString targetSummary = formatJointTargetsDeg(target);
    host_->appendLog(
        QStringLiteral("UR3e: Move (joint, MoveIt) requested — %1").arg(targetSummary));
    stopRequested_.store(false, std::memory_order_release);
    motionInProgress_ = true;
    setBusy(true);
    setJointPollIntervalMs(kMotionPollIntervalMs);
    std::thread([this, serverUrl, target, targetSummary]() {
        const Ur3eScanWaypointMoveResult result =
            ur3eExecuteScanWaypoint(serverUrl, target, nullptr, nullptr, false, true);
        const bool ok = result.ok;
        QString detail;
        if (ok)
            detail = QStringLiteral("MoveIt move complete — %1").arg(targetSummary);
        else if (result.stopped)
            detail = result.errorMessage.isEmpty() ? QStringLiteral("Motion stopped.") : result.errorMessage;
        else if (result.skipped)
            detail = result.errorMessage.isEmpty()
                         ? QStringLiteral("Move skipped — no collision-free path.")
                         : result.errorMessage;
        else
            detail = result.errorMessage.isEmpty() ? QStringLiteral("MoveIt motion failed.") : result.errorMessage;
        QMetaObject::invokeMethod(
            this,
            [this, ok, detail]() { finishMove(ok, detail); },
            Qt::QueuedConnection);
    }).detach();
}

void Ur3ePanelController::onStartMoveItRequested()
{
    if (moveItManager_ == nullptr || busy_)
        return;

    if (!isSidecarRunning())
    {
        host_->appendLog(QStringLiteral("UR3e: sidecar not ready — cannot start MoveIt."));
        return;
    }

    if (!robotConnected_)
    {
        host_->appendLog(QStringLiteral(
            "UR3e: connect the robot first (MoveIt needs the running UR driver)."));
        return;
    }

    if (moveItManager_->isRunning())
    {
        host_->appendLog(QStringLiteral("UR3e: stopping MoveIt\u2026"));
        moveItManager_->stop();
        updateRobotUi();
        return;
    }

    if (rvizManager_->isRunning())
    {
        host_->appendLog(QStringLiteral("UR3e: stopping RViz (MoveIt launches its own window)\u2026"));
        rvizManager_->stop();
    }

    const Ur3eJointsState joints = ur3eGetJoints(serverManager_->serverUrl());
    if (!joints.ok || joints.positionsRad.size() < MainWindow::kUr3eJointCount)
    {
        host_->appendLog(QStringLiteral(
            "UR3e: /joint_states not available — connect the robot and wait for the driver "
            "before starting MoveIt."));
        return;
    }

    host_->appendLog(QStringLiteral("UR3e: starting MoveIt 2 + RViz in WSL\u2026"));
    moveItManager_->start();
    pushWorkspaceBoundaryToMoveIt();
    updateRobotUi();
}

void Ur3ePanelController::onPlanHemisphereScanRequested()
{
    if (host_->ur3eHemisphereScanSettings_ == nullptr || busy_)
        return;

    if (!robotConnected_ || serverManager_ == nullptr)
    {
        host_->appendLog(
            QStringLiteral("UR3e scan plan: connect the robot first (MoveIt needs the UR driver)."));
        return;
    }

    const Ur3eHemisphereScanParams scanParams = host_->ur3eHemisphereScanSettings_->params();
    const Ur3eWorkspaceBoundary boundary =
        workspaceBoundaryFromConfig(hf::hardwareConfig().ur3e);
    const QString serverUrl = serverManager_->serverUrl();

    host_->appendLog(QStringLiteral("UR3e scan plan: running MoveIt IK + collision check…"));
    scanPlanning_ = true;
    setBusy(true);

    std::thread([this, scanParams, boundary, serverUrl]() {
        QString errorMessage;
        const Ur3eHemisphereScanPlan plan =
            evaluateHemisphereScanPlanMoveIt(serverUrl, scanParams, boundary, &errorMessage);
        QMetaObject::invokeMethod(
            this,
            [this, plan, errorMessage]() { finishScanPlan(plan, errorMessage); },
            Qt::QueuedConnection);
    }).detach();
}

void Ur3ePanelController::finishScanPlan(const Ur3eHemisphereScanPlan &plan,
                                         const QString &errorMessage)
{
    scanPlanning_ = false;
    setBusy(false);

    if (!errorMessage.isEmpty() && plan.points.empty())
    {
        host_->appendLog(QStringLiteral("UR3e scan plan failed: %1").arg(errorMessage));
        scanPlanReady_ = false;
        updateRobotUi();
        if (host_->capturePanel() != nullptr)
            host_->capturePanel()->syncBfsAnd3dRgbCaptureControls();
        return;
    }

    plannedScanPlan_ = plan;
    scanPlanReady_ = !plan.points.empty();

    host_->appendLog(
        QStringLiteral("UR3e scan plan (MoveIt): %1 points — %2 reachable, %3 unreachable.")
            .arg(plan.points.size())
            .arg(plan.reachableCount)
            .arg(plan.unreachableCount));

    if (!plan.errorMessage.isEmpty())
        host_->appendLog(QStringLiteral("UR3e scan plan: %1").arg(plan.errorMessage));

    if (plan.points.empty())
    {
        updateRobotUi();
        if (host_->capturePanel() != nullptr)
            host_->capturePanel()->syncBfsAnd3dRgbCaptureControls();
        return;
    }

    if (plan.reachableCount == 0)
    {
        host_->appendLog(QStringLiteral(
            "UR3e scan plan: no collision-free IK poses — adjust grid, radius, or workspace."));
        appendScanPlanFailureReport(host_, plan);
    }
    else if (plan.unreachableCount > 0)
    {
        appendScanPlanFailureReport(host_, plan);
    }

    if (host_->ur3eScanRoutePlanWidget_ != nullptr)
    {
        const Ur3eHemisphereScanParams scanParams = host_->ur3eHemisphereScanSettings_->params();
        host_->ur3eScanRoutePlanWidget_->setScanParams(scanParams);
        host_->ur3eScanRoutePlanWidget_->setScanPlan(plan);
    }

    saveCachedScanPlan();

    updateRobotUi();
    if (host_->capturePanel() != nullptr)
        host_->capturePanel()->syncBfsAnd3dRgbCaptureControls();
}

void Ur3ePanelController::saveCachedScanPlan() const
{
    if (host_ == nullptr || host_->ur3eHemisphereScanSettings_ == nullptr)
        return;
    if (!host_->ur3eHemisphereScanSettings_->rememberLastPlan())
        return;
    if (plannedScanPlan_.points.empty())
        return;

    const QString fingerprint = ur3eScanPlanFingerprint(hf::hardwareConfig().ur3e,
                                                        host_->ur3eHemisphereScanSettings_->params());
    QString error;
    if (!saveUr3eScanPlanCache(defaultUr3eScanPlanCachePath(), fingerprint, plannedScanPlan_,
                               &error))
    {
        host_->appendLog(QStringLiteral("UR3e scan plan cache: save failed — %1").arg(error));
        return;
    }
    host_->appendLog(QStringLiteral("UR3e scan plan cache: saved (%1 points).")
                         .arg(plannedScanPlan_.points.size()));
}

void Ur3ePanelController::tryLoadCachedScanPlan()
{
    if (host_ == nullptr || host_->ur3eHemisphereScanSettings_ == nullptr)
        return;
    if (!host_->ur3eHemisphereScanSettings_->rememberLastPlan())
        return;

    const QString fingerprint = ur3eScanPlanFingerprint(hf::hardwareConfig().ur3e,
                                                        host_->ur3eHemisphereScanSettings_->params());
    Ur3eHemisphereScanPlan plan;
    QString error;
    if (!loadUr3eScanPlanCache(defaultUr3eScanPlanCachePath(), fingerprint, plan, &error))
    {
        if (QFile::exists(defaultUr3eScanPlanCachePath()))
            host_->appendLog(QStringLiteral("UR3e scan plan cache: not loaded — %1").arg(error));
        return;
    }

    plannedScanPlan_ = plan;
    scanPlanReady_ = plan.reachableCount > 0;

    host_->appendLog(
        QStringLiteral("UR3e scan plan cache: loaded %1 points (%2 reachable, %3 unreachable).")
            .arg(plan.points.size())
            .arg(plan.reachableCount)
            .arg(plan.unreachableCount));

    if (host_->ur3eScanRoutePlanWidget_ != nullptr)
    {
        host_->ur3eScanRoutePlanWidget_->setScanParams(
            host_->ur3eHemisphereScanSettings_->params());
        host_->ur3eScanRoutePlanWidget_->setScanPlan(plan);
    }

    updateRobotUi();
    if (host_->capturePanel() != nullptr)
        host_->capturePanel()->syncBfsAnd3dRgbCaptureControls();
}

void Ur3ePanelController::onExecuteHemisphereScanRequested()
{
    const bool bfsConnected =
        host_ != nullptr && host_->bfsPanel() != nullptr
        && host_->bfsPanel()->isCameraConnected();

    if (!bfsConnected)
    {
        startHemisphereScanExecute({});
        return;
    }

    const QString parentDir = QFileDialog::getExistingDirectory(
        host_,
        QStringLiteral("Save 3D scanning images"),
        QString(),
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (parentDir.isEmpty())
    {
        host_->appendLog(QStringLiteral("UR3e scan execute: cancelled (no save folder)."));
        return;
    }

    const QString stamp =
        QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss"));
    const QString captureDir =
        QDir(parentDir).filePath(QStringLiteral("3d_scanning_%1").arg(stamp));
    if (!QDir().mkpath(captureDir))
    {
        host_->appendLog(
            QStringLiteral("UR3e scan execute rejected: could not create %1").arg(captureDir));
        QMessageBox::warning(host_,
                             QStringLiteral("UR3e Scan Execute"),
                             QStringLiteral("Could not create folder:\n%1").arg(captureDir));
        return;
    }

    HemisphereScanExecuteOptions opts;
    opts.captureOutputDir = captureDir;
    opts.stabilizeMs = hf::hardwareConfig().ur3e.scanCaptureStabilizeMs;
    if (!startHemisphereScanExecute(opts))
    {
        host_->appendLog(QStringLiteral("UR3e scan execute: capture start rejected."));
    }
}

bool Ur3ePanelController::startHemisphereScanExecute(const HemisphereScanExecuteOptions &options)
{
    if (host_ == nullptr || host_->ur3eHemisphereScanSettings_ == nullptr || busy_
        || !robotConnected_ || serverManager_ == nullptr || scanExecuting_)
        return false;

    if (!scanPlanReady_ || plannedScanPlan_.reachableCount == 0)
    {
        host_->appendLog(
            QStringLiteral("UR3e scan execute rejected: plan a route with reachable points first."));
        return false;
    }

    const std::vector<int> order = buildHemisphereScanExecutionOrder(plannedScanPlan_);
    if (order.empty())
    {
        host_->appendLog(QStringLiteral("UR3e scan execute rejected: no stored joint solutions."));
        return false;
    }

    const QString captureDir = options.captureOutputDir.trimmed();
    const bool captureStills = !captureDir.isEmpty();
    const hf::HardwareConfig::Ur3eConfig &ur3eCfg = hf::hardwareConfig().ur3e;
    const int stabilizeMs =
        options.stabilizeMs > 0 ? options.stabilizeMs
                                : (ur3eCfg.scanCaptureStabilizeMs >= 0 ? ur3eCfg.scanCaptureStabilizeMs
                                                                      : kScanCaptureStabilizeMs);
    const bool wristSweepEnabled = ur3eCfg.scanWristSweepEnabled;
    const double wristSweepStepDeg = ur3eCfg.scanWristSweepStepDeg;
    const int wristSweepStepsEachWay = ur3eCfg.scanWristSweepStepsEachWay;
    const int wristPosesPerPin =
        wristSweepEnabled
            ? (1 + (2 * wristSweepStepsEachWay) * (2 * wristSweepStepsEachWay))
            : 1;

    if (captureStills)
    {
        if (host_->bfsPanel() == nullptr || !host_->bfsPanel()->isCameraConnected())
        {
            host_->appendLog(QStringLiteral(
                "UR3e scan execute rejected: BFS camera must be connected for 3D capture."));
            return false;
        }
        QDir().mkpath(captureDir);
    }

    const QString serverUrl = serverManager_->serverUrl();
    host_->appendLog(
        QStringLiteral("UR3e scan execute: %1 reachable point(s), top-ring-first sweep "
                       "(%2 ms settle%3%4)…")
            .arg(order.size())
            .arg(stabilizeMs)
            .arg(captureStills ? QStringLiteral(", BFS stills → ") + captureDir
                               : QStringLiteral(", motion-only"))
            .arg(wristSweepEnabled
                     ? QStringLiteral(", wrist_2/3 grid %1×%2° → %3 pose(s)/pin")
                           .arg(2 * wristSweepStepsEachWay)
                           .arg(wristSweepStepDeg, 0, 'f', 0)
                           .arg(wristPosesPerPin)
                     : QString()));
    stopRequested_.store(false, std::memory_order_release);
    scanExecuting_ = true;
    if (host_->ur3eScanRoutePlanWidget_ != nullptr)
        host_->ur3eScanRoutePlanWidget_->beginScanExecution();
    setBusy(true);
    setJointPollIntervalMs(kMotionPollIntervalMs);

    const Ur3eHemisphereScanPlan planCopy = plannedScanPlan_;
    const int sessionId = ++scanExecuteSessionId_;

    {
        std::lock_guard<std::mutex> lock(scanExecuteThreadMutex_);
        if (scanExecuteThread_.joinable())
            scanExecuteThread_.join();

        scanExecuteThread_ = std::thread([this,
                                          serverUrl,
                                          order,
                                          planCopy,
                                          sessionId,
                                          captureDir,
                                          captureStills,
                                          stabilizeMs,
                                          wristSweepEnabled,
                                          wristSweepStepDeg,
                                          wristSweepStepsEachWay]() {
            int executed = 0;
            int skipped = 0;
            int captured = 0;
            int wristSkipped = 0;
            QString errorMessage;
            bool ok = true;
            const int total = static_cast<int>(order.size());
            TransformsJsonDocument transformsDoc;
            const std::vector<double> wristOffsets =
                wristSweepOffsetsRad(wristSweepStepsEachWay, wristSweepStepDeg);

            const auto sessionActive = [this, sessionId]() {
                return !shutdownRequested_.load(std::memory_order_acquire)
                       && sessionId == scanExecuteSessionId_.load(std::memory_order_acquire);
            };

            const auto finishWithCapture = [this, &transformsDoc, captureStills, captureDir](
                                               bool finishOk,
                                               const QString &finishError,
                                               int executedCount,
                                               bool stopped,
                                               int capturedCount,
                                               qint64 elapsedMs) {
                if (captureStills && !transformsDoc.frames.empty())
                {
                    QString writeError;
                    if (!writeTransformsJson(captureDir, transformsDoc, &writeError))
                    {
                        QMetaObject::invokeMethod(
                            this,
                            [this, writeError]() {
                                host_->appendLog(
                                    QStringLiteral("UR3e scan capture: transforms.json failed — %1")
                                        .arg(writeError));
                            },
                            Qt::QueuedConnection);
                    }
                    else
                    {
                        QMetaObject::invokeMethod(
                            this,
                            [this, captureDir, capturedCount]() {
                                host_->appendLog(
                                    QStringLiteral(
                                        "UR3e scan capture: wrote %1 frame(s) + transforms.json → %2")
                                        .arg(capturedCount)
                                        .arg(captureDir));
                            },
                            Qt::QueuedConnection);
                    }
                }
                QMetaObject::invokeMethod(
                    this,
                    "scanExecuteFinish",
                    Qt::QueuedConnection,
                    Q_ARG(bool, finishOk),
                    Q_ARG(QString, finishError),
                    Q_ARG(int, executedCount),
                    Q_ARG(bool, stopped),
                    Q_ARG(int, capturedCount),
                    Q_ARG(qint64, elapsedMs));
            };

            const auto captureStillAtPose = [&](const Ur3eScanTcpPose &plannedTcp,
                                                const int pointIndex) -> bool {
                if (!captureStills)
                    return true;

                hf::bfs::BfsRgbFrame frame;
                bool gotFrame = false;
                QMetaObject::invokeMethod(
                    this,
                    [this, &frame, &gotFrame]() {
                        if (host_->bfsPanel() != nullptr)
                            gotFrame = host_->bfsPanel()->tryCopyLastFrame(frame);
                    },
                    Qt::BlockingQueuedConnection);

                if (!gotFrame)
                {
                    QMetaObject::invokeMethod(
                        this,
                        [this, pointIndex]() {
                            host_->appendLog(
                                QStringLiteral(
                                    "UR3e scan capture: no BFS frame at pin %1 — skipping still.")
                                    .arg(pointIndex));
                        },
                        Qt::QueuedConnection);
                    return true;
                }

                Ur3eScanTcpPose tcpForPose = plannedTcp;
                const Ur3ePoseResult livePose = ur3eGetTcpPose(serverUrl);
                if (livePose.ok)
                    tcpForPose = scanTcpFromLivePose(livePose.pose, plannedTcp);

                const QString stem =
                    QStringLiteral("%1").arg(captured, 5, 10, QLatin1Char('0'));
                const QString tiffPath =
                    QDir(captureDir).filePath(stem + QStringLiteral(".tif"));
                const std::string saveError =
                    hf::bfs::saveRgb8AsTiff(tiffPath,
                                           frame.width,
                                           frame.height,
                                           frame.rgb.data(),
                                           frame.rgb.size());
                if (!saveError.empty())
                {
                    ok = false;
                    errorMessage =
                        QStringLiteral("Failed to save BFS TIFF %1: %2")
                            .arg(tiffPath, QString::fromStdString(saveError));
                    return false;
                }

                const Mat4 c2w = cameraToWorldOpenGlFromTcp(tcpForPose);
                const QString imageName = stem + QStringLiteral(".tif");
                const QString poseJsonPath =
                    QDir(captureDir).filePath(stem + QStringLiteral(".json"));
                QString poseError;
                if (!writeCameraPoseJson(poseJsonPath,
                                         tcpForPose,
                                         c2w,
                                         imageName,
                                         frame.width,
                                         frame.height,
                                         &poseError))
                {
                    ok = false;
                    errorMessage =
                        QStringLiteral("Failed to save pose JSON %1: %2")
                            .arg(poseJsonPath, poseError);
                    return false;
                }

                if (transformsDoc.width <= 0)
                {
                    transformsDoc.width = frame.width;
                    transformsDoc.height = frame.height;
                }
                TransformsJsonFrame entry;
                entry.filePathStem = stem;
                entry.transformMatrix = c2w;
                transformsDoc.frames.push_back(std::move(entry));
                ++captured;
                return true;
            };

            QMetaObject::invokeMethod(
                this,
                [this]() {
                    host_->appendLog(
                        QStringLiteral("UR3e scan execute: verifying scan home before scan…"));
                    syncHomeJointTargetSliders();
                },
                Qt::BlockingQueuedConnection);

            const HomeEnsureOutcome preHomeOutcome =
                ensureRobotAtHomeSync(HomeEnsureContext::BeforeScanExecute);
            if (preHomeOutcome.cancelled)
            {
                finishWithCapture(false,
                                  QStringLiteral("Scan aborted — homing cancelled."),
                                  0,
                                  false,
                                  0,
                                  0);
                return;
            }

            QMetaObject::invokeMethod(
                this, &Ur3ePanelController::syncHomeJointTargetSliders, Qt::QueuedConnection);

            bool returnHomeAfterScan = true;
            const auto scanStartedAt = std::chrono::steady_clock::now();

            for (int step = 0; step < total; ++step)
            {
                if (!sessionActive() || stopRequested_.load(std::memory_order_acquire))
                {
                    ur3eStopMotion(serverUrl);
                    break;
                }

                const int pointIndex = order[static_cast<std::size_t>(step)];
                const Ur3ePlannedScanPoint &point =
                    planCopy.points[static_cast<std::size_t>(pointIndex)];
                const QString tcpSummary =
                    QStringLiteral("tcp=(%1, %2, %3) m theta=%4° phi=%5°")
                        .arg(point.tcp.xM, 0, 'f', 3)
                        .arg(point.tcp.yM, 0, 'f', 3)
                        .arg(point.tcp.zM, 0, 'f', 3)
                        .arg(point.gridPoint.thetaDeg, 0, 'f', 1)
                        .arg(point.gridPoint.phiDeg, 0, 'f', 1);
                const QString targetSummary = formatJointTargetsDeg(point.jointPositionsRad);

                QVariantList targetPositionsVariant;
                targetPositionsVariant.reserve(
                    static_cast<int>(point.jointPositionsRad.size()));
                for (const double value : point.jointPositionsRad)
                    targetPositionsVariant.append(value);

                if (!sessionActive())
                    break;

                QMetaObject::invokeMethod(
                    this,
                    "scanExecuteSetActivePoint",
                    Qt::QueuedConnection,
                    Q_ARG(int, pointIndex));
                QMetaObject::invokeMethod(
                    this,
                    "scanExecuteLogMoving",
                    Qt::QueuedConnection,
                    Q_ARG(int, step + 1),
                    Q_ARG(int, total),
                    Q_ARG(int, pointIndex),
                    Q_ARG(QString, tcpSummary),
                    Q_ARG(QString, targetSummary),
                    Q_ARG(QVariantList, targetPositionsVariant));

                const Ur3eScanWaypointMoveResult moveResult = ur3eExecuteScanWaypoint(
                    serverUrl,
                    point.jointPositionsRad,
                    &point.tcp,
                    nullptr,
                    false,
                    false);
                if (moveResult.stopped)
                {
                    if (stopRequested_.load(std::memory_order_acquire))
                    {
                        ur3eStopMotion(serverUrl);
                    }
                    else
                    {
                        ok = false;
                        errorMessage = moveResult.errorMessage.isEmpty()
                                           ? QStringLiteral("MoveIt execution stopped before motion started.")
                                           : moveResult.errorMessage;
                    }
                    break;
                }
                if (moveResult.skipped)
                {
                    const int stepNum = step + 1;
                    const QString reason = moveResult.errorMessage.isEmpty()
                                               ? QStringLiteral("no collision-free path")
                                               : moveResult.errorMessage;
                    QMetaObject::invokeMethod(
                        this,
                        [this, stepNum, total, pointIndex, reason]() {
                            host_->appendLog(
                                QStringLiteral(
                                    "UR3e scan execute [%1/%2] pt %3: skipped — %4")
                                    .arg(stepNum)
                                    .arg(total)
                                    .arg(pointIndex)
                                    .arg(reason));
                        },
                        Qt::QueuedConnection);
                    QMetaObject::invokeMethod(
                        this,
                        "scanExecuteMarkFailed",
                        Qt::QueuedConnection,
                        Q_ARG(int, pointIndex));
                    ++skipped;
                    continue;
                }
                if (!moveResult.ok)
                {
                    ok = false;
                    errorMessage = moveResult.errorMessage;
                    returnHomeAfterScan = false;
                    break;
                }

                if (!sessionActive())
                    break;

                const Ur3eJointsState joints = ur3eGetJoints(serverUrl);
                const Ur3ePoseResult pose = ur3eGetTcpPose(serverUrl);
                const QString arrivedJoints = joints.ok ? formatJointTargetsDeg(joints.positionsRad)
                                                        : QStringLiteral("(n/a)");
                const QString arrivedPose =
                    pose.ok ? formatTcpPose(pose.pose) : QStringLiteral("(n/a)");

                QVariantList positionsVariant;
                if (joints.ok)
                {
                    positionsVariant.reserve(static_cast<int>(joints.positionsRad.size()));
                    for (const double value : joints.positionsRad)
                        positionsVariant.append(value);
                }

                QMetaObject::invokeMethod(
                    this,
                    "scanExecuteLogArrived",
                    Qt::QueuedConnection,
                    Q_ARG(int, step + 1),
                    Q_ARG(int, total),
                    Q_ARG(int, pointIndex),
                    Q_ARG(QString, arrivedPose),
                    Q_ARG(QString, arrivedJoints),
                    Q_ARG(QVariantList, positionsVariant),
                    Q_ARG(QStringList, joints.names));

                std::this_thread::sleep_for(std::chrono::milliseconds(stabilizeMs));

                if (!sessionActive() || stopRequested_.load(std::memory_order_acquire))
                {
                    ur3eStopMotion(serverUrl);
                    break;
                }

                if (!captureStillAtPose(point.tcp, pointIndex))
                {
                    returnHomeAfterScan = true;
                    break;
                }

                if (wristSweepEnabled && !wristOffsets.empty()
                    && point.jointPositionsRad.size() >= 6)
                {
                    for (const double dWrist2 : wristOffsets)
                    {
                        for (const double dWrist3 : wristOffsets)
                        {
                            if (!sessionActive()
                                || stopRequested_.load(std::memory_order_acquire))
                            {
                                ur3eStopMotion(serverUrl);
                                goto scan_execute_loop_done;
                            }

                            std::vector<double> wristJoints = point.jointPositionsRad;
                            wristJoints[4] += dWrist2;
                            wristJoints[5] += dWrist3;

                            const Ur3eScanWaypointMoveResult wristMove =
                                ur3eExecuteScanWaypoint(serverUrl,
                                                        wristJoints,
                                                        nullptr,
                                                        nullptr,
                                                        false,
                                                        true);
                            if (wristMove.stopped)
                            {
                                if (stopRequested_.load(std::memory_order_acquire))
                                    ur3eStopMotion(serverUrl);
                                else
                                {
                                    ok = false;
                                    errorMessage = wristMove.errorMessage.isEmpty()
                                                       ? QStringLiteral(
                                                             "Wrist sweep stopped before motion.")
                                                       : wristMove.errorMessage;
                                }
                                goto scan_execute_loop_done;
                            }
                            if (!wristMove.ok)
                            {
                                ++wristSkipped;
                                const QString reason = wristMove.errorMessage.isEmpty()
                                                           ? QStringLiteral("collision / no path")
                                                           : wristMove.errorMessage;
                                const double w2Deg = dWrist2 * (180.0 / 3.14159265358979323846);
                                const double w3Deg = dWrist3 * (180.0 / 3.14159265358979323846);
                                QMetaObject::invokeMethod(
                                    this,
                                    [this, pointIndex, w2Deg, w3Deg, reason]() {
                                        host_->appendLog(
                                            QStringLiteral(
                                                "UR3e scan wrist sweep pin %1: skip Δw2=%2° "
                                                "Δw3=%3° — %4")
                                                .arg(pointIndex)
                                                .arg(w2Deg, 0, 'f', 1)
                                                .arg(w3Deg, 0, 'f', 1)
                                                .arg(reason));
                                    },
                                    Qt::QueuedConnection);
                                continue;
                            }

                            std::this_thread::sleep_for(std::chrono::milliseconds(stabilizeMs));

                            if (!sessionActive()
                                || stopRequested_.load(std::memory_order_acquire))
                            {
                                ur3eStopMotion(serverUrl);
                                goto scan_execute_loop_done;
                            }

                            if (!captureStillAtPose(point.tcp, pointIndex))
                            {
                                returnHomeAfterScan = true;
                                goto scan_execute_loop_done;
                            }
                        }
                    }

                    // Return to nominal pin joints before the next pin.
                    if (sessionActive() && !stopRequested_.load(std::memory_order_acquire))
                    {
                        const Ur3eScanWaypointMoveResult returnPin =
                            ur3eExecuteScanWaypoint(serverUrl,
                                                    point.jointPositionsRad,
                                                    nullptr,
                                                    nullptr,
                                                    false,
                                                    true);
                        if (returnPin.stopped)
                        {
                            if (stopRequested_.load(std::memory_order_acquire))
                                ur3eStopMotion(serverUrl);
                            goto scan_execute_loop_done;
                        }
                        if (!returnPin.ok)
                        {
                            const QString reason = returnPin.errorMessage.isEmpty()
                                                       ? QStringLiteral("could not return to pin")
                                                       : returnPin.errorMessage;
                            QMetaObject::invokeMethod(
                                this,
                                [this, pointIndex, reason]() {
                                    host_->appendLog(
                                        QStringLiteral(
                                            "UR3e scan wrist sweep pin %1: return-to-pin "
                                            "warning — %2")
                                            .arg(pointIndex)
                                            .arg(reason));
                                },
                                Qt::QueuedConnection);
                        }
                    }
                }

                QMetaObject::invokeMethod(
                    this,
                    "scanExecuteMarkCompleted",
                    Qt::QueuedConnection,
                    Q_ARG(int, pointIndex));

                ++executed;
            }

        scan_execute_loop_done:
            if (!sessionActive())
                return;

            // User Stop must always retreat to home after cancelling the current motion.
            const bool userStopped = stopRequested_.load(std::memory_order_acquire);
            if (userStopped)
                returnHomeAfterScan = true;

            if (returnHomeAfterScan)
            {
                QMetaObject::invokeMethod(
                    this,
                    [this, userStopped]() {
                        host_->appendLog(
                            userStopped
                                ? QStringLiteral(
                                      "UR3e scan execute: stop — returning to home pose…")
                                : QStringLiteral(
                                      "UR3e scan execute: returning to home pose…"));
                        syncHomeJointTargetSliders();
                    },
                    Qt::BlockingQueuedConnection);

                // /execute_move_home clears the sidecar stop latch so MoveIt can run again.
                const Ur3eScanWaypointMoveResult postHomeResult = ur3eExecuteMoveHome(serverUrl);
                if (postHomeResult.ok)
                {
                    QMetaObject::invokeMethod(
                        this,
                        &Ur3ePanelController::syncHomeJointTargetSliders,
                        Qt::QueuedConnection);
                }
                if (!postHomeResult.ok && !postHomeResult.stopped)
                {
                    const QString reason = postHomeResult.errorMessage.isEmpty()
                                               ? QStringLiteral("could not return to home")
                                               : postHomeResult.errorMessage;
                    QMetaObject::invokeMethod(
                        this,
                        [this, reason]() {
                            host_->appendLog(
                                QStringLiteral("UR3e scan execute warning: %1").arg(reason));
                        },
                        Qt::QueuedConnection);
                }
            }

            if (skipped > 0)
            {
                const int skippedCount = skipped;
                QMetaObject::invokeMethod(
                    this,
                    [this, skippedCount]() {
                        host_->appendLog(
                            QStringLiteral(
                                "UR3e scan execute: %1 pin(s) skipped (no collision-free path).")
                                .arg(skippedCount));
                    },
                    Qt::QueuedConnection);
            }
            if (wristSkipped > 0)
            {
                const int wristSkippedCount = wristSkipped;
                QMetaObject::invokeMethod(
                    this,
                    [this, wristSkippedCount]() {
                        host_->appendLog(
                            QStringLiteral(
                                "UR3e scan execute: %1 wrist pose(s) skipped (collision / no path).")
                                .arg(wristSkippedCount));
                    },
                    Qt::QueuedConnection);
            }

            const bool stopped = stopRequested_.load(std::memory_order_acquire);
            const qint64 elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                        std::chrono::steady_clock::now() - scanStartedAt)
                                        .count();
            finishWithCapture(ok, errorMessage, executed, stopped || userStopped, captured,
                              elapsedMs);
        });
    }

    return true;
}

void Ur3ePanelController::scanExecuteSetActivePoint(const int pointIndex)
{
    if (!scanExecuting_)
        return;

    if (host_->ur3eScanRoutePlanWidget_ != nullptr)
        host_->ur3eScanRoutePlanWidget_->setActiveScanPoint(pointIndex);
}

void Ur3ePanelController::scanExecuteMarkCompleted(const int pointIndex)
{
    if (!scanExecuting_)
        return;

    if (host_->ur3eScanRoutePlanWidget_ != nullptr)
        host_->ur3eScanRoutePlanWidget_->markScanPointCompleted(pointIndex);
}

void Ur3ePanelController::scanExecuteMarkFailed(const int pointIndex)
{
    if (!scanExecuting_)
        return;

    if (host_->ur3eScanRoutePlanWidget_ != nullptr)
        host_->ur3eScanRoutePlanWidget_->markScanPointFailed(pointIndex);
}

void Ur3ePanelController::scanExecuteLogMoving(const int step,
                                               const int total,
                                               const int pointIndex,
                                               const QString &tcpSummary,
                                               const QString &targetSummary,
                                               const QVariantList &targetPositionsRad)
{
    if (!scanExecuting_)
        return;

    if (!targetPositionsRad.isEmpty())
    {
        std::vector<double> targets;
        targets.reserve(static_cast<std::size_t>(targetPositionsRad.size()));
        for (const QVariant &value : targetPositionsRad)
            targets.push_back(value.toDouble());
        applyJointTargets(targets);
        pollJoints();
    }

    host_->appendLog(
        QStringLiteral("UR3e scan execute [%1/%2] pt %3: MoveIt moving — %4; target %5")
            .arg(step)
            .arg(total)
            .arg(pointIndex)
            .arg(tcpSummary)
            .arg(targetSummary));
}

void Ur3ePanelController::scanExecuteLogArrived(const int step,
                                              const int total,
                                              const int pointIndex,
                                              const QString &arrivedPose,
                                              const QString &arrivedJoints,
                                              const QVariantList &positionsRad,
                                              const QStringList &names)
{
    if (!scanExecuting_)
        return;

    host_->appendLog(
        QStringLiteral("UR3e scan execute [%1/%2] pt %3: arrived pose %4; joints %5")
            .arg(step)
            .arg(total)
            .arg(pointIndex)
            .arg(arrivedPose)
            .arg(arrivedJoints));

    if (positionsRad.isEmpty())
        return;

    std::vector<double> positions;
    positions.reserve(static_cast<std::size_t>(positionsRad.size()));
    for (const QVariant &value : positionsRad)
        positions.push_back(value.toDouble());

    applyJointPositions(positions, names, false);
}

void Ur3ePanelController::scanExecuteFinish(const bool ok,
                                            const QString &errorMessage,
                                            const int executedCount,
                                            const bool stopped,
                                            const int capturedFrameCount,
                                            const qint64 elapsedMs)
{
    if (!scanExecuting_)
        return;

    {
        std::lock_guard<std::mutex> lock(scanExecuteThreadMutex_);
        if (scanExecuteThread_.joinable())
            scanExecuteThread_.join();
    }

    scanExecuting_ = false;
    stopRequested_.store(false, std::memory_order_release);
    if (host_->ur3eScanRoutePlanWidget_ != nullptr)
        host_->ur3eScanRoutePlanWidget_->endScanExecution();

    QString detail;
    if (!ok)
    {
        if (errorMessage.isEmpty())
            detail = QStringLiteral("scan execute failed.");
        else if (executedCount > 0)
            detail = QStringLiteral("Failed after %1 completed waypoint(s): %2")
                         .arg(executedCount)
                         .arg(errorMessage);
        else
            detail = errorMessage;
    }
    else if (stopped)
        detail = QStringLiteral("scan stopped after %1 waypoint(s).").arg(executedCount);
    else
        detail = QStringLiteral("scan complete — %1 waypoint(s).").arg(executedCount);

    if (capturedFrameCount > 0)
        detail += QStringLiteral(" Captured %1 BFS frame(s).").arg(capturedFrameCount);
    if (elapsedMs > 0)
    {
        const qint64 totalSec = elapsedMs / 1000;
        const qint64 minutes = totalSec / 60;
        const qint64 seconds = totalSec % 60;
        detail += QStringLiteral(" Time %1:%2.")
                      .arg(minutes)
                      .arg(seconds, 2, 10, QLatin1Char('0'));
    }

    finishScanExecute(ok, detail, capturedFrameCount, executedCount, elapsedMs, stopped);
}

void Ur3ePanelController::finishScanExecute(const bool ok,
                                            const QString &detail,
                                            const int capturedFrameCount,
                                            const int successfulPins,
                                            const qint64 elapsedMs,
                                            const bool stopped)
{
    setBusy(false);
    setJointPollIntervalMs(kPosePollIntervalMs);
    host_->appendLog(QStringLiteral("UR3e scan execute: %1").arg(detail));

    const qint64 totalSec = (elapsedMs > 0 ? elapsedMs : qint64{0}) / 1000;
    const qint64 minutes = totalSec / 60;
    const qint64 seconds = totalSec % 60;
    const QString durationText =
        QStringLiteral("%1:%2").arg(minutes).arg(seconds, 2, 10, QLatin1Char('0'));

    QString summaryTitle;
    if (!ok)
        summaryTitle = QStringLiteral("UR3e Scan Failed");
    else if (stopped)
        summaryTitle = QStringLiteral("UR3e Scan Stopped");
    else
        summaryTitle = QStringLiteral("UR3e Scan Complete");

    const QString summaryBody =
        QStringLiteral("Successful pins: %1\n"
                       "Multiview images recorded: %2\n"
                       "Scanning time: %3")
            .arg(successfulPins)
            .arg(capturedFrameCount)
            .arg(durationText);

    if (!ok)
        QMessageBox::warning(host_, summaryTitle, summaryBody + QStringLiteral("\n\n") + detail);
    else
        QMessageBox::information(host_, summaryTitle, summaryBody);

    if (ok)
        pollJoints();
    updateRobotUi();
    emit hemisphereScanExecuteFinished(ok, detail, capturedFrameCount);
}

void Ur3ePanelController::onMoveItStateChanged(const bool running, const QString &detail)
{
    if (running)
        pushWorkspaceBoundaryToMoveIt();

    if (detail.isEmpty())
    {
        updateRobotUi();
        return;
    }

    const QStringList lines = detail.split(QLatin1Char('\n'), Qt::SkipEmptyParts);
    for (const QString &line : lines)
    {
        const QString trimmed = line.trimmed();
        if (!trimmed.isEmpty())
            host_->appendLog(QStringLiteral("UR3e MoveIt: %1").arg(trimmed));
    }
    updateRobotUi();
}

void Ur3ePanelController::onStartRvizRequested()
{
    if (rvizManager_ == nullptr || busy_)
        return;

    if (rvizManager_->isRunning())
    {
        host_->appendLog(QStringLiteral("UR3e: stopping RViz\u2026"));
        rvizManager_->stop();
        updateRobotUi();
        return;
    }

    if (!isSidecarRunning())
    {
        host_->appendLog(QStringLiteral("UR3e: sidecar not ready — cannot start RViz."));
        return;
    }

    if (!robotConnected_)
    {
        host_->appendLog(QStringLiteral(
            "UR3e: connect the robot first (RViz needs the running UR driver for the model)."));
        return;
    }

    host_->appendLog(QStringLiteral("UR3e: starting RViz (visualization only) in WSL\u2026"));
    rvizManager_->start();
    updateRobotUi();
}

void Ur3ePanelController::onRvizStateChanged(const bool running, const QString &detail)
{
    if (detail.isEmpty())
    {
        updateRobotUi();
        Q_UNUSED(running);
        return;
    }

    const QStringList lines = detail.split(QLatin1Char('\n'), Qt::SkipEmptyParts);
    for (const QString &line : lines)
    {
        const QString trimmed = line.trimmed();
        if (!trimmed.isEmpty())
            host_->appendLog(QStringLiteral("UR3e RViz: %1").arg(trimmed));
    }
    updateRobotUi();
    Q_UNUSED(running);
}

void Ur3ePanelController::onConnectRequested()
{
    if (serverManager_ == nullptr || busy_ || connectInProgress_)
        return;

    if (!serverManager_->isServerConnected())
    {
        host_->appendLog(QStringLiteral("UR3e: sidecar not ready yet — wait for WSL startup (see UR3e log tab)."));
        return;
    }

    const hf::HardwareConfig::Ur3eConfig &cfg = hf::hardwareConfig().ur3e;
    if (cfg.prestartDriver && !driverPrestartReady_)
    {
        host_->appendLog(
            cfg.useMockHardware
                ? QStringLiteral(
                      "UR3e: simulation driver still warming up — wait for \"simulation driver ready\" in the log.")
                : QStringLiteral(
                      "UR3e: robot driver still warming up — wait for \"robot driver ready\" in the log."));
        return;
    }

    const QString robotIp = host_->ur3eRobotIpEdit_ != nullptr
                                ? host_->ur3eRobotIpEdit_->text().trimmed()
                                : cfg.robotIp;
    const int connectTimeoutMs = qMax(30000, cfg.connectTimeoutMs);
    const int connectTimeoutSec = connectTimeoutMs / 1000;

    host_->appendLog(
        cfg.useMockHardware
            ? QStringLiteral("UR3e: connecting (simulation)\u2026")
            : QStringLiteral(
                  "UR3e: connecting to %1 — press Play on External Control (%2:50002) within %3 min\u2026")
                  .arg(robotIp)
                  .arg(cfg.reverseIp)
                  .arg((connectTimeoutSec + 59) / 60));

    setBusy(true);
    connectInProgress_ = true;
    lastConnectStatusPhase_.clear();
    lastConnectStatusMessage_.clear();
    const int sessionId = ++connectSessionId_;
    connectDeadlineMs_ = QDateTime::currentMSecsSinceEpoch() + connectTimeoutMs;

    dismissConnectWaitDialog();
    connectWaitDialog_ = new Ur3eExternalControlWaitDialog(host_);
    connectWaitDialog_->configure(cfg.useMockHardware, cfg.reverseIp, connectTimeoutSec);
    connect(connectWaitDialog_,
            &Ur3eExternalControlWaitDialog::cancelRequested,
            this,
            &Ur3ePanelController::onConnectDialogCancelled);
    connectWaitDialog_->show();
    connectWaitDialog_->raise();
    connectWaitDialog_->activateWindow();

    const QString serverUrl = serverManager_->serverUrl();
    std::thread([this, serverUrl, robotIp, sessionId]() {
        QString error;
        Ur3eConnectAsyncStatus status;
        bool started = ur3eConnectStart(serverUrl, robotIp, &status, &error);
        bool legacyConnect = false;
        Ur3eConnectResult legacyResult;
        if (!started && error.contains(QStringLiteral("not found"), Qt::CaseInsensitive))
        {
            legacyConnect = true;
            legacyResult = ur3eConnectRobot(serverUrl, robotIp, &error);
        }
        const bool legacyOk = legacyConnect && legacyResult.ok;
        const QString legacyDetail =
            legacyOk ? (legacyResult.useMockHardware ? QStringLiteral("connected (simulation)")
                                                     : QStringLiteral("connected (hardware)"))
                     : error;
        QMetaObject::invokeMethod(
            this,
            [this, started, status, error, sessionId, legacyConnect, legacyOk, legacyDetail]() {
                if (sessionId != connectSessionId_.load())
                    return;
                if (legacyConnect)
                {
                    dismissConnectWaitDialog();
                    connectInProgress_ = false;
                    if (connectPollTimer_ != nullptr)
                        connectPollTimer_->stop();
                    if (connectCountdownTimer_ != nullptr)
                        connectCountdownTimer_->stop();
                    finishConnect(legacyOk, legacyDetail);
                    return;
                }
                if (!started)
                {
                    dismissConnectWaitDialog();
                    connectInProgress_ = false;
                    if (connectPollTimer_ != nullptr)
                        connectPollTimer_->stop();
                    if (connectCountdownTimer_ != nullptr)
                        connectCountdownTimer_->stop();
                    finishConnect(false, error);
                    return;
                }

                applyConnectAsyncStatus(status);
                if (status.ok && !status.inProgress)
                {
                    dismissConnectWaitDialog();
                    connectInProgress_ = false;
                    if (connectPollTimer_ != nullptr)
                        connectPollTimer_->stop();
                    if (connectCountdownTimer_ != nullptr)
                        connectCountdownTimer_->stop();
                    finishConnect(
                        true,
                        status.useMockHardware ? QStringLiteral("connected (simulation)")
                                               : QStringLiteral("connected (hardware)"));
                    return;
                }

                if (connectPollTimer_ != nullptr)
                    connectPollTimer_->start();
                if (connectCountdownTimer_ != nullptr)
                    connectCountdownTimer_->start();
                onConnectCountdownTick();
            },
            Qt::QueuedConnection);
    }).detach();
}

void Ur3ePanelController::dismissConnectWaitDialog()
{
    if (connectWaitDialog_ == nullptr)
        return;

    connectWaitDialog_->dismiss();
    connectWaitDialog_->deleteLater();
    connectWaitDialog_ = nullptr;
}

void Ur3ePanelController::applyConnectAsyncStatus(const Ur3eConnectAsyncStatus &status)
{
    if (connectWaitDialog_ == nullptr)
        return;

    const QString phase = status.phase.trimmed().toLower();
    if (!status.message.isEmpty() && status.message != lastConnectStatusMessage_)
    {
        lastConnectStatusMessage_ = status.message;
        connectWaitDialog_->setDetailText(status.message);
    }

    if (phase == lastConnectStatusPhase_)
        return;
    lastConnectStatusPhase_ = phase;

    const bool mock = status.useMockHardware;
    if (phase == QStringLiteral("waiting_external_control")
        || (status.scriptPortListening && !status.reverseConnected && !mock))
    {
        connectWaitDialog_->setPhase(Ur3eExternalControlWaitDialog::Phase::PressPlay);
    }
    else if (phase == QStringLiteral("bridge_setup") || phase == QStringLiteral("finishing")
             || status.reverseConnected)
    {
        connectWaitDialog_->setPhase(Ur3eExternalControlWaitDialog::Phase::Finishing);
    }
    else if (phase == QStringLiteral("starting_driver"))
    {
        connectWaitDialog_->setPhase(mock ? Ur3eExternalControlWaitDialog::Phase::SimulationStarting
                                          : Ur3eExternalControlWaitDialog::Phase::StartingDriver);
    }
}

void Ur3ePanelController::updateConnectDialogFromSidecarLine(const QString &line)
{
    if (!connectInProgress_ || connectWaitDialog_ == nullptr)
        return;

    const QString lower = line.toLower();
    if (lower.contains(QStringLiteral("port 50002 listening"))
        || lower.contains(QStringLiteral("press play on external control")))
    {
        connectWaitDialog_->setPhase(Ur3eExternalControlWaitDialog::Phase::PressPlay);
    }
    else if (lower.contains(QStringLiteral("external control connected"))
             || lower.contains(QStringLiteral("trajectory action server ready")))
    {
        connectWaitDialog_->setPhase(Ur3eExternalControlWaitDialog::Phase::Finishing);
    }
    else if (lower.contains(QStringLiteral("starting ur_robot_driver"))
             || lower.contains(QStringLiteral("waiting for controller manager")))
    {
        connectWaitDialog_->setPhase(Ur3eExternalControlWaitDialog::Phase::StartingDriver);
    }
    else if (lower.contains(QStringLiteral("connect phase=waiting_external_control")))
    {
        connectWaitDialog_->setPhase(Ur3eExternalControlWaitDialog::Phase::PressPlay);
    }
    else if (lower.contains(QStringLiteral("connect phase=bridge_setup"))
             || lower.contains(QStringLiteral("connect phase=finishing")))
    {
        connectWaitDialog_->setPhase(Ur3eExternalControlWaitDialog::Phase::Finishing);
    }

    if (line.contains(QStringLiteral("connect phase="), Qt::CaseInsensitive))
        connectWaitDialog_->setDetailText(line);
}

void Ur3ePanelController::onConnectPollTick()
{
    if (!connectInProgress_ || serverManager_ == nullptr)
        return;

    // One wsl.exe/curl at a time — spawning every 500 ms without this guard stalls the UI.
    if (connectPollInFlight_.exchange(true, std::memory_order_acq_rel))
        return;

    const int sessionId = connectSessionId_.load();
    const QString serverUrl = serverManager_->serverUrl();
    std::thread([this, serverUrl, sessionId]() {
        Ur3eConnectAsyncStatus status;
        QString error;
        const bool polled = ur3eConnectStatus(serverUrl, &status, &error);
        QMetaObject::invokeMethod(
            this,
            [this, polled, status, error, sessionId]() {
                connectPollInFlight_.store(false, std::memory_order_release);
                if (sessionId != connectSessionId_.load() || !connectInProgress_)
                    return;
                if (!polled)
                {
                    if (connectWaitDialog_ != nullptr)
                        connectWaitDialog_->setDetailText(error);
                    return;
                }

                applyConnectAsyncStatus(status);
                const QString phase = status.phase.trimmed().toLower();
                if (status.ok && phase == QStringLiteral("complete"))
                {
                    if (connectPollTimer_ != nullptr)
                        connectPollTimer_->stop();
                    if (connectCountdownTimer_ != nullptr)
                        connectCountdownTimer_->stop();
                    dismissConnectWaitDialog();
                    connectInProgress_ = false;
                    finishConnect(
                        true,
                        status.useMockHardware ? QStringLiteral("connected (simulation)")
                                               : QStringLiteral("connected (hardware)"));
                    return;
                }

                if (phase == QStringLiteral("failed"))
                {
                    if (connectPollTimer_ != nullptr)
                        connectPollTimer_->stop();
                    if (connectCountdownTimer_ != nullptr)
                        connectCountdownTimer_->stop();
                    dismissConnectWaitDialog();
                    connectInProgress_ = false;
                    finishConnect(false, status.errorMessage.isEmpty() ? status.message : status.errorMessage);
                    return;
                }

                if (phase == QStringLiteral("cancelled"))
                {
                    if (connectPollTimer_ != nullptr)
                        connectPollTimer_->stop();
                    if (connectCountdownTimer_ != nullptr)
                        connectCountdownTimer_->stop();
                    dismissConnectWaitDialog();
                    connectInProgress_ = false;
                    setBusy(false);
                    host_->appendLog(QStringLiteral("UR3e connect cancelled."));
                    updateRobotUi();
                }
            },
            Qt::QueuedConnection);
    }).detach();
}

void Ur3ePanelController::onConnectCountdownTick()
{
    if (!connectInProgress_ || connectWaitDialog_ == nullptr)
        return;

    const qint64 remainingMs = connectDeadlineMs_ - QDateTime::currentMSecsSinceEpoch();
    const int remainingSec = static_cast<int>((remainingMs + 999) / 1000);
    connectWaitDialog_->setRemainingSeconds(remainingSec);
    if (remainingSec <= 0)
        onConnectTimedOut();
}

void Ur3ePanelController::onConnectDialogCancelled()
{
    if (!connectInProgress_)
        return;

    ++connectSessionId_;
    connectPollInFlight_.store(false, std::memory_order_release);
    if (connectPollTimer_ != nullptr)
        connectPollTimer_->stop();
    if (connectCountdownTimer_ != nullptr)
        connectCountdownTimer_->stop();

    host_->appendLog(QStringLiteral("UR3e: connect cancelled by user."));
    dismissConnectWaitDialog();

    if (serverManager_ != nullptr)
    {
        const QString serverUrl = serverManager_->serverUrl();
        std::thread([this, serverUrl]() {
            QString error;
            (void)ur3eConnectCancel(serverUrl, &error);
            QMetaObject::invokeMethod(
                this,
                [this]() {
                    connectInProgress_ = false;
                    setBusy(false);
                    updateRobotUi();
                },
                Qt::QueuedConnection);
        }).detach();
    }
    else
    {
        connectInProgress_ = false;
        setBusy(false);
        updateRobotUi();
    }
}

void Ur3ePanelController::onConnectTimedOut()
{
    if (!connectInProgress_)
        return;

    ++connectSessionId_;
    connectPollInFlight_.store(false, std::memory_order_release);
    if (connectPollTimer_ != nullptr)
        connectPollTimer_->stop();
    if (connectCountdownTimer_ != nullptr)
        connectCountdownTimer_->stop();

    dismissConnectWaitDialog();
    const bool useMockHardware = hf::hardwareConfig().ur3e.useMockHardware;
    host_->appendLog(
        useMockHardware
            ? QStringLiteral("UR3e: connect timed out waiting for simulation driver.")
            : QStringLiteral("UR3e: connect timed out waiting for External Control Play."));

    if (serverManager_ != nullptr)
    {
        const QString serverUrl = serverManager_->serverUrl();
        std::thread([this, serverUrl, useMockHardware]() {
            QString error;
            (void)ur3eConnectCancel(serverUrl, &error);
            QMetaObject::invokeMethod(
                this,
                [this, useMockHardware]() {
                    connectInProgress_ = false;
                    finishConnect(
                        false,
                        useMockHardware
                            ? QStringLiteral(
                                  "Simulation connect timed out. The ROS mock driver may still be "
                                  "starting in WSL — wait for \"simulation driver ready\", then try again.")
                            : QStringLiteral(
                                  "Timed out waiting for External Control. On the teach pendant open "
                                  "External Control, confirm remote PC, and press Play while connecting."));
                },
                Qt::QueuedConnection);
        }).detach();
    }
    else
    {
        connectInProgress_ = false;
        finishConnect(false, QStringLiteral("Connect timed out."));
    }
}

void Ur3ePanelController::onDisconnectRequested()
{
    if (serverManager_ == nullptr || busy_ || !robotConnected_)
        return;

    host_->appendLog(QStringLiteral("UR3e: preparing to disconnect\u2026"));
    setBusy(true);
    beginHomeMotionUi();
    const QString serverUrl = serverManager_->serverUrl();
    std::thread([this, serverUrl]() {
        const HomeEnsureOutcome homeOutcome =
            ensureRobotAtHomeSync(HomeEnsureContext::BeforeDisconnect);
        if (homeOutcome.cancelled)
        {
            QMetaObject::invokeMethod(
                this,
                [this]() {
                    endHomeMotionUi();
                    setBusy(false);
                    host_->appendLog(
                        QStringLiteral("UR3e: disconnect cancelled (home positioning)."));
                    pollJointsSync();
                    syncTargetsFromCurrent();
                    updateRobotUi();
                },
                Qt::QueuedConnection);
            return;
        }

        if (!homeOutcome.success && !homeOutcome.alreadyAtHome)
        {
            host_->appendLog(
                QStringLiteral("UR3e: disconnecting without reaching scan home."));
        }

        const Ur3eConnectResult result = ur3eDisconnectRobot(serverUrl);
        const bool ok = result.ok;
        const QString detail = ok ? QStringLiteral("disconnected") : result.errorMessage;
        QMetaObject::invokeMethod(
            this,
            [this, ok, detail]() { finishDisconnect(ok, detail); },
            Qt::QueuedConnection);
    }).detach();
}

void Ur3ePanelController::onStopMotionRequested()
{
    requestStopMotion();
}

void Ur3ePanelController::requestStopMotion()
{
    if (serverManager_ == nullptr || !robotConnected_)
        return;

    stopRequested_.store(true, std::memory_order_release);
    host_->appendLog(QStringLiteral("UR3e: stop requested"));
    updateRobotUi();

    const QString serverUrl = serverManager_->serverUrl();
    std::thread([this, serverUrl]() {
        QString error;
        const bool ok = ur3eStopMotion(serverUrl, &error);
        const QString detail = ok ? QStringLiteral("motion stopped") : error;
        QMetaObject::invokeMethod(
            this,
            [this, ok, detail]() { finishStop(ok, detail); },
            Qt::QueuedConnection);
    }).detach();
}

void Ur3ePanelController::finishConnect(const bool ok, const QString &detail)
{
    if (!ok)
    {
        setBusy(false);
        host_->appendLog(QStringLiteral("UR3e connect failed: %1").arg(detail));
        updateRobotUi();
        return;
    }

    robotConnected_ = true;
    host_->appendLog(QStringLiteral("UR3e: %1").arg(detail));
    pushWorkspaceBoundaryToMoveIt();
    if (host_->ur3ePosePollTimer_ != nullptr)
    {
        host_->ur3ePosePollTimer_->start(kPosePollIntervalMs);
        const Ur3eJointsState joints = ur3eGetJoints(serverManager_->serverUrl());
        if (joints.ok && joints.positionsRad.size() >= MainWindow::kUr3eJointCount)
            applyJointPositions(joints.positionsRad, joints.names, false);
        else
            pollJointsSync();

        const hf::HardwareConfig::Ur3eConfig &cfg = hf::hardwareConfig().ur3e;
        if (cfg.useMockHardware)
            applyConfiguredInitialJointTargets();
        else
            syncTargetsFromCurrent();
    }

    updateRobotUi();
    host_->appendLog(QStringLiteral("UR3e: verifying scan home position\u2026"));
    setBusy(true);
    beginHomeMotionUi();

    std::thread([this]() {
        const HomeEnsureOutcome outcome = ensureRobotAtHomeSync(HomeEnsureContext::AfterConnect);
        QMetaObject::invokeMethod(
            this,
            [this, outcome]() { finishHomeEnsureAfterConnect(outcome); },
            Qt::QueuedConnection);
    }).detach();
}

Ur3ePanelController::HomeEnsureOutcome Ur3ePanelController::ensureRobotAtHomeSync(
    const HomeEnsureContext context)
{
    HomeEnsureOutcome outcome;
    if (serverManager_ == nullptr || !serverManager_->isServerConnected() || !robotConnected_)
    {
        outcome.success = true;
        return outcome;
    }

    const QString serverUrl = serverManager_->serverUrl();
    for (;;)
    {
        if (context != HomeEnsureContext::BeforeShutdown
            && shutdownRequested_.load(std::memory_order_acquire))
        {
            outcome.cancelled = true;
            return outcome;
        }

        host_->appendLog(QStringLiteral("UR3e: MoveIt verifying scan home\u2026"));
        const Ur3eScanWaypointMoveResult moveResult = ur3eExecuteMoveHome(serverUrl);
        if (moveResult.stopped)
        {
            outcome.cancelled = true;
            return outcome;
        }

        if (moveResult.ok)
        {
            outcome.success = true;
            outcome.alreadyAtHome = moveResult.alreadyAtHome;
            outcome.atHomeVerified = true;
            host_->appendLog(moveResult.alreadyAtHome
                                ? QStringLiteral("UR3e: verified at scan home position.")
                                : QStringLiteral("UR3e: moved to scan home position."));
            pollJointsSync();
            QMetaObject::invokeMethod(
                this, &Ur3ePanelController::syncHomeJointTargetSliders, Qt::BlockingQueuedConnection);
            return outcome;
        }

        const QString reason = moveResult.errorMessage.trimmed().isEmpty()
                                   ? QStringLiteral("MoveIt could not reach scan home.")
                                   : moveResult.errorMessage.trimmed();
        const HomeEnsurePromptChoice choice = promptManualHomePositioning(reason, context);
        if (choice == HomeEnsurePromptChoice::Retry)
            continue;

        if (choice == HomeEnsurePromptChoice::ContinueWithoutHoming
            && (context == HomeEnsureContext::AfterConnect
                || context == HomeEnsureContext::BeforeScanExecute))
        {
            outcome.success = true;
            if (context == HomeEnsureContext::BeforeScanExecute)
                host_->appendLog(QStringLiteral("UR3e: starting scan without valid scan home."));
            else
                host_->appendLog(QStringLiteral("UR3e: continuing without scan home verification."));
            return outcome;
        }

        if (choice == HomeEnsurePromptChoice::ProceedAnyway
            && (context == HomeEnsureContext::BeforeDisconnect
                || context == HomeEnsureContext::BeforeShutdown))
        {
            outcome.success = true;
            host_->appendLog(QStringLiteral("UR3e: proceeding without reaching scan home."));
            return outcome;
        }

        outcome.cancelled = true;
        return outcome;
    }
}

Ur3ePanelController::HomeEnsurePromptChoice Ur3ePanelController::promptManualHomePositioning(
    const QString &reason,
    const HomeEnsureContext context)
{
    if (QThread::currentThread() == thread())
        return showManualHomePositioningDialog(reason, context);

    HomeEnsurePromptChoice choice = HomeEnsurePromptChoice::Cancel;
    QMetaObject::invokeMethod(
        this,
        [this, reason, context, &choice]() {
            choice = showManualHomePositioningDialog(reason, context);
        },
        Qt::BlockingQueuedConnection);
    return choice;
}

Ur3ePanelController::HomeEnsurePromptChoice Ur3ePanelController::showManualHomePositioningDialog(
    const QString &reason,
    const HomeEnsureContext context)
{
    if (host_ == nullptr)
        return HomeEnsurePromptChoice::Cancel;

    QMessageBox box(host_);
    box.setIcon(QMessageBox::Warning);
    if (context == HomeEnsureContext::AfterConnect)
        box.setWindowTitle(QStringLiteral("UR3e Homing Failed"));
    else if (context == HomeEnsureContext::BeforeScanExecute)
        box.setWindowTitle(QStringLiteral("UR3e Homing Failed — Cannot Start Scan"));
    else if (context == HomeEnsureContext::BeforeDisconnect)
        box.setWindowTitle(QStringLiteral("UR3e Homing Failed — Before Disconnect"));
    else
        box.setWindowTitle(QStringLiteral("UR3e Homing Failed — Before Closing"));

    const QString body =
        QStringLiteral(
            "The robot could not reach or verify the configured scan home pose.\n\n"
            "Reason: %1\n\n"
            "Configured home: %2\n\n"
            "Manually jog the robot closer to home using the joint controls or teach "
            "pendant, then click Retry.")
            .arg(reason, formatConfiguredHomeJointsDeg());
    box.setText(body);

    QPushButton *retryButton = box.addButton(QStringLiteral("Retry"), QMessageBox::AcceptRole);
    box.setDefaultButton(retryButton);

    QPushButton *secondaryButton = nullptr;
    QPushButton *cancelButton = nullptr;
    if (context == HomeEnsureContext::AfterConnect)
    {
        secondaryButton =
            box.addButton(QStringLiteral("Continue without homing"), QMessageBox::DestructiveRole);
        cancelButton = box.addButton(QStringLiteral("Disconnect"), QMessageBox::RejectRole);
    }
    else if (context == HomeEnsureContext::BeforeScanExecute)
    {
        secondaryButton =
            box.addButton(QStringLiteral("Start scan anyway"), QMessageBox::DestructiveRole);
        cancelButton = box.addButton(QStringLiteral("Abort scan"), QMessageBox::RejectRole);
    }
    else if (context == HomeEnsureContext::BeforeDisconnect)
    {
        secondaryButton =
            box.addButton(QStringLiteral("Disconnect anyway"), QMessageBox::DestructiveRole);
        cancelButton = box.addButton(QStringLiteral("Cancel"), QMessageBox::RejectRole);
    }
    else
    {
        secondaryButton = box.addButton(QStringLiteral("Close anyway"), QMessageBox::DestructiveRole);
        cancelButton = box.addButton(QStringLiteral("Cancel"), QMessageBox::RejectRole);
    }

    const bool wasBusy = busy_;
    setBusy(false);
    updateRobotUi();
    box.exec();
    setBusy(wasBusy);
    updateRobotUi();

    QAbstractButton *clicked = box.clickedButton();
    if (clicked == retryButton)
        return HomeEnsurePromptChoice::Retry;
    if (clicked == secondaryButton)
    {
        return context == HomeEnsureContext::AfterConnect ? HomeEnsurePromptChoice::ContinueWithoutHoming
                                                        : HomeEnsurePromptChoice::ProceedAnyway;
    }
    return HomeEnsurePromptChoice::Cancel;
}

void Ur3ePanelController::finishHomeEnsureAfterConnect(const HomeEnsureOutcome &outcome)
{
    if (outcome.cancelled)
    {
        host_->appendLog(QStringLiteral("UR3e: home verification cancelled — disconnecting."));
        setBusy(true);
        const QString serverUrl = serverManager_->serverUrl();
        std::thread([this, serverUrl]() {
            const Ur3eConnectResult result = ur3eDisconnectRobot(serverUrl);
            const bool ok = result.ok;
            const QString detail = ok ? QStringLiteral("disconnected") : result.errorMessage;
            QMetaObject::invokeMethod(
                this,
                [this, ok, detail]() { finishDisconnect(ok, detail); },
                Qt::QueuedConnection);
        }).detach();
        return;
    }

    endHomeMotionUi();
    setBusy(false);
    if (outcome.alreadyAtHome)
        host_->appendLog(QStringLiteral("UR3e: verified at scan home position."));
    else if (outcome.atHomeVerified)
        host_->appendLog(QStringLiteral("UR3e: moved to scan home position."));
    pollJointsSync();
    if (outcome.atHomeVerified)
        applyScanHomeJointTargets();
    updateRobotUi();
}

void Ur3ePanelController::finishDisconnect(const bool ok, const QString &detail)
{
    endHomeMotionUi();
    setBusy(false);
    robotConnected_ = false;
    if (host_->ur3ePosePollTimer_ != nullptr)
        host_->ur3ePosePollTimer_->stop();

    if (ok)
        host_->appendLog(QStringLiteral("UR3e: %1").arg(detail));
    else
        host_->appendLog(QStringLiteral("UR3e disconnect failed: %1").arg(detail));

    updateRobotUi();
}

void Ur3ePanelController::finishMove(const bool ok, const QString &detail)
{
    const bool stoppedByUser = stopRequested_.exchange(false, std::memory_order_acq_rel);
    motionInProgress_ = false;
    setBusy(false);
    setJointPollIntervalMs(kPosePollIntervalMs);
    if (stoppedByUser)
    {
        host_->appendLog(QStringLiteral("UR3e: move interrupted by stop"));
        pollJointsSync();
        syncTargetsFromCurrent();
    }
    else if (ok)
    {
        host_->appendLog(QStringLiteral("UR3e: %1").arg(detail));
        pollJointsSync();
        syncTargetsFromCurrent();
    }
    else
    {
        host_->appendLog(QStringLiteral("UR3e move failed: %1").arg(detail));
        if (host_ != nullptr)
        {
            QMessageBox::warning(host_,
                                 QStringLiteral("UR3e Move Failed"),
                                 detail);
        }
    }
    updateRobotUi();
}

void Ur3ePanelController::finishStop(const bool ok, const QString &detail)
{
    motionInProgress_ = false;
    if (scanExecuting_)
    {
        // Keep stopRequested_ set and stay busy so the scan thread exits the pin loop
        // and returns to home. Clearing the flag here used to resume the scan in place.
        if (ok)
            host_->appendLog(QStringLiteral("UR3e: %1 — scan will return to home").arg(detail));
        else
            host_->appendLog(QStringLiteral("UR3e stop failed: %1").arg(detail));
        updateRobotUi();
        return;
    }

    setBusy(false);
    setJointPollIntervalMs(kPosePollIntervalMs);
    stopRequested_.store(false, std::memory_order_release);
    if (ok)
        host_->appendLog(QStringLiteral("UR3e: %1").arg(detail));
    else
        host_->appendLog(QStringLiteral("UR3e stop failed: %1").arg(detail));
    pollJointsSync();
    syncTargetsFromCurrent();
    updateRobotUi();
}

} // namespace hf::ur3e
