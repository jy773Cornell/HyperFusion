// UR3e tab orchestration implementation.
#include "frontend/controllers/Ur3ePanelController.hpp"

#include "backend/HyperFusionConfig.hpp"
#include "backend/ur3e/Ur3eClient.hpp"
#include "backend/ur3e/Ur3eHemisphereScan.hpp"
#include "backend/ur3e/Ur3eHemisphereScanReachability.hpp"
#include "backend/ur3e/Ur3eWorkspaceBoundary.hpp"
#include "backend/ur3e/Ur3eMoveItManager.hpp"
#include "backend/ur3e/Ur3eRvizManager.hpp"
#include "backend/ur3e/Ur3eServerManager.hpp"
#include "frontend/widgets/MainWindow.hpp"
#include "frontend/widgets/Ur3eHemisphereScanSettingsWidget.hpp"
#include "frontend/widgets/Ur3eJointBarWidget.hpp"
#include "frontend/widgets/Ur3eScanRoutePlanWidget.hpp"

#include <QLineEdit>
#include <QMessageBox>
#include <QMetaObject>
#include <QPushButton>
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
constexpr double kMockInitialJointDeg[kUr3eJointCount] = {0.0, -150.0, 120.0, 0.0, 90.0, 0.0};
constexpr const char *kUr3eJointNames[kUr3eJointCount] = {
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
};

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
}

Ur3ePanelController::~Ur3ePanelController()
{
    shutdownSync();
}

bool Ur3ePanelController::isSidecarRunning() const
{
    return serverManager_ != nullptr && serverManager_->isServerConnected();
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
    host_->appendLog(QStringLiteral("UR3e: starting WSL sidecar (use_mock_hardware=%1, prestart_driver=%2)\u2026")
                        .arg(cfg.useMockHardware ? QStringLiteral("true") : QStringLiteral("false"))
                        .arg(cfg.prestartDriver ? QStringLiteral("true") : QStringLiteral("false")));
    serverManager_->tryAutoStart();
}

void Ur3ePanelController::refreshUi()
{
    updateRobotUi();
}

void Ur3ePanelController::shutdownSync()
{
    shutdownRequested_.store(true, std::memory_order_release);
    stopRequested_.store(true, std::memory_order_release);
    ++scanExecuteSessionId_;
    scanExecuting_ = false;
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
        ur3eDisconnectRobot(serverManager_->serverUrl());

    if (serverManager_ != nullptr)
        serverManager_->stopServer();

    if (moveItManager_ != nullptr)
        moveItManager_->stop();

    if (rvizManager_ != nullptr)
        rvizManager_->stop();

    robotConnected_ = false;
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
                    updateRobotUi();
                });
    }

    if (host_->ur3ePosePollTimer_ != nullptr)
    {
        connect(host_->ur3ePosePollTimer_, &QTimer::timeout, this, [this]() { pollJoints(); });
    }
}

void Ur3ePanelController::onSidecarStateChanged(const Ur3eServerManager::State state,
                                                  const QString &detail)
{
    if (serverManager_ != nullptr)
    {
        const bool stateChanged = state != lastLoggedSidecarState_;
        if (stateChanged)
        {
            lastLoggedSidecarState_ = state;
            if (state == Ur3eServerManager::State::Running
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
                host_->appendLog(trimmed);
            }
        }
    }

    if (!isSidecarRunning())
    {
        robotConnected_ = false;
        if (host_->ur3ePosePollTimer_ != nullptr)
            host_->ur3ePosePollTimer_->stop();
        updateRobotUi();
    }
    else
    {
        updateRobotUi();
    }
}

void Ur3ePanelController::setBusy(const bool busy)
{
    busy_ = busy;
    updateRobotUi();
}

void Ur3ePanelController::updateRobotUi()
{
    const bool sidecarRunning = isSidecarRunning();
    const bool captureActive = host_->isCaptureSessionActive();

    if (host_->ur3eRobotIpEdit_ != nullptr)
        host_->ur3eRobotIpEdit_->setEnabled(!robotConnected_ && !busy_ && !captureActive);

    if (host_->ur3eConnectBtn_ != nullptr)
    {
        host_->ur3eConnectBtn_->setEnabled(sidecarRunning && !robotConnected_ && !busy_
                                            && !captureActive);
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
        host_->ur3eScanRoutePlanWidget_->setWorkspaceBoundary(boundary);
}

void Ur3ePanelController::applyConfiguredInitialJointTargets()
{
    const hf::HardwareConfig::Ur3eConfig &cfg = hf::hardwareConfig().ur3e;
    if (!cfg.useMockHardware)
        return;

    for (int jointIndex = 0; jointIndex < MainWindow::kUr3eJointCount; ++jointIndex)
    {
        ui::Ur3eJointBarWidget *bar = host_->ur3eJointBars_[jointIndex];
        if (bar == nullptr)
            continue;
        const double radians = kMockInitialJointDeg[jointIndex] * M_PI / 180.0;
        bar->setValueRadians(radians);
    }
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
        || shutdownRequested_.load(std::memory_order_acquire))
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

    const hf::HardwareConfig::Ur3eConfig &cfg = hf::hardwareConfig().ur3e;
    const QString motionType = cfg.motionType.trimmed().toLower();
    const QString serverUrl = serverManager_->serverUrl();

    if (motionType == QStringLiteral("move_l"))
    {
        stopRequested_.store(false, std::memory_order_release);
        motionInProgress_ = true;
        setBusy(true);
        setJointPollIntervalMs(kMotionPollIntervalMs);
        const double speed = cfg.maxLinearSpeedMPerS;
        const double accel = cfg.maxLinearAccelMPerS2;
        std::thread([this, serverUrl, speed, accel]() {
            const Ur3ePoseResult poseResult = ur3eGetTcpPose(serverUrl);
            if (!poseResult.ok)
            {
                QMetaObject::invokeMethod(
                    this,
                    [this, detail = poseResult.errorMessage]() { finishMove(false, detail); },
                    Qt::QueuedConnection);
                return;
            }

            const QString poseSummary = formatTcpPose(poseResult.pose);
            QMetaObject::invokeMethod(
                this,
                [this, poseSummary]() {
                    host_->appendLog(
                        QStringLiteral("UR3e: Move (linear) requested — pose %1").arg(poseSummary));
                },
                Qt::QueuedConnection);

            const Ur3eMoveResult result =
                ur3eMoveLinear(serverUrl, poseResult.pose, speed, accel, true);
            const bool ok = result.ok;
            const QString detail = ok ? QStringLiteral("move complete — pose %1").arg(poseSummary)
                                      : result.errorMessage;
            QMetaObject::invokeMethod(
                this,
                [this, ok, detail]() { finishMove(ok, detail); },
                Qt::QueuedConnection);
        }).detach();
        return;
    }

    std::vector<double> target;
    target.reserve(MainWindow::kUr3eJointCount);
    for (int jointIndex = 0; jointIndex < MainWindow::kUr3eJointCount; ++jointIndex)
    {
        ui::Ur3eJointBarWidget *bar = host_->ur3eJointBars_[jointIndex];
        target.push_back(bar != nullptr ? bar->valueRadians() : 0.0);
    }

    const QString targetSummary = formatJointTargetsDeg(target);
    host_->appendLog(
        QStringLiteral("UR3e: Move (joint) requested — %1").arg(targetSummary));
    stopRequested_.store(false, std::memory_order_release);
    motionInProgress_ = true;
    setBusy(true);
    setJointPollIntervalMs(kMotionPollIntervalMs);
    std::thread([this, serverUrl, target, targetSummary]() {
        const Ur3eJointsMoveResult result = ur3eMoveJoints(serverUrl, target, true);
        const bool ok = result.ok;
        const QString detail = ok ? QStringLiteral("move complete — %1").arg(targetSummary)
                                  : result.errorMessage;
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
    setBusy(false);

    if (!errorMessage.isEmpty() && plan.points.empty())
    {
        host_->appendLog(QStringLiteral("UR3e scan plan failed: %1").arg(errorMessage));
        scanPlanReady_ = false;
        updateRobotUi();
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

    updateRobotUi();
}

void Ur3ePanelController::onExecuteHemisphereScanRequested()
{
    if (host_->ur3eHemisphereScanSettings_ == nullptr || busy_ || !robotConnected_
        || serverManager_ == nullptr)
        return;

    if (!scanPlanReady_ || plannedScanPlan_.reachableCount == 0)
    {
        host_->appendLog(
            QStringLiteral("UR3e scan execute rejected: plan a route with reachable points first."));
        return;
    }

    const std::vector<int> order = buildHemisphereScanExecutionOrder(plannedScanPlan_);
    if (order.empty())
    {
        host_->appendLog(QStringLiteral("UR3e scan execute rejected: no stored joint solutions."));
        return;
    }

    const QString serverUrl = serverManager_->serverUrl();
    host_->appendLog(
        QStringLiteral("UR3e scan execute: %1 reachable point(s), top-to-bottom rings (2 s dwell each)…")
            .arg(order.size()));
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

        scanExecuteThread_ = std::thread([this, serverUrl, order, planCopy, sessionId]() {
            int executed = 0;
            int skipped = 0;
            QString errorMessage;
            bool ok = true;
            const int total = static_cast<int>(order.size());

            const auto sessionActive = [this, sessionId]() {
                return !shutdownRequested_.load(std::memory_order_acquire)
                       && sessionId == scanExecuteSessionId_.load(std::memory_order_acquire);
            };

            QMetaObject::invokeMethod(
                this,
                [this]() {
                    host_->appendLog(
                        QStringLiteral("UR3e scan execute: moving to home pose before scan…"));
                },
                Qt::QueuedConnection);

            const Ur3eScanWaypointMoveResult preHomeResult = ur3eExecuteMoveHome(serverUrl);
            if (preHomeResult.stopped)
            {
                if (stopRequested_.load(std::memory_order_acquire))
                    ur3eStopMotion(serverUrl);
                QMetaObject::invokeMethod(
                    this,
                    "scanExecuteFinish",
                    Qt::QueuedConnection,
                    Q_ARG(bool, false),
                    Q_ARG(QString, preHomeResult.errorMessage),
                    Q_ARG(int, 0),
                    Q_ARG(bool, true));
                return;
            }
            if (!preHomeResult.ok)
            {
                const QString reason = preHomeResult.errorMessage.isEmpty()
                                           ? QStringLiteral("could not move to home")
                                           : preHomeResult.errorMessage;
                QMetaObject::invokeMethod(
                    this,
                    [this, reason]() {
                        host_->appendLog(
                            QStringLiteral("UR3e scan execute aborted: %1").arg(reason));
                    },
                    Qt::QueuedConnection);
                QMetaObject::invokeMethod(
                    this,
                    "scanExecuteFinish",
                    Qt::QueuedConnection,
                    Q_ARG(bool, false),
                    Q_ARG(QString, reason),
                    Q_ARG(int, 0),
                    Q_ARG(bool, false));
                return;
            }

            bool returnHomeAfterScan = true;

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

                const Ur3eScanWaypointMoveResult moveResult =
                    ur3eExecuteScanWaypoint(serverUrl, point.jointPositionsRad, &point.tcp);
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

                std::this_thread::sleep_for(std::chrono::milliseconds(kScanExecuteDwellMs));

                if (!sessionActive() || stopRequested_.load(std::memory_order_acquire))
                {
                    ur3eStopMotion(serverUrl);
                    break;
                }

                QMetaObject::invokeMethod(
                    this,
                    "scanExecuteMarkCompleted",
                    Qt::QueuedConnection,
                    Q_ARG(int, pointIndex));

                ++executed;
            }

            if (!sessionActive())
                return;

            if (returnHomeAfterScan)
            {
                QMetaObject::invokeMethod(
                    this,
                    [this]() {
                        host_->appendLog(
                            QStringLiteral("UR3e scan execute: returning to home pose…"));
                    },
                    Qt::QueuedConnection);

                const Ur3eScanWaypointMoveResult postHomeResult = ur3eExecuteMoveHome(serverUrl);
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

            const bool stopped = stopRequested_.load(std::memory_order_acquire);
            QMetaObject::invokeMethod(
                this,
                "scanExecuteFinish",
                Qt::QueuedConnection,
                Q_ARG(bool, ok),
                Q_ARG(QString, errorMessage),
                Q_ARG(int, executed),
                Q_ARG(bool, stopped));
        });
    }
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
                                            const bool stopped)
{
    if (!scanExecuting_)
        return;

    {
        std::lock_guard<std::mutex> lock(scanExecuteThreadMutex_);
        if (scanExecuteThread_.joinable())
            scanExecuteThread_.join();
    }

    scanExecuting_ = false;
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

    finishScanExecute(ok, detail);
}

void Ur3ePanelController::finishScanExecute(const bool ok, const QString &detail)
{
    setBusy(false);
    setJointPollIntervalMs(kPosePollIntervalMs);
    host_->appendLog(QStringLiteral("UR3e scan execute: %1").arg(detail));
    if (!ok)
        QMessageBox::warning(host_, QStringLiteral("UR3e Scan Execute Failed"), detail);
    if (ok)
        pollJoints();
    updateRobotUi();
}

void Ur3ePanelController::onMoveItStateChanged(const bool running, const QString &detail)
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
            host_->appendLog(QStringLiteral("UR3e MoveIt: %1").arg(trimmed));
    }
    updateRobotUi();
    Q_UNUSED(running);
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
    if (serverManager_ == nullptr || busy_)
        return;

    if (!serverManager_->isServerConnected())
    {
        host_->appendLog(QStringLiteral("UR3e: sidecar not ready yet — wait for WSL startup (see UR3e log tab)."));
        return;
    }

    const hf::HardwareConfig::Ur3eConfig &cfg = hf::hardwareConfig().ur3e;
    const QString robotIp = host_->ur3eRobotIpEdit_ != nullptr
                                ? host_->ur3eRobotIpEdit_->text().trimmed()
                                : cfg.robotIp;

    const int connectTimeoutSec = qMax(30, cfg.connectTimeoutMs / 1000);
    host_->appendLog(QStringLiteral("UR3e: connecting (%1, dashboard=%2, rtde=%3) — may take up to %4 min\u2026")
                         .arg(robotIp)
                         .arg(cfg.dashboardPort)
                         .arg(cfg.rtdePort)
                         .arg((connectTimeoutSec + 59) / 60));

    setBusy(true);
    const QString serverUrl = serverManager_->serverUrl();
    std::thread([this, serverUrl, robotIp]() {
        const Ur3eConnectResult result = ur3eConnectRobot(serverUrl, robotIp);
        const bool ok = result.ok;
        const QString detail =
            ok ? (result.useMockHardware ? QStringLiteral("connected (simulation)")
                                         : QStringLiteral("connected (hardware)"))
               : result.errorMessage;
        QMetaObject::invokeMethod(
            this,
            [this, ok, detail]() { finishConnect(ok, detail); },
            Qt::QueuedConnection);
    }).detach();
}

void Ur3ePanelController::onDisconnectRequested()
{
    if (serverManager_ == nullptr || busy_ || !robotConnected_)
        return;

    host_->appendLog(QStringLiteral("UR3e: disconnecting\u2026"));
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
}

void Ur3ePanelController::onStopMotionRequested()
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
    setBusy(false);
    if (ok)
    {
        robotConnected_ = true;
        host_->appendLog(QStringLiteral("UR3e: %1").arg(detail));
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
    }
    else
    {
        host_->appendLog(QStringLiteral("UR3e connect failed: %1").arg(detail));
    }
    updateRobotUi();
}

void Ur3ePanelController::finishDisconnect(const bool ok, const QString &detail)
{
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
    }
    updateRobotUi();
}

void Ur3ePanelController::finishStop(const bool ok, const QString &detail)
{
    motionInProgress_ = false;
    setBusy(false);
    if (!scanExecuting_)
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
