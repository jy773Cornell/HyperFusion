// UR3e tab orchestration implementation.
#include "frontend/controllers/Ur3ePanelController.hpp"

#include "backend/HyperFusionConfig.hpp"
#include "backend/ur3e/Ur3eClient.hpp"
#include "backend/ur3e/Ur3eMoveItManager.hpp"
#include "backend/ur3e/Ur3eRvizManager.hpp"
#include "backend/ur3e/Ur3eServerManager.hpp"
#include "frontend/widgets/MainWindow.hpp"
#include "frontend/widgets/Ur3eJointBarWidget.hpp"

#include <QLineEdit>
#include <QMetaObject>
#include <QPushButton>
#include <QTimer>

#include <cmath>
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

QString formatJointTargetsDeg(const std::vector<double> &targetRad)
{
    QStringList parts;
    const int count = qMin(static_cast<int>(targetRad.size()), kUr3eJointCount);
    for (int jointIndex = 0; jointIndex < count; ++jointIndex)
    {
        const int degrees =
            static_cast<int>(std::lround(targetRad[static_cast<std::size_t>(jointIndex)] * 180.0 / M_PI));
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
    {
        host_->ur3eStopMotionBtn_->setEnabled(motionReady);
        if (busy_)
        {
            host_->ur3eStopMotionBtn_->setText(QStringLiteral("E-Stop"));
            host_->ur3eStopMotionBtn_->setStyleSheet(
                QStringLiteral("background-color: #c0392b; color: white; font-weight: bold;"));
        }
        else
        {
            host_->ur3eStopMotionBtn_->setText(QStringLiteral("Stop"));
            host_->ur3eStopMotionBtn_->setStyleSheet(QString());
        }
    }
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

void Ur3ePanelController::syncTargetsFromCurrent()
{
    for (int jointIndex = 0; jointIndex < MainWindow::kUr3eJointCount; ++jointIndex)
    {
        ui::Ur3eJointBarWidget *bar = host_->ur3eJointBars_[jointIndex];
        if (bar != nullptr)
            bar->syncTargetFromCurrent();
    }
}

void Ur3ePanelController::pollJoints()
{
    if (!robotConnected_ || serverManager_ == nullptr || !serverManager_->isServerConnected()
        || shutdownRequested_.load(std::memory_order_acquire) || busy_)
    {
        return;
    }

    const Ur3eJointsState result = ur3eGetJoints(serverManager_->serverUrl());
    if (!result.ok || result.positionsRad.size() < MainWindow::kUr3eJointCount)
        return;

    applyJointPositions(result.positionsRad, result.names, false);
}

void Ur3ePanelController::onSyncJointsRequested()
{
    if (serverManager_ == nullptr || busy_ || !robotConnected_)
        return;

    pollJoints();
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
        setBusy(true);
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
    setBusy(true);
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

    host_->appendLog(QStringLiteral("UR3e: connecting (%1, dashboard=%2, rtde=%3) — may take up to 2 min\u2026")
                         .arg(robotIp)
                         .arg(cfg.dashboardPort)
                         .arg(cfg.rtdePort));

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
    host_->appendLog(QStringLiteral("UR3e: e-stop requested"));
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
                applyJointPositions(joints.positionsRad, joints.names, true);
            else
                pollJoints();
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
    setBusy(false);
    if (stoppedByUser)
    {
        host_->appendLog(QStringLiteral("UR3e: move interrupted by e-stop"));
        pollJoints();
        syncTargetsFromCurrent();
    }
    else if (ok)
    {
        host_->appendLog(QStringLiteral("UR3e: %1").arg(detail));
        pollJoints();
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
    setBusy(false);
    stopRequested_.store(false, std::memory_order_release);
    if (ok)
        host_->appendLog(QStringLiteral("UR3e: %1").arg(detail));
    else
        host_->appendLog(QStringLiteral("UR3e stop failed: %1").arg(detail));
    pollJoints();
    syncTargetsFromCurrent();
    updateRobotUi();
}

} // namespace hf::ur3e
