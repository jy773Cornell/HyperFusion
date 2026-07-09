// UR3e tab orchestration: WSL sidecar lifecycle, robot connect, pose polling, motion.
#pragma once

#include <QObject>
#include <QString>
#include <QStringList>
#include <QVariant>

#include "backend/ur3e/Ur3eHemisphereScanReachability.hpp"
#include "backend/ur3e/Ur3eServerManager.hpp"

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

class MainWindow;
class QTimer;

namespace hf::ur3e
{
class Ur3eMoveItManager;
class Ur3eRvizManager;

class Ur3ePanelController : public QObject
{
    Q_OBJECT

public:
    explicit Ur3ePanelController(MainWindow *host, QObject *parent = nullptr);
    ~Ur3ePanelController() override;

    void wireSettingsTabConnections();
    void applyHardwareConfigToUi();
    void startSidecarOnLaunch();
    void shutdownSync();
    void refreshUi();

    [[nodiscard]] bool isRobotConnected() const { return robotConnected_; }
    [[nodiscard]] bool isSidecarRunning() const;

public slots:
    void onSidecarStateChanged(Ur3eServerManager::State state, const QString &detail);

private:
    void updateRobotUi();
    void pollJoints();
    void pollJointsSync();
    void applyJointPositions(const std::vector<double> &positionsRad,
                             const QStringList &names,
                             bool syncTargets);
    void joinJointPollThread();
    void applyJointTargets(const std::vector<double> &positionsRad,
                           const QStringList &names = QStringList());
    void setJointPollIntervalMs(int intervalMs);
    void applyConfiguredInitialJointTargets();
    void syncWorkspaceBoundaryPreview();
    void syncTargetsFromCurrent();
    void setBusy(bool busy);

    void onConnectRequested();
    void onDisconnectRequested();
    void onSyncJointsRequested();
    void onMoveRequested();
    void onStopMotionRequested();
    void onStartRvizRequested();
    void onStartMoveItRequested();
    void onPlanHemisphereScanRequested();
    void onExecuteHemisphereScanRequested();
    void onMoveItStateChanged(bool running, const QString &detail);
    void onRvizStateChanged(bool running, const QString &detail);

private slots:
    void scanExecuteSetActivePoint(int pointIndex);
    void scanExecuteMarkCompleted(int pointIndex);
    void scanExecuteLogMoving(int step,
                              int total,
                              int pointIndex,
                              const QString &tcpSummary,
                              const QString &targetSummary,
                              const QVariantList &targetPositionsRad);
    void scanExecuteLogArrived(int step,
                               int total,
                               int pointIndex,
                               const QString &arrivedPose,
                               const QString &arrivedJoints,
                               const QVariantList &positionsRad,
                               const QStringList &names);
    void scanExecuteFinish(bool ok,
                           const QString &errorMessage,
                           int executedCount,
                           bool stopped);
    void applyPolledJoints(const QVariantList &positionsRad, const QStringList &names);

private:
    void finishConnect(bool ok, const QString &detail);
    void finishDisconnect(bool ok, const QString &detail);
    void finishMove(bool ok, const QString &detail);
    void finishStop(bool ok, const QString &detail);
    void finishScanPlan(const Ur3eHemisphereScanPlan &plan, const QString &errorMessage);
    void finishScanExecute(bool ok, const QString &detail);

    static constexpr int kPosePollIntervalMs = 500;
    static constexpr int kMotionPollIntervalMs = 100;
    static constexpr int kScanExecuteDwellMs = 2000;

    MainWindow *host_ = nullptr;
    std::unique_ptr<Ur3eServerManager> serverManager_;
    std::unique_ptr<Ur3eMoveItManager> moveItManager_;
    std::unique_ptr<Ur3eRvizManager> rvizManager_;
    bool robotConnected_ = false;
    bool busy_ = false;
    bool scanPlanReady_ = false;
    bool scanExecuting_ = false;
    bool motionInProgress_ = false;
    Ur3eHemisphereScanPlan plannedScanPlan_;
    std::atomic<bool> stopRequested_{false};
    std::atomic<bool> shutdownRequested_{false};
    std::atomic<int> scanExecuteSessionId_{0};
    std::mutex scanExecuteThreadMutex_;
    std::thread scanExecuteThread_;
    std::atomic<bool> jointPollInFlight_{false};
    std::mutex jointPollThreadMutex_;
    std::thread jointPollThread_;
    Ur3eServerManager::State lastLoggedSidecarState_ = Ur3eServerManager::State::Stopped;
};

} // namespace hf::ur3e
