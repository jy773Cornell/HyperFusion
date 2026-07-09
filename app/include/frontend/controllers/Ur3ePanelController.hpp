// UR3e tab orchestration: WSL sidecar lifecycle, robot connect, pose polling, motion.
#pragma once

#include <QObject>
#include <QString>
#include <QStringList>

#include "backend/ur3e/Ur3eServerManager.hpp"

#include <atomic>
#include <memory>
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
    void applyJointPositions(const std::vector<double> &positionsRad,
                             const QStringList &names,
                             bool syncTargets);
    void syncTargetsFromCurrent();
    void setBusy(bool busy);

    void onConnectRequested();
    void onDisconnectRequested();
    void onSyncJointsRequested();
    void onMoveRequested();
    void onStopMotionRequested();
    void onStartRvizRequested();
    void onStartMoveItRequested();
    void onMoveItStateChanged(bool running, const QString &detail);
    void onRvizStateChanged(bool running, const QString &detail);

    void finishConnect(bool ok, const QString &detail);
    void finishDisconnect(bool ok, const QString &detail);
    void finishMove(bool ok, const QString &detail);
    void finishStop(bool ok, const QString &detail);

    static constexpr int kPosePollIntervalMs = 500;

    MainWindow *host_ = nullptr;
    std::unique_ptr<Ur3eServerManager> serverManager_;
    std::unique_ptr<Ur3eMoveItManager> moveItManager_;
    std::unique_ptr<Ur3eRvizManager> rvizManager_;
    bool robotConnected_ = false;
    bool busy_ = false;
    std::atomic<bool> stopRequested_{false};
    std::atomic<bool> shutdownRequested_{false};
    Ur3eServerManager::State lastLoggedSidecarState_ = Ur3eServerManager::State::Stopped;
};

} // namespace hf::ur3e
