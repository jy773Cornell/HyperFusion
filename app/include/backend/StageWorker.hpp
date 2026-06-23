#pragma once

#include "adapters/zaber/ZaberStageProfile.hpp"
#include "backend/IStageController.hpp"

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

class StageWorker
{
public:
    using StateCallback = std::function<void(StageState)>;
    using TopologyCallback = std::function<void(const StageTopology &)>;
    using ErrorCallback = std::function<void(const StageError &)>;

    explicit StageWorker(std::shared_ptr<IStageController> controller);
    ~StageWorker();

    void start();
    void stop();

    void requestConnect(const StageConnectSettings &settings);
    void requestDisconnect();
    /// Home lockstep (if connected), then disconnect. Optional callback runs on the worker thread.
    void requestDisconnectWithHoming(std::function<void()> onComplete = nullptr);
    /// Home (if connected), disconnect, and stop the worker thread.
    /// When @p homeBeforeDisconnect is false, motion is stopped and the stage disconnects immediately.
    void shutdownSync(bool homeBeforeDisconnect = true);
    void requestStopMotion(std::function<void()> onComplete = nullptr, bool waitUntilIdle = true);
    void requestHome();
    void requestMoveRelativeMm(double distanceMm,
                               double speedMmPerSec = zaber_stage::kMaxSpeedMmPerSec);
    void requestMoveAbsoluteMm(double positionMm,
                               double speedMmPerSec = zaber_stage::kMaxSpeedMmPerSec,
                               bool waitUntilIdle = false);
    void requestMoveAbsoluteMm(double positionMm,
                               double speedMmPerSec,
                               bool waitUntilIdle,
                               std::function<void(bool success)> onComplete);
    void requestMoveVelocityMm(double velocityMmPerSec);

    using PositionCallback = std::function<void(double positionMm, bool ok)>;
    void requestPrimaryPosition(PositionCallback callback);

    StageState currentState() const;
    StageTopology currentTopology() const;

    void setStateCallback(StateCallback callback);
    void setTopologyCallback(TopologyCallback callback);
    void setErrorCallback(ErrorCallback callback);

private:
    using ControlCommand = std::function<void()>;

    enum class CommandKind
    {
        Normal,
        PositionPoll,
    };

    struct QueuedCommand
    {
        CommandKind kind = CommandKind::Normal;
        ControlCommand fn;
    };

    void enqueueCommand(ControlCommand command, CommandKind kind = CommandKind::Normal);
    void enqueuePriorityCommand(ControlCommand command);
    void purgePendingPositionPolls();
    void controlLoop();
    void notifyState(StageState state);
    void notifyTopology(const StageTopology &topology);
    void notifyError(const StageError &error);

    std::shared_ptr<IStageController> controller_;

    mutable std::mutex callbackMutex_;
    StateCallback stateCallback_;
    TopologyCallback topologyCallback_;
    ErrorCallback errorCallback_;

    mutable std::mutex commandMutex_;
    std::condition_variable commandCv_;
    std::deque<QueuedCommand> commandQueue_;

    std::atomic<bool> running_{false};
    std::atomic<bool> commandInFlight_{false};
    std::thread controlThread_;
};
