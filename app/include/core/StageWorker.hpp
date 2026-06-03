#pragma once

#include "core/IStageController.hpp"

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

    StageState currentState() const;
    StageTopology currentTopology() const;

    void setStateCallback(StateCallback callback);
    void setTopologyCallback(TopologyCallback callback);
    void setErrorCallback(ErrorCallback callback);

private:
    using ControlCommand = std::function<void()>;

    void enqueueCommand(ControlCommand command);
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
    std::deque<ControlCommand> commandQueue_;

    std::atomic<bool> running_{false};
    std::thread controlThread_;
};
