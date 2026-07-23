// Threaded BFS camera worker: control queue + RGB poll loop (backend/3dscanning).
#pragma once

#include "backend/3dscanning/BfsCameraTypes.hpp"
#include "backend/3dscanning/BfsSpinnakerCamera.hpp"

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

namespace hf::bfs
{
class BfsCameraWorker
{
public:
    using StateCallback = std::function<void(BfsCameraState)>;
    using ErrorCallback = std::function<void(const BfsError &)>;
    using FrameCallback = std::function<void(const BfsRgbFrame &)>;
    using DevicesCallback = std::function<void(const std::vector<BfsDeviceInfo> &)>;

    BfsCameraWorker();
    ~BfsCameraWorker();

    void start();
    void stop();
    void shutdownSync();

    void requestEnumerate();
    void requestConnect(const BfsCameraSettings &settings);
    void requestApplySettings(const BfsCameraSettings &settings);
    void requestDisconnect();

    [[nodiscard]] BfsCameraState currentState() const;

    void setStateCallback(StateCallback callback);
    void setErrorCallback(ErrorCallback callback);
    void setFrameCallback(FrameCallback callback);
    void setDevicesCallback(DevicesCallback callback);

private:
    using ControlCommand = std::function<void()>;

    void enqueue(ControlCommand command);
    void controlLoop();
    void streamLoop();
    void notifyState(BfsCameraState state);
    void notifyError(const BfsError &error);

    std::unique_ptr<BfsSpinnakerCamera> camera_;

    mutable std::mutex callbackMutex_;
    StateCallback stateCallback_;
    ErrorCallback errorCallback_;
    FrameCallback frameCallback_;
    DevicesCallback devicesCallback_;

    mutable std::mutex commandMutex_;
    std::condition_variable commandCv_;
    std::deque<ControlCommand> commandQueue_;

    std::atomic<bool> running_{false};
    std::atomic<bool> streamEnabled_{false};
    std::atomic<bool> streamInPoll_{false};
    std::atomic<BfsCameraState> state_{BfsCameraState::Disconnected};

    std::thread controlThread_;
    std::thread streamThread_;
};
} // namespace hf::bfs
