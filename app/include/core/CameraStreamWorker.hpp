// Dedicated background thread for frame polling during active acquisition.
#pragma once

#include "core/ICameraController.hpp"

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

class CameraStreamWorker
{
public:
    using FrameCallback = std::function<void(const FramePacket &)>;
    using ErrorCallback = std::function<void(const CameraError &)>;
    using StateCallback = std::function<void(CameraState)>;

    explicit CameraStreamWorker(std::shared_ptr<ICameraController> controller);
    ~CameraStreamWorker();

    void start();
    void stop();

    void setEnabled(bool enabled);
    bool enabled() const;

    void setFrameCallback(FrameCallback callback);
    void setErrorCallback(ErrorCallback callback);
    void setStateCallback(StateCallback callback);

private:
    void streamLoop();
    void notifyError(const CameraError &error);
    void notifyState(CameraState state);

    std::shared_ptr<ICameraController> controller_;

    mutable std::mutex callbackMutex_;
    FrameCallback frameCallback_;
    ErrorCallback errorCallback_;
    StateCallback stateCallback_;

    std::atomic<bool> running_{false};
    std::atomic<bool> enabled_{false};
    std::thread streamThread_;
};
