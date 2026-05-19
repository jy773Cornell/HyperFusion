// Threaded camera worker interface for control commands and frame streaming.
#pragma once

#include "core/ICameraController.hpp"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

class CameraWorker
{
public:
    using StateCallback = std::function<void(CameraState)>;
    using ErrorCallback = std::function<void(const CameraError &)>;
    using FrameCallback = std::function<void(const FramePacket &)>;
    /// Runs a task on the Qt GUI thread (e.g. BlockingQueuedConnection for modal SDK UI).
    using GuiTaskRunner = std::function<void(std::function<void()>)>;

    explicit CameraWorker(std::shared_ptr<ICameraController> controller);
    ~CameraWorker();

    void start();
    void stop();

    void requestConnect();
    void requestInitialize();
    void requestInitializeOnGuiThread();
    /// Connect to SDK, then Initialize on the GUI thread (single queued operation).
    void requestConnectAndInitializeOnGuiThread();
    void requestApplySettings(const CameraSettings &settings);
    void requestArm();
    void requestStartStreaming();
    void requestStopStreaming();
    void requestDisconnect();

    CameraState currentState() const;
    std::string name() const;

    void setStateCallback(StateCallback callback);
    void setErrorCallback(ErrorCallback callback);
    void setFrameCallback(FrameCallback callback);
    void setGuiTaskRunner(GuiTaskRunner runner);

private:
    using ControlCommand = std::function<void()>;

    void enqueueCommand(ControlCommand command);
    void controlLoop();
    void streamLoop();
    void notifyState(CameraState state);
    void notifyError(const CameraError &error);

    std::shared_ptr<ICameraController> controller_;

    mutable std::mutex callbackMutex_;
    StateCallback stateCallback_;
    ErrorCallback errorCallback_;
    FrameCallback frameCallback_;
    GuiTaskRunner guiTaskRunner_;

    mutable std::mutex commandMutex_;
    std::condition_variable commandCv_;
    std::deque<ControlCommand> commandQueue_;

    std::atomic<bool> running_{false};
    std::atomic<bool> streamEnabled_{false};

    std::thread controlThread_;
    std::thread streamThread_;
};
