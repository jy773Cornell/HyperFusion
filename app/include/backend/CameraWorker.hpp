// Threaded camera worker interface for control commands and frame streaming.
#pragma once

#include "backend/ICameraController.hpp"

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
    using ShutterStateCallback = std::function<void(bool isOpen)>;
    using SettingsAppliedCallback = std::function<void(const CameraSettingsApplyReport &)>;
    /// Runs a task on the Qt GUI thread (BlockingQueuedConnection — for modal SDK UI only).
    using GuiTaskRunner = std::function<void(std::function<void()>)>;
    /// Posts a task to the GUI thread without blocking the control thread (disconnect teardown).
    using GuiAsyncTaskRunner = std::function<void(std::function<void()>)>;

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
    void requestBeginStreaming(const CameraSettings &settings);
    void requestArm();
    void requestStartStreaming();
    void requestStopStreaming();
    void requestDisconnect();
    /// Stop streaming and disconnect on the GUI thread (required for Pleora teardown).
    void requestDisconnectOnGuiThread();
    void requestOpenShutter();
    void requestCloseShutter();
    void requestQueryShutterState();
    /// Synchronous teardown for application shutdown (call from GUI thread).
    void shutdownSync();

    CameraState currentState() const;
    std::string name() const;

    void setStateCallback(StateCallback callback);
    void setErrorCallback(ErrorCallback callback);
    void setFrameCallback(FrameCallback callback);
    void setShutterStateCallback(ShutterStateCallback callback);
    void setSettingsAppliedCallback(SettingsAppliedCallback callback);
    void setGuiTaskRunner(GuiTaskRunner runner);
    void setGuiAsyncTaskRunner(GuiAsyncTaskRunner runner);

private:
    using ControlCommand = std::function<void()>;

    void enqueueCommand(ControlCommand command);
    void controlLoop();
    void streamLoop();
    void notifyState(CameraState state);
    void notifyError(const CameraError &error);
    void notifySettingsApplied(const CameraSettingsApplyReport &report);
    void publishShutterState();
    void waitForStreamIdle();

    std::shared_ptr<ICameraController> controller_;

    mutable std::mutex callbackMutex_;
    StateCallback stateCallback_;
    ErrorCallback errorCallback_;
    FrameCallback frameCallback_;
    ShutterStateCallback shutterStateCallback_;
    SettingsAppliedCallback settingsAppliedCallback_;
    GuiTaskRunner guiTaskRunner_;
    GuiAsyncTaskRunner guiAsyncTaskRunner_;

    mutable std::mutex commandMutex_;
    std::condition_variable commandCv_;
    std::deque<ControlCommand> commandQueue_;

    std::atomic<bool> running_{false};
    std::atomic<bool> streamEnabled_{false};
    std::atomic<bool> streamInPoll_{false};

    std::thread controlThread_;
    std::thread streamThread_;
};
