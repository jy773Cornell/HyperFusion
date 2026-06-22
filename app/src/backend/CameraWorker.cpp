// Threaded camera worker implementation for command/control and streaming loops.
// Both FX10e (Pleora) and SWIR3 (NI grabber) use LumoCamera / Swir3NiCamera behind this worker.
// Per camera: control thread (commands) + stream thread (pollFrame). Connect/init/disconnect and
// applySettings/arm/start run on a dedicated SdkLifecycleRunner Qt thread when
// requiresGuiThreadForSdkLifecycle() (SWIR3 cam007 serial during Initialize must not run on the
// camera control thread; main GUI thread must stay free for Windows responsiveness).
#include "backend/CameraWorker.hpp"

#include <QCoreApplication>
#include <QThread>

#include <chrono>
#include <functional>

namespace
{
constexpr std::uint32_t kFramePollTimeoutMs = 100;
}

CameraWorker::CameraWorker(std::shared_ptr<ICameraController> controller)
    : controller_(std::move(controller))
{
}

CameraWorker::~CameraWorker()
{
    stop();
}

void CameraWorker::start()
{
    if (running_.exchange(true))
        return;

    controlThread_ = std::thread(&CameraWorker::controlLoop, this);
    streamThread_ = std::thread(&CameraWorker::streamLoop, this);
}

void CameraWorker::stop()
{
    if (!running_.exchange(false))
        return;

    streamEnabled_ = false;
    commandCv_.notify_all();

    if (controlThread_.joinable())
        controlThread_.join();
    if (streamThread_.joinable())
        streamThread_.join();
}

void CameraWorker::requestConnect()
{
    enqueueCommand([this]() {
        CameraError error;
        if (controller_->connect(error))
            notifyState(controller_->state());
        else
            notifyError(error);
    });
}

void CameraWorker::requestInitialize()
{
    enqueueCommand([this]() {
        CameraError error;
        if (controller_->initialize(error))
            notifyState(controller_->state());
        else
            notifyError(error);
    });
}

void CameraWorker::requestInitializeOnGuiThread()
{
    enqueueCommand([this]() {
        CameraError error;
        bool ok = false;

        if (guiTaskRunner_)
        {
            guiTaskRunner_([this, &ok, &error]() { ok = controller_->initialize(error); });
        }
        else
        {
            ok = controller_->initialize(error);
        }

        if (ok)
            notifyState(controller_->state());
        else
            notifyError(error);
    });
}

void CameraWorker::requestConnectAndInitializeOnGuiThread()
{
    enqueueCommand([this]() {
        CameraError error;
        bool success = false;

        runSdkLifecycleTask([this, &success, &error]() {
            const CameraState stateBefore = controller_->state();
            if (stateBefore == CameraState::Disconnected || stateBefore == CameraState::Fault)
            {
                if (!controller_->connect(error))
                {
                    notifyState(controller_->state());
                    return;
                }
            }

            if (controller_->initialize(error))
            {
                success = true;
            }
            else if (controller_->state() != CameraState::Disconnected)
            {
                controller_->disconnect();
            }

            notifyState(controller_->state());
        });

        if (!success)
            notifyError(error);
    });
}

void CameraWorker::setGuiTaskRunner(GuiTaskRunner runner)
{
    std::lock_guard<std::mutex> lock(callbackMutex_);
    guiTaskRunner_ = std::move(runner);
}

void CameraWorker::waitForStreamIdle()
{
    for (int attempt = 0; attempt < 200; ++attempt)
    {
        if (!streamInPoll_.load())
            return;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void CameraWorker::runSdkLifecycleTask(const std::function<void()> &task)
{
    GuiTaskRunner runner;
    {
        std::lock_guard<std::mutex> lock(callbackMutex_);
        runner = guiTaskRunner_;
    }

    if (controller_->requiresGuiThreadForSdkLifecycle() && runner)
        runner(task);
    else
        task();
}

void CameraWorker::requestApplySettings(const CameraSettings &settings)
{
    enqueueCommand([this, settings]() {
        CameraError error;
        CameraTimingApplyResult timing;
        CameraSettingsApplyReport report;
        report.requested = settings;

        const bool resumeStreaming = streamEnabled_.load()
                                     || controller_->state() == CameraState::Streaming;
        if (resumeStreaming)
        {
            streamEnabled_ = false;
            waitForStreamIdle();
        }

        bool ok = false;
        runSdkLifecycleTask([this, &settings, &error, &timing, &report, resumeStreaming, &ok]() {
            if (resumeStreaming)
                controller_->stop();

            if (!controller_->applySettings(settings, error, &timing))
                return;

            report.timing = timing;
            notifySettingsApplied(report);

            if (!resumeStreaming)
            {
                ok = true;
                return;
            }

            if (!controller_->arm(error))
                return;

            if (!controller_->start(error))
                return;

            ok = true;
        });

        if (ok)
        {
            if (resumeStreaming)
                streamEnabled_ = true;
            notifyState(controller_->state());
            return;
        }

        notifyError(error);
        notifyState(controller_->state());
    });
}

void CameraWorker::requestBeginStreaming(const CameraSettings &settings)
{
    enqueueCommand([this, settings]() {
        CameraError error;
        CameraTimingApplyResult timing;
        CameraSettingsApplyReport report;
        report.requested = settings;

        bool ok = false;
        runSdkLifecycleTask([this, &settings, &error, &timing, &report, &ok]() {
            if (!controller_->applySettings(settings, error, &timing))
                return;

            report.timing = timing;
            notifySettingsApplied(report);

            if (!controller_->arm(error))
                return;

            if (!controller_->start(error))
                return;

            ok = true;
        });

        if (ok)
        {
            streamEnabled_ = true;
            notifyState(controller_->state());
            return;
        }

        notifyError(error);
        notifyState(controller_->state());
    });
}

void CameraWorker::requestArm()
{
    enqueueCommand([this]() {
        CameraError error;
        if (controller_->arm(error))
            notifyState(controller_->state());
        else
            notifyError(error);
    });
}

void CameraWorker::requestStartStreaming()
{
    enqueueCommand([this]() {
        CameraError error;
        if (controller_->start(error))
        {
            streamEnabled_ = true;
            notifyState(controller_->state());
        }
        else
        {
            notifyError(error);
        }
    });
}

void CameraWorker::requestStopStreaming()
{
    enqueueCommand([this]() {
        streamEnabled_ = false;
        waitForStreamIdle();
        controller_->stop();
        notifyState(controller_->state());
    });
}

void CameraWorker::requestDisconnect()
{
    requestDisconnectOnGuiThread();
}

void CameraWorker::requestDisconnectOnGuiThread()
{
    enqueueCommand([this]() {
        streamEnabled_ = false;
        waitForStreamIdle();

        controller_->stop();

        const auto runDisconnect = [this]() { controller_->disconnect(); };
        if (controller_->requiresGuiThreadForSdkLifecycle() && guiTaskRunner_)
            guiTaskRunner_(runDisconnect);
        else
            runDisconnect();

        notifyState(controller_->state());
    });
}

void CameraWorker::setGuiAsyncTaskRunner(GuiAsyncTaskRunner runner)
{
    std::lock_guard<std::mutex> lock(callbackMutex_);
    guiAsyncTaskRunner_ = std::move(runner);
}

void CameraWorker::requestOpenShutter()
{
    enqueueCommand([this]() {
        CameraError error;
        if (!controller_->openShutter(error))
            notifyError(error);
        else
            publishShutterState();
    });
}

void CameraWorker::requestCloseShutter()
{
    enqueueCommand([this]() {
        CameraError error;
        if (!controller_->closeShutter(error))
            notifyError(error);
        else
            publishShutterState();
    });
}

void CameraWorker::requestQueryShutterState()
{
    enqueueCommand([this]() { publishShutterState(); });
}

void CameraWorker::publishShutterState()
{
    bool isOpen = false;
    CameraError error;
    if (!controller_->shutterIsOpen(isOpen, error))
        return;

    std::lock_guard<std::mutex> lock(callbackMutex_);
    if (shutterStateCallback_)
        shutterStateCallback_(isOpen);
}

void CameraWorker::shutdownSync()
{
    streamEnabled_ = false;
    waitForStreamIdle();

    {
        std::lock_guard<std::mutex> lock(callbackMutex_);
        frameCallback_ = nullptr;
        settingsAppliedCallback_ = nullptr;
    }

    if (controller_)
    {
        const auto teardown = [this]() {
            controller_->stop();
            controller_->disconnect();
        };

        const bool onGuiThread =
            QCoreApplication::instance() != nullptr
            && QThread::currentThread() == QCoreApplication::instance()->thread();
        if (controller_->requiresGuiThreadForSdkLifecycle() && guiTaskRunner_ && !onGuiThread)
            guiTaskRunner_(teardown);
        else
            teardown();
    }
    stop();
}

CameraState CameraWorker::currentState() const
{
    return controller_->state();
}

std::string CameraWorker::name() const
{
    return controller_->name();
}

void CameraWorker::setStateCallback(StateCallback callback)
{
    std::lock_guard<std::mutex> lock(callbackMutex_);
    stateCallback_ = std::move(callback);
}

void CameraWorker::setErrorCallback(ErrorCallback callback)
{
    std::lock_guard<std::mutex> lock(callbackMutex_);
    errorCallback_ = std::move(callback);
}

void CameraWorker::setFrameCallback(FrameCallback callback)
{
    std::lock_guard<std::mutex> lock(callbackMutex_);
    frameCallback_ = std::move(callback);
}

void CameraWorker::setShutterStateCallback(ShutterStateCallback callback)
{
    std::lock_guard<std::mutex> lock(callbackMutex_);
    shutterStateCallback_ = std::move(callback);
}

void CameraWorker::setSettingsAppliedCallback(SettingsAppliedCallback callback)
{
    std::lock_guard<std::mutex> lock(callbackMutex_);
    settingsAppliedCallback_ = std::move(callback);
}

void CameraWorker::enqueueCommand(ControlCommand command)
{
    {
        std::lock_guard<std::mutex> lock(commandMutex_);
        commandQueue_.push_back(std::move(command));
    }
    commandCv_.notify_one();
}

void CameraWorker::controlLoop()
{
    while (running_)
    {
        ControlCommand command;
        {
            std::unique_lock<std::mutex> lock(commandMutex_);
            commandCv_.wait(lock, [this]() { return !running_ || !commandQueue_.empty(); });
            if (!running_ && commandQueue_.empty())
                break;

            command = std::move(commandQueue_.front());
            commandQueue_.pop_front();
        }

        if (command)
            command();
    }
}

void CameraWorker::streamLoop()
{
    while (running_)
    {
        if (!streamEnabled_)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        FramePacket frame;
        CameraError error;
        streamInPoll_.store(true);
        const bool gotFrame = controller_->pollFrame(frame, kFramePollTimeoutMs, error);
        streamInPoll_.store(false);

        if (!streamEnabled_.load())
            continue;

        if (gotFrame)
        {
            std::lock_guard<std::mutex> lock(callbackMutex_);
            if (frameCallback_)
                frameCallback_(frame);
            continue;
        }

        if (error.code == CameraErrorCode::Timeout)
            continue;

        if (error.code == CameraErrorCode::InvalidState)
            continue;

        notifyError(error);
        streamEnabled_ = false;
        controller_->stop();
        notifyState(controller_->state());
    }
}

void CameraWorker::notifyState(CameraState state)
{
    std::lock_guard<std::mutex> lock(callbackMutex_);
    if (stateCallback_)
        stateCallback_(state);
}

void CameraWorker::notifyError(const CameraError &error)
{
    std::lock_guard<std::mutex> lock(callbackMutex_);
    if (errorCallback_)
        errorCallback_(error);
}

void CameraWorker::notifySettingsApplied(const CameraSettingsApplyReport &report)
{
    std::lock_guard<std::mutex> lock(callbackMutex_);
    if (settingsAppliedCallback_)
        settingsAppliedCallback_(report);
}
