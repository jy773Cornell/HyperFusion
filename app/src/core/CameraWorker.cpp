// Threaded camera worker implementation for command/control and streaming loops.
#include "core/CameraWorker.hpp"

#include <chrono>

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

        const auto runOnGui = [this](const std::function<void()> &task) {
            if (guiTaskRunner_)
                guiTaskRunner_(task);
            else
                task();
        };

        runOnGui([this, &success, &error]() {
            const CameraState stateBefore = controller_->state();
            if (stateBefore == CameraState::Disconnected || stateBefore == CameraState::Fault)
            {
                if (!controller_->connect(error))
                    return;

                notifyState(controller_->state());
            }

            if (controller_->initialize(error))
            {
                success = true;
                notifyState(controller_->state());
            }
        });

        if (!success && error.code != CameraErrorCode::None)
            notifyError(error);
    });
}

void CameraWorker::setGuiTaskRunner(GuiTaskRunner runner)
{
    std::lock_guard<std::mutex> lock(callbackMutex_);
    guiTaskRunner_ = std::move(runner);
}

void CameraWorker::requestApplySettings(const CameraSettings &settings)
{
    enqueueCommand([this, settings]() {
        CameraError error;
        if (controller_->applySettings(settings, error))
            notifyState(controller_->state());
        else
            notifyError(error);
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
        controller_->stop();
        notifyState(controller_->state());
    });
}

void CameraWorker::requestDisconnect()
{
    enqueueCommand([this]() {
        streamEnabled_ = false;
        controller_->disconnect();
        notifyState(controller_->state());
    });
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
        if (controller_->pollFrame(frame, kFramePollTimeoutMs, error))
        {
            std::lock_guard<std::mutex> lock(callbackMutex_);
            if (frameCallback_)
                frameCallback_(frame);
            continue;
        }

        if (error.code == CameraErrorCode::Timeout)
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
