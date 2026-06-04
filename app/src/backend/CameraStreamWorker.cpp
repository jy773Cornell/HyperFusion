// Frame polling loop on a dedicated thread (separate from camera control commands).
#include "backend/CameraStreamWorker.hpp"

#include <chrono>

namespace
{
constexpr std::uint32_t kFramePollTimeoutMs = 100;
constexpr int kIdlePollMs = 10;
} // namespace

CameraStreamWorker::CameraStreamWorker(std::shared_ptr<ICameraController> controller)
    : controller_(std::move(controller))
{
}

CameraStreamWorker::~CameraStreamWorker()
{
    stop();
}

void CameraStreamWorker::start()
{
    if (running_.exchange(true))
        return;

    streamThread_ = std::thread(&CameraStreamWorker::streamLoop, this);
}

void CameraStreamWorker::stop()
{
    if (!running_.exchange(false))
        return;

    enabled_ = false;

    if (streamThread_.joinable())
        streamThread_.join();
}

void CameraStreamWorker::setEnabled(const bool enabled)
{
    enabled_ = enabled;
}

bool CameraStreamWorker::enabled() const
{
    return enabled_.load();
}

void CameraStreamWorker::setFrameCallback(FrameCallback callback)
{
    std::lock_guard<std::mutex> lock(callbackMutex_);
    frameCallback_ = std::move(callback);
}

void CameraStreamWorker::setErrorCallback(ErrorCallback callback)
{
    std::lock_guard<std::mutex> lock(callbackMutex_);
    errorCallback_ = std::move(callback);
}

void CameraStreamWorker::setStateCallback(StateCallback callback)
{
    std::lock_guard<std::mutex> lock(callbackMutex_);
    stateCallback_ = std::move(callback);
}

void CameraStreamWorker::streamLoop()
{
    while (running_)
    {
        if (!enabled_)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(kIdlePollMs));
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

        enabled_ = false;
        controller_->stop();
        notifyError(error);
        notifyState(controller_->state());
    }
}

void CameraStreamWorker::notifyError(const CameraError &error)
{
    std::lock_guard<std::mutex> lock(callbackMutex_);
    if (errorCallback_)
        errorCallback_(error);
}

void CameraStreamWorker::notifyState(const CameraState state)
{
    std::lock_guard<std::mutex> lock(callbackMutex_);
    if (stateCallback_)
        stateCallback_(state);
}
