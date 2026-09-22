// Threaded BFS camera worker (backend/multiview).
#include "backend/multiview/BfsCameraWorker.hpp"

#include <chrono>
#include <utility>

namespace hf::bfs
{
BfsCameraWorker::BfsCameraWorker()
    : camera_(std::make_unique<BfsSpinnakerCamera>())
{
}

BfsCameraWorker::~BfsCameraWorker()
{
    shutdownSync();
}

void BfsCameraWorker::start()
{
    if (running_.exchange(true))
        return;
    controlThread_ = std::thread([this]() { controlLoop(); });
    streamThread_ = std::thread([this]() { streamLoop(); });
}

void BfsCameraWorker::stop()
{
    shutdownSync();
}

void BfsCameraWorker::shutdownSync()
{
    // Prefer a clean Spinnaker stop/disconnect on the control thread while it is still alive.
    if (running_.load())
    {
        streamEnabled_ = false;

        std::mutex doneMutex;
        std::condition_variable doneCv;
        bool done = false;

        enqueue([this, &doneMutex, &doneCv, &done]() {
            while (streamInPoll_.load())
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            if (camera_)
            {
                camera_->stopStreaming();
                camera_->disconnect();
            }
            state_ = BfsCameraState::Disconnected;
            {
                std::lock_guard<std::mutex> lock(doneMutex);
                done = true;
            }
            doneCv.notify_one();
        });

        {
            std::unique_lock<std::mutex> lock(doneMutex);
            doneCv.wait_for(lock, std::chrono::seconds(8), [&done]() { return done; });
        }

        running_ = false;
        commandCv_.notify_all();
        if (controlThread_.joinable())
            controlThread_.join();
        if (streamThread_.joinable())
            streamThread_.join();
    }

    // Fallback if the control thread never ran the disconnect (or was already stopped).
    if (camera_)
    {
        camera_->stopStreaming();
        camera_->disconnect();
    }
    state_ = BfsCameraState::Disconnected;
}

void BfsCameraWorker::enqueue(ControlCommand command)
{
    {
        std::lock_guard<std::mutex> lock(commandMutex_);
        commandQueue_.push_back(std::move(command));
    }
    commandCv_.notify_one();
}

void BfsCameraWorker::requestEnumerate()
{
    enqueue([this]() {
        BfsError error;
        const std::vector<BfsDeviceInfo> devices = BfsSpinnakerCamera::enumerateDevices(&error);
        if (error.code != BfsErrorCode::None)
            notifyError(error);
        DevicesCallback cb;
        {
            std::lock_guard<std::mutex> lock(callbackMutex_);
            cb = devicesCallback_;
        }
        if (cb)
            cb(devices);
    });
}

void BfsCameraWorker::requestConnect(const BfsCameraSettings &settings)
{
    enqueue([this, settings]() {
        streamEnabled_ = false;
        while (streamInPoll_)
            std::this_thread::sleep_for(std::chrono::milliseconds(5));

        camera_->stopStreaming();
        camera_->disconnect();

        BfsError error;
        if (!camera_->connect(settings.cameraId, error))
        {
            notifyState(BfsCameraState::Fault);
            notifyError(error);
            return;
        }
        notifyState(BfsCameraState::Connected);

        if (!camera_->applySettings(settings, error))
        {
            notifyError(error);
            // Still try to stream with defaults.
        }

        if (!camera_->startStreaming(error))
        {
            notifyState(BfsCameraState::Fault);
            notifyError(error);
            camera_->disconnect();
            notifyState(BfsCameraState::Disconnected);
            return;
        }

        notifyState(BfsCameraState::Streaming);
        consecutiveTimeouts_ = 0;
        streamEnabled_ = true;
    });
}

void BfsCameraWorker::requestApplySettings(const BfsCameraSettings &settings)
{
    enqueue([this, settings]() {
        if (state_ != BfsCameraState::Connected && state_ != BfsCameraState::Streaming)
            return;
        const bool wasStreaming = streamEnabled_.exchange(false);
        while (streamInPoll_)
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        if (wasStreaming)
            camera_->stopStreaming();

        BfsError error;
        if (!camera_->applySettings(settings, error))
            notifyError(error);

        if (wasStreaming || state_ == BfsCameraState::Streaming
            || state_ == BfsCameraState::Connected)
        {
            if (!camera_->startStreaming(error))
            {
                notifyError(error);
                notifyState(camera_->state());
                return;
            }
            notifyState(BfsCameraState::Streaming);
            consecutiveTimeouts_ = 0;
            streamEnabled_ = true;
        }
    });
}

bool BfsCameraWorker::applySettingsBlocking(const BfsCameraSettings &settings,
                                            BfsError *errorOut,
                                            const int timeoutMs)
{
    if (!running_.load())
    {
        if (errorOut != nullptr)
            *errorOut = {BfsErrorCode::InvalidState, "BFS worker not running.", false};
        return false;
    }

    std::mutex doneMutex;
    std::condition_variable doneCv;
    bool done = false;
    bool ok = false;
    BfsError error;

    enqueue([this, settings, &doneMutex, &doneCv, &done, &ok, &error]() {
        if (state_ != BfsCameraState::Connected && state_ != BfsCameraState::Streaming)
        {
            error = {BfsErrorCode::InvalidState, "BFS not connected.", false};
            ok = false;
        }
        else
        {
            const bool wasStreaming = streamEnabled_.exchange(false);
            while (streamInPoll_)
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            if (wasStreaming)
                camera_->stopStreaming();

            if (!camera_->applySettings(settings, error))
            {
                ok = false;
                notifyError(error);
            }
            else
            {
                ok = true;
            }

            if (wasStreaming || state_ == BfsCameraState::Streaming
                || state_ == BfsCameraState::Connected)
            {
                BfsError streamError;
                if (!camera_->startStreaming(streamError))
                {
                    ok = false;
                    error = streamError;
                    notifyError(streamError);
                    notifyState(camera_->state());
                }
                else
                {
                    notifyState(BfsCameraState::Streaming);
                    consecutiveTimeouts_ = 0;
                    streamEnabled_ = true;
                }
            }
        }
        {
            std::lock_guard<std::mutex> lock(doneMutex);
            done = true;
        }
        doneCv.notify_one();
    });

    {
        std::unique_lock<std::mutex> lock(doneMutex);
        if (!doneCv.wait_for(lock, std::chrono::milliseconds(std::max(1, timeoutMs)),
                             [&done]() { return done; }))
        {
            if (errorOut != nullptr)
                *errorOut = {BfsErrorCode::Timeout, "BFS applySettings timed out.", false};
            return false;
        }
    }

    if (errorOut != nullptr)
        *errorOut = error;
    return ok;
}

void BfsCameraWorker::requestDisconnect()
{
    enqueue([this]() {
        streamEnabled_ = false;
        while (streamInPoll_)
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        camera_->stopStreaming();
        camera_->disconnect();
        notifyState(BfsCameraState::Disconnected);
    });
}

BfsCameraState BfsCameraWorker::currentState() const
{
    return state_.load();
}

void BfsCameraWorker::setStateCallback(StateCallback callback)
{
    std::lock_guard<std::mutex> lock(callbackMutex_);
    stateCallback_ = std::move(callback);
}

void BfsCameraWorker::setErrorCallback(ErrorCallback callback)
{
    std::lock_guard<std::mutex> lock(callbackMutex_);
    errorCallback_ = std::move(callback);
}

void BfsCameraWorker::setFrameCallback(FrameCallback callback)
{
    std::lock_guard<std::mutex> lock(callbackMutex_);
    frameCallback_ = std::move(callback);
}

void BfsCameraWorker::setDevicesCallback(DevicesCallback callback)
{
    std::lock_guard<std::mutex> lock(callbackMutex_);
    devicesCallback_ = std::move(callback);
}

void BfsCameraWorker::notifyState(const BfsCameraState state)
{
    state_ = state;
    StateCallback cb;
    {
        std::lock_guard<std::mutex> lock(callbackMutex_);
        cb = stateCallback_;
    }
    if (cb)
        cb(state);
}

void BfsCameraWorker::notifyError(const BfsError &error)
{
    ErrorCallback cb;
    {
        std::lock_guard<std::mutex> lock(callbackMutex_);
        cb = errorCallback_;
    }
    if (cb)
        cb(error);
}

void BfsCameraWorker::controlLoop()
{
    while (running_)
    {
        ControlCommand command;
        {
            std::unique_lock<std::mutex> lock(commandMutex_);
            commandCv_.wait(lock, [this]() { return !commandQueue_.empty() || !running_; });
            if (!running_ && commandQueue_.empty())
                break;
            if (commandQueue_.empty())
                continue;
            command = std::move(commandQueue_.front());
            commandQueue_.pop_front();
        }
        if (command)
            command();
    }
}

void BfsCameraWorker::streamLoop()
{
    while (running_)
    {
        if (!streamEnabled_)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }

        streamInPoll_ = true;
        BfsRgbFrame frame;
        BfsError error;
        const bool ok = camera_->pollFrame(frame, 2000, error);
        streamInPoll_ = false;

        if (!streamEnabled_)
            continue;

        if (!ok)
        {
            if (error.code == BfsErrorCode::Timeout)
            {
                const int n = ++consecutiveTimeouts_;
                // ~5s at 500 ms poll — surface once so blank preview is diagnosable.
                if (n == 5)
                {
                    notifyError(
                        {BfsErrorCode::Timeout,
                         "BFS streaming but no frames (GetNextImage timeout). "
                         "Try Frame Rate ≤5 Hz, raise Device Link Throughput Limit, "
                         "TriggerMode=Off, close SpinView, check GigE NIC.",
                         false});
                }
                continue;
            }
            consecutiveTimeouts_ = 0;
            notifyError(error);
            continue;
        }

        consecutiveTimeouts_ = 0;

        FrameCallback cb;
        {
            std::lock_guard<std::mutex> lock(callbackMutex_);
            cb = frameCallback_;
        }
        if (cb)
            cb(frame);
    }
}
} // namespace hf::bfs
