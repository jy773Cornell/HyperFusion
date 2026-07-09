#include "backend/stage/StageWorker.hpp"

#include <algorithm>
#include <future>

StageWorker::StageWorker(std::shared_ptr<IStageController> controller)
    : controller_(std::move(controller))
{
}

StageWorker::~StageWorker()
{
    stop();
}

void StageWorker::start()
{
    if (running_.exchange(true))
        return;

    controlThread_ = std::thread(&StageWorker::controlLoop, this);
}

void StageWorker::stop()
{
    shutdownSync();
}

void StageWorker::requestConnect(const StageConnectSettings &settings)
{
    enqueueCommand([this, settings]() {
        StageError error;
        notifyState(StageState::Connecting);

        if (controller_->connect(settings, error))
        {
            notifyTopology(controller_->topology());
            notifyState(StageState::Homing);

            if (controller_->homeWithLocalization(error))
            {
                notifyTopology(controller_->topology());
                notifyState(controller_->state());
            }
            else
            {
                controller_->disconnect();
                notifyTopology({});
                notifyState(StageState::Fault);
                notifyError(error);
            }
        }
        else
        {
            notifyState(controller_->state());
            notifyError(error);
        }
    });
}

void StageWorker::requestDisconnect()
{
    requestDisconnectWithHoming(nullptr);
}

void StageWorker::requestDisconnectWithHoming(std::function<void()> onComplete)
{
    enqueueCommand([this, onComplete = std::move(onComplete)]() {
        if (controller_ && controller_->state() == StageState::Connected)
        {
            notifyState(StageState::Homing);
            StageError error;
            if (!controller_->home(error))
                notifyError(error);
        }

        if (controller_)
            controller_->disconnect();
        notifyTopology({});
        notifyState(StageState::Disconnected);

        if (onComplete)
            onComplete();
    });
}

void StageWorker::shutdownSync(const bool homeBeforeDisconnect)
{
    if (!running_.load())
        return;

    std::promise<void> done;
    auto future = done.get_future();
    enqueuePriorityCommand([this, homeBeforeDisconnect, &done]() {
        if (controller_ && controller_->state() == StageState::Connected)
        {
            controller_->stopMotion();
            if (homeBeforeDisconnect)
            {
                notifyState(StageState::Homing);
                StageError error;
                (void)controller_->home(error);
            }
        }

        if (controller_)
            controller_->disconnect();
        notifyTopology({});
        notifyState(StageState::Disconnected);
        done.set_value();
    });
    commandCv_.notify_all();
    future.wait();

    if (!running_.exchange(false))
        return;

    commandCv_.notify_all();
    if (controlThread_.joinable())
        controlThread_.join();
}

void StageWorker::requestStopMotion(std::function<void()> onComplete, const bool waitUntilIdle)
{
    auto command = [this, onComplete = std::move(onComplete), waitUntilIdle]() {
        if (controller_)
            controller_->stopMotion(waitUntilIdle);

        if (onComplete)
            onComplete();
    };

    if (waitUntilIdle)
        enqueuePriorityCommand(std::move(command));
    else
        enqueueCommand(std::move(command));
}

void StageWorker::requestHome()
{
    enqueueCommand([this]() {
        StageError error;
        notifyState(StageState::Homing);

        if (controller_->home(error))
        {
            notifyTopology(controller_->topology());
            notifyState(StageState::Connected);
        }
        else
        {
            notifyState(StageState::Connected);
            notifyError(error);
        }
    });
}

void StageWorker::requestPrimaryPosition(PositionCallback callback)
{
    {
        std::lock_guard lock(commandMutex_);
        purgePendingPositionPolls();
    }

    enqueueCommand(
        [this, callback = std::move(callback)]() {
            if (!controller_ || !callback)
                return;

            StageError error;
            double positionMm = 0.0;
            const bool ok = controller_->getPrimaryPositionMm(positionMm, error);
            callback(positionMm, ok);
        },
        CommandKind::PositionPoll);
}

void StageWorker::requestMoveRelativeMm(const double distanceMm, const double speedMmPerSec)
{
    enqueueCommand([this, distanceMm, speedMmPerSec]() {
        StageError error;
        if (controller_->moveRelativeMm(distanceMm, speedMmPerSec, error))
            return;

        notifyError(error);
    });
}

void StageWorker::requestMoveAbsoluteMm(const double positionMm,
                                        const double speedMmPerSec,
                                        const bool waitUntilIdle)
{
    requestMoveAbsoluteMm(positionMm, speedMmPerSec, waitUntilIdle, nullptr);
}

void StageWorker::requestMoveAbsoluteMm(const double positionMm,
                                        const double speedMmPerSec,
                                        const bool waitUntilIdle,
                                        std::function<void(bool success)> onComplete)
{
    enqueueCommand([this, positionMm, speedMmPerSec, waitUntilIdle, onComplete = std::move(onComplete)]() {
        StageError error;
        const bool ok = controller_->moveAbsoluteMm(positionMm, speedMmPerSec, waitUntilIdle, error);
        if (!ok)
            notifyError(error);

        if (onComplete)
            onComplete(ok);
    });
}

void StageWorker::requestMoveVelocityMm(const double velocityMmPerSec)
{
    enqueuePriorityCommand([this, velocityMmPerSec]() {
        StageError error;
        if (controller_->moveVelocityMm(velocityMmPerSec, error))
            return;

        notifyError(error);
    });
}

StageState StageWorker::currentState() const
{
    return controller_ ? controller_->state() : StageState::Disconnected;
}

StageTopology StageWorker::currentTopology() const
{
    return controller_ ? controller_->topology() : StageTopology{};
}

void StageWorker::setStateCallback(StateCallback callback)
{
    std::lock_guard lock(callbackMutex_);
    stateCallback_ = std::move(callback);
}

void StageWorker::setTopologyCallback(TopologyCallback callback)
{
    std::lock_guard lock(callbackMutex_);
    topologyCallback_ = std::move(callback);
}

void StageWorker::setErrorCallback(ErrorCallback callback)
{
    std::lock_guard lock(callbackMutex_);
    errorCallback_ = std::move(callback);
}

void StageWorker::enqueueCommand(ControlCommand command, const CommandKind kind)
{
    {
        std::lock_guard lock(commandMutex_);
        commandQueue_.push_back({kind, std::move(command)});
    }
    commandCv_.notify_one();
}

void StageWorker::purgePendingPositionPolls()
{
    commandQueue_.erase(std::remove_if(commandQueue_.begin(),
                                       commandQueue_.end(),
                                       [](const QueuedCommand &cmd) {
                                           return cmd.kind == CommandKind::PositionPoll;
                                       }),
                        commandQueue_.end());
}

void StageWorker::enqueuePriorityCommand(ControlCommand command)
{
    {
        std::lock_guard lock(commandMutex_);
        purgePendingPositionPolls();
        commandQueue_.push_front({CommandKind::Normal, std::move(command)});
    }
    commandCv_.notify_one();
}

void StageWorker::controlLoop()
{
    while (running_)
    {
        QueuedCommand queued;
        {
            std::unique_lock lock(commandMutex_);
            commandCv_.wait(lock, [this]() { return !commandQueue_.empty() || !running_; });
            if (!running_ && commandQueue_.empty())
                break;
            if (commandQueue_.empty())
                continue;

            queued = std::move(commandQueue_.front());
            commandQueue_.pop_front();
        }

        commandInFlight_.store(true, std::memory_order_release);
        if (queued.fn)
            queued.fn();
        commandInFlight_.store(false, std::memory_order_release);
    }
}

void StageWorker::notifyState(const StageState state)
{
    StateCallback callback;
    {
        std::lock_guard lock(callbackMutex_);
        callback = stateCallback_;
    }
    if (callback)
        callback(state);
}

void StageWorker::notifyTopology(const StageTopology &topology)
{
    TopologyCallback callback;
    {
        std::lock_guard lock(callbackMutex_);
        callback = topologyCallback_;
    }
    if (callback)
        callback(topology);
}

void StageWorker::notifyError(const StageError &error)
{
    ErrorCallback callback;
    {
        std::lock_guard lock(callbackMutex_);
        callback = errorCallback_;
    }
    if (callback)
        callback(error);
}
