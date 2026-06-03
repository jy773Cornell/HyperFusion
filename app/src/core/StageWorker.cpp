#include "core/StageWorker.hpp"

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
    if (!running_.exchange(false))
        return;

    enqueueCommand([this]() {
        if (controller_)
            controller_->disconnect();
    });
    commandCv_.notify_all();

    if (controlThread_.joinable())
        controlThread_.join();
}

void StageWorker::requestConnect(const StageConnectSettings &settings)
{
    enqueueCommand([this, settings]() {
        StageError error;
        notifyState(StageState::Connecting);

        if (controller_->connect(settings, error))
        {
            notifyTopology(controller_->topology());
            notifyState(controller_->state());
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
    enqueueCommand([this]() {
        controller_->disconnect();
        notifyTopology({});
        notifyState(controller_->state());
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

void StageWorker::enqueueCommand(ControlCommand command)
{
    {
        std::lock_guard lock(commandMutex_);
        commandQueue_.push_back(std::move(command));
    }
    commandCv_.notify_one();
}

void StageWorker::controlLoop()
{
    while (running_)
    {
        ControlCommand command;
        {
            std::unique_lock lock(commandMutex_);
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
