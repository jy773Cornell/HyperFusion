#include "backend/LighthouseWorker.hpp"

#include <future>

LighthouseWorker::LighthouseWorker(std::shared_ptr<ILighthouseController> controller)
    : controller_(std::move(controller))
{
}

LighthouseWorker::~LighthouseWorker()
{
    stop();
}

void LighthouseWorker::start()
{
    if (running_.exchange(true))
        return;

    controlThread_ = std::thread(&LighthouseWorker::controlLoop, this);
}

void LighthouseWorker::stop()
{
    shutdownSync();
}

void LighthouseWorker::requestScan()
{
    enqueueCommand([this]() {
        LighthouseError error;
        notifyState(LighthouseState::Scanning);

        if (controller_->scan(error))
        {
            notifyDeviceInfo(controller_->deviceInfo());
            notifyState(controller_->state());
            return;
        }

        notifyState(LighthouseState::Fault);
        notifyError(error);
    });
}

void LighthouseWorker::requestConnect(const LighthouseSettings &connectDefaults)
{
    enqueueCommand([this, connectDefaults]() {
        LighthouseError error;
        notifyState(LighthouseState::Connecting);

        if (controller_)
            controller_->setConnectDefaults(connectDefaults);

        if (controller_->connect(error))
        {
            notifyDeviceInfo(controller_->deviceInfo());
            notifySettings(controller_->settings());
            notifyState(controller_->state());

            LighthouseControllerPowerStatus status;
            if (controller_->pollControllerPowerStatus(status, error))
            {
                powerStatus_ = status;
                notifyPowerStatus(status);
            }
            else
            {
                notifyError(error);
            }
            return;
        }

        notifyState(controller_->state());
        notifyError(error);
    });
}

void LighthouseWorker::requestDisconnect()
{
    enqueueCommand([this]() {
        if (controller_)
        {
            LighthouseError error;
            if (controller_->state() == LighthouseState::Connected)
                (void)controller_->shutdownAll(error);
            controller_->disconnect();
            notifyDeviceInfo(controller_->deviceInfo());
            notifySettings(controller_->settings());
            powerStatus_ = {};
            notifyPowerStatus(powerStatus_);
        }

        notifyState(LighthouseState::Disconnected);
    });
}

void LighthouseWorker::shutdownSync()
{
    if (!running_.load())
        return;

    std::promise<void> done;
    auto future = done.get_future();
    enqueuePriorityCommand([this, &done]() {
        if (controller_)
        {
            LighthouseError error;
            if (controller_->state() == LighthouseState::Connected)
                (void)controller_->shutdownAll(error);
            controller_->disconnect();
            notifyDeviceInfo(controller_->deviceInfo());
            notifySettings(controller_->settings());
            powerStatus_ = {};
            notifyPowerStatus(powerStatus_);
        }

        notifyState(LighthouseState::Disconnected);
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

void LighthouseWorker::requestSetGroupIntensity(const LighthouseIntensityGroup group, const int percent)
{
    enqueueCommand([this, group, percent]() {
        LighthouseError error;
        if (controller_->setGroupIntensityPercent(group, percent, error))
        {
            notifySettings(controller_->settings());
            return;
        }

        notifyError(error);
    });
}

void LighthouseWorker::requestSetLampOn(const LighthouseLamp lamp, const bool on)
{
    enqueueCommand([this, lamp, on]() {
        LighthouseError error;
        if (controller_->setLampOn(lamp, on, error))
        {
            notifySettings(controller_->settings());
            return;
        }

        notifyError(error);
    });
}

void LighthouseWorker::requestPollControllerPowerStatus()
{
    enqueueCommand([this]() {
        if (controller_ == nullptr || controller_->state() != LighthouseState::Connected)
            return;

        LighthouseError error;
        LighthouseControllerPowerStatus status;
        if (!controller_->pollControllerPowerStatus(status, error))
        {
            notifyError(error);
            return;
        }

        powerStatus_ = status;
        notifyPowerStatus(status);
    });
}

LighthouseState LighthouseWorker::currentState() const
{
    return controller_ ? controller_->state() : LighthouseState::Disconnected;
}

LighthouseDeviceInfo LighthouseWorker::currentDeviceInfo() const
{
    return controller_ ? controller_->deviceInfo() : LighthouseDeviceInfo{};
}

LighthouseSettings LighthouseWorker::currentSettings() const
{
    return controller_ ? controller_->settings() : LighthouseSettings{};
}

LighthouseControllerPowerStatus LighthouseWorker::currentControllerPowerStatus() const
{
    return powerStatus_;
}

void LighthouseWorker::setStateCallback(StateCallback callback)
{
    std::lock_guard lock(callbackMutex_);
    stateCallback_ = std::move(callback);
}

void LighthouseWorker::setDeviceInfoCallback(DeviceInfoCallback callback)
{
    std::lock_guard lock(callbackMutex_);
    deviceInfoCallback_ = std::move(callback);
}

void LighthouseWorker::setSettingsCallback(SettingsCallback callback)
{
    std::lock_guard lock(callbackMutex_);
    settingsCallback_ = std::move(callback);
}

void LighthouseWorker::setPowerStatusCallback(PowerStatusCallback callback)
{
    std::lock_guard lock(callbackMutex_);
    powerStatusCallback_ = std::move(callback);
}

void LighthouseWorker::setErrorCallback(ErrorCallback callback)
{
    std::lock_guard lock(callbackMutex_);
    errorCallback_ = std::move(callback);
}

void LighthouseWorker::enqueueCommand(ControlCommand command)
{
    {
        std::lock_guard lock(commandMutex_);
        commandQueue_.push_back(std::move(command));
    }
    commandCv_.notify_one();
}

void LighthouseWorker::enqueuePriorityCommand(ControlCommand command)
{
    {
        std::lock_guard lock(commandMutex_);
        commandQueue_.push_front(std::move(command));
    }
    commandCv_.notify_one();
}

void LighthouseWorker::controlLoop()
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

void LighthouseWorker::notifyState(const LighthouseState state)
{
    StateCallback callback;
    {
        std::lock_guard lock(callbackMutex_);
        callback = stateCallback_;
    }
    if (callback)
        callback(state);
}

void LighthouseWorker::notifyDeviceInfo(const LighthouseDeviceInfo &info)
{
    DeviceInfoCallback callback;
    {
        std::lock_guard lock(callbackMutex_);
        callback = deviceInfoCallback_;
    }
    if (callback)
        callback(info);
}

void LighthouseWorker::notifySettings(const LighthouseSettings &settings)
{
    SettingsCallback callback;
    {
        std::lock_guard lock(callbackMutex_);
        callback = settingsCallback_;
    }
    if (callback)
        callback(settings);
}

void LighthouseWorker::notifyPowerStatus(const LighthouseControllerPowerStatus &status)
{
    PowerStatusCallback callback;
    {
        std::lock_guard lock(callbackMutex_);
        callback = powerStatusCallback_;
    }
    if (callback)
        callback(status);
}

void LighthouseWorker::notifyError(const LighthouseError &error)
{
    if (error.code == LighthouseErrorCode::None && error.message.empty())
        return;

    ErrorCallback callback;
    {
        std::lock_guard lock(callbackMutex_);
        callback = errorCallback_;
    }
    if (callback)
        callback(error);
}
