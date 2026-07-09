#pragma once

#include "backend/light/ILighthouseController.hpp"

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

class LighthouseWorker
{
public:
    using StateCallback = std::function<void(LighthouseState)>;
    using DeviceInfoCallback = std::function<void(const LighthouseDeviceInfo &)>;
    using SettingsCallback = std::function<void(const LighthouseSettings &)>;
    using PowerStatusCallback = std::function<void(const LighthouseControllerPowerStatus &)>;
    using ErrorCallback = std::function<void(const LighthouseError &)>;

    explicit LighthouseWorker(std::shared_ptr<ILighthouseController> controller);
    ~LighthouseWorker();

    void start();
    void stop();

    void requestScan();
    void requestConnect(const LighthouseSettings &connectDefaults);
    void requestDisconnect();
    void shutdownSync();

    void requestSetGroupIntensity(LighthouseIntensityGroup group, int percent);
    void requestSetLampOn(LighthouseLamp lamp, bool on);
    void requestPollControllerPowerStatus();

    LighthouseState currentState() const;
    LighthouseDeviceInfo currentDeviceInfo() const;
    LighthouseSettings currentSettings() const;
    LighthouseControllerPowerStatus currentControllerPowerStatus() const;

    void setStateCallback(StateCallback callback);
    void setDeviceInfoCallback(DeviceInfoCallback callback);
    void setSettingsCallback(SettingsCallback callback);
    void setPowerStatusCallback(PowerStatusCallback callback);
    void setErrorCallback(ErrorCallback callback);

private:
    using ControlCommand = std::function<void()>;

    void enqueueCommand(ControlCommand command);
    void enqueuePriorityCommand(ControlCommand command);
    void controlLoop();
    void notifyState(LighthouseState state);
    void notifyDeviceInfo(const LighthouseDeviceInfo &info);
    void notifySettings(const LighthouseSettings &settings);
    void notifyPowerStatus(const LighthouseControllerPowerStatus &status);
    void notifyError(const LighthouseError &error);

    std::shared_ptr<ILighthouseController> controller_;

    mutable std::mutex callbackMutex_;
    StateCallback stateCallback_;
    DeviceInfoCallback deviceInfoCallback_;
    SettingsCallback settingsCallback_;
    PowerStatusCallback powerStatusCallback_;
    ErrorCallback errorCallback_;

    LighthouseControllerPowerStatus powerStatus_;

    mutable std::mutex commandMutex_;
    std::condition_variable commandCv_;
    std::deque<ControlCommand> commandQueue_;

    std::atomic<bool> running_{false};
    std::thread controlThread_;
};
