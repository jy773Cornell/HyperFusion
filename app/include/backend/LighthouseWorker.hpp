#pragma once

#include "backend/ILighthouseController.hpp"

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
    using ErrorCallback = std::function<void(const LighthouseError &)>;

    explicit LighthouseWorker(std::shared_ptr<ILighthouseController> controller);
    ~LighthouseWorker();

    void start();
    void stop();

    void requestScan();
    void requestConnect();
    void requestDisconnect();
    void shutdownSync();

    void requestSetGroupIntensity(LighthouseIntensityGroup group, int percent);
    void requestSetLampOn(LighthouseLamp lamp, bool on);

    LighthouseState currentState() const;
    LighthouseDeviceInfo currentDeviceInfo() const;
    LighthouseSettings currentSettings() const;

    void setStateCallback(StateCallback callback);
    void setDeviceInfoCallback(DeviceInfoCallback callback);
    void setSettingsCallback(SettingsCallback callback);
    void setErrorCallback(ErrorCallback callback);

private:
    using ControlCommand = std::function<void()>;

    void enqueueCommand(ControlCommand command);
    void enqueuePriorityCommand(ControlCommand command);
    void controlLoop();
    void notifyState(LighthouseState state);
    void notifyDeviceInfo(const LighthouseDeviceInfo &info);
    void notifySettings(const LighthouseSettings &settings);
    void notifyError(const LighthouseError &error);

    std::shared_ptr<ILighthouseController> controller_;

    mutable std::mutex callbackMutex_;
    StateCallback stateCallback_;
    DeviceInfoCallback deviceInfoCallback_;
    SettingsCallback settingsCallback_;
    ErrorCallback errorCallback_;

    mutable std::mutex commandMutex_;
    std::condition_variable commandCv_;
    std::deque<ControlCommand> commandQueue_;

    std::atomic<bool> running_{false};
    std::thread controlThread_;
};
