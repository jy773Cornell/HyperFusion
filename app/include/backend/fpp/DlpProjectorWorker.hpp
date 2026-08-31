// Threaded DLP projector worker: control queue only (backend/fpp). No light on start.
#pragma once

#include "adapters/dlp/Dlpc3478Projector.hpp"
#include "backend/fpp/DlpTypes.hpp"

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

namespace hf::dlp
{
class DlpProjectorWorker
{
public:
    using StateCallback = std::function<void(DlpProjectorState)>;
    using ErrorCallback = std::function<void(const DlpError &)>;
    using DevicesCallback = std::function<void(const std::vector<DlpDeviceInfo> &)>;
    using LogCallback = std::function<void(const std::string &)>;

    DlpProjectorWorker();
    ~DlpProjectorWorker();

    void start();
    void shutdownSync();

    void requestEnumerate();
    void requestConnect(const DlpProjectorSettings &settings);
    void requestDisconnect();
    void requestArm(const DlpProjectorSettings &settings);
    void requestBlank();
    void requestShowTestPattern(const DlpProjectorSettings &settings);
    void requestApplyLedCurrents(const DlpProjectorSettings &settings);

    [[nodiscard]] DlpProjectorState currentState() const;

    void setStateCallback(StateCallback callback);
    void setErrorCallback(ErrorCallback callback);
    void setDevicesCallback(DevicesCallback callback);
    void setLogCallback(LogCallback callback);

private:
    using ControlCommand = std::function<void()>;

    void enqueue(ControlCommand command);
    void stopFppScan();
    void runFppScanningLoop();
    void controlLoop();
    void notifyState(DlpProjectorState state);
    void notifyError(const DlpError &error);
    void notifyLog(const std::string &message);

    std::unique_ptr<Dlpc3478Projector> projector_;

    mutable std::mutex callbackMutex_;
    StateCallback stateCallback_;
    ErrorCallback errorCallback_;
    DevicesCallback devicesCallback_;
    LogCallback logCallback_;

    mutable std::mutex commandMutex_;
    std::condition_variable commandCv_;
    std::deque<ControlCommand> commandQueue_;

    std::atomic<bool> running_{false};
    std::atomic<bool> fppStop_{true};
    std::atomic<DlpProjectorState> state_{DlpProjectorState::Disconnected};

    std::thread controlThread_;
};
} // namespace hf::dlp
