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
    /// GUI-thread HDMI paint. Called from the DLP control thread.
    using HdmiShowCallback = std::function<bool(int stepIndex, DlpError &)>;

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

    /// Stop the GUI FPP loop, show one burst step, return when the EVM has it.
    /// Call from a non-control thread (scan execute). Timeout 20 s.
    [[nodiscard]] bool showFppStepSync(int stepIndex, DlpError &error);
    /// Apply LED currents and wait for the EVM before the next captured frame.
    [[nodiscard]] bool applyLedCurrentsSync(const DlpProjectorSettings &settings,
                                            DlpError &error);
    /// Stop the GUI FPP loop and blank. Call from a non-control thread.
    [[nodiscard]] bool blankSync(DlpError &error);

    [[nodiscard]] DlpProjectorState currentState() const;

    void setStateCallback(StateCallback callback);
    void setErrorCallback(ErrorCallback callback);
    void setDevicesCallback(DevicesCallback callback);
    void setLogCallback(LogCallback callback);
    void setHdmiShowCallback(HdmiShowCallback callback);

private:
    using ControlCommand = std::function<void()>;

    void enqueue(ControlCommand command);
    void stopFppScan();
    bool applyFppStep(int stepIndex, DlpError &error);
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
    HdmiShowCallback hdmiShowCallback_;

    mutable std::mutex commandMutex_;
    std::condition_variable commandCv_;
    std::deque<ControlCommand> commandQueue_;

    std::atomic<bool> running_{false};
    std::atomic<bool> fppStop_{true};
    std::atomic<DlpProjectorState> state_{DlpProjectorState::Disconnected};

    std::thread controlThread_;
};
} // namespace hf::dlp
