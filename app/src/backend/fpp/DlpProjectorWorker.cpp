// Threaded DLP projector worker (backend/fpp). Blank + disconnect on shutdown.
#include "backend/fpp/DlpProjectorWorker.hpp"

#include <QString>

#include <chrono>
#include <utility>

namespace hf::dlp
{
DlpProjectorWorker::DlpProjectorWorker()
    : projector_(std::make_unique<Dlpc3478Projector>())
{
}

DlpProjectorWorker::~DlpProjectorWorker()
{
    shutdownSync();
}

void DlpProjectorWorker::start()
{
    if (running_.exchange(true))
        return;
    controlThread_ = std::thread([this]() { controlLoop(); });
}

void DlpProjectorWorker::shutdownSync()
{
    stopFppScan();
    if (running_.load())
    {
        std::mutex doneMutex;
        std::condition_variable doneCv;
        bool done = false;

        enqueue([this, &doneMutex, &doneCv, &done]() {
            DlpError error;
            if (projector_)
            {
                (void)projector_->blank(error);
                projector_->disconnect();
            }
            state_ = DlpProjectorState::Disconnected;
            {
                std::lock_guard<std::mutex> lock(doneMutex);
                done = true;
            }
            doneCv.notify_one();
        });

        {
            std::unique_lock<std::mutex> lock(doneMutex);
            doneCv.wait_for(lock, std::chrono::seconds(4), [&done]() { return done; });
        }

        running_ = false;
        commandCv_.notify_all();
        if (controlThread_.joinable())
            controlThread_.join();
    }

    if (projector_)
    {
        DlpError error;
        (void)projector_->blank(error);
        projector_->disconnect();
    }
    state_ = DlpProjectorState::Disconnected;
}

void DlpProjectorWorker::stopFppScan()
{
    fppStop_ = true;
    commandCv_.notify_all();
}

bool DlpProjectorWorker::applyFppStep(int stepIndex, DlpError &error)
{
    if (projector_ == nullptr)
    {
        error = {DlpErrorCode::InternalError, "DLP projector missing.", true};
        return false;
    }
    if (stepIndex < 0 || stepIndex >= kFppScanningStepCount)
    {
        error = {DlpErrorCode::InvalidState, "FPP step index out of range.", false};
        return false;
    }

    const FppScanStep &step = kFppScanningSteps[stepIndex];
    if (step.kind == FppScanStepKind::AmbientBlank)
    {
        if (!projector_->blank(error))
            return false;
    }
    else if (!projector_->showExternalVideo(error))
        return false;

    HdmiShowCallback hdmi;
    {
        std::lock_guard<std::mutex> lock(callbackMutex_);
        hdmi = hdmiShowCallback_;
    }
    if (!hdmi)
    {
        error = {DlpErrorCode::InternalError, "HDMI pattern window is not attached.", false};
        return false;
    }
    return hdmi(stepIndex, error);
}

bool DlpProjectorWorker::showFppStepSync(int stepIndex, DlpError &error)
{
    stopFppScan();
    if (!running_.load() || projector_ == nullptr)
    {
        error = {DlpErrorCode::InvalidState, "DLP worker is not running.", false};
        return false;
    }

    std::mutex doneMutex;
    std::condition_variable doneCv;
    bool done = false;
    bool ok = false;
    DlpError localError;

    enqueue([this, stepIndex, &doneMutex, &doneCv, &done, &ok, &localError]() {
        ok = applyFppStep(stepIndex, localError);
        if (!ok)
        {
            notifyState(DlpProjectorState::Fault);
            notifyError(localError);
        }
        else
        {
            const FppScanStep &step = kFppScanningSteps[stepIndex];
            notifyState(step.kind == FppScanStepKind::AmbientBlank ? DlpProjectorState::Armed
                                                                  : DlpProjectorState::Projecting);
            notifyLog(std::string("DLP: FPP pin: ") + step.label);
        }
        {
            std::lock_guard<std::mutex> lock(doneMutex);
            done = true;
        }
        doneCv.notify_one();
    });

    {
        std::unique_lock<std::mutex> lock(doneMutex);
        if (!doneCv.wait_for(lock, std::chrono::seconds(20), [&done]() { return done; }))
        {
            error = {DlpErrorCode::InternalError, "Timed out showing FPP step.", false};
            return false;
        }
    }
    error = localError;
    return ok;
}

bool DlpProjectorWorker::blankSync(DlpError &error)
{
    stopFppScan();
    if (!running_.load() || projector_ == nullptr)
    {
        error = {DlpErrorCode::InvalidState, "DLP worker is not running.", false};
        return false;
    }

    std::mutex doneMutex;
    std::condition_variable doneCv;
    bool done = false;
    bool ok = false;
    DlpError localError;

    enqueue([this, &doneMutex, &doneCv, &done, &ok, &localError]() {
        ok = projector_->blank(localError);
        if (!ok)
        {
            notifyState(DlpProjectorState::Fault);
            notifyError(localError);
        }
        else
        {
            notifyLog("DLP: blanked.");
            notifyState(DlpProjectorState::Armed);
        }
        {
            std::lock_guard<std::mutex> lock(doneMutex);
            done = true;
        }
        doneCv.notify_one();
    });

    {
        std::unique_lock<std::mutex> lock(doneMutex);
        if (!doneCv.wait_for(lock, std::chrono::seconds(8), [&done]() { return done; }))
        {
            error = {DlpErrorCode::InternalError, "Timed out blanking DLP.", false};
            return false;
        }
    }
    error = localError;
    return ok;
}

void DlpProjectorWorker::runFppScanningLoop()
{
    fppStop_ = false;
    notifyLog("DLP: FPP HDMI sine loop (GUI test). Blank or Disconnect to stop.");
    notifyState(DlpProjectorState::Projecting);

    int cycle = 0;
    while (running_.load() && !fppStop_.load())
    {
        ++cycle;
        for (int i = 0; i < kFppScanningStepCount; ++i)
        {
            if (!running_.load() || fppStop_.load())
                break;

            DlpError error;
            if (!applyFppStep(i, error))
            {
                notifyState(DlpProjectorState::Fault);
                notifyError(error);
                fppStop_ = true;
                return;
            }

            if (cycle == 1)
                notifyLog(std::string("DLP: FPP scanning: ") + kFppScanningSteps[i].label);

            std::unique_lock<std::mutex> lock(commandMutex_);
            commandCv_.wait_for(lock, std::chrono::milliseconds(kFppScanningDwellMs), [this]() {
                return fppStop_.load() || !running_.load();
            });
        }
    }

    fppStop_ = true;
}

void DlpProjectorWorker::enqueue(ControlCommand command)
{
    {
        std::lock_guard<std::mutex> lock(commandMutex_);
        commandQueue_.push_back(std::move(command));
    }
    commandCv_.notify_one();
}

void DlpProjectorWorker::requestEnumerate()
{
    enqueue([this]() {
        DlpError error;
        const std::vector<DlpDeviceInfo> devices = projector_->listDevices(&error);
        if (error.code != DlpErrorCode::None)
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

void DlpProjectorWorker::requestConnect(const DlpProjectorSettings &settings)
{
    stopFppScan();
    enqueue([this, settings]() {
        projector_->disconnect();
        DlpError error;
        if (!projector_->connect(settings.deviceId, error))
        {
            notifyState(DlpProjectorState::Fault);
            notifyError(error);
            return;
        }
        if (!projector_->applyLedCurrents(settings, error))
        {
            projector_->disconnect();
            notifyState(DlpProjectorState::Fault);
            notifyError(error);
            return;
        }
        if (!projector_->arm(settings, error))
        {
            projector_->disconnect();
            notifyState(DlpProjectorState::Fault);
            notifyError(error);
            return;
        }
        notifyLog(Dlpc3478Projector::hardwareBackendAvailable()
                      ? (std::string("DLP: connected ") + projector_->connectedId() + " and armed.")
                      : "DLP: connected (stub — no light, DLPC-API not linked).");
        notifyState(DlpProjectorState::Armed);
    });
}

void DlpProjectorWorker::requestDisconnect()
{
    stopFppScan();
    enqueue([this]() {
        DlpError error;
        (void)projector_->blank(error);
        projector_->disconnect();
        notifyLog("DLP: disconnected.");
        notifyState(DlpProjectorState::Disconnected);
    });
}

void DlpProjectorWorker::requestArm(const DlpProjectorSettings &settings)
{
    enqueue([this, settings]() {
        DlpError error;
        if (!projector_->applyLedCurrents(settings, error))
        {
            notifyError(error);
            return;
        }
        if (!projector_->arm(settings, error))
        {
            notifyState(DlpProjectorState::Fault);
            notifyError(error);
            return;
        }
        notifyLog(Dlpc3478Projector::hardwareBackendAvailable() ? "DLP: armed."
                                                               : "DLP: armed (stub — LED output not enabled).");
        notifyState(DlpProjectorState::Armed);
    });
}

void DlpProjectorWorker::requestBlank()
{
    stopFppScan();
    enqueue([this]() {
        DlpError error;
        if (!projector_->blank(error))
        {
            notifyState(DlpProjectorState::Fault);
            notifyError(error);
            return;
        }
        notifyLog("DLP: blanked.");
        notifyState(DlpProjectorState::Armed);
    });
}

void DlpProjectorWorker::requestShowTestPattern(const DlpProjectorSettings &settings)
{
    stopFppScan();
    enqueue([this, settings]() {
        if (isFppScanningPattern(settings.testPattern))
        {
            runFppScanningLoop();
            if (fppStop_.load() && running_.load()
                && projector_->state() != DlpProjectorState::Fault
                && projector_->state() != DlpProjectorState::Disconnected)
            {
                DlpError error;
                (void)projector_->blank(error);
                notifyLog("DLP: FPP scanning stopped.");
                notifyState(DlpProjectorState::Armed);
            }
            return;
        }

        DlpError error;
        if (!projector_->showTestPattern(settings.testPattern, error))
        {
            notifyError(error);
            return;
        }
        notifyLog(std::string("DLP: test pattern: ") + settings.testPattern.toStdString());
        notifyState(DlpProjectorState::Projecting);
    });
}

void DlpProjectorWorker::requestApplyLedCurrents(const DlpProjectorSettings &settings)
{
    enqueue([this, settings]() {
        DlpError error;
        if (!projector_->applyLedCurrents(settings, error))
            notifyError(error);
    });
}

DlpProjectorState DlpProjectorWorker::currentState() const
{
    return state_.load();
}

void DlpProjectorWorker::setStateCallback(StateCallback callback)
{
    std::lock_guard<std::mutex> lock(callbackMutex_);
    stateCallback_ = std::move(callback);
}

void DlpProjectorWorker::setErrorCallback(ErrorCallback callback)
{
    std::lock_guard<std::mutex> lock(callbackMutex_);
    errorCallback_ = std::move(callback);
}

void DlpProjectorWorker::setDevicesCallback(DevicesCallback callback)
{
    std::lock_guard<std::mutex> lock(callbackMutex_);
    devicesCallback_ = std::move(callback);
}

void DlpProjectorWorker::setLogCallback(LogCallback callback)
{
    std::lock_guard<std::mutex> lock(callbackMutex_);
    logCallback_ = std::move(callback);
}

void DlpProjectorWorker::setHdmiShowCallback(HdmiShowCallback callback)
{
    std::lock_guard<std::mutex> lock(callbackMutex_);
    hdmiShowCallback_ = std::move(callback);
}

void DlpProjectorWorker::controlLoop()
{
    while (running_.load())
    {
        ControlCommand command;
        {
            std::unique_lock<std::mutex> lock(commandMutex_);
            commandCv_.wait(lock, [this]() { return !commandQueue_.empty() || !running_.load(); });
            if (!running_.load() && commandQueue_.empty())
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

void DlpProjectorWorker::notifyState(const DlpProjectorState state)
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

void DlpProjectorWorker::notifyError(const DlpError &error)
{
    ErrorCallback cb;
    {
        std::lock_guard<std::mutex> lock(callbackMutex_);
        cb = errorCallback_;
    }
    if (cb)
        cb(error);
}

void DlpProjectorWorker::notifyLog(const std::string &message)
{
    LogCallback cb;
    {
        std::lock_guard<std::mutex> lock(callbackMutex_);
        cb = logCallback_;
    }
    if (cb)
        cb(message);
}
} // namespace hf::dlp
