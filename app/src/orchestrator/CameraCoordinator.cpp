// Multi-camera coordinator implementation.
#include "orchestrator/CameraCoordinator.hpp"

#include <sstream>
#include <stdexcept>

namespace
{
std::string toStateString(const CameraState state)
{
    switch (state)
    {
    case CameraState::Disconnected:
        return "Disconnected";
    case CameraState::Connected:
        return "Connected";
    case CameraState::Initialized:
        return "Initialized";
    case CameraState::Configured:
        return "Configured";
    case CameraState::Armed:
        return "Armed";
    case CameraState::Streaming:
        return "Streaming";
    case CameraState::SafeStopped:
        return "SafeStopped";
    case CameraState::Fault:
        return "Fault";
    default:
        return "Unknown";
    }
}
} // namespace

CameraCoordinator::CameraCoordinator(std::vector<std::shared_ptr<ICameraController>> cameras)
{
    stateCallbacks_.resize(cameras.size());
    errorCallbacks_.resize(cameras.size());
    shutterStateCallbacks_.resize(cameras.size());

    for (std::size_t index = 0; index < cameras.size(); ++index)
    {
        auto worker = std::make_unique<CameraWorker>(std::move(cameras[index]));
        wireWorkerCallbacks(index, *worker);
        workers_.push_back(std::move(worker));
    }
}

CameraCoordinator::~CameraCoordinator()
{
    stop();
}

std::size_t CameraCoordinator::cameraCount() const
{
    return workers_.size();
}

CameraWorker &CameraCoordinator::workerAt(const std::size_t cameraIndex)
{
    if (cameraIndex >= workers_.size())
        throw std::out_of_range("CameraCoordinator: camera index out of range");

    return *workers_.at(cameraIndex);
}

void CameraCoordinator::start()
{
    for (auto &worker : workers_)
        worker->start();

    log("Camera coordinator started (" + std::to_string(workers_.size()) + " camera(s)).");
}

void CameraCoordinator::stop()
{
    for (auto &worker : workers_)
        worker->stop();
}

void CameraCoordinator::connect(const std::size_t cameraIndex)
{
    workerAt(cameraIndex).requestConnect();
}

void CameraCoordinator::initializeOnGuiThread(const std::size_t cameraIndex)
{
    workerAt(cameraIndex).requestInitializeOnGuiThread();
}

void CameraCoordinator::connectAndInitializeOnGuiThread(const std::size_t cameraIndex)
{
    workerAt(cameraIndex).requestConnectAndInitializeOnGuiThread();
}

void CameraCoordinator::applySettings(const std::size_t cameraIndex, const CameraSettings &settings)
{
    workerAt(cameraIndex).requestApplySettings(settings);
}

void CameraCoordinator::beginStreaming(const std::size_t cameraIndex, const CameraSettings &settings)
{
    workerAt(cameraIndex).requestBeginStreaming(settings);
}

void CameraCoordinator::arm(const std::size_t cameraIndex)
{
    workerAt(cameraIndex).requestArm();
}

void CameraCoordinator::startStream(const std::size_t cameraIndex)
{
    workerAt(cameraIndex).requestStartStreaming();
}

void CameraCoordinator::stopStream(const std::size_t cameraIndex)
{
    workerAt(cameraIndex).requestStopStreaming();
}

void CameraCoordinator::disconnect(const std::size_t cameraIndex)
{
    workerAt(cameraIndex).requestDisconnect();
}

void CameraCoordinator::disconnectOnGuiThread(const std::size_t cameraIndex)
{
    workerAt(cameraIndex).requestDisconnectOnGuiThread();
}

void CameraCoordinator::openShutter(const std::size_t cameraIndex)
{
    workerAt(cameraIndex).requestOpenShutter();
}

void CameraCoordinator::closeShutter(const std::size_t cameraIndex)
{
    workerAt(cameraIndex).requestCloseShutter();
}

void CameraCoordinator::refreshShutterState(const std::size_t cameraIndex)
{
    workerAt(cameraIndex).requestQueryShutterState();
}

void CameraCoordinator::shutdownSync()
{
    for (auto &worker : workers_)
    {
        if (worker)
            worker->shutdownSync();
    }
}

void CameraCoordinator::setLogCallback(std::function<void(const std::string &)> callback)
{
    logCallback_ = std::move(callback);
}

void CameraCoordinator::setFrameCallback(std::function<void(const FramePacket &)> callback)
{
    frameCallback_ = std::move(callback);
}

void CameraCoordinator::setCameraStateCallback(const std::size_t cameraIndex,
                                                 std::function<void(CameraState)> callback)
{
    if (cameraIndex >= stateCallbacks_.size())
        throw std::out_of_range("CameraCoordinator: camera index out of range");

    stateCallbacks_[cameraIndex] = std::move(callback);
}

void CameraCoordinator::setCameraErrorCallback(const std::size_t cameraIndex,
                                                   std::function<void(const CameraError &)> callback)
{
    if (cameraIndex >= errorCallbacks_.size())
        throw std::out_of_range("CameraCoordinator: camera index out of range");

    errorCallbacks_[cameraIndex] = std::move(callback);
}

void CameraCoordinator::setCameraShutterStateCallback(const std::size_t cameraIndex,
                                                        std::function<void(bool)> callback)
{
    if (cameraIndex >= shutterStateCallbacks_.size())
        throw std::out_of_range("CameraCoordinator: camera index out of range");

    shutterStateCallbacks_[cameraIndex] = std::move(callback);
}

void CameraCoordinator::setGuiTaskRunner(CameraWorker::GuiTaskRunner runner)
{
    for (auto &worker : workers_)
        worker->setGuiTaskRunner(runner);
}

void CameraCoordinator::setGuiAsyncTaskRunner(CameraWorker::GuiAsyncTaskRunner runner)
{
    for (auto &worker : workers_)
        worker->setGuiAsyncTaskRunner(runner);
}

void CameraCoordinator::wireWorkerCallbacks(const std::size_t cameraIndex, CameraWorker &worker)
{
    worker.setStateCallback([this, cameraIndex, &worker](const CameraState state) {
        std::ostringstream oss;
        oss << worker.name() << " state -> " << toStateString(state);
        if (state == CameraState::Connected)
            oss << " (SSP profile open; camera not initialized yet)";
        log(oss.str());

        if (cameraIndex < stateCallbacks_.size() && stateCallbacks_[cameraIndex])
            stateCallbacks_[cameraIndex](state);
    });

    worker.setErrorCallback([this, cameraIndex, &worker](const CameraError &error) {
        std::ostringstream oss;
        oss << worker.name() << " error [" << static_cast<int>(error.code) << "]: " << error.message;
        log(oss.str());

        if (cameraIndex < errorCallbacks_.size() && errorCallbacks_[cameraIndex])
            errorCallbacks_[cameraIndex](error);
    });

    worker.setFrameCallback([this](const FramePacket &frame) {
        if (frameCallback_)
            frameCallback_(frame);
    });

    worker.setShutterStateCallback([this, cameraIndex](const bool isOpen) {
        if (cameraIndex < shutterStateCallbacks_.size() && shutterStateCallbacks_[cameraIndex])
            shutterStateCallbacks_[cameraIndex](isOpen);
    });
}

void CameraCoordinator::log(const std::string &message)
{
    if (logCallback_)
        logCallback_(message);
}
