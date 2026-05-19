// Multi-camera orchestrator: routes UI commands to per-camera workers.
#pragma once

#include "core/CameraWorker.hpp"

#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <vector>

class CameraCoordinator
{
public:
    explicit CameraCoordinator(std::vector<std::shared_ptr<ICameraController>> cameras);
    ~CameraCoordinator();

    std::size_t cameraCount() const;

    void start();
    void stop();

    void connect(std::size_t cameraIndex);
    void initializeOnGuiThread(std::size_t cameraIndex);
    void connectAndInitializeOnGuiThread(std::size_t cameraIndex);
    void applySettings(std::size_t cameraIndex, const CameraSettings &settings);
    void arm(std::size_t cameraIndex);
    void startStream(std::size_t cameraIndex);
    void stopStream(std::size_t cameraIndex);
    void disconnect(std::size_t cameraIndex);

    void setLogCallback(std::function<void(const std::string &)> callback);
    void setFrameCallback(std::function<void(const FramePacket &)> callback);
    void setCameraStateCallback(std::size_t cameraIndex,
                                std::function<void(CameraState)> callback);
    void setGuiTaskRunner(CameraWorker::GuiTaskRunner runner);

private:
    CameraWorker &workerAt(std::size_t cameraIndex);
    void wireWorkerCallbacks(std::size_t cameraIndex, CameraWorker &worker);
    void log(const std::string &message);

    std::vector<std::unique_ptr<CameraWorker>> workers_;
    std::function<void(const std::string &)> logCallback_;
    std::function<void(const FramePacket &)> frameCallback_;
    std::vector<std::function<void(CameraState)>> stateCallbacks_;
};
