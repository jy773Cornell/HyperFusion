// Multi-camera coordinator: routes UI commands to per-camera workers.
#pragma once

#include "backend/CameraWorker.hpp"

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
    void beginStreaming(std::size_t cameraIndex, const CameraSettings &settings);
    void arm(std::size_t cameraIndex);
    void startStream(std::size_t cameraIndex);
    void stopStream(std::size_t cameraIndex);
    void disconnect(std::size_t cameraIndex);
    void disconnectOnGuiThread(std::size_t cameraIndex);
    void openShutter(std::size_t cameraIndex);
    void closeShutter(std::size_t cameraIndex);
    void refreshShutterState(std::size_t cameraIndex);
    void shutdownSync();

    void setLogCallback(std::function<void(const std::string &)> callback);
    void setFrameCallback(std::function<void(const FramePacket &)> callback);
    void setCameraStateCallback(std::size_t cameraIndex,
                                std::function<void(CameraState)> callback);
    void setCameraErrorCallback(std::size_t cameraIndex,
                                std::function<void(const CameraError &)> callback);
    void setCameraShutterStateCallback(std::size_t cameraIndex,
                                       std::function<void(bool isOpen)> callback);
    void setCameraSettingsAppliedCallback(
        std::size_t cameraIndex,
        std::function<void(const CameraSettingsApplyReport &)> callback);
    void setGuiTaskRunner(CameraWorker::GuiTaskRunner runner);
    void setGuiAsyncTaskRunner(CameraWorker::GuiAsyncTaskRunner runner);

private:
    CameraWorker &workerAt(std::size_t cameraIndex);
    void wireWorkerCallbacks(std::size_t cameraIndex, CameraWorker &worker);
    void log(const std::string &message);

    std::vector<std::unique_ptr<CameraWorker>> workers_;
    std::function<void(const std::string &)> logCallback_;
    std::function<void(const FramePacket &)> frameCallback_;
    std::vector<std::function<void(CameraState)>> stateCallbacks_;
    std::vector<std::function<void(const CameraError &)>> errorCallbacks_;
    std::vector<std::function<void(bool)>> shutterStateCallbacks_;
    std::vector<std::function<void(const CameraSettingsApplyReport &)>> settingsAppliedCallbacks_;
};
