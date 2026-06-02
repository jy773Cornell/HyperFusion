// Camera session orchestration for the UI (coordinator wiring, connect/stream policy).
#pragma once

#include <QObject>

#include <cstddef>
#include <memory>
#include <string>

#include "core/CameraTypes.hpp"

class LumoCamera;
class CameraCoordinator;
struct LumoCameraUi;

class CameraAppController : public QObject
{
    Q_OBJECT

public:
    explicit CameraAppController(QObject *parent = nullptr);
    ~CameraAppController() override;

    void bindCameras(LumoCameraUi &camera1, LumoCameraUi &camera2);
    void start();
    void shutdown();

    static QString defaultFx10eCalibrationPackPath();
    static CameraSettings settingsFromUi(const LumoCameraUi &ui);

    void refreshDeviceProfiles();
    void connectOrDisconnect(LumoCameraUi &ui, const QString &panelTitle);
    void applySettings(LumoCameraUi &ui, const QString &panelTitle);

signals:
    void logMessage(const QString &message);
    void detectorFrameReady(std::size_t cameraIndex, const QImage &image);
    void cameraStateUpdated(LumoCameraUi &ui, CameraState state);
    void connectionFailed(std::size_t cameraIndex, const QString &title, const QString &message);

private:
    void onCameraStateChanged(LumoCameraUi &ui, CameraState state);
    void onCameraError(LumoCameraUi &ui, const CameraError &error);
    void onFrame(const FramePacket &frame);

    struct CameraSession
    {
        bool autoStreamStarted = false;
        bool connectAttemptActive = false;
    };

    LumoCameraUi *cameraUi(std::size_t cameraIndex);
    CameraSession &sessionFor(std::size_t cameraIndex);

    std::unique_ptr<CameraCoordinator> coordinator_;
    LumoCameraUi *camera1_ = nullptr;
    LumoCameraUi *camera2_ = nullptr;
    CameraSession session1_;
    CameraSession session2_;
};
