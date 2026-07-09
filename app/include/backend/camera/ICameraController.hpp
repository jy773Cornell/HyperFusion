// Camera backend interface contract for lifecycle and frame acquisition.
#pragma once

#include "adapters/lumo/LumoDeviceTypes.hpp"
#include "backend/camera/CameraTypes.hpp"

#include <string>

class ICameraController
{
public:
    virtual ~ICameraController() = default;

    virtual std::string name() const = 0;
    virtual CameraBackendId backendId() const = 0;
    virtual LumoSensorKind sensorKind() const { return LumoSensorKind::Fx10ePleora; }

    /// Pleora (FX10e) and SWIR3 SDK lifecycle must not run on the camera control thread.
    virtual bool requiresGuiThreadForSdkLifecycle() const { return true; }

    virtual bool connect(CameraError &error) = 0;
    virtual bool initialize(CameraError &error) = 0;
    virtual bool applySettings(const CameraSettings &settings,
                               CameraError &error,
                               CameraTimingApplyResult *timingOut = nullptr) = 0;
    virtual bool openShutter(CameraError &error);
    virtual bool closeShutter(CameraError &error);
    virtual bool shutterIsOpen(bool &isOpen, CameraError &error);
    virtual bool arm(CameraError &error) = 0;
    virtual bool start(CameraError &error) = 0;
    virtual void stop() = 0;
    virtual void disconnect() = 0;

    virtual CameraState state() const = 0;
    virtual bool pollFrame(FramePacket &frame, std::uint32_t timeoutMs, CameraError &error) = 0;
};
