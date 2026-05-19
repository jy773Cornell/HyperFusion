// Camera backend interface contract for lifecycle and frame acquisition.
#pragma once

#include "core/CameraTypes.hpp"

#include <string>

class ICameraController
{
public:
    virtual ~ICameraController() = default;

    virtual std::string name() const = 0;
    virtual CameraBackendId backendId() const = 0;

    virtual bool connect(CameraError &error) = 0;
    virtual bool initialize(CameraError &error) = 0;
    virtual bool applySettings(const CameraSettings &settings, CameraError &error) = 0;
    virtual bool arm(CameraError &error) = 0;
    virtual bool start(CameraError &error) = 0;
    virtual void stop() = 0;
    virtual void disconnect() = 0;

    virtual CameraState state() const = 0;
    virtual bool pollFrame(FramePacket &frame, std::uint32_t timeoutMs, CameraError &error) = 0;
};
