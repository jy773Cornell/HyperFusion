// Default ICameraController implementations for optional APIs (e.g. shutter control).
// Backends that do not support a feature inherit these safe NotImplemented responses.
#include "core/ICameraController.hpp"

bool ICameraController::openShutter(CameraError &error)
{
    error.code = CameraErrorCode::NotImplemented;
    error.message = "Shutter open not supported for this camera backend.";
    error.fatal = false;
    return false;
}

bool ICameraController::closeShutter(CameraError &error)
{
    error.code = CameraErrorCode::NotImplemented;
    error.message = "Shutter close not supported for this camera backend.";
    error.fatal = false;
    return false;
}

bool ICameraController::shutterIsOpen(bool &isOpen, CameraError &error)
{
    isOpen = false;
    error.code = CameraErrorCode::NotImplemented;
    error.message = "Shutter status not supported for this camera backend.";
    error.fatal = false;
    return false;
}
