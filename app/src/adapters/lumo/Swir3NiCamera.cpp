// SWIR3 + NI Lumo adapter (delegates to LumoCamera with Swir3Ni sensor kind).
#include "adapters/lumo/Swir3NiCamera.hpp"

Swir3NiCamera::Swir3NiCamera(const CameraBackendId backendId, std::string instanceLabel)
    : LumoCamera(backendId, std::move(instanceLabel), LumoSensorKind::Swir3Ni)
{
}
