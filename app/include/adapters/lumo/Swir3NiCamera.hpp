// SWIR3 hyperspectral camera via Lumo Sensor SDK + National Instruments frame grabber (SSP).
#pragma once

#include "adapters/lumo/LumoCamera.hpp"

class Swir3NiCamera final : public LumoCamera
{
public:
    Swir3NiCamera(CameraBackendId backendId, std::string instanceLabel);
};
