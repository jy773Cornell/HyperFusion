// Lumo SDK device discovery types (FX10e / SWIR3 / SpecSensor).
#pragma once

#include <string>
#include <vector>

/// Which Lumo SSP / grabber stack this camera instance uses.
enum class LumoSensorKind
{
    Fx10ePleora,
    Swir3Ni
};

struct LumoDeviceEntry
{
    int index = 0;
    std::string name;
    std::string description;
};
