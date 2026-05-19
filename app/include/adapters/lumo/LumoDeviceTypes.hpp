// Lumo SDK device discovery types (FX10e / SpecSensor).
#pragma once

#include <string>
#include <vector>

struct LumoDeviceEntry
{
    int index = 0;
    std::string name;
    std::string description;
};
