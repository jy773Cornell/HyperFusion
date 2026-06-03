#pragma once

#include "core/StageTypes.hpp"

class IStageController
{
public:
    virtual ~IStageController() = default;

    virtual std::string name() const = 0;
    virtual StageState state() const = 0;
    virtual StageTopology topology() const = 0;

    virtual bool connect(const StageConnectSettings &settings, StageError &error) = 0;
    virtual void disconnect() = 0;
};
