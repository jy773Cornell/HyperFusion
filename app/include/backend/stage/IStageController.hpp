#pragma once

#include "backend/stage/StageTypes.hpp"

class IStageController
{
public:
    virtual ~IStageController() = default;

    virtual std::string name() const = 0;
    virtual StageState state() const = 0;
    virtual StageTopology topology() const = 0;

    virtual bool connect(const StageConnectSettings &settings, StageError &error) = 0;
    virtual bool homeWithLocalization(StageError &error) = 0;
    virtual bool home(StageError &error) = 0;
    virtual void stopMotion(bool waitUntilIdle = true) = 0;
    virtual void disconnect() = 0;

    // All motion after connect uses lockstep group 1; distances refer to the primary axis only.
    virtual bool moveRelativeMm(double distanceMm, double speedMmPerSec, StageError &error) = 0;
    virtual bool moveAbsoluteMm(double positionMm,
                                double speedMmPerSec,
                                bool waitUntilIdle,
                                StageError &error) = 0;
    virtual bool moveVelocityMm(double velocityMmPerSec, StageError &error) = 0;
    virtual bool getPrimaryPositionMm(double &positionMm, StageError &error) = 0;
};
