#pragma once

#include "adapters/zaber/ZaberStageProfile.hpp"
#include "backend/stage/StageTypes.hpp"

#ifdef HF_HAVE_ZML
namespace zaber::motion::ascii
{
class Device;
class Lockstep;
} // namespace zaber::motion::ascii
#endif

// Post-connect stage motion for the LC40B lockstep pair.
// All moves go through lockstep group 1; distances/positions refer to the primary axis only.
class ZaberStageMotion
{
public:
#ifdef HF_HAVE_ZML
    static zaber::motion::ascii::Lockstep requireEnabledLockstep(zaber::motion::ascii::Device &device,
                                                                 StageError &error);

    static void enableStageDrivers(zaber::motion::ascii::Device &device);

    static bool moveRelativeMm(zaber::motion::ascii::Lockstep &lockstep,
                               double distanceMm,
                               StageError &error,
                               bool waitUntilIdle = true,
                               double speedMmPerSec = zaber_stage::kMaxSpeedMmPerSec,
                               double accelerationMmPerSec2 = zaber_stage::kDefaultMotionAccelerationMmPerSec2);

    static bool moveAbsoluteMm(zaber::motion::ascii::Lockstep &lockstep,
                               double positionMm,
                               StageError &error,
                               bool waitUntilIdle = true,
                               double speedMmPerSec = zaber_stage::kMaxSpeedMmPerSec,
                               double accelerationMmPerSec2 = zaber_stage::kDefaultMotionAccelerationMmPerSec2);

    static bool homeLockstep(zaber::motion::ascii::Lockstep &lockstep, StageError &error);

    static bool stopLockstep(zaber::motion::ascii::Lockstep &lockstep,
                             StageError &error,
                             bool waitUntilIdle = true);

    static bool moveVelocityMm(zaber::motion::ascii::Lockstep &lockstep,
                               double velocityMmPerSec,
                               StageError &error,
                               double accelerationMmPerSec2 = zaber_stage::kDefaultMotionAccelerationMmPerSec2);

    static bool getPrimaryPositionMm(zaber::motion::ascii::Lockstep &lockstep,
                                     double &positionMm,
                                     StageError &error);
#endif
};
