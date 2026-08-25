// Linear-stage homing (adapter): cruise to a standoff, then creep to the sensor.
// limit.approach.maxspeed is not writable over direct serial; firmware home follows maxspeed.

#include "adapters/zaber/ZaberStageHoming.hpp"

#include "adapters/zaber/ZaberStageMotion.hpp"
#include "adapters/zaber/ZaberStageProfile.hpp"

#ifdef HF_HAVE_ZML
#include <exception>

#include <zaber/motion/ascii.h>
#include <zaber/motion/ascii/setting_constants.h>
#endif

#ifdef HF_HAVE_ZML
namespace
{
using zaber::motion::Units;
using zaber::motion::ascii::setting_constants::MAXSPEED;

bool applyAxisMaxSpeedMmPerSec(zaber::motion::ascii::Device &device, const double speedMmPerSec)
{
    try
    {
        for (int axisNumber = 1; axisNumber <= zaber_stage::kAxisCount; ++axisNumber)
        {
            device.getAxis(axisNumber)
                .getSettings()
                .set(MAXSPEED, speedMmPerSec, Units::VELOCITY_MILLIMETRES_PER_SECOND);
        }
        return true;
    }
    catch (const std::exception &)
    {
        return false;
    }
}

struct HomingMaxSpeedRestorer
{
    zaber::motion::ascii::Device *device = nullptr;
    bool restore = false;

    ~HomingMaxSpeedRestorer()
    {
        if (restore && device != nullptr)
            (void)applyAxisMaxSpeedMmPerSec(*device, zaber_stage::kMaxSpeedMmPerSec);
    }
};

bool primaryAxisIsReferenced(zaber::motion::ascii::Device &device)
{
    try
    {
        return device.getAxis(zaber_stage::kPrimaryAxisNumber).isHomed();
    }
    catch (const std::exception &)
    {
        return false;
    }
}

void cruiseToApproachStandoff(zaber::motion::ascii::Lockstep &lockstep)
{
    double positionMm = 0.0;
    StageError positionError;
    if (!ZaberStageMotion::getPrimaryPositionMm(lockstep, positionMm, positionError))
        return;

    constexpr double kMinCruiseMm = 5.0;
    if (positionMm <= zaber_stage::kHomingApproachDistanceMm + kMinCruiseMm)
        return;

    StageError moveError;
    (void)ZaberStageMotion::moveAbsoluteMm(lockstep,
                                           zaber_stage::kHomingApproachDistanceMm,
                                           moveError,
                                           true,
                                           zaber_stage::kMaxSpeedMmPerSec);
}
} // namespace

bool ZaberStageHoming::performLocalizationHoming(zaber::motion::ascii::Device &device, StageError &error)
{
    return performSimpleHoming(device, error);
}

bool ZaberStageHoming::performSimpleHoming(zaber::motion::ascii::Device &device, StageError &error)
{
    try
    {
        zaber::motion::ascii::Lockstep lockstep = ZaberStageMotion::requireEnabledLockstep(device, error);
        ZaberStageMotion::enableStageDrivers(device);

        if (primaryAxisIsReferenced(device))
            cruiseToApproachStandoff(lockstep);

        HomingMaxSpeedRestorer restorer;
        restorer.device = &device;
        if (applyAxisMaxSpeedMmPerSec(device, zaber_stage::kHomingApproachSpeedMmPerSec))
            restorer.restore = true;

        return ZaberStageMotion::homeLockstep(lockstep, error);
    }
    catch (const std::exception &)
    {
        return false;
    }
}
#endif
