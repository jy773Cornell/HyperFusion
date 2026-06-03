#include "adapters/zaber/ZaberStageHoming.hpp"

#include "adapters/zaber/ZaberStageMotion.hpp"
#include "adapters/zaber/ZaberStageProfile.hpp"

#ifdef HF_HAVE_ZML
#include <zaber/motion/ascii.h>
#endif

#ifdef HF_HAVE_ZML
namespace
{
bool tryLocalizationPremove(zaber::motion::ascii::Device &device,
                            zaber::motion::ascii::Lockstep &lockstep,
                            StageError &error)
{
    zaber::motion::ascii::Axis primary = device.getAxis(zaber_stage::kPrimaryAxisNumber);

    // Before a reference exists (WR flag), Zaber rejects most relative moves with BADDATA.
    if (!primary.isHomed())
        return true;

    StageError premoveError;
    if (ZaberStageMotion::moveRelativeMm(lockstep,
                                         zaber_stage::kHomingLocalizationPremoveMm,
                                         premoveError,
                                         true,
                                         zaber_stage::kHomingSpeedMmPerSec))
        return true;

    if (premoveError.message.find("BADDATA") != std::string::npos)
        return true;

    error = premoveError;
    return false;
}
} // namespace

bool ZaberStageHoming::performLocalizationHoming(zaber::motion::ascii::Device &device, StageError &error)
{
    try
    {
        zaber::motion::ascii::Lockstep lockstep = ZaberStageMotion::requireEnabledLockstep(device, error);
        ZaberStageMotion::enableStageDrivers(device);

        if (!tryLocalizationPremove(device, lockstep, error))
            return false;

        return ZaberStageMotion::homeLockstep(lockstep, error);
    }
    catch (const std::exception &)
    {
        return false;
    }
}

bool ZaberStageHoming::performSimpleHoming(zaber::motion::ascii::Device &device, StageError &error)
{
    try
    {
        zaber::motion::ascii::Lockstep lockstep = ZaberStageMotion::requireEnabledLockstep(device, error);
        ZaberStageMotion::enableStageDrivers(device);
        return ZaberStageMotion::homeLockstep(lockstep, error);
    }
    catch (const std::exception &)
    {
        return false;
    }
}
#endif
