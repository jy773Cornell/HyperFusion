#include "adapters/zaber/ZaberStageHoming.hpp"

#include "adapters/zaber/ZaberStageMotion.hpp"

#ifdef HF_HAVE_ZML
#include <zaber/motion/ascii.h>
#endif

#ifdef HF_HAVE_ZML
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
        return ZaberStageMotion::homeLockstep(lockstep, error);
    }
    catch (const std::exception &)
    {
        return false;
    }
}
#endif
