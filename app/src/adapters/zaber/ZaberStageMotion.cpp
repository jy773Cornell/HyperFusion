#include "adapters/zaber/ZaberStageMotion.hpp"

#include "adapters/zaber/ZaberStageProfile.hpp"

#ifdef HF_HAVE_ZML
#include <zaber/motion/ascii.h>
#include <zaber/motion/exceptions/motion_lib_exception.h>
#endif

#ifdef HF_HAVE_ZML
namespace
{
using zaber::motion::Units;
using zaber::motion::ascii::Lockstep;

Lockstep::MoveRelativeOptions makeMoveRelativeOptions(const bool waitUntilIdle, const double speedMmPerSec)
{
    Lockstep::MoveRelativeOptions options;
    options.waitUntilIdle = waitUntilIdle;
    options.velocity = speedMmPerSec;
    options.velocityUnit = Units::VELOCITY_MILLIMETRES_PER_SECOND;
    // acceleration left at 0 → use axis accel setting (set at connect).
    return options;
}

Lockstep::MoveAbsoluteOptions makeMoveAbsoluteOptions(const bool waitUntilIdle, const double speedMmPerSec)
{
    Lockstep::MoveAbsoluteOptions options;
    options.waitUntilIdle = waitUntilIdle;
    options.velocity = speedMmPerSec;
    options.velocityUnit = Units::VELOCITY_MILLIMETRES_PER_SECOND;
    return options;
}

bool mapMotionError(const std::string &context, StageError &error, const std::exception &ex)
{
    error.code = StageErrorCode::InternalError;
    error.message = context + ": " + ex.what();
    return false;
}
} // namespace

zaber::motion::ascii::Lockstep ZaberStageMotion::requireEnabledLockstep(zaber::motion::ascii::Device &device,
                                                                        StageError &error)
{
    zaber::motion::ascii::Lockstep lockstep = device.getLockstep(zaber_stage::kLockstepGroupId);
    if (!lockstep.isEnabled())
    {
        error.code = StageErrorCode::InvalidState;
        error.message = "Lockstep group " + std::to_string(zaber_stage::kLockstepGroupId) + " is not enabled";
        throw std::runtime_error(error.message);
    }

    return lockstep;
}

void ZaberStageMotion::enableStageDrivers(zaber::motion::ascii::Device &device)
{
    for (int axisNumber = 1; axisNumber <= zaber_stage::kAxisCount; ++axisNumber)
        device.getAxis(axisNumber).driverEnable();
}

bool ZaberStageMotion::moveRelativeMm(zaber::motion::ascii::Lockstep &lockstep,
                                      const double distanceMm,
                                      StageError &error,
                                      const bool waitUntilIdle,
                                      const double speedMmPerSec,
                                      const double /*accelerationMmPerSec2*/)
{
    try
    {
        lockstep.moveRelative(distanceMm,
                              Units::LENGTH_MILLIMETRES,
                              makeMoveRelativeOptions(waitUntilIdle, speedMmPerSec));
        return true;
    }
    catch (const zaber::motion::exceptions::MotionLibException &ex)
    {
        error.code = StageErrorCode::SdkError;
        error.message = "Primary axis move relative failed: " + ex.getMessage();
        return false;
    }
    catch (const std::exception &ex)
    {
        return mapMotionError("Primary axis move relative failed", error, ex);
    }
}

bool ZaberStageMotion::moveAbsoluteMm(zaber::motion::ascii::Lockstep &lockstep,
                                      const double positionMm,
                                      StageError &error,
                                      const bool waitUntilIdle,
                                      const double speedMmPerSec,
                                      const double /*accelerationMmPerSec2*/)
{
    try
    {
        lockstep.moveAbsolute(positionMm,
                              Units::LENGTH_MILLIMETRES,
                              makeMoveAbsoluteOptions(waitUntilIdle, speedMmPerSec));
        return true;
    }
    catch (const zaber::motion::exceptions::MotionLibException &ex)
    {
        error.code = StageErrorCode::SdkError;
        error.message = "Primary axis move absolute failed: " + ex.getMessage();
        return false;
    }
    catch (const std::exception &ex)
    {
        return mapMotionError("Primary axis move absolute failed", error, ex);
    }
}

bool ZaberStageMotion::homeLockstep(zaber::motion::ascii::Lockstep &lockstep, StageError &error)
{
    try
    {
        Lockstep::HomeOptions homeOptions;
        homeOptions.waitUntilIdle = true;
        lockstep.home(homeOptions);
        return true;
    }
    catch (const zaber::motion::exceptions::MotionLibException &ex)
    {
        error.code = StageErrorCode::SdkError;
        error.message = "Lockstep homing failed: " + ex.getMessage();
        return false;
    }
    catch (const std::exception &ex)
    {
        return mapMotionError("Lockstep homing failed", error, ex);
    }
}

bool ZaberStageMotion::stopLockstep(zaber::motion::ascii::Lockstep &lockstep,
                                    StageError &error,
                                    const bool waitUntilIdle)
{
    try
    {
        Lockstep::StopOptions options;
        options.waitUntilIdle = waitUntilIdle;
        lockstep.stop(options);
        return true;
    }
    catch (const zaber::motion::exceptions::MotionLibException &ex)
    {
        error.code = StageErrorCode::SdkError;
        error.message = "Lockstep stop failed: " + ex.getMessage();
        return false;
    }
    catch (const std::exception &ex)
    {
        return mapMotionError("Lockstep stop failed", error, ex);
    }
}

bool ZaberStageMotion::moveVelocityMm(zaber::motion::ascii::Lockstep &lockstep,
                                      const double velocityMmPerSec,
                                      StageError &error,
                                      const double /*accelerationMmPerSec2*/)
{
    try
    {
        lockstep.moveVelocity(velocityMmPerSec, Units::VELOCITY_MILLIMETRES_PER_SECOND);
        return true;
    }
    catch (const zaber::motion::exceptions::MotionLibException &ex)
    {
        error.code = StageErrorCode::SdkError;
        error.message = "Primary axis velocity move failed: " + ex.getMessage();
        return false;
    }
    catch (const std::exception &ex)
    {
        return mapMotionError("Primary axis velocity move failed", error, ex);
    }
}

bool ZaberStageMotion::getPrimaryPositionMm(zaber::motion::ascii::Lockstep &lockstep,
                                            double &positionMm,
                                            StageError &error)
{
    try
    {
        positionMm = lockstep.getPosition(Units::LENGTH_MILLIMETRES);
        return true;
    }
    catch (const zaber::motion::exceptions::MotionLibException &ex)
    {
        error.code = StageErrorCode::SdkError;
        error.message = "Primary axis position read failed: " + ex.getMessage();
        return false;
    }
    catch (const std::exception &ex)
    {
        return mapMotionError("Primary axis position read failed", error, ex);
    }
}
#endif
