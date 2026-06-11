#include "adapters/zaber/ZaberStageController.hpp"

#include "adapters/zaber/ZaberStageConfigurator.hpp"
#include "adapters/zaber/ZaberStageHoming.hpp"
#include "adapters/zaber/ZaberStageMotion.hpp"
#include "adapters/zaber/ZaberStageProfile.hpp"

#include <utility>

#ifdef HF_HAVE_ZML
#include <zaber/motion/ascii.h>
#include <zaber/motion/exceptions/motion_lib_exception.h>
#endif

ZaberStageController::ZaberStageController() = default;

ZaberStageController::~ZaberStageController()
{
    disconnect();
}

std::string ZaberStageController::name() const
{
    return "Zaber stage";
}

StageState ZaberStageController::state() const
{
    return state_;
}

StageTopology ZaberStageController::topology() const
{
    return topology_;
}

bool ZaberStageController::connect(const StageConnectSettings &settings, StageError &error)
{
    disconnect();

    if (settings.portName.empty())
    {
        error.code = StageErrorCode::InternalError;
        error.message = "Stage port name is empty";
        state_ = StageState::Fault;
        return false;
    }

#ifndef HF_HAVE_ZML
    error.code = StageErrorCode::NotImplemented;
    error.message = "Zaber Motion Library is not linked (install ZML and rebuild)";
    state_ = StageState::Fault;
    return false;
#else
    try
    {
        zaber::motion::ascii::Connection::OpenSerialPortOptions options;
        options.baudRate = settings.baudRate;
        options.direct = true;
        options.testPort = true;

        zaber::motion::ascii::Connection connection =
            zaber::motion::ascii::Connection::openSerialPort(settings.portName, options);

        std::vector<zaber::motion::ascii::Device> devices = connection.detectDevices(true);
        if (devices.empty())
        {
            connection.close();
            error.code = StageErrorCode::SdkError;
            error.message = "No Zaber devices detected on " + settings.portName;
            state_ = StageState::Fault;
            return false;
        }

        StageTopology topology;
        topology.portName = settings.portName;
        topology.baudRate = settings.baudRate;

        if (!ZaberStageConfigurator::prepare(connection, topology, error, settings.motionAccelerationMmPerSec2))
        {
            connection.close();
            state_ = StageState::Fault;
            return false;
        }

        if (!ZaberStageConfigurator::enableLockstep(connection, topology, error))
        {
            connection.close();
            state_ = StageState::Fault;
            return false;
        }

        connection_ = std::move(connection);
        topology_ = std::move(topology);
        motionAccelerationMmPerSec2_ = topology_.motionAccelerationMmPerSec2;
        state_ = StageState::Homing;
        return true;
    }
    catch (const zaber::motion::exceptions::MotionLibException &ex)
    {
        error.code = StageErrorCode::SdkError;
        error.message = ex.getMessage();
        state_ = StageState::Fault;
        return false;
    }
    catch (const std::exception &ex)
    {
        error.code = StageErrorCode::InternalError;
        error.message = ex.what();
        state_ = StageState::Fault;
        return false;
    }
#endif
}

bool ZaberStageController::homeWithLocalization(StageError &error)
{
#ifndef HF_HAVE_ZML
    error.code = StageErrorCode::NotImplemented;
    error.message = "Zaber Motion Library is not linked (install ZML and rebuild)";
    state_ = StageState::Fault;
    return false;
#else
    if (!connection_.has_value())
    {
        error.code = StageErrorCode::InvalidState;
        error.message = "Stage is not connected";
        return false;
    }

    try
    {
        state_ = StageState::Homing;

        zaber::motion::ascii::Device device =
            connection_->getDevice(zaber_stage::kDeviceAddress);

        if (!ZaberStageHoming::performLocalizationHoming(device, error))
        {
            state_ = StageState::Fault;
            return false;
        }

        topology_.axesHomed = true;
        state_ = StageState::Connected;
        return true;
    }
    catch (const zaber::motion::exceptions::MotionLibException &ex)
    {
        error.code = StageErrorCode::SdkError;
        error.message = ex.getMessage();
        state_ = StageState::Fault;
        return false;
    }
    catch (const std::exception &ex)
    {
        error.code = StageErrorCode::InternalError;
        error.message = ex.what();
        state_ = StageState::Fault;
        return false;
    }
#endif
}

bool ZaberStageController::home(StageError &error)
{
#ifndef HF_HAVE_ZML
    error.code = StageErrorCode::NotImplemented;
    error.message = "Zaber Motion Library is not linked (install ZML and rebuild)";
    return false;
#else
    if (!connection_.has_value())
    {
        error.code = StageErrorCode::InvalidState;
        error.message = "Stage is not connected";
        return false;
    }

    try
    {
        state_ = StageState::Homing;

        zaber::motion::ascii::Device device =
            connection_->getDevice(zaber_stage::kDeviceAddress);

        if (!ZaberStageHoming::performSimpleHoming(device, error))
        {
            state_ = StageState::Connected;
            return false;
        }

        topology_.axesHomed = true;
        state_ = StageState::Connected;
        return true;
    }
    catch (const zaber::motion::exceptions::MotionLibException &ex)
    {
        error.code = StageErrorCode::SdkError;
        error.message = ex.getMessage();
        state_ = StageState::Connected;
        return false;
    }
    catch (const std::exception &ex)
    {
        error.code = StageErrorCode::InternalError;
        error.message = ex.what();
        state_ = StageState::Connected;
        return false;
    }
#endif
}

bool ZaberStageController::ensureReadyForMotion(StageError &error) const
{
    if (state_ != StageState::Connected)
    {
        error.code = StageErrorCode::InvalidState;
        error.message = "Stage is not ready for motion";
        return false;
    }

    if (!connection_.has_value() || !topology_.lockstepEnabled || !topology_.axesHomed)
    {
        error.code = StageErrorCode::InvalidState;
        error.message = "Stage lockstep is not ready for motion";
        return false;
    }

    return true;
}

bool ZaberStageController::ensureCanReadPosition(StageError &error) const
{
    if (state_ != StageState::Connected && state_ != StageState::Homing)
    {
        error.code = StageErrorCode::InvalidState;
        error.message = "Stage is not connected";
        return false;
    }

    if (!connection_.has_value() || !topology_.lockstepEnabled)
    {
        error.code = StageErrorCode::InvalidState;
        error.message = "Stage lockstep is not available";
        return false;
    }

    return true;
}

bool ZaberStageController::moveRelativeMm(const double distanceMm,
                                          const double speedMmPerSec,
                                          StageError &error)
{
#ifndef HF_HAVE_ZML
    error.code = StageErrorCode::NotImplemented;
    error.message = "Zaber Motion Library is not linked (install ZML and rebuild)";
    return false;
#else
    if (!ensureReadyForMotion(error))
        return false;

    try
    {
        zaber::motion::ascii::Device device =
            connection_->getDevice(zaber_stage::kDeviceAddress);
        zaber::motion::ascii::Lockstep lockstep = ZaberStageMotion::requireEnabledLockstep(device, error);
        return ZaberStageMotion::moveRelativeMm(
            lockstep, distanceMm, error, false, speedMmPerSec, motionAccelerationMmPerSec2_);
    }
    catch (const std::exception &)
    {
        return false;
    }
#endif
}

bool ZaberStageController::moveAbsoluteMm(const double positionMm,
                                          const double speedMmPerSec,
                                          const bool waitUntilIdle,
                                          StageError &error)
{
#ifndef HF_HAVE_ZML
    error.code = StageErrorCode::NotImplemented;
    error.message = "Zaber Motion Library is not linked (install ZML and rebuild)";
    return false;
#else
    if (!ensureReadyForMotion(error))
        return false;

    try
    {
        zaber::motion::ascii::Device device =
            connection_->getDevice(zaber_stage::kDeviceAddress);
        zaber::motion::ascii::Lockstep lockstep = ZaberStageMotion::requireEnabledLockstep(device, error);
        return ZaberStageMotion::moveAbsoluteMm(
            lockstep, positionMm, error, waitUntilIdle, speedMmPerSec, motionAccelerationMmPerSec2_);
    }
    catch (const std::exception &)
    {
        return false;
    }
#endif
}

bool ZaberStageController::moveVelocityMm(const double velocityMmPerSec, StageError &error)
{
#ifndef HF_HAVE_ZML
    error.code = StageErrorCode::NotImplemented;
    error.message = "Zaber Motion Library is not linked (install ZML and rebuild)";
    return false;
#else
    if (!ensureReadyForMotion(error))
        return false;

    try
    {
        zaber::motion::ascii::Device device =
            connection_->getDevice(zaber_stage::kDeviceAddress);
        zaber::motion::ascii::Lockstep lockstep = ZaberStageMotion::requireEnabledLockstep(device, error);
        return ZaberStageMotion::moveVelocityMm(lockstep, velocityMmPerSec, error, motionAccelerationMmPerSec2_);
    }
    catch (const std::exception &)
    {
        return false;
    }
#endif
}

bool ZaberStageController::getPrimaryPositionMm(double &positionMm, StageError &error)
{
#ifndef HF_HAVE_ZML
    error.code = StageErrorCode::NotImplemented;
    error.message = "Zaber Motion Library is not linked (install ZML and rebuild)";
    return false;
#else
    if (!ensureCanReadPosition(error))
        return false;

    try
    {
        zaber::motion::ascii::Device device =
            connection_->getDevice(zaber_stage::kDeviceAddress);
        zaber::motion::ascii::Lockstep lockstep = ZaberStageMotion::requireEnabledLockstep(device, error);
        return ZaberStageMotion::getPrimaryPositionMm(lockstep, positionMm, error);
    }
    catch (const std::exception &)
    {
        return false;
    }
#endif
}

void ZaberStageController::stopMotion()
{
#ifndef HF_HAVE_ZML
    return;
#else
    if (!connection_.has_value())
        return;

    try
    {
        zaber::motion::ascii::Device device =
            connection_->getDevice(zaber_stage::kDeviceAddress);
        StageError error;
        zaber::motion::ascii::Lockstep lockstep = ZaberStageMotion::requireEnabledLockstep(device, error);
        ZaberStageMotion::stopLockstep(lockstep, error);
    }
    catch (...)
    {
    }
#endif
}

void ZaberStageController::disconnect()
{
#ifdef HF_HAVE_ZML
    if (connection_.has_value())
    {
        try
        {
            if (topology_.lockstepEnabled)
            {
                zaber::motion::ascii::Device device =
                    connection_->getDevice(zaber_stage::kDeviceAddress);
                zaber::motion::ascii::Lockstep lockstep =
                    device.getLockstep(zaber_stage::kLockstepGroupId);
                if (lockstep.isEnabled())
                    lockstep.disable();
            }

            connection_->close();
        }
        catch (...)
        {
        }
        connection_.reset();
    }
#endif

    topology_ = {};
    motionAccelerationMmPerSec2_ = zaber_stage::kDefaultMotionAccelerationMmPerSec2;
    state_ = StageState::Disconnected;
}
