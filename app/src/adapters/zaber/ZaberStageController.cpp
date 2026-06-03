#include "adapters/zaber/ZaberStageController.hpp"

#include "adapters/zaber/ZaberStageConfigurator.hpp"
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

        if (!ZaberStageConfigurator::configure(connection, topology, error))
        {
            connection.close();
            state_ = StageState::Fault;
            return false;
        }

        connection_ = std::move(connection);
        topology_ = std::move(topology);
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
    if (state_ != StageState::Connecting)
        state_ = StageState::Disconnected;
}
