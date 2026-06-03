#include "adapters/zaber/ZaberStageConfigurator.hpp"

#include "adapters/zaber/ZaberStageProfile.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>

#ifdef HF_HAVE_ZML
#include <zaber/motion/ascii.h>
#include <zaber/motion/ascii/setting_constants.h>
#include <zaber/motion/exceptions/motion_lib_exception.h>
#endif

namespace
{
#ifdef HF_HAVE_ZML
using zaber::motion::Units;
using zaber::motion::ascii::setting_constants::LIMIT_HOME_OFFSET;
using zaber::motion::ascii::setting_constants::LIMIT_MAX;
using zaber::motion::ascii::setting_constants::LIMIT_MIN;
using zaber::motion::ascii::setting_constants::MAXSPEED;
using zaber::motion::ascii::setting_constants::POS;

bool containsIgnoreCase(const std::string &haystack, const std::string &needle)
{
    if (needle.empty())
        return true;

    return std::search(haystack.begin(),
                       haystack.end(),
                       needle.begin(),
                       needle.end(),
                       [](const char left, const char right) {
                           return std::tolower(static_cast<unsigned char>(left))
                                  == std::tolower(static_cast<unsigned char>(right));
                       }) != haystack.end();
}

std::string formatFirmwareVersion(const zaber::motion::FirmwareVersion &version)
{
    std::ostringstream stream;
    stream << version.getMajor() << '.' << version.getMinor() << '.' << version.getBuild();
    return stream.str();
}

StageDeviceInfo buildStageDeviceInfo(zaber::motion::ascii::Device &device)
{
    StageDeviceInfo info;
    info.deviceAddress = device.getDeviceAddress();
    info.name = device.getName();
    info.serialNumber = device.getSerialNumber();
    info.firmwareVersion = formatFirmwareVersion(device.getFirmwareVersion());
    info.axisCount = device.getAxisCount();

    for (int axisNumber = 1; axisNumber <= info.axisCount; ++axisNumber)
    {
        const zaber::motion::ascii::Axis axis = device.getAxis(axisNumber);
        const zaber::motion::ascii::AxisIdentity axisIdentity = axis.getIdentity();

        StageAxisInfo axisInfo;
        axisInfo.deviceAddress = info.deviceAddress;
        axisInfo.axisNumber = axisNumber;
        axisInfo.peripheralName = axisIdentity.getPeripheralName();
        axisInfo.peripheralSerialNumber = axisIdentity.getPeripheralSerialNumber();
        axisInfo.isPeripheral = axisIdentity.getIsPeripheral();
        info.axes.push_back(std::move(axisInfo));
    }

    return info;
}

bool validateStageDevice(zaber::motion::ascii::Device &device, StageError &error)
{
    if (device.getDeviceAddress() != zaber_stage::kDeviceAddress)
    {
        error.code = StageErrorCode::SdkError;
        error.message = "Expected stage controller at device address "
                          + std::to_string(zaber_stage::kDeviceAddress);
        return false;
    }

    if (!containsIgnoreCase(device.getName(), zaber_stage::kExpectedControllerPrefix))
    {
        error.code = StageErrorCode::SdkError;
        error.message = "Unexpected controller: " + device.getName();
        return false;
    }

    if (device.getAxisCount() < zaber_stage::kAxisCount)
    {
        error.code = StageErrorCode::SdkError;
        error.message = "Expected at least " + std::to_string(zaber_stage::kAxisCount) + " axes on device "
                          + std::to_string(device.getDeviceAddress());
        return false;
    }

    for (int axisNumber = 1; axisNumber <= zaber_stage::kAxisCount; ++axisNumber)
    {
        zaber::motion::ascii::Axis axis = device.getAxis(axisNumber);
        const std::string peripheralName = axis.getIdentity().getPeripheralName();
        if (!containsIgnoreCase(peripheralName, zaber_stage::kExpectedPeripheralName))
        {
            error.code = StageErrorCode::SdkError;
            error.message = "Axis " + std::to_string(axisNumber) + " peripheral mismatch: " + peripheralName;
            return false;
        }
    }

    return true;
}

void applyTravelLimits(zaber::motion::ascii::Axis &axis)
{
    zaber::motion::ascii::AxisSettings settings = axis.getSettings();
    settings.set(LIMIT_MIN, zaber_stage::kTravelMinimumMm, Units::LENGTH_MILLIMETRES);
    settings.set(LIMIT_MAX, zaber_stage::kTravelLengthMm, Units::LENGTH_MILLIMETRES);
    settings.set(LIMIT_HOME_OFFSET, zaber_stage::kHomeOffsetMm, Units::LENGTH_MILLIMETRES);
}

void applyMotionSpeedLimits(zaber::motion::ascii::Axis &axis)
{
    // limit.approach.maxspeed is not writable at the default direct-serial access level
    // (NOACCESS). Homing uses min(limit.approach.maxspeed, maxspeed), so maxspeed alone
    // caps approach speed when it is the lower value.
    zaber::motion::ascii::AxisSettings settings = axis.getSettings();
    settings.set(MAXSPEED, zaber_stage::kMaxSpeedMmPerSec, Units::VELOCITY_MILLIMETRES_PER_SECOND);
}

void disableLockstepIfEnabled(zaber::motion::ascii::Device &device)
{
    zaber::motion::ascii::Lockstep lockstep = device.getLockstep(zaber_stage::kLockstepGroupId);
    if (lockstep.isEnabled())
        lockstep.disable();
}

void applyTravelLimitsToStageAxes(zaber::motion::ascii::Device &device)
{
    for (int axisNumber = 1; axisNumber <= zaber_stage::kAxisCount; ++axisNumber)
    {
        zaber::motion::ascii::Axis axis = device.getAxis(axisNumber);
        applyTravelLimits(axis);
        applyMotionSpeedLimits(axis);
    }
}

void alignAxesForZeroLockstepOffset(zaber::motion::ascii::Device &device)
{
    zaber::motion::ascii::Axis primary = device.getAxis(zaber_stage::kPrimaryAxisNumber);
    const double primaryPositionMm = primary.getPosition(Units::LENGTH_MILLIMETRES);

    for (int axisNumber = zaber_stage::kSecondaryAxisNumber; axisNumber <= zaber_stage::kAxisCount; ++axisNumber)
    {
        zaber::motion::ascii::Axis secondary = device.getAxis(axisNumber);
        zaber::motion::ascii::AxisSettings settings = secondary.getSettings();
        settings.set(POS, primaryPositionMm, Units::LENGTH_MILLIMETRES);
    }
}
#endif
} // namespace

#ifdef HF_HAVE_ZML
bool ZaberStageConfigurator::prepare(zaber::motion::ascii::Connection &connection,
                                     StageTopology &topology,
                                     StageError &error)
{
    topology.lockstepEnabled = false;
    topology.axesHomed = false;

    zaber::motion::ascii::Device device = connection.getDevice(zaber_stage::kDeviceAddress);
    if (!validateStageDevice(device, error))
        return false;

    disableLockstepIfEnabled(device);
    applyTravelLimitsToStageAxes(device);

    topology.travelLengthMm = zaber_stage::kTravelLengthMm;
    topology.stageType = zaber_stage::kStageType;
    topology.maxSpeedMmPerSec = zaber_stage::kMaxSpeedMmPerSec;
    topology.devices.clear();
    topology.devices.push_back(buildStageDeviceInfo(device));
    return true;
}

bool ZaberStageConfigurator::enableLockstep(zaber::motion::ascii::Connection &connection,
                                            StageTopology &topology,
                                            StageError &error)
{
    zaber::motion::ascii::Device device = connection.getDevice(zaber_stage::kDeviceAddress);
    zaber::motion::ascii::Lockstep lockstep = device.getLockstep(zaber_stage::kLockstepGroupId);
    if (lockstep.isEnabled())
        lockstep.disable();

    try
    {
        alignAxesForZeroLockstepOffset(device);

        lockstep.enable({zaber_stage::kPrimaryAxisNumber, zaber_stage::kSecondaryAxisNumber});

        const std::vector<double> offsets = lockstep.getOffsets(Units::LENGTH_MILLIMETRES);
        const double secondaryOffsetMm = offsets.empty() ? 0.0 : offsets.front();
        topology.lockstepSecondaryOffsetMm = secondaryOffsetMm;

        if (std::abs(secondaryOffsetMm - zaber_stage::kLockstepSecondaryOffsetMm) > 0.01)
        {
            error.code = StageErrorCode::SdkError;
            error.message = "Lockstep secondary offset is " + std::to_string(secondaryOffsetMm)
                              + " mm (expected "
                              + std::to_string(zaber_stage::kLockstepSecondaryOffsetMm) + " mm)";
            lockstep.disable();
            return false;
        }
    }
    catch (const zaber::motion::exceptions::MotionLibException &ex)
    {
        error.code = StageErrorCode::SdkError;
        error.message = ex.getMessage();
        return false;
    }
    catch (const std::exception &ex)
    {
        error.code = StageErrorCode::InternalError;
        error.message = ex.what();
        return false;
    }

    topology.lockstepGroupId = zaber_stage::kLockstepGroupId;
    topology.lockstepPrimaryAxis = zaber_stage::kPrimaryAxisNumber;
    topology.lockstepSecondaryAxis = zaber_stage::kSecondaryAxisNumber;
    topology.lockstepEnabled = lockstep.isEnabled();
    return true;
}
#endif
