#pragma once

#include <cstdint>
#include <string>
#include <vector>

enum class StageState
{
    Disconnected,
    Connecting,
    Homing,
    Connected,
    Fault
};

enum class StageErrorCode
{
    None,
    InvalidState,
    NotImplemented,
    SdkError,
    InternalError
};

struct StageError
{
    StageErrorCode code = StageErrorCode::None;
    std::string message;
    bool fatal = false;
};

struct StageConnectSettings
{
    std::string portName;
    int baudRate = 115200;
    /// Trapezoidal profile acceleration (mm/s²) applied to both axes on connect and all lockstep moves.
    double motionAccelerationMmPerSec2 = 0.0;
};

struct StageAxisInfo
{
    int deviceAddress = 0;
    int axisNumber = 0;
    std::string peripheralName;
    std::uint32_t peripheralSerialNumber = 0;
    bool isPeripheral = false;
};

struct StageDeviceInfo
{
    int deviceAddress = 0;
    std::string name;
    std::uint32_t serialNumber = 0;
    std::string firmwareVersion;
    int axisCount = 0;
    std::vector<StageAxisInfo> axes;
};

struct StageTopology
{
    std::string portName;
    int baudRate = 0;
    std::vector<StageDeviceInfo> devices;

    bool lockstepEnabled = false;
    int lockstepGroupId = 0;
    int lockstepPrimaryAxis = 0;
    int lockstepSecondaryAxis = 0;
    double travelLengthMm = 0.0;
    std::string stageType;
    bool axesHomed = false;
    double maxSpeedMmPerSec = 0.0;
    double motionAccelerationRequestedMmPerSec2 = 0.0;
    double motionAccelerationMmPerSec2 = 0.0;
    double lockstepSecondaryOffsetMm = 0.0;
};
