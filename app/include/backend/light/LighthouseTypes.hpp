#pragma once

#include <array>
#include <cstdint>
#include <string>

enum class LighthouseState
{
    Disconnected,
    Scanning,
    Detected,
    Connecting,
    Connected,
    Fault
};

enum class LighthouseErrorCode
{
    None,
    InvalidState,
    DeviceNotFound,
    NotImplemented,
    SdkError,
    InternalError
};

enum class LighthouseLamp : int
{
    Reflectance1 = 0,
    Reflectance2 = 1,
    Transmittance1 = 2,
    Transmittance2 = 3
};

enum class LighthouseIntensityGroup : int
{
    Reflectance = 0,
    Transmittance = 1
};

inline constexpr int kLighthouseLampCount = 4;
inline constexpr int kLighthouseIntensityPercentMax = 100;

// USB-1208FS-Plus lighthouse wiring (see docs/DC950).
// On/off: 8-ch relay board on DIO Port A (A0-A3). Active-low: LOW = lamp on, HIGH = lamp off.
inline constexpr int kLighthouseRelayPortReflectance1 = 0; // Port A0
inline constexpr int kLighthouseRelayPortReflectance2 = 1; // Port A1
inline constexpr int kLighthouseRelayPortTransmittance1 = 2; // Port A2
inline constexpr int kLighthouseRelayPortTransmittance2 = 3; // Port A3

// Intensity: shared analog voltage per pair (0-5 V from UI 0-100%).
inline constexpr int kLighthouseAnalogChannelReflectance = 0; // AO0
inline constexpr int kLighthouseAnalogChannelTransmittance = 1; // AO1

// DC950 controller power monitor: Pin 1 (+5V) on AI CH0-CH3 (see docs/DC950).
inline constexpr int kLighthousePowerMonitorChannelStart = 0;
inline constexpr float kLighthouseControllerAliveVoltsThreshold = 3.0f;

struct LighthouseError
{
    LighthouseErrorCode code = LighthouseErrorCode::None;
    std::string message;
    bool fatal = false;
};

struct LighthouseDeviceInfo
{
    std::string deviceName;
    int boardNumber = -1;
    std::string details;
};

struct LighthouseControllerPowerStatus
{
    std::array<float, kLighthouseLampCount> monitorVolts{};
    std::array<bool, kLighthouseLampCount> controllerAlive{};
    bool valid = false;
};

struct LighthouseSettings
{
    int reflectancePercent = 0;
    int transmittancePercent = 0;
    std::array<bool, kLighthouseLampCount> lampOn{};

    int groupPercent(const LighthouseIntensityGroup group) const
    {
        return group == LighthouseIntensityGroup::Reflectance ? reflectancePercent : transmittancePercent;
    }

    void setGroupPercent(const LighthouseIntensityGroup group, const int percent)
    {
        if (group == LighthouseIntensityGroup::Reflectance)
            reflectancePercent = percent;
        else
            transmittancePercent = percent;
    }

    int lampPercent(const LighthouseLamp lamp) const
    {
        const auto group = static_cast<int>(lamp) < 2 ? LighthouseIntensityGroup::Reflectance
                                                      : LighthouseIntensityGroup::Transmittance;
        return groupPercent(group);
    }
};

inline LighthouseIntensityGroup lighthouseGroupForLamp(const LighthouseLamp lamp)
{
    return static_cast<int>(lamp) < 2 ? LighthouseIntensityGroup::Reflectance
                                      : LighthouseIntensityGroup::Transmittance;
}

inline LighthouseIntensityGroup lighthouseGroupForRowIndex(const int rowIndex)
{
    return rowIndex < 2 ? LighthouseIntensityGroup::Reflectance : LighthouseIntensityGroup::Transmittance;
}

inline int lighthouseRelayPortForLamp(const LighthouseLamp lamp)
{
    switch (lamp)
    {
    case LighthouseLamp::Reflectance1:
        return kLighthouseRelayPortReflectance1;
    case LighthouseLamp::Reflectance2:
        return kLighthouseRelayPortReflectance2;
    case LighthouseLamp::Transmittance1:
        return kLighthouseRelayPortTransmittance1;
    case LighthouseLamp::Transmittance2:
        return kLighthouseRelayPortTransmittance2;
    default:
        return -1;
    }
}

inline int lighthouseAnalogChannelForGroup(const LighthouseIntensityGroup group)
{
    return group == LighthouseIntensityGroup::Reflectance ? kLighthouseAnalogChannelReflectance
                                                          : kLighthouseAnalogChannelTransmittance;
}

// Maps UI "lamp on" to DIO level for the active-low relay board.
inline bool lighthouseRelayOutputHigh(const bool lampOn)
{
    return !lampOn;
}

inline int lighthousePowerMonitorChannelForLamp(const LighthouseLamp lamp)
{
    return static_cast<int>(lamp);
}

inline bool lighthouseControllerIsAlive(const float monitorVolts)
{
    return monitorVolts > kLighthouseControllerAliveVoltsThreshold;
}

inline const char *lighthouseWiringDetailsText()
{
    return "Reflectance 1 and 2 share AO0 intensity (on/off: DIO A0, A1).\n"
           "Transmittance 1 and 2 share AO1 intensity (on/off: DIO A2, A3).\n"
           "Relay active-low (LOW = on, HIGH = off).\n"
           "DC950 power: AI CH0-CH3 (~5 V = controller alive).";
}
