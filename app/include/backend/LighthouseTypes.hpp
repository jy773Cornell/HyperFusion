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
    Transmission1 = 2,
    Transmission2 = 3
};

enum class LighthouseIntensityGroup : int
{
    Reflectance = 0,
    Transmission = 1
};

inline constexpr int kLighthouseLampCount = 4;
inline constexpr int kLighthouseIntensityPercentMax = 100;

// USB-1208FS-Plus lighthouse wiring (see docs/DC950).
// On/off: 8-ch relay board on DIO Port A (A0-A3). Active-low: LOW = lamp on, HIGH = lamp off.
inline constexpr int kLighthouseRelayPortReflectance1 = 0; // Port A0
inline constexpr int kLighthouseRelayPortReflectance2 = 1; // Port A1
inline constexpr int kLighthouseRelayPortTransmission1 = 2; // Port A2
inline constexpr int kLighthouseRelayPortTransmission2 = 3; // Port A3

// Intensity: shared analog voltage per pair (0-5 V from UI 0-100%).
inline constexpr int kLighthouseAnalogChannelReflectance = 0; // AO0
inline constexpr int kLighthouseAnalogChannelTransmission = 1; // AO1

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

struct LighthouseSettings
{
    int reflectancePercent = 0;
    int transmissionPercent = 0;
    std::array<bool, kLighthouseLampCount> lampOn{};

    int groupPercent(const LighthouseIntensityGroup group) const
    {
        return group == LighthouseIntensityGroup::Reflectance ? reflectancePercent : transmissionPercent;
    }

    void setGroupPercent(const LighthouseIntensityGroup group, const int percent)
    {
        if (group == LighthouseIntensityGroup::Reflectance)
            reflectancePercent = percent;
        else
            transmissionPercent = percent;
    }

    int lampPercent(const LighthouseLamp lamp) const
    {
        const auto group = static_cast<int>(lamp) < 2 ? LighthouseIntensityGroup::Reflectance
                                                     : LighthouseIntensityGroup::Transmission;
        return groupPercent(group);
    }
};

inline LighthouseIntensityGroup lighthouseGroupForLamp(const LighthouseLamp lamp)
{
    return static_cast<int>(lamp) < 2 ? LighthouseIntensityGroup::Reflectance
                                      : LighthouseIntensityGroup::Transmission;
}

inline LighthouseIntensityGroup lighthouseGroupForRowIndex(const int rowIndex)
{
    return rowIndex < 2 ? LighthouseIntensityGroup::Reflectance : LighthouseIntensityGroup::Transmission;
}

inline int lighthouseRelayPortForLamp(const LighthouseLamp lamp)
{
    switch (lamp)
    {
    case LighthouseLamp::Reflectance1:
        return kLighthouseRelayPortReflectance1;
    case LighthouseLamp::Reflectance2:
        return kLighthouseRelayPortReflectance2;
    case LighthouseLamp::Transmission1:
        return kLighthouseRelayPortTransmission1;
    case LighthouseLamp::Transmission2:
        return kLighthouseRelayPortTransmission2;
    default:
        return -1;
    }
}

inline int lighthouseAnalogChannelForGroup(const LighthouseIntensityGroup group)
{
    return group == LighthouseIntensityGroup::Reflectance ? kLighthouseAnalogChannelReflectance
                                                          : kLighthouseAnalogChannelTransmission;
}

// Maps UI "lamp on" to DIO level for the active-low relay board.
inline bool lighthouseRelayOutputHigh(const bool lampOn)
{
    return !lampOn;
}
