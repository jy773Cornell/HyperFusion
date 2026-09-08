// Adapter: USB-1208FS-Plus lighthouse connect/scan/AO/DIO. No motion; USB DAQ only after Connect.
#include "adapters/mcc/Mcc1208LighthouseController.hpp"

#include "adapters/mcc/Mcc1208Profile.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace
{
constexpr double kAnalogOutputVoltsMax = 5.0;

double percentToVolts(const int percent)
{
    const int clamped = std::clamp(percent, 0, kLighthouseIntensityPercentMax);
    return (static_cast<double>(clamped) / static_cast<double>(kLighthouseIntensityPercentMax))
           * kAnalogOutputVoltsMax;
}
} // namespace

void Mcc1208LighthouseController::setLogCallback(LogCallback callback)
{
    logCallback_ = std::move(callback);
}

void Mcc1208LighthouseController::logMessage(const std::string &message) const
{
    if (logCallback_)
        logCallback_(message);
}

std::string Mcc1208LighthouseController::name() const
{
    return "USB-1208FS-Plus";
}

LighthouseState Mcc1208LighthouseController::state() const
{
    return state_;
}

LighthouseDeviceInfo Mcc1208LighthouseController::deviceInfo() const
{
    return deviceInfo_;
}

LighthouseSettings Mcc1208LighthouseController::settings() const
{
    return settings_;
}

LighthouseControllerPowerStatus Mcc1208LighthouseController::controllerPowerStatus() const
{
    return powerStatus_;
}

bool Mcc1208LighthouseController::ensureLibraryLoaded(LighthouseError &error)
{
    if (ul_.isLoaded())
        return true;

    return ul_.load(error);
}

bool Mcc1208LighthouseController::scan(LighthouseError &error)
{
    error = {};

    if (!ensureLibraryLoaded(error))
    {
        state_ = LighthouseState::Fault;
        return false;
    }

    LighthouseDeviceInfo detected;
    if (!ul_.scanFor1208FsPlus(detected, error))
    {
        deviceDetected_ = false;
        deviceInfo_ = {};
        activeBoardNumber_ = -1;
        state_ = LighthouseState::Disconnected;
        return false;
    }

    deviceDetected_ = true;
    deviceInfo_ = detected;
    activeBoardNumber_ = detected.boardNumber;
    state_ = LighthouseState::Detected;
    logMessage("Light backend: USB-1208FS-Plus detected on InstaCal board "
               + std::to_string(activeBoardNumber_));
    return true;
}

void Mcc1208LighthouseController::setConnectDefaults(const LighthouseSettings &settings)
{
    connectDefaults_ = settings;
    connectDefaults_.reflectancePercent = clampPercent(connectDefaults_.reflectancePercent);
    connectDefaults_.transmittancePercent = clampPercent(connectDefaults_.transmittancePercent);
    connectDefaults_.lampOn.fill(true);
}

bool Mcc1208LighthouseController::connect(LighthouseError &error)
{
    error = {};

    if (!deviceDetected_ || activeBoardNumber_ < 0)
    {
        error.code = LighthouseErrorCode::DeviceNotFound;
        error.message = "USB-1208FS-Plus not detected. Refresh before connecting.";
        state_ = LighthouseState::Disconnected;
        return false;
    }

    if (!ensureLibraryLoaded(error))
    {
        state_ = LighthouseState::Fault;
        return false;
    }

    LighthouseDeviceInfo detected;
    if (!ul_.scanFor1208FsPlus(detected, error))
    {
        deviceDetected_ = false;
        deviceInfo_ = {};
        activeBoardNumber_ = -1;
        state_ = LighthouseState::Disconnected;
        return false;
    }

    activeBoardNumber_ = detected.boardNumber;
    deviceInfo_ = detected;

    // SE vs DIFF is InstaCal on this board. cbAInputMode() faults (BADFUNCTION) here.
    logMessage("Light backend: analog input mode from InstaCal (USB-1208FS-Plus has no cbAInputMode)");

    if (!ul_.configurePortAOutput(activeBoardNumber_, error))
    {
        state_ = LighthouseState::Fault;
        return false;
    }

    if (!applyConnectDefaults(error))
    {
        state_ = LighthouseState::Fault;
        return false;
    }

    deviceInfo_.details = detected.details + "\nConnected. Port A configured for output.";
    state_ = LighthouseState::Connected;
    logMessage("Light backend: USB-1208FS-Plus connected on board "
               + std::to_string(activeBoardNumber_)
               + " \u2014 all lamps on, reflectance "
               + std::to_string(settings_.reflectancePercent)
               + "%, transmittance "
               + std::to_string(settings_.transmittancePercent)
               + "%");
    return true;
}

void Mcc1208LighthouseController::disconnect()
{
    LighthouseError error;
    if (state_ == LighthouseState::Connected && activeBoardNumber_ >= 0)
    {
        if (!shutdownAll(error))
            logMessage("Light backend: shutdown outputs failed: " + error.message);
    }

    deviceDetected_ = false;
    deviceInfo_ = {};
    activeBoardNumber_ = -1;
    powerStatus_ = {};
    state_ = LighthouseState::Disconnected;
    logMessage("Light backend: disconnected, all outputs off");
}

bool Mcc1208LighthouseController::pollControllerPowerStatus(LighthouseControllerPowerStatus &status,
                                                            LighthouseError &error)
{
    error = {};
    status = {};

    if (state_ != LighthouseState::Connected || activeBoardNumber_ < 0)
        return true;

    LighthouseControllerPowerStatus reading;
    for (int lampIndex = 0; lampIndex < kLighthouseLampCount; ++lampIndex)
    {
        const int channel = lighthousePowerMonitorChannelForLamp(static_cast<LighthouseLamp>(lampIndex));
        float volts = 0.0f;
        if (!ul_.readAnalogInputVolts(activeBoardNumber_, channel, volts, error))
            return false;

        reading.monitorVolts[static_cast<std::size_t>(lampIndex)] = volts;
        reading.controllerAlive[static_cast<std::size_t>(lampIndex)] = lighthouseControllerIsAlive(volts);
    }

    reading.valid = true;
    powerStatus_ = reading;
    status = reading;
    return true;
}

bool Mcc1208LighthouseController::setGroupIntensityPercent(const LighthouseIntensityGroup group,
                                                           const int percent,
                                                           LighthouseError &error)
{
    error = {};

    if (state_ != LighthouseState::Connected || activeBoardNumber_ < 0)
    {
        error.code = LighthouseErrorCode::InvalidState;
        error.message = "Lighthouse controller is not connected.";
        return false;
    }

    const int clamped = clampPercent(percent);
    settings_.setGroupPercent(group, clamped);

    const int analogChannel = lighthouseAnalogChannelForGroup(group);
    const float volts = static_cast<float>(percentToVolts(clamped));

    if (!ul_.writeAnalogVolts(activeBoardNumber_, analogChannel, volts, error))
        return false;

    std::ostringstream message;
    message << "Light backend: AO" << analogChannel << " -> " << volts << " V (" << clamped << "%)";
    logMessage(message.str());
    return true;
}

bool Mcc1208LighthouseController::setLampOn(const LighthouseLamp lamp, const bool on, LighthouseError &error)
{
    error = {};

    if (state_ != LighthouseState::Connected || activeBoardNumber_ < 0)
    {
        error.code = LighthouseErrorCode::InvalidState;
        error.message = "Lighthouse controller is not connected.";
        return false;
    }

    const int relayPort = lighthouseRelayPortForLamp(lamp);
    if (relayPort < 0)
    {
        error.code = LighthouseErrorCode::InternalError;
        error.message = "Invalid lighthouse lamp index.";
        return false;
    }

    const int lampIndex = static_cast<int>(lamp);
    settings_.lampOn[static_cast<std::size_t>(lampIndex)] = on;

    if (!ul_.writeDigitalBit(activeBoardNumber_, relayPort, lighthouseRelayOutputHigh(on), error))
        return false;

    std::ostringstream message;
    message << "Light backend: DIO A" << relayPort << " -> " << (on ? "ON" : "OFF");
    logMessage(message.str());
    return true;
}

bool Mcc1208LighthouseController::shutdownAll(LighthouseError &error)
{
    error = {};

    settings_.reflectancePercent = 0;
    settings_.transmittancePercent = 0;
    settings_.lampOn.fill(false);

    if (state_ != LighthouseState::Connected || activeBoardNumber_ < 0)
        return true;

    if (!ul_.shutdownLighthouseOutputs(activeBoardNumber_, error))
        return false;

    logMessage("Light backend: shutdown all outputs");
    return true;
}

bool Mcc1208LighthouseController::applyConnectDefaults(LighthouseError &error)
{
    error = {};
    if (activeBoardNumber_ < 0)
    {
        error.code = LighthouseErrorCode::InvalidState;
        error.message = "USB-1208FS-Plus board number is not set.";
        return false;
    }

    settings_.reflectancePercent = connectDefaults_.reflectancePercent;
    settings_.transmittancePercent = connectDefaults_.transmittancePercent;
    settings_.lampOn.fill(true);

    const float reflectanceVolts =
        static_cast<float>(percentToVolts(settings_.reflectancePercent));
    const float transmittanceVolts =
        static_cast<float>(percentToVolts(settings_.transmittancePercent));

    if (!ul_.writeAnalogVolts(activeBoardNumber_,
                              kLighthouseAnalogChannelReflectance,
                              reflectanceVolts,
                              error))
        return false;

    if (!ul_.writeAnalogVolts(activeBoardNumber_,
                              kLighthouseAnalogChannelTransmittance,
                              transmittanceVolts,
                              error))
        return false;

    for (int lampIndex = 0; lampIndex < kLighthouseLampCount; ++lampIndex)
    {
        const int relayPort = lighthouseRelayPortForLamp(static_cast<LighthouseLamp>(lampIndex));
        if (!ul_.writeDigitalBit(activeBoardNumber_,
                                 relayPort,
                                 lighthouseRelayOutputHigh(true),
                                 error))
            return false;
    }

    return true;
}

bool Mcc1208LighthouseController::applySafeIdleOutputs(LighthouseError &error)
{
    error = {};
    settings_.reflectancePercent = 0;
    settings_.transmittancePercent = 0;
    settings_.lampOn.fill(false);

    if (activeBoardNumber_ < 0)
        return true;

    return ul_.shutdownLighthouseOutputs(activeBoardNumber_, error);
}

int Mcc1208LighthouseController::clampPercent(const int percent)
{
    return std::clamp(percent, 0, kLighthouseIntensityPercentMax);
}
