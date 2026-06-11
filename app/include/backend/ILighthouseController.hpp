#pragma once

#include "backend/LighthouseTypes.hpp"

class ILighthouseController
{
public:
    virtual ~ILighthouseController() = default;

    virtual std::string name() const = 0;
    virtual LighthouseState state() const = 0;
    virtual LighthouseDeviceInfo deviceInfo() const = 0;
    virtual LighthouseSettings settings() const = 0;
    virtual LighthouseControllerPowerStatus controllerPowerStatus() const = 0;

    virtual bool scan(LighthouseError &error) = 0;
    virtual void setConnectDefaults(const LighthouseSettings &settings) = 0;
    virtual bool connect(LighthouseError &error) = 0;
    virtual void disconnect() = 0;

    virtual bool setGroupIntensityPercent(LighthouseIntensityGroup group,
                                          int percent,
                                          LighthouseError &error) = 0;
    virtual bool setLampOn(LighthouseLamp lamp, bool on, LighthouseError &error) = 0;
    virtual bool shutdownAll(LighthouseError &error) = 0;
    virtual bool pollControllerPowerStatus(LighthouseControllerPowerStatus &status,
                                         LighthouseError &error) = 0;
};
