#pragma once

#include "adapters/mcc/MccUniversalLibrary.hpp"
#include "backend/light/ILighthouseController.hpp"

#include <functional>
#include <string>

class Mcc1208LighthouseController : public ILighthouseController
{
public:
    using LogCallback = std::function<void(const std::string &message)>;

    void setLogCallback(LogCallback callback);

    std::string name() const override;
    LighthouseState state() const override;
    LighthouseDeviceInfo deviceInfo() const override;
    LighthouseSettings settings() const override;
    LighthouseControllerPowerStatus controllerPowerStatus() const override;

    bool scan(LighthouseError &error) override;
    void setConnectDefaults(const LighthouseSettings &settings) override;
    bool connect(LighthouseError &error) override;
    void disconnect() override;

    bool setGroupIntensityPercent(LighthouseIntensityGroup group,
                                  int percent,
                                  LighthouseError &error) override;
    bool setLampOn(LighthouseLamp lamp, bool on, LighthouseError &error) override;
    bool shutdownAll(LighthouseError &error) override;
    bool pollControllerPowerStatus(LighthouseControllerPowerStatus &status,
                                 LighthouseError &error) override;

private:
    bool ensureLibraryLoaded(LighthouseError &error);
    bool applyConnectDefaults(LighthouseError &error);
    bool applySafeIdleOutputs(LighthouseError &error);
    static int clampPercent(int percent);
    void logMessage(const std::string &message) const;

    mcc::MccUniversalLibrary ul_;
    LogCallback logCallback_;
    LighthouseState state_ = LighthouseState::Disconnected;
    LighthouseDeviceInfo deviceInfo_;
    LighthouseSettings settings_;
    LighthouseSettings connectDefaults_;
    LighthouseControllerPowerStatus powerStatus_;
    bool deviceDetected_ = false;
    int activeBoardNumber_ = -1;
};
