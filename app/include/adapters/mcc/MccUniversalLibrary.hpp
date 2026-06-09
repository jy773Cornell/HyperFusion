#pragma once

#include "backend/LighthouseTypes.hpp"

#include <string>

namespace mcc
{
class MccUniversalLibrary
{
public:
    bool load(LighthouseError &error);
    void unload();
    bool isLoaded() const { return loaded_; }

    bool scanFor1208FsPlus(LighthouseDeviceInfo &deviceInfo, LighthouseError &error) const;
    bool configurePortAOutput(int boardNumber, LighthouseError &error) const;
    bool writeAnalogVolts(int boardNumber, int channel, float volts, LighthouseError &error) const;
    bool writeDigitalBit(int boardNumber, int portBit, bool high, LighthouseError &error) const;
    bool shutdownLighthouseOutputs(int boardNumber, LighthouseError &error) const;

private:
    // cbGetConfig(InfoType, BoardNum, DevNum, ConfigItem, ConfigVal*)
    using GetConfigFn = int (*)(int, int, int, int, int *);
    using GetErrMsgFn = int (*)(int, char *);
    using VOutFn = int (*)(int, int, int, float, int);
    using DBitOutFn = int (*)(int, int, int, unsigned short);
    using DConfigPortFn = int (*)(int, int, int);

    bool resolveSymbols(LighthouseError &error);
    std::string formatUlError(int errorCode) const;

    void *module_ = nullptr;
    GetConfigFn getConfig_ = nullptr;
    GetErrMsgFn getErrMsg_ = nullptr;
    VOutFn vOut_ = nullptr;
    DBitOutFn dBitOut_ = nullptr;
    DConfigPortFn dConfigPort_ = nullptr;
    bool loaded_ = false;
};
} // namespace mcc
