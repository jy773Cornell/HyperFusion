#include "adapters/mcc/MccUniversalLibrary.hpp"

#include "adapters/mcc/Mcc1208Profile.hpp"

#include <Windows.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <sstream>
#include <vector>

namespace
{
constexpr int kNoErrors = 0;
constexpr int kGlobalInfo = 1;
constexpr int kBoardInfo = 2;
constexpr int kGiNumBoards = 38;
constexpr int kBiBoardType = 1;

constexpr int kFirstPortA = 10;
constexpr int kDigitalOut = 1;

std::wstring defaultMccDllPath()
{
    std::vector<std::wstring> candidates;

    wchar_t modulePath[MAX_PATH] = {};
    if (GetModuleFileNameW(nullptr, modulePath, MAX_PATH) != 0)
    {
        std::wstring exeDir = modulePath;
        const std::size_t slash = exeDir.find_last_of(L"\\/");
        if (slash != std::wstring::npos)
            exeDir.resize(slash + 1);
        candidates.push_back(exeDir + L"cbw64.dll");
    }

    candidates.emplace_back(L"C:\\Program Files (x86)\\Measurement Computing\\DAQ\\cbw64.dll");
    candidates.emplace_back(L"C:\\Program Files\\Measurement Computing\\DAQ\\cbw64.dll");

    for (const std::wstring &path : candidates)
    {
        if (GetFileAttributesW(path.c_str()) != INVALID_FILE_ATTRIBUTES)
            return path;
    }

    return candidates.back();
}
} // namespace

namespace mcc
{
bool MccUniversalLibrary::load(LighthouseError &error)
{
    error = {};

    if (loaded_)
        return true;

    const std::wstring dllPath = defaultMccDllPath();
    module_ = LoadLibraryW(dllPath.c_str());
    if (module_ == nullptr)
    {
        error.code = LighthouseErrorCode::SdkError;
        error.message = "MCC Universal Library not found (cbw64.dll). Install MCC DAQ / InstaCal.";
        return false;
    }

    if (!resolveSymbols(error))
    {
        unload();
        return false;
    }

    loaded_ = true;
    return true;
}

void MccUniversalLibrary::unload()
{
    if (module_ != nullptr)
    {
        FreeLibrary(static_cast<HMODULE>(module_));
        module_ = nullptr;
    }

    getConfig_ = nullptr;
    getErrMsg_ = nullptr;
    vOut_ = nullptr;
    aIn_ = nullptr;
    toEngUnits_ = nullptr;
    aInputMode_ = nullptr;
    dBitOut_ = nullptr;
    dConfigPort_ = nullptr;
    loaded_ = false;
}

bool MccUniversalLibrary::resolveSymbols(LighthouseError &error)
{
    auto *const module = static_cast<HMODULE>(module_);
    getConfig_ = reinterpret_cast<GetConfigFn>(GetProcAddress(module, "cbGetConfig"));
    getErrMsg_ = reinterpret_cast<GetErrMsgFn>(GetProcAddress(module, "cbGetErrMsg"));
    vOut_ = reinterpret_cast<VOutFn>(GetProcAddress(module, "cbVOut"));
    aIn_ = reinterpret_cast<AInFn>(GetProcAddress(module, "cbAIn"));
    toEngUnits_ = reinterpret_cast<ToEngUnitsFn>(GetProcAddress(module, "cbToEngUnits"));
    aInputMode_ = reinterpret_cast<AInputModeFn>(GetProcAddress(module, "cbAInputMode"));
    dBitOut_ = reinterpret_cast<DBitOutFn>(GetProcAddress(module, "cbDBitOut"));
    dConfigPort_ = reinterpret_cast<DConfigPortFn>(GetProcAddress(module, "cbDConfigPort"));

    if (getConfig_ == nullptr || getErrMsg_ == nullptr || vOut_ == nullptr || aIn_ == nullptr
        || toEngUnits_ == nullptr || aInputMode_ == nullptr || dBitOut_ == nullptr
        || dConfigPort_ == nullptr)
    {
        error.code = LighthouseErrorCode::SdkError;
        error.message = "MCC Universal Library entry points missing in cbw64.dll.";
        return false;
    }

    return true;
}

std::string MccUniversalLibrary::formatUlError(const int errorCode) const
{
    if (getErrMsg_ == nullptr)
        return "MCC UL error " + std::to_string(errorCode);

    std::array<char, 256> buffer{};
    if (getErrMsg_(errorCode, buffer.data()) == kNoErrors)
        return std::string(buffer.data());

    return "MCC UL error " + std::to_string(errorCode);
}

bool MccUniversalLibrary::scanFor1208FsPlus(LighthouseDeviceInfo &deviceInfo,
                                            LighthouseError &error) const
{
    error = {};
    deviceInfo = {};

    if (!loaded_ || getConfig_ == nullptr)
    {
        error.code = LighthouseErrorCode::InvalidState;
        error.message = "MCC Universal Library is not loaded.";
        return false;
    }

    int maxBoards = kMaxConfiguredBoards;
    const int maxBoardsResult =
        getConfig_(kGlobalInfo, 0, 0, kGiNumBoards, &maxBoards);
    if (maxBoardsResult != kNoErrors)
        maxBoards = kMaxConfiguredBoards;

    maxBoards = std::clamp(maxBoards, 1, kMaxConfiguredBoards);

    for (int boardNumber = 0; boardNumber < maxBoards; ++boardNumber)
    {
        int boardType = 0;
        const int result =
            getConfig_(kBoardInfo, boardNumber, 0, kBiBoardType, &boardType);
        if (result != kNoErrors)
            continue;

        if (boardType != kUsb1208FsPlusBoardType)
            continue;

        deviceInfo.deviceName = "USB-1208FS-Plus";
        deviceInfo.boardNumber = boardNumber;
        std::ostringstream details;
        details << "USB-1208FS-Plus detected (InstaCal board " << boardNumber << ").\n"
                << lighthouseWiringDetailsText();
        deviceInfo.details = details.str();
        return true;
    }

    error.code = LighthouseErrorCode::DeviceNotFound;
    error.message =
        "USB-1208FS-Plus not found in InstaCal configuration. Open InstaCal and assign the device.";
    return false;
}

bool MccUniversalLibrary::configureAnalogInputSingleEnded(const int boardNumber,
                                                          LighthouseError &error) const
{
    error = {};
    if (!loaded_ || aInputMode_ == nullptr)
    {
        error.code = LighthouseErrorCode::InvalidState;
        error.message = "MCC Universal Library is not loaded.";
        return false;
    }

    const int result = aInputMode_(boardNumber, kAnalogInputModeSingleEnded);
    if (result != kNoErrors)
    {
        error.code = LighthouseErrorCode::SdkError;
        error.message = "cbAInputMode(SINGLE_ENDED): " + formatUlError(result);
        return false;
    }

    return true;
}

bool MccUniversalLibrary::configurePortAOutput(const int boardNumber, LighthouseError &error) const
{
    error = {};
    if (!loaded_ || dConfigPort_ == nullptr)
    {
        error.code = LighthouseErrorCode::InvalidState;
        error.message = "MCC Universal Library is not loaded.";
        return false;
    }

    const int result = dConfigPort_(boardNumber, kFirstPortA, kDigitalOut);
    if (result != kNoErrors)
    {
        error.code = LighthouseErrorCode::SdkError;
        error.message = "cbDConfigPort(FIRSTPORTA): " + formatUlError(result);
        return false;
    }

    return true;
}

bool MccUniversalLibrary::writeAnalogVolts(const int boardNumber,
                                           const int channel,
                                           const float volts,
                                           LighthouseError &error) const
{
    error = {};
    if (!loaded_ || vOut_ == nullptr)
    {
        error.code = LighthouseErrorCode::InvalidState;
        error.message = "MCC Universal Library is not loaded.";
        return false;
    }

    const float clamped = std::clamp(volts, 0.0f, kAnalogOutputVoltsMax);
    const int result = vOut_(boardNumber, channel, kAnalogOutputRangeUni5Volts, clamped, 0);
    if (result != kNoErrors)
    {
        error.code = LighthouseErrorCode::SdkError;
        error.message = "cbVOut: " + formatUlError(result);
        return false;
    }

    return true;
}

bool MccUniversalLibrary::readAnalogInputVolts(const int boardNumber,
                                             const int channel,
                                             float &volts,
                                             LighthouseError &error) const
{
    error = {};
    volts = 0.0f;

    if (!loaded_ || aIn_ == nullptr || toEngUnits_ == nullptr)
    {
        error.code = LighthouseErrorCode::InvalidState;
        error.message = "MCC Universal Library is not loaded.";
        return false;
    }

    if (channel < 0 || channel > 7)
    {
        error.code = LighthouseErrorCode::InternalError;
        error.message = "Invalid analog input channel index.";
        return false;
    }

    unsigned short rawValue = 0;
    const int readResult =
        aIn_(boardNumber, channel, kAnalogInputRangeSingleEnded10V, &rawValue);
    if (readResult != kNoErrors)
    {
        error.code = LighthouseErrorCode::SdkError;
        error.message = "cbAIn: " + formatUlError(readResult);
        return false;
    }

    float engineeringValue = 0.0f;
    const int convertResult =
        toEngUnits_(boardNumber, kAnalogInputRangeSingleEnded10V, rawValue, &engineeringValue);
    if (convertResult != kNoErrors)
    {
        error.code = LighthouseErrorCode::SdkError;
        error.message = "cbToEngUnits: " + formatUlError(convertResult);
        return false;
    }

    volts = engineeringValue;
    return true;
}

bool MccUniversalLibrary::writeDigitalBit(const int boardNumber,
                                          const int portBit,
                                          const bool high,
                                          LighthouseError &error) const
{
    error = {};
    if (!loaded_ || dBitOut_ == nullptr)
    {
        error.code = LighthouseErrorCode::InvalidState;
        error.message = "MCC Universal Library is not loaded.";
        return false;
    }

    if (portBit < 0 || portBit > 7)
    {
        error.code = LighthouseErrorCode::InternalError;
        error.message = "Invalid DIO port bit index.";
        return false;
    }

    const int result = dBitOut_(boardNumber, kFirstPortA, portBit, high ? 1 : 0);
    if (result != kNoErrors)
    {
        error.code = LighthouseErrorCode::SdkError;
        error.message = "cbDBitOut: " + formatUlError(result);
        return false;
    }

    return true;
}

bool MccUniversalLibrary::shutdownLighthouseOutputs(const int boardNumber,
                                                    LighthouseError &error) const
{
    for (int bit = 0; bit < kLighthouseLampCount; ++bit)
    {
        if (!writeDigitalBit(boardNumber, bit, lighthouseRelayOutputHigh(false), error))
            return false;
    }

    for (const int channel : {kLighthouseAnalogChannelReflectance, kLighthouseAnalogChannelTransmission})
    {
        if (!writeAnalogVolts(boardNumber, channel, 0.0f, error))
            return false;
    }

    return true;
}
} // namespace mcc
