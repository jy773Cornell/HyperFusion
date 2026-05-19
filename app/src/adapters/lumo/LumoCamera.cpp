// Lumo Sensor SDK camera adapter (real SDK when HF_HAVE_LUMO_SDK, else safe stub).
#include "adapters/lumo/LumoCamera.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <string>

#if defined(_WIN32)
#include <windows.h>
#endif

#if defined(HF_HAVE_LUMO_SDK)
#include "SI_errors.h"
#include "SI_sensor.h"
#include "SI_types.h"
#endif

namespace
{
constexpr int kStubFrameWidth = 320;
constexpr int kStubFrameHeight = 256;

#if defined(_WIN32)
std::wstring toWide(const std::string &utf8)
{
    if (utf8.empty())
        return {};

    const int required =
        MultiByteToWideChar(CP_UTF8, 0, utf8.c_str(), -1, nullptr, 0);
    if (required <= 1)
        return {};

    std::wstring wide(static_cast<std::size_t>(required - 1), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, utf8.c_str(), -1, wide.data(), required);
    return wide;
}

std::string wideToUtf8(const wchar_t *wide)
{
    if (wide == nullptr || wide[0] == L'\0')
        return {};

    const int required = WideCharToMultiByte(CP_UTF8, 0, wide, -1, nullptr, 0, nullptr, nullptr);
    if (required <= 1)
        return {};

    std::string utf8(static_cast<std::size_t>(required - 1), '\0');
    WideCharToMultiByte(CP_UTF8, 0, wide, -1, utf8.data(), required, nullptr, nullptr);
    return utf8;
}
#endif

#if defined(HF_HAVE_LUMO_SDK)
std::mutex g_sdkMutex;
int g_sdkLoadCount = 0;
std::atomic<int> g_openSensorHandles{0};
std::wstring g_pendingProfilesDirectory;

bool checkSiCode(const int code, const char *operation, CameraError &error)
{
    if (code == siNoError)
        return true;

    error.code = CameraErrorCode::SdkError;
    error.message = std::string(operation) + ": " + wideToUtf8(SI_GetErrorString(code));
    error.fatal = (code < 0);
    return false;
}

bool ensureGlobalSdkLoaded(const CameraSettings &prep, CameraError &error, bool &loadedHere)
{
    loadedHere = false;
    if (!prep.lumoProfilesDirectory.empty())
    {
        std::lock_guard<std::mutex> sdkLock(g_sdkMutex);
        g_pendingProfilesDirectory = toWide(prep.lumoProfilesDirectory);
    }

    std::lock_guard<std::mutex> sdkLock(g_sdkMutex);
    if (g_sdkLoadCount > 0)
    {
        loadedHere = false;
        return true;
    }

    if (!g_pendingProfilesDirectory.empty())
    {
        std::vector<SI_WC> profilesDir(g_pendingProfilesDirectory.begin(), g_pendingProfilesDirectory.end());
        profilesDir.push_back(L'\0');
        if (!checkSiCode(SI_SetString(SI_SYSTEM, L"ProfilesDirectory", profilesDir.data()),
                         "SI_SetString(ProfilesDirectory)",
                         error))
            return false;
    }

    const std::wstring licensePath = toWide(prep.lumoLicensePath);
    const SI_WC *licenseArg = licensePath.empty() ? L"" : licensePath.c_str();
    if (!checkSiCode(SI_Load(licenseArg), "SI_Load", error))
        return false;

    ++g_sdkLoadCount;
    loadedHere = true;
    return true;
}

void releaseGlobalSdkLoad(const bool loadedHere)
{
    if (!loadedHere)
        return;

    std::lock_guard<std::mutex> sdkLock(g_sdkMutex);
    if (g_sdkLoadCount <= 0)
        return;

    --g_sdkLoadCount;
    if (g_sdkLoadCount == 0)
        SI_Unload();
}

std::uint64_t steadyNowNs()
{
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
}
#endif
} // namespace

LumoCamera::LumoCamera(const CameraBackendId backendId, std::string instanceLabel)
    : backendId_(backendId),
      instanceLabel_(std::move(instanceLabel))
{
}

LumoCamera::~LumoCamera()
{
    disconnect();
}

std::string LumoCamera::tag() const
{
    return instanceLabel_;
}

void LumoCamera::prepareConnection(const CameraSettings &settings)
{
    std::lock_guard<std::mutex> lock(mutex_);
    settings_.deviceIndex = settings.deviceIndex;
    settings_.lumoLicensePath = settings.lumoLicensePath;
    settings_.lumoProfilesDirectory = settings.lumoProfilesDirectory;
    settings_.grabberChannel = settings.grabberChannel;
}

std::string LumoCamera::name() const
{
#if defined(HF_HAVE_LUMO_SDK)
    return instanceLabel_ + " (Lumo SDK)";
#else
    return instanceLabel_ + " (Lumo stub)";
#endif
}

CameraBackendId LumoCamera::backendId() const
{
    return backendId_;
}

namespace
{
const char *stateName(const CameraState state)
{
    switch (state)
    {
    case CameraState::Disconnected:
        return "Disconnected";
    case CameraState::Connected:
        return "Connected";
    case CameraState::Initialized:
        return "Initialized";
    case CameraState::Configured:
        return "Configured";
    case CameraState::Armed:
        return "Armed";
    case CameraState::Streaming:
        return "Streaming";
    case CameraState::SafeStopped:
        return "SafeStopped";
    case CameraState::Fault:
        return "Fault";
    default:
        return "Unknown";
    }
}
} // namespace

bool LumoCamera::expectState(CameraState expected, CameraError &error)
{
    error.code = CameraErrorCode::InvalidState;
    error.message = tag() + std::string(": expected ") + stateName(expected) + ", but camera is "
                      + stateName(state_) + ".";
    error.fatal = false;
    return false;
}

void LumoCamera::setFault(const CameraError &error)
{
    state_ = CameraState::Fault;
    (void)error;
}

#if defined(HF_HAVE_LUMO_SDK)

bool LumoCamera::enumerateDevices(const CameraSettings &prep,
                                  std::vector<LumoDeviceEntry> &devices,
                                  CameraError &error)
{
    devices.clear();

    if (g_openSensorHandles.load() > 0)
    {
        error.code = CameraErrorCode::InvalidState;
        error.message = "Cannot refresh SSP list while a camera is connected. Disconnect first.";
        error.fatal = false;
        return false;
    }

    bool loadedHere = false;
    if (!ensureGlobalSdkLoaded(prep, error, loadedHere))
        return false;

    SI_64 deviceCount = 0;
    if (!checkSiCode(SI_GetInt(SI_SYSTEM, L"DeviceCount", &deviceCount), "SI_GetInt(DeviceCount)", error))
    {
        releaseGlobalSdkLoad(loadedHere);
        return false;
    }

    for (int index = 0; index < static_cast<int>(deviceCount); ++index)
    {
        wchar_t nameBuffer[4096] = {};
        wchar_t descriptionBuffer[4096] = {};
        if (!checkSiCode(SI_GetEnumStringByIndex(SI_SYSTEM,
                                                 L"DeviceName",
                                                 index,
                                                 nameBuffer,
                                                 static_cast<int>(sizeof(nameBuffer) / sizeof(nameBuffer[0]))),
                         "SI_GetEnumStringByIndex(DeviceName)",
                         error))
        {
            releaseGlobalSdkLoad(loadedHere);
            return false;
        }

        if (!checkSiCode(SI_GetEnumStringByIndex(SI_SYSTEM,
                                                 L"DeviceDescription",
                                                 index,
                                                 descriptionBuffer,
                                                 static_cast<int>(sizeof(descriptionBuffer)
                                                                   / sizeof(descriptionBuffer[0]))),
                         "SI_GetEnumStringByIndex(DeviceDescription)",
                         error))
        {
            releaseGlobalSdkLoad(loadedHere);
            return false;
        }

        LumoDeviceEntry entry;
        entry.index = index;
        entry.name = wideToUtf8(nameBuffer);
        entry.description = wideToUtf8(descriptionBuffer);
        devices.push_back(std::move(entry));
    }

    releaseGlobalSdkLoad(loadedHere);
    return true;
}

bool LumoCamera::checkSi(const int code, const char *operation, CameraError &error)
{
    return checkSiCode(code, operation, error);
}

bool LumoCamera::ensureGrabberChannelSelected(void *const handlePtr, CameraError &error)
{
    SI_H handle = static_cast<SI_H>(handlePtr);

    auto readChannel = [&]() -> std::string {
        SI_BOOL readable = SI_FALSE;
        if (!SI_SUCCEEDED(SI_IsReadable(handle, L"Grabber.Channel", &readable)) || !readable)
            return {};

        int maxLength = 0;
        if (!SI_SUCCEEDED(SI_GetStringMaxLength(handle, L"Grabber.Channel", &maxLength)) || maxLength <= 0)
            return {};

        std::vector<wchar_t> buffer(static_cast<std::size_t>(maxLength) + 2U, L'\0');
        if (!SI_SUCCEEDED(SI_GetString(handle,
                                      L"Grabber.Channel",
                                      buffer.data(),
                                      static_cast<int>(buffer.size()))))
            return {};

        return wideToUtf8(buffer.data());
    };

    auto isImplemented = [&](const wchar_t *feature) -> bool {
        SI_BOOL implemented = SI_FALSE;
        return SI_SUCCEEDED(SI_IsImplemented(handle, feature, &implemented)) && implemented;
    };

    auto tryCommand = [&](const wchar_t *feature) -> bool {
        if (!isImplemented(feature))
            return false;
        return SI_Command(handle, feature) == siNoError;
    };

    auto applyChannelString = [&](const std::string &channel) -> bool {
        if (channel.empty())
            return false;

        const std::wstring channelWide = toWide(channel);
        std::vector<SI_WC> channelValue(channelWide.begin(), channelWide.end());
        channelValue.push_back(L'\0');
        return checkSi(SI_SetString(handle, L"Grabber.Channel", channelValue.data()),
                       "SI_SetString(Grabber.Channel)",
                       error);
    };

    if (!readChannel().empty())
        return true;

    if (!settings_.grabberChannel.empty())
    {
        if (applyChannelString(settings_.grabberChannel))
            return true;
        return false;
    }

    // Pleora/eBUS: executing Grabber.Channel as a command opens the native device picker on some profiles.
    static const wchar_t *kGrabberPickerCommands[] = {
        L"Grabber.Channel",
        L"Grabber.SelectChannel",
        L"Grabber.SelectDevice",
    };

    for (const wchar_t *command : kGrabberPickerCommands)
    {
        if (!tryCommand(command))
            continue;

        if (!readChannel().empty())
            return true;
    }

    if (isImplemented(L"Grabber.Channels"))
    {
        int channelCount = 0;
        if (checkSi(SI_GetEnumCount(handle, L"Grabber.Channels", &channelCount),
                    "SI_GetEnumCount(Grabber.Channels)",
                    error)
            && channelCount > 0)
        {
            wchar_t channelBuffer[4096] = {};
            if (checkSi(SI_GetEnumStringByIndex(handle,
                                                L"Grabber.Channels",
                                                0,
                                                channelBuffer,
                                                static_cast<int>(sizeof(channelBuffer) / sizeof(channelBuffer[0]))),
                        "SI_GetEnumStringByIndex(Grabber.Channels)",
                        error))
            {
                const std::string firstChannel = wideToUtf8(channelBuffer);
                if (applyChannelString(firstChannel))
                    return true;
            }
        }
    }

    error.code = CameraErrorCode::SdkError;
    error.message =
        tag()
        + ": Grabber.Channel is empty. For Pleora GigE, select a device in the eBUS picker (shown during "
          "connect), set HF_LUMO_GRABBER_CHANNEL to the device connection string, or install/configure "
          "Pleora eBUS SDK.";
    error.fatal = false;
    return false;
}

bool LumoCamera::ensureSdkLoaded(CameraError &error)
{
    std::lock_guard<std::mutex> sdkLock(g_sdkMutex);
    if (g_sdkLoadCount > 0)
    {
        ++g_sdkLoadCount;
        return true;
    }

    if (!g_pendingProfilesDirectory.empty())
    {
        std::vector<SI_WC> profilesDir(g_pendingProfilesDirectory.begin(), g_pendingProfilesDirectory.end());
        profilesDir.push_back(L'\0');
        if (!checkSiCode(SI_SetString(SI_SYSTEM, L"ProfilesDirectory", profilesDir.data()),
                         "SI_SetString(ProfilesDirectory)",
                         error))
            return false;
    }

    const std::wstring licensePath = toWide(settings_.lumoLicensePath);
    const SI_WC *licenseArg = licensePath.empty() ? L"" : licensePath.c_str();
    if (!checkSiCode(SI_Load(licenseArg), "SI_Load", error))
        return false;

    ++g_sdkLoadCount;
    return true;
}

void LumoCamera::releaseSdkLoad()
{
    std::lock_guard<std::mutex> sdkLock(g_sdkMutex);
    if (g_sdkLoadCount <= 0)
        return;

    --g_sdkLoadCount;
    if (g_sdkLoadCount == 0)
        SI_Unload();
}

bool LumoCamera::refreshImageGeometry(CameraError &error)
{
    SI_H handle = static_cast<SI_H>(handle_);
    SI_64 width = 0;
    SI_64 height = 0;
    SI_64 sizeBytes = 0;

    if (!checkSi(SI_GetInt(handle, L"Camera.Image.Width", &width), "SI_GetInt(Camera.Image.Width)", error))
        return false;
    if (!checkSi(SI_GetInt(handle, L"Camera.Image.Height", &height), "SI_GetInt(Camera.Image.Height)", error))
        return false;
    if (!checkSi(SI_GetInt(handle, L"Camera.Image.SizeBytes", &sizeBytes), "SI_GetInt(Camera.Image.SizeBytes)", error))
        return false;

    imageWidth_ = static_cast<int>(width);
    imageHeight_ = static_cast<int>(height);
    imageSizeBytes_ = sizeBytes;
    return imageWidth_ > 0 && imageHeight_ > 0 && imageSizeBytes_ > 0;
}

int lumoDataCallbackEntry(std::uint8_t *buffer,
                          const std::int64_t frameSize,
                          const std::int64_t frameNumber,
                          void *context)
{
    if (context == nullptr)
        return siNoError;

    static_cast<LumoCamera *>(context)->onFrame(buffer, frameSize, frameNumber);
    return siNoError;
}

namespace
{
int SI_IMPEXP_CONV lumoDataCallback(SI_U8 *buffer,
                                    const SI_64 frameSize,
                                    const SI_64 frameNumber,
                                    void *context)
{
    return lumoDataCallbackEntry(buffer, frameSize, frameNumber, context);
}
} // namespace

void LumoCamera::onFrame(const std::uint8_t *buffer,
                              const std::int64_t frameSize,
                              const std::int64_t frameNumber)
{
    if (buffer == nullptr || frameSize <= 0)
        return;

    std::lock_guard<std::mutex> lock(frameMutex_);
    if (static_cast<std::int64_t>(latestFrameBytes_.size()) < frameSize)
        latestFrameBytes_.resize(static_cast<std::size_t>(frameSize));

    std::memcpy(latestFrameBytes_.data(), buffer, static_cast<std::size_t>(frameSize));
    latestFrameNumber_ = frameNumber;
    frameReady_ = true;
    frameCv_.notify_all();
}

bool LumoCamera::registerDataCallback(CameraError &error)
{
    if (callbackRegistered_)
        return true;

    SI_H handle = static_cast<SI_H>(handle_);
    if (!checkSi(SI_RegisterDataCallback(handle, lumoDataCallback, this),
                 "SI_RegisterDataCallback",
                 error))
        return false;

    callbackRegistered_ = true;
    return true;
}

void LumoCamera::unregisterDataCallback()
{
    if (!callbackRegistered_)
        return;

    SI_H handle = static_cast<SI_H>(handle_);
    SI_UnregisterDataCallback(handle);
    callbackRegistered_ = false;
}

bool LumoCamera::connect(CameraError &error)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (state_ == CameraState::Fault)
        return expectState(CameraState::Disconnected, error);

    if (state_ == CameraState::Streaming)
    {
        error.code = CameraErrorCode::InvalidState;
        error.message = tag() + " is streaming. Stop preview before reconnecting.";
        error.fatal = false;
        return false;
    }

    if (state_ == CameraState::SafeStopped && handle_ != nullptr)
    {
        state_ = CameraState::Configured;
        return true;
    }

    if (state_ == CameraState::Connected || state_ == CameraState::Initialized
        || state_ == CameraState::Configured || state_ == CameraState::Armed)
        return true;

    if (state_ != CameraState::Disconnected)
        return expectState(CameraState::Disconnected, error);

    if (!settings_.lumoProfilesDirectory.empty())
    {
        std::lock_guard<std::mutex> sdkLock(g_sdkMutex);
        g_pendingProfilesDirectory = toWide(settings_.lumoProfilesDirectory);
    }

    if (!ensureSdkLoaded(error))
    {
        setFault(error);
        return false;
    }

    SI_H handle = 0;
    if (!checkSi(SI_Open(settings_.deviceIndex, &handle), "SI_Open", error))
    {
        releaseSdkLoad();
        setFault(error);
        return false;
    }

    handle_ = handle;
    ++g_openSensorHandles;
    state_ = CameraState::Connected;
    return true;
}

bool LumoCamera::initialize(CameraError &error)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (state_ == CameraState::Streaming)
    {
        error.code = CameraErrorCode::InvalidState;
        error.message = tag() + " is streaming. Stop preview before re-initializing.";
        error.fatal = false;
        return false;
    }

    if (state_ == CameraState::Initialized || state_ == CameraState::Configured
        || state_ == CameraState::Armed || state_ == CameraState::SafeStopped)
        return true;

    if (state_ != CameraState::Connected)
        return expectState(CameraState::Connected, error);

    SI_H handle = static_cast<SI_H>(handle_);
    if (!ensureGrabberChannelSelected(handle, error))
    {
        setFault(error);
        return false;
    }

    if (!checkSi(SI_Command(handle, L"Initialize"), "SI_Command(Initialize)", error))
    {
        setFault(error);
        return false;
    }

    if (!refreshImageGeometry(error))
    {
        setFault(error);
        return false;
    }

    state_ = CameraState::Initialized;
    return true;
}

bool LumoCamera::applySettings(const CameraSettings &settings, CameraError &error)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != CameraState::Initialized && state_ != CameraState::Configured
        && state_ != CameraState::SafeStopped)
        return expectState(CameraState::Initialized, error);

    settings_ = settings;
    SI_H handle = static_cast<SI_H>(handle_);

    if (!checkSi(SI_SetFloat(handle, L"Camera.ExposureTime", settings_.exposureMs),
                 "SI_SetFloat(Camera.ExposureTime)",
                 error))
        return false;

    if (!checkSi(SI_SetFloat(handle, L"Camera.FrameRate", settings_.frameRateHz),
                 "SI_SetFloat(Camera.FrameRate)",
                 error))
        return false;

    const wchar_t *triggerMode = settings_.externalTrigger ? L"External" : L"Internal";
    if (!checkSi(SI_SetEnumIndexByString(handle, L"Camera.Trigger.Mode", triggerMode),
                 "SI_SetEnumIndexByString(Camera.Trigger.Mode)",
                 error))
        return false;

    const double timeoutMs = static_cast<double>(settings_.acquisitionTimeoutMs);
    if (!checkSi(SI_SetFloat(handle, L"Acquisition.Timeout", timeoutMs),
                 "SI_SetFloat(Acquisition.Timeout)",
                 error))
        return false;

    if (!refreshImageGeometry(error))
        return false;

    state_ = CameraState::Configured;
    return true;
}

bool LumoCamera::arm(CameraError &error)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != CameraState::Configured && state_ != CameraState::SafeStopped)
        return expectState(CameraState::Configured, error);

    {
        std::lock_guard<std::mutex> frameLock(frameMutex_);
        frameReady_ = false;
        latestFrameNumber_ = 0;
        if (imageSizeBytes_ > 0)
            latestFrameBytes_.assign(static_cast<std::size_t>(imageSizeBytes_), 0);
    }

    if (!registerDataCallback(error))
        return false;

    state_ = CameraState::Armed;
    return true;
}

bool LumoCamera::start(CameraError &error)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != CameraState::Armed)
        return expectState(CameraState::Armed, error);

    SI_H handle = static_cast<SI_H>(handle_);
    if (!checkSi(SI_Command(handle, L"Acquisition.Start"), "SI_Command(Acquisition.Start)", error))
    {
        setFault(error);
        return false;
    }

    state_ = CameraState::Streaming;
    return true;
}

void LumoCamera::stop()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != CameraState::Streaming)
        return;

    SI_H handle = static_cast<SI_H>(handle_);
    SI_Command(handle, L"Acquisition.Stop");
    unregisterDataCallback();

    {
        std::lock_guard<std::mutex> frameLock(frameMutex_);
        frameReady_ = false;
    }

    state_ = CameraState::SafeStopped;
}

void LumoCamera::disconnect()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == CameraState::Disconnected)
        return;

    if (state_ == CameraState::Streaming)
    {
        SI_H handle = static_cast<SI_H>(handle_);
        SI_Command(handle, L"Acquisition.Stop");
    }

    unregisterDataCallback();

    if (handle_ != nullptr)
    {
        SI_Close(static_cast<SI_H>(handle_));
        handle_ = nullptr;
        --g_openSensorHandles;
    }

    releaseSdkLoad();
    state_ = CameraState::Disconnected;
}

bool LumoCamera::pollFrame(FramePacket &frame, const std::uint32_t timeoutMs, CameraError &error)
{
    CameraState localState;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        localState = state_;
    }

    if (localState != CameraState::Streaming)
    {
        error.code = CameraErrorCode::InvalidState;
        error.message = tag() + " stream not active.";
        error.fatal = false;
        return false;
    }

    std::unique_lock<std::mutex> frameLock(frameMutex_);
  const bool signaled = frameCv_.wait_for(
        frameLock, std::chrono::milliseconds(timeoutMs), [this]() { return frameReady_; });

    if (!signaled)
    {
        error.code = CameraErrorCode::Timeout;
        error.message = tag() + " frame timeout.";
        error.fatal = false;
        return false;
    }

    const std::size_t pixelCount =
        static_cast<std::size_t>(std::max<std::int64_t>(0, imageWidth_ * imageHeight_));
    frame.source = backendId_;
    frame.frameIndex = static_cast<std::uint64_t>(latestFrameNumber_);
    frame.hostTimestampNs = steadyNowNs();
    frame.width = imageWidth_;
    frame.height = imageHeight_;

    if (pixelCount == 0)
    {
        error.code = CameraErrorCode::InternalError;
        error.message = tag() + " image geometry not available.";
        error.fatal = true;
        return false;
    }

    const std::size_t bytesNeeded = pixelCount * sizeof(std::uint16_t);
    if (latestFrameBytes_.size() < bytesNeeded)
    {
        error.code = CameraErrorCode::InternalError;
        error.message = tag() + " frame buffer smaller than expected image.";
        error.fatal = true;
        return false;
    }

    frame.pixels.resize(pixelCount);
    std::memcpy(frame.pixels.data(),
                latestFrameBytes_.data(),
                bytesNeeded);
    frameReady_ = false;
    return true;
}

#else // !HF_HAVE_LUMO_SDK

bool LumoCamera::enumerateDevices(const CameraSettings &prep,
                                         std::vector<LumoDeviceEntry> &devices,
                                         CameraError &error)
{
    (void)prep;
    devices.clear();
    error.code = CameraErrorCode::NotImplemented;
    error.message = tag() + " Lumo SDK not available.";
    error.fatal = false;
    return false;
}

bool LumoCamera::connect(CameraError &error)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != CameraState::Disconnected && state_ != CameraState::SafeStopped)
        return expectState(CameraState::Disconnected, error);

    error.code = CameraErrorCode::NotImplemented;
    error.message = tag()
                      + " Lumo SDK not linked. Set LUMO_SDK_ROOT and rebuild with SpecSensor.lib.";
    error.fatal = false;
    return false;
}

bool LumoCamera::initialize(CameraError &error)
{
    (void)error;
    return false;
}

bool LumoCamera::applySettings(const CameraSettings &settings, CameraError &error)
{
    (void)settings;
    (void)error;
    return false;
}

bool LumoCamera::arm(CameraError &error)
{
    (void)error;
    return false;
}

bool LumoCamera::start(CameraError &error)
{
    (void)error;
    return false;
}

void LumoCamera::stop() {}

void LumoCamera::disconnect()
{
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = CameraState::Disconnected;
}

bool LumoCamera::pollFrame(FramePacket &frame, const std::uint32_t timeoutMs, CameraError &error)
{
    (void)frame;
    (void)timeoutMs;
    error.code = CameraErrorCode::NotImplemented;
    error.message = tag() + " Lumo SDK not available.";
    error.fatal = false;
    return false;
}

#endif

CameraState LumoCamera::state() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}
