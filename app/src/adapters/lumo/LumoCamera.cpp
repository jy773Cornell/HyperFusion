// Lumo Sensor SDK camera adapter (real SDK when HF_HAVE_LUMO_SDK, else safe stub).
#include "adapters/lumo/LumoCamera.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <string>

#if defined(_WIN32)
#include <windows.h>
#endif

#if defined(HF_HAVE_LUMO_SDK)
#include "SI_errors.h"
#include "SI_sensor.h"
#include "SI_types.h"
#include "backend/HyperFusionConfig.hpp"
#include "backend/processing/SwirBprCorrector.hpp"
#include "backend/processing/SwirColumnProfileCorrector.hpp"

#include <QString>
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
/// Serializes SpecSensor SI_* calls across all LumoCamera instances (FX10e + SWIR3 dual connect).
std::recursive_mutex g_lumoGlobalMutex;
int g_sdkLoadCount = 0;
std::atomic<int> g_openSensorHandles{0};

struct LumoGlobalLock
{
    LumoGlobalLock() { g_lumoGlobalMutex.lock(); }
    ~LumoGlobalLock() { g_lumoGlobalMutex.unlock(); }
};

struct LumoGlobalTryLock
{
    bool locked = false;

    LumoGlobalTryLock() { locked = g_lumoGlobalMutex.try_lock(); }
    ~LumoGlobalTryLock()
    {
        if (locked)
            g_lumoGlobalMutex.unlock();
    }

    explicit operator bool() const { return locked; }
};

bool checkSiCode(const int code, const char *operation, CameraError &error)
{
    if (code == siNoError)
        return true;

    error.code = CameraErrorCode::SdkError;
    error.message = std::string(operation) + ": " + wideToUtf8(SI_GetErrorString(code));
    if (code != siNoError)
        error.message += " (" + std::to_string(code) + ")";
    error.fatal = (code < 0);
    return false;
}

bool setAcquisitionTimeoutMs(const SI_H handle,
                             const std::uint32_t timeoutMs,
                             CameraError &error)
{
    SI_BOOL implemented = SI_FALSE;
    SI_BOOL writable = SI_FALSE;
    if (!SI_SUCCEEDED(SI_IsImplemented(handle, L"Acquisition.Timeout", &implemented)) || !implemented
        || !SI_SUCCEEDED(SI_IsWritable(handle, L"Acquisition.Timeout", &writable)) || !writable)
        return true;

    return checkSiCode(SI_SetFloat(handle, L"Acquisition.Timeout", static_cast<double>(timeoutMs)),
                       "SI_SetFloat(Acquisition.Timeout)",
                       error);
}

std::string lumoSdkRootPath()
{
#ifdef HF_LUMO_SDK_ROOT
    return HF_LUMO_SDK_ROOT;
#else
    return "C:/Program Files (x86)/Specim/SDKs/SpecSensor/2020_519";
#endif
}

bool setHandleStringFeature(const SI_H handle,
                            const wchar_t *feature,
                            const std::wstring &value,
                            CameraError &error,
                            const bool required)
{
    if (value.empty())
        return true;

    SI_BOOL implemented = SI_FALSE;
    SI_BOOL writable = SI_FALSE;
    if (!SI_SUCCEEDED(SI_IsImplemented(handle, feature, &implemented)) || !implemented
        || !SI_SUCCEEDED(SI_IsWritable(handle, feature, &writable)) || !writable)
    {
        if (required)
        {
            error.code = CameraErrorCode::SdkError;
            error.message = std::string("Feature not writable: ") + wideToUtf8(feature);
            error.fatal = false;
            return false;
        }
        return true;
    }

    std::vector<SI_WC> buffer(value.begin(), value.end());
    buffer.push_back(L'\0');
    const std::string operation = std::string("SI_SetString(") + wideToUtf8(feature) + ")";
    return checkSiCode(SI_SetString(handle, feature, buffer.data()), operation.c_str(), error);
}

std::string getHandleStringFeature(const SI_H handle, const wchar_t *feature)
{
    int maxLength = 0;
    if (!SI_SUCCEEDED(SI_GetStringMaxLength(handle, feature, &maxLength)) || maxLength <= 0)
        return {};

    std::vector<wchar_t> buffer(static_cast<std::size_t>(maxLength) + 1, L'\0');
    if (!SI_SUCCEEDED(SI_GetString(handle,
                                   feature,
                                   buffer.data(),
                                   static_cast<int>(buffer.size() * sizeof(wchar_t)))))
        return {};

    return wideToUtf8(buffer.data());
}

std::string enumerateHandleFeatureStrings(const SI_H handle, const wchar_t *feature)
{
    int count = 0;
    if (!SI_SUCCEEDED(SI_GetEnumCount(handle, feature, &count)) || count <= 0)
        return {};

    std::string joined;
    for (int index = 0; index < count; ++index)
    {
        wchar_t buffer[512] = {};
        if (!SI_SUCCEEDED(SI_GetEnumStringByIndex(handle,
                                                feature,
                                                index,
                                                buffer,
                                                static_cast<int>(sizeof(buffer) / sizeof(buffer[0])))))
            continue;

        if (!joined.empty())
            joined += ", ";
        joined += wideToUtf8(buffer);
    }
    return joined;
}

bool readHandleBoolFeature(const SI_H handle, const wchar_t *feature, SI_BOOL &value)
{
    SI_BOOL implemented = SI_FALSE;
    if (!SI_SUCCEEDED(SI_IsImplemented(handle, feature, &implemented)) || !implemented)
        return false;

    return SI_SUCCEEDED(SI_GetBool(handle, feature, &value));
}

bool readSdkBprEnabled(const SI_H handle, bool &enabledOut)
{
    SI_BOOL enabled = SI_FALSE;
    if (!readHandleBoolFeature(handle, L"Camera.BPR", enabled))
        return false;

    enabledOut = (enabled == SI_TRUE);
    return true;
}

bool enableSdkBpr(const SI_H handle, CameraError &error)
{
    SI_BOOL bprImplemented = SI_FALSE;
    if (!SI_SUCCEEDED(SI_IsImplemented(handle, L"Camera.BPR", &bprImplemented)) || !bprImplemented)
        return true;

    SI_BOOL writable = SI_FALSE;
    if (!SI_SUCCEEDED(SI_IsWritable(handle, L"Camera.BPR", &writable)) || !writable)
        return true;

    if (!checkSiCode(SI_SetBool(handle, L"Camera.BPR", SI_TRUE), "SI_SetBool(Camera.BPR,true)", error))
        return false;

    bool enabled = false;
    if (readSdkBprEnabled(handle, enabled) && !enabled)
    {
        error.code = CameraErrorCode::SdkError;
        error.message = "Camera.BPR read back false after enable.";
        error.fatal = false;
        return false;
    }

    return true;
}

bool disableSdkBpr(const SI_H handle, CameraError &error)
{
    SI_BOOL bprImplemented = SI_FALSE;
    if (!SI_SUCCEEDED(SI_IsImplemented(handle, L"Camera.BPR", &bprImplemented)) || !bprImplemented)
        return true;

    SI_BOOL writable = SI_FALSE;
    if (!SI_SUCCEEDED(SI_IsWritable(handle, L"Camera.BPR", &writable)) || !writable)
        return true;

    if (!checkSiCode(SI_SetBool(handle, L"Camera.BPR", SI_FALSE), "SI_SetBool(Camera.BPR,false)", error))
        return false;

    bool enabled = true;
    if (readSdkBprEnabled(handle, enabled) && enabled)
    {
        error.code = CameraErrorCode::SdkError;
        error.message = "Camera.BPR read back true after disable.";
        error.fatal = false;
        return false;
    }

    return true;
}

bool verifyCalpackLoadedAfterInitialize(const SI_H handle, CameraError &error)
{
    SI_BOOL calpackLoaded = SI_FALSE;
    if (!readHandleBoolFeature(handle, L"Camera.CalibrationPack.IsLoaded", calpackLoaded))
        return true;

    if (calpackLoaded == SI_TRUE)
        return true;

    error.code = CameraErrorCode::SdkError;
    error.message =
        "Camera.CalibrationPack.IsLoaded is false after Initialize (BPR/radiometric not active).";
    error.fatal = false;
    return false;
}

std::string formatHandleBoolFeature(const SI_H handle, const wchar_t *feature)
{
    SI_BOOL value = SI_FALSE;
    if (!readHandleBoolFeature(handle, feature, value))
        return "(n/a)";

    return value == SI_TRUE ? "true" : "false";
}

std::string formatHandleIntFeature(const SI_H handle, const wchar_t *feature)
{
    SI_BOOL implemented = SI_FALSE;
    if (!SI_SUCCEEDED(SI_IsImplemented(handle, feature, &implemented)) || !implemented)
        return "(n/a)";

    SI_64 value = 0;
    if (!SI_SUCCEEDED(SI_GetInt(handle, feature, &value)))
        return "(n/a)";

    return std::to_string(static_cast<long long>(value));
}

std::string formatHandleFloatFeature(const SI_H handle, const wchar_t *feature)
{
    SI_BOOL implemented = SI_FALSE;
    if (!SI_SUCCEEDED(SI_IsImplemented(handle, feature, &implemented)) || !implemented)
        return "(n/a)";

    double value = 0.0;
    if (!SI_SUCCEEDED(SI_GetFloat(handle, feature, &value)))
        return "(n/a)";

    return std::to_string(value);
}

std::string formatHandleEnumFeature(const SI_H handle, const wchar_t *feature)
{
    SI_BOOL implemented = SI_FALSE;
    if (!SI_SUCCEEDED(SI_IsImplemented(handle, feature, &implemented)) || !implemented)
        return "(n/a)";

    int index = 0;
    if (!SI_SUCCEEDED(SI_GetEnumIndex(handle, feature, &index)))
        return "(n/a)";

    wchar_t buffer[512] = {};
    if (!SI_SUCCEEDED(SI_GetEnumStringByIndex(handle,
                                              feature,
                                              index,
                                              buffer,
                                              static_cast<int>(sizeof(buffer) / sizeof(buffer[0])))))
        return "(n/a)";

    return wideToUtf8(buffer);
}

bool applySwirNiSetupBeforeInitialize(const SI_H handle,
                                      const CameraSettings &settings,
                                      CameraError &error)
{
    const std::string icdPath = LumoCamera::resolveNiImaqCameraFilePath(settings.niImaqCameraFile);
    if (!settings.niImaqCameraFile.empty() && !std::filesystem::exists(icdPath))
    {
        error.code = CameraErrorCode::SdkError;
        error.message = "NiImaq.CameraFile not found: " + icdPath
                        + " (install SpecSensor SDK or copy external/NI beside app.exe).";
        error.fatal = false;
        return false;
    }

    if (!setHandleStringFeature(handle,
                                L"NiImaq.CameraFile",
                                toWide(icdPath),
                                error,
                                true))
        return false;

    if (!setHandleStringFeature(handle,
                                L"Grabber.Channel",
                                toWide(settings.niGrabberChannel),
                                error,
                                true))
        return false;

    // Camera.Channel is cam007 serial (AIM SWIR head), not the IMAQdx grabber name. Scb.Channel is
    // left to the SDK \u2014 HyperFusion does not set it (wrong COM causes Initialize SCB Read / -1301).
    if (!setHandleStringFeature(handle,
                                L"Camera.Channel",
                                toWide(settings.niCameraSerialPort),
                                error,
                                false))
        return false;

    return true;
}

bool ensureGlobalSdkLoaded(const CameraSettings &prep, CameraError &error, bool &loadedHere)
{
    loadedHere = false;
    LumoGlobalLock lumoApi;
    if (g_sdkLoadCount > 0)
    {
        loadedHere = false;
        return true;
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

    LumoGlobalLock lumoApi;
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

std::string LumoCamera::resolveNiImaqCameraFilePath(const std::string &fileName)
{
    if (fileName.empty())
        return {};

#ifdef HF_LUMO_SDK_ROOT
    const std::string sdkRoot = HF_LUMO_SDK_ROOT;
#else
    const std::string sdkRoot = "C:/Program Files (x86)/Specim/SDKs/SpecSensor/2020_519";
#endif

    const std::filesystem::path direct(fileName);
    if (direct.is_absolute() && std::filesystem::exists(direct))
        return direct.string();

    const std::filesystem::path sdkPath =
        std::filesystem::path(sdkRoot) / "external" / "NI" / fileName;
    if (std::filesystem::exists(sdkPath))
        return sdkPath.string();

    const std::filesystem::path localPath =
        std::filesystem::path("external") / "NI" / fileName;
    if (std::filesystem::exists(localPath))
        return std::filesystem::absolute(localPath).string();

    return sdkPath.string();
}

std::string LumoCamera::swirNiConnectionSummary() const
{
#if defined(HF_HAVE_LUMO_SDK)
    LumoGlobalLock lumoApi;
    std::lock_guard<std::mutex> lock(mutex_);
    if (sensorKind_ != LumoSensorKind::Swir3Ni || handle_ == nullptr)
        return {};

    const SI_H handle = static_cast<SI_H>(handle_);
    const auto readFeature = [handle](const wchar_t *feature) -> std::string {
        const std::string value = getHandleStringFeature(handle, feature);
        return value.empty() ? std::string("(none)") : value;
    };

    return "Grabber.Channel=" + readFeature(L"Grabber.Channel") + ", Camera.Channel="
           + readFeature(L"Camera.Channel") + ", Scb.Channel=" + readFeature(L"Scb.Channel")
           + ", CalibrationPack.IsLoaded=" + formatHandleBoolFeature(handle, L"Camera.CalibrationPack.IsLoaded")
           + ", BPR.MapAvailable=" + formatHandleBoolFeature(handle, L"Camera.BPR.MapAvailable")
           + ", BPR=" + formatHandleBoolFeature(handle, L"Camera.BPR")
           + ", BPR.BadPixelCount=" + formatHandleIntFeature(handle, L"Camera.BPR.BadPixelCount")
           + (bprStatusSummary_.empty() ? std::string() : std::string(", ") + bprStatusSummary_)
           + (nucStatusSummary_.empty() ? std::string() : std::string(", ") + nucStatusSummary_);
#else
    return {};
#endif
}

bool LumoCamera::readAppliedFrameRateHz(double &outHz, CameraError &error)
{
#if defined(HF_HAVE_LUMO_SDK)
    LumoGlobalTryLock lumoApi;
    if (!lumoApi)
    {
        error.code = CameraErrorCode::Timeout;
        error.message = tag() + " Lumo SDK busy (connect/init in progress).";
        error.fatal = false;
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (handle_ == nullptr)
    {
        error.code = CameraErrorCode::InvalidState;
        error.message = tag() + " sensor handle not open.";
        error.fatal = false;
        return false;
    }

    if (state_ != CameraState::Initialized && state_ != CameraState::Configured
        && state_ != CameraState::Armed && state_ != CameraState::Streaming
        && state_ != CameraState::SafeStopped)
    {
        error.code = CameraErrorCode::InvalidState;
        error.message = tag() + " sensor not initialized.";
        error.fatal = false;
        return false;
    }

    const SI_H handle = static_cast<SI_H>(handle_);
    double hz = settings_.frameRateHz;
    if (!checkSi(SI_GetFloat(handle, L"Camera.FrameRate", &hz),
                 "SI_GetFloat(Camera.FrameRate)",
                 error))
        return false;

    outHz = hz;
    return true;
#else
    (void)error;
    outHz = settings_.frameRateHz;
    return handle_ != nullptr;
#endif
}

LumoCamera::LumoCamera(const CameraBackendId backendId,
                       std::string instanceLabel,
                       const LumoSensorKind sensorKind)
    : backendId_(backendId),
      instanceLabel_(std::move(instanceLabel)),
      sensorKind_(sensorKind)
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
    settings_.lumoCalibrationPackPath = settings.lumoCalibrationPackPath;
    settings_.profileName = settings.profileName;
    settings_.acquisitionTimeoutMs = settings.acquisitionTimeoutMs;
    settings_.niGrabberChannel = settings.niGrabberChannel;
    settings_.niImaqCameraFile = settings.niImaqCameraFile;
    settings_.niCameraSerialPort = settings.niCameraSerialPort;
}

std::string LumoCamera::name() const
{
#if defined(HF_HAVE_LUMO_SDK)
    if (sensorKind_ == LumoSensorKind::Swir3Ni)
        return instanceLabel_ + " (Lumo SDK / NI)";
    return instanceLabel_ + " (Lumo SDK / Pleora)";
#else
    if (sensorKind_ == LumoSensorKind::Swir3Ni)
        return instanceLabel_ + " (Lumo stub / NI)";
    return instanceLabel_ + " (Lumo stub / Pleora)";
#endif
}

CameraBackendId LumoCamera::backendId() const
{
    return backendId_;
}

LumoSensorKind LumoCamera::sensorKind() const
{
    return sensorKind_;
}

bool LumoCamera::requiresGuiThreadForSdkLifecycle() const
{
    return true;
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
    LumoGlobalLock lumoApi;
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

void LumoCamera::rollbackOpenConnection()
{
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

bool LumoCamera::ensureSdkLoaded(CameraError &error)
{
    LumoGlobalLock lumoApi;
    if (g_sdkLoadCount > 0)
    {
        ++g_sdkLoadCount;
        return true;
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
    LumoGlobalLock lumoApi;
    if (g_sdkLoadCount <= 0)
        return;

    --g_sdkLoadCount;
    if (g_sdkLoadCount == 0 && g_openSensorHandles.load() == 0)
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

    {
        std::lock_guard<std::mutex> stateLock(mutex_);
        if (state_ != CameraState::Streaming && state_ != CameraState::Armed)
            return;
    }

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
    LumoGlobalLock lumoApi;
    std::lock_guard<std::mutex> lock(mutex_);

    if (state_ == CameraState::Fault)
        state_ = CameraState::Disconnected;

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

    if (!ensureSdkLoaded(error))
        return false;

    SI_H handle = 0;
    if (!checkSi(SI_Open(settings_.deviceIndex, &handle), "SI_Open", error))
    {
        releaseSdkLoad();
        if (error.message.find("Loading the module failed") != std::string::npos)
        {
            if (sensorKind_ == LumoSensorKind::Swir3Ni)
            {
                error.message +=
                    " Ensure SpecSensor and NI IMAQ/IMAQdx runtime folders are on PATH (SDK bin\\x64, NI Vision).";
            }
            else
            {
                error.message += " Ensure SpecSensor and Pleora/eBUS DLL folders are on PATH (SDK bin\\x64).";
            }
        }
        return false;
    }

    handle_ = handle;
    ++g_openSensorHandles;
    state_ = CameraState::Connected;
    return true;
}

bool LumoCamera::initialize(CameraError &error)
{
    LumoGlobalLock lumoApi;
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

    wchar_t deviceName[4096] = {};
    const bool hasDeviceName =
        SI_SUCCEEDED(SI_GetEnumStringByIndex(SI_SYSTEM,
                                             L"DeviceName",
                                             settings_.deviceIndex,
                                             deviceName,
                                             static_cast<int>(sizeof(deviceName) / sizeof(deviceName[0]))));
    const std::string deviceNameUtf8 = hasDeviceName ? wideToUtf8(deviceName) : std::string();
    if (sensorKind_ == LumoSensorKind::Swir3Ni)
    {
        if (!applySwirNiSetupBeforeInitialize(handle, settings_, error))
        {
            rollbackOpenConnection();
            return false;
        }
    }

    if (sensorKind_ == LumoSensorKind::Swir3Ni && settings_.lumoCalibrationPackPath.empty())
    {
        error.code = CameraErrorCode::SdkError;
        error.message = tag() + ": SWIR3 requires a calibration pack (.scp) before Initialize.";
        error.fatal = false;
        rollbackOpenConnection();
        return false;
    }

    if (!settings_.lumoCalibrationPackPath.empty())
    {
        const std::filesystem::path calpackPath(settings_.lumoCalibrationPackPath);
        if (!std::filesystem::exists(calpackPath))
        {
            error.code = CameraErrorCode::SdkError;
            error.message = tag() + ": calibration pack not found: " + settings_.lumoCalibrationPackPath
                            + " (BPR/radiometric corrections require a valid .scp before Initialize).";
            error.fatal = false;
            rollbackOpenConnection();
            return false;
        }

        const std::wstring calibWide = toWide(settings_.lumoCalibrationPackPath);
        std::vector<SI_WC> calibValue(calibWide.begin(), calibWide.end());
        calibValue.push_back(L'\0');
        if (!checkSi(SI_SetString(handle, L"Camera.CalibrationPack", calibValue.data()),
                     "SI_SetString(Camera.CalibrationPack)",
                     error))
        {
            rollbackOpenConnection();
            return false;
        }
    }

    const std::uint32_t initTimeoutMs =
        settings_.acquisitionTimeoutMs > 0 ? settings_.acquisitionTimeoutMs
                                           : (sensorKind_ == LumoSensorKind::Swir3Ni ? 30000U : 5000U);
    if (!setAcquisitionTimeoutMs(handle, initTimeoutMs, error))
    {
        rollbackOpenConnection();
        return false;
    }

    if (!checkSi(SI_Command(handle, L"Initialize"), "SI_Command(Initialize)", error))
    {
        if (error.message.find("Invalid camera channel") != std::string::npos)
        {
            error.message =
                tag()
                + ": Initialize failed (invalid grabber channel). Pick another SSP profile from the list.";
        }
        else if (error.message.find("BFF6902C") != std::string::npos
                 || error.message.find("Unable to connect to the camera") != std::string::npos
                 || error.message.find("IMAQdx") != std::string::npos)
        {
            error.message +=
                " NI IMAQdx could not open the camera \u2014 fix in NI MAX first (Snap/Grab on the IMAQdx device): "
                "GigE: camera and NIC on same subnet, use NI GigE Vision driver; "
                "Camera Link: power (PoCL), Base/Medium cable orientation, frame grabber in MAX. "
                "Close other apps using the camera, then retry.";
        }
        else if (sensorKind_ == LumoSensorKind::Swir3Ni
                 && (error.message.find("Communication timeout") != std::string::npos
                     || error.message.find("-1101") != std::string::npos))
        {
            const std::string grabberOptions =
                enumerateHandleFeatureStrings(handle, L"Grabber.Channel");
            const std::string icdPath = LumoCamera::resolveNiImaqCameraFilePath(settings_.niImaqCameraFile);
            const std::string grabberReadback = getHandleStringFeature(handle, L"Grabber.Channel");
            const std::string icdReadback = getHandleStringFeature(handle, L"NiImaq.CameraFile");
            error.message +=
                " Specim -1101: cam007 OpenSerialPort (Camera.Channel / AIM SWIR), not PCIe-1433. Close NI MAX Grab first. ";
            error.message += "Grabber.Channel=" + (settings_.niGrabberChannel.empty()
                                                       ? std::string("(not set)")
                                                       : settings_.niGrabberChannel);
            if (!grabberReadback.empty())
                error.message += " (readback: " + grabberReadback + ")";
            error.message += ", NiImaq.CameraFile=" + icdPath;
            if (!icdReadback.empty() && icdReadback != icdPath)
                error.message += " (readback: " + icdReadback + ")";
            if (!settings_.niCameraSerialPort.empty())
                error.message += ", Camera.Channel=" + settings_.niCameraSerialPort;
            else
                error.message += ", Camera.Channel=(not set)";
            error.message += ". Match working bench: img0 + Specim_SWIR3.icd (SWIR3 with NI SSP default).";
            if (!grabberOptions.empty())
                error.message += " SDK Grabber.Channel options: " + grabberOptions + ".";
            if (!deviceNameUtf8.empty())
                error.message += " SSP profile: " + deviceNameUtf8 + ".";
        }
        else if (sensorKind_ == LumoSensorKind::Swir3Ni
                 && (error.message.find("SCB Read failed") != std::string::npos
                     || error.message.find("-1301") != std::string::npos))
        {
            error.message +=
                " Specim -1301: SCB serial init failed. HyperFusion does not set Scb.Channel \u2014 remove "
                "scb_serial_port from hyperfusion.cfg if present, ensure PCU USB1 is connected (Lumo "
                "autoconnects SCB), and verify Camera.Channel/COM6 is the camera head only.";
        }
        rollbackOpenConnection();
        return false;
    }

    SI_BOOL initialized = SI_FALSE;
    if (!checkSi(SI_GetBool(handle, L"IsInitialized", &initialized), "SI_GetBool(IsInitialized)", error))
    {
        rollbackOpenConnection();
        return false;
    }

    if (!initialized)
    {
        error.code = CameraErrorCode::SdkError;
        error.message = tag() + ": SI_Command(Initialize) returned OK but IsInitialized is false.";
        error.fatal = false;
        rollbackOpenConnection();
        return false;
    }

    if (!refreshImageGeometry(error))
    {
        rollbackOpenConnection();
        return false;
    }

    if (!settings_.lumoCalibrationPackPath.empty())
    {
        if (!configureBprAfterInitialize(handle, error))
        {
            rollbackOpenConnection();
            return false;
        }
    }

    state_ = CameraState::Initialized;
    return true;
}

bool LumoCamera::configureBprAfterInitialize(void *handlePtr, CameraError &error)
{
    const SI_H handle = static_cast<SI_H>(handlePtr);

    if (!verifyCalpackLoadedAfterInitialize(handle, error))
        return false;

    const bool useSoftwareBpr = sensorKind_ == LumoSensorKind::Swir3Ni;
    if (!useSoftwareBpr)
        return true;

    swirSoftwareBpr_.reset();
    swirColumnProfileCorrector_.reset();

    const auto &preprocess = hf::hardwareConfig().preprocessing;
    if (preprocess.swir3ColumnProfileCorrect)
    {
        if (!disableSdkBpr(handle, error))
            return false;

        swirColumnProfileCorrector_ = std::make_unique<hf::processing::SwirColumnProfileCorrector>();
        hf::processing::SwirColumnProfileSettings columnSettings;
        columnSettings.baselineRadius =
            preprocess.swir3ColumnProfileBaselineRadius < 1 ? 1 : preprocess.swir3ColumnProfileBaselineRadius;
        columnSettings.valleyGainMin = preprocess.swir3ColumnProfileValleyGainMin;
        columnSettings.minBandDn = preprocess.swir3ColumnProfileMinBandDn;
        columnSettings.minConsecutiveHits =
            preprocess.swir3ColumnProfileMinHits < 1 ? 1 : preprocess.swir3ColumnProfileMinHits;
        columnSettings.minValleyDn = preprocess.swir3ColumnProfileMinValleyDn;
        swirColumnProfileCorrector_->setSettings(columnSettings);
        swirColumnProfileCorrector_->resetState();

        bprStatusSummary_ = "ColumnProfile=on, SDK.BPR=off";
        return true;
    }

    if (!enableSdkBpr(handle, error))
        return false;

    bool sdkBprEnabled = false;
    const bool sdkBprReadable = readSdkBprEnabled(handle, sdkBprEnabled);

    swirSoftwareBpr_ = std::make_unique<hf::processing::SwirBprCorrector>();
    QString loadError;
    const bool mapLoaded =
        swirSoftwareBpr_->loadFromCalpack(QString::fromStdString(settings_.lumoCalibrationPackPath),
                                          &loadError);

    bprStatusSummary_ = "SDK.BPR=" + std::string(sdkBprReadable && sdkBprEnabled ? "on" : "off");
    const std::string sdkCount = formatHandleIntFeature(handle, L"Camera.BPR.BadPixelCount");
    if (sdkCount != "(n/a)")
        bprStatusSummary_ += ", SDK.BadPixelCount=" + sdkCount;

    if (mapLoaded)
    {
        bprStatusSummary_ += ", SoftwareBPR.map=" + std::to_string(swirSoftwareBpr_->badPixelCount())
                            + " (mask " + std::to_string(swirSoftwareBpr_->fullSampleCount()) + "x"
                            + std::to_string(swirSoftwareBpr_->fullBandCount()) + ")";
        if (sdkCount != "(n/a)")
        {
            try
            {
                const int sdkBadCount = std::stoi(sdkCount);
                if (sdkBadCount >= 0
                    && static_cast<int>(swirSoftwareBpr_->badPixelCount()) != sdkBadCount)
                {
                    bprStatusSummary_ += ", mapCountMismatch";
                }
            }
            catch (...)
            {
            }
        }
    }
    else
    {
        swirSoftwareBpr_.reset();
        bprStatusSummary_ += ", SoftwareBPR.map=load_failed";
    }

    if (sdkBprReadable && sdkBprEnabled)
        swirSoftwareBpr_.reset();

    return true;
}

bool LumoCamera::configureSwirAutoNucAfterApplySettings(void *handlePtr, CameraError &error)
{
    const SI_H handle = static_cast<SI_H>(handlePtr);
    nucStatusSummary_.clear();

    if (sensorKind_ != LumoSensorKind::Swir3Ni)
        return true;

    if (!hf::hardwareConfig().preprocessing.swir3AutoNuc)
    {
        nucStatusSummary_ = "AutoNUC=cfg-off";
        return true;
    }

    SI_BOOL implemented = SI_FALSE;
    if (SI_SUCCEEDED(SI_IsImplemented(handle, L"Camera.CheckTemperatures", &implemented)) && implemented)
        (void)SI_Command(handle, L"Camera.CheckTemperatures");

    implemented = SI_FALSE;
    if (!SI_SUCCEEDED(SI_IsImplemented(handle, L"Camera.AutoNUC", &implemented)) || !implemented)
    {
        nucStatusSummary_ = "AutoNUC=(n/a)";
        return true;
    }

    SI_BOOL writable = SI_FALSE;
    if (!SI_SUCCEEDED(SI_IsWritable(handle, L"Camera.AutoNUC", &writable)) || !writable)
    {
        nucStatusSummary_ = "AutoNUC=(not-writable)";
        return true;
    }

    if (!checkSiCode(SI_SetBool(handle, L"Camera.AutoNUC", SI_TRUE),
                     "SI_SetBool(Camera.AutoNUC,true)",
                     error))
        return false;

    double exposureMs = settings_.exposureMs;
    if (SI_SUCCEEDED(SI_GetFloat(handle, L"Camera.ExposureTime", &exposureMs)))
    {
        if (!checkSiCode(SI_SetFloat(handle, L"Camera.ExposureTime", exposureMs),
                         "SI_SetFloat(Camera.ExposureTime,auto-nuc-refresh)",
                         error))
            return false;
    }

    SI_BOOL autoNucEnabled = SI_FALSE;
    if (readHandleBoolFeature(handle, L"Camera.AutoNUC", autoNucEnabled))
        nucStatusSummary_ = std::string("AutoNUC=") + (autoNucEnabled == SI_TRUE ? "on" : "off");
    else
        nucStatusSummary_ = "AutoNUC=on";

    const std::string nucTable = formatHandleEnumFeature(handle, L"Camera.NUC");
    if (nucTable != "(n/a)")
        nucStatusSummary_ += ", NUC=" + nucTable;

    const std::string sensorTemp = formatHandleFloatFeature(handle, L"Camera.Temperature");
    if (sensorTemp != "(n/a)")
        nucStatusSummary_ += ", TempC=" + sensorTemp;

    return true;
}

bool LumoCamera::applyCameraTiming(void *handlePtr,
                                   const CameraSettings &requested,
                                   CameraError &error,
                                   CameraTimingApplyResult *timingOut)
{
    SI_H handle = static_cast<SI_H>(handlePtr);
    CameraTimingApplyResult timing;
    timing.requestedFrameRateHz = requested.frameRateHz;
    timing.requestedExposureMs = requested.exposureMs;

    {
        SI_BOOL implemented = SI_FALSE;
        SI_BOOL writable = SI_FALSE;
        if (SI_SUCCEEDED(SI_IsImplemented(handle, L"Camera.ExposureTime.Auto", &implemented))
            && implemented
            && SI_SUCCEEDED(SI_IsWritable(handle, L"Camera.ExposureTime.Auto", &writable))
            && writable)
        {
            if (!checkSi(SI_SetBool(handle, L"Camera.ExposureTime.Auto", SI_TRUE),
                         "SI_SetBool(Camera.ExposureTime.Auto)",
                         error))
                return false;
            timing.exposureTimeAutoEnabled = true;
        }
        else
        {
            SI_BOOL autoEnabled = SI_FALSE;
            if (SI_SUCCEEDED(SI_GetBool(handle, L"Camera.ExposureTime.Auto", &autoEnabled)))
                timing.exposureTimeAutoEnabled = (autoEnabled == SI_TRUE);
        }
    }

    if (!checkSi(SI_SetFloat(handle, L"Camera.FrameRate", requested.frameRateHz),
                 "SI_SetFloat(Camera.FrameRate)",
                 error))
        return false;

    double exposureMs = requested.exposureMs;
    double exposureMin = 0.0;
    double exposureMax = 0.0;
    if (SI_SUCCEEDED(SI_GetFloatMin(handle, L"Camera.ExposureTime", &exposureMin))
        && SI_SUCCEEDED(SI_GetFloatMax(handle, L"Camera.ExposureTime", &exposureMax))
        && exposureMax > exposureMin)
        exposureMs = std::clamp(exposureMs, exposureMin, exposureMax);

    if (!checkSi(SI_SetFloat(handle, L"Camera.ExposureTime", exposureMs),
                 "SI_SetFloat(Camera.ExposureTime)",
                 error))
    {
        error.message +=
            " (timing budget: readout + exposure ≈ 1000 / frame rate ms; "
            "with Camera.ExposureTime.Auto, exposure may be maximized for the set fps).";
        return false;
    }

    double appliedFrameRateHz = requested.frameRateHz;
    double appliedExposureMs = exposureMs;
    if (!checkSi(SI_GetFloat(handle, L"Camera.FrameRate", &appliedFrameRateHz),
                 "SI_GetFloat(Camera.FrameRate)",
                 error))
        return false;
    if (!checkSi(SI_GetFloat(handle, L"Camera.ExposureTime", &appliedExposureMs),
                 "SI_GetFloat(Camera.ExposureTime)",
                 error))
        return false;

    double readoutTimeMs = 0.0;
    if (SI_SUCCEEDED(SI_GetFloat(handle, L"Camera.Image.ReadoutTime", &readoutTimeMs)))
        timing.readoutTimeMs = readoutTimeMs;

    timing.appliedFrameRateHz = appliedFrameRateHz;
    timing.appliedExposureMs = appliedExposureMs;
    timing.valid = true;

    settings_.frameRateHz = appliedFrameRateHz;
    settings_.exposureMs = appliedExposureMs;

    if (timingOut != nullptr)
        *timingOut = timing;

    return true;
}

bool LumoCamera::applySettings(const CameraSettings &settings,
                               CameraError &error,
                               CameraTimingApplyResult *timingOut)
{
    LumoGlobalLock lumoApi;
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != CameraState::Initialized && state_ != CameraState::Configured
        && state_ != CameraState::SafeStopped && state_ != CameraState::Armed)
        return expectState(CameraState::Initialized, error);

    settings_ = settings;
    SI_H handle = static_cast<SI_H>(handle_);

    if (!applyCameraTiming(handle_, settings, error, timingOut))
        return false;

    const wchar_t *triggerMode = settings_.externalTrigger ? L"External" : L"Internal";
    {
        SI_BOOL implemented = SI_FALSE;
        SI_BOOL writable = SI_FALSE;
        if (SI_SUCCEEDED(SI_IsImplemented(handle, L"Camera.Trigger.Mode", &implemented)) && implemented
            && SI_SUCCEEDED(SI_IsWritable(handle, L"Camera.Trigger.Mode", &writable)) && writable)
        {
            if (!checkSi(SI_SetEnumIndexByString(handle, L"Camera.Trigger.Mode", triggerMode),
                         "SI_SetEnumIndexByString(Camera.Trigger.Mode)",
                         error))
                return false;
        }
    }

    {
        SI_BOOL implemented = SI_FALSE;
        SI_BOOL writable = SI_FALSE;
        if (SI_SUCCEEDED(SI_IsImplemented(handle, L"Acquisition.Timeout", &implemented)) && implemented
            && SI_SUCCEEDED(SI_IsWritable(handle, L"Acquisition.Timeout", &writable)) && writable)
        {
            const double timeoutMs = static_cast<double>(settings_.acquisitionTimeoutMs);
            if (!checkSi(SI_SetFloat(handle, L"Acquisition.Timeout", timeoutMs),
                         "SI_SetFloat(Acquisition.Timeout)",
                         error))
                return false;
        }
    }

    const auto setBinning = [this, handle](const wchar_t *feature,
                                           const int binning,
                                           const char *label,
                                           CameraError &applyError) -> bool {
        if (binning != 1 && binning != 2 && binning != 4 && binning != 8)
        {
            applyError.code = CameraErrorCode::InternalError;
            applyError.message = tag() + std::string(": invalid ") + label + " binning value.";
            applyError.fatal = false;
            return false;
        }

        SI_BOOL implemented = SI_FALSE;
        SI_BOOL writable = SI_FALSE;
        if (!SI_SUCCEEDED(SI_IsImplemented(handle, feature, &implemented)) || !implemented
            || !SI_SUCCEEDED(SI_IsWritable(handle, feature, &writable)) || !writable)
            return true;

        const std::wstring value = std::to_wstring(binning);
        if (!checkSi(SI_SetEnumIndexByString(handle, feature, value.c_str()),
                     label,
                     applyError))
            return false;

        return true;
    };

    if (!setBinning(L"Camera.Binning.Spectral",
                    settings_.spectralBinning,
                    "SI_SetEnumIndexByString(Camera.Binning.Spectral)",
                    error))
        return false;

    if (!setBinning(L"Camera.Binning.Spatial",
                    settings_.spatialBinning,
                    "SI_SetEnumIndexByString(Camera.Binning.Spatial)",
                    error))
        return false;

    if (!refreshImageGeometry(error))
        return false;

    if (!configureSwirAutoNucAfterApplySettings(handle_, error))
        return false;

    state_ = CameraState::Configured;
    return true;
}

bool LumoCamera::openShutter(CameraError &error)
{
    LumoGlobalLock lumoApi;
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != CameraState::Initialized && state_ != CameraState::Configured
        && state_ != CameraState::Armed && state_ != CameraState::Streaming
        && state_ != CameraState::SafeStopped)
        return expectState(CameraState::Initialized, error);

    SI_H handle = static_cast<SI_H>(handle_);
    SI_BOOL implemented = SI_FALSE;
    if (!SI_SUCCEEDED(SI_IsImplemented(handle, L"Camera.OpenShutter", &implemented)) || !implemented)
    {
        error.code = CameraErrorCode::NotImplemented;
        error.message = tag() + ": Camera.OpenShutter is not available on this profile.";
        error.fatal = false;
        return false;
    }

    return checkSi(SI_Command(handle, L"Camera.OpenShutter"), "SI_Command(Camera.OpenShutter)", error);
}

bool LumoCamera::closeShutter(CameraError &error)
{
    LumoGlobalLock lumoApi;
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != CameraState::Initialized && state_ != CameraState::Configured
        && state_ != CameraState::Armed && state_ != CameraState::Streaming
        && state_ != CameraState::SafeStopped)
        return expectState(CameraState::Initialized, error);

    SI_H handle = static_cast<SI_H>(handle_);
    SI_BOOL implemented = SI_FALSE;
    if (!SI_SUCCEEDED(SI_IsImplemented(handle, L"Camera.CloseShutter", &implemented)) || !implemented)
    {
        error.code = CameraErrorCode::NotImplemented;
        error.message = tag() + ": Camera.CloseShutter is not available on this profile.";
        error.fatal = false;
        return false;
    }

    return checkSi(SI_Command(handle, L"Camera.CloseShutter"), "SI_Command(Camera.CloseShutter)", error);
}

bool LumoCamera::shutterIsOpen(bool &isOpen, CameraError &error)
{
    LumoGlobalLock lumoApi;
    std::lock_guard<std::mutex> lock(mutex_);
    isOpen = false;

    if (state_ == CameraState::Disconnected || state_ == CameraState::Connected
        || state_ == CameraState::Fault)
    {
        error.code = CameraErrorCode::InvalidState;
        error.message = tag() + ": shutter status requires an initialized camera.";
        error.fatal = false;
        return false;
    }

    SI_H handle = static_cast<SI_H>(handle_);
    SI_BOOL implemented = SI_FALSE;
    if (!SI_SUCCEEDED(SI_IsImplemented(handle, L"Camera.Shutter.IsOpen", &implemented)) || !implemented)
    {
        error.code = CameraErrorCode::NotImplemented;
        error.message = tag() + ": Camera.Shutter.IsOpen is not available on this profile.";
        error.fatal = false;
        return false;
    }

    SI_BOOL open = SI_FALSE;
    if (!checkSi(SI_GetBool(handle, L"Camera.Shutter.IsOpen", &open), "SI_GetBool(Camera.Shutter.IsOpen)", error))
        return false;

    isOpen = open == SI_TRUE;
    return true;
}

bool LumoCamera::arm(CameraError &error)
{
    LumoGlobalLock lumoApi;
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
    LumoGlobalLock lumoApi;
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

void LumoCamera::haltAcquisition()
{
    LumoGlobalLock lumoApi;
    SI_H handle = nullptr;
    bool shouldStopCommand = false;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (state_ != CameraState::Streaming && state_ != CameraState::Armed)
            return;

        shouldStopCommand = state_ == CameraState::Streaming;
        state_ = CameraState::SafeStopped;
        handle = static_cast<SI_H>(handle_);
    }

    {
        std::lock_guard<std::mutex> frameLock(frameMutex_);
        frameReady_ = false;
    }
    frameCv_.notify_all();

    if (handle != nullptr && shouldStopCommand)
        SI_Command(handle, L"Acquisition.Stop");

    unregisterDataCallback();
}

void LumoCamera::stop()
{
    haltAcquisition();
}

void LumoCamera::disconnect()
{
    LumoGlobalLock lumoApi;
    haltAcquisition();

    void *handleToClose = nullptr;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (state_ == CameraState::Disconnected)
            return;

        handleToClose = handle_;
        handle_ = nullptr;
        swirSoftwareBpr_.reset();
        swirColumnProfileCorrector_.reset();
        bprStatusSummary_.clear();
        nucStatusSummary_.clear();
        state_ = CameraState::Disconnected;
    }

    {
        std::lock_guard<std::mutex> frameLock(frameMutex_);
        frameReady_ = false;
        latestFrameNumber_ = 0;
        latestFrameBytes_.clear();
    }
    frameCv_.notify_all();

    if (handleToClose != nullptr)
    {
        SI_Close(static_cast<SI_H>(handleToClose));
        --g_openSensorHandles;
    }

    releaseSdkLoad();
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

    {
        std::lock_guard<std::mutex> stateLock(mutex_);
        if (state_ != CameraState::Streaming)
        {
            error.code = CameraErrorCode::InvalidState;
            error.message = tag() + " stream stopped.";
            error.fatal = false;
            return false;
        }
    }

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
    frameLock.unlock();

    hf::processing::SwirColumnProfileCorrector *columnProfileCorrector = nullptr;
    hf::processing::SwirBprCorrector *softwareBpr = nullptr;
    int spatialBinning = 1;
    int spectralBinning = 1;
    {
        std::lock_guard<std::mutex> stateLock(mutex_);
        columnProfileCorrector = swirColumnProfileCorrector_.get();
        softwareBpr = swirSoftwareBpr_.get();
        spatialBinning = settings_.spatialBinning;
        spectralBinning = settings_.spectralBinning;
    }

    if (columnProfileCorrector != nullptr)
    {
        (void)spatialBinning;
        (void)spectralBinning;
        columnProfileCorrector->processFrame(frame);
        if (frame.frameIndex % 150U == 0U)
        {
            std::lock_guard<std::mutex> stateLock(mutex_);
            bprStatusSummary_ = "ColumnProfile=on, bad_columns="
                                + std::to_string(columnProfileCorrector->badColumnCount())
                                + ", SDK.BPR=off";
        }
    }
    else if (softwareBpr != nullptr && softwareBpr->isLoaded())
        softwareBpr->apply(frame, spatialBinning, spectralBinning);

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

bool LumoCamera::applySettings(const CameraSettings &settings,
                               CameraError &error,
                               CameraTimingApplyResult *timingOut)
{
    (void)settings;
    (void)error;
    if (timingOut != nullptr)
    {
        timingOut->valid = true;
        timingOut->requestedFrameRateHz = settings.frameRateHz;
        timingOut->requestedExposureMs = settings.exposureMs;
        timingOut->appliedFrameRateHz = settings.frameRateHz;
        timingOut->appliedExposureMs = settings.exposureMs;
    }
    return false;
}

bool LumoCamera::openShutter(CameraError &error)
{
    (void)error;
    return false;
}

bool LumoCamera::closeShutter(CameraError &error)
{
    (void)error;
    return false;
}

bool LumoCamera::shutterIsOpen(bool &isOpen, CameraError &error)
{
    isOpen = false;
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
