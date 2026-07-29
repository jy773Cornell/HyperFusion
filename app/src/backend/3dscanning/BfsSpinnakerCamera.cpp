// Blackfly S Spinnaker adapter — Windows GigE path (backend/3dscanning).
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#if defined(HF_HAVE_SPINNAKER)
// SpinnakerPlatform.h (C++14 path) expands to [[deprecated]] __declspec(dllimport) class,
// which MSVC rejects (C3837). Force the MSVC-compatible form before other Spinnaker headers.
#include "SpinnakerPlatform.h"
#ifdef SPINNAKER_DEPRECATED_CLASS
#undef SPINNAKER_DEPRECATED_CLASS
#endif
#define SPINNAKER_DEPRECATED_CLASS(msg) class SPINNAKER_API __declspec(deprecated(msg))
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#endif

#include "backend/3dscanning/BfsSpinnakerCamera.hpp"

#include <algorithm>
#include <chrono>
#include <utility>
namespace hf::bfs
{
namespace
{
#if defined(HF_HAVE_SPINNAKER)
using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

template <typename T>
T clampValue(T value, T lo, T hi)
{
    if (value < lo)
        return lo;
    if (value > hi)
        return hi;
    return value;
}

bool setEnumByName(INodeMap &nodeMap, const char *nodeName, const char *entryName, std::string *detail)
{
    CEnumerationPtr node = nodeMap.GetNode(nodeName);
    if (!IsAvailable(node) || !IsWritable(node))
    {
        if (detail != nullptr)
            *detail = std::string(nodeName) + " not writable";
        return false;
    }
    CEnumEntryPtr entry = node->GetEntryByName(entryName);
    if (!IsAvailable(entry) || !IsReadable(entry))
    {
        if (detail != nullptr)
            *detail = std::string(nodeName) + "." + entryName + " missing";
        return false;
    }
    node->SetIntValue(entry->GetValue());
    return true;
}

bool setBoolNode(INodeMap &nodeMap, const char *nodeName, const bool value)
{
    CBooleanPtr node = nodeMap.GetNode(nodeName);
    if (!IsAvailable(node) || !IsWritable(node))
        return false;
    node->SetValue(value);
    return true;
}

bool setFloatNode(INodeMap &nodeMap, const char *nodeName, const double value)
{
    CFloatPtr node = nodeMap.GetNode(nodeName);
    if (!IsAvailable(node) || !IsWritable(node))
        return false;
    node->SetValue(clampValue(value, node->GetMin(), node->GetMax()));
    return true;
}

bool setIntNode(INodeMap &nodeMap, const char *nodeName, const int64_t value)
{
    CIntegerPtr node = nodeMap.GetNode(nodeName);
    if (!IsAvailable(node) || !IsWritable(node))
        return false;
    node->SetValue(clampValue(value, node->GetMin(), node->GetMax()));
    return true;
}

QString readStringNode(INodeMap &nodeMap, const char *nodeName)
{
    CStringPtr node = nodeMap.GetNode(nodeName);
    if (!IsAvailable(node) || !IsReadable(node))
        return {};
    const gcstring value = node->GetValue();
    return QString::fromUtf8(value.c_str());
}

bool setStreamModeTeledyneGigE(CameraPtr cam)
{
    INodeMap &sNodeMap = cam->GetTLStreamNodeMap();
    CEnumerationPtr ptrStreamMode = sNodeMap.GetNode("StreamMode");
    if (!IsReadable(ptrStreamMode) || !IsWritable(ptrStreamMode))
        return true;
    CEnumEntryPtr entry = ptrStreamMode->GetEntryByName("TeledyneGigeVision");
    if (!IsReadable(entry))
        return true;
    ptrStreamMode->SetIntValue(entry->GetValue());
    return true;
}

void configureGigEStreamForGrab(CameraPtr cam)
{
    INodeMap &streamMap = cam->GetTLStreamNodeMap();
    // Drop stale buffers so GetNextImage sees fresh frames after restart.
    setEnumByName(streamMap, "StreamBufferHandlingMode", "NewestOnly", nullptr);

    CIntegerPtr packetSize = streamMap.GetNode("StreamPacketSize");
    if (!IsAvailable(packetSize) || !IsWritable(packetSize))
        packetSize = cam->GetNodeMap().GetNode("GevSCPSPacketSize");
    if (IsAvailable(packetSize) && IsWritable(packetSize))
        packetSize->SetValue(packetSize->GetMax());
}

bool configureFreeRunAcquisition(INodeMap &nodeMap, std::string *detail)
{
    // Some firmwares require selector before TriggerMode.
    setEnumByName(nodeMap, "TriggerSelector", "FrameStart", detail);
    const bool triggerOff = setEnumByName(nodeMap, "TriggerMode", "Off", detail);
    const bool continuous = setEnumByName(nodeMap, "AcquisitionMode", "Continuous", detail);
    return triggerOff && continuous;
}
#endif
} // namespace

#if defined(HF_HAVE_SPINNAKER)
struct BfsSpinnakerCamera::Impl
{
    SystemPtr system;
    CameraPtr camera;
    ImageProcessor processor;
    BfsCameraState state = BfsCameraState::Disconnected;
    std::string serial;
    std::uint64_t frameIndex = 0;
};
#else
struct BfsSpinnakerCamera::Impl
{
    BfsCameraState state = BfsCameraState::Disconnected;
    std::string serial;
};
#endif

BfsSpinnakerCamera::BfsSpinnakerCamera()
    : impl_(std::make_unique<Impl>())
{
}

BfsSpinnakerCamera::~BfsSpinnakerCamera()
{
    disconnect();
}

bool BfsSpinnakerCamera::sdkAvailable()
{
#if defined(HF_HAVE_SPINNAKER)
    return true;
#else
    return false;
#endif
}

std::vector<BfsDeviceInfo> BfsSpinnakerCamera::enumerateDevices(BfsError *error)
{
    std::vector<BfsDeviceInfo> devices;
#if defined(HF_HAVE_SPINNAKER)
    try
    {
        SystemPtr system = System::GetInstance();
        CameraList cameras = system->GetCameras();
        const unsigned int count = cameras.GetSize();
        for (unsigned int i = 0; i < count; ++i)
        {
            CameraPtr cam = cameras.GetByIndex(i);
            INodeMap &tl = cam->GetTLDeviceNodeMap();
            BfsDeviceInfo info;
            info.serial = readStringNode(tl, "DeviceSerialNumber");
            info.model = readStringNode(tl, "DeviceModelName");
            if (info.serial.isEmpty() && info.model.isEmpty())
                info.serial = QStringLiteral("Camera %1").arg(i);
            devices.push_back(info);
        }
        cameras.Clear();
        system->ReleaseInstance();
    }
    catch (const Spinnaker::Exception &ex)
    {
        if (error != nullptr)
        {
            error->code = BfsErrorCode::SdkError;
            error->message = std::string("Spinnaker enumerate failed: ") + ex.what();
            error->fatal = false;
        }
    }
#else
    if (error != nullptr)
    {
        error->code = BfsErrorCode::NotAvailable;
        error->message = "Spinnaker SDK not available (HF_HAVE_SPINNAKER off).";
        error->fatal = false;
    }
#endif
    return devices;
}

BfsCameraState BfsSpinnakerCamera::state() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return impl_->state;
}

std::string BfsSpinnakerCamera::connectedSerial() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return impl_->serial;
}

bool BfsSpinnakerCamera::connect(const QString &cameraId, BfsError &error)
{
    std::lock_guard<std::mutex> lock(mutex_);
#if !defined(HF_HAVE_SPINNAKER)
    error = {BfsErrorCode::NotAvailable, "Spinnaker SDK not available.", true};
    impl_->state = BfsCameraState::Fault;
    return false;
#else
    if (impl_->state == BfsCameraState::Connected || impl_->state == BfsCameraState::Streaming)
    {
        error = {BfsErrorCode::InvalidState, "BFS already connected.", false};
        return false;
    }

    try
    {
        impl_->system = System::GetInstance();
        CameraList cameras = impl_->system->GetCameras();
        if (cameras.GetSize() == 0)
        {
            cameras.Clear();
            error = {BfsErrorCode::SdkError, "No Spinnaker cameras found.", false};
            impl_->state = BfsCameraState::Fault;
            return false;
        }

        CameraPtr chosen;
        const QString want = cameraId.trimmed();
        for (unsigned int i = 0; i < cameras.GetSize(); ++i)
        {
            CameraPtr cam = cameras.GetByIndex(i);
            INodeMap &tl = cam->GetTLDeviceNodeMap();
            const QString serial = readStringNode(tl, "DeviceSerialNumber");
            const QString model = readStringNode(tl, "DeviceModelName");
            const QString display = model.isEmpty() ? serial : (model + QStringLiteral(" ") + serial);
            if (want.isEmpty() || want == serial || want == display || display.contains(want))
            {
                chosen = cam;
                impl_->serial = serial.toStdString();
                break;
            }
        }
        if (!chosen)
        {
            chosen = cameras.GetByIndex(0);
            INodeMap &tl = chosen->GetTLDeviceNodeMap();
            impl_->serial = readStringNode(tl, "DeviceSerialNumber").toStdString();
        }

        chosen->Init();
        setStreamModeTeledyneGigE(chosen);
        configureGigEStreamForGrab(chosen);
        // Bayer color cameras need demosaic before RGB8 convert.
        impl_->processor.SetColorProcessing(SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR);
        impl_->camera = chosen;
        cameras.Clear();
        impl_->state = BfsCameraState::Connected;
        impl_->frameIndex = 0;
        return true;
    }
    catch (const Spinnaker::Exception &ex)
    {
        error = {BfsErrorCode::SdkError, std::string("Spinnaker connect failed: ") + ex.what(), true};
        impl_->camera = nullptr;
        if (impl_->system)
        {
            impl_->system->ReleaseInstance();
            impl_->system = nullptr;
        }
        impl_->state = BfsCameraState::Fault;
        return false;
    }
#endif
}

bool BfsSpinnakerCamera::applySettings(const BfsCameraSettings &settings, BfsError &error)
{
    std::lock_guard<std::mutex> lock(mutex_);
#if !defined(HF_HAVE_SPINNAKER)
    error = {BfsErrorCode::NotAvailable, "Spinnaker SDK not available.", true};
    return false;
#else
    if (!impl_->camera
        || (impl_->state != BfsCameraState::Connected && impl_->state != BfsCameraState::Streaming))
    {
        error = {BfsErrorCode::InvalidState, "BFS not connected.", false};
        return false;
    }

    try
    {
        INodeMap &nodeMap = impl_->camera->GetNodeMap();
        std::string detail;

        configureFreeRunAcquisition(nodeMap, &detail);

        setBoolNode(nodeMap, "AcquisitionFrameRateEnable", settings.acquisitionFrameRateEnable);
        if (settings.acquisitionFrameRateEnable)
            setFloatNode(nodeMap, "AcquisitionFrameRate", settings.acquisitionFrameRateHz);

        // Prefer max link throughput unless UI asks for less (12MP needs headroom).
        {
            CIntegerPtr thr = nodeMap.GetNode("DeviceLinkThroughputLimit");
            if (IsAvailable(thr) && IsWritable(thr))
            {
                const int64_t want = static_cast<int64_t>(settings.deviceLinkThroughputLimit);
                const int64_t lo = thr->GetMin();
                const int64_t hi = thr->GetMax();
                thr->SetValue(clampValue(want > 0 ? want : hi, lo, hi));
            }
        }

        setEnumByName(nodeMap, "ExposureMode", settings.exposureMode.toUtf8().constData(), &detail);
        setEnumByName(nodeMap, "ExposureAuto", settings.exposureAuto.toUtf8().constData(), &detail);
        if (settings.exposureAuto.compare(QStringLiteral("Off"), Qt::CaseInsensitive) == 0)
            setFloatNode(nodeMap, "ExposureTime", settings.exposureTimeUs);

        // Auto-exposure window (names vary slightly by firmware).
        if (!setFloatNode(nodeMap, "AutoExposureTimeLowerLimit",
                          static_cast<double>(settings.exposureTimeLowerLimitMinUs)))
            setFloatNode(nodeMap, "ExposureTimeLowerLimit",
                         static_cast<double>(settings.exposureTimeLowerLimitMinUs));
        if (!setFloatNode(nodeMap, "AutoExposureTimeUpperLimit",
                          static_cast<double>(settings.exposureTimeLowerLimitMaxUs)))
            setFloatNode(nodeMap, "ExposureTimeUpperLimit",
                         static_cast<double>(settings.exposureTimeLowerLimitMaxUs));

        setFloatNode(nodeMap, "AutoExposureEVCompensation", settings.evCompensation);
        setFloatNode(nodeMap, "EvCompensation", settings.evCompensation);

        setEnumByName(nodeMap, "GainAuto", settings.gainAuto.toUtf8().constData(), &detail);
        if (settings.gainAuto.compare(QStringLiteral("Off"), Qt::CaseInsensitive) == 0)
            setFloatNode(nodeMap, "Gain", settings.gainDb);

        setBoolNode(nodeMap, "GammaEnable", settings.gammaEnable);
        if (settings.gammaEnable)
            setFloatNode(nodeMap, "Gamma", settings.gamma);

        setEnumByName(nodeMap, "BlackLevelSelector", settings.blackLevelSelector.toUtf8().constData(),
                      &detail);
        setFloatNode(nodeMap, "BlackLevel", settings.blackLevelPercent);

        setEnumByName(nodeMap, "BalanceRatioSelector",
                      settings.balanceRatioSelector.toUtf8().constData(), &detail);
        setEnumByName(nodeMap, "BalanceWhiteAuto", settings.balanceWhiteAuto.toUtf8().constData(),
                      &detail);
        if (settings.balanceWhiteAuto.compare(QStringLiteral("Off"), Qt::CaseInsensitive) == 0)
            setFloatNode(nodeMap, "BalanceRatio", settings.balanceRatio);

        return true;
    }
    catch (const Spinnaker::Exception &ex)
    {
        error = {BfsErrorCode::SdkError, std::string("Spinnaker applySettings failed: ") + ex.what(),
                 false};
        return false;
    }
#endif
}

bool BfsSpinnakerCamera::startStreaming(BfsError &error)
{
    std::lock_guard<std::mutex> lock(mutex_);
#if !defined(HF_HAVE_SPINNAKER)
    error = {BfsErrorCode::NotAvailable, "Spinnaker SDK not available.", true};
    return false;
#else
    if (!impl_->camera || impl_->state != BfsCameraState::Connected)
    {
        error = {BfsErrorCode::InvalidState, "BFS must be Connected before streaming.", false};
        return false;
    }
    try
    {
        INodeMap &nodeMap = impl_->camera->GetNodeMap();
        std::string detail;
        configureFreeRunAcquisition(nodeMap, &detail);
        configureGigEStreamForGrab(impl_->camera);

        impl_->camera->BeginAcquisition();
        impl_->state = BfsCameraState::Streaming;
        return true;
    }
    catch (const Spinnaker::Exception &ex)
    {
        error = {BfsErrorCode::SdkError, std::string("BeginAcquisition failed: ") + ex.what(), true};
        impl_->state = BfsCameraState::Fault;
        return false;
    }
#endif
}

bool BfsSpinnakerCamera::pollFrame(BfsRgbFrame &frame, const std::uint32_t timeoutMs, BfsError &error)
{
    std::lock_guard<std::mutex> lock(mutex_);
#if !defined(HF_HAVE_SPINNAKER)
    error = {BfsErrorCode::NotAvailable, "Spinnaker SDK not available.", true};
    return false;
#else
    if (!impl_->camera || impl_->state != BfsCameraState::Streaming)
    {
        error = {BfsErrorCode::InvalidState, "BFS not streaming.", false};
        return false;
    }

    try
    {
        ImagePtr raw = impl_->camera->GetNextImage(timeoutMs);
        if (!raw || raw->IsIncomplete())
        {
            if (raw)
                raw->Release();
            error = {BfsErrorCode::Timeout, "Incomplete or missing BFS frame.", false};
            return false;
        }

        ImagePtr converted =
            impl_->processor.Convert(raw, Spinnaker::PixelFormat_RGB8);
        raw->Release();

        const auto width = static_cast<int>(converted->GetWidth());
        const auto height = static_cast<int>(converted->GetHeight());
        const std::size_t bytes = static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 3u;
        frame.width = width;
        frame.height = height;
        frame.frameIndex = ++impl_->frameIndex;
        frame.rgb.resize(bytes);
        const auto *src = static_cast<const std::uint8_t *>(converted->GetData());
        std::copy(src, src + static_cast<std::ptrdiff_t>(bytes), frame.rgb.begin());
        converted->Release();
        return true;
    }
    catch (const Spinnaker::Exception &ex)
    {
        // GenTL -1011: wait for NEW_BUFFER_DATA timed out (no frame in timeoutMs).
        const bool bufferTimeout =
            ex.GetError() == SPINNAKER_ERR_TIMEOUT
            || std::string(ex.what()).find("-1011") != std::string::npos
            || std::string(ex.what()).find("NEW_BUFFER_DATA") != std::string::npos;
        if (bufferTimeout)
        {
            error = {BfsErrorCode::Timeout,
                     std::string("GetNextImage timed out (no buffer): ") + ex.what(),
                     false};
            return false;
        }
        error = {BfsErrorCode::SdkError, std::string("GetNextImage failed: ") + ex.what(), false};
        return false;
    }
#endif
}

void BfsSpinnakerCamera::stopStreaming()
{
    std::lock_guard<std::mutex> lock(mutex_);
#if defined(HF_HAVE_SPINNAKER)
    if (impl_->camera && impl_->state == BfsCameraState::Streaming)
    {
        try
        {
            impl_->camera->EndAcquisition();
        }
        catch (...)
        {
        }
        impl_->state = BfsCameraState::Connected;
    }
#endif
}

void BfsSpinnakerCamera::disconnect()
{
    std::lock_guard<std::mutex> lock(mutex_);
#if defined(HF_HAVE_SPINNAKER)
    if (impl_->camera)
    {
        try
        {
            if (impl_->state == BfsCameraState::Streaming)
                impl_->camera->EndAcquisition();
        }
        catch (...)
        {
        }
        try
        {
            if (impl_->camera->IsInitialized())
                impl_->camera->DeInit();
        }
        catch (...)
        {
        }
        impl_->camera = nullptr;
    }
    if (impl_->system)
    {
        try
        {
            CameraList cameras = impl_->system->GetCameras();
            cameras.Clear();
            impl_->system->ReleaseInstance();
        }
        catch (...)
        {
        }
        impl_->system = nullptr;
    }
    impl_->serial.clear();
    impl_->state = BfsCameraState::Disconnected;
#else
    impl_->serial.clear();
    impl_->state = BfsCameraState::Disconnected;
#endif
}

} // namespace hf::bfs
