// BFS (Blackfly S) camera types for 3D Scanning RGB path (backend/3dscanning).
// Separate from hyperspectral FramePacket (uint16) — RGB8 preview only.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <QString>

namespace hf::bfs
{
enum class BfsCameraState
{
    Disconnected,
    Connected,
    Streaming,
    Fault
};

enum class BfsErrorCode
{
    None,
    NotAvailable,
    InvalidState,
    Timeout,
    SdkError,
    InternalError
};

struct BfsError
{
    BfsErrorCode code = BfsErrorCode::None;
    std::string message;
    bool fatal = false;
};

struct BfsDeviceInfo
{
    QString serial;
    QString model;
    QString displayName() const
    {
        if (model.isEmpty())
            return serial;
        if (serial.isEmpty())
            return model;
        return model + QStringLiteral(" ") + serial;
    }
};

struct BfsCameraSettings
{
    QString cameraId;
    QString acquisitionMode = QStringLiteral("Continuous");
    bool acquisitionFrameRateEnable = true;
    double acquisitionFrameRateHz = 5.0;
    // ~95 MB/s default from SpinView; 12MP @ >~7 Hz can starve GigE and time out.
    int deviceLinkThroughputLimit = 125000000;
    double evCompensation = 0.0;
    QString exposureMode = QStringLiteral("Timed");
    QString exposureAuto = QStringLiteral("Continuous");
    double exposureTimeUs = 15005.0;
    int exposureTimeLowerLimitMinUs = 100;
    int exposureTimeLowerLimitMaxUs = 15000;
    QString gainAuto = QStringLiteral("Continuous");
    double gainDb = 16.9;
    bool gammaEnable = true;
    double gamma = 0.8;
    QString blackLevelSelector = QStringLiteral("All");
    double blackLevelPercent = 0.0;
    QString balanceRatioSelector = QStringLiteral("Red");
    double balanceRatio = 1.21;
    QString balanceWhiteAuto = QStringLiteral("Continuous");
};

struct BfsRgbFrame
{
    std::uint64_t frameIndex = 0;
    int width = 0;
    int height = 0;
    /// Interleaved RGB888, size = width * height * 3.
    std::vector<std::uint8_t> rgb;
};
} // namespace hf::bfs
