// Shared camera backend data types, state enums, and frame payloads.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

enum class CameraBackendId
{
    Camera1,
    Camera2
};

enum class CameraState
{
    Disconnected,
    Connected,
    Initialized,
    Configured,
    Armed,
    Streaming,
    SafeStopped,
    Fault
};

enum class CameraErrorCode
{
    None,
    InvalidState,
    Timeout,
    NotImplemented,
    SdkError,
    InternalError
};

struct CameraError
{
    CameraErrorCode code = CameraErrorCode::None;
    std::string message;
    bool fatal = false;
};

struct CameraSettings
{
    double exposureMs = 15.0;
    double frameRateHz = 100.0;
    /// Camera.Binning.Spectral enum value (1, 2, 4, or 8 on FX10e).
    int spectralBinning = 1;
    /// Camera.Binning.Spatial enum value (1, 2, 4, or 8 on FX10e).
    int spatialBinning = 1;
    bool externalTrigger = false;
    std::uint32_t acquisitionTimeoutMs = 1000;
    std::string profileName;
    /// Lumo device index (SI_Open).
    int deviceIndex = 0;
    /// Lumo license file path; empty uses default search.
    std::string lumoLicensePath;
    /// Camera.CalibrationPack path (.scp), applied for FX10e SSP profiles when set.
    std::string lumoCalibrationPackPath;
};

struct FramePacket
{
    CameraBackendId source = CameraBackendId::Camera1;
    std::uint64_t frameIndex = 0;
    std::uint64_t hostTimestampNs = 0;
    int width = 0;
    int height = 0;
    std::vector<std::uint16_t> pixels;
};
