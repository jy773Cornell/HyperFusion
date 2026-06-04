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
    double exposureMs = 18.0;
    double frameRateHz = 50.0;
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
    /// Camera.CalibrationPack path (.scp), applied before SI_Command(Initialize) when set.
    std::string lumoCalibrationPackPath;
    /// NI IMAQdx (SWIR3): Grabber.Channel resource name, e.g. img0 (see NI MAX under frame grabber).
    std::string niGrabberChannel;
    /// NI IMAQdx (SWIR3): NiImaq.CameraFile ICD in SDK external/NI, e.g. Fenix SWIR.icd.
    std::string niImaqCameraFile;
    /// NI SWIR SCB serial port for camera head, e.g. COM4 (Specim -1101 if wrong/missing).
    std::string niScbSerialPort;
    /// False-color RGB band indices (from calibration pack wavelength table).
    int redBandIndex = 193;
    int greenBandIndex = 112;
    int blueBandIndex = 25;
};

/// SDK readback after applySettings (Lumo Camera.FrameRate / Camera.ExposureTime).
struct CameraTimingApplyResult
{
    bool valid = false;
    bool exposureTimeAutoEnabled = false;
    double readoutTimeMs = 0.0;
    double requestedFrameRateHz = 0.0;
    double requestedExposureMs = 0.0;
    double appliedFrameRateHz = 0.0;
    double appliedExposureMs = 0.0;
};

struct CameraSettingsApplyReport
{
    CameraSettings requested;
    CameraTimingApplyResult timing;
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
