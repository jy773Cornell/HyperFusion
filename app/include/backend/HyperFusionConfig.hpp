#pragma once

#include <QString>
#include <QStringList>

#include <array>
#include <cstddef>

namespace hf
{
struct HardwareConfig
{
    double distanceDualCameraMm = 190.0;
    /// Index 0 = FX10e, index 1 = SWIR3.
    double whiteRefMm[2] = {735.0, 545.0};
    double brightRefMm[2] = {760.0, 570.0};
    double sampleScanStartMm[2] = {840.0, 650.0};
    double tempStopPositionMm = 500.0;
    /// Legacy UI mirror of whiteRefMm (capture position spin boxes).
    double cameraPositionMm[2] = {735.0, 545.0};
    double sampleWindowMaxLengthMm = 500.0;
    double operationScanningSpeedMmPerSec = 100.0;
    int whiteReferenceFrames = 100;
    int blackReferenceFrames = 100;
    /// Minimum wait after shutter close before black-reference frames count (ms).
    int blackReferenceShutterSettleMs = 1500;
    /// Spatial scale along the scan axis (mm per detector pixel). Index 0 = FX10e, 1 = SWIR3.
    double spatialMmPerPixel[2] = {0.205, 0.4};
    /// Exposure (ms) for transmittance scans when both illumination modes are selected. Index 0 = FX10e, 1 = SWIR3.
    double transmittanceExposureMs[2] = {12.0, 8.0};
    int lighthouseIdleIntensityPercent = 0;
    int lighthouseReflectancePercent = 100;
    int lighthouseTransmittancePercent = 40;
    double stageMotionAccelerationMmPerSec2 = 30.0;

    struct CameraCalibrationConfig
    {
        /// Spatial resolution FWHM along scan axis (mm). Index 0 = FX10e, 1 = SWIR3. Record only (not applied yet).
        double spatialFwhmMm[2] = {0.982, 1.1};
        /// Spectral sampling (nm per band at binning 1). Record only (not applied yet).
        double spectralNmPerPixel[2] = {1.35, 5.6};
        /// Spectral band FWHM (nm). Record only (not applied yet).
        double spectralFwhmNm[2] = {5.5, 12.0};
    };

    CameraCalibrationConfig cameraCalibration;

    struct WavelengthRangeNm
    {
        double minNm = 0.0;
        double maxNm = 0.0;
    };

    struct PreprocessingConfig
    {
        int illuminantD = 65;
        double ffcEpsilon = 1e-6;
        double ffcClampMin = 0.0;
        double ffcClampMax = 1.0;
        double truncateNm = 780.0;
        /// SWIR post-capture false-color PNG: mean reflectance per channel over these nm ranges.
        WavelengthRangeNm swirFalseColorRed{1550.0, 1700.0};
        WavelengthRangeNm swirFalseColorGreen{1100.0, 1300.0};
        WavelengthRangeNm swirFalseColorBlue{950.0, 1050.0};
        /// SWIR3: enable Camera.AutoNUC after timing apply (SDK picks NUC table for exposure).
        bool swir3AutoNuc = true;
        /// SWIR3: band-median spatial profile column destripe (disables SDK Camera.BPR when true).
        bool swir3ColumnProfileCorrect = true;
        int swir3ColumnProfileBaselineRadius = 12;
        double swir3ColumnProfileValleyGainMin = 0.88;
        double swir3ColumnProfileMinBandDn = 64.0;
        int swir3ColumnProfileMinHits = 1;
        double swir3ColumnProfileMinValleyDn = 0.0;
    };

    PreprocessingConfig preprocessing;

    struct SegmentationConfig
    {
        QString wslDistro = QStringLiteral("Ubuntu");
        QString wslBashCommand;
        QString sam2RepoLinux;
        int serverPort = 8765;
        double boxThreshold = 0.30;
        bool multimaskOutput = false;
        bool warmupOnStart = true;
        QString hfModelId = QStringLiteral("IDEA-Research/grounding-dino-base");
        QString sam2Config = QStringLiteral("configs/sam2.1/sam2.1_hiera_l.yaml");
        QString sam2Checkpoint = QStringLiteral("checkpoints/sam2.1_hiera_large.pt");
        QString detectorDevice = QStringLiteral("cuda");
        QString sam2Device = QStringLiteral("cuda");
    };

    SegmentationConfig segmentation;

    struct FusionConfig
    {
        double defaultMarginMm = 5.0;
        int subprocessTimeoutMs = 3600000;
    };

    FusionConfig fusion;

    struct Ur3eConfig
    {
        /// When false, 3D Scanning tabs (UR3e + BFS) and WSL sidecar/driver are disabled.
        bool use3dScanning = true;
        QString wslDistro = QStringLiteral("Ubuntu");
        QString wslBashCommand;
        QString ur3eRepoLinux;
        int serverPort = 8766;
        QString robotIp = QStringLiteral("192.168.0.10");
        QString reverseIp = QStringLiteral("0.0.0.0");
        int dashboardPort = 29999;
        int rtdePort = 30004;
        bool prestartDriver = false;
        /// HTTP /connect client timeout (ms). Driver startup can take ~2 min per attempt.
        int connectTimeoutMs = 120000;
        QString rosDistro = QStringLiteral("jazzy");
        QString urType = QStringLiteral("ur3e");
        bool useMockHardware = true;
        double maxLinearSpeedMPerS = 0.05;
        double maxLinearAccelMPerS2 = 0.3;
        /// MoveIt joint-space peak speed (deg/s). UR hardware allows up to 190.
        double maxJointVelocityDegS = 60.0;
        /// Tool payload collision dome radius on tool0 (mm).
        double toolPayloadRadiusMm = 100.0;
        /// Robot base mount height in world frame (mm). Z=0 is tray floor; mount plane is at this height.
        double ceilingMountHeightMm = 650.0;
        /// MoveIt workspace collision cube (mm). Extends downward from the mount plane (relative to robot).
        bool workspaceBoundaryEnabled = true;
        double workspaceLengthMm = 600.0;
        double workspaceWidthMm = 600.0;
        double workspaceHeightMm = 650.0;
        /// World -> base_link mount orientation (degrees). Default roll=180 = ceiling upside-down.
        double mountRollDeg = 180.0;
        double mountPitchDeg = 0.0;
        double mountYawDeg = 0.0;
        /// Lateral mount offset (mm) from workspace origin in X/Y.
        double mountOffsetXMm = 0.0;
        double mountOffsetYMm = 0.0;
        /// Scan / retreat home pose (degrees): shoulder_pan, lift, elbow, wrist_1, wrist_2, wrist_3.
        std::array<double, 6> homeJointsDeg = {0.0, -150.0, 120.0, 0.0, 90.0, 0.0};
    };

    Ur3eConfig ur3e;

    QString filePath;
    bool loadedFromFile = false;
    QStringList warnings;
};

QStringList hyperFusionConfigSearchPaths();
HardwareConfig loadHardwareConfig();
const HardwareConfig &hardwareConfig();
void setHardwareConfig(HardwareConfig config);
/// Stage speed (mm/s) matching one line per frame: frame rate × spatial scale along scan axis.
double recordScanSpeedMmPerSec(double frameRateHz, double spatialMmPerPixel);
double spatialMmPerPixelForStageCamera(const HardwareConfig &config, std::size_t stageCameraIndex);
/// cfg spatial_mm_per_pixel is at binning 1; multiply by spatial binning for along-scan line spacing.
double effectiveSpatialMmPerPixel(double baseSpatialMmPerPixel, int spatialBinning);
double effectiveSpatialMmPerPixelForStageCamera(const HardwareConfig &config,
                                                std::size_t stageCameraIndex,
                                                int spatialBinning);
bool writeDefaultHardwareConfigFile(const QString &path, QString *errorMessage = nullptr);
} // namespace hf
