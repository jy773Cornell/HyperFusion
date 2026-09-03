#pragma once

#include <QString>
#include <QStringList>

#include <array>
#include <cstddef>
#include <vector>

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
    /// Stage absolute pose (mm) for Capture Multiview RGB / hemisphere BFS stills.
    double sampleMultiviewPositionMm = 1600.0;
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
        /// SWIR3 post-process: residual comb columns from white/dark refs, then FFC.
        bool swir3RefBprCorrect = true;
        int swir3RefBprBaselineRadius = 2;
        double swir3RefBprWhiteRatioMin = 0.88;
        double swir3RefBprWhiteRatioMax = 1.12;
        double swir3RefBprDarkAbsMinDn = 40.0;
        double swir3RefBprDarkAbsScale = 4.0;
        double swir3RefBprColumnPromoteFrac = 0.25;
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
        /// When false, Multiview tabs (UR3e + BFS) and WSL sidecar/driver are disabled.
        bool useMultiview = true;
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
        /// HTTP /plan_hemisphere_scan client timeout (ms). Large grids + path checks can take >15 min.
        int planTimeoutMs = 3600000;
        QString rosDistro = QStringLiteral("jazzy");
        QString urType = QStringLiteral("ur3e");
        bool useMockHardware = true;
        double maxLinearSpeedMPerS = 0.05;
        double maxLinearAccelMPerS2 = 0.3;
        /// MoveIt joint-space peak speed (deg/s). UR hardware allows up to 190.
        double maxJointVelocityDegS = 60.0;
        /// Pinch-guard bounding sphere at tool0 (mm). MoveIt collision uses toolPayloadMesh.
        double toolPayloadRadiusMm = 77.0;
        QString toolPayloadShape = QStringLiteral("mesh");
        QString toolPayloadMesh = QStringLiteral("ur_tool_payload.stl");
        /// Optical TCP (BFS optical origin) in tool0: Tsai translation (mm) + URDF rpy (deg).
        double toolTcpXMm = 0.715;
        double toolTcpYMm = -54.197;
        double toolTcpZMm = 73.755;
        double toolTcpRollDeg = -1.9138;
        double toolTcpPitchDeg = 0.7450;
        double toolTcpYawDeg = 0.2868;
        /// Robot base mount height in world frame (mm). Z=0 is tray surface; mount plane is at this height.
        double ceilingMountHeightMm = 650.0;
        /// MoveIt workspace collision cube (mm). Extends downward from the mount plane (relative to robot).
        bool workspaceBoundaryEnabled = true;
        double workspaceLengthMm = 600.0;
        double workspaceWidthMm = 600.0;
        double workspaceHeightMm = 650.0;
        /// Keep-out below ceiling mount plane before collision box top (mm).
        double workspaceCeilingClearanceMm = 40.0;
        /// World -> base_link mount orientation (degrees). Default roll=180 = ceiling upside-down.
        double mountRollDeg = 180.0;
        double mountPitchDeg = 0.0;
        double mountYawDeg = 0.0;
        /// Lateral mount offset (mm) from workspace origin in X/Y.
        double mountOffsetXMm = 0.0;
        double mountOffsetYMm = 0.0;
        /// Scan / retreat home pose (degrees): shoulder_pan, lift, elbow, wrist_1, wrist_2, wrist_3.
        std::array<double, 6> homeJointsDeg = {0.0, -150.0, 120.0, 0.0, 90.0, 0.0};
        /// After each scan pin: wrist grid (±steps×stepDeg) on enabled wrists + center.
        bool scanWristSweepEnabled = true;
        double scanWristSweepStepDeg = 3.0;
        int scanWristSweepStepsEachWay = 1;
        bool scanWristSweepWrist1 = false;
        bool scanWristSweepWrist2 = true;
        bool scanWristSweepWrist3 = false;
        /// Settle time after each pin / wrist pose before BFS still (or motion-only dwell).
        int scanCaptureStabilizeMs = 500;
        /// Lock scan TCP roll so image-up ≈ tray/world +Z (projected ⊥ look-at). Pin centers only.
        bool scanCameraUpWorldZ = true;
        /// Half-angle (deg) tip in the vertical plane (look-at × camera-up); 0 = nominal only.
        /// Plan searches inside→out (±mid, ±half; no left/right) for the first reachable pose.
        double pinPoseToleranceDeg = 5.0;
        /// Angular offset (deg) of tool +Z from the nominal look-at-center TCP pose.
        /// Not a wrist joint nudge: baked into TCP orientation (rx,ry,rz).
        /// + = tip toward camera-up; − = tip toward tray. Apex (θ=0) stays look-down.
        /// Pin-pose tolerance tips around this tilted look-at in the same vertical plane.
        double pinTcpTiltDeg = 0.0;
        /// Semi Plan: number of φ candidates per θ ring when hunting base-sweep OK entries.
        /// Spaced evenly over 360° (e.g. 260 → every ~1.4°).
        int semiRingSearchCandidates = 360;
        /// Persist last MoveIt plan beside app.exe; reload on start if cfg fingerprint matches.
        bool rememberLastScanPlan = true;
        /// BFS OpenCV intrinsics for multiview pose JSON / transforms.json (pixels).
        /// fx/fy ≤ 0 → width/height (+ optional cx/cy defaults) only until calibrated.
        double bfsCameraFx = 1787.820905328422;
        double bfsCameraFy = 1787.499380533722;
        double bfsCameraCx = 2084.4011955271963;
        double bfsCameraCy = 1526.0259879957282;
        /// Brown-Conrady distortion: k1,k2,p1,p2,k3 (optional).
        std::vector<double> bfsCameraDistortion{-0.16223465, 0.10156067, 0.0025228506,
                                                0.00068692294, -0.029189458};
    };

    Ur3eConfig ur3e;

    struct DlpConfig
    {
        /// Hard cap for RGB LED current spin boxes (mA). DLP3010EVM-LC optical engine: 2400 mA.
        int ledMaxMa = 2400;
        int ledRedMa = 2400;
        int ledGreenMa = 2400;
        int ledBlueMa = 2400;
    };

    DlpConfig dlp;

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
