#pragma once

#include <QString>
#include <QStringList>

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
    /// Spatial scale along the scan axis (mm per detector pixel). Index 0 = FX10e, 1 = SWIR3.
    double spatialMmPerPixel[2] = {0.205, 0.4};
    /// Exposure (ms) for transmittance scans when both illumination modes are selected. Index 0 = FX10e, 1 = SWIR3.
    double transmittanceExposureMs[2] = {12.0, 8.0};
    int lighthouseIdleIntensityPercent = 0;
    int lighthouseReflectancePercent = 100;
    int lighthouseTransmittancePercent = 40;
    double stageMotionAccelerationMmPerSec2 = 30.0;

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
        /// SWIR3: stream-adaptive software BPR (disables SDK Camera.BPR when true).
        bool swir3AdaptiveBpr = false;
        double swir3AdaptiveBprGainMin = 0.3;
        double swir3AdaptiveBprGainMax = 1.5;
        double swir3AdaptiveBprMinNeighborDn = 64.0;
        int swir3AdaptiveBprMinHits = 3;
        int swir3AdaptiveBprMaxPixels = 4096;
    };

    PreprocessingConfig preprocessing;

    struct SegmentationConfig
    {
        QString wslDistro = QStringLiteral("Ubuntu");
        QString wslBashCommand = QStringLiteral("source ~/venvs/gsam2/bin/activate");
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
