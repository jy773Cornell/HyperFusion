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
    /// Legacy UI mirror of whiteRefMm (capture position spin boxes).
    double cameraPositionMm[2] = {735.0, 545.0};
    double sampleWindowMaxLengthMm = 500.0;
    double operationScanningSpeedMmPerSec = 100.0;
    double recordScanningSpeedMmPerSec = 15.0;
    double whiteReferenceScanningLengthMm = 10.0;
    int blackReferenceFrames = 100;
    double spatialMmPerPixel = 0.050;
    int lighthouseIdleIntensityPercent = 0;
    int lighthouseReflectancePercent = 100;
    int lighthouseTransmittancePercent = 40;
    double stageMotionAccelerationMmPerSec2 = 30.0;

    struct PreprocessingConfig
    {
        int illuminantD = 65;
        double ffcEpsilon = 1e-6;
        double ffcClampMin = 0.0;
        double ffcClampMax = 1.0;
        double truncateNm = 780.0;
    };

    PreprocessingConfig preprocessing;

    QString filePath;
    bool loadedFromFile = false;
    QStringList warnings;
};

QStringList hyperFusionConfigSearchPaths();
HardwareConfig loadHardwareConfig();
const HardwareConfig &hardwareConfig();
void setHardwareConfig(HardwareConfig config);
bool writeDefaultHardwareConfigFile(const QString &path, QString *errorMessage = nullptr);
} // namespace hf
