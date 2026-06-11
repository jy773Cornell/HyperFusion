#pragma once

#include <QString>
#include <QStringList>

#include <cstddef>

namespace hf
{
struct HardwareConfig
{
    /// Index 0 = FX10e, index 1 = SWIR3.
    double cameraPositionMm[2] = {735.0, 545.0};
    double frontEdgeSampleWindowMm = 760.0;
    double sampleWindowLengthMm = 550.0;
    double operationScanningSpeedMmPerSec = 100.0;
    double whiteReferenceScanningLengthMm = 10.0;
    int blackReferenceFrames = 100;
    double spatialMmPerPixel = 0.050;
    int lighthouseReflectancePercent = 100;
    int lighthouseTransmissionPercent = 40;

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
