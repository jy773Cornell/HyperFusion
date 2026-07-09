#pragma once

#include <cstddef>

#include <QString>

class QSettings;

struct PersistedCapturePosition
{
    double targetLengthMm = 125.0;
    double scanningSpeedMmPerSec = 25.0;
    bool useStageForRecording = true;
    bool preprocessAfterScan = true;
    bool saveFfcImage = true;
    bool runGsamSegmentation = false;
    bool runHfFusion = false;
    QString gsamPrompt = QStringLiteral("sample.");
    int gsamSampleCount = 5;
    bool dualCameraAutoSync = true;
    QString saveFolder;
};

struct PersistedStageConnection
{
    QString port;
    QString baud = QStringLiteral("115200");
};

struct PersistedCameraSettings
{
    QString profile;
    double frameRateHz = 50.0;
    double exposureMs = 18.0;
    QString spectralBinning = QStringLiteral("1");
    QString spatialBinning = QStringLiteral("1");
    QString calibrationPackPath;
    int redBandIndex = -1;
    int greenBandIndex = -1;
    int blueBandIndex = -1;
};

struct PersistedLighthouseSettings
{
    int reflectancePercent = 100;
    int transmittancePercent = 40;
};

struct PersistedUr3eHemisphereScanSettings
{
    double sphereRadiusMm = 500.0;
    int horizontalPoints = 12;
    int verticalPoints = 5;
    double thetaMinDeg = 30.0;
    double thetaMaxDeg = 90.0;
};

class AppSettingsStore
{
public:
    static QSettings &storage();

    static PersistedCapturePosition loadCapturePosition();
    static void saveCapturePosition(const PersistedCapturePosition &position);

    static PersistedStageConnection loadStageConnection();
    static void saveStageConnection(const PersistedStageConnection &connection);

    static PersistedCameraSettings loadCameraSettings(std::size_t cameraIndex);
    static void saveCameraSettings(std::size_t cameraIndex, const PersistedCameraSettings &settings);

    static PersistedLighthouseSettings loadLighthouseSettings();
    static void saveLighthouseSettings(const PersistedLighthouseSettings &settings);

    static PersistedUr3eHemisphereScanSettings loadUr3eHemisphereScan();
    static void saveUr3eHemisphereScan(const PersistedUr3eHemisphereScanSettings &settings);

    static void sync();
};
