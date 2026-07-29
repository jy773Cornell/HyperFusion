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
    bool wristSweepEnabled = true;
    double wristSweepStepDeg = 3.0;
    int wristSweepStepsEachWay = 4;
    bool wristSweepWrist1 = false;
    bool wristSweepWrist2 = true;
    bool wristSweepWrist3 = true;
};

struct PersistedBfsCameraSettings
{
    QString cameraId;
    QString acquisitionMode = QStringLiteral("Continuous");
    bool acquisitionFrameRateEnable = true;
    double acquisitionFrameRateHz = 5.0;
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

    static PersistedBfsCameraSettings loadBfsCameraSettings();
    static void saveBfsCameraSettings(const PersistedBfsCameraSettings &settings);

    static void sync();
};
