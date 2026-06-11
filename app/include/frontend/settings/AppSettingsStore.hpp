#pragma once

#include <cstddef>

#include <QString>

class QSettings;

struct PersistedCapturePosition
{
    double targetLengthMm = 125.0;
    double scanningSpeedMmPerSec = 25.0;
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
    QString trigger = QStringLiteral("Internal");
    QString calibrationPackPath;
    int redBandIndex = -1;
    int greenBandIndex = -1;
    int blueBandIndex = -1;
};

struct PersistedLighthouseSettings
{
    int reflectancePercent = 100;
    int transmissionPercent = 40;
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

    static void sync();
};
