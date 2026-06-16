#include "frontend/settings/AppSettingsStore.hpp"

#include <QSettings>

namespace
{
constexpr char kOrg[] = "HyperFusion";
constexpr char kApp[] = "HyperFusion";

QString cameraKey(const std::size_t cameraIndex, const char *suffix)
{
    return QStringLiteral("camera/%1/%2").arg(cameraIndex).arg(QString::fromLatin1(suffix));
}
} // namespace

QSettings &AppSettingsStore::storage()
{
    static QSettings settings(QString::fromLatin1(kOrg), QString::fromLatin1(kApp));
    return settings;
}

PersistedCapturePosition AppSettingsStore::loadCapturePosition()
{
    QSettings &settings = storage();
    PersistedCapturePosition position;
    position.targetLengthMm = settings.value(QStringLiteral("capture/position/targetLengthMm"), 125.0).toDouble();
    position.scanningSpeedMmPerSec =
        settings.value(QStringLiteral("capture/position/scanningSpeedMmPerSec"), 25.0).toDouble();
    position.useStageForRecording =
        settings.value(QStringLiteral("capture/position/useStageForRecording"), true).toBool();
    position.preprocessAfterScan =
        settings.value(QStringLiteral("capture/preprocess/afterScan"), true).toBool();
    position.saveFfcImage = settings.value(QStringLiteral("capture/preprocess/saveFfcImage"), true).toBool();
    position.dualCameraAutoSync =
        settings.value(QStringLiteral("capture/dualCameraAutoSync"), true).toBool();
    position.saveFolder = settings.value(QStringLiteral("capture/metadata/saveFolder")).toString();
    return position;
}

void AppSettingsStore::saveCapturePosition(const PersistedCapturePosition &position)
{
    QSettings &settings = storage();
    settings.setValue(QStringLiteral("capture/position/targetLengthMm"), position.targetLengthMm);
    settings.setValue(QStringLiteral("capture/position/scanningSpeedMmPerSec"), position.scanningSpeedMmPerSec);
    settings.setValue(QStringLiteral("capture/position/useStageForRecording"), position.useStageForRecording);
    settings.setValue(QStringLiteral("capture/preprocess/afterScan"), position.preprocessAfterScan);
    settings.setValue(QStringLiteral("capture/preprocess/saveFfcImage"), position.saveFfcImage);
    settings.setValue(QStringLiteral("capture/dualCameraAutoSync"), position.dualCameraAutoSync);
    settings.setValue(QStringLiteral("capture/metadata/saveFolder"), position.saveFolder);
}

PersistedStageConnection AppSettingsStore::loadStageConnection()
{
    QSettings &settings = storage();
    PersistedStageConnection connection;
    connection.port = settings.value(QStringLiteral("stage/port")).toString();
    connection.baud = settings.value(QStringLiteral("stage/baud"), QStringLiteral("115200")).toString();
    return connection;
}

void AppSettingsStore::saveStageConnection(const PersistedStageConnection &connection)
{
    QSettings &settings = storage();
    settings.setValue(QStringLiteral("stage/port"), connection.port);
    settings.setValue(QStringLiteral("stage/baud"), connection.baud);
}

PersistedCameraSettings AppSettingsStore::loadCameraSettings(const std::size_t cameraIndex)
{
    QSettings &settings = storage();
    PersistedCameraSettings camera;
    camera.profile = settings.value(cameraKey(cameraIndex, "profile")).toString();
    camera.frameRateHz = settings.value(cameraKey(cameraIndex, "frameRateHz"), 50.0).toDouble();
    camera.exposureMs = settings.value(cameraKey(cameraIndex, "exposureMs"), 18.0).toDouble();
    camera.spectralBinning =
        settings.value(cameraKey(cameraIndex, "spectralBinning"), QStringLiteral("1")).toString();
    camera.spatialBinning =
        settings.value(cameraKey(cameraIndex, "spatialBinning"), QStringLiteral("1")).toString();
    camera.trigger =
        settings.value(cameraKey(cameraIndex, "trigger"), QStringLiteral("Internal")).toString();
    camera.calibrationPackPath = settings.value(cameraKey(cameraIndex, "calibrationPackPath")).toString();
    camera.redBandIndex = settings.value(cameraKey(cameraIndex, "redBandIndex"), -1).toInt();
    camera.greenBandIndex = settings.value(cameraKey(cameraIndex, "greenBandIndex"), -1).toInt();
    camera.blueBandIndex = settings.value(cameraKey(cameraIndex, "blueBandIndex"), -1).toInt();
    return camera;
}

PersistedLighthouseSettings AppSettingsStore::loadLighthouseSettings()
{
    QSettings &settings = storage();
    PersistedLighthouseSettings lighthouse;
    lighthouse.reflectancePercent = settings.value(QStringLiteral("light/reflectancePercent"), 100).toInt();
    lighthouse.transmittancePercent =
        settings.value(QStringLiteral("light/transmittancePercent"),
                       settings.value(QStringLiteral("light/transmissionPercent"), 40))
            .toInt();
    return lighthouse;
}

void AppSettingsStore::saveLighthouseSettings(const PersistedLighthouseSettings &lighthouse)
{
    QSettings &settings = storage();
    settings.setValue(QStringLiteral("light/reflectancePercent"), lighthouse.reflectancePercent);
    settings.setValue(QStringLiteral("light/transmittancePercent"), lighthouse.transmittancePercent);
}

void AppSettingsStore::saveCameraSettings(const std::size_t cameraIndex,
                                          const PersistedCameraSettings &camera)
{
    QSettings &settings = storage();
    settings.setValue(cameraKey(cameraIndex, "profile"), camera.profile);
    settings.setValue(cameraKey(cameraIndex, "frameRateHz"), camera.frameRateHz);
    settings.setValue(cameraKey(cameraIndex, "exposureMs"), camera.exposureMs);
    settings.setValue(cameraKey(cameraIndex, "spectralBinning"), camera.spectralBinning);
    settings.setValue(cameraKey(cameraIndex, "spatialBinning"), camera.spatialBinning);
    settings.setValue(cameraKey(cameraIndex, "trigger"), camera.trigger);
    settings.setValue(cameraKey(cameraIndex, "calibrationPackPath"), camera.calibrationPackPath);
    settings.setValue(cameraKey(cameraIndex, "redBandIndex"), camera.redBandIndex);
    settings.setValue(cameraKey(cameraIndex, "greenBandIndex"), camera.greenBandIndex);
    settings.setValue(cameraKey(cameraIndex, "blueBandIndex"), camera.blueBandIndex);
}

void AppSettingsStore::sync()
{
    storage().sync();
}
