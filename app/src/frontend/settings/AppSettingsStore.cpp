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
    position.runGsamSegmentation =
        settings.value(QStringLiteral("capture/preprocess/runGsamSegmentation"), false).toBool();
    position.runHfFusion =
        settings.value(QStringLiteral("capture/preprocess/runHfFusion"), false).toBool();
    position.gsamPrompt =
        settings.value(QStringLiteral("capture/preprocess/gsamPrompt"), QStringLiteral("sample.")).toString();
    position.gsamSampleCount =
        settings.value(QStringLiteral("capture/preprocess/gsamSampleCount"), 5).toInt();
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
    settings.setValue(QStringLiteral("capture/preprocess/runGsamSegmentation"), position.runGsamSegmentation);
    settings.setValue(QStringLiteral("capture/preprocess/runHfFusion"), position.runHfFusion);
    settings.setValue(QStringLiteral("capture/preprocess/gsamPrompt"), position.gsamPrompt);
    settings.setValue(QStringLiteral("capture/preprocess/gsamSampleCount"), position.gsamSampleCount);
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

PersistedUr3eHemisphereScanSettings AppSettingsStore::loadUr3eHemisphereScan()
{
    QSettings &settings = storage();
    PersistedUr3eHemisphereScanSettings scan;
    scan.sphereRadiusMm =
        settings.value(QStringLiteral("ur3e/hemisphereScan/sphereRadiusMm"), 500.0).toDouble();
    scan.horizontalPoints =
        settings.value(QStringLiteral("ur3e/hemisphereScan/horizontalPoints"), 12).toInt();
    scan.verticalPoints =
        settings.value(QStringLiteral("ur3e/hemisphereScan/verticalPoints"), 5).toInt();
    scan.thetaMinDeg =
        settings.value(QStringLiteral("ur3e/hemisphereScan/thetaMinDeg"), 30.0).toDouble();
    scan.thetaMaxDeg =
        settings.value(QStringLiteral("ur3e/hemisphereScan/thetaMaxDeg"), 90.0).toDouble();
    return scan;
}

void AppSettingsStore::saveUr3eHemisphereScan(const PersistedUr3eHemisphereScanSettings &scan)
{
    QSettings &settings = storage();
    settings.setValue(QStringLiteral("ur3e/hemisphereScan/sphereRadiusMm"), scan.sphereRadiusMm);
    settings.setValue(QStringLiteral("ur3e/hemisphereScan/horizontalPoints"), scan.horizontalPoints);
    settings.setValue(QStringLiteral("ur3e/hemisphereScan/verticalPoints"), scan.verticalPoints);
    settings.setValue(QStringLiteral("ur3e/hemisphereScan/thetaMinDeg"), scan.thetaMinDeg);
    settings.setValue(QStringLiteral("ur3e/hemisphereScan/thetaMaxDeg"), scan.thetaMaxDeg);
}

PersistedBfsCameraSettings AppSettingsStore::loadBfsCameraSettings()
{
    QSettings &settings = storage();
    const     PersistedBfsCameraSettings defaults;
    PersistedBfsCameraSettings bfs;
    bfs.cameraId = settings.value(QStringLiteral("bfs/cameraId"), defaults.cameraId).toString();
    bfs.acquisitionMode =
        settings.value(QStringLiteral("bfs/acquisitionMode"), defaults.acquisitionMode).toString();
    bfs.acquisitionFrameRateEnable =
        settings.value(QStringLiteral("bfs/acquisitionFrameRateEnable"),
                       defaults.acquisitionFrameRateEnable)
            .toBool();
    bfs.acquisitionFrameRateHz =
        settings.value(QStringLiteral("bfs/acquisitionFrameRateHz"), defaults.acquisitionFrameRateHz)
            .toDouble();
    bfs.deviceLinkThroughputLimit =
        settings.value(QStringLiteral("bfs/deviceLinkThroughputLimit"),
                       defaults.deviceLinkThroughputLimit)
            .toInt();
    bfs.evCompensation =
        settings.value(QStringLiteral("bfs/evCompensation"), defaults.evCompensation).toDouble();
    bfs.exposureMode =
        settings.value(QStringLiteral("bfs/exposureMode"), defaults.exposureMode).toString();
    bfs.exposureAuto =
        settings.value(QStringLiteral("bfs/exposureAuto"), defaults.exposureAuto).toString();
    bfs.exposureTimeUs =
        settings.value(QStringLiteral("bfs/exposureTimeUs"), defaults.exposureTimeUs).toDouble();
    bfs.exposureTimeLowerLimitMinUs =
        settings.value(QStringLiteral("bfs/exposureTimeLowerLimitMinUs"),
                       defaults.exposureTimeLowerLimitMinUs)
            .toInt();
    bfs.exposureTimeLowerLimitMaxUs =
        settings.value(QStringLiteral("bfs/exposureTimeLowerLimitMaxUs"),
                       defaults.exposureTimeLowerLimitMaxUs)
            .toInt();
    bfs.gainAuto = settings.value(QStringLiteral("bfs/gainAuto"), defaults.gainAuto).toString();
    bfs.gainDb = settings.value(QStringLiteral("bfs/gainDb"), defaults.gainDb).toDouble();
    bfs.gammaEnable =
        settings.value(QStringLiteral("bfs/gammaEnable"), defaults.gammaEnable).toBool();
    bfs.gamma = settings.value(QStringLiteral("bfs/gamma"), defaults.gamma).toDouble();
    bfs.blackLevelSelector =
        settings.value(QStringLiteral("bfs/blackLevelSelector"), defaults.blackLevelSelector)
            .toString();
    bfs.blackLevelPercent =
        settings.value(QStringLiteral("bfs/blackLevelPercent"), defaults.blackLevelPercent).toDouble();
    bfs.balanceRatioSelector =
        settings.value(QStringLiteral("bfs/balanceRatioSelector"), defaults.balanceRatioSelector)
            .toString();
    bfs.balanceRatio =
        settings.value(QStringLiteral("bfs/balanceRatio"), defaults.balanceRatio).toDouble();
    bfs.balanceWhiteAuto =
        settings.value(QStringLiteral("bfs/balanceWhiteAuto"), defaults.balanceWhiteAuto).toString();
    return bfs;
}

void AppSettingsStore::saveBfsCameraSettings(const PersistedBfsCameraSettings &bfs)
{
    QSettings &settings = storage();
    settings.setValue(QStringLiteral("bfs/cameraId"), bfs.cameraId);
    settings.setValue(QStringLiteral("bfs/acquisitionMode"), bfs.acquisitionMode);
    settings.setValue(QStringLiteral("bfs/acquisitionFrameRateEnable"), bfs.acquisitionFrameRateEnable);
    settings.setValue(QStringLiteral("bfs/acquisitionFrameRateHz"), bfs.acquisitionFrameRateHz);
    settings.setValue(QStringLiteral("bfs/deviceLinkThroughputLimit"), bfs.deviceLinkThroughputLimit);
    settings.setValue(QStringLiteral("bfs/evCompensation"), bfs.evCompensation);
    settings.setValue(QStringLiteral("bfs/exposureMode"), bfs.exposureMode);
    settings.setValue(QStringLiteral("bfs/exposureAuto"), bfs.exposureAuto);
    settings.setValue(QStringLiteral("bfs/exposureTimeUs"), bfs.exposureTimeUs);
    settings.setValue(QStringLiteral("bfs/exposureTimeLowerLimitMinUs"),
                      bfs.exposureTimeLowerLimitMinUs);
    settings.setValue(QStringLiteral("bfs/exposureTimeLowerLimitMaxUs"),
                      bfs.exposureTimeLowerLimitMaxUs);
    settings.setValue(QStringLiteral("bfs/gainAuto"), bfs.gainAuto);
    settings.setValue(QStringLiteral("bfs/gainDb"), bfs.gainDb);
    settings.setValue(QStringLiteral("bfs/gammaEnable"), bfs.gammaEnable);
    settings.setValue(QStringLiteral("bfs/gamma"), bfs.gamma);
    settings.setValue(QStringLiteral("bfs/blackLevelSelector"), bfs.blackLevelSelector);
    settings.setValue(QStringLiteral("bfs/blackLevelPercent"), bfs.blackLevelPercent);
    settings.setValue(QStringLiteral("bfs/balanceRatioSelector"), bfs.balanceRatioSelector);
    settings.setValue(QStringLiteral("bfs/balanceRatio"), bfs.balanceRatio);
    settings.setValue(QStringLiteral("bfs/balanceWhiteAuto"), bfs.balanceWhiteAuto);
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
    settings.setValue(cameraKey(cameraIndex, "calibrationPackPath"), camera.calibrationPackPath);
    settings.setValue(cameraKey(cameraIndex, "redBandIndex"), camera.redBandIndex);
    settings.setValue(cameraKey(cameraIndex, "greenBandIndex"), camera.greenBandIndex);
    settings.setValue(cameraKey(cameraIndex, "blueBandIndex"), camera.blueBandIndex);
}

void AppSettingsStore::sync()
{
    storage().sync();
}
