#include "frontend/settings/AppSettingsStore.hpp"

#include <QSettings>

#include <algorithm>

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
    position.scanningHomeMm =
        settings.value(QStringLiteral("capture/position/scanningHomeMm"), 50.0).toDouble();
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
    position.gsamPlanId = settings.value(QStringLiteral("capture/preprocess/gsamPlanId")).toString();
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
    settings.setValue(QStringLiteral("capture/position/scanningHomeMm"), position.scanningHomeMm);
    settings.setValue(QStringLiteral("capture/position/useStageForRecording"), position.useStageForRecording);
    settings.setValue(QStringLiteral("capture/preprocess/afterScan"), position.preprocessAfterScan);
    settings.setValue(QStringLiteral("capture/preprocess/saveFfcImage"), position.saveFfcImage);
    settings.setValue(QStringLiteral("capture/preprocess/runGsamSegmentation"), position.runGsamSegmentation);
    settings.setValue(QStringLiteral("capture/preprocess/runHfFusion"), position.runHfFusion);
    settings.setValue(QStringLiteral("capture/preprocess/gsamPrompt"), position.gsamPrompt);
    settings.setValue(QStringLiteral("capture/preprocess/gsamSampleCount"), position.gsamSampleCount);
    settings.setValue(QStringLiteral("capture/preprocess/gsamPlanId"), position.gsamPlanId);
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

namespace
{
PersistedUr3eScanModePanelSettings loadScanModePanel(QSettings &settings,
                                                     const QString &prefix,
                                                     const PersistedUr3eScanModePanelSettings &defaults)
{
    PersistedUr3eScanModePanelSettings panel = defaults;
    panel.sphereRadiusMm =
        settings.value(prefix + QStringLiteral("sphereRadiusMm"), defaults.sphereRadiusMm)
            .toDouble();
    panel.horizontalPoints =
        settings.value(prefix + QStringLiteral("horizontalPoints"), defaults.horizontalPoints)
            .toInt();
    panel.verticalPoints =
        settings.value(prefix + QStringLiteral("verticalPoints"), defaults.verticalPoints).toInt();
    panel.thetaMinDeg =
        settings.value(prefix + QStringLiteral("thetaMinDeg"), defaults.thetaMinDeg).toDouble();
    panel.thetaMaxDeg =
        settings.value(prefix + QStringLiteral("thetaMaxDeg"), defaults.thetaMaxDeg).toDouble();
    panel.wristSweepEnabled =
        settings.value(prefix + QStringLiteral("wristSweepEnabled"), defaults.wristSweepEnabled)
            .toBool();
    panel.wristSweepStepDeg =
        settings.value(prefix + QStringLiteral("wristSweepStepDeg"), defaults.wristSweepStepDeg)
            .toDouble();
    panel.wristSweepStepsEachWay =
        settings
            .value(prefix + QStringLiteral("wristSweepStepsEachWay"),
                   defaults.wristSweepStepsEachWay)
            .toInt();
    panel.wristSweepWrist1 =
        settings.value(prefix + QStringLiteral("wristSweepWrist1"), defaults.wristSweepWrist1)
            .toBool();
    panel.wristSweepWrist2 =
        settings.value(prefix + QStringLiteral("wristSweepWrist2"), defaults.wristSweepWrist2)
            .toBool();
    panel.wristSweepWrist3 =
        settings.value(prefix + QStringLiteral("wristSweepWrist3"), defaults.wristSweepWrist3)
            .toBool();
    panel.imagingIntervalDeg =
        settings.value(prefix + QStringLiteral("imagingIntervalDeg"), defaults.imagingIntervalDeg)
            .toDouble();
    panel.panDirection =
        settings.value(prefix + QStringLiteral("panDirection"), defaults.panDirection).toInt();
    if (panel.panDirection >= 0)
        panel.panDirection = 1;
    else
        panel.panDirection = -1;
    return panel;
}

void saveScanModePanel(QSettings &settings,
                       const QString &prefix,
                       const PersistedUr3eScanModePanelSettings &panel)
{
    settings.setValue(prefix + QStringLiteral("sphereRadiusMm"), panel.sphereRadiusMm);
    settings.setValue(prefix + QStringLiteral("horizontalPoints"), panel.horizontalPoints);
    settings.setValue(prefix + QStringLiteral("verticalPoints"), panel.verticalPoints);
    settings.setValue(prefix + QStringLiteral("thetaMinDeg"), panel.thetaMinDeg);
    settings.setValue(prefix + QStringLiteral("thetaMaxDeg"), panel.thetaMaxDeg);
    settings.setValue(prefix + QStringLiteral("wristSweepEnabled"), panel.wristSweepEnabled);
    settings.setValue(prefix + QStringLiteral("wristSweepStepDeg"), panel.wristSweepStepDeg);
    settings.setValue(prefix + QStringLiteral("wristSweepStepsEachWay"),
                      panel.wristSweepStepsEachWay);
    settings.setValue(prefix + QStringLiteral("wristSweepWrist1"), panel.wristSweepWrist1);
    settings.setValue(prefix + QStringLiteral("wristSweepWrist2"), panel.wristSweepWrist2);
    settings.setValue(prefix + QStringLiteral("wristSweepWrist3"), panel.wristSweepWrist3);
    settings.setValue(prefix + QStringLiteral("imagingIntervalDeg"), panel.imagingIntervalDeg);
    settings.setValue(prefix + QStringLiteral("panDirection"), panel.panDirection);
}

void mirrorActivePanelAliases(PersistedUr3eHemisphereScanSettings &scan)
{
    const PersistedUr3eScanModePanelSettings &panel =
        scan.scanExecuteMode == 1 ? scan.semiPanel : scan.autoPanel;
    scan.sphereRadiusMm = panel.sphereRadiusMm;
    scan.horizontalPoints = panel.horizontalPoints;
    scan.verticalPoints = panel.verticalPoints;
    scan.thetaMinDeg = panel.thetaMinDeg;
    scan.thetaMaxDeg = panel.thetaMaxDeg;
    scan.wristSweepEnabled = panel.wristSweepEnabled;
    scan.wristSweepStepDeg = panel.wristSweepStepDeg;
    scan.wristSweepStepsEachWay = panel.wristSweepStepsEachWay;
    scan.wristSweepWrist1 = panel.wristSweepWrist1;
    scan.wristSweepWrist2 = panel.wristSweepWrist2;
    scan.wristSweepWrist3 = panel.wristSweepWrist3;
    scan.semiFixedIntervalDeg = scan.semiPanel.imagingIntervalDeg;
    scan.semiFixedPanDirection = scan.semiPanel.panDirection;
}
} // namespace

PersistedUr3eHemisphereScanSettings AppSettingsStore::loadUr3eHemisphereScan()
{
    QSettings &settings = storage();
    PersistedUr3eHemisphereScanSettings scan;
    const PersistedUr3eHemisphereScanSettings defaults;

    scan.scanExecuteMode = settings
                               .value(QStringLiteral("ur3e/hemisphereScan/scanExecuteMode"),
                                      defaults.scanExecuteMode)
                               .toInt();
    if (scan.scanExecuteMode != 0 && scan.scanExecuteMode != 1)
        scan.scanExecuteMode = 0;

    const bool hasSplitAuto =
        settings.contains(QStringLiteral("ur3e/hemisphereScan/auto/sphereRadiusMm"));
    const bool hasSplitSemi =
        settings.contains(QStringLiteral("ur3e/hemisphereScan/semi/sphereRadiusMm"));

    if (hasSplitAuto)
    {
        scan.autoPanel = loadScanModePanel(settings, QStringLiteral("ur3e/hemisphereScan/auto/"),
                                           defaults.autoPanel);
    }
    else
    {
        // Migrate legacy flat keys → Auto panel.
        scan.autoPanel.sphereRadiusMm =
            settings.value(QStringLiteral("ur3e/hemisphereScan/sphereRadiusMm"), 500.0).toDouble();
        scan.autoPanel.horizontalPoints =
            settings.value(QStringLiteral("ur3e/hemisphereScan/horizontalPoints"), 12).toInt();
        scan.autoPanel.verticalPoints =
            settings.value(QStringLiteral("ur3e/hemisphereScan/verticalPoints"), 5).toInt();
        scan.autoPanel.thetaMinDeg =
            settings.value(QStringLiteral("ur3e/hemisphereScan/thetaMinDeg"), 30.0).toDouble();
        scan.autoPanel.thetaMaxDeg =
            settings.value(QStringLiteral("ur3e/hemisphereScan/thetaMaxDeg"), 90.0).toDouble();
        scan.autoPanel.wristSweepEnabled =
            settings
                .value(QStringLiteral("ur3e/hemisphereScan/wristSweepEnabled"),
                       defaults.autoPanel.wristSweepEnabled)
                .toBool();
        scan.autoPanel.wristSweepStepDeg =
            settings
                .value(QStringLiteral("ur3e/hemisphereScan/wristSweepStepDeg"),
                       defaults.autoPanel.wristSweepStepDeg)
                .toDouble();
        scan.autoPanel.wristSweepStepsEachWay =
            settings
                .value(QStringLiteral("ur3e/hemisphereScan/wristSweepStepsEachWay"),
                       defaults.autoPanel.wristSweepStepsEachWay)
                .toInt();
        scan.autoPanel.wristSweepWrist1 =
            settings
                .value(QStringLiteral("ur3e/hemisphereScan/wristSweepWrist1"),
                       defaults.autoPanel.wristSweepWrist1)
                .toBool();
        scan.autoPanel.wristSweepWrist2 =
            settings
                .value(QStringLiteral("ur3e/hemisphereScan/wristSweepWrist2"),
                       defaults.autoPanel.wristSweepWrist2)
                .toBool();
        scan.autoPanel.wristSweepWrist3 =
            settings
                .value(QStringLiteral("ur3e/hemisphereScan/wristSweepWrist3"),
                       defaults.autoPanel.wristSweepWrist3)
                .toBool();
    }

    if (hasSplitSemi)
    {
        scan.semiPanel = loadScanModePanel(settings, QStringLiteral("ur3e/hemisphereScan/semi/"),
                                           defaults.semiPanel);
    }
    else
    {
        // Seed Semi from Auto (or legacy flat) + old interval/direction keys.
        scan.semiPanel = scan.autoPanel;
        scan.semiPanel.imagingIntervalDeg =
            settings
                .value(QStringLiteral("ur3e/hemisphereScan/semiFixedIntervalDeg"),
                       defaults.semiPanel.imagingIntervalDeg)
                .toDouble();
        scan.semiPanel.panDirection =
            settings
                .value(QStringLiteral("ur3e/hemisphereScan/semiFixedPanDirection"),
                       defaults.semiPanel.panDirection)
                .toInt();
        if (scan.semiPanel.panDirection >= 0)
            scan.semiPanel.panDirection = 1;
        else
            scan.semiPanel.panDirection = -1;
        // Semi layer default: reuse vertical if horizontal looks like Auto φ count.
        if (scan.semiPanel.horizontalPoints > 36)
            scan.semiPanel.horizontalPoints = std::max(1, scan.semiPanel.verticalPoints);
    }

    scan.lastAutoRoutePath =
        settings
            .value(QStringLiteral("ur3e/hemisphereScan/lastAutoRoutePath"),
                   defaults.lastAutoRoutePath)
            .toString();
    scan.lastSemiFixedPlanPath =
        settings
            .value(QStringLiteral("ur3e/hemisphereScan/lastSemiFixedPlanPath"),
                   defaults.lastSemiFixedPlanPath)
            .toString();
    scan.lastSemiFixedRoutePath =
        settings
            .value(QStringLiteral("ur3e/hemisphereScan/lastSemiFixedRoutePath"),
                   defaults.lastSemiFixedRoutePath)
            .toString();

    mirrorActivePanelAliases(scan);
    return scan;
}

void AppSettingsStore::saveUr3eHemisphereScan(const PersistedUr3eHemisphereScanSettings &scanIn)
{
    PersistedUr3eHemisphereScanSettings scan = scanIn;
    if (scan.scanExecuteMode != 0 && scan.scanExecuteMode != 1)
        scan.scanExecuteMode = 0;
    mirrorActivePanelAliases(scan);

    QSettings &settings = storage();
    settings.setValue(QStringLiteral("ur3e/hemisphereScan/scanExecuteMode"), scan.scanExecuteMode);
    saveScanModePanel(settings, QStringLiteral("ur3e/hemisphereScan/auto/"), scan.autoPanel);
    saveScanModePanel(settings, QStringLiteral("ur3e/hemisphereScan/semi/"), scan.semiPanel);

    // Keep legacy flat keys synced to the active mode (older readers / debugging).
    settings.setValue(QStringLiteral("ur3e/hemisphereScan/sphereRadiusMm"), scan.sphereRadiusMm);
    settings.setValue(QStringLiteral("ur3e/hemisphereScan/horizontalPoints"), scan.horizontalPoints);
    settings.setValue(QStringLiteral("ur3e/hemisphereScan/verticalPoints"), scan.verticalPoints);
    settings.setValue(QStringLiteral("ur3e/hemisphereScan/thetaMinDeg"), scan.thetaMinDeg);
    settings.setValue(QStringLiteral("ur3e/hemisphereScan/thetaMaxDeg"), scan.thetaMaxDeg);
    settings.setValue(QStringLiteral("ur3e/hemisphereScan/wristSweepEnabled"), scan.wristSweepEnabled);
    settings.setValue(QStringLiteral("ur3e/hemisphereScan/wristSweepStepDeg"), scan.wristSweepStepDeg);
    settings.setValue(QStringLiteral("ur3e/hemisphereScan/wristSweepStepsEachWay"),
                      scan.wristSweepStepsEachWay);
    settings.setValue(QStringLiteral("ur3e/hemisphereScan/wristSweepWrist1"), scan.wristSweepWrist1);
    settings.setValue(QStringLiteral("ur3e/hemisphereScan/wristSweepWrist2"), scan.wristSweepWrist2);
    settings.setValue(QStringLiteral("ur3e/hemisphereScan/wristSweepWrist3"), scan.wristSweepWrist3);
    settings.setValue(QStringLiteral("ur3e/hemisphereScan/semiFixedIntervalDeg"),
                      scan.semiFixedIntervalDeg);
    settings.setValue(QStringLiteral("ur3e/hemisphereScan/semiFixedPanDirection"),
                      scan.semiFixedPanDirection);
    settings.setValue(QStringLiteral("ur3e/hemisphereScan/lastAutoRoutePath"),
                      scan.lastAutoRoutePath);
    settings.setValue(QStringLiteral("ur3e/hemisphereScan/lastSemiFixedPlanPath"),
                      scan.lastSemiFixedPlanPath);
    settings.setValue(QStringLiteral("ur3e/hemisphereScan/lastSemiFixedRoutePath"),
                      scan.lastSemiFixedRoutePath);
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

PersistedDlpProjectorSettings AppSettingsStore::loadDlpProjectorSettings()
{
    QSettings &settings = storage();
    const PersistedDlpProjectorSettings defaults;
    PersistedDlpProjectorSettings dlp;
    dlp.deviceId = settings.value(QStringLiteral("dlp/deviceId"), defaults.deviceId).toString();
    dlp.testPattern =
        settings.value(QStringLiteral("dlp/testPattern"), defaults.testPattern).toString();
    dlp.ledRedMa = settings.value(QStringLiteral("dlp/ledRedMa"), defaults.ledRedMa).toInt();
    dlp.ledGreenMa = settings.value(QStringLiteral("dlp/ledGreenMa"), defaults.ledGreenMa).toInt();
    dlp.ledBlueMa = settings.value(QStringLiteral("dlp/ledBlueMa"), defaults.ledBlueMa).toInt();
    return dlp;
}

void AppSettingsStore::saveDlpProjectorSettings(const PersistedDlpProjectorSettings &dlp)
{
    QSettings &settings = storage();
    settings.setValue(QStringLiteral("dlp/deviceId"), dlp.deviceId);
    settings.setValue(QStringLiteral("dlp/testPattern"), dlp.testPattern);
    settings.setValue(QStringLiteral("dlp/ledRedMa"), dlp.ledRedMa);
    settings.setValue(QStringLiteral("dlp/ledGreenMa"), dlp.ledGreenMa);
    settings.setValue(QStringLiteral("dlp/ledBlueMa"), dlp.ledBlueMa);
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
