// Persisted UI settings implementation.
#include "frontend/controllers/UiSettingsController.hpp"

#include "backend/HyperFusionConfig.hpp"
#include "adapters/zaber/ZaberStageProfile.hpp"
#include "frontend/controllers/CameraPanelController.hpp"
#include "frontend/controllers/CapturePanelController.hpp"
#include "frontend/controllers/LightPanelController.hpp"
#include "frontend/settings/AppSettingsStore.hpp"
#include "frontend/controllers/StagePanelController.hpp"
#include "frontend/widgets/MainWindow.hpp"

#include <QAbstractButton>
#include <QCheckBox>
#include <QComboBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileInfo>
#include <QLineEdit>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QTimer>

#include "frontend/widgets/IntensityBarWidget.hpp"

#include <algorithm>

namespace hf::settings {

UiSettingsController::UiSettingsController(MainWindow *host, QObject *parent)
    : QObject(parent)
    , host_(host)
{
    saveTimer_ = new QTimer(this);
    saveTimer_->setSingleShot(true);
    saveTimer_->setInterval(400);
    connect(saveTimer_, &QTimer::timeout, this, &UiSettingsController::savePersistedUiSettings);
}

void UiSettingsController::loadHardwareConfig()
{
    const hf::HardwareConfig config = hf::loadHardwareConfig();
    applyHardwareConfigToUi();

    if (config.loadedFromFile)
    {
        host_->appendLog(QStringLiteral("Hardware config: loaded %1").arg(config.filePath));
    }
    else
    {
        host_->appendLog(QStringLiteral("Hardware config: using built-in defaults (hyperfusion.cfg not found)"));
    }

    host_->appendLog(QStringLiteral("  fx10e white=%1 mm, swir3 white=%2 mm, fx10e bright=%3 mm, swir3 bright=%4 mm")
                  .arg(config.whiteRefMm[0], 0, 'f', 2)
                  .arg(config.whiteRefMm[1], 0, 'f', 2)
                  .arg(config.brightRefMm[0], 0, 'f', 2)
                  .arg(config.brightRefMm[1], 0, 'f', 2));
    host_->appendLog(QStringLiteral("  fx10e sample start=%1 mm, swir3 sample start=%2 mm, sample window max=%3 mm")
                  .arg(config.sampleScanStartMm[0], 0, 'f', 2)
                  .arg(config.sampleScanStartMm[1], 0, 'f', 2)
                  .arg(config.sampleWindowMaxLengthMm, 0, 'f', 2));
    host_->appendLog(QStringLiteral("  operation_speed=%1 mm/s, fx10e_spatial=%2 mm/px, swir3_spatial=%3 mm/px, "
                              "white_ref_frames=%4, black_ref_frames=%5")
                  .arg(config.operationScanningSpeedMmPerSec, 0, 'f', 1)
                  .arg(config.spatialMmPerPixel[0], 0, 'f', 4)
                  .arg(config.spatialMmPerPixel[1], 0, 'f', 4)
                  .arg(config.whiteReferenceFrames)
                  .arg(config.blackReferenceFrames));
    host_->appendLog(QStringLiteral("  lighthouse idle=%1%%, reflectance=%2%%, transmittance=%3%%")
                  .arg(config.lighthouseIdleIntensityPercent)
                  .arg(config.lighthouseReflectancePercent)
                  .arg(config.lighthouseTransmittancePercent));
    host_->appendLog(QStringLiteral("  dlp led_max=%1 mA, RGB=%2/%3/%4 mA")
                  .arg(config.dlp.ledMaxMa)
                  .arg(config.dlp.ledRedMa)
                  .arg(config.dlp.ledGreenMa)
                  .arg(config.dlp.ledBlueMa));

    for (const QString &warning : config.warnings)
        host_->appendLog(QStringLiteral("Hardware config: %1").arg(warning));
}

void UiSettingsController::applyHardwareConfigToUi()
{
    const hf::HardwareConfig &config = hf::hardwareConfig();

    if (host_->capturePanel() != nullptr)
        host_->capturePanel()->updateScanningSpeedControls();

    if (host_->captureTargetLengthSpin_ != nullptr)
    {
        constexpr double kMinTargetLengthMm = 0.01;
        const double maxTarget = std::min(config.sampleWindowMaxLengthMm, zaber_stage::kTravelLengthMm);
        host_->captureTargetLengthSpin_->setRange(kMinTargetLengthMm, std::max(kMinTargetLengthMm, maxTarget));
        if (host_->captureTargetLengthSpin_->value() > maxTarget)
            host_->captureTargetLengthSpin_->setValue(maxTarget);
        else if (host_->captureTargetLengthSpin_->value() < kMinTargetLengthMm)
            host_->captureTargetLengthSpin_->setValue(kMinTargetLengthMm);
    }

    if (host_->captureScanningHomeSpin_ != nullptr)
    {
        host_->captureScanningHomeSpin_->setRange(zaber_stage::kTravelMinimumMm,
                                                 zaber_stage::kTravelLengthMm);
        const double homeMm = host_->captureScanningHomeSpin_->value();
        host_->captureScanningHomeSpin_->setValue(
            qBound(zaber_stage::kTravelMinimumMm, homeMm, zaber_stage::kTravelLengthMm));
    }
}

void UiSettingsController::loadPersistedUiSettings()
{
    const PersistedCapturePosition capturePosition = AppSettingsStore::loadCapturePosition();
    if (host_->captureTargetLengthSpin_ != nullptr)
        host_->captureTargetLengthSpin_->setValue(capturePosition.targetLengthMm);
    if (host_->captureScanningHomeSpin_ != nullptr)
        host_->captureScanningHomeSpin_->setValue(
            qBound(zaber_stage::kTravelMinimumMm, capturePosition.scanningHomeMm,
                   zaber_stage::kTravelLengthMm));
    if (host_->captureUseStageForRecordingCheck_ != nullptr)
        host_->captureUseStageForRecordingCheck_->setChecked(capturePosition.useStageForRecording);
    if (host_->capturePreprocessAfterScanCheck_ != nullptr)
        host_->capturePreprocessAfterScanCheck_->setChecked(capturePosition.preprocessAfterScan);
    if (host_->captureSaveFfcImageCheck_ != nullptr)
        host_->captureSaveFfcImageCheck_->setChecked(capturePosition.saveFfcImage);
    if (host_->captureRunGsamCheck_ != nullptr)
        host_->captureRunGsamCheck_->setChecked(capturePosition.runGsamSegmentation);
    if (host_->captureRunHfFusionCheck_ != nullptr)
        host_->captureRunHfFusionCheck_->setChecked(capturePosition.runHfFusion);
    if (host_->captureGsamPromptEdit_ != nullptr)
        host_->captureGsamPromptEdit_->setText(capturePosition.gsamPrompt);
    if (host_->captureGsamSampleCountSpin_ != nullptr)
        host_->captureGsamSampleCountSpin_->setValue(std::max(1, capturePosition.gsamSampleCount));
    if (host_->capturePanel() != nullptr)
        host_->capturePanel()->refreshGsamPlanCombo(capturePosition.gsamPlanId);
    if (host_->captureDualCameraAutoCheck_ != nullptr)
        host_->captureDualCameraAutoCheck_->setChecked(capturePosition.dualCameraAutoSync);
    if (host_->captureSaveFolderEdit_ != nullptr && !capturePosition.saveFolder.isEmpty())
        host_->captureSaveFolderEdit_->setText(QDir::toNativeSeparators(capturePosition.saveFolder));

    const PersistedStageConnection stageConnection = AppSettingsStore::loadStageConnection();
    host_->persistedStagePort_ = stageConnection.port;
    if (host_->stageBaudCombo_ != nullptr && !stageConnection.baud.isEmpty())
        host_->stageBaudCombo_->setCurrentText(stageConnection.baud);

    applyPersistedCameraUiValues(host_->camera1Ui_);
    applyPersistedCameraUiValues(host_->camera2Ui_);
    host_->lightPanel()->applyPersistedUiValues();
}

void UiSettingsController::savePersistedUiSettings()
{
    PersistedCapturePosition capturePosition;
    if (host_->captureTargetLengthSpin_ != nullptr)
        capturePosition.targetLengthMm = host_->captureTargetLengthSpin_->value();
    if (host_->captureScanningHomeSpin_ != nullptr)
        capturePosition.scanningHomeMm = host_->captureScanningHomeSpin_->value();
    if (host_->captureUseStageForRecordingCheck_ != nullptr)
        capturePosition.useStageForRecording = host_->captureUseStageForRecordingCheck_->isChecked();
    if (host_->capturePreprocessAfterScanCheck_ != nullptr)
        capturePosition.preprocessAfterScan = host_->capturePreprocessAfterScanCheck_->isChecked();
    if (host_->captureSaveFfcImageCheck_ != nullptr)
        capturePosition.saveFfcImage = host_->captureSaveFfcImageCheck_->isChecked();
    if (host_->captureRunGsamCheck_ != nullptr)
        capturePosition.runGsamSegmentation = host_->captureRunGsamCheck_->isChecked();
    if (host_->captureRunHfFusionCheck_ != nullptr)
        capturePosition.runHfFusion = host_->captureRunHfFusionCheck_->isChecked();
    if (host_->captureGsamPromptEdit_ != nullptr)
        capturePosition.gsamPrompt = host_->captureGsamPromptEdit_->text().trimmed();
    if (host_->captureGsamSampleCountSpin_ != nullptr)
        capturePosition.gsamSampleCount = host_->captureGsamSampleCountSpin_->value();
    if (host_->captureGsamPlanCombo_ != nullptr)
    {
        const QString planPath = host_->captureGsamPlanCombo_->currentData().toString();
        capturePosition.gsamPlanId =
            planPath.isEmpty() ? QString() : QFileInfo(planPath).completeBaseName();
    }
    if (host_->captureDualCameraAutoCheck_ != nullptr)
        capturePosition.dualCameraAutoSync = host_->captureDualCameraAutoCheck_->isChecked();
    if (host_->captureSaveFolderEdit_ != nullptr)
        capturePosition.saveFolder = host_->captureSaveFolderEdit_->text().trimmed();
    AppSettingsStore::saveCapturePosition(capturePosition);

    PersistedStageConnection stageConnection;
    stageConnection.port = host_->stagePanel()->selectedPortName();
    if (stageConnection.port.startsWith(QLatin1Char('(')))
        stageConnection.port.clear();
    if (host_->stageBaudCombo_ != nullptr)
        stageConnection.baud = host_->stageBaudCombo_->currentText();
    AppSettingsStore::saveStageConnection(stageConnection);
    host_->persistedStagePort_ = stageConnection.port;

    savePersistedCameraSettings(host_->camera1Ui_);
    savePersistedCameraSettings(host_->camera2Ui_);
    host_->lightPanel()->savePersistedSettings();

    AppSettingsStore::sync();
}

void UiSettingsController::schedulePersistedUiSettingsSave()
{
    if (saveTimer_ != nullptr)
        saveTimer_->start();
}

void UiSettingsController::applyPersistedCameraUiValues(LumoCameraUi &ui)
{
    const PersistedCameraSettings saved = AppSettingsStore::loadCameraSettings(ui.cameraIndex);

    if (ui.frameRateSpin != nullptr)
        ui.frameRateSpin->setValue(saved.frameRateHz);
    if (ui.exposureSpin != nullptr)
        ui.exposureSpin->setValue(saved.exposureMs);
    if (ui.spectralBinningCombo != nullptr)
    {
        const int index = ui.spectralBinningCombo->findText(saved.spectralBinning);
        if (index >= 0)
            ui.spectralBinningCombo->setCurrentIndex(index);
    }
    if (ui.spatialBinningCombo != nullptr)
    {
        const int index = ui.spatialBinningCombo->findText(saved.spatialBinning);
        if (index >= 0)
            ui.spatialBinningCombo->setCurrentIndex(index);
    }

    if (ui.calibrationPackEdit != nullptr && !saved.calibrationPackPath.isEmpty()
        && QFileInfo::exists(saved.calibrationPackPath))
        hf::camera::CameraPanelController::setCalibrationPackDisplay(ui.calibrationPackEdit, saved.calibrationPackPath);
}

void UiSettingsController::savePersistedCameraSettings(const LumoCameraUi &ui)
{
    PersistedCameraSettings saved;
    if (ui.deviceCombo != nullptr)
        saved.profile = ui.deviceCombo->currentText();
    if (ui.frameRateSpin != nullptr)
        saved.frameRateHz = ui.frameRateSpin->value();
    if (ui.exposureSpin != nullptr)
        saved.exposureMs = ui.exposureSpin->value();
    if (ui.spectralBinningCombo != nullptr)
        saved.spectralBinning = ui.spectralBinningCombo->currentText();
    if (ui.spatialBinningCombo != nullptr)
        saved.spatialBinning = ui.spatialBinningCombo->currentText();

    const QString calpackPath = hf::camera::CameraPanelController::calibrationPackPath(ui);
    if (!calpackPath.isEmpty())
        saved.calibrationPackPath = calpackPath;

    if (ui.redBandCombo != nullptr && ui.redBandCombo->currentIndex() >= 0)
        saved.redBandIndex = ui.redBandCombo->currentData().toInt();
    if (ui.greenBandCombo != nullptr && ui.greenBandCombo->currentIndex() >= 0)
        saved.greenBandIndex = ui.greenBandCombo->currentData().toInt();
    if (ui.blueBandCombo != nullptr && ui.blueBandCombo->currentIndex() >= 0)
        saved.blueBandIndex = ui.blueBandCombo->currentData().toInt();

    AppSettingsStore::saveCameraSettings(ui.cameraIndex, saved);
}

bool UiSettingsController::selectDeviceProfileByName(QComboBox *combo, const QString &profileName) const
{
    if (combo == nullptr || profileName.trimmed().isEmpty())
        return false;

    for (int index = 0; index < combo->count(); ++index)
    {
        if (combo->itemText(index).compare(profileName, Qt::CaseInsensitive) == 0)
        {
            combo->setCurrentIndex(index);
            return true;
        }
    }

    for (int index = 0; index < combo->count(); ++index)
    {
        if (combo->itemText(index).contains(profileName, Qt::CaseInsensitive))
        {
            combo->setCurrentIndex(index);
            return true;
        }
    }

    return false;
}

void UiSettingsController::applyPersistedCameraProfileSelection(LumoCameraUi &ui)
{
    if (ui.deviceCombo == nullptr)
        return;

    const PersistedCameraSettings saved = AppSettingsStore::loadCameraSettings(ui.cameraIndex);
    if (!saved.profile.isEmpty() && selectDeviceProfileByName(ui.deviceCombo, saved.profile))
        return;

    if (ui.sensorKind == LumoSensorKind::Fx10ePleora)
    {
        if (!selectDeviceProfileByName(ui.deviceCombo, QStringLiteral("FX10e with Pleora")))
            selectDeviceProfileByName(ui.deviceCombo, QStringLiteral("FX10"));
    }
    else
    {
        if (!selectDeviceProfileByName(ui.deviceCombo, QStringLiteral("SWIR3 with NI")))
            selectDeviceProfileByName(ui.deviceCombo, QStringLiteral("SWIR"));
    }
}

void UiSettingsController::applyPersistedCameraProfilesAndBands()
{
    applyPersistedCameraProfileSelection(host_->camera1Ui_);
    applyPersistedCameraProfileSelection(host_->camera2Ui_);

    host_->cameraPanel()->refreshBandCombos(host_->camera1Ui_);
    host_->cameraPanel()->refreshBandCombos(host_->camera2Ui_);

    const auto applyBandSelections = [this](LumoCameraUi &ui) {
        const PersistedCameraSettings saved = AppSettingsStore::loadCameraSettings(ui.cameraIndex);
        if (saved.redBandIndex >= 0)
            hf::camera::CameraPanelController::selectBandComboIndex(ui.redBandCombo, saved.redBandIndex);
        if (saved.greenBandIndex >= 0)
            hf::camera::CameraPanelController::selectBandComboIndex(ui.greenBandCombo, saved.greenBandIndex);
        if (saved.blueBandIndex >= 0)
            hf::camera::CameraPanelController::selectBandComboIndex(ui.blueBandCombo, saved.blueBandIndex);
    };

    applyBandSelections(host_->camera1Ui_);
    applyBandSelections(host_->camera2Ui_);

    if (hf::camera::CameraPanelController::calibrationPackPath(host_->camera1Ui_).isEmpty())
        host_->cameraPanel()->syncCalibrationPackToSelectedProfile(host_->camera1Ui_);
    else
        host_->cameraPanel()->ensureCalibrationPackResolved(host_->camera1Ui_);
    if (hf::camera::CameraPanelController::calibrationPackPath(host_->camera2Ui_).isEmpty())
        host_->cameraPanel()->syncCalibrationPackToSelectedProfile(host_->camera2Ui_);
    else
        host_->cameraPanel()->ensureCalibrationPackResolved(host_->camera2Ui_);
}

void UiSettingsController::connectAutosave()
{
    const auto schedule = [this]() { schedulePersistedUiSettingsSave(); };

    if (host_->captureTargetLengthSpin_ != nullptr)
        connect(host_->captureTargetLengthSpin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, schedule);
    if (host_->captureScanningHomeSpin_ != nullptr)
        connect(host_->captureScanningHomeSpin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, schedule);
    if (host_->captureUseStageForRecordingCheck_ != nullptr)
        connect(host_->captureUseStageForRecordingCheck_, &QCheckBox::toggled, this, schedule);
    if (host_->capturePreprocessAfterScanCheck_ != nullptr)
    {
        connect(host_->capturePreprocessAfterScanCheck_, &QCheckBox::toggled, this, schedule);
        connect(host_->capturePreprocessAfterScanCheck_, &QCheckBox::toggled, this, [this]() {
            host_->capturePanel()->updateRecorderControls();
        });
    }
    if (host_->captureSaveFfcImageCheck_ != nullptr)
        connect(host_->captureSaveFfcImageCheck_, &QCheckBox::toggled, this, schedule);
    if (host_->captureRunGsamCheck_ != nullptr)
    {
        connect(host_->captureRunGsamCheck_, &QCheckBox::toggled, this, schedule);
        connect(host_->captureRunGsamCheck_, &QCheckBox::toggled, this, [this](bool checked) {
            if (!checked && host_->captureRunHfFusionCheck_ != nullptr)
            {
                const QSignalBlocker blocker(host_->captureRunHfFusionCheck_);
                host_->captureRunHfFusionCheck_->setChecked(false);
            }
            host_->capturePanel()->updateRecorderControls();
        });
    }
    if (host_->captureRunHfFusionCheck_ != nullptr)
    {
        connect(host_->captureRunHfFusionCheck_, &QCheckBox::toggled, this, schedule);
        connect(host_->captureRunHfFusionCheck_, &QCheckBox::toggled, this, [this]() {
            host_->capturePanel()->updateRecorderControls();
        });
    }
    if (host_->captureGsamPromptEdit_ != nullptr)
        connect(host_->captureGsamPromptEdit_, &QLineEdit::textChanged, this, schedule);
    if (host_->captureGsamSampleCountSpin_ != nullptr)
        connect(host_->captureGsamSampleCountSpin_, qOverload<int>(&QSpinBox::valueChanged), this, schedule);
    if (host_->captureGsamPlanCombo_ != nullptr)
    {
        connect(host_->captureGsamPlanCombo_, qOverload<int>(&QComboBox::currentIndexChanged), this,
                schedule);
        connect(host_->captureGsamPlanCombo_, qOverload<int>(&QComboBox::currentIndexChanged), this,
                [this]() { host_->capturePanel()->updateRecorderControls(); });
    }
    if (host_->stagePortCombo_ != nullptr)
        connect(host_->stagePortCombo_, &QComboBox::currentIndexChanged, this, schedule);
    if (host_->stageBaudCombo_ != nullptr)
        connect(host_->stageBaudCombo_, &QComboBox::currentIndexChanged, this, schedule);

    for (const auto &rowUi : host_->lighthouseRows_)
    {
        if (rowUi.bar != nullptr)
            connect(rowUi.bar, &ui::IntensityBarWidget::percentChanged, this, schedule);
    }

    const auto connectCamera = [this, schedule](LumoCameraUi &ui) {
        if (ui.deviceCombo != nullptr)
            connect(ui.deviceCombo, &QComboBox::currentIndexChanged, this, schedule);
        if (ui.frameRateSpin != nullptr)
        {
            connect(ui.frameRateSpin, qOverload<double>(&QDoubleSpinBox::valueChanged), this, schedule);
            connect(ui.frameRateSpin,
                    qOverload<double>(&QDoubleSpinBox::valueChanged),
                    this,
                    [this]() {
                        if (host_->capturePanel() != nullptr)
                            host_->capturePanel()->updateScanningSpeedControls();
                    });
            if (ui.sensorKind == LumoSensorKind::Fx10ePleora)
            {
                connect(ui.frameRateSpin,
                        qOverload<double>(&QDoubleSpinBox::valueChanged),
                        this,
                        [this]() {
                            if (host_->capturePanel() == nullptr || !host_->capturePanel()->isApplyingDualCameraScanSync())
                                host_->capturePanel()->applyDualCameraScanSync();
                        });
            }
        }
        if (ui.exposureSpin != nullptr)
            connect(ui.exposureSpin, qOverload<double>(&QDoubleSpinBox::valueChanged), this, schedule);
        if (ui.spectralBinningCombo != nullptr)
            connect(ui.spectralBinningCombo, &QComboBox::currentIndexChanged, this, schedule);
        if (ui.spatialBinningCombo != nullptr)
        {
            connect(ui.spatialBinningCombo, &QComboBox::currentIndexChanged, this, schedule);
            connect(ui.spatialBinningCombo,
                    &QComboBox::currentIndexChanged,
                    this,
                    [this]() {
                        if (host_->capturePanel() != nullptr)
        host_->capturePanel()->updateScanningSpeedControls();
                        if (host_->capturePanel() == nullptr || !host_->capturePanel()->isApplyingDualCameraScanSync())
                            host_->capturePanel()->applyDualCameraScanSync();
                    });
        }
        if (ui.redBandCombo != nullptr)
            connect(ui.redBandCombo, &QComboBox::currentIndexChanged, this, schedule);
        if (ui.greenBandCombo != nullptr)
            connect(ui.greenBandCombo, &QComboBox::currentIndexChanged, this, schedule);
        if (ui.blueBandCombo != nullptr)
            connect(ui.blueBandCombo, &QComboBox::currentIndexChanged, this, schedule);
        if (ui.calibrationPackBrowseBtn != nullptr)
        {
            connect(ui.calibrationPackBrowseBtn, &QPushButton::clicked, this, schedule);
        }
    };

    connectCamera(host_->camera1Ui_);
    connectCamera(host_->camera2Ui_);
}

} // namespace hf::settings
