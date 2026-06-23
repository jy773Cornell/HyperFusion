// Camera tab orchestration implementation.
#include "frontend/controllers/CameraPanelController.hpp"

#include "adapters/lumo/LumoCamera.hpp"
#include "adapters/lumo/Swir3NiCamera.hpp"
#include "adapters/lumo/CalpackBandCatalog.hpp"
#include "adapters/lumo/CalibrationPackPaths.hpp"
#include "backend/CameraCoordinator.hpp"
#include "backend/HyperFusionConfig.hpp"
#include "backend/SdkLifecycleRunner.hpp"
#include "frontend/controllers/CapturePanelController.hpp"
#include "frontend/controllers/StagePanelController.hpp"
#include "frontend/processing/DetectorFrameConverter.hpp"
#include "frontend/processing/WavelengthLookup.hpp"
#include "frontend/widgets/CameraStreamTabBuilder.hpp"
#include "frontend/widgets/DetectorCrosshairWidget.hpp"
#include "frontend/widgets/MainWindow.hpp"
#include "frontend/controllers/UiSettingsController.hpp"
#include "frontend/widgets/MainWindowTabHelpers.hpp"
#include "frontend/widgets/OperationWaitDialog.hpp"
#include "frontend/widgets/ProfilePlotWidget.hpp"
#include "frontend/widgets/StreamPaneHelpers.hpp"
#include "frontend/widgets/WaterfallDisplayWidget.hpp"

#include <QApplication>
#include <QComboBox>
#include <QCoreApplication>
#include <QEventLoop>
#include <QDir>
#include <QFileInfo>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QMetaObject>
#include <QPixmap>
#include <QTabWidget>
#include <QTimer>
#include <QSignalBlocker>
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QPushButton>
#include <QTabWidget>

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <vector>

namespace {

constexpr auto kCalibrationPackPathProperty = "hf_calibrationPackPath";
constexpr int kDefaultRedBandIndex = 193;
constexpr int kDefaultGreenBandIndex = 112;
constexpr int kDefaultBlueBandIndex = 25;
constexpr int kFullCalpackBandCount = 448;

int scaledDefaultBandIndex(const int fullCalpackIndex, const int tableBandCount)
{
    if (tableBandCount <= 0)
        return 0;
    if (tableBandCount >= kFullCalpackBandCount)
        return std::clamp(fullCalpackIndex, 0, tableBandCount - 1);

    const int scaled =
        (fullCalpackIndex * (tableBandCount - 1)) / (kFullCalpackBandCount - 1);
    return std::clamp(scaled, 0, tableBandCount - 1);
}

QString defaultCameraTabName(const std::size_t cameraIndex)
{
    return QStringLiteral("Camera %1").arg(cameraIndex + 1);
}

QString ordinalSuffix(const int oneBased)
{
    const int mod100 = oneBased % 100;
    if (mod100 >= 11 && mod100 <= 13)
        return QStringLiteral("th");

    switch (oneBased % 10)
    {
    case 1:
        return QStringLiteral("st");
    case 2:
        return QStringLiteral("nd");
    case 3:
        return QStringLiteral("rd");
    default:
        return QStringLiteral("th");
    }
}

QString formatOrdinalPixel(const int zeroBasedSpatialIndex)
{
    const int pixelNumber = zeroBasedSpatialIndex + 1;
    return QStringLiteral("%1%2 pixel").arg(pixelNumber).arg(ordinalSuffix(pixelNumber));
}

QString formatSessionUptime(const qint64 elapsedMs)
{
    const qint64 totalSeconds = std::max<qint64>(0, elapsedMs / 1000);
    const qint64 hours = totalSeconds / 3600;
    const qint64 minutes = (totalSeconds % 3600) / 60;
    const qint64 seconds = totalSeconds % 60;

    if (hours > 0)
    {
        return QStringLiteral("%1:%2:%3")
            .arg(hours, 2, 10, QChar('0'))
            .arg(minutes, 2, 10, QChar('0'))
            .arg(seconds, 2, 10, QChar('0'));
    }

    return QStringLiteral("%1:%2")
        .arg(minutes, 2, 10, QChar('0'))
        .arg(seconds, 2, 10, QChar('0'));
}

} // namespace

namespace hf::camera {

CameraPanelController::CameraPanelController(MainWindow *host, QObject *parent)
    : QObject(parent)
    , host_(host)
{
    fpsOverlayTimer_ = new QTimer(this);
    fpsOverlayTimer_->setInterval(500);
    connect(fpsOverlayTimer_, &QTimer::timeout, this, [this]() { refreshAcquisitionFpsOverlays(); });
    fpsOverlayTimer_->start();

    sdkFrameRatePollTimer_ = new QTimer(this);
    sdkFrameRatePollTimer_->setInterval(3000);
    connect(sdkFrameRatePollTimer_, &QTimer::timeout, this, [this]() { pollSdkFrameRates(); });
    sdkFrameRatePollTimer_->start();
}


void CameraPanelController::initializeCameras()
{
    host_->camera1Ui_.sensorKind = LumoSensorKind::Fx10ePleora;
    host_->camera1Ui_.camera = std::make_shared<LumoCamera>(CameraBackendId::Camera1,
                                                              QStringLiteral("FX10e").toStdString(),
                                                              LumoSensorKind::Fx10ePleora);
    host_->camera1Ui_.cameraIndex = 0;

    host_->camera2Ui_.sensorKind = LumoSensorKind::Swir3Ni;
    host_->camera2Ui_.camera =
        std::make_shared<Swir3NiCamera>(CameraBackendId::Camera2, QStringLiteral("SWIR3").toStdString());
    host_->camera2Ui_.cameraIndex = 1;

    coordinator_ = std::make_unique<CameraCoordinator>(
        std::vector<std::shared_ptr<ICameraController>>{host_->camera1Ui_.camera, host_->camera2Ui_.camera});

    sdkLifecycleRunner_ = std::make_unique<SdkLifecycleRunner>(this);

    coordinator_->setLogCallback([this](const std::string &message) {
        const QString line = QString::fromStdString(message);
        QMetaObject::invokeMethod(
            host_,
            [this, line]() { host_->appendLog(line); },
            Qt::QueuedConnection);
    });

    coordinator_->setGuiTaskRunner([runner = sdkLifecycleRunner_.get()](std::function<void()> task) {
        if (runner != nullptr)
            runner->runSync(std::move(task));
    });

    coordinator_->setGuiAsyncTaskRunner([](std::function<void()> task) {
        QMetaObject::invokeMethod(qApp, std::move(task), Qt::QueuedConnection);
    });

    coordinator_->setCameraStateCallback(0, [this](const CameraState state) {
        QMetaObject::invokeMethod(
            host_,
            [this, state]() { onCameraStateChanged(host_->camera1Ui_, state); },
            Qt::QueuedConnection);
    });

    coordinator_->setCameraStateCallback(1, [this](const CameraState state) {
        QMetaObject::invokeMethod(
            host_,
            [this, state]() { onCameraStateChanged(host_->camera2Ui_, state); },
            Qt::QueuedConnection);
    });

    coordinator_->setCameraErrorCallback(0, [this](const CameraError &error) {
        QMetaObject::invokeMethod(
            host_,
            [this, error]() { onCameraError(host_->camera1Ui_, error); },
            Qt::QueuedConnection);
    });

    coordinator_->setCameraErrorCallback(1, [this](const CameraError &error) {
        QMetaObject::invokeMethod(
            host_,
            [this, error]() { onCameraError(host_->camera2Ui_, error); },
            Qt::QueuedConnection);
    });

    coordinator_->setCameraShutterStateCallback(0, [this](const bool isOpen) {
        QMetaObject::invokeMethod(
            host_,
            [this, isOpen]() { onShutterStateChanged(host_->camera1Ui_, isOpen); },
            Qt::QueuedConnection);
    });

    coordinator_->setCameraShutterStateCallback(1, [this](const bool isOpen) {
        QMetaObject::invokeMethod(
            host_,
            [this, isOpen]() { onShutterStateChanged(host_->camera2Ui_, isOpen); },
            Qt::QueuedConnection);
    });

    coordinator_->setCameraSettingsAppliedCallback(0, [this](const CameraSettingsApplyReport &report) {
        QMetaObject::invokeMethod(
            host_,
            [this, report]() { onSettingsApplied(host_->camera1Ui_, report); },
            Qt::QueuedConnection);
    });

    coordinator_->setCameraSettingsAppliedCallback(1, [this](const CameraSettingsApplyReport &report) {
        QMetaObject::invokeMethod(
            host_,
            [this, report]() { onSettingsApplied(host_->camera2Ui_, report); },
            Qt::QueuedConnection);
    });

    setupStreamPipeline();
    wireWaterfallPaneResizeHandlers();

    coordinator_->setFrameCallback([this](FramePacket frame) {
        noteStreamFrame(frame);

        if (host_->isCaptureSessionActive())
        {
            const auto packet = std::make_shared<FramePacket>(frame);
            QMetaObject::invokeMethod(
                host_,
                [this, packet]() { onStreamFrame(packet); },
                Qt::QueuedConnection);
        }

        if (streamPipeline_ != nullptr)
            streamPipeline_->ingestFrame(std::move(frame));
    });

    coordinator_->start();
    host_->appendLog("HyperFusion UI initialized; camera coordinator started.");
}

void CameraPanelController::shutdownCoordinatorSync()
{
    if (coordinator_ != nullptr)
        coordinator_->shutdownSync();
    sdkLifecycleRunner_.reset();
}

void CameraPanelController::stopStreamPipeline()
{
    if (streamPipeline_ != nullptr)
        streamPipeline_->stop();
}

CameraCoordinator *CameraPanelController::coordinator() const
{
    return coordinator_.get();
}


QString CameraPanelController::shortProfileTabName(const QString &profileName)
{
    QString name = profileName.trimmed();
    if (name.isEmpty())
        return {};

    const int withIndex = name.indexOf(QStringLiteral(" with "), Qt::CaseInsensitive);
    if (withIndex > 0)
        name = name.left(withIndex).trimmed();

    return name;
}

QString CameraPanelController::profileTabNameForUi(const LumoCameraUi &ui) const
{
    if (ui.sensorKind == LumoSensorKind::Fx10ePleora)
        return QStringLiteral("FX10e");
    if (ui.sensorKind == LumoSensorKind::Swir3Ni)
        return QStringLiteral("SWIR3");
    return defaultCameraTabName(ui.cameraIndex);
}

void CameraPanelController::updateTabLabel(const LumoCameraUi &ui)
{
    const QString tabName = profileTabNameForUi(ui);
    const int tabIndex = static_cast<int>(ui.cameraIndex);

    if (host_->cameraSettingsTabs_ != nullptr && tabIndex >= 0 && tabIndex < host_->cameraSettingsTabs_->count())
        host_->cameraSettingsTabs_->setTabText(tabIndex, tabName);

    if (host_->streamTabs_ != nullptr && tabIndex >= MainWindow::kStreamTabCamera1
        && tabIndex <= MainWindow::kStreamTabCamera2)
        host_->streamTabs_->setTabText(tabIndex, tabName);
}

void CameraPanelController::selectBandComboIndex(QComboBox *combo, const int bandIndex)
{
    if (combo == nullptr)
        return;

    for (int i = 0; i < combo->count(); ++i)
    {
        if (combo->itemData(i).toInt() == bandIndex)
        {
            combo->setCurrentIndex(i);
            return;
        }
    }
}

void CameraPanelController::refreshBandCombos(LumoCameraUi &ui)
{
    if (ui.redBandCombo == nullptr || ui.greenBandCombo == nullptr || ui.blueBandCombo == nullptr)
        return;

    QString calpackPath = calibrationPackPath(ui);
    if (calpackPath.isEmpty())
    {
        const QString profileName =
            ui.deviceCombo != nullptr ? ui.deviceCombo->currentText() : QString();
        calpackPath = defaultCalibrationPackPathForProfile(profileName, ui.sensorKind);
    }

    if (calpackPath.isEmpty() || !QFileInfo::exists(calpackPath))
    {
        for (QComboBox *combo : {ui.redBandCombo, ui.greenBandCombo, ui.blueBandCombo})
        {
            combo->clear();
            combo->addItem(QStringLiteral("(Set calibration pack \u2014 Browse\u2026)"));
            combo->setEnabled(true);
        }
        host_->appendLog(QStringLiteral("%1: calibration pack not set or not found \u2014 use Browse to load "
                                "an .scp file for RGB band selection.")
                      .arg(profileTabNameForUi(ui)));
        return;
    }

    int spectralBinning = 1;
    if (ui.spectralBinningCombo != nullptr && ui.spectralBinningCombo->currentIndex() >= 0)
        spectralBinning = ui.spectralBinningCombo->currentText().toInt();

    std::vector<SpectralBand> bands;
    std::string error;
    if (!CalpackBandCatalog::loadFromCalpack(calpackPath, spectralBinning, bands, error))
    {
        for (QComboBox *combo : {ui.redBandCombo, ui.greenBandCombo, ui.blueBandCombo})
        {
            combo->clear();
            combo->addItem(QStringLiteral("(Failed to read calibration pack)"));
        }
        host_->appendLog(QStringLiteral("%1: band list \u2014 %2")
                      .arg(profileTabNameForUi(ui), QString::fromStdString(error)));
        return;
    }

    const auto populateCombo = [&bands](QComboBox *combo) {
        combo->clear();
        for (const SpectralBand &band : bands)
            combo->addItem(CalpackBandCatalog::formatBandLabel(band), band.index);
    };

    const int prevRed =
        ui.redBandCombo->currentIndex() >= 0 ? ui.redBandCombo->currentData().toInt() : -1;
    const int prevGreen =
        ui.greenBandCombo->currentIndex() >= 0 ? ui.greenBandCombo->currentData().toInt() : -1;
    const int prevBlue =
        ui.blueBandCombo->currentIndex() >= 0 ? ui.blueBandCombo->currentData().toInt() : -1;

    populateCombo(ui.redBandCombo);
    populateCombo(ui.greenBandCombo);
    populateCombo(ui.blueBandCombo);
    ui.spectralBands = bands;

    if (prevRed >= 0)
        selectBandComboIndex(ui.redBandCombo, prevRed);
    else
        selectBandComboIndex(ui.redBandCombo,
                             scaledDefaultBandIndex(kDefaultRedBandIndex,
                                                    static_cast<int>(bands.size())));
    if (prevGreen >= 0)
        selectBandComboIndex(ui.greenBandCombo, prevGreen);
    else
        selectBandComboIndex(ui.greenBandCombo,
                             scaledDefaultBandIndex(kDefaultGreenBandIndex,
                                                    static_cast<int>(bands.size())));
    if (prevBlue >= 0)
        selectBandComboIndex(ui.blueBandCombo, prevBlue);
    else
        selectBandComboIndex(ui.blueBandCombo,
                             scaledDefaultBandIndex(kDefaultBlueBandIndex,
                                                    static_cast<int>(bands.size())));

    syncWaterfallBands(ui);
    syncProfileRgbMarkers(ui);

    host_->appendLog(QStringLiteral("%1: loaded %2 spectral bands (binning %3) from %4")
                  .arg(profileTabNameForUi(ui))
                  .arg(bands.size())
                  .arg(spectralBinning)
                  .arg(QFileInfo(calpackPath).fileName()));
}

QString CameraPanelController::calibrationPackPath(const LumoCameraUi &ui)
{
    QString stored;
    if (ui.calibrationPackEdit != nullptr)
    {
        const QVariant property = ui.calibrationPackEdit->property(kCalibrationPackPathProperty);
        if (property.isValid())
            stored = property.toString();
        if (stored.isEmpty())
            stored = ui.calibrationPackEdit->text();
    }

    return lumo::resolveCalibrationPackPath(stored, ui.sensorKind);
}

QString CameraPanelController::ensureCalibrationPackResolved(LumoCameraUi &ui)
{
    const QString resolved = calibrationPackPath(ui);
    if (resolved.isEmpty() || ui.calibrationPackEdit == nullptr)
        return resolved;

    const QVariant property = ui.calibrationPackEdit->property(kCalibrationPackPathProperty);
    const QString previous = property.isValid() ? property.toString() : QString();
    if (!QFileInfo::exists(previous) || QDir::cleanPath(previous) != resolved)
        setCalibrationPackDisplay(ui.calibrationPackEdit, resolved);

    return resolved;
}

void CameraPanelController::setCalibrationPackDisplay(QLineEdit *edit, const QString &fullPath)
{
    if (edit == nullptr)
        return;

    const QString cleaned = QDir::cleanPath(fullPath);
    edit->setProperty(kCalibrationPackPathProperty, cleaned);
    edit->setText(QFileInfo(cleaned).fileName());
    edit->setToolTip(cleaned);
}

QString CameraPanelController::defaultFx10eCalibrationPackPath()
{
    return lumo::resolveBundledCalibrationPackPath(LumoSensorKind::Fx10ePleora);
}

QString CameraPanelController::defaultSwir3CalibrationPackPath()
{
    return lumo::resolveBundledCalibrationPackPath(LumoSensorKind::Swir3Ni);
}

QString CameraPanelController::defaultCalibrationPackPathForProfile(const QString &profileName,
                                                       const LumoSensorKind sensorKind)
{
    return lumo::defaultCalibrationPackPathForProfile(profileName, sensorKind);
}

void CameraPanelController::syncCalibrationPackToSelectedProfile(LumoCameraUi &ui)
{
    if (ui.calibrationPackEdit == nullptr)
        return;

    const QString profileName =
        ui.deviceCombo != nullptr && ui.deviceCombo->currentIndex() >= 0 ? ui.deviceCombo->currentText()
                                                                         : QString();
    const QString path = defaultCalibrationPackPathForProfile(profileName, ui.sensorKind);
    if (path.isEmpty() || !QFileInfo::exists(path))
        return;

    setCalibrationPackDisplay(ui.calibrationPackEdit, path);
    refreshBandCombos(ui);
}

CameraSettings CameraPanelController::buildSettings(const LumoCameraUi &ui) const
{
    CameraSettings settings;
    settings.frameRateHz = ui.frameRateSpin->value();
    settings.exposureMs = ui.exposureSpin->value();
    if (ui.spectralBinningCombo != nullptr)
        settings.spectralBinning = ui.spectralBinningCombo->currentText().toInt();
    if (ui.spatialBinningCombo != nullptr)
        settings.spatialBinning = ui.spatialBinningCombo->currentText().toInt();
    settings.externalTrigger = false;
    settings.acquisitionTimeoutMs = ui.sensorKind == LumoSensorKind::Swir3Ni ? kSwir3AcquisitionTimeoutMs
                                                                               : kFx10eAcquisitionTimeoutMs;
    if (ui.sensorKind == LumoSensorKind::Swir3Ni)
    {
        settings.niGrabberChannel = "img0";
        settings.niImaqCameraFile = "Specim_SWIR3.icd";
        settings.niCameraSerialPort.clear();
    }
    settings.deviceIndex = ui.deviceCombo->currentData().toInt();
    if (ui.deviceCombo != nullptr)
        settings.profileName = ui.deviceCombo->currentText().toStdString();
    settings.lumoCalibrationPackPath = calibrationPackPath(ui).toStdString();
    const auto bandIndexFromCombo = [](const QComboBox *combo) -> int {
        if (combo == nullptr || combo->count() == 0 || combo->currentIndex() < 0)
            return -1;
        const QVariant data = combo->currentData();
        if (!data.isValid())
            return -1;
        return data.toInt();
    };

    const int red = bandIndexFromCombo(ui.redBandCombo);
    const int green = bandIndexFromCombo(ui.greenBandCombo);
    const int blue = bandIndexFromCombo(ui.blueBandCombo);
    if (red >= 0)
        settings.redBandIndex = red;
    if (green >= 0)
        settings.greenBandIndex = green;
    if (blue >= 0)
        settings.blueBandIndex = blue;
    return settings;
}

bool CameraPanelController::isSessionActive(const CameraState state)
{
    return state != CameraState::Disconnected && state != CameraState::Fault;
}

bool CameraPanelController::anySessionActive() const
{
    return isSessionActive(host_->camera1Ui_.state) || isSessionActive(host_->camera2Ui_.state);
}

void CameraPanelController::refreshDeviceLists()
{
    if (host_->camera1Ui_.camera == nullptr)
        return;

    const CameraSettings prep;

    std::vector<LumoDeviceEntry> devices;
    CameraError error;
    if (!LumoCamera::enumerateDevices(prep, devices, error))
    {
        host_->appendLog(QString("Lumo: profile refresh failed \u2014 %1").arg(QString::fromStdString(error.message)));
        return;
    }

    auto populateCombo = [&devices](QComboBox *combo,
                                    const std::function<bool(const LumoDeviceEntry &)> &include =
                                        nullptr) {
        if (combo == nullptr)
            return;

        combo->clear();
        for (const LumoDeviceEntry &device : devices)
        {
            if (include != nullptr && !include(device))
                continue;

            const QString label = QString::fromStdString(device.name);
            combo->addItem(label, device.index);
        }
    };

    auto selectProfileHint = [](QComboBox *combo, const QString &hint) -> bool {
        if (combo == nullptr)
            return false;
        for (int i = 0; i < combo->count(); ++i)
        {
            if (combo->itemText(i).contains(hint, Qt::CaseInsensitive))
            {
                combo->setCurrentIndex(i);
                return true;
            }
        }
        return false;
    };

    populateCombo(host_->camera1Ui_.deviceCombo, [](const LumoDeviceEntry &device) {
        return ui::lumoProfileMatchesFx10eSlot(QString::fromStdString(device.name));
    });
    populateCombo(host_->camera2Ui_.deviceCombo, [](const LumoDeviceEntry &device) {
        return ui::lumoProfileMatchesSwir3Slot(QString::fromStdString(device.name));
    });

    if (host_->camera1Ui_.deviceCombo != nullptr && host_->camera1Ui_.deviceCombo->count() == 0)
    {
        host_->appendLog(QStringLiteral(
            "FX10e: no matching Pleora/FX10e SSP profile found \u2014 check SpecSensor SDK install."));
    }
    if (host_->camera2Ui_.deviceCombo != nullptr && host_->camera2Ui_.deviceCombo->count() == 0)
    {
        host_->appendLog(QStringLiteral(
            "SWIR3: no matching SWIR/NI SSP profile found \u2014 check SpecSensor SDK install."));
    }

    host_->settingsPanel()->applyPersistedCameraProfileSelection(host_->camera1Ui_);
    host_->settingsPanel()->applyPersistedCameraProfileSelection(host_->camera2Ui_);

    updateTabLabel(host_->camera1Ui_);
    updateTabLabel(host_->camera2Ui_);

    host_->appendLog(QString("Lumo: found %1 SSP profile(s) (from SDK install).").arg(devices.size()));
    for (const LumoDeviceEntry &device : devices)
        host_->appendLog(QString("  [%1] %2").arg(device.index).arg(QString::fromStdString(device.name)));
}

void CameraPanelController::onSettingsApplied(LumoCameraUi &ui, const CameraSettingsApplyReport &report)
{
    const bool dualSyncCompleted = ui.cameraIndex == 1 && host_->capturePanel() != nullptr
                                   && host_->capturePanel()->isDualCameraSyncHardwareApplyPending();
    if (dualSyncCompleted)
        host_->capturePanel()->notifyDualCameraSyncSettingsApplied(report);

    if (ui.cameraIndex < 2 && operationWaits_[ui.cameraIndex].active
        && operationWaits_[ui.cameraIndex].operation == CameraWaitOperation::ApplyingSettings)
    {
        dismissCameraOperationWait(ui.cameraIndex);
    }

    const CameraTimingApplyResult &timing = report.timing;
    if (!timing.valid)
        return;

    constexpr double kHzTolerance = 0.05;
    constexpr double kMsTolerance = 0.05;
    const bool frameRateDiffers =
        std::abs(timing.requestedFrameRateHz - timing.appliedFrameRateHz) > kHzTolerance;
    const bool exposureDiffers =
        std::abs(timing.requestedExposureMs - timing.appliedExposureMs) > kMsTolerance;

    if (ui.frameRateSpin != nullptr)
    {
        QSignalBlocker blocker(ui.frameRateSpin);
        ui.frameRateSpin->setValue(timing.appliedFrameRateHz);
    }
    if (ui.exposureSpin != nullptr)
    {
        QSignalBlocker blocker(ui.exposureSpin);
        ui.exposureSpin->setValue(timing.appliedExposureMs);
    }

    if (dualSyncCompleted)
        return;

    if (!frameRateDiffers && !exposureDiffers)
        return;

    const QString cameraLabel = QStringLiteral("Camera %1").arg(ui.cameraIndex + 1);
    QString reason;
    if (timing.exposureTimeAutoEnabled)
    {
        reason = QStringLiteral(
            "Camera.ExposureTime.Auto is enabled. When frame rate is applied, the Lumo SDK "
            "sets exposure so readout time + exposure time fits the frame period "
            "(1000 / frame rate ms).");
    }
    else
    {
        reason = QStringLiteral(
            "The Lumo SDK limited timing to the valid range for the current frame rate "
            "(readout + exposure must fit within the frame period).");
    }

    const QString message = QStringLiteral(
                                "%1\n\n"
                                "Requested:  %2 Hz, %3 ms\n"
                                "Applied:    %4 Hz, %5 ms\n"
                                "Readout:    %6 ms (Camera.Image.ReadoutTime)")
                                .arg(reason)
                                .arg(timing.requestedFrameRateHz, 0, 'f', 2)
                                .arg(timing.requestedExposureMs, 0, 'f', 2)
                                .arg(timing.appliedFrameRateHz, 0, 'f', 2)
                                .arg(timing.appliedExposureMs, 0, 'f', 2)
                                .arg(timing.readoutTimeMs, 0, 'f', 2);

    QMessageBox::information(host_,
                             QStringLiteral("%1 timing adjusted").arg(cameraLabel),
                             message);
    host_->appendLog(QStringLiteral("%1: timing adjusted \u2014 requested %2 Hz / %3 ms, applied %4 Hz / %5 ms")
                  .arg(cameraLabel)
                  .arg(timing.requestedFrameRateHz, 0, 'f', 2)
                  .arg(timing.requestedExposureMs, 0, 'f', 2)
                  .arg(timing.appliedFrameRateHz, 0, 'f', 2)
                  .arg(timing.appliedExposureMs, 0, 'f', 2));
}

void CameraPanelController::onCameraError(LumoCameraUi &ui, const CameraError &error)
{
    const QString message = QString::fromStdString(error.message);

    if (ui.cameraIndex == 1 && host_->capturePanel() != nullptr
        && host_->capturePanel()->isDualCameraSyncHardwareApplyPending())
    {
        host_->capturePanel()->notifyDualCameraSyncApplyFailed(message);
    }

    dismissCameraOperationWait(ui.cameraIndex);

    if (!ui.connectAttemptActive)
    {
        host_->appendLog(QString("%1: %2").arg(profileTabNameForUi(ui), message));
        return;
    }

    ui.connectAttemptActive = false;
    ui.autoStreamStarted = true;

    const QString title = QString("Camera %1 connection failed").arg(ui.cameraIndex + 1);
    host_->appendLog(QString("%1: %2").arg(title, message));
    QMessageBox::warning(host_, title, message);
}

void CameraPanelController::updateShutterDisplay(LumoCameraUi &ui, const bool isOpen)
{
    ui.shutterReportedOpen = isOpen;

    if (ui.shutterIndicator != nullptr)
    {
        ui.shutterIndicator->setStyleSheet(
            isOpen ? QStringLiteral("background-color: #e67e22; border-radius: 2px;")
                   : QStringLiteral("background-color: #666666; border-radius: 2px;"));
    }

    if (ui.shutterStatusLabel != nullptr)
    {
        ui.shutterStatusLabel->setText(isOpen ? QStringLiteral("Opened") : QStringLiteral("Closed"));
    }

    if (ui.shutterToggleBtn != nullptr)
        ui.shutterToggleBtn->setText(isOpen ? QStringLiteral("Close") : QStringLiteral("Open"));
}

void CameraPanelController::onShutterStateChanged(LumoCameraUi &ui, const bool isOpen)
{
    updateShutterDisplay(ui, isOpen);
}

void CameraPanelController::onCameraStateChanged(LumoCameraUi &ui, const CameraState state)
{
    if (ui.cameraIndex < 2)
    {
        const CameraOperationWait &wait = operationWaits_[ui.cameraIndex];
        if (wait.active)
        {
            if (wait.operation == CameraWaitOperation::Connecting
                && (state == CameraState::Initialized || state == CameraState::Streaming))
            {
                dismissCameraOperationWait(ui.cameraIndex);
            }
            else if (wait.operation == CameraWaitOperation::Connecting && state == CameraState::Fault)
            {
                dismissCameraOperationWait(ui.cameraIndex);
            }
            else if (wait.operation == CameraWaitOperation::Disconnecting
                     && (state == CameraState::Disconnected || state == CameraState::Fault))
            {
                dismissCameraOperationWait(ui.cameraIndex);
            }
        }
    }

    updateCameraControls(ui, state);
    host_->capturePanel()->updateCamerasList();
    host_->capturePanel()->updateRecorderControls();
    syncStreamDisplayLoad();

    if (host_->capturePanel() != nullptr
        && (state == CameraState::Initialized || state == CameraState::Streaming))
    {
        host_->capturePanel()->maybeApplyInitialDualCameraSync();
    }

    if (coordinator_ != nullptr
        && (state == CameraState::Initialized || state == CameraState::Configured
            || state == CameraState::Armed || state == CameraState::Streaming
            || state == CameraState::SafeStopped))
        coordinator_->refreshShutterState(ui.cameraIndex);

    if (state == CameraState::Initialized && coordinator_ != nullptr && !ui.autoStreamStarted)
    {
        ui.autoStreamStarted = true;
        const std::size_t cameraIndex = ui.cameraIndex;
        QTimer::singleShot(0, this, [this, cameraIndex]() { finishAutoStreamStartup(cameraIndex); });
    }

    if (state == CameraState::Streaming)
        ui.connectAttemptActive = false;

    if (state == CameraState::Disconnected)
    {
        ui.autoStreamStarted = false;
        ui.connectAttemptActive = false;
        updateShutterDisplay(ui, false);
        clearDetectorView(ui);
    }
}

void CameraPanelController::finishAutoStreamStartup(const std::size_t cameraIndex)
{
    LumoCameraUi *ui = cameraUiForIndex(cameraIndex);
    if (ui == nullptr || coordinator_ == nullptr || !ui->autoStreamStarted)
        return;

    refreshBandCombos(*ui);
    syncWaterfallBands(*ui);
    syncProfileRgbMarkers(*ui);
    if (ui::WaterfallProcessor *processor = waterfallProcessorFor(*ui))
        processor->reset();
    if (ui::ProfileProcessor *profileProcessor = profileProcessorFor(*ui))
        profileProcessor->reset();

    if (ui->sensorKind == LumoSensorKind::Swir3Ni && ui->camera != nullptr)
    {
        const std::string summary = ui->camera->swirNiConnectionSummary();
        if (!summary.empty())
        {
            host_->appendLog(QStringLiteral("SWIR3 NI autoconnect readback: %1")
                          .arg(QString::fromStdString(summary)));
        }
    }

    const CameraSettings settings = buildSettings(*ui);
    coordinator_->beginStreaming(ui->cameraIndex, settings);
    host_->appendLog(QString("Camera %1: streaming started automatically.").arg(ui->cameraIndex + 1));
}

void CameraPanelController::setConnectDisplayPaused(const bool paused)
{
    if (paused)
    {
        ++connectDisplayPauseDepth_;
    }
    else if (connectDisplayPauseDepth_ > 0)
    {
        --connectDisplayPauseDepth_;
    }

    if (streamPipeline_ != nullptr)
        streamPipeline_->setDisplayPaused(connectDisplayPauseDepth_ > 0);
}

LumoCameraUi *CameraPanelController::cameraUiForIndex(const std::size_t cameraIndex)
{
    if (cameraIndex == 0)
        return &host_->camera1Ui_;
    if (cameraIndex == 1)
        return &host_->camera2Ui_;
    return nullptr;
}

bool CameraPanelController::anyCameraOperationWaitActive() const
{
    for (const CameraOperationWait &slot : operationWaits_)
    {
        if (slot.active)
            return true;
    }
    return false;
}

void CameraPanelController::showCameraOperationWait(const std::size_t cameraIndex,
                                                    const CameraWaitOperation operation)
{
    if (cameraIndex >= 2 || host_ == nullptr)
        return;

    LumoCameraUi *ui = cameraUiForIndex(cameraIndex);
    if (ui == nullptr)
        return;

    dismissCameraOperationWait(cameraIndex);

    const QString cameraName = profileTabNameForUi(*ui);
    CameraOperationWait &slot = operationWaits_[cameraIndex];
    slot.dialog = new OperationWaitDialog(host_);
    slot.dialog->setWindowModality(Qt::NonModal);
    slot.operation = operation;
    slot.active = true;

    switch (operation)
    {
    case CameraWaitOperation::Connecting:
        slot.dialog->setWindowTitle(tr("Connecting %1").arg(cameraName));
        if (ui->sensorKind == LumoSensorKind::Swir3Ni)
        {
            slot.dialog->setStatusText(
                tr("Connecting to %1...\n\nThis may still take up to 30 seconds.")
                    .arg(cameraName));
        }
        else
        {
            slot.dialog->setStatusText(
                tr("Connecting to %1...\n\nThis may take up to %2 seconds.")
                    .arg(cameraName)
                    .arg(kFx10eConnectWaitTimeoutMs / 1000));
        }
        break;
    case CameraWaitOperation::ApplyingSettings:
        slot.dialog->setWindowTitle(tr("Applying settings \u2014 %1").arg(cameraName));
        slot.dialog->setStatusText(tr("Applying settings to %1...").arg(cameraName));
        break;
    case CameraWaitOperation::Disconnecting:
        slot.dialog->setWindowTitle(tr("Disconnecting %1").arg(cameraName));
        slot.dialog->setStatusText(tr("Disconnecting %1...").arg(cameraName));
        break;
    }

    slot.dialog->show();
    slot.dialog->raise();

    if (operation == CameraWaitOperation::Connecting)
        setConnectDisplayPaused(true);

    if (operation == CameraWaitOperation::Connecting)
    {
        if (connectTimeoutTimer_ == nullptr)
        {
            connectTimeoutTimer_ = new QTimer(this);
            connectTimeoutTimer_->setSingleShot(true);
            connect(connectTimeoutTimer_, &QTimer::timeout, this, [this]() {
                onCameraOperationWaitTimedOut(connectTimeoutCameraIndex_);
            });
        }
        connectTimeoutCameraIndex_ = cameraIndex;
        const int timeoutMs = ui->sensorKind == LumoSensorKind::Swir3Ni ? kSwir3ConnectWaitTimeoutMs
                                                                          : kFx10eConnectWaitTimeoutMs;
        connectTimeoutTimer_->start(timeoutMs);
    }
}

void CameraPanelController::onCameraOperationWaitTimedOut(const std::size_t cameraIndex)
{
    if (cameraIndex >= 2)
        return;

    CameraOperationWait &slot = operationWaits_[cameraIndex];
    if (!slot.active || slot.operation != CameraWaitOperation::Connecting)
        return;

    LumoCameraUi *ui = cameraUiForIndex(cameraIndex);
    if (ui != nullptr)
    {
        ui->connectAttemptActive = false;
        const int timeoutSec = ui->sensorKind == LumoSensorKind::Swir3Ni ? kSwir3ConnectWaitTimeoutMs / 1000
                                                                           : kFx10eConnectWaitTimeoutMs / 1000;
        host_->appendLog(
            QStringLiteral("%1: connect wait timed out after %2 s \u2014 check the log, then retry Connect.")
                .arg(profileTabNameForUi(*ui))
                .arg(timeoutSec));
    }
    dismissCameraOperationWait(cameraIndex);
}

void CameraPanelController::dismissCameraOperationWait(const std::size_t cameraIndex)
{
    if (cameraIndex >= 2)
        return;

    if (connectTimeoutTimer_ != nullptr && connectTimeoutCameraIndex_ == cameraIndex)
        connectTimeoutTimer_->stop();

    CameraOperationWait &slot = operationWaits_[cameraIndex];
    if (!slot.active)
        return;

    slot.active = false;
    if (slot.dialog == nullptr)
        return;

    const bool wasConnecting = slot.operation == CameraWaitOperation::Connecting;
    slot.dialog->finish();
    slot.dialog->deleteLater();
    slot.dialog = nullptr;

    if (wasConnecting)
        setConnectDisplayPaused(false);
}

void CameraPanelController::dismissAllCameraOperationWaits()
{
    for (std::size_t i = 0; i < 2; ++i)
        dismissCameraOperationWait(i);
}

void CameraPanelController::updateStreamPaneTitles(LumoCameraUi &ui)
{
    const int width = ui.frameWidth;
    const int height = ui.frameHeight;

    if (ui.detectorPane != nullptr)
    {
        if (width > 0 && height > 0)
        {
            ui.detectorPane->setTitle(
                QStringLiteral("Detector (%1 \u00D7 %2)").arg(width).arg(height));
        }
        else
            ui.detectorPane->setTitle(QStringLiteral("Detector"));
    }

    updateProfilePaneTitles(ui);
}

void CameraPanelController::updateProfilePaneTitles(LumoCameraUi &ui)
{
    const int width = ui.frameWidth;
    const int bands = ui.frameHeight;

    int spatialIndex = 0;
    int bandIndex = 0;
    if (ui.detectorView != nullptr && width > 0 && bands > 0)
    {
        spatialIndex = ui.detectorView->spatialIndex();
        bandIndex = ui.detectorView->bandIndex();
    }

    if (ui.wavelengthPane != nullptr)
    {
        if (bands > 0 && width > 0)
        {
            ui.wavelengthPane->setTitle(
                QStringLiteral("Wavelength (%1 bands @ %2)")
                    .arg(bands)
                    .arg(formatOrdinalPixel(spatialIndex)));
        }
        else if (bands > 0)
            ui.wavelengthPane->setTitle(QStringLiteral("Wavelength (%1 bands)").arg(bands));
        else
            ui.wavelengthPane->setTitle(QStringLiteral("Wavelength"));
    }

    if (ui.pixelStreamPane != nullptr)
    {
        if (width > 0 && bands > 0)
        {
            const std::vector<double> &wlLookup = ui::cachedWavelengthNmLookup(
                ui.spectralBands, bands, ui.wavelengthAxisCache, ui.wavelengthAxisBandCount);
            double wavelengthNm = 0.0;
            if (bandIndex >= 0 && bandIndex < static_cast<int>(wlLookup.size()))
                wavelengthNm = wlLookup[static_cast<std::size_t>(bandIndex)];

            ui.pixelStreamPane->setTitle(
                QStringLiteral("Pixel (%1 positions @ %2 nm)")
                    .arg(width)
                    .arg(wavelengthNm, 0, 'f', 2));
        }
        else if (width > 0)
            ui.pixelStreamPane->setTitle(QStringLiteral("Pixel (%1 positions)").arg(width));
        else
            ui.pixelStreamPane->setTitle(QStringLiteral("Pixel"));
    }
}

void CameraPanelController::setupStreamPipeline()
{
    streamPipeline_ = std::make_unique<ui::CameraStreamPipeline>(host_);
    streamPipeline_->setCameraUi(0, &host_->camera1Ui_);
    streamPipeline_->setCameraUi(1, &host_->camera2Ui_);

    ui::CameraStreamDisplayHooks hooks;
    hooks.applyDetectorImage = [this](LumoCameraUi &ui, const QImage &image) {
        applyDetectorDisplay(ui, image);
    };
    hooks.applyWaterfallImage = [this](LumoCameraUi &ui,
                                       QImage image,
                                       const ui::WaterfallDisplayTarget target) {
        applyWaterfallDisplay(ui, std::move(image), target);
    };
    hooks.applyProfiles = [this](LumoCameraUi &ui, const ui::ProfileExtraction &profiles) {
        applyProfileDisplay(ui, profiles);
    };
    hooks.isStreamTabVisible = [this](const std::size_t cameraIndex) {
        if (host_->streamTabs_ == nullptr)
            return true;
        return host_->streamTabs_->currentIndex() == static_cast<int>(cameraIndex);
    };
    streamPipeline_->setDisplayHooks(std::move(hooks));

    if (host_->streamTabs_ != nullptr)
    {
        connect(host_->streamTabs_,
                &QTabWidget::currentChanged,
                this,
                [this](const int index) {
                    refreshWaterfallDisplayTargets();
                    if (index == MainWindow::kStreamTabCapture && host_->capturePanel() != nullptr)
                        host_->capturePanel()->onCaptureStreamTabActivated();
                });
    }
    refreshWaterfallDisplayTargets();

    for (LumoCameraUi *ui : {&host_->camera1Ui_, &host_->camera2Ui_})
    {
        if (ui == nullptr)
            continue;

        syncWaterfallBands(*ui);

        if (ui::ProfileProcessor *processor = profileProcessorFor(*ui))
        {
            if (ui->detectorView != nullptr)
            {
                ui::ProfileCursor cursor;
                cursor.spatialX = ui->detectorView->spatialIndex();
                cursor.bandY = ui->detectorView->bandIndex();
                processor->setCursor(cursor);
            }
            syncProfileRgbMarkers(*ui);
        }
    }

    streamPipeline_->start();
}

ui::WaterfallProcessor *CameraPanelController::waterfallProcessorFor(const LumoCameraUi &ui)
{
    if (streamPipeline_ == nullptr)
        return nullptr;
    return streamPipeline_->waterfallProcessor(ui.cameraIndex);
}

ui::ProfileProcessor *CameraPanelController::profileProcessorFor(const LumoCameraUi &ui)
{
    if (streamPipeline_ == nullptr)
        return nullptr;
    return streamPipeline_->profileProcessor(ui.cameraIndex);
}

void CameraPanelController::noteStreamFrame(const FramePacket &frame)
{
    const std::size_t cameraIndex =
        frame.source == CameraBackendId::Camera1 ? 0U : 1U;
    if (cameraIndex < 2)
        streamFpsTrackers_[cameraIndex].noteFrame();
}

void CameraPanelController::refreshAcquisitionFpsOverlays()
{
    for (LumoCameraUi *ui : {&host_->camera1Ui_, &host_->camera2Ui_})
    {
        if (ui == nullptr || ui->detectorView == nullptr)
            continue;

        double fps = 0.0;
        if (ui->cameraIndex < 2)
            fps = streamFpsTrackers_[ui->cameraIndex].fps();

        if (fps <= 0.0 && ui->cameraIndex < 2)
            fps = sdkFrameRateHz_[ui->cameraIndex];

        ui->detectorView->setAcquisitionFps(fps);
    }

    refreshSessionUptimeLabels();
}

void CameraPanelController::syncSessionUptimeClock(LumoCameraUi &ui, const CameraState state)
{
    if (ui.cameraIndex >= 2)
        return;

    const bool sessionActive = isSessionActive(state);
    const bool wasActive = sessionUptimeActive_[ui.cameraIndex];

    if (sessionActive && !wasActive)
    {
        sessionUptimeTimers_[ui.cameraIndex].restart();
        sessionUptimeActive_[ui.cameraIndex] = true;
    }
    else if (!sessionActive && wasActive)
    {
        sessionUptimeActive_[ui.cameraIndex] = false;
    }

    if (ui.sessionUptimeLabel == nullptr)
        return;

    if (!sessionActive)
        ui.sessionUptimeLabel->setText(QStringLiteral("\u2014"));
    else if (!sessionUptimeActive_[ui.cameraIndex])
        ui.sessionUptimeLabel->setText(QStringLiteral("00:00"));
    else
        ui.sessionUptimeLabel->setText(formatSessionUptime(sessionUptimeTimers_[ui.cameraIndex].elapsed()));
}

void CameraPanelController::refreshSessionUptimeLabels()
{
    for (LumoCameraUi *ui : {&host_->camera1Ui_, &host_->camera2Ui_})
    {
        if (ui == nullptr || ui->sessionUptimeLabel == nullptr || ui->cameraIndex >= 2)
            continue;

        if (!sessionUptimeActive_[ui->cameraIndex] || !isSessionActive(ui->state))
            continue;

        ui->sessionUptimeLabel->setText(formatSessionUptime(sessionUptimeTimers_[ui->cameraIndex].elapsed()));
    }
}

void CameraPanelController::pollSdkFrameRates()
{
    if (anyCameraOperationWaitActive())
        return;

    for (LumoCameraUi *ui : {&host_->camera1Ui_, &host_->camera2Ui_})
    {
        if (ui == nullptr || ui->camera == nullptr || ui->cameraIndex >= 2)
            continue;

        if (ui->state != CameraState::Streaming && ui->state != CameraState::Armed
            && ui->state != CameraState::Configured)
            continue;

        double hz = 0.0;
        CameraError error;
        if (ui->camera->readAppliedFrameRateHz(hz, error))
            sdkFrameRateHz_[ui->cameraIndex] = hz;
    }
}

void CameraPanelController::syncStreamDisplayLoad()
{
    if (streamPipeline_ == nullptr)
        return;

    streamPipeline_->setDisplayIntervalMs(33);
}

void CameraPanelController::onStreamFrame(const SharedFramePacket &frame)
{
    if (!frame)
        return;

    if (host_->capturePanel() != nullptr)
        host_->capturePanel()->onStreamFrame(frame);

    if (!host_->isCaptureSessionActive())
        return;

    LumoCameraUi *ui = nullptr;
    if (frame->source == CameraBackendId::Camera1)
        ui = &host_->camera1Ui_;
    else if (frame->source == CameraBackendId::Camera2)
        ui = &host_->camera2Ui_;

    if (ui == nullptr)
        return;

    applyDetectorDisplay(*ui, ui::framePacketToQImage(*frame));
}

void CameraPanelController::syncWaterfallBands(LumoCameraUi &ui)
{
    ui::WaterfallProcessor *processor = waterfallProcessorFor(ui);
    if (processor == nullptr)
        return;

    const CameraSettings settings = buildSettings(ui);
    const int spectralBin = std::max(1, settings.spectralBinning);
    const int frameBands = ui.frameHeight > 0 ? ui.frameHeight : 0;

    ui::RgbBandIndices bands;
    bands.red = settings.redBandIndex;
    bands.green = settings.greenBandIndex;
    bands.blue = settings.blueBandIndex;
    if (frameBands > 0)
    {
        bands.red = ui::mapCalpackBandToBilRow(bands.red, frameBands, spectralBin);
        bands.green = ui::mapCalpackBandToBilRow(bands.green, frameBands, spectralBin);
        bands.blue = ui::mapCalpackBandToBilRow(bands.blue, frameBands, spectralBin);
    }
    processor->setBandIndices(bands);
}

void CameraPanelController::syncWaterfallMaxLines(LumoCameraUi &ui)
{
    ui::WaterfallProcessor *processor = waterfallProcessorFor(ui);
    if (processor == nullptr || ui.frameWidth <= 0)
        return;

    int maxNeeded = 64;
    const auto considerPane = [&](QWidget *pane) {
        if (pane == nullptr)
            return;
        const QSize paneSize = pane->size();
        if (paneSize.width() <= 0 || paneSize.height() <= 0)
            return;
        const int neededLines = static_cast<int>(std::ceil(
            static_cast<double>(paneSize.height()) * static_cast<double>(ui.frameWidth)
            / static_cast<double>(paneSize.width())));
        maxNeeded = std::max(maxNeeded, neededLines);
    };

    considerPane(ui.waterfallView);
    if (ui.cameraIndex < 2)
        considerPane(host_->captureWaterfallViews_[ui.cameraIndex]);

    constexpr int kMinLines = 64;
    constexpr int kMaxLinesCap = 8192;
    processor->setMaxLines(std::clamp(maxNeeded, kMinLines, kMaxLinesCap));
}

void CameraPanelController::wireWaterfallPaneResizeHandlers()
{
    const auto connectPane = [this](LumoCameraUi &ui, ui::WaterfallDisplayWidget *widget) {
        if (widget == nullptr)
            return;

        connect(widget,
                &ui::WaterfallDisplayWidget::paneGeometryChanged,
                this,
                [this, &ui]() { syncWaterfallMaxLines(ui); });
    };

    connectPane(host_->camera1Ui_, host_->camera1Ui_.waterfallView);
    connectPane(host_->camera2Ui_, host_->camera2Ui_.waterfallView);
    if (host_->captureWaterfallViews_[0] != nullptr)
        connectPane(host_->camera1Ui_, host_->captureWaterfallViews_[0]);
    if (host_->captureWaterfallViews_[1] != nullptr)
        connectPane(host_->camera2Ui_, host_->captureWaterfallViews_[1]);
}

void CameraPanelController::syncProfileRgbMarkers(LumoCameraUi &ui)
{
    if (ui.wavelengthView == nullptr)
        return;

    const CameraSettings settings = buildSettings(ui);
    const int spectralBin = std::max(1, settings.spectralBinning);
    const int frameBands = ui.frameHeight > 0 ? ui.frameHeight : 0;

    int red = settings.redBandIndex;
    int green = settings.greenBandIndex;
    int blue = settings.blueBandIndex;
    if (frameBands > 0)
    {
        red = ui::mapCalpackBandToBilRow(red, frameBands, spectralBin);
        green = ui::mapCalpackBandToBilRow(green, frameBands, spectralBin);
        blue = ui::mapCalpackBandToBilRow(blue, frameBands, spectralBin);
    }

    ui.wavelengthView->setRgbBandMarkers(red, green, blue);
}

void CameraPanelController::onProfileLinesChanged(LumoCameraUi &ui,
                                       const int spatialIndex,
                                       const int bandIndex)
{
    Q_UNUSED(spatialIndex);
    Q_UNUSED(bandIndex);

    updateProfilePaneTitles(ui);

    ui::ProfileProcessor *processor = profileProcessorFor(ui);
    if (processor == nullptr)
        return;

    ui::ProfileCursor cursor;
    cursor.spatialX = spatialIndex;
    cursor.bandY = bandIndex;
    processor->setCursor(cursor);
    processor->requestRefresh();
}

void CameraPanelController::applyProfileDisplay(LumoCameraUi &ui, const ui::ProfileExtraction &profiles)
{
    updateProfilePlots(ui, profiles);
}

void CameraPanelController::applyDetectorDisplay(LumoCameraUi &ui, const QImage &image)
{
    if (image.isNull() || ui.detectorView == nullptr)
        return;

    if (ui.frameWidth != image.width() || ui.frameHeight != image.height())
    {
        ui.frameWidth = image.width();
        ui.frameHeight = image.height();
        ui.wavelengthAxisBandCount = -1;
        ui.detectorView->setFrameSize(image.width(), image.height());
        updateStreamPaneTitles(ui);
        syncWaterfallBands(ui);
        syncWaterfallMaxLines(ui);
        syncProfileRgbMarkers(ui);
    }

    ui.detectorView->setDetectorImage(image);
}

void CameraPanelController::refreshWaterfallDisplayTargets()
{
    constexpr int kWaterfallPublishActiveMs = 33;
    constexpr int kWaterfallPublishIdleMs = 1000;

    const bool captureActive = host_->isCaptureSessionActive();

    for (std::size_t cameraIndex = 0; cameraIndex < 2; ++cameraIndex)
    {
        ui::WaterfallProcessor *processor = streamPipeline_ != nullptr
                                                ? streamPipeline_->waterfallProcessor(cameraIndex)
                                                : nullptr;
        if (processor == nullptr)
            continue;

        const LumoCameraUi *ui = cameraUiForIndex(cameraIndex);
        const bool cameraSession = ui != nullptr && isSessionActive(ui->state);
        const int intervalMs = (cameraSession || captureActive) ? kWaterfallPublishActiveMs
                                                                  : kWaterfallPublishIdleMs;
        processor->setPublishIntervalMs(intervalMs);
        republishGlobalWaterfallViews(cameraIndex);
    }
}

void CameraPanelController::publishWaterfallToAllViews(std::size_t cameraIndex,
                                                       std::shared_ptr<const QImage> image)
{
    if (image == nullptr || image->isNull())
        return;

    LumoCameraUi *ui = cameraUiForIndex(cameraIndex);
    if (ui != nullptr)
        syncWaterfallMaxLines(*ui);

    if (ui != nullptr && ui->waterfallView != nullptr)
        ui->waterfallView->setSharedImage(image);

    if (cameraIndex < 2 && host_->captureWaterfallViews_[cameraIndex] != nullptr)
        host_->captureWaterfallViews_[cameraIndex]->setSharedImage(image);
}

void CameraPanelController::republishGlobalWaterfallViews(const std::size_t cameraIndex)
{
    if (cameraIndex >= globalWaterfallImages_.size())
        return;

    const std::shared_ptr<const QImage> &image = globalWaterfallImages_[cameraIndex];
    if (image == nullptr || image->isNull())
        return;

    publishWaterfallToAllViews(cameraIndex, image);
}

void CameraPanelController::applyWaterfallDisplay(LumoCameraUi &ui,
                                                  QImage image,
                                                  const ui::WaterfallDisplayTarget target)
{
    Q_UNUSED(target);
    if (image.isNull())
        return;

    const std::shared_ptr<const QImage> shared = std::make_shared<QImage>(std::move(image));
    if (ui.cameraIndex < globalWaterfallImages_.size())
        globalWaterfallImages_[ui.cameraIndex] = shared;

    publishWaterfallToAllViews(ui.cameraIndex, shared);
}

void CameraPanelController::updateProfilePlots(LumoCameraUi &ui, const ui::ProfileExtraction &profiles)
{
    if (!profiles.valid)
        return;

    if (ui.wavelengthView != nullptr && !profiles.wavelengthDn.empty())
    {
        const int bandMax = std::max(0, profiles.frameBands - 1);
        const std::vector<double> &wavelengthAxis = ui::cachedWavelengthNmLookup(
            ui.spectralBands, profiles.frameBands, ui.wavelengthAxisCache, ui.wavelengthAxisBandCount);
        ui.wavelengthView->setWavelengthAxis(wavelengthAxis);
        ui.wavelengthView->setProfile(profiles.wavelengthDn, bandMax);
    }

    if (ui.pixelStreamView != nullptr && !profiles.spatialDn.empty())
        ui.pixelStreamView->setProfile(profiles.spatialDn, std::max(0, profiles.frameWidth - 1));
}

void CameraPanelController::updateWaterfallView(LumoCameraUi &ui,
                                                const QImage &image,
                                                const ui::WaterfallDisplayTarget target)
{
    Q_UNUSED(target);
    applyWaterfallDisplay(ui, image, ui::WaterfallDisplayTarget::StreamTab);
}

void CameraPanelController::clearDetectorView(LumoCameraUi &ui)
{
    ui.frameWidth = 0;
    ui.frameHeight = 0;
    ui.wavelengthAxisBandCount = -1;
    ui.wavelengthAxisCache.clear();
    if (ui.cameraIndex < 2)
    {
        streamFpsTrackers_[ui.cameraIndex].reset();
        sdkFrameRateHz_[ui.cameraIndex] = 0.0;
    }
    updateStreamPaneTitles(ui);

    if (ui::WaterfallProcessor *processor = waterfallProcessorFor(ui))
        processor->reset();
    if (ui::ProfileProcessor *profileProcessor = profileProcessorFor(ui))
        profileProcessor->reset();

    if (ui.cameraIndex < 2)
        globalWaterfallImages_[ui.cameraIndex].reset();

    const QString cameraName = QStringLiteral("Camera %1").arg(ui.cameraIndex + 1);
    if (ui.detectorView != nullptr)
        ui.detectorView->clearDisplay(cameraName + QStringLiteral(" detector (disconnected)"));
    if (ui.waterfallView != nullptr)
        ui::setWaterfallDisconnectedText(ui.waterfallView, QStringLiteral("waterfall"), cameraName);
    if (ui.cameraIndex < 2 && host_->captureWaterfallViews_[ui.cameraIndex] != nullptr)
    {
        ui::setWaterfallDisconnectedText(host_->captureWaterfallViews_[ui.cameraIndex],
                                         QStringLiteral("waterfall"),
                                         profileTabNameForUi(ui));
    }
    if (ui.wavelengthView != nullptr)
        ui.wavelengthView->clearDisplay(cameraName + QStringLiteral(" wavelength (disconnected)"));
    if (ui.pixelStreamView != nullptr)
        ui.pixelStreamView->clearDisplay(cameraName + QStringLiteral(" pixel (disconnected)"));
}

void CameraPanelController::updateCameraControls(LumoCameraUi &ui, const CameraState state)
{
    ui.state = state;

    const bool connected = state != CameraState::Disconnected && state != CameraState::Fault;
    const bool captureActive = host_->isCaptureSessionActive();

    if (captureActive)
    {
        if (ui.connectBtn != nullptr)
        {
            ui.connectBtn->setEnabled(false);
            ui.connectBtn->setText(connected ? QStringLiteral("Disconnect camera")
                                             : QStringLiteral("Connect camera"));
        }
        if (ui.deviceCombo != nullptr)
            ui.deviceCombo->setEnabled(false);
        if (ui.calibrationPackEdit != nullptr)
            ui.calibrationPackEdit->setEnabled(false);
        if (ui.calibrationPackBrowseBtn != nullptr)
            ui.calibrationPackBrowseBtn->setEnabled(false);
        if (ui.shutterToggleBtn != nullptr)
            ui.shutterToggleBtn->setEnabled(false);
        if (ui.spectralBinningCombo != nullptr)
            ui.spectralBinningCombo->setEnabled(false);
        if (ui.spatialBinningCombo != nullptr)
            ui.spatialBinningCombo->setEnabled(false);
        if (ui.exposureSpin != nullptr)
            ui.exposureSpin->setEnabled(false);
        if (ui.frameRateSpin != nullptr)
            ui.frameRateSpin->setEnabled(false);
        if (ui.redBandCombo != nullptr)
            ui.redBandCombo->setEnabled(false);
        if (ui.greenBandCombo != nullptr)
            ui.greenBandCombo->setEnabled(false);
        if (ui.blueBandCombo != nullptr)
            ui.blueBandCombo->setEnabled(false);
        if (ui.applyBtn != nullptr)
            ui.applyBtn->setEnabled(false);
        syncSessionUptimeClock(ui, state);
        return;
    }

    if (ui.connectBtn != nullptr)
    {
        ui.connectBtn->setEnabled(true);
        ui.connectBtn->setText(connected ? QStringLiteral("Disconnect camera")
                                         : QStringLiteral("Connect camera"));
    }

    if (ui.deviceCombo != nullptr)
        ui.deviceCombo->setEnabled(!connected);

    if (ui.calibrationPackEdit != nullptr)
        ui.calibrationPackEdit->setEnabled(!connected);
    if (ui.calibrationPackBrowseBtn != nullptr)
        ui.calibrationPackBrowseBtn->setEnabled(!connected);

    const bool readyForCameraFeatures = state == CameraState::Initialized || state == CameraState::Configured
                                        || state == CameraState::Armed || state == CameraState::Streaming
                                        || state == CameraState::SafeStopped;
    if (ui.shutterToggleBtn != nullptr)
        ui.shutterToggleBtn->setEnabled(readyForCameraFeatures);
    if (ui.spectralBinningCombo != nullptr)
        ui.spectralBinningCombo->setEnabled(readyForCameraFeatures);
    if (ui.spatialBinningCombo != nullptr)
        ui.spatialBinningCombo->setEnabled(readyForCameraFeatures);
    if (ui.exposureSpin != nullptr)
        ui.exposureSpin->setEnabled(readyForCameraFeatures);
    if (ui.frameRateSpin != nullptr)
    {
        bool frameRateEnabled = readyForCameraFeatures;
        if (ui.sensorKind == LumoSensorKind::Swir3Ni && host_->capturePanel() != nullptr
            && host_->capturePanel()->shouldLockSwir3ForDualSync())
            frameRateEnabled = false;
        ui.frameRateSpin->setEnabled(frameRateEnabled);
    }
    if (ui.redBandCombo != nullptr)
        ui.redBandCombo->setEnabled(readyForCameraFeatures);
    if (ui.greenBandCombo != nullptr)
        ui.greenBandCombo->setEnabled(readyForCameraFeatures);
    if (ui.blueBandCombo != nullptr)
        ui.blueBandCombo->setEnabled(readyForCameraFeatures);

    const bool readyForApply = state == CameraState::Initialized || state == CameraState::Configured
                               || state == CameraState::Armed || state == CameraState::SafeStopped
                               || state == CameraState::Streaming;
    if (ui.applyBtn != nullptr)
        ui.applyBtn->setEnabled(readyForApply);

    syncSessionUptimeClock(ui, state);
}

} // namespace hf::camera
