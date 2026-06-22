// Camera settings tab and per-sensor Lumo configuration group.
// MainWindow method definitions extracted from MainWindow.cpp for clarity.
#include "frontend/controllers/CameraPanelController.hpp"
#include "frontend/controllers/UiSettingsController.hpp"
#include "frontend/widgets/MainWindow.hpp"
#include "adapters/lumo/LumoCamera.hpp"
#include "adapters/lumo/LumoDeviceTypes.hpp"
#include "backend/CameraCoordinator.hpp"
#include "backend/HyperFusionConfig.hpp"
#include "frontend/processing/Overexposure.hpp"
#include "frontend/widgets/MainWindowTabHelpers.hpp"
#include "frontend/widgets/OperationWaitDialog.hpp"
#include "frontend/processing/ProfileProcessor.hpp"
#include "frontend/processing/WaterfallProcessor.hpp"

#include <QComboBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSignalBlocker>
#include <QTabWidget>
#include <QTimer>
#include <QVBoxLayout>
QWidget *MainWindow::createCameraSettingsTab()
{
    auto *page = new QWidget(this);
    auto *layout = new QVBoxLayout(page);
    layout->setContentsMargins(0, 0, 0, 0);

    cameraSettingsTabs_ = new QTabWidget(page);
    cameraSettingsTabs_->addTab(createLumoCameraGroup(cameraSettingsTabs_, camera1Ui_, LumoSensorKind::Fx10ePleora),
                                QStringLiteral("FX10e"));
    cameraSettingsTabs_->addTab(createLumoCameraGroup(cameraSettingsTabs_, camera2Ui_, LumoSensorKind::Swir3Ni),
                                QStringLiteral("SWIR3"));

    layout->addWidget(cameraSettingsTabs_, 1);
    return page;
}

QWidget *MainWindow::createLumoCameraGroup(QWidget *parent,
                                           LumoCameraUi &ui,
                                           const LumoSensorKind sensorKind)
{
    ui.sensorKind = sensorKind;

    auto *page = new QWidget(parent);
    auto *form = new QFormLayout(page);
    form->setContentsMargins(8, 8, 8, 8);

    ui.deviceCombo = new QComboBox(page);
    ui.deviceCombo->setMinimumWidth(260);

    ui.calibrationPackEdit = new QLineEdit(page);
    ui.calibrationPackEdit->setMinimumWidth(0);
    ui.calibrationPackEdit->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

    ui.calibrationPackBrowseBtn = new QPushButton(QStringLiteral("Browse\u2026"), page);
    ui.calibrationPackBrowseBtn->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    ui.calibrationPackBrowseBtn->setFixedWidth(72);

    {
        const QString initialCalpack =
            hf::camera::CameraPanelController::defaultCalibrationPackPathForProfile(QString(), sensorKind);
        if (!initialCalpack.isEmpty())
            hf::camera::CameraPanelController::setCalibrationPackDisplay(ui.calibrationPackEdit, initialCalpack);
    }

    auto *calibrationRow = new QWidget(page);
    auto *calibrationLayout = new QHBoxLayout(calibrationRow);
    calibrationLayout->setContentsMargins(0, 0, 0, 0);
    calibrationLayout->setSpacing(6);
    calibrationLayout->addWidget(ui.calibrationPackEdit, 1);
    calibrationLayout->addWidget(ui.calibrationPackBrowseBtn, 0);

    auto *shutterRow = new QWidget(page);
    auto *shutterLayout = new QHBoxLayout(shutterRow);
    shutterLayout->setContentsMargins(0, 0, 0, 0);
    shutterLayout->setSpacing(8);

    ui.shutterIndicator = new QLabel(shutterRow);
    ui.shutterIndicator->setFixedSize(14, 14);
    ui.shutterStatusLabel = new QLabel(QStringLiteral("\u2014"), shutterRow);
    ui.shutterToggleBtn = new QPushButton(QStringLiteral("Toggle"), shutterRow);
    ui.shutterToggleBtn->setEnabled(false);
    shutterLayout->addWidget(ui.shutterIndicator);
    shutterLayout->addWidget(ui.shutterStatusLabel, 1);
    shutterLayout->addWidget(ui.shutterToggleBtn);

    ui.frameRateSpin = new QDoubleSpinBox(page);
    ui.frameRateSpin->setRange(1.0, 500.0);
    ui.frameRateSpin->setDecimals(2);
    ui.frameRateSpin->setValue(50.0);
    ui.frameRateSpin->setSuffix(QStringLiteral(" Hz"));

    ui.exposureSpin = new QDoubleSpinBox(page);
    ui.exposureSpin->setRange(0.01, 400.0);
    ui.exposureSpin->setDecimals(2);
    ui.exposureSpin->setSingleStep(0.1);
    ui.exposureSpin->setValue(18.0);
    ui.exposureSpin->setSuffix(QStringLiteral(" ms"));

    ui.spectralBinningCombo = new QComboBox(page);
    ui.spatialBinningCombo = new QComboBox(page);
    for (QComboBox *binningCombo : {ui.spectralBinningCombo, ui.spatialBinningCombo})
    {
        binningCombo->addItems({QStringLiteral("1"),
                                QStringLiteral("2"),
                                QStringLiteral("4"),
                                QStringLiteral("8")});
    }
    if (ui.spectralBinningCombo != nullptr)
    {
        connect(ui.spectralBinningCombo,
                &QComboBox::currentIndexChanged,
                this,
                [this, &ui](const int) { cameraPanel()->refreshBandCombos(ui); });
    }

    ui.redBandCombo = new QComboBox(page);
    ui.greenBandCombo = new QComboBox(page);
    ui.blueBandCombo = new QComboBox(page);
    for (QComboBox *combo : {ui.redBandCombo, ui.greenBandCombo, ui.blueBandCombo})
        combo->setMinimumWidth(220);

    ui.connectBtn = new QPushButton("Connect camera", page);
    ui.applyBtn = new QPushButton("Apply settings", page);

    ui.applyBtn->setEnabled(false);

    ui.sessionUptimeLabel = new QLabel(QStringLiteral("\u2014"), page);
    ui.sessionUptimeLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);

    form->addRow("Sensor profile", ui.deviceCombo);
    form->addRow("Calibration pack", calibrationRow);
    form->addRow("Shutter", shutterRow);
    form->addRow("Frame rate (Hz)", ui.frameRateSpin);
    form->addRow("Exposure time (ms)", ui.exposureSpin);
    form->addRow("Spectral binning", ui.spectralBinningCombo);
    form->addRow("Spatial binning", ui.spatialBinningCombo);
    form->addRow("Red band", ui.redBandCombo);
    form->addRow("Green band", ui.greenBandCombo);
    form->addRow("Blue band", ui.blueBandCombo);
    form->addRow(QStringLiteral("Session uptime"), ui.sessionUptimeLabel);
    form->addRow("", ui.connectBtn);
    form->addRow("", ui.applyBtn);

    connect(ui.calibrationPackBrowseBtn, &QPushButton::clicked, this, [this, &ui]() {
        if (ui.calibrationPackEdit == nullptr)
            return;

        const QString currentPath = hf::camera::CameraPanelController::calibrationPackPath(ui);
        const QString profileName =
            ui.deviceCombo != nullptr ? ui.deviceCombo->currentText() : QString();
        const QString profileDefault =
            hf::camera::CameraPanelController::defaultCalibrationPackPathForProfile(profileName, ui.sensorKind);
        const QString startDir =
            currentPath.isEmpty()
                ? (profileDefault.isEmpty() ? QDir::homePath()
                                            : QFileInfo(profileDefault).absolutePath())
                : QFileInfo(currentPath).absolutePath();
        const QString path = QFileDialog::getOpenFileName(
            this,
            QStringLiteral("Select calibration pack"),
            startDir,
            QStringLiteral("Specim calibration (*.scp);;All files (*.*)"));
        if (!path.isEmpty())
        {
            hf::camera::CameraPanelController::setCalibrationPackDisplay(ui.calibrationPackEdit, path);
            cameraPanel()->refreshBandCombos(ui);
        }
    });

    connect(ui.deviceCombo,
            &QComboBox::currentIndexChanged,
            this,
            [this, &ui](const int) {
                cameraPanel()->updateTabLabel(ui);
                cameraPanel()->syncCalibrationPackToSelectedProfile(ui);
                settingsPanel()->schedulePersistedUiSettingsSave();
            });

    connect(ui.connectBtn, &QPushButton::clicked, this, [this, &ui]() {
        if (!coordinator() || !ui.camera)
            return;

        const QString cameraLabel = cameraPanel()->profileTabNameForUi(ui);
        const bool disconnectRequested =
            ui.state != CameraState::Disconnected && ui.state != CameraState::Fault;

        if (disconnectRequested)
        {
            ui.connectAttemptActive = false;
            cameraPanel()->showCameraOperationWait(ui.cameraIndex,
                                                     hf::camera::CameraWaitOperation::Disconnecting);
            if (ui.connectBtn != nullptr)
            {
                ui.connectBtn->setEnabled(false);
                ui.connectBtn->setText(QStringLiteral("Disconnecting\u2026"));
            }
            coordinator()->disconnectOnGuiThread(ui.cameraIndex);
            appendLog(QString("%1: disconnect requested.").arg(cameraLabel));
            return;
        }

        if (ui.deviceCombo->count() == 0)
        {
            appendLog(QString("%1: refresh SSP profiles before connecting.").arg(cameraLabel));
            return;
        }

        const QString profileName = ui.deviceCombo->currentText();
        if (ui.sensorKind == LumoSensorKind::Fx10ePleora && !ui::lumoProfileMatchesFx10eSlot(profileName))
        {
            appendLog(QStringLiteral(
                "FX10e: this tab only supports FX10e / Pleora SSP profiles \u2014 pick a profile from the FX10e list."));
            return;
        }
        if (ui.sensorKind == LumoSensorKind::Swir3Ni && !ui::lumoProfileMatchesSwir3Slot(profileName))
        {
            appendLog(QStringLiteral(
                "SWIR3: this tab only supports SWIR / NI SSP profiles \u2014 pick a profile from the SWIR3 list."));
            return;
        }

        const CameraSettings connectionSettings = cameraPanel()->buildSettings(ui);
        ui.camera->prepareConnection(connectionSettings);
        ui.connectAttemptActive = true;
        if (ui.sensorKind == LumoSensorKind::Swir3Ni)
        {
            const std::string icdPath =
                LumoCamera::resolveNiImaqCameraFilePath(connectionSettings.niImaqCameraFile);
            const bool icdExists = QFileInfo::exists(QString::fromStdString(icdPath));
            if (camera1Ui_.state != CameraState::Disconnected
                && camera1Ui_.state != CameraState::Fault)
            {
                appendLog(QStringLiteral(
                    "SWIR3: FX10e is still connected \u2014 disconnect it before SWIR3 NI bring-up."));
            }

            appendLog(QStringLiteral("SWIR3 NI: Grabber=%1, Camera.Channel=%2, ICD=%3 (exists=%4)")
                          .arg(QString::fromStdString(connectionSettings.niGrabberChannel),
                               QStringLiteral("(autoconnect)"),
                               QString::fromStdString(icdPath),
                               icdExists ? QStringLiteral("yes") : QStringLiteral("NO \u2014 rebuild app")));
        }
        const QString grabberNote = ui.sensorKind == LumoSensorKind::Swir3Ni
                                        ? QStringLiteral("NI IMAQdx \u2014 stop Grab in NI MAX before connect")
                                        : QStringLiteral("Pleora eBUS picker may appear");
        appendLog(QString("%1: connect camera \u2014 profile %2 (%3; not ready until Initialized).")
                      .arg(cameraLabel, ui.deviceCombo->currentText(), grabberNote));

        cameraPanel()->showCameraOperationWait(ui.cameraIndex,
                                               hf::camera::CameraWaitOperation::Connecting);

        QTimer::singleShot(0, this, [this, cameraIndex = ui.cameraIndex]() {
            if (coordinator())
                coordinator()->connectAndInitializeOnGuiThread(cameraIndex);
        });
    });

    connect(ui.shutterToggleBtn, &QPushButton::clicked, this, [this, &ui]() {
        if (!coordinator())
            return;

        if (ui.shutterReportedOpen)
            coordinator()->closeShutter(ui.cameraIndex);
        else
            coordinator()->openShutter(ui.cameraIndex);
    });

    connect(ui.applyBtn, &QPushButton::clicked, this, [this, &ui]() {
        if (!coordinator())
            return;

        const QString cameraLabel = cameraPanel()->profileTabNameForUi(ui);
        const CameraSettings settings = cameraPanel()->buildSettings(ui);
        cameraPanel()->showCameraOperationWait(ui.cameraIndex,
                                               hf::camera::CameraWaitOperation::ApplyingSettings);
        coordinator()->applySettings(ui.cameraIndex, settings);
        cameraPanel()->syncWaterfallBands(ui);
        cameraPanel()->syncProfileRgbMarkers(ui);
        if (ui::WaterfallProcessor *processor = cameraPanel()->waterfallProcessorFor(ui))
            processor->reset();

        appendLog(QString("%1: apply settings (fps=%2 Hz, exposure=%3 ms, spectral=%4, spatial=%5, "
                          "RGB=%6/%7/%8)")
                      .arg(cameraLabel)
                      .arg(settings.frameRateHz, 0, 'f', 2)
                      .arg(settings.exposureMs, 0, 'f', 2)
                      .arg(settings.spectralBinning)
                      .arg(settings.spatialBinning)
                      .arg(settings.redBandIndex)
                      .arg(settings.greenBandIndex)
                      .arg(settings.blueBandIndex));
    });

    const auto onBandSelectionChanged = [this, &ui]() {
        cameraPanel()->syncWaterfallBands(ui);
        cameraPanel()->syncProfileRgbMarkers(ui);
        if (ui::WaterfallProcessor *processor = cameraPanel()->waterfallProcessorFor(ui))
            processor->reset();
        if (ui::ProfileProcessor *profileProcessor = cameraPanel()->profileProcessorFor(ui))
            profileProcessor->requestRefresh();
    };
    if (ui.redBandCombo != nullptr)
        connect(ui.redBandCombo, &QComboBox::currentIndexChanged, this, onBandSelectionChanged);
    if (ui.greenBandCombo != nullptr)
        connect(ui.greenBandCombo, &QComboBox::currentIndexChanged, this, onBandSelectionChanged);
    if (ui.blueBandCombo != nullptr)
        connect(ui.blueBandCombo, &QComboBox::currentIndexChanged, this, onBandSelectionChanged);

    cameraPanel()->updateShutterDisplay(ui, false);
    if (!hf::camera::CameraPanelController::calibrationPackPath(ui).isEmpty())
        cameraPanel()->refreshBandCombos(ui);

    return page;
}
