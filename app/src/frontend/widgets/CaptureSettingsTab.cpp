// Capture / recorder settings tab (scan parameters, metadata, preprocessing).
// MainWindow method definitions extracted from MainWindow.cpp for clarity.
#include "frontend/controllers/CapturePanelController.hpp"
#include "frontend/controllers/CameraPanelController.hpp"
#include "frontend/controllers/UiSettingsController.hpp"
#include "frontend/widgets/MainWindow.hpp"
#include "adapters/zaber/ZaberStageProfile.hpp"
#include "backend/HyperFusionConfig.hpp"
#include "backend/camera/processing/Gsam2ServerManager.hpp"
#include "backend/stage/StageWorker.hpp"
#include "frontend/widgets/MainWindowTabHelpers.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFontMetrics>
#include <QFormLayout>
#include <QFrame>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QSpinBox>
#include <QTimer>
#include <QVBoxLayout>
QWidget *MainWindow::createCaptureSettingsTab()
{
    auto *scrollArea = new QScrollArea(this);
    scrollArea->setWidgetResizable(true);
    scrollArea->setFrameShape(QFrame::NoFrame);
    scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    auto *page = new QWidget();
    scrollArea->setWidget(page);
    ui::applyWhiteSettingsScrollBackground(scrollArea);

    auto *layout = new QVBoxLayout(page);
    layout->setContentsMargins(0, 0, 0, 0);

    auto *recorderBox = new QGroupBox("Recorder", page);
    auto *recorderLayout = new QVBoxLayout(recorderBox);
    recorderLayout->setContentsMargins(6, 4, 6, 6);

    auto *recorderButtonLayout = new QHBoxLayout();
    recorderButtonLayout->setContentsMargins(0, 0, 0, 0);
    recorderButtonLayout->setSpacing(8);

    captureRecorderStopBtn_ =
        ui::makeRecorderButton(recorderBox, ui::makeRecorderStopIcon(), QStringLiteral("Stop"));
    captureRecorderPreviewBtn_ =
        ui::makeRecorderButton(recorderBox, ui::makeRecorderPreviewIcon(), QStringLiteral("Preview"));
    captureRecorderRecordBtn_ =
        ui::makeRecorderButton(recorderBox, ui::makeRecorderRecordIcon(), QStringLiteral("Record"));
    captureRecorderStopBtn_->setToolTip(tr("Stop preview or recording"));
    captureRecorderPreviewBtn_->setToolTip(tr("Preview scan without saving"));
    captureRecorderRecordBtn_->setToolTip(tr("Record scan to SSD"));

    recorderButtonLayout->addWidget(captureRecorderStopBtn_, 1);
    recorderButtonLayout->addWidget(captureRecorderPreviewBtn_, 1);
    recorderButtonLayout->addWidget(captureRecorderRecordBtn_, 1);
    recorderLayout->addLayout(recorderButtonLayout);

    auto *recorderStatusLayout = new QHBoxLayout();
    recorderStatusLayout->setContentsMargins(0, 4, 0, 0);
    recorderStatusLayout->setSpacing(8);
    captureRecorderStatusIndicator_ = new QLabel(recorderBox);
    captureRecorderStatusIndicator_->setFixedSize(14, 14);
    captureRecorderStatusIndicator_->setToolTip(tr("Recorder activity"));
    captureRecorderStatusLabel_ = new QLabel(recorderBox);
    captureRecorderStatusLabel_->setWordWrap(true);
    captureRecorderStatusLabel_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    recorderStatusLayout->addWidget(captureRecorderStatusIndicator_);
    recorderStatusLayout->addWidget(captureRecorderStatusLabel_, 1);
    recorderLayout->addLayout(recorderStatusLayout);

    auto *cameraStatusLayout = new QVBoxLayout();
    cameraStatusLayout->setContentsMargins(22, 0, 0, 0);
    cameraStatusLayout->setSpacing(2);
    for (std::size_t cameraIndex = 0; cameraIndex < 2; ++cameraIndex)
    {
        captureRecorderCameraStatusLabels_[cameraIndex] = new QLabel(recorderBox);
        captureRecorderCameraStatusLabels_[cameraIndex]->setWordWrap(true);
        captureRecorderCameraStatusLabels_[cameraIndex]->setTextInteractionFlags(Qt::TextSelectableByMouse);
        captureRecorderCameraStatusLabels_[cameraIndex]->hide();
        cameraStatusLayout->addWidget(captureRecorderCameraStatusLabels_[cameraIndex]);
    }
    recorderLayout->addLayout(cameraStatusLayout);

    captureCamerasBox_ = new QGroupBox("Cameras", page);
    auto *camerasLayout = new QVBoxLayout(captureCamerasBox_);
    camerasLayout->setContentsMargins(6, 4, 6, 6);
    camerasLayout->setSpacing(4);
    captureCamerasEmptyLabel_ =
        new QLabel(QStringLiteral("No cameras connected."), captureCamerasBox_);
    captureCamerasEmptyLabel_->setWordWrap(true);
    captureCamera1Check_ = new QCheckBox(captureCamerasBox_);
    captureCamera2Check_ = new QCheckBox(captureCamerasBox_);
    captureBfsCheck_ = new QCheckBox(QStringLiteral("BFS"), captureCamerasBox_);
    captureCamera1Check_->hide();
    captureCamera2Check_->hide();
    captureBfsCheck_->hide();
    const auto refreshRecorder = [this]() {
        if (capturePanel() == nullptr)
            return;
        capturePanel()->updateDualCameraSyncControls();
        capturePanel()->updateRecorderControls();
        capturePanel()->updateScanningSpeedControls();
        capturePanel()->updateCaptureStreamLayout();
    };
    connect(captureCamera1Check_, &QCheckBox::toggled, this, refreshRecorder);
    connect(captureCamera2Check_, &QCheckBox::toggled, this, refreshRecorder);
    connect(captureBfsCheck_, &QCheckBox::toggled, this, [this, refreshRecorder](bool) {
        if (capturePanel() != nullptr)
            capturePanel()->syncBfsAndMultiviewRgbCaptureControls(captureBfsCheck_);
        refreshRecorder();
    });
    camerasLayout->addWidget(captureCamerasEmptyLabel_);
    camerasLayout->addWidget(captureCamera1Check_);
    camerasLayout->addWidget(captureCamera2Check_);
    camerasLayout->addWidget(captureBfsCheck_);
    captureDualCameraAutoCheck_ = new QCheckBox(
        QStringLiteral("Auto-sync FX10e and SWIR3 scan rate"), captureCamerasBox_);
    captureDualCameraAutoCheck_->setChecked(true);
    captureDualCameraAutoCheck_->setToolTip(
        tr("When both cameras are selected, use FX10e frame rate and spatial scale to set "
           "scanning speed, then match SWIR3 frame rate so both cover the same physical distance "
           "per line. Also enables Auto scanning speed."));
    captureDualCameraAutoCheck_->hide();
    camerasLayout->addWidget(captureDualCameraAutoCheck_);
    connect(captureDualCameraAutoCheck_, &QCheckBox::toggled, this, [this](const bool checked) {
        if (capturePanel() == nullptr)
            return;

        if (checked && captureScanningSpeedAutoCheck_ != nullptr)
        {
            QSignalBlocker blocker(captureScanningSpeedAutoCheck_);
            captureScanningSpeedAutoCheck_->setChecked(true);
        }

        if (checked)
        {
            capturePanel()->resetDualCameraScanSyncHardwareState();
            capturePanel()->applyDualCameraScanSync(true);
        }
        else
        {
            capturePanel()->resetDualCameraScanSyncHardwareState();
        }

        capturePanel()->updateDualCameraSyncControls();
        capturePanel()->updateRecorderControls();
        settingsPanel()->schedulePersistedUiSettingsSave();
    });

    captureModesBox_ = new QGroupBox(QStringLiteral("Modes"), page);
    captureModesBox_->setToolTip(
        tr("Requires a connected camera and stage with \"Use HyperFusion Stage for recording\" enabled."));
    auto *modesLayout = new QHBoxLayout(captureModesBox_);
    modesLayout->setContentsMargins(6, 4, 6, 6);
    modesLayout->setSpacing(16);
    captureReflectanceCheck_ = new QCheckBox(QStringLiteral("Reflectance"), captureModesBox_);
    captureTransmittanceCheck_ = new QCheckBox(QStringLiteral("Transmittance"), captureModesBox_);
    captureMultiviewRgbCheck_ = new QCheckBox(QStringLiteral("Multiview RGB"), captureModesBox_);
    captureReflectanceCheck_->setChecked(true);
    captureTransmittanceCheck_->setChecked(true);
    captureMultiviewRgbCheck_->setChecked(false);
    captureMultiviewRgbCheck_->setEnabled(false);
    captureReflectanceCheck_->setToolTip(
        tr("Include reflectance scan (requires connected camera, stage, and stage recording)"));
    captureTransmittanceCheck_->setToolTip(
        tr("Include transmittance scan (requires connected camera, stage, and stage recording)"));
    captureMultiviewRgbCheck_->setToolTip(
        tr("BFS RGB for Multiview. Enabled after a successful UR3e scan plan with reachable poses."));
    modesLayout->addWidget(captureReflectanceCheck_);
    modesLayout->addWidget(captureTransmittanceCheck_);
    modesLayout->addWidget(captureMultiviewRgbCheck_);
    modesLayout->addStretch(1);
    connect(captureReflectanceCheck_, &QCheckBox::toggled, this, refreshRecorder);
    connect(captureTransmittanceCheck_, &QCheckBox::toggled, this, refreshRecorder);
    connect(captureMultiviewRgbCheck_, &QCheckBox::toggled, this, [this, refreshRecorder](bool) {
        if (capturePanel() != nullptr)
            capturePanel()->syncBfsAndMultiviewRgbCaptureControls(captureMultiviewRgbCheck_);
        refreshRecorder();
    });

    auto *positionBox = new QGroupBox(QStringLiteral("Position"), page);
    capturePositionBox_ = positionBox;
    positionBox->setToolTip(
        tr("Requires a connected camera and stage. Configure target length, scan speed, "
           "and scanning home. Reference positions come from hyperfusion.cfg."));
    auto *positionLayout = new QVBoxLayout(positionBox);
    positionLayout->setContentsMargins(6, 4, 6, 6);
    positionLayout->setSpacing(6);

    captureUseStageForRecordingCheck_ =
        new QCheckBox(QStringLiteral("Use HyperFusion Stage for recording"), positionBox);
    captureUseStageForRecordingCheck_->setChecked(false);
    captureUseStageForRecordingCheck_->setToolTip(
        tr("When enabled, Preview and Record follow the stage scanning procedure "
           "(black ref, white ref, sample scan). When disabled, Record saves reflectance "
           "frames only and Preview is unavailable."));
    positionLayout->addWidget(captureUseStageForRecordingCheck_);
    connect(captureUseStageForRecordingCheck_, &QCheckBox::toggled, this, [this](const bool checked) {
        if (checked && !capturePanel()->isSessionActive())
        {
            const QSignalBlocker reflectanceBlocker(captureReflectanceCheck_);
            const QSignalBlocker transmittanceBlocker(captureTransmittanceCheck_);
            if (captureReflectanceCheck_ != nullptr)
                captureReflectanceCheck_->setChecked(true);
            if (captureTransmittanceCheck_ != nullptr)
                captureTransmittanceCheck_->setChecked(true);
        }
        capturePanel()->updateRecorderControls();
    });

    capturePositionContent_ = new QWidget(positionBox);
    auto *positionContentLayout = new QVBoxLayout(capturePositionContent_);
    positionContentLayout->setContentsMargins(0, 0, 0, 0);
    positionContentLayout->setSpacing(6);

    auto *targetLengthRowLayout = new QHBoxLayout();
    targetLengthRowLayout->setSpacing(6);
    auto *targetLengthLabel = new QLabel(QStringLiteral("Target length"), capturePositionContent_);
    targetLengthLabel->setMinimumWidth(96);
    captureTargetLengthSpin_ = new QDoubleSpinBox(capturePositionContent_);
    captureTargetLengthSpin_->setRange(0.0, zaber_stage::kTravelLengthMm);
    captureTargetLengthSpin_->setDecimals(2);
    captureTargetLengthSpin_->setSingleStep(1.0);
    captureTargetLengthSpin_->setSuffix(QStringLiteral(" mm"));
    captureTargetLengthSpin_->setValue(125.0);
    targetLengthRowLayout->addWidget(targetLengthLabel);
    targetLengthRowLayout->addWidget(captureTargetLengthSpin_, 1);
    positionContentLayout->addLayout(targetLengthRowLayout);

    const auto configureEditableSpeedSpin = [](QDoubleSpinBox *spin, const QString &tooltip) {
        spin->setRange(0.0, zaber_stage::kMaxSpeedMmPerSec);
        spin->setDecimals(1);
        spin->setSingleStep(1.0);
        spin->setSuffix(QStringLiteral(" mm/s"));
        spin->setToolTip(tooltip);
    };

    auto *scanningSpeedRowLayout = new QHBoxLayout();
    scanningSpeedRowLayout->setSpacing(6);
    auto *scanningSpeedLabel =
        new QLabel(QStringLiteral("Scanning speed"), capturePositionContent_);
    scanningSpeedLabel->setMinimumWidth(96);
    captureScanningSpeedSpin_ = new QDoubleSpinBox(capturePositionContent_);
    configureEditableSpeedSpin(
        captureScanningSpeedSpin_,
        QStringLiteral("White-reference and sample scan speed. Uncheck Auto to override for this session."));
    captureScanningSpeedSpin_->setValue(15.0);
    captureScanningSpeedAutoCheck_ = new QCheckBox(QStringLiteral("Auto"), capturePositionContent_);
    captureScanningSpeedAutoCheck_->setChecked(true);
    captureScanningSpeedAutoCheck_->setToolTip(
        QStringLiteral("Record scan speed = frame rate (Hz) \u00D7 spatial_mm_per_pixel \u00D7 spatial binning "
                       "from hyperfusion.cfg. With dual-camera sync, FX10e is the reference. "
                       "Otherwise uses the slowest selected camera when multiple are active."));
    connect(captureScanningSpeedAutoCheck_, &QCheckBox::toggled, this, [this]() {
        capturePanel()->updateScanningSpeedControls();
    });
    scanningSpeedRowLayout->addWidget(scanningSpeedLabel);
    scanningSpeedRowLayout->addWidget(captureScanningSpeedSpin_, 1);
    scanningSpeedRowLayout->addWidget(captureScanningSpeedAutoCheck_);
    scanningSpeedRowLayout->addSpacing(40);
    positionContentLayout->addLayout(scanningSpeedRowLayout);

    auto *scanningHomeRowLayout = new QHBoxLayout();
    scanningHomeRowLayout->setSpacing(6);
    auto *scanningHomeLabel =
        new QLabel(QStringLiteral("Scanning home"), capturePositionContent_);
    scanningHomeLabel->setMinimumWidth(96);
    captureScanningHomeSpin_ = new QDoubleSpinBox(capturePositionContent_);
    captureScanningHomeSpin_->setRange(zaber_stage::kTravelMinimumMm, zaber_stage::kTravelLengthMm);
    captureScanningHomeSpin_->setDecimals(2);
    captureScanningHomeSpin_->setSingleStep(1.0);
    captureScanningHomeSpin_->setSuffix(QStringLiteral(" mm"));
    captureScanningHomeSpin_->setValue(50.0);
    captureScanningHomeSpin_->setToolTip(
        tr("Park position after a scan. The stage waits here for the next capture "
           "instead of seeking the home sensor. Full homing still runs on disconnect "
           "and application shutdown."));
    scanningHomeRowLayout->addWidget(scanningHomeLabel);
    scanningHomeRowLayout->addWidget(captureScanningHomeSpin_, 1);
    positionContentLayout->addLayout(scanningHomeRowLayout);

    positionLayout->addWidget(capturePositionContent_);
    if (capturePanel() != nullptr)
    {
        capturePanel()->updatePositionControls(stageWorker() != nullptr ? stageWorker()->currentState()
                                                                       : StageState::Disconnected);
    }

    auto *preprocessingBox = new QGroupBox(QStringLiteral("Preprocessing"), page);
    capturePreprocessingBox_ = preprocessingBox;
    preprocessingBox->setToolTip(
        tr("Post-processing requires a connected camera and stage. "
           "Enable \"Use HyperFusion Stage for recording\" to use these options."));
    auto *preprocessingLayout = new QVBoxLayout(preprocessingBox);
    preprocessingLayout->setContentsMargins(6, 4, 6, 6);
    preprocessingLayout->setSpacing(6);

    capturePreprocessAfterScanCheck_ = new QCheckBox(
        QStringLiteral("Preprocess the image when the scanning is done"), preprocessingBox);
    capturePreprocessAfterScanCheck_->setChecked(true);
    capturePreprocessAfterScanCheck_->setToolTip(
        tr("Run post-processing on captured data after a stage scan sequence completes. "
           "Requires a connected camera and stage."));
    preprocessingLayout->addWidget(capturePreprocessAfterScanCheck_);

    captureSaveFfcImageCheck_ =
        new QCheckBox(QStringLiteral("Save FFC image"), preprocessingBox);
    captureSaveFfcImageCheck_->setChecked(true);
    captureSaveFfcImageCheck_->setToolTip(
        tr("Write flat-field corrected sample data as ENVI under preprocessed/ when post-processing runs."));
    preprocessingLayout->addWidget(captureSaveFfcImageCheck_);

    auto *gsamRow = new QWidget(preprocessingBox);
    auto *gsamLayout = new QHBoxLayout(gsamRow);
    gsamLayout->setContentsMargins(0, 0, 0, 0);
    gsamLayout->setSpacing(6);

    captureRunGsamCheck_ = new QCheckBox(QStringLiteral("GSAM"), gsamRow);
    captureRunGsamCheck_->setChecked(false);
    captureRunGsamCheck_->setEnabled(false);
    captureRunGsamCheck_->setToolTip(
        tr("After preprocessing, send the RGB preview to the GSAM2 WSL server and write masks "
           "and ROI spectra under preprocessed/segmentation/. "
           "Requires a connected GSAM2 server."));
    gsamLayout->addWidget(captureRunGsamCheck_);

    captureGsamPromptEdit_ = new QLineEdit(gsamRow);
    captureGsamPromptEdit_->setPlaceholderText(QStringLiteral("prompt"));
    captureGsamPromptEdit_->setToolTip(
        tr("GroundingDINO text prompt (e.g. \"grape. leaf.\")."));
    gsamLayout->addWidget(captureGsamPromptEdit_, 1);

    auto *roiLabel = new QLabel(QStringLiteral("ROI"), gsamRow);
    captureGsamSampleCountSpin_ = new QSpinBox(gsamRow);
    captureGsamSampleCountSpin_->setRange(1, 100);
    captureGsamSampleCountSpin_->setValue(5);
    captureGsamSampleCountSpin_->setToolTip(
        tr("Maximum number of detections / ROIs (intended sample count)."));
    gsamLayout->addWidget(roiLabel);
    gsamLayout->addWidget(captureGsamSampleCountSpin_);

    captureGsamServerStatusLabel_ = new QLabel(QStringLiteral("\u25CF"), gsamRow);
    captureGsamServerStatusLabel_->setAlignment(Qt::AlignCenter);
    captureGsamServerStatusLabel_->setFixedWidth(16);
    captureGsamServerStatusLabel_->setToolTip(
        tr("GSAM2 sidecar status. The app tries to start the server automatically at launch."));
    gsamLayout->addWidget(captureGsamServerStatusLabel_);

    preprocessingLayout->addWidget(gsamRow);

    auto *gsamPlanRow = new QWidget(preprocessingBox);
    auto *gsamPlanLayout = new QHBoxLayout(gsamPlanRow);
    gsamPlanLayout->setContentsMargins(0, 0, 0, 0);
    gsamPlanLayout->setSpacing(6);
    auto *gsamPlanLabel = new QLabel(QStringLiteral("GSAM plan"), gsamPlanRow);
    captureGsamPlanCombo_ = new QComboBox(gsamPlanRow);
    captureGsamPlanCombo_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    captureGsamPlanCombo_->setToolTip(
        tr("Per-stream GSAM plan from gsam_plans beside the app. Two-stage plans find well "
           "boxes then segment the object inside each crop. Transmittance can set "
           "reuse_reflectance_masks to copy reflectance masks. (Manual) uses the prompt and ROI "
           "fields above."));
    gsamPlanLayout->addWidget(gsamPlanLabel);
    gsamPlanLayout->addWidget(captureGsamPlanCombo_, 1);
    preprocessingLayout->addWidget(gsamPlanRow);

    captureRunHfFusionCheck_ =
        new QCheckBox(QStringLiteral("Run spectral fusion (FX10e + SWIR3)"), preprocessingBox);
    captureRunHfFusionCheck_->setEnabled(false);
    captureRunHfFusionCheck_->setToolTip(
        tr("After preprocessing, align and fuse FX10e + SWIR3 cubes per chip ROI "
           "for each illumination mode in the session (reflectance, transmittance, …). "
           "Available only when both FX10e and SWIR3 are connected. Also requires "
           "GSAM segmentation and the hf_fusion Python environment beside the app."));
    preprocessingLayout->addWidget(captureRunHfFusionCheck_);

    auto *metadataBox = new QGroupBox(QStringLiteral("Metadata"), page);
    captureMetadataBox_ = metadataBox;
    auto *metadataForm = new QFormLayout(metadataBox);
    metadataForm->setFieldGrowthPolicy(QFormLayout::ExpandingFieldsGrow);
    metadataForm->setRowWrapPolicy(QFormLayout::DontWrapRows);
    captureDatasetEdit_ = new QLineEdit(metadataBox);
    captureDatasetEdit_->setPlaceholderText("Dataset name");

    auto *saveFolderRow = new QWidget(metadataBox);
    saveFolderRow->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    auto *saveFolderLayout = new QHBoxLayout(saveFolderRow);
    saveFolderLayout->setContentsMargins(0, 0, 0, 0);
    saveFolderLayout->setSpacing(6);
    captureSaveFolderEdit_ = new QLineEdit(saveFolderRow);
    captureSaveFolderEdit_->setPlaceholderText("Output location");
    captureSaveFolderEdit_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    captureSaveFolderBrowseBtn_ = new QPushButton(QStringLiteral("Browse\u2026"), saveFolderRow);
    captureSaveFolderBrowseBtn_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    captureSaveFolderBrowseBtn_->setFixedWidth(72);
    saveFolderLayout->addWidget(captureSaveFolderEdit_, 1);
    saveFolderLayout->addWidget(captureSaveFolderBrowseBtn_, 0);

    captureOperatorEdit_ = new QLineEdit(metadataBox);
    captureOperatorEdit_->setPlaceholderText("Operator name");
    captureDescriptionEdit_ = new QPlainTextEdit(metadataBox);
    captureDescriptionEdit_->setPlaceholderText("Sample description, notes, or experiment details");
    captureDescriptionEdit_->setTabChangesFocus(true);
    captureDescriptionEdit_->setLineWrapMode(QPlainTextEdit::WidgetWidth);
    {
        const QFontMetrics fm(captureDescriptionEdit_->fontMetrics());
        const int framePadding = captureDescriptionEdit_->frameWidth() * 2 + 6;
        captureDescriptionEdit_->setFixedHeight(fm.lineSpacing() * 2 + framePadding);
    }
    captureDescriptionEdit_->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    metadataForm->addRow("Dataset", captureDatasetEdit_);
    metadataForm->addRow("Save folder", saveFolderRow);
    metadataForm->addRow("Operator", captureOperatorEdit_);
    metadataForm->addRow("Description", captureDescriptionEdit_);

    connect(captureSaveFolderBrowseBtn_, &QPushButton::clicked, this, [this]() {
        QString startDir = captureSaveFolderEdit_->text().trimmed();
        if (startDir.isEmpty())
        {
            startDir = QDir::homePath();
        }
        else
        {
            const QFileInfo startInfo(startDir);
            if (!startInfo.exists() || !startInfo.isDir())
                startDir = QDir::homePath();
        }

        const QString path = QFileDialog::getExistingDirectory(
            this, tr("Select output location"), startDir);
        if (path.isEmpty())
            return;

        const QFileInfo picked(path);
        if (!picked.exists() || !picked.isDir())
        {
            appendLog(QStringLiteral("Capture: save folder does not exist: %1").arg(path));
            return;
        }

        captureSaveFolderEdit_->setText(QDir::toNativeSeparators(path));
        settingsPanel()->schedulePersistedUiSettingsSave();
        capturePanel()->updateRecorderControls();
    });
    if (captureDatasetEdit_ != nullptr)
    {
        connect(captureDatasetEdit_, &QLineEdit::textChanged, this, [this]() {
            capturePanel()->updateRecorderControls();
        });
    }
    if (captureSaveFolderEdit_ != nullptr)
    {
        connect(captureSaveFolderEdit_, &QLineEdit::textChanged, this, [this]() {
            capturePanel()->updateRecorderControls();
            settingsPanel()->schedulePersistedUiSettingsSave();
        });
    }

    layout->addWidget(recorderBox);
    layout->addWidget(metadataBox);
    layout->addWidget(captureCamerasBox_);
    layout->addWidget(captureModesBox_);
    layout->addWidget(positionBox);
    layout->addWidget(preprocessingBox);
    if (capturePanel() != nullptr)
    {
        capturePanel()->wireSettingsTabConnections();
        capturePanel()->updateCamerasList();
        capturePanel()->updateRecorderControls();
    }
    return scrollArea;
}
