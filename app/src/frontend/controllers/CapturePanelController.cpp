// Capture tab orchestration implementation (recorder, stage scan sequence,
// writer).
#include "frontend/controllers/CapturePanelController.hpp"

#include "adapters/lumo/LumoDeviceTypes.hpp"
#include "adapters/zaber/ZaberStageProfile.hpp"
#include "backend/camera/CameraCoordinator.hpp"
#include "backend/camera/CaptureWriterWorker.hpp"
#include "backend/camera/DualCameraScanOrchestrator.hpp"
#include "backend/HyperFusionConfig.hpp"
#include "backend/light/LighthouseTypes.hpp"
#include "backend/light/LighthouseWorker.hpp"
#include "backend/stage/StageWorker.hpp"
#include "backend/stage/StageTypes.hpp"
#include "backend/camera/processing/CapturePostProcessor.hpp"
#include "backend/camera/processing/CapturePostProcessorWorker.hpp"
#include "backend/camera/processing/Gsam2ServerManager.hpp"
#include "backend/camera/processing/GsamPlan.hpp"
#include "backend/camera/processing/HfFusionWorker.hpp"
#include "frontend/controllers/BfsPanelController.hpp"
#include "frontend/controllers/CameraPanelController.hpp"
#include "frontend/controllers/LightPanelController.hpp"
#include "frontend/controllers/StagePanelController.hpp"
#include "frontend/controllers/Ur3ePanelController.hpp"
#include "frontend/controllers/UiSettingsController.hpp"
#include "frontend/widgets/CaptureSessionSummaryDialog.hpp"
#include "frontend/widgets/LumoCameraUi.hpp"
#include "frontend/widgets/MainWindow.hpp"
#include "frontend/widgets/OperationWaitDialog.hpp"
#include "frontend/widgets/StreamPaneHelpers.hpp"
#include "frontend/widgets/Ur3eHemisphereScanSettingsWidget.hpp"
#include "frontend/widgets/WaterfallDisplayWidget.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QGridLayout>
#include <QGroupBox>
#include <QAbstractButton>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QMetaObject>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSignalBlocker>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
QString defaultCameraTabName(const std::size_t cameraIndex) {
  return QStringLiteral("Camera %1").arg(cameraIndex + 1);
}

QString
lighthouseControllerAliveText(const LighthouseControllerPowerStatus &status,
                              const int lampIndex, const QString &label) {
  if (!status.valid)
    return QStringLiteral("%1: not connected").arg(label);
  const bool alive =
      status.controllerAlive[static_cast<std::size_t>(lampIndex)];
  const float volts = status.monitorVolts[static_cast<std::size_t>(lampIndex)];
  return QStringLiteral("%1: %2 (%3 V)")
      .arg(label)
      .arg(alive ? QStringLiteral("Alive") : QStringLiteral("Off"))
      .arg(static_cast<double>(volts), 0, 'f', 2);
}

QString formatSpatialResolutionMmPerPixel(const double mmPerPixel) {
  if (mmPerPixel <= 0.0)
    return QStringLiteral("Unknown");

  if (mmPerPixel < 1.0)
    return QStringLiteral("%1 \u00b5m/pixel")
        .arg(mmPerPixel * 1000.0, 0, 'f', 2);

  return QStringLiteral("%1 mm/pixel").arg(mmPerPixel, 0, 'f', 4);
}
} // namespace

namespace hf::capture {
CapturePanelController::CapturePanelController(MainWindow *host,
                                               QObject *parent)
    : QObject(parent), host_(host) {}

void CapturePanelController::initializeWorkers() {
  captureWriterWorker_ = std::make_unique<CaptureWriterWorker>();
  captureWriterWorker_->setErrorCallback([this](const QString &message) {
    QMetaObject::invokeMethod(
        this,
        [this, message]() {
          host_->appendLog(QStringLiteral("Capture record: %1").arg(message));
          stopRecorder();
        },
        Qt::QueuedConnection);
  });
  captureWriterWorker_->start();

  capturePostProcessorWorker_ = std::make_unique<CapturePostProcessorWorker>();
  capturePostProcessorWorker_->setStatusListener([this]() {
    QMetaObject::invokeMethod(
        this, [this]() { updateRecorderControls(); }, Qt::QueuedConnection);
  });
  capturePostProcessorWorker_->start();

  hfFusionWorker_ = std::make_unique<HfFusionWorker>();
  hfFusionWorker_->setStatusListener([this]() {
    QMetaObject::invokeMethod(
        this, [this]() { updateRecorderControls(); }, Qt::QueuedConnection);
  });
  hfFusionWorker_->start();

  gsam2ServerManager_ =
      std::make_unique<hf::processing::Gsam2ServerManager>(host_);
  connect(gsam2ServerManager_.get(),
          &hf::processing::Gsam2ServerManager::stateChanged, host_,
          [this](const hf::processing::Gsam2ServerManager::State state,
                 const QString &detail) {
            updateGsamServerUi();
            updateRecorderControls();

            if (state == hf::processing::Gsam2ServerManager::State::Running &&
                !detail.isEmpty())
              host_->appendLog(QStringLiteral("GSAM2 server: %1").arg(detail));
            else if (state ==
                         hf::processing::Gsam2ServerManager::State::Failed &&
                     !detail.isEmpty())
              host_->appendLog(QStringLiteral("GSAM2 server: %1").arg(detail));
          });
}

void CapturePanelController::shutdownWorkers() {
  dismissDualCameraSyncWaitDialog();
  dualCameraSyncHardwareApplyPending_ = false;
  pendingDualSyncSummary_.valid = false;

  if (captureWriterWorker_ != nullptr)
    captureWriterWorker_->stop();
  if (capturePostProcessorWorker_ != nullptr)
    capturePostProcessorWorker_->stop();
  if (gsam2ServerManager_ != nullptr)
    gsam2ServerManager_->stopServer();
}

bool CapturePanelController::isSessionActive() const {
  return captureRecorderMode_ != CaptureRecorderMode::Idle;
}

CapturePanelController::CaptureRecorderMode
CapturePanelController::recorderMode() const {
  return captureRecorderMode_;
}

void CapturePanelController::onStagePosition(const double positionMm) {
  if (captureRecorderMode_ == CaptureRecorderMode::Idle)
    return;

  captureLastKnownStagePositionMm_ = positionMm;
  captureStagePositionKnown_ = true;

  if (captureScanTimingActive_) {
    if (captureScanPhase_ == CaptureScanPhase::SampleScan &&
        captureSampleRecordingActive_)
      updateSampleScanWindowProgress(positionMm);

    if (captureScanPhase_ == CaptureScanPhase::CombinedRecordScan &&
        captureSampleRecordingActive_)
      updateSampleScanWindowProgress(positionMm);

    if (captureScanPhase_ == CaptureScanPhase::SampleScan &&
        isStageScanPositionTrustworthy(positionMm) &&
        allSelectedCamerasPastSampleWindow(positionMm) &&
        canCompleteSampleScan()) {
      QMetaObject::invokeMethod(
          this, [this]() { onCaptureSampleScanComplete(); },
          Qt::QueuedConnection);
    } else if (captureScanPhase_ == CaptureScanPhase::CombinedRecordScan &&
               isStageScanPositionTrustworthy(positionMm) &&
               allSelectedCamerasPastSampleWindow(positionMm) &&
               canCompleteCombinedRecordScan()) {
      QMetaObject::invokeMethod(
          this, [this]() { onCaptureCombinedRecordScanComplete(); },
          Qt::QueuedConnection);
    } else if (captureScanPhase_ == CaptureScanPhase::WhiteReferenceScan &&
               isStageScanPositionTrustworthy(positionMm) &&
               allSelectedCamerasPastWhiteReferenceWindow(positionMm)) {
      QMetaObject::invokeMethod(
          this, [this]() { onCaptureWhiteReferenceSequenceComplete(); },
          Qt::QueuedConnection);
    }
  }
}

void CapturePanelController::onStageHomedForCapture() {
  if (captureRecorderMode_ == CaptureRecorderMode::Idle)
    return;

  host_->stageHomingKind_ = MainWindow::StageHomingKind::None;
  startCaptureSequence();
}

void CapturePanelController::onStageHomedAfterCapture() {
  captureRecordCompleteHomingPending_ = false;
  capturePreviewCompleteHomingPending_ = false;
  tryNotifyRecordComplete();
  tryNotifyPreviewComplete();
}

void CapturePanelController::onCaptureStageHomingFailed(
    const QString &message) {
  failCaptureSequence(message);
}

void CapturePanelController::handleCapturePreviewFrame(
    const SharedFramePacket &frame) {
  if (!frame)
    return;

  std::vector<std::size_t> selected;
  if (!selectedCaptureCameraIndices(selected))
    return;

  const std::size_t cameraIndex =
      frame->source == CameraBackendId::Camera1 ? 0 : 1;
  if (std::find(selected.begin(), selected.end(), cameraIndex) ==
      selected.end())
    return;

  const LumoCameraUi &cameraUi =
      cameraIndex == 0 ? host_->camera1Ui_ : host_->camera2Ui_;

  if (captureScanPhase_ == CaptureScanPhase::BlackReference) {
    if (shouldAcceptBlackReferenceFrame(cameraIndex)) {
      ++captureBlackRefFramesCollected_[cameraIndex];
      if (selectedCamerasReachedBlackReferenceTarget())
        onCaptureBlackReferenceComplete();
      else
        updateRecorderStatus();
    }
    return;
  }

  if (captureScanPhase_ == CaptureScanPhase::WhiteReferenceScan) {
    if (shouldAcceptWhiteReferenceFrame(cameraIndex))
      onWhiteReferenceFrameCollected(cameraIndex);
    return;
  }

  if (captureScanPhase_ == CaptureScanPhase::CombinedRecordScan) {
    const std::size_t stageCameraIndex =
        stageCameraIndexForUi(cameraUi, cameraIndex);

    if (shouldRecordWhiteReferenceFrameForCamera(
            stageCameraIndex, cameraIndex, currentStageScanPositionMm())) {
      const double windowEnd =
          whiteRefStartMmForStageCamera(captureScanPlan_,
                                        captureRecordingIlluminationMode_,
                                        stageCameraIndex) +
          captureScanPlan_.whiteRefScanDistanceMm[stageCameraIndex];
      if (currentStageScanPositionMm() + 0.05 >= windowEnd)
        captureWhiteRefWindowComplete_[stageCameraIndex] = true;

      onWhiteReferenceFrameCollected(cameraIndex);
      return;
    }

    if (!captureSampleRecordingActive_)
      return;

    const std::optional<double> stagePositionMm = knownStageScanPositionMm();
    if (!stagePositionMm)
      return;

    updateSampleScanWindowProgress(*stagePositionMm);

    if (!shouldRecordSampleFrameForCamera(stageCameraIndex, *stagePositionMm))
      return;

    captureSampleWindowEntered_[stageCameraIndex] = true;
    ++captureSampleFramesCollected_[cameraIndex];

    const double windowEnd =
        captureScanPlan_.sampleScanStartMm[stageCameraIndex] +
        captureScanPlan_.sampleScanLengthMm;
    if (*stagePositionMm + 0.05 >= windowEnd)
      captureSampleWindowComplete_[stageCameraIndex] = true;

    updateRecorderStatus();

    if (allSelectedCamerasPastSampleWindow(*stagePositionMm) &&
        canCompleteCombinedRecordScan())
      onCaptureCombinedRecordScanComplete();
    return;
  }

  if (captureScanPhase_ == CaptureScanPhase::SampleScan) {
    if (!captureSampleRecordingActive_)
      return;

    const std::size_t stageCameraIndex =
        stageCameraIndexForUi(cameraUi, cameraIndex);
    const std::optional<double> stagePositionMm = knownStageScanPositionMm();
    if (!stagePositionMm)
      return;

    updateSampleScanWindowProgress(*stagePositionMm);

    if (!shouldRecordSampleFrameForCamera(stageCameraIndex, *stagePositionMm))
      return;

    captureSampleWindowEntered_[stageCameraIndex] = true;
    ++captureSampleFramesCollected_[cameraIndex];

    const double windowEnd =
        captureScanPlan_.sampleScanStartMm[stageCameraIndex] +
        captureScanPlan_.sampleScanLengthMm;
    if (*stagePositionMm + 0.05 >= windowEnd)
      captureSampleWindowComplete_[stageCameraIndex] = true;

    updateRecorderStatus();

    if (allSelectedCamerasPastSampleWindow(*stagePositionMm) &&
        canCompleteSampleScan())
      onCaptureSampleScanComplete();
  }
}

void CapturePanelController::onStreamFrame(const SharedFramePacket &frame) {
  if (!frame)
    return;

  if (captureRecorderMode_ == CaptureRecorderMode::Preview)
    handleCapturePreviewFrame(frame);

  appendCaptureRecordFrame(*frame);
}

void CapturePanelController::wireSettingsTabConnections() {
  if (captureScanTimer_ == nullptr) {
    captureScanTimer_ = new QTimer(this);
    captureScanTimer_->setSingleShot(true);
    connect(captureScanTimer_, &QTimer::timeout, this,
            &CapturePanelController::finishScan);
  }

  if (captureRecorderStatusTimer_ == nullptr) {
    captureRecorderStatusTimer_ = new QTimer(this);
    captureRecorderStatusTimer_->setInterval(300);
    connect(captureRecorderStatusTimer_, &QTimer::timeout, this,
            &CapturePanelController::updateRecorderStatus);
  }

  if (host_->captureRecorderStopBtn_ != nullptr) {
    connect(host_->captureRecorderStopBtn_, &QPushButton::clicked, this,
            &CapturePanelController::stopRecorder, Qt::UniqueConnection);
  }
  if (host_->captureRecorderPreviewBtn_ != nullptr) {
    connect(host_->captureRecorderPreviewBtn_, &QPushButton::clicked, this,
            &CapturePanelController::startPreview, Qt::UniqueConnection);
  }
  if (host_->captureRecorderRecordBtn_ != nullptr) {
    connect(host_->captureRecorderRecordBtn_, &QPushButton::clicked, this,
            &CapturePanelController::startRecord, Qt::UniqueConnection);
  }
}

} // namespace hf::capture

QWidget *hf::capture::CapturePanelController::createStreamTab() {
  host_->captureStreamPage_ = new QWidget(host_);
  auto *layout = new QVBoxLayout(host_->captureStreamPage_);
  layout->setContentsMargins(8, 8, 8, 8);
  layout->setSpacing(8);

  host_->captureStreamEmptyLabel_ = new QLabel(
      QStringLiteral(
          "Select one or more connected cameras under Capture \u2192 Cameras."),
      host_->captureStreamPage_);
  host_->captureStreamEmptyLabel_->setAlignment(Qt::AlignCenter);
  host_->captureStreamEmptyLabel_->setWordWrap(true);
  host_->captureStreamEmptyLabel_->setStyleSheet(
      QStringLiteral("color: #888888;"));

  auto *gridHost = new QWidget(host_->captureStreamPage_);
  host_->captureStreamGrid_ = new QGridLayout(gridHost);
  host_->captureStreamGrid_->setContentsMargins(0, 0, 0, 0);
  host_->captureStreamGrid_->setSpacing(10);

  for (std::size_t cameraIndex = 0; cameraIndex < 2; ++cameraIndex) {
    LumoCameraUi *ui = cameraUiForIndex(cameraIndex);
    const QString cameraName =
        ui != nullptr ? host_->cameraPanel()->profileTabNameForUi(*ui)
                      : defaultCameraTabName(cameraIndex);

    ui::WaterfallDisplayWidget *waterfallWidget = nullptr;
    host_->captureWaterfallPanes_[cameraIndex] = ui::createWaterfallPane(
        gridHost, cameraName + QStringLiteral(" waterfall"), waterfallWidget);
    host_->captureWaterfallViews_[cameraIndex] = waterfallWidget;
    ui::setWaterfallDisconnectedText(host_->captureWaterfallViews_[cameraIndex],
                                     QStringLiteral("waterfall"), cameraName);
    host_->captureWaterfallPanes_[cameraIndex]->hide();
  }

  layout->addWidget(host_->captureStreamEmptyLabel_);
  layout->addWidget(gridHost, 1);

  QLabel *bfsLabel = nullptr;
  host_->captureBfsPreviewPane_ =
      ui::createPreviewPane(gridHost, QStringLiteral("BFS"), bfsLabel);
  host_->captureBfsPreviewLabel_ = bfsLabel;
  ui::setPreviewDisconnectedText(host_->captureBfsPreviewLabel_,
                                 QStringLiteral("stream"),
                                 QStringLiteral("BFS"));
  host_->captureBfsPreviewPane_->hide();

  updateCaptureStreamLayout();
  return host_->captureStreamPage_;
}
void hf::capture::CapturePanelController::updatePositionControls(
    const StageState state) {
  if ((state == StageState::Disconnected || state == StageState::Fault) &&
      captureRecorderMode_ != CaptureRecorderMode::Idle)
    stopRecorder();

  updateRecorderControls();
}

void hf::capture::CapturePanelController::updateScanningSpeedControls() {
  updateDualCameraSyncControls();

  const bool scanActive = captureRecorderMode_ != CaptureRecorderMode::Idle;
  const bool useAuto = host_->captureScanningSpeedAutoCheck_ != nullptr &&
                       host_->captureScanningSpeedAutoCheck_->isChecked();

  if (host_->captureScanningSpeedAutoCheck_ != nullptr)
    host_->captureScanningSpeedAutoCheck_->setEnabled(!scanActive);

  if (host_->captureScanningSpeedSpin_ == nullptr)
    return;

  if (useAuto) {
    const double autoSpeed = qBound(0.0, autoRecordScanSpeedMmPerSec(),
                                    zaber_stage::kMaxSpeedMmPerSec);
    host_->captureScanningSpeedSpin_->setValue(autoSpeed);
    host_->captureScanningSpeedSpin_->setEnabled(false);
  } else {
    host_->captureScanningSpeedSpin_->setEnabled(!scanActive);
  }
}

double
hf::capture::CapturePanelController::autoRecordScanSpeedMmPerSec() const {
  const hf::HardwareConfig &hw = hf::hardwareConfig();

  if (dualCameraScanSyncActive()) {
    const CameraSettings fx10eSettings =
        host_->cameraPanel()->buildSettings(host_->camera1Ui_);
    const double spatial = hf::effectiveSpatialMmPerPixelForStageCamera(
        hw, 0, fx10eSettings.spatialBinning);
    if (spatial <= 0.0)
      return 0.0;

    return hf::recordScanSpeedMmPerSec(fx10eSettings.frameRateHz, spatial);
  }

  const LumoCameraUi *uis[] = {&host_->camera1Ui_, &host_->camera2Ui_};
  std::vector<std::size_t> selectedCameras;
  const bool hasSelection = selectedCaptureCameraIndices(selectedCameras);

  double minSpeed = std::numeric_limits<double>::max();
  const auto considerCamera = [&](const std::size_t cameraIndex) {
    const LumoCameraUi &ui = *uis[cameraIndex];
    const std::size_t stageCameraIndex = stageCameraIndexForUi(ui, cameraIndex);
    const CameraSettings settings = host_->cameraPanel()->buildSettings(ui);
    const double spatial = hf::effectiveSpatialMmPerPixelForStageCamera(
        hw, stageCameraIndex, settings.spatialBinning);
    if (spatial <= 0.0)
      return;

    const double speed =
        hf::recordScanSpeedMmPerSec(settings.frameRateHz, spatial);
    if (speed > 0.0)
      minSpeed = std::min(minSpeed, speed);
  };

  if (hasSelection) {
    for (const std::size_t cameraIndex : selectedCameras)
      considerCamera(cameraIndex);
  } else {
    for (std::size_t cameraIndex = 0; cameraIndex < 2; ++cameraIndex)
      considerCamera(cameraIndex);
  }

  if (minSpeed == std::numeric_limits<double>::max())
    return 0.0;

  return minSpeed;
}

bool hf::capture::CapturePanelController::
    bothFx10eAndSwir3CaptureCamerasConnected() const {
  const auto isConnected = [](const LumoCameraUi &ui) {
    return ui.state != CameraState::Disconnected &&
           ui.state != CameraState::Fault;
  };

  return host_->camera1Ui_.sensorKind == LumoSensorKind::Fx10ePleora &&
         isConnected(host_->camera1Ui_) &&
         host_->camera2Ui_.sensorKind == LumoSensorKind::Swir3Ni &&
         isConnected(host_->camera2Ui_);
}

bool hf::capture::CapturePanelController::dualCameraScanSyncActive() const {
  if (host_->captureDualCameraAutoCheck_ == nullptr ||
      !host_->captureDualCameraAutoCheck_->isChecked())
    return false;

  if (!bothFx10eAndSwir3CaptureCamerasConnected())
    return false;

  std::vector<std::size_t> selectedCameras;
  if (!selectedCaptureCameraIndices(selectedCameras))
    return false;

  bool fx10eSelected = false;
  bool swir3Selected = false;
  for (const std::size_t cameraIndex : selectedCameras) {
    if (cameraIndex == 0 &&
        host_->camera1Ui_.sensorKind == LumoSensorKind::Fx10ePleora)
      fx10eSelected = true;
    if (cameraIndex == 1 &&
        host_->camera2Ui_.sensorKind == LumoSensorKind::Swir3Ni)
      swir3Selected = true;
  }

  return fx10eSelected && swir3Selected;
}

bool hf::capture::CapturePanelController::dualCameraScanSyncReadyForHardware()
    const {
  if (host_->captureDualCameraAutoCheck_ == nullptr ||
      !host_->captureDualCameraAutoCheck_->isChecked())
    return false;

  if (!bothFx10eAndSwir3CaptureCamerasConnected())
    return false;

  return host_->isCameraSessionActive(host_->camera1Ui_.state) &&
         host_->isCameraSessionActive(host_->camera2Ui_.state);
}

bool hf::capture::CapturePanelController::shouldLockSwir3ForDualSync() const {
  return host_->captureDualCameraAutoCheck_ != nullptr &&
         host_->captureDualCameraAutoCheck_->isChecked() &&
         bothFx10eAndSwir3CaptureCamerasConnected();
}

void hf::capture::CapturePanelController::onCaptureStreamTabActivated() {
  updateDualCameraSyncControls();
  if (host_->cameraPanel() != nullptr)
    host_->cameraPanel()->refreshWaterfallDisplayTargets();
}

void hf::capture::CapturePanelController::
    resetDualCameraScanSyncHardwareState() {
  dualCameraScanSyncHardwareApplied_ = false;
  lastAppliedSwir3SyncFrameRateHz_ = -1.0;
}

void hf::capture::CapturePanelController::updateDualCameraSyncControls() {
  const bool bothConnected = bothFx10eAndSwir3CaptureCamerasConnected();
  if (!bothConnected)
    resetDualCameraScanSyncHardwareState();

  if (host_->captureDualCameraAutoCheck_ != nullptr)
    host_->captureDualCameraAutoCheck_->setVisible(bothConnected);

  if (bothConnected && host_->captureDualCameraAutoCheck_ != nullptr &&
      host_->captureDualCameraAutoCheck_->isChecked() &&
      captureRecorderMode_ != CaptureRecorderMode::Idle) {
    const auto ensureChecked = [](QCheckBox *checkbox, const LumoCameraUi &ui) {
      if (checkbox == nullptr || ui.state == CameraState::Disconnected ||
          ui.state == CameraState::Fault)
        return;

      if (!checkbox->isChecked()) {
        QSignalBlocker blocker(checkbox);
        checkbox->setChecked(true);
      }
    };

    ensureChecked(host_->captureCamera1Check_, host_->camera1Ui_);
    ensureChecked(host_->captureCamera2Check_, host_->camera2Ui_);
  }

  const bool syncActive = dualCameraScanSyncActive();
  const bool lockSwirSettings = shouldLockSwir3ForDualSync();
  const bool scanActive = captureRecorderMode_ != CaptureRecorderMode::Idle;

  if (host_->captureDualCameraAutoCheck_ != nullptr)
    host_->captureDualCameraAutoCheck_->setEnabled(bothConnected &&
                                                   !scanActive);

  if (syncActive && host_->captureScanningSpeedAutoCheck_ != nullptr) {
    QSignalBlocker blocker(host_->captureScanningSpeedAutoCheck_);
    host_->captureScanningSpeedAutoCheck_->setChecked(true);
    host_->captureScanningSpeedAutoCheck_->setEnabled(false);
  } else if (host_->captureScanningSpeedAutoCheck_ != nullptr) {
    host_->captureScanningSpeedAutoCheck_->setEnabled(!scanActive);
  }

  if (host_->camera2Ui_.frameRateSpin != nullptr) {
    const bool readyForCameraFeatures =
        host_->camera2Ui_.state == CameraState::Initialized ||
        host_->camera2Ui_.state == CameraState::Configured ||
        host_->camera2Ui_.state == CameraState::Armed ||
        host_->camera2Ui_.state == CameraState::Streaming ||
        host_->camera2Ui_.state == CameraState::SafeStopped;
    host_->camera2Ui_.frameRateSpin->setEnabled(
        readyForCameraFeatures && !lockSwirSettings && !scanActive);
  }

  if (host_->cameraPanel() != nullptr)
    host_->cameraPanel()->updateCameraControls(host_->camera2Ui_,
                                               host_->camera2Ui_.state);

  maybeApplyInitialDualCameraSync();
}

void hf::capture::CapturePanelController::maybeApplyInitialDualCameraSync() {
  if (!dualCameraScanSyncReadyForHardware() ||
      dualCameraScanSyncHardwareApplied_ ||
      dualCameraSyncHardwareApplyPending_ || applyingDualCameraScanSync_)
    return;

  if (captureRecorderMode_ != CaptureRecorderMode::Idle)
    return;

  const QString fx10eName =
      host_->cameraPanel()->profileTabNameForUi(host_->camera1Ui_);
  const QString swirName =
      host_->cameraPanel()->profileTabNameForUi(host_->camera2Ui_);

  host_->appendLog(QStringLiteral("Dual-camera sync: %1 and %2 connected — "
                                  "synchronizing SWIR3 to %1 scan geometry.")
                       .arg(fx10eName, swirName));

  QMessageBox::information(
      host_, QStringLiteral("Dual-camera sync"),
      QStringLiteral(
          "%1 and %2 are both connected.\n\n"
          "HyperFusion will synchronize SWIR3 frame rate and exposure to match "
          "%1 "
          "so both cameras cover the same physical scan distance.\n\n"
          "SWIR3 settings will be applied now. You can disable this later "
          "under "
          "Capture settings.")
          .arg(fx10eName, swirName));

  applyDualCameraScanSync(true);
}

void hf::capture::CapturePanelController::applyDualCameraScanSync(
    const bool applyToHardware) {
  if (!dualCameraScanSyncActive() || applyingDualCameraScanSync_)
    return;

  applyingDualCameraScanSync_ = true;
  struct ApplyingDualCameraScanSyncGuard {
    CapturePanelController &controller;
    explicit ApplyingDualCameraScanSyncGuard(CapturePanelController &panel)
        : controller(panel) {}
    ~ApplyingDualCameraScanSyncGuard() {
      controller.applyingDualCameraScanSync_ = false;
    }
  } guard(*this);

  const hf::HardwareConfig &hw = hf::hardwareConfig();
  const CameraSettings fx10eSettings =
      host_->cameraPanel()->buildSettings(host_->camera1Ui_);
  const CameraSettings swir3Settings =
      host_->cameraPanel()->buildSettings(host_->camera2Ui_);
  hf::DualCameraScanSyncInput input;
  input.fx10eFrameRateHz = fx10eSettings.frameRateHz;
  input.fx10eSpatialMmPerPixel = hf::effectiveSpatialMmPerPixelForStageCamera(
      hw, 0, fx10eSettings.spatialBinning);
  input.swir3SpatialMmPerPixel = hf::effectiveSpatialMmPerPixelForStageCamera(
      hw, 1, swir3Settings.spatialBinning);

  const hf::DualCameraScanSyncResult sync =
      hf::computeDualCameraScanSync(input);
  if (!sync.valid)
    return;

  double clampedRate = sync.syncedSwir3FrameRateHz;
  if (host_->camera2Ui_.frameRateSpin != nullptr) {
    const double minHz = host_->camera2Ui_.frameRateSpin->minimum();
    const double maxHz = host_->camera2Ui_.frameRateSpin->maximum();
    clampedRate = qBound(minHz, sync.syncedSwir3FrameRateHz, maxHz);

    QSignalBlocker blocker(host_->camera2Ui_.frameRateSpin);
    host_->camera2Ui_.frameRateSpin->setValue(clampedRate);
  }

  constexpr double kRateToleranceHz = 0.05;
  const bool rateAlreadyApplied =
      dualCameraScanSyncHardwareApplied_ &&
      std::abs(clampedRate - lastAppliedSwir3SyncFrameRateHz_) <=
          kRateToleranceHz;

  if (applyToHardware && !rateAlreadyApplied &&
      host_->coordinator() != nullptr &&
      host_->isCameraSessionActive(host_->camera2Ui_.state)) {
    pendingDualSyncSummary_.fx10eFrameRateHz = fx10eSettings.frameRateHz;
    pendingDualSyncSummary_.fx10eExposureMs = fx10eSettings.exposureMs;
    pendingDualSyncSummary_.fx10eSpatialMmPerPixel =
        hf::effectiveSpatialMmPerPixelForStageCamera(
            hw, 0, fx10eSettings.spatialBinning);
    pendingDualSyncSummary_.swirSpatialMmPerPixel =
        hf::effectiveSpatialMmPerPixelForStageCamera(
            hw, 1, swir3Settings.spatialBinning);
    pendingDualSyncSummary_.requestedSwirFrameRateHz = clampedRate;
    pendingDualSyncSummary_.valid = true;

    showDualCameraSyncWaitDialog();
    dualCameraSyncHardwareApplyPending_ = true;

    const CameraSettings settings =
        host_->cameraPanel()->buildSettings(host_->camera2Ui_);
    host_->coordinator()->applySettings(host_->camera2Ui_.cameraIndex,
                                        settings);
    lastAppliedSwir3SyncFrameRateHz_ = clampedRate;
    dualCameraScanSyncHardwareApplied_ = true;
  }

  updateScanningSpeedControls();
}

bool hf::capture::CapturePanelController::isDualCameraSyncHardwareApplyPending()
    const {
  return dualCameraSyncHardwareApplyPending_;
}

bool hf::capture::CapturePanelController::shouldSuppressCameraTimingDialogs()
    const {
  return captureIlluminationExposureSwitchActive_;
}

void hf::capture::CapturePanelController::showDualCameraSyncWaitDialog() {
  dismissDualCameraSyncWaitDialog();

  dualCameraSyncWaitDialog_ = new OperationWaitDialog(host_);
  dualCameraSyncWaitDialog_->setWindowModality(Qt::NonModal);
  dualCameraSyncWaitDialog_->setWindowTitle(
      QStringLiteral("Syncing SWIR3 to FX10e"));
  dualCameraSyncWaitDialog_->setStatusText(
      QStringLiteral("Both cameras are connected.\n"
                     "Applying synchronized SWIR3 settings to match FX10e scan "
                     "geometry\u2026\n\n"
                     "This may take a few seconds."));
  dualCameraSyncWaitDialog_->show();
  dualCameraSyncWaitDialog_->raise();
}

void hf::capture::CapturePanelController::dismissDualCameraSyncWaitDialog() {
  if (dualCameraSyncWaitDialog_ == nullptr)
    return;

  dualCameraSyncWaitDialog_->finish();
  dualCameraSyncWaitDialog_->deleteLater();
  dualCameraSyncWaitDialog_ = nullptr;
}

void hf::capture::CapturePanelController::notifyDualCameraSyncSettingsApplied(
    const CameraSettingsApplyReport &report) {
  if (!dualCameraSyncHardwareApplyPending_)
    return;

  dismissDualCameraSyncWaitDialog();
  dualCameraSyncHardwareApplyPending_ = false;
  showDualCameraSyncSummaryDialog(report);
}

void hf::capture::CapturePanelController::notifyDualCameraSyncApplyFailed(
    const QString &message) {
  if (!dualCameraSyncHardwareApplyPending_)
    return;

  dismissDualCameraSyncWaitDialog();
  dualCameraSyncHardwareApplyPending_ = false;
  pendingDualSyncSummary_.valid = false;
  dualCameraScanSyncHardwareApplied_ = false;
  lastAppliedSwir3SyncFrameRateHz_ = -1.0;

  host_->appendLog(QStringLiteral("Dual-camera sync failed: %1").arg(message));
  QMessageBox::warning(host_, QStringLiteral("Dual-camera sync failed"),
                       message);
}

void hf::capture::CapturePanelController::showDualCameraSyncSummaryDialog(
    const CameraSettingsApplyReport &report) {
  if (!pendingDualSyncSummary_.valid)
    return;

  double swirFrameRateHz = pendingDualSyncSummary_.requestedSwirFrameRateHz;
  double swirExposureMs = host_->camera2Ui_.exposureSpin != nullptr
                              ? host_->camera2Ui_.exposureSpin->value()
                              : 0.0;
  if (report.timing.valid) {
    swirFrameRateHz = report.timing.appliedFrameRateHz;
    swirExposureMs = report.timing.appliedExposureMs;
  }

  const QString fx10eName =
      host_->cameraPanel()->profileTabNameForUi(host_->camera1Ui_);
  const QString swirName =
      host_->cameraPanel()->profileTabNameForUi(host_->camera2Ui_);
  const double fx10eFrameRateHz = pendingDualSyncSummary_.fx10eFrameRateHz;

  const QString body =
      QStringLiteral(
          "SWIR3 settings were synchronized to match %1 scan geometry.\n\n"
          "%2 (reference):\n"
          "  Frame rate: %3 Hz\n"
          "  Exposure: %4 ms\n"
          "  Spatial resolution: %5\n\n"
          "%6 (applied):\n"
          "  Frame rate: %7 Hz\n"
          "  Exposure: %8 ms\n"
          "  Spatial resolution: %9")
          .arg(
              fx10eName, fx10eName,
              QString::number(pendingDualSyncSummary_.fx10eFrameRateHz, 'f', 2),
              QString::number(pendingDualSyncSummary_.fx10eExposureMs, 'f', 2),
              formatSpatialResolutionMmPerPixel(
                  pendingDualSyncSummary_.fx10eSpatialMmPerPixel),
              swirName, QString::number(swirFrameRateHz, 'f', 2),
              QString::number(swirExposureMs, 'f', 2),
              formatSpatialResolutionMmPerPixel(
                  pendingDualSyncSummary_.swirSpatialMmPerPixel));

  pendingDualSyncSummary_.valid = false;

  host_->appendLog(
      QStringLiteral(
          "Dual-camera sync complete \u2014 %1 frame rate set to %2 Hz "
          "(from %3 @ %4 Hz).")
          .arg(swirName)
          .arg(swirFrameRateHz, 0, 'f', 2)
          .arg(fx10eName)
          .arg(fx10eFrameRateHz, 0, 'f', 2));

  QMessageBox::information(host_, QStringLiteral("Dual-camera sync complete"),
                           body);
}

void hf::capture::CapturePanelController::
    syncCaptureIlluminationModeControls() {
  const bool stagePresent = isCaptureStagePresent();
  const bool stageConnected = isCaptureStageConnected();
  const bool scanActive = captureRecorderMode_ != CaptureRecorderMode::Idle;

  if (!scanActive && host_->captureUseStageForRecordingCheck_ != nullptr) {
    const QSignalBlocker useStageBlocker(
        host_->captureUseStageForRecordingCheck_);
    if (!stagePresent)
      host_->captureUseStageForRecordingCheck_->setChecked(false);
    else if (!captureStageWasPresent_)
      host_->captureUseStageForRecordingCheck_->setChecked(true);
  }

  if (!scanActive && host_->captureReflectanceCheck_ != nullptr &&
      host_->captureTransmittanceCheck_ != nullptr) {
    const QSignalBlocker reflectanceBlocker(host_->captureReflectanceCheck_);
    const QSignalBlocker transmittanceBlocker(
        host_->captureTransmittanceCheck_);

    if (!stagePresent) {
      host_->captureReflectanceCheck_->setChecked(false);
      host_->captureTransmittanceCheck_->setChecked(false);
    } else if (!captureStageWasPresent_) {
      host_->captureReflectanceCheck_->setChecked(true);
      host_->captureTransmittanceCheck_->setChecked(true);
    }
  }

  captureStageWasPresent_ = stagePresent;

  const bool anyCameraConnected = host_->anyCameraSessionActive();
  const bool captureHardwareReady = stageConnected && anyCameraConnected;
  const bool useStageRecording = isStageRecordingEnabledInUi();
  const bool modesBoxEnabled =
      captureHardwareReady && useStageRecording && !scanActive;
  const bool modeSelectionEnabled = modesBoxEnabled;

  if (host_->captureModesBox_ != nullptr) {
    host_->captureModesBox_->setEnabled(modesBoxEnabled);
    if (!modesBoxEnabled && !scanActive) {
      if (!captureHardwareReady) {
        if (!stageConnected && !anyCameraConnected) {
          host_->captureModesBox_->setToolTip(tr(
              "Connect a camera and the stage to select illumination modes."));
        } else if (!stageConnected) {
          host_->captureModesBox_->setToolTip(
              tr("Connect the stage on the Stage tab to select illumination "
                 "modes."));
        } else {
          host_->captureModesBox_->setToolTip(
              tr("Connect at least one camera on the Camera tab to select "
                 "illumination modes."));
        }
      } else {
        host_->captureModesBox_->setToolTip(
            tr("Enable \"Use HyperFusion Stage for recording\" in Position to "
               "select modes."));
      }
    } else {
      host_->captureModesBox_->setToolTip(tr(
          "Select reflectance and/or transmittance scans for stage capture."));
    }
  }

  if (host_->captureReflectanceCheck_ != nullptr) {
    host_->captureReflectanceCheck_->setEnabled(modeSelectionEnabled);
    host_->captureReflectanceCheck_->setToolTip(
        modeSelectionEnabled ? tr("Include reflectance scan")
        : !captureHardwareReady
            ? (!stageConnected && !anyCameraConnected
                   ? tr("Connect a camera and the stage to enable illumination "
                        "modes")
               : !stageConnected
                   ? tr("Connect the stage to enable illumination modes")
                   : tr("Connect a camera to enable illumination modes"))
            : tr("Enable \"Use HyperFusion Stage for recording\" to select "
                 "modes"));
  }
  if (host_->captureTransmittanceCheck_ != nullptr) {
    host_->captureTransmittanceCheck_->setEnabled(modeSelectionEnabled);
    host_->captureTransmittanceCheck_->setToolTip(
        modeSelectionEnabled ? tr("Include transmittance scan")
        : !captureHardwareReady
            ? (!stageConnected && !anyCameraConnected
                   ? tr("Connect a camera and the stage to enable illumination "
                        "modes")
               : !stageConnected
                   ? tr("Connect the stage to enable illumination modes")
                   : tr("Connect a camera to enable illumination modes"))
            : tr("Enable \"Use HyperFusion Stage for recording\" to select "
                 "modes"));
  }

  // Multiview RGB is independent of stage illumination modes; refresh enablement last.
  syncBfsAndMultiviewRgbCaptureControls();
  if (host_->captureModesBox_ != nullptr) {
    const bool multiviewEnabled = host_->captureMultiviewRgbCheck_ != nullptr &&
                               host_->captureMultiviewRgbCheck_->isEnabled();
    host_->captureModesBox_->setEnabled((modesBoxEnabled || multiviewEnabled) &&
                                        !scanActive);
  }
}

void hf::capture::CapturePanelController::updateRecorderControls() {
  const bool stageConnected = isCaptureStageConnected();
  const bool useStage = useStageForCapture();
  const bool anyCameraConnected = host_->anyCameraSessionActive();
  const bool scanActive = captureRecorderMode_ != CaptureRecorderMode::Idle;
  const bool postProcessActive =
      capturePostProcessorWorker_ != nullptr &&
      capturePostProcessorWorker_->queueStatus().outstandingTotal() > 0;
  const bool fusionWorkerActive =
      hfFusionWorker_ != nullptr && hfFusionWorker_->isBusy();
  const bool backgroundJobActive = postProcessActive || fusionWorkerActive;

  const bool captureHardwareReady = stageConnected && anyCameraConnected;
  const bool positionBoxEnabled = captureHardwareReady && !scanActive;
  if (host_->capturePositionBox_ != nullptr) {
    host_->capturePositionBox_->setEnabled(positionBoxEnabled);
    if (!positionBoxEnabled && !scanActive) {
      if (!stageConnected && !anyCameraConnected) {
        host_->capturePositionBox_->setToolTip(
            tr("Connect a camera and the stage to configure scan position."));
      } else if (!stageConnected) {
        host_->capturePositionBox_->setToolTip(tr(
            "Connect the stage on the Stage tab to configure scan position."));
      } else {
        host_->capturePositionBox_->setToolTip(
            tr("Connect at least one camera on the Camera tab to configure "
               "scan position."));
      }
    } else {
      host_->capturePositionBox_->setToolTip(
          tr("Stage scan position, target length, and scanning speed for "
             "capture."));
    }
  }
  if (host_->capturePositionContent_ != nullptr)
    host_->capturePositionContent_->setEnabled(positionBoxEnabled);

  const bool useStageRecording = isStageRecordingEnabledInUi();
  const bool afterScanPreprocessReady =
      captureHardwareReady && useStageRecording && !scanActive;
  const bool idleOfflinePreprocess = !scanActive && !backgroundJobActive;
  const bool preprocessingEnabled = afterScanPreprocessReady;
  if (host_->capturePreprocessingBox_ != nullptr) {
    if (!afterScanPreprocessReady && !scanActive) {
      host_->capturePreprocessingBox_->setToolTip(
          tr("After-scan preprocessing needs a connected camera and stage. "
             "GSAM plan selection still works for offline scripts and after-scan runs "
             "(two-stage when the plan JSON enables it)."));
    } else {
      host_->capturePreprocessingBox_->setToolTip(
          tr("Post-processing runs after a stage scan completes (FFC, GSAM plan "
             "including two-stage, optional spectral fusion)."));
    }
  }

  if (host_->capturePreprocessAfterScanCheck_ != nullptr)
    host_->capturePreprocessAfterScanCheck_->setEnabled(preprocessingEnabled);

  if (host_->captureSaveFfcImageCheck_ != nullptr)
    host_->captureSaveFfcImageCheck_->setEnabled(idleOfflinePreprocess);

  const bool gsamChildEnabled = idleOfflinePreprocess;
  const bool gsamServerConnected = gsam2ServerManager_ != nullptr &&
                                   gsam2ServerManager_->isServerConnected();
  const bool gsamSegmentationEnabled = gsamChildEnabled && gsamServerConnected;
  const bool gsamPlanSelected =
      host_->captureGsamPlanCombo_ != nullptr &&
      !host_->captureGsamPlanCombo_->currentData().toString().isEmpty();
  if (host_->captureRunGsamCheck_ != nullptr) {
    if (gsamPlanSelected && !host_->captureRunGsamCheck_->isChecked()) {
      const QSignalBlocker blocker(host_->captureRunGsamCheck_);
      host_->captureRunGsamCheck_->setChecked(true);
    }
    // Plan locks GSAM on; Manual leaves the checkbox user-controlled when enabled.
    host_->captureRunGsamCheck_->setEnabled(gsamSegmentationEnabled && !gsamPlanSelected);
    if (gsamPlanSelected) {
      host_->captureRunGsamCheck_->setToolTip(
          tr("GSAM is required while a GSAM plan is selected. Choose (Manual) to turn it off."));
    } else if (!gsamServerConnected) {
      host_->captureRunGsamCheck_->setToolTip(
          tr("Requires a connected GSAM2 server. The app tries to start the "
             "server automatically "
             "at launch when WSL and app/sidecars/gsam2 are available."));
    } else {
      host_->captureRunGsamCheck_->setToolTip(
          tr("After preprocessing, send the RGB preview to the GSAM2 WSL "
             "server and write masks "
             "and ROI spectra under preprocessed/segmentation/."));
    }
  }
  const bool gsamChecked =
      host_->captureRunGsamCheck_ != nullptr &&
      host_->captureRunGsamCheck_->isChecked();
  const bool bothCamerasConnected = bothFx10eAndSwir3CaptureCamerasConnected();
  // Keep the user's fusion choice during a scan/post-process. Only clear it
  // when GSAM is off or a camera is missing — otherwise after-scan fusion
  // reads a checkbox that updateCaptureControls() just unchecked.
  const bool fusionPrereqs = gsamChecked && bothCamerasConnected;
  const bool fusionEnabled = fusionPrereqs && idleOfflinePreprocess;
  if (host_->captureRunHfFusionCheck_ != nullptr) {
    if (!fusionPrereqs) {
      const QSignalBlocker blocker(host_->captureRunHfFusionCheck_);
      host_->captureRunHfFusionCheck_->setChecked(false);
    }
    host_->captureRunHfFusionCheck_->setEnabled(fusionEnabled);
    if (!bothCamerasConnected) {
      host_->captureRunHfFusionCheck_->setToolTip(
          tr("Requires both FX10e and SWIR3 connected."));
    } else if (!gsamChecked) {
      host_->captureRunHfFusionCheck_->setToolTip(
          tr("Enable GSAM segmentation first. Fusion needs chip ROIs from GSAM "
             "on both FX10e and SWIR3."));
    } else {
      host_->captureRunHfFusionCheck_->setToolTip(
          tr("After preprocessing, align and fuse FX10e + SWIR3 cubes per chip ROI "
             "for each illumination mode in the session (reflectance, transmittance, …). "
             "Requires GSAM segmentation on both cameras and the hf_fusion Python environment "
             "beside the app."));
    }
  }
  if (host_->captureGsamPromptEdit_ != nullptr)
    host_->captureGsamPromptEdit_->setEnabled(gsamSegmentationEnabled &&
                                              !gsamPlanSelected);
  if (host_->captureGsamSampleCountSpin_ != nullptr)
    host_->captureGsamSampleCountSpin_->setEnabled(gsamSegmentationEnabled &&
                                                   !gsamPlanSelected);
  if (host_->captureGsamPlanCombo_ != nullptr)
    host_->captureGsamPlanCombo_->setEnabled(!scanActive);

  updateGsamServerUi();

  if (host_->captureRecorderPreviewBtn_ != nullptr) {
    const bool preview3d =
        isMultiviewRgbCaptureSelected() && canRunMultiviewRgbCapture() && !scanActive;
    host_->captureRecorderPreviewBtn_->setEnabled(
        (useStage && !scanActive) || preview3d);
    if (preview3d) {
      host_->captureRecorderPreviewBtn_->setToolTip(
          tr("Multiview RGB: same as UR3e Execute (motion only, no photos)"));
    } else if (!stageConnected) {
      host_->captureRecorderPreviewBtn_->setToolTip(
          tr("Connect the stage on the Stage tab to enable preview"));
    } else if (!useStage) {
      host_->captureRecorderPreviewBtn_->setToolTip(
          tr("Enable \"Use HyperFusion Stage for recording\" to run a stage "
             "scan preview"));
    } else {
      host_->captureRecorderPreviewBtn_->setToolTip(
          tr("Run the same stage scan sequence as Record (hood prompts, refs, "
             "sample) without saving"));
    }
  }

  if (host_->captureRecorderRecordBtn_ != nullptr) {
    const bool record3d =
        isMultiviewRgbCaptureSelected() && canRunMultiviewRgbCapture() && !scanActive
        && !backgroundJobActive;
    host_->captureRecorderRecordBtn_->setEnabled(
        ((anyCameraConnected || record3d) && !scanActive && !backgroundJobActive));
    if (record3d && !anyCameraConnected) {
      host_->captureRecorderRecordBtn_->setToolTip(
          tr("Multiview RGB: stage to apex, then MVS stop; hemisphere scan; save BFS "
             "stills under dataset/multiview/"));
    } else if (!anyCameraConnected) {
      host_->captureRecorderRecordBtn_->setToolTip(tr(
          "Connect at least one camera on the Camera tab to enable recording"));
    } else if (backgroundJobActive) {
      host_->captureRecorderRecordBtn_->setToolTip(
          tr("Wait for background post-processing or fusion to finish before "
             "starting a new record"));
    } else if (useStage) {
      host_->captureRecorderRecordBtn_->setToolTip(
          tr("Run full scan sequence (black ref, white ref, sample) and save "
             ".raw frames"));
    } else {
      host_->captureRecorderRecordBtn_->setToolTip(
          tr("Record reflectance .raw frames locally without stage scanning "
             "\u2014 press Stop when done"));
    }
  }

  if (host_->captureRecorderStopBtn_ != nullptr)
    host_->captureRecorderStopBtn_->setEnabled(scanActive || captureMultiviewInProgress_);

  if (captureRecorderStatusTimer_ != nullptr) {
    if (scanActive)
      captureRecorderStatusTimer_->start();
    else
      captureRecorderStatusTimer_->stop();
  }

  syncCaptureIlluminationModeControls();

  if (host_->captureTargetLengthSpin_ != nullptr)
    host_->captureTargetLengthSpin_->setEnabled(!scanActive);
  if (host_->captureScanningHomeSpin_ != nullptr)
    host_->captureScanningHomeSpin_->setEnabled(!scanActive);
  updateScanningSpeedControls();

  host_->lightPanel()->updateConnectionDisplay();
  host_->lightPanel()->updateControlsEnabled();
  updateSessionUiLock();
  updateRecorderStatus();

  if (host_->cameraPanel() != nullptr)
    host_->cameraPanel()->refreshWaterfallDisplayTargets();
}

void hf::capture::CapturePanelController::tryAutoStartGsamServer() {
  if (gsam2ServerManager_ == nullptr)
    return;

  gsam2ServerManager_->tryAutoStart();
}

void hf::capture::CapturePanelController::updateGsamServerUi() {
  if (gsam2ServerManager_ == nullptr ||
      host_->captureGsamServerStatusLabel_ == nullptr)
    return;

  using GsamState = hf::processing::Gsam2ServerManager::State;
  const GsamState state = gsam2ServerManager_->state();

  QString statusText;
  QString color;

  switch (state) {
  case GsamState::Starting:
    statusText = QStringLiteral("connecting\u2026");
    color = QStringLiteral("#b8860b");
    break;
  case GsamState::Running:
    statusText = QStringLiteral("connected");
    color = QStringLiteral("#1b8f1b");
    break;
  case GsamState::Stopped:
    statusText = QStringLiteral("stopped");
    color = QStringLiteral("#888888");
    break;
  case GsamState::Failed:
    statusText = QStringLiteral("failed");
    color = QStringLiteral("#c62828");
    break;
  case GsamState::Unavailable:
  default:
    statusText = QStringLiteral("not available");
    color = QStringLiteral("#888888");
    break;
  }

  host_->captureGsamServerStatusLabel_->setText(QStringLiteral("\u25CF"));
  host_->captureGsamServerStatusLabel_->setToolTip(
      QStringLiteral("GSAM server: %1").arg(statusText));
  host_->captureGsamServerStatusLabel_->setStyleSheet(
      QStringLiteral("color: %1; background: transparent; font-size: 14px;")
          .arg(color));
}

void hf::capture::CapturePanelController::refreshGsamPlanCombo(
    const QString &preferredPlanId) {
  if (host_->captureGsamPlanCombo_ == nullptr)
    return;

  const QString previousPath = host_->captureGsamPlanCombo_->currentData().toString();
  const QString preferId = preferredPlanId.trimmed();
  const QSignalBlocker blocker(host_->captureGsamPlanCombo_);
  host_->captureGsamPlanCombo_->clear();
  host_->captureGsamPlanCombo_->addItem(QStringLiteral("(Manual)"), QString());

  const QString plansDir = hf::processing::defaultGsamPlansDirectory();
  const QStringList files = hf::processing::listGsamPlanJsonFiles(plansDir);
  int selectIndex = 0;
  for (const QString &fileName : files) {
    const QString path = QDir(plansDir).filePath(fileName);
    const QString planId = QFileInfo(fileName).completeBaseName();
    QString display = planId;
    hf::processing::GsamPlan plan;
    if (hf::processing::loadGsamPlan(path, &plan, nullptr) && !plan.name.isEmpty())
      display = hf::processing::gsamPlanComboLabel(plan, planId);
    else
      display = planId;
    host_->captureGsamPlanCombo_->addItem(display, path);
    const int index = host_->captureGsamPlanCombo_->count() - 1;
    const QString tip = hf::processing::gsamPlanTooltip(plan);
    if (!tip.isEmpty())
      host_->captureGsamPlanCombo_->setItemData(index, tip, Qt::ToolTipRole);
    if (!preferId.isEmpty() && planId.compare(preferId, Qt::CaseInsensitive) == 0)
      selectIndex = index;
    else if (preferId.isEmpty() && !previousPath.isEmpty() &&
             QFileInfo(previousPath).fileName().compare(fileName, Qt::CaseInsensitive) == 0)
      selectIndex = index;
  }

  if (selectIndex >= 0 && selectIndex < host_->captureGsamPlanCombo_->count())
    host_->captureGsamPlanCombo_->setCurrentIndex(selectIndex);
  updateRecorderControls();
}

void hf::capture::CapturePanelController::updateSessionUiLock() {
  const bool scanActive = captureRecorderMode_ != CaptureRecorderMode::Idle;

  // Tabs stay switchable during preview/record; individual controls are locked
  // below.
  if (host_->captureMetadataBox_ != nullptr)
    host_->captureMetadataBox_->setEnabled(!scanActive);
  if (host_->captureCamerasBox_ != nullptr)
    host_->captureCamerasBox_->setEnabled(!scanActive);
  if (host_->captureModesBox_ != nullptr && scanActive)
    host_->captureModesBox_->setEnabled(false);
  if (host_->capturePositionBox_ != nullptr && scanActive)
    host_->capturePositionBox_->setEnabled(false);

  if (!scanActive)
    syncCaptureIlluminationModeControls();

  if (host_->ur3eSettingsPage_ != nullptr)
    host_->ur3eSettingsPage_->setEnabled(!scanActive);
  if (host_->ur3ePanel() != nullptr)
    host_->ur3ePanel()->refreshUi();

  if (scanActive && host_->captureRecorderStopBtn_ != nullptr)
    host_->captureRecorderStopBtn_->setEnabled(true);

  if (host_->stageWorker() != nullptr)
    host_->stagePanel()->updateConnectionControls(
        host_->stageWorker()->currentState(), false);

  host_->cameraPanel()->updateCameraControls(host_->camera1Ui_,
                                             host_->camera1Ui_.state);
  host_->cameraPanel()->updateCameraControls(host_->camera2Ui_,
                                             host_->camera2Ui_.state);
}

void hf::capture::CapturePanelController::clearRecorderCameraStatusLabels() {
  for (std::size_t cameraIndex = 0; cameraIndex < 2; ++cameraIndex) {
    if (host_->captureRecorderCameraStatusLabels_[cameraIndex] == nullptr)
      continue;

    host_->captureRecorderCameraStatusLabels_[cameraIndex]->setVisible(false);
    host_->captureRecorderCameraStatusLabels_[cameraIndex]->clear();
  }
}

void hf::capture::CapturePanelController::updateRecorderStatus() {
  if (host_->captureRecorderStatusIndicator_ == nullptr ||
      host_->captureRecorderStatusLabel_ == nullptr)
    return;

  const auto setIndicator = [this](const QString &color) {
    host_->captureRecorderStatusIndicator_->setStyleSheet(
        QStringLiteral("background-color: %1; border-radius: 7px;").arg(color));
  };

  const bool postProcessActive =
      capturePostProcessorWorker_ != nullptr &&
      capturePostProcessorWorker_->queueStatus().outstandingTotal() > 0;
  const bool fusionWorkerActive =
      hfFusionWorker_ != nullptr && hfFusionWorker_->isBusy();
  const bool backgroundJobActive = postProcessActive || fusionWorkerActive;

  if (captureRecorderMode_ == CaptureRecorderMode::Idle) {
    if (backgroundJobActive) {
      setIndicator(QStringLiteral("#f39c12"));
      host_->captureRecorderStatusLabel_->setText(
          fusionWorkerActive && !postProcessActive
              ? tr("Running spectral fusion in background\u2026")
              : tr("Processing previous capture in background\u2026"));
      clearRecorderCameraStatusLabels();
      return;
    }

    setIndicator(QStringLiteral("#2ecc71"));
    host_->captureRecorderStatusLabel_->setText(tr("Ready"));
    clearRecorderCameraStatusLabels();
    return;
  }

  const bool preview = captureRecorderMode_ == CaptureRecorderMode::Preview;
  setIndicator(preview ? QStringLiteral("#3498db") : QStringLiteral("#e74c3c"));

  const QString activityPrefix = preview ? tr("Preview") : tr("Recording");

  if (!useStageForCapture()) {
    std::vector<std::size_t> selectedCameras;
    if (selectedCaptureCameraIndices(selectedCameras) &&
        selectedCameras.size() == 1) {
      host_->captureRecorderStatusLabel_->setText(
          tr("%1 \u2014 reflectance \u2014 %2 frames (press Stop when "
             "finished)")
              .arg(activityPrefix)
              .arg(captureSampleFramesCollected_[selectedCameras.front()]));
    } else {
      int maxFrames = 0;
      for (const std::size_t index : selectedCameras)
        maxFrames = std::max(maxFrames, captureSampleFramesCollected_[index]);

      host_->captureRecorderStatusLabel_->setText(
          selectedCameras.size() > 1 ? tr("%1 \u2014 reflectance \u2014 saving "
                                          "frames (press Stop when finished)")
                                           .arg(activityPrefix)
                                     : tr("%1 \u2014 reflectance \u2014 %2 "
                                          "frames (press Stop when finished)")
                                           .arg(activityPrefix)
                                           .arg(maxFrames));
    }

    for (std::size_t cameraIndex = 0; cameraIndex < 2; ++cameraIndex) {
      if (host_->captureRecorderCameraStatusLabels_[cameraIndex] == nullptr)
        continue;

      QCheckBox *checkbox = cameraIndex == 0 ? host_->captureCamera1Check_
                                             : host_->captureCamera2Check_;
      const LumoCameraUi &ui =
          cameraIndex == 0 ? host_->camera1Ui_ : host_->camera2Ui_;
      const bool connected = ui.state != CameraState::Disconnected &&
                             ui.state != CameraState::Fault;
      const bool selected =
          checkbox != nullptr && checkbox->isVisible() && checkbox->isChecked();
      const bool showLine = selected && connected && selectedCameras.size() > 1;

      host_->captureRecorderCameraStatusLabels_[cameraIndex]->setVisible(
          showLine);
      if (showLine) {
        host_->captureRecorderCameraStatusLabels_[cameraIndex]->setText(
            recorderCameraStatusText(cameraIndex));
      }
    }
    return;
  }

  if (host_->stageWorker() != nullptr &&
      host_->stageWorker()->currentState() == StageState::Homing &&
      host_->stageHomingKind_ == MainWindow::StageHomingKind::BeforeCapture) {
    host_->captureRecorderStatusLabel_->setText(
        tr("%1 \u2014 homing stage\u2026").arg(activityPrefix));
    return;
  }

  QString modeLabel =
      captureIlluminationFolderName(captureRecordingIlluminationMode_);
  if (capturePendingIlluminationModes_.size() > 1) {
    modeLabel = QStringLiteral("%1 (%2/%3)")
                    .arg(modeLabel)
                    .arg(captureCurrentModeIndex_ + 1)
                    .arg(capturePendingIlluminationModes_.size());
  }

  QString phaseDetail;
  switch (captureScanPhase_) {
  case CaptureScanPhase::Idle:
    phaseDetail = tr("preparing\u2026");
    break;
  case CaptureScanPhase::MoveToFirstRefPosition:
    phaseDetail =
        captureRecorderMode_ == CaptureRecorderMode::Record
            ? tr("moving to first reference (before black reference)\u2026")
            : tr("moving to first reference\u2026");
    break;
  case CaptureScanPhase::MoveToTempStopPosition:
    phaseDetail = tr("moving to temp stop (prepare transmittance)\u2026");
    break;
  case CaptureScanPhase::MoveToWhiteRefScanOrigin: {
    const QString refLabel = captureRecordingIlluminationMode_ ==
                                     CaptureIlluminationMode::Reflectance
                                 ? tr("white reference")
                                 : tr("bright reference");
    phaseDetail = tr("moving to %1 scan origin\u2026").arg(refLabel);
    break;
  }
  case CaptureScanPhase::BlackReference: {
    if (!captureBlackReferenceCollectionActive_) {
      phaseDetail = tr("black reference (waiting for shutters to close\u2026)");
      break;
    }
    int collected = 0;
    std::vector<std::size_t> selectedCameras;
    if (selectedCaptureCameraIndices(selectedCameras)) {
      for (const std::size_t index : selectedCameras)
        collected = std::max(collected, captureBlackRefFramesCollected_[index]);
    }
    const int target = captureScanPlan_.blackReferenceFrameCount;
    phaseDetail = tr("black reference (%1/%2 frames, shutters closed)")
                      .arg(collected)
                      .arg(target);
    break;
  }
  case CaptureScanPhase::WhiteReferenceScan: {
    const QString refLabel = captureRecordingIlluminationMode_ ==
                                     CaptureIlluminationMode::Reflectance
                                 ? tr("white reference")
                                 : tr("bright reference");
    int collected = 0;
    std::vector<std::size_t> selectedCameras;
    if (selectedCaptureCameraIndices(selectedCameras)) {
      for (const std::size_t index : selectedCameras)
        collected = std::max(collected, captureWhiteRefFramesCollected_[index]);
    }
    phaseDetail = tr("%1 scan (%2/%3 frames)")
                      .arg(refLabel)
                      .arg(collected)
                      .arg(captureScanPlan_.whiteReferenceFrameCount);
    break;
  }
  case CaptureScanPhase::CombinedRecordScan:
    if (captureScanTimingActive_) {
      phaseDetail = tr("record scan @ %1 mm")
                        .arg(currentStageScanPositionMm(), 0, 'f', 1);
    } else {
      phaseDetail = tr("record scan\u2026");
    }
    break;
  case CaptureScanPhase::MoveToSampleScanOrigin:
    phaseDetail = tr("moving to sample scan origin\u2026");
    break;
  case CaptureScanPhase::SampleScan:
    if (captureScanTimingActive_) {
      phaseDetail = tr("sample scan @ %1 mm\u2026")
                        .arg(currentStageScanPositionMm(), 0, 'f', 1);
    } else {
      phaseDetail = tr("sample scan\u2026");
    }
    break;
  case CaptureScanPhase::MoveToMultiviewPosition:
    phaseDetail = captureMultiviewNeedRingsLeg_
                      ? tr("moving to MVS ring stage pose\u2026")
                      : tr("moving to Multiview apex stage pose\u2026");
    break;
  }

  host_->captureRecorderStatusLabel_->setText(
      QStringLiteral("%1 \u2014 %2 \u2014 %3")
          .arg(activityPrefix, modeLabel, phaseDetail));

  for (std::size_t cameraIndex = 0; cameraIndex < 2; ++cameraIndex) {
    if (host_->captureRecorderCameraStatusLabels_[cameraIndex] == nullptr)
      continue;

    QCheckBox *checkbox = cameraIndex == 0 ? host_->captureCamera1Check_
                                           : host_->captureCamera2Check_;
    const LumoCameraUi &ui =
        cameraIndex == 0 ? host_->camera1Ui_ : host_->camera2Ui_;
    const bool connected =
        ui.state != CameraState::Disconnected && ui.state != CameraState::Fault;
    const bool selected =
        checkbox != nullptr && checkbox->isVisible() && checkbox->isChecked();
    const bool showLine = selected && connected;

    host_->captureRecorderCameraStatusLabels_[cameraIndex]->setVisible(
        showLine);
    if (showLine) {
      host_->captureRecorderCameraStatusLabels_[cameraIndex]->setText(
          recorderCameraStatusText(cameraIndex));
    }
  }
}

void hf::capture::CapturePanelController::notifyRecordComplete() {
  const CaptureWriterSessionSummary &summary = lastEndedCaptureSessionSummary_;
  if (summary.sessionDirectory.isEmpty() && lastCaptureMultiviewSummaryText_.isEmpty()) {
    QMessageBox::information(host_, tr("Recording complete"),
                             tr("The capture scan sequence finished."));
    return;
  }

  const QString extra = lastCaptureMultiviewSummaryText_;
  lastCaptureMultiviewSummaryText_.clear();
  CaptureSessionSummaryDialog::execForSession(host_, summary, extra);
}

void hf::capture::CapturePanelController::notifyPreviewComplete() {
  QStringList detailLines;
  if (!lastPreviewCompleteModeFolders_.isEmpty()) {
    detailLines.push_back(
        tr("Illumination: %1")
            .arg(lastPreviewCompleteModeFolders_.join(QStringLiteral(", "))));
  }
  for (const std::size_t cameraIndex : lastPreviewCompleteCameraIndices_) {
    const LumoCameraUi &ui =
        cameraIndex == 0 ? host_->camera1Ui_ : host_->camera2Ui_;
    detailLines.push_back(
        QStringLiteral("%1 \u2014 %2 sample frames")
            .arg(host_->cameraPanel()->profileTabNameForUi(ui))
            .arg(lastPreviewCompleteSampleFrames_[cameraIndex]));
  }
  detailLines.push_back(tr("No data was saved."));

  QMessageBox box(host_);
  box.setIcon(QMessageBox::Information);
  box.setWindowTitle(tr("Preview complete"));
  box.setText(tr("Capture preview scan sequence finished successfully."));
  box.setInformativeText(detailLines.join(QLatin1Char('\n')));
  box.setStandardButtons(QMessageBox::Ok);
  box.exec();
}

void hf::capture::CapturePanelController::beginRecordCompleteNotify() {
  pendingCaptureRecordCompleteNotify_ = true;

  // Post-process may already be running (started at HSI end) or already finished during Multiview.
  captureRecordCompletePostProcessPending_ = capturePostProcessInFlight_;

  captureRecordCompleteHomingPending_ =
      !host_->performingGracefulShutdown_ && host_->stageWorker() != nullptr &&
      host_->stageWorker()->currentState() == StageState::Connected;

  tryNotifyRecordComplete();
}

void hf::capture::CapturePanelController::tryNotifyRecordComplete() {
  if (!pendingCaptureRecordCompleteNotify_)
    return;

  if (captureRecordCompleteHomingPending_ ||
      captureRecordCompletePostProcessPending_)
    return;

  pendingCaptureRecordCompleteNotify_ = false;
  notifyRecordComplete();
}

void hf::capture::CapturePanelController::beginPreviewCompleteNotify() {
  pendingCapturePreviewCompleteNotify_ = true;

  capturePreviewCompleteHomingPending_ =
      !host_->performingGracefulShutdown_ && host_->stageWorker() != nullptr &&
      host_->stageWorker()->currentState() == StageState::Connected;

  tryNotifyPreviewComplete();
}

void hf::capture::CapturePanelController::tryNotifyPreviewComplete() {
  if (!pendingCapturePreviewCompleteNotify_)
    return;

  if (capturePreviewCompleteHomingPending_)
    return;

  pendingCapturePreviewCompleteNotify_ = false;
  notifyPreviewComplete();
}

bool hf::capture::CapturePanelController::buildCaptureScanPlan(
    CaptureScanPlan &plan, QString &errorMessage) const {
  if (host_->stageWorker() == nullptr ||
      host_->stageWorker()->currentState() != StageState::Connected) {
    errorMessage = QStringLiteral("Stage is not connected.");
    return false;
  }

  if (host_->captureTargetLengthSpin_ == nullptr ||
      host_->captureScanningSpeedSpin_ == nullptr) {
    errorMessage = QStringLiteral("Capture scan controls are not available.");
    return false;
  }

  const hf::HardwareConfig &hw = hf::hardwareConfig();
  plan.sampleScanLengthMm = host_->captureTargetLengthSpin_->value();
  plan.operationSpeedMmPerSec = hw.operationScanningSpeedMmPerSec;
  if (host_->captureScanningSpeedAutoCheck_ != nullptr &&
      host_->captureScanningSpeedAutoCheck_->isChecked())
    plan.recordScanSpeedMmPerSec = autoRecordScanSpeedMmPerSec();
  else
    plan.recordScanSpeedMmPerSec = host_->captureScanningSpeedSpin_->value();
  plan.whiteReferenceFrameCount = hw.whiteReferenceFrames;
  plan.blackReferenceFrameCount = hw.blackReferenceFrames;
  plan.blackReferenceShutterSettleMs = hw.blackReferenceShutterSettleMs;
  plan.whiteRefStartMm[0] = hw.whiteRefMm[0];
  plan.whiteRefStartMm[1] = hw.whiteRefMm[1];
  plan.brightRefStartMm[0] = hw.brightRefMm[0];
  plan.brightRefStartMm[1] = hw.brightRefMm[1];
  plan.sampleScanStartMm[0] = hw.sampleScanStartMm[0];
  plan.sampleScanStartMm[1] = hw.sampleScanStartMm[1];

  std::vector<std::size_t> selectedCameras;
  if (!selectedCaptureCameraIndices(selectedCameras)) {
    errorMessage = QStringLiteral(
        "Select at least one connected camera in the Cameras list.");
    return false;
  }

  bool hasSampleOrigin = false;
  double sampleOriginMm = 0.0;
  double sampleEndMm = 0.0;
  for (const std::size_t cameraIndex : selectedCameras) {
    const LumoCameraUi &ui =
        cameraIndex == 0 ? host_->camera1Ui_ : host_->camera2Ui_;
    const std::size_t stageCameraIndex = stageCameraIndexForUi(ui, cameraIndex);

    const double whiteStart = whiteRefStartMmForStageCamera(
        plan, CaptureIlluminationMode::Reflectance, stageCameraIndex);
    const double brightStart = whiteRefStartMmForStageCamera(
        plan, CaptureIlluminationMode::Transmittance, stageCameraIndex);
    const double sampleStart = plan.sampleScanStartMm[stageCameraIndex];
    const double whiteRefScanDistanceMm =
        whiteReferenceScanDistanceMmForCamera(ui, plan);
    plan.whiteRefScanDistanceMm[stageCameraIndex] = whiteRefScanDistanceMm;

    if (whiteStart + whiteRefScanDistanceMm > zaber_stage::kTravelLengthMm) {
      errorMessage =
          QStringLiteral(
              "Reflectance white-reference scan for camera %1 would exceed "
              "stage travel limit.")
              .arg(captureCameraFolderName(ui));
      return false;
    }

    if (brightStart + whiteRefScanDistanceMm > zaber_stage::kTravelLengthMm) {
      errorMessage =
          QStringLiteral(
              "Transmittance bright-reference scan for camera %1 would exceed "
              "stage travel limit.")
              .arg(captureCameraFolderName(ui));
      return false;
    }

    if (sampleStart + plan.sampleScanLengthMm > zaber_stage::kTravelLengthMm) {
      errorMessage =
          QStringLiteral(
              "Sample scan for camera %1 would exceed stage travel limit.")
              .arg(captureCameraFolderName(ui));
      return false;
    }

    if (!hasSampleOrigin) {
      sampleOriginMm = sampleStart;
      sampleEndMm = sampleStart + plan.sampleScanLengthMm;
      hasSampleOrigin = true;
    } else {
      sampleOriginMm = std::min(sampleOriginMm, sampleStart);
      sampleEndMm =
          std::max(sampleEndMm, sampleStart + plan.sampleScanLengthMm);
    }
  }

  if (!hasSampleOrigin) {
    errorMessage = QStringLiteral(
        "No camera positions available for sample scan planning.");
    return false;
  }

  plan.sampleScanOriginMm = sampleOriginMm;
  plan.sampleScanTotalDistanceMm = sampleEndMm - sampleOriginMm;
  if (plan.sampleScanTotalDistanceMm <= 0.0) {
    errorMessage = QStringLiteral("Computed sample scan distance must be > 0.");
    return false;
  }

  if (plan.sampleScanLengthMm > hw.sampleWindowMaxLengthMm) {
    errorMessage = QStringLiteral("Target length (%1 mm) must not exceed "
                                  "sample_window_max_length_mm (%2 mm).")
                       .arg(plan.sampleScanLengthMm, 0, 'f', 2)
                       .arg(hw.sampleWindowMaxLengthMm, 0, 'f', 2);
    return false;
  }

  if (plan.operationSpeedMmPerSec <= 0.0) {
    errorMessage = QStringLiteral(
        "operation_scanning_speed_mm_per_sec must be > 0 in hyperfusion.cfg.");
    return false;
  }

  if (plan.recordScanSpeedMmPerSec <= 0.0) {
    errorMessage =
        QStringLiteral("Record scanning speed must be > 0 (check frame rate "
                       "and spatial_mm_per_pixel in hyperfusion.cfg).");
    return false;
  }

  if (plan.operationSpeedMmPerSec > zaber_stage::kMaxSpeedMmPerSec) {
    errorMessage =
        QStringLiteral("Operation scanning speed exceeds the stage limit.");
    return false;
  }

  if (plan.recordScanSpeedMmPerSec > zaber_stage::kMaxSpeedMmPerSec) {
    errorMessage =
        QStringLiteral("Record scanning speed exceeds the stage limit.");
    return false;
  }

  if (plan.whiteReferenceFrameCount <= 0) {
    errorMessage = QStringLiteral(
        "white_reference_frames must be > 0 in hyperfusion.cfg.");
    return false;
  }

  if (plan.blackReferenceFrameCount <= 0) {
    errorMessage = QStringLiteral(
        "black_reference_frames must be > 0 in hyperfusion.cfg.");
    return false;
  }

  return true;
}

QString hf::capture::CapturePanelController::captureSequenceLogPrefix() const {
  return captureRecorderMode_ == CaptureRecorderMode::Record
             ? QStringLiteral("Capture record")
             : QStringLiteral("Capture preview");
}

void hf::capture::CapturePanelController::resetCaptureSequenceState() {
  captureScanPhase_ = CaptureScanPhase::Idle;
  captureMoveCompletePhase_ = CaptureScanPhase::Idle;
  captureBlackRefFramesCollected_ = {0, 0};
  captureBlackReferenceCollectionActive_ = false;
  captureWhiteRefFramesCollected_ = {0, 0};
  captureSampleFramesCollected_ = {0, 0};
  captureWhiteRefWindowComplete_ = {false, false};
  captureSampleWindowComplete_ = {false, false};
  captureSampleWindowEntered_ = {false, false};
  captureSampleRecordingActive_ = false;
  if (captureRecorderMode_ == CaptureRecorderMode::Idle)
    captureStageSequenceActive_ = false;
  captureStagePositionKnown_ = false;
  captureScanTimingActive_ = false;
  captureRelativeScanTimerActiveToken_ = 0;
  captureSampleScanTimerExtendCount_ = 0;
  if (captureScanTimer_ != nullptr)
    captureScanTimer_->stop();
  updateRecorderStatus();
}

void hf::capture::CapturePanelController::initializeCaptureModeQueue() {
  capturePendingIlluminationModes_.clear();
  effectiveCaptureIlluminationModes(capturePendingIlluminationModes_);

  std::stable_sort(
      capturePendingIlluminationModes_.begin(),
      capturePendingIlluminationModes_.end(),
      [](const CaptureIlluminationMode a, const CaptureIlluminationMode b) {
        if (a == b)
          return false;
        return a == CaptureIlluminationMode::Reflectance;
      });

  captureCurrentModeIndex_ = 0;
  if (!capturePendingIlluminationModes_.empty())
    captureRecordingIlluminationMode_ =
        capturePendingIlluminationModes_.front();

  dualModeExposureSwitchEnabled_ = hasReflectanceAndTransmittanceCaptureModes();
  captureUsingTransmittanceExposure_ = false;
  if (dualModeExposureSwitchEnabled_)
    saveReflectanceExposuresForDualModeCapture();
}

bool hf::capture::CapturePanelController::confirmCaptureStart(
    const LighthouseControllerPowerStatus &powerStatus) const {
  const bool record = captureRecorderMode_ == CaptureRecorderMode::Record;
  const QString action =
      record ? QStringLiteral("recording") : QStringLiteral("preview");

  QMessageBox box(host_);
  box.setIcon(QMessageBox::Information);
  box.setWindowTitle(tr("Scanning procedure"));
  box.setText(tr("Ready to start %1?").arg(action));
  box.setInformativeText(buildScanningProcedureSummary(&powerStatus));
  box.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
  box.setDefaultButton(QMessageBox::Ok);
  if (QAbstractButton *continueButton = box.button(QMessageBox::Ok))
    continueButton->setText(QStringLiteral("Continue"));

  return box.exec() == QMessageBox::Ok;
}

QString hf::capture::CapturePanelController::buildScanningProcedureSummary(
    const LighthouseControllerPowerStatus *powerStatus) const {
  QStringList sections;

  QStringList modeLabels;
  for (const CaptureIlluminationMode mode : capturePendingIlluminationModes_)
    modeLabels.push_back(captureIlluminationFolderName(mode));

  if (!modeLabels.isEmpty()) {
    sections.push_back(
        QStringLiteral("Hyperspectral: %1")
            .arg(modeLabels.join(QStringLiteral(" \u2192 "))));
  } else if (captureMultiviewOnlySession_ || captureMultiviewPending_) {
    sections.push_back(QStringLiteral("Hyperspectral: (none — Multiview only)"));
  }

  QStringList cameraLines;
  std::vector<std::size_t> selectedCameras;
  if (selectedCaptureCameraIndices(selectedCameras)) {
    for (const std::size_t cameraIndex : selectedCameras) {
      const LumoCameraUi &ui =
          cameraIndex == 0 ? host_->camera1Ui_ : host_->camera2Ui_;
      cameraLines.push_back(
          host_->cameraPanel()->profileTabNameForUi(ui));
    }
  }
  if (!cameraLines.isEmpty()) {
    sections.push_back(
        QStringLiteral("Cameras: %1")
            .arg(cameraLines.join(QStringLiteral(", "))));
  }

  if (useStageForCapture() && captureScanPlan_.sampleScanTotalDistanceMm > 0.0) {
    sections.push_back(
        QStringLiteral(
            "Stage HSI: sample origin %1 mm, length %2 mm @ %3 mm/s")
            .arg(captureScanPlan_.sampleScanOriginMm, 0, 'f', 1)
            .arg(captureScanPlan_.sampleScanLengthMm, 0, 'f', 1)
            .arg(captureScanPlan_.recordScanSpeedMmPerSec, 0, 'f', 1));
  }

  if (captureMultiviewPending_ && canRunMultiviewRgbCapture()) {
    QStringList multiviewLines;
    multiviewLines.push_back(
        QStringLiteral("Multiview RGB: UR3e / BFS hemisphere scan"));
    multiviewLines.push_back(
        QStringLiteral("Stage Multiview apex: %1 mm, MVS stop: %2 mm")
            .arg(hf::hardwareConfig().sampleMultiviewApexPositionMm, 0, 'f', 1)
            .arg(hf::hardwareConfig().sampleMultiviewPositionMm, 0, 'f', 1));

    if (host_->ur3eHemisphereScanSettings_ != nullptr) {
      const auto params = host_->ur3eHemisphereScanSettings_->params();
      multiviewLines.push_back(
          QStringLiteral("Route grid: R%1 mm, %2×%3, θ%4–%5°")
              .arg(qRound(params.sphereRadiusM * 1000.0))
              .arg(params.horizontalPoints)
              .arg(params.verticalPoints)
              .arg(qRound(params.thetaMinDeg))
              .arg(qRound(params.thetaMaxDeg)));
      if (host_->ur3ePanel() != nullptr && host_->ur3ePanel()->isScanPlanReady()) {
        const int reachable =
            host_->ur3eHemisphereScanSettings_->plannedReachablePins();
        if (reachable > 0) {
          multiviewLines.push_back(
              QStringLiteral("Reachable pins: %1").arg(reachable));
        } else {
          multiviewLines.push_back(QStringLiteral("Plan: ready"));
        }
      }
    }

    sections.push_back(multiviewLines.join(QLatin1Char('\n')));
  }

  if (host_->capturePreprocessAfterScanCheck_ != nullptr &&
      host_->capturePreprocessAfterScanCheck_->isChecked() &&
      captureRecorderMode_ == CaptureRecorderMode::Record) {
    QStringList pp;
    pp.push_back(QStringLiteral("Post-process after scan: on"));
    if (host_->captureSaveFfcImageCheck_ != nullptr &&
        host_->captureSaveFfcImageCheck_->isChecked())
      pp.push_back(QStringLiteral("FFC image"));
    if (host_->captureRunGsamCheck_ != nullptr &&
        host_->captureRunGsamCheck_->isChecked())
      pp.push_back(QStringLiteral("GSAM"));
    if (host_->captureRunHfFusionCheck_ != nullptr &&
        host_->captureRunHfFusionCheck_->isChecked())
      pp.push_back(QStringLiteral("spectral fusion"));
    sections.push_back(pp.join(QStringLiteral(" · ")));
  }

  if (powerStatus != nullptr) {
    QStringList controllerLines;
    controllerLines << lighthouseControllerAliveText(
        *powerStatus, 0, QStringLiteral("Reflectance 1"));
    controllerLines << lighthouseControllerAliveText(
        *powerStatus, 1, QStringLiteral("Reflectance 2"));
    controllerLines << lighthouseControllerAliveText(
        *powerStatus, 2, QStringLiteral("Transmittance 1"));
    controllerLines << lighthouseControllerAliveText(
        *powerStatus, 3, QStringLiteral("Transmittance 2"));
    sections.push_back(
        QStringLiteral(
            "Illumination hoods are prepared manually during the scan — "
            "follow the on-screen prompts.\n\n"
            "Lighthouse controller status:\n%1")
            .arg(controllerLines.join(QLatin1Char('\n'))));
  }

  return sections.join(QStringLiteral("\n\n"));
}

bool hf::capture::CapturePanelController::confirmCaptureHoodPreparation(
    const CaptureIlluminationMode mode,
    const bool betweenReflectanceAndTransmittance) {
  const bool dualMode = betweenReflectanceAndTransmittance ||
                        hasReflectanceAndTransmittanceCaptureModes();
  int remainingSeconds = dualMode ? 10 : 5;

  QMessageBox box(host_);
  box.setIcon(QMessageBox::Information);
  box.setStandardButtons(QMessageBox::Yes | QMessageBox::Cancel);
  box.setDefaultButton(QMessageBox::Yes);
  QAbstractButton *yesButton = box.button(QMessageBox::Yes);
  const auto setYesLabel = [yesButton](const int secondsLeft) {
    if (yesButton == nullptr)
      return;
    yesButton->setText(QStringLiteral("Yes (%1)").arg(secondsLeft));
  };
  setYesLabel(remainingSeconds);

  if (betweenReflectanceAndTransmittance) {
    box.setWindowTitle(QStringLiteral("Prepare transmittance scan"));
    box.setText(QStringLiteral("Reflectance scan finished."));
    QString informativeText = QStringLiteral(
        "Cover the reflectance light guide hood, open the transmittance light "
        "guide hood, then click Yes to start the transmittance scan.");
    if (dualModeExposureSwitchEnabled_ && captureUsingTransmittanceExposure_) {
      informativeText += QStringLiteral("\n\n%1").arg(
          buildTransmittanceExposureChangeNotice());
    }
    box.setInformativeText(informativeText);
  } else if (mode == CaptureIlluminationMode::Reflectance) {
    box.setWindowTitle(QStringLiteral("Prepare reflectance scan"));
    box.setText(QStringLiteral(
        "Cover the transmittance light guide hood, then click Yes "
        "to start the reflectance scan."));
  } else {
    box.setWindowTitle(QStringLiteral("Prepare transmittance scan"));
    box.setText(QStringLiteral(
        "Cover the reflectance light guide hood, then click Yes to "
        "start the transmittance scan."));
  }

  QTimer countdown;
  countdown.setInterval(1000);
  QObject::connect(&countdown, &QTimer::timeout, &box, [&]() {
    --remainingSeconds;
    if (remainingSeconds <= 0) {
      countdown.stop();
      if (yesButton != nullptr)
        yesButton->click();
      return;
    }
    setYesLabel(remainingSeconds);
  });
  countdown.start();

  return box.exec() == QMessageBox::Yes;
}

bool hf::capture::CapturePanelController::confirmContinuousCaptureWithoutStage()
    const {
  const QString reason = isCaptureStageConnected()
                             ? QStringLiteral("Stage scanning is disabled.")
                             : QStringLiteral("The stage is not connected.");

  QMessageBox box(host_);
  box.setIcon(QMessageBox::Information);
  box.setWindowTitle(QStringLiteral("Record without stage"));
  box.setText(QStringLiteral("Recording will not use the stage."));
  box.setInformativeText(
      QStringLiteral("%1\n\nFframes will be saved continuously until you press "
                     "Stop.")
          .arg(reason));
  box.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
  box.setDefaultButton(QMessageBox::Ok);
  if (QAbstractButton *continueButton = box.button(QMessageBox::Ok))
    continueButton->setText(QStringLiteral("Continue"));

  return box.exec() == QMessageBox::Ok;
}

bool hf::capture::CapturePanelController::confirmContinueWithMultiviewAfterHsi()
    const {
  QMessageBox box(host_);
  box.setIcon(QMessageBox::Question);
  box.setWindowTitle(tr("Continue with Multiview?"));
  box.setText(tr("Stage is at the Multiview position."));
  box.setInformativeText(
      tr("Hyperspectral acquisition is complete.\n\n"
         "Continue to run the UR3e / BFS hemisphere scan?\n\n"
         "Choose Skip Multiview to keep only the hyperspectral data "
         "(preprocessing already continues in the background when enabled)."));
  box.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
  box.setDefaultButton(QMessageBox::Yes);
  if (QAbstractButton *yesButton = box.button(QMessageBox::Yes))
    yesButton->setText(tr("Continue Multiview"));
  if (QAbstractButton *noButton = box.button(QMessageBox::No))
    noButton->setText(tr("Skip Multiview"));

  return box.exec() == QMessageBox::Yes;
}

void hf::capture::CapturePanelController::finishRecordAfterSkippingMultiview() {
  host_->appendLog(QStringLiteral(
      "Capture record: Multiview skipped by operator — keeping "
      "hyperspectral data only."));
  captureMultiviewAwaitingOperatorConfirm_ = false;
  captureMultiviewNeedRingsLeg_ = false;
  captureMultiviewFramesSoFar_ = 0;
  captureMultiviewPinsSoFar_ = 0;
  captureMultiviewElapsedMsSoFar_ = 0;
  captureMultiviewPending_ = false;
  captureMultiviewOnlySession_ = false;
  captureMultiviewInProgress_ = false;
  captureMultiviewSessionDirectory_.clear();
  lastCaptureMultiviewSummaryText_.clear();

  if (isStageRecordingEnabledInUi() && host_->stageWorker() != nullptr)
    host_->stageWorker()->requestStopMotion();

  captureRecorderMode_ = CaptureRecorderMode::Idle;
  updateRecorderControls();
  host_->lightPanel()->updateConnectionDisplay();
  host_->lightPanel()->updateControlsEnabled();
  host_->appendLog(QStringLiteral("Capture record: scan sequence finished."));
  homeStageAfterCapture();
  beginRecordCompleteNotify();
}

void hf::capture::CapturePanelController::startCurrentCaptureMode() {
  if (captureCurrentModeIndex_ >= capturePendingIlluminationModes_.size()) {
    completeCaptureSequence();
    return;
  }

  captureRecordingIlluminationMode_ =
      capturePendingIlluminationModes_[captureCurrentModeIndex_];

  host_->appendLog(
      QStringLiteral("%1: starting %2 mode (%3 of %4)\u2026")
          .arg(captureSequenceLogPrefix())
          .arg(captureIlluminationFolderName(captureRecordingIlluminationMode_))
          .arg(captureCurrentModeIndex_ + 1)
          .arg(capturePendingIlluminationModes_.size()));

  if (!confirmCaptureHoodPreparation(captureRecordingIlluminationMode_,
                                     false)) {
    failCaptureSequence(
        QStringLiteral("%1: cancelled by operator at hood preparation.")
            .arg(captureSequenceLogPrefix()));
    return;
  }

  beginCaptureModeMotion();
}

void hf::capture::CapturePanelController::beginCaptureModeMotion() {
  resetCaptureSequenceState();
  beginCaptureMoveToFirstRefPosition();
}

void hf::capture::CapturePanelController::beginCaptureMoveToFirstRefPosition() {
  std::vector<std::size_t> selectedCameras;
  if (!selectedCaptureCameraIndices(selectedCameras)) {
    failCaptureSequence(QStringLiteral("%1: no capture cameras selected.")
                            .arg(captureSequenceLogPrefix()));
    return;
  }

  const auto proceed = [this, selectedCameras](const double currentPositionMm,
                                               const bool positionOk) {
    double targetMm = 0.0;
    bool hasTarget = false;
    double bestDistanceMm = -1.0;

    for (const std::size_t cameraIndex : selectedCameras) {
      const LumoCameraUi &ui =
          cameraIndex == 0 ? host_->camera1Ui_ : host_->camera2Ui_;
      const std::size_t stageCameraIndex =
          stageCameraIndexForUi(ui, cameraIndex);
      const double refMm = whiteRefStartMmForStageCamera(
          captureScanPlan_, captureRecordingIlluminationMode_,
          stageCameraIndex);
      const double distanceMm =
          positionOk ? std::abs(currentPositionMm - refMm) : 0.0;
      if (!hasTarget || distanceMm < bestDistanceMm) {
        hasTarget = true;
        bestDistanceMm = distanceMm;
        targetMm = refMm;
      }
    }

    if (!hasTarget) {
      failCaptureSequence(
          QStringLiteral("%1: no white reference positions available.")
              .arg(captureSequenceLogPrefix()));
      return;
    }

    const QString refLabel = captureRecordingIlluminationMode_ ==
                                     CaptureIlluminationMode::Reflectance
                                 ? QStringLiteral("white reference")
                                 : QStringLiteral("bright reference");

    captureScanPhase_ = CaptureScanPhase::MoveToFirstRefPosition;
    host_->appendLog(
        QStringLiteral("%1 (%2): moving to closest %3 position %4 mm before "
                       "black reference @ %5 mm/s\u2026")
            .arg(captureSequenceLogPrefix())
            .arg(captureIlluminationFolderName(
                captureRecordingIlluminationMode_))
            .arg(refLabel)
            .arg(targetMm, 0, 'f', 2)
            .arg(captureScanPlan_.operationSpeedMmPerSec, 0, 'f', 1));
    requestCaptureAbsoluteMove(targetMm,
                               CaptureScanPhase::MoveToFirstRefPosition);
    updateRecorderStatus();
  };

  if (host_->stageWorker() == nullptr) {
    failCaptureSequence(QStringLiteral("%1: stage worker unavailable.")
                            .arg(captureSequenceLogPrefix()));
    return;
  }

  host_->stageWorker()->requestPrimaryPosition(
      [this, proceed](const double positionMm, const bool ok) {
        QMetaObject::invokeMethod(
            this,
            [this, proceed, positionMm, ok]() { proceed(positionMm, ok); },
            Qt::QueuedConnection);
      });
}

void hf::capture::CapturePanelController::beginCaptureMoveToTempStopPosition() {
  const double targetMm = hf::hardwareConfig().tempStopPositionMm;
  captureScanPhase_ = CaptureScanPhase::MoveToTempStopPosition;
  host_->appendLog(
      QStringLiteral("%1: moving to temp stop position %2 mm before "
                     "transmittance scan @ %3 mm/s\u2026")
          .arg(captureSequenceLogPrefix())
          .arg(targetMm, 0, 'f', 2)
          .arg(captureScanPlan_.operationSpeedMmPerSec, 0, 'f', 1));
  requestCaptureAbsoluteMove(targetMm,
                             CaptureScanPhase::MoveToTempStopPosition);
  updateRecorderStatus();
}

void hf::capture::CapturePanelController::setSelectedCameraShutters(
    const bool open) {
  if (host_->coordinator() == nullptr)
    return;

  std::vector<std::size_t> cameraIndices;
  if (!selectedCaptureCameraIndices(cameraIndices))
    return;

  for (const std::size_t index : cameraIndices) {
    if (open)
      host_->coordinator()->openShutter(index);
    else
      host_->coordinator()->closeShutter(index);
  }
}

bool hf::capture::CapturePanelController::
    selectedCamerasReachedBlackReferenceTarget() const {
  std::vector<std::size_t> cameraIndices;
  if (!selectedCaptureCameraIndices(cameraIndices))
    return false;

  for (const std::size_t index : cameraIndices) {
    if (captureBlackRefFramesCollected_[index] <
        captureScanPlan_.blackReferenceFrameCount)
      return false;
  }

  return true;
}

bool hf::capture::CapturePanelController::shouldAcceptBlackReferenceFrame(
    const std::size_t cameraIndex) const {
  if (captureScanPhase_ != CaptureScanPhase::BlackReference)
    return false;

  if (!captureBlackReferenceCollectionActive_)
    return false;

  if (cameraIndex >= captureBlackRefFramesCollected_.size())
    return false;

  return captureBlackRefFramesCollected_[cameraIndex] <
         captureScanPlan_.blackReferenceFrameCount;
}

bool hf::capture::CapturePanelController::selectedCaptureCameraShuttersReportedClosed() const {
  std::vector<std::size_t> cameraIndices;
  if (!selectedCaptureCameraIndices(cameraIndices))
    return true;

  for (const std::size_t index : cameraIndices) {
    const LumoCameraUi &ui = index == 0 ? host_->camera1Ui_ : host_->camera2Ui_;
    if (ui.shutterReportedOpen)
      return false;
  }

  return true;
}

void hf::capture::CapturePanelController::activateBlackReferenceCollection(
    const QString &logLine) {
  if (captureScanPhase_ != CaptureScanPhase::BlackReference ||
      captureBlackReferenceCollectionActive_)
    return;

  captureBlackReferenceCollectionActive_ = true;
  host_->appendLog(logLine);
  updateRecorderStatus();
}

void hf::capture::CapturePanelController::scheduleBlackReferenceShutterSettleCheck() {
  if (captureScanPhase_ != CaptureScanPhase::BlackReference)
    return;

  std::vector<std::size_t> cameraIndices;
  if (host_->coordinator() != nullptr && selectedCaptureCameraIndices(cameraIndices)) {
    for (const std::size_t index : cameraIndices)
      host_->coordinator()->refreshShutterState(index);
  }

  const qint64 elapsedMs =
      captureBlackRefShutterSettleTimer_.isValid() ? captureBlackRefShutterSettleTimer_.elapsed() : 0;
  const int minSettleMs = std::max(0, captureScanPlan_.blackReferenceShutterSettleMs);
  const int maxWaitMs = std::max(minSettleMs + 3000, minSettleMs * 3);
  const bool minDelayMet = elapsedMs >= minSettleMs;
  const bool shuttersClosed = selectedCaptureCameraShuttersReportedClosed();

  if (minDelayMet && shuttersClosed) {
    activateBlackReferenceCollection(
        QStringLiteral("%1: shutters closed \u2014 collecting black reference frames\u2026")
            .arg(captureSequenceLogPrefix()));
    return;
  }

  if (elapsedMs >= maxWaitMs) {
    QStringList openCameras;
    for (const std::size_t index : cameraIndices) {
      const LumoCameraUi &ui = index == 0 ? host_->camera1Ui_ : host_->camera2Ui_;
      if (ui.shutterReportedOpen)
        openCameras.push_back(host_->cameraPanel()->profileTabNameForUi(ui));
    }

    if (openCameras.isEmpty()) {
      host_->appendLog(
          QStringLiteral("%1: black reference shutter settle timeout \u2014 collecting "
                         "frames anyway.")
              .arg(captureSequenceLogPrefix()));
    } else {
      host_->appendLog(
          QStringLiteral("%1: shutter settle timeout with %2 still open \u2014 collecting "
                         "frames anyway (black reference may be biased).")
              .arg(captureSequenceLogPrefix(), openCameras.join(QStringLiteral(", "))));
    }

    activateBlackReferenceCollection(
        QStringLiteral("%1: collecting black reference frames\u2026")
            .arg(captureSequenceLogPrefix()));
    return;
  }

  QTimer::singleShot(100, this, [this]() { scheduleBlackReferenceShutterSettleCheck(); });
}

void hf::capture::CapturePanelController::requestCaptureAbsoluteMove(
    const double positionMm, const CaptureScanPhase expectedPhaseOnComplete,
    const double speedMmPerSec) {
  captureMoveCompletePhase_ = expectedPhaseOnComplete;
  const double moveSpeed = speedMmPerSec > 0.0
                               ? speedMmPerSec
                               : captureScanPlan_.operationSpeedMmPerSec;
  host_->stageWorker()->requestMoveAbsoluteMm(
      positionMm, moveSpeed, true, [this](const bool success) {
        QMetaObject::invokeMethod(
            this, [this, success]() { onCaptureAbsoluteMoveComplete(success); },
            Qt::QueuedConnection);
      });
}

void hf::capture::CapturePanelController::onCaptureAbsoluteMoveComplete(
    const bool success) {
  if (captureRecorderMode_ == CaptureRecorderMode::Idle)
    return;

  const CaptureScanPhase completedPhase = captureMoveCompletePhase_;
  captureMoveCompletePhase_ = CaptureScanPhase::Idle;

  if (!success) {
    failCaptureSequence(QStringLiteral("%1: stage move failed.")
                            .arg(captureSequenceLogPrefix()));
    return;
  }

  if (completedPhase == CaptureScanPhase::MoveToFirstRefPosition) {
    beginCaptureBlackReference();
    return;
  }

  if (completedPhase == CaptureScanPhase::MoveToTempStopPosition) {
    if (dualModeExposureSwitchEnabled_) {
      applyTransmittanceExposuresFromConfig();
      captureUsingTransmittanceExposure_ = true;
    }

    if (!confirmCaptureHoodPreparation(CaptureIlluminationMode::Transmittance,
                                       true)) {
      failCaptureSequence(
          QStringLiteral("%1: cancelled by operator at hood preparation.")
              .arg(captureSequenceLogPrefix()));
      return;
    }

    beginCaptureModeMotion();
    return;
  }

  if (completedPhase == CaptureScanPhase::MoveToWhiteRefScanOrigin) {
    beginCombinedRecordScan();
    return;
  }

  if (completedPhase == CaptureScanPhase::MoveToSampleScanOrigin) {
    verifySampleScanOriginAndStartScan();
    return;
  }

  if (completedPhase == CaptureScanPhase::MoveToMultiviewPosition) {
    if (captureMultiviewAwaitingOperatorConfirm_) {
      captureMultiviewAwaitingOperatorConfirm_ = false;
      if (!confirmContinueWithMultiviewAfterHsi()) {
        finishRecordAfterSkippingMultiview();
        return;
      }
      host_->appendLog(QStringLiteral(
          "Capture record: operator confirmed Multiview — starting hemisphere capture…"));
    }

    startHemisphereMultiviewCaptureAfterStageMove();
    return;
  }
}

void hf::capture::CapturePanelController::beginCaptureBlackReference() {
  captureScanPhase_ = CaptureScanPhase::BlackReference;
  captureBlackRefFramesCollected_ = {0, 0};
  captureBlackReferenceCollectionActive_ = false;
  captureBlackRefShutterSettleTimer_.restart();

  host_->appendLog(QStringLiteral("%1: closing shutters for black reference "
                                  "(%2 frames per camera)\u2026")
                       .arg(captureSequenceLogPrefix())
                       .arg(captureScanPlan_.blackReferenceFrameCount));

  setSelectedCameraShutters(false);
  scheduleBlackReferenceShutterSettleCheck();
  updateRecorderStatus();
}

void hf::capture::CapturePanelController::onCaptureBlackReferenceComplete() {
  if (captureScanPhase_ != CaptureScanPhase::BlackReference)
    return;

  captureScanPhase_ = CaptureScanPhase::Idle;

  host_->appendLog(QStringLiteral("%1: black reference complete \u2014 opening "
                                  "shutters for white reference\u2026")
                       .arg(captureSequenceLogPrefix()));

  setSelectedCameraShutters(true);

  QTimer::singleShot(750, this, [this]() { beginCaptureRecordScanSequence(); });
  updateRecorderStatus();
}

void hf::capture::CapturePanelController::beginCaptureRecordScanSequence() {
  captureWhiteRefFramesCollected_ = {0, 0};
  captureSampleFramesCollected_ = {0, 0};
  captureWhiteRefWindowComplete_ = {false, false};
  captureSampleWindowComplete_ = {false, false};
  captureSampleWindowEntered_ = {false, false};
  updateCombinedRecordScanGeometry();
  beginCaptureMoveToWhiteRefScanOrigin();
}

void hf::capture::CapturePanelController::updateCombinedRecordScanGeometry() {
  std::vector<std::size_t> selectedCameras;
  if (!selectedCaptureCameraIndices(selectedCameras))
    return;

  bool hasOrigin = false;
  double originMm = 0.0;
  double endMm = 0.0;

  for (const std::size_t cameraIndex : selectedCameras) {
    const LumoCameraUi &ui =
        cameraIndex == 0 ? host_->camera1Ui_ : host_->camera2Ui_;
    const std::size_t stageCameraIndex = stageCameraIndexForUi(ui, cameraIndex);
    const double refStart = whiteRefStartMmForStageCamera(
        captureScanPlan_, captureRecordingIlluminationMode_, stageCameraIndex);
    const double refDistance =
        captureScanPlan_.whiteRefScanDistanceMm[stageCameraIndex] > 0.0
            ? captureScanPlan_.whiteRefScanDistanceMm[stageCameraIndex]
            : whiteReferenceScanDistanceMmForCamera(ui, captureScanPlan_);
    captureScanPlan_.whiteRefScanDistanceMm[stageCameraIndex] = refDistance;

    const double refEnd = refStart + refDistance;
    const double sampleStart =
        captureScanPlan_.sampleScanStartMm[stageCameraIndex];
    const double sampleEnd = sampleStart + captureScanPlan_.sampleScanLengthMm;

    if (!hasOrigin) {
      originMm = std::min({refStart, sampleStart});
      endMm = std::max({refEnd, sampleEnd});
      hasOrigin = true;
    } else {
      originMm = std::min({originMm, refStart, sampleStart});
      endMm = std::max({endMm, refEnd, sampleEnd});
    }
  }

  captureScanPlan_.recordScanOriginMm = originMm;
  captureScanPlan_.recordScanTotalDistanceMm = std::max(0.0, endMm - originMm);
  captureScanPlan_.whiteRefScanOriginMm = originMm;
  captureScanPlan_.whiteRefScanTotalDistanceMm =
      captureScanPlan_.recordScanTotalDistanceMm;
  captureScanPlan_.sampleScanOriginMm = originMm;
  captureScanPlan_.sampleScanTotalDistanceMm =
      captureScanPlan_.recordScanTotalDistanceMm;
}

void hf::capture::CapturePanelController::
    beginCaptureMoveToWhiteRefScanOrigin() {
  captureScanPhase_ = CaptureScanPhase::MoveToWhiteRefScanOrigin;
  host_->appendLog(
      QStringLiteral(
          "%1 (%2): moving to record scan origin %3 mm @ %4 mm/s\u2026")
          .arg(captureSequenceLogPrefix())
          .arg(captureIlluminationFolderName(captureRecordingIlluminationMode_))
          .arg(captureScanPlan_.recordScanOriginMm, 0, 'f', 2)
          .arg(captureScanPlan_.operationSpeedMmPerSec, 0, 'f', 1));
  requestCaptureAbsoluteMove(captureScanPlan_.recordScanOriginMm,
                             CaptureScanPhase::MoveToWhiteRefScanOrigin,
                             captureScanPlan_.operationSpeedMmPerSec);
  updateRecorderStatus();
}

void hf::capture::CapturePanelController::beginCombinedRecordScan() {
  if (captureRecorderMode_ == CaptureRecorderMode::Preview)
    setSelectedCameraShutters(true);

  const QString refLabel =
      captureRecordingIlluminationMode_ == CaptureIlluminationMode::Reflectance
          ? QStringLiteral("white-reference")
          : QStringLiteral("bright-reference");

  host_->appendLog(
      QStringLiteral("%1: one-pass record scan \u2014 %2 + sample windows, %3 "
                     "white-ref frames per "
                     "camera @ %4 mm/s (stage travel %5 mm)\u2026")
          .arg(captureSequenceLogPrefix())
          .arg(refLabel)
          .arg(captureScanPlan_.whiteReferenceFrameCount)
          .arg(captureScanPlan_.recordScanSpeedMmPerSec, 0, 'f', 1)
          .arg(captureScanPlan_.recordScanTotalDistanceMm, 0, 'f', 2));

  startCaptureRelativeScan(captureScanPlan_.recordScanTotalDistanceMm,
                           captureScanPlan_.recordScanSpeedMmPerSec,
                           CaptureScanPhase::CombinedRecordScan);
  updateRecorderStatus();
}

void hf::capture::CapturePanelController::
    onCaptureWhiteReferenceSequenceComplete() {
  if (captureScanPhase_ != CaptureScanPhase::WhiteReferenceScan)
    return;

  if (captureScanTimer_ != nullptr)
    captureScanTimer_->stop();
  captureRelativeScanTimerActiveToken_ = 0;

  if (host_->stageWorker() != nullptr)
    host_->stageWorker()->requestStopMotion();

  captureScanTimingActive_ = false;
  captureStagePositionKnown_ = false;

  if (!selectedCamerasReachedWhiteReferenceTarget()) {
    failCaptureSequence(
        QStringLiteral(
            "%1: white/bright reference incomplete \u2014 check frame rate and "
            "reference positions.")
            .arg(captureSequenceLogPrefix()));
    return;
  }

  const QString refLabel =
      captureRecordingIlluminationMode_ == CaptureIlluminationMode::Reflectance
          ? QStringLiteral("White reference")
          : QStringLiteral("Bright reference");

  std::vector<std::size_t> selectedCameras;
  if (selectedCaptureCameraIndices(selectedCameras)) {
    for (const std::size_t cameraIndex : selectedCameras) {
      const LumoCameraUi &ui =
          cameraIndex == 0 ? host_->camera1Ui_ : host_->camera2Ui_;
      host_->appendLog(QStringLiteral("%1: %2 complete for %3 (%4 frames).")
                           .arg(captureSequenceLogPrefix())
                           .arg(refLabel)
                           .arg(host_->cameraPanel()->profileTabNameForUi(ui))
                           .arg(captureWhiteRefFramesCollected_[cameraIndex]));
    }
  }

  beginCaptureSampleScan();
  updateRecorderStatus();
}

void hf::capture::CapturePanelController::onWhiteReferenceFrameCollected(
    const std::size_t cameraIndex) {
  ++captureWhiteRefFramesCollected_[cameraIndex];

  if (captureScanPhase_ == CaptureScanPhase::WhiteReferenceScan) {
    if (selectedCamerasReachedWhiteReferenceTarget())
      onCaptureWhiteReferenceSequenceComplete();
    else
      updateRecorderStatus();
    return;
  }

  updateRecorderStatus();
}

bool hf::capture::CapturePanelController::
    selectedCamerasReachedWhiteReferenceTarget() const {
  std::vector<std::size_t> cameraIndices;
  if (!selectedCaptureCameraIndices(cameraIndices))
    return false;

  for (const std::size_t index : cameraIndices) {
    if (captureWhiteRefFramesCollected_[index] <
        captureScanPlan_.whiteReferenceFrameCount)
      return false;
  }

  return true;
}

bool hf::capture::CapturePanelController::
    shouldRecordWhiteReferenceFrameForCamera(
        const std::size_t stageCameraIndex, const std::size_t cameraIndex,
        const double stagePositionMm) const {
  if (!isStageScanPositionTrustworthy(stagePositionMm))
    return false;

  if (stageCameraIndex >= captureWhiteRefWindowComplete_.size() ||
      captureWhiteRefWindowComplete_[stageCameraIndex]) {
    return false;
  }

  if (captureWhiteRefFramesCollected_[cameraIndex] >=
      captureScanPlan_.whiteReferenceFrameCount)
    return false;

  const double windowStart = whiteRefStartMmForStageCamera(
      captureScanPlan_, captureRecordingIlluminationMode_, stageCameraIndex);
  const double windowLength =
      captureScanPlan_.whiteRefScanDistanceMm[stageCameraIndex];
  if (windowLength <= 0.0)
    return false;

  const double windowEnd = windowStart + windowLength;
  if (stagePositionMm - 0.05 > windowEnd)
    return false;

  return isStagePositionWithinSampleWindow(stagePositionMm, windowStart,
                                           windowLength);
}

bool hf::capture::CapturePanelController::
    allSelectedCamerasPastWhiteReferenceWindow(
        const double stagePositionMm) const {
  std::vector<std::size_t> selectedCameras;
  if (!selectedCaptureCameraIndices(selectedCameras))
    return true;

  for (const std::size_t cameraIndex : selectedCameras) {
    const LumoCameraUi &ui =
        cameraIndex == 0 ? host_->camera1Ui_ : host_->camera2Ui_;
    const std::size_t stageCameraIndex = stageCameraIndexForUi(ui, cameraIndex);
    const double windowEnd =
        whiteRefStartMmForStageCamera(captureScanPlan_,
                                      captureRecordingIlluminationMode_,
                                      stageCameraIndex) +
        captureScanPlan_.whiteRefScanDistanceMm[stageCameraIndex];
    constexpr double kPositionToleranceMm = 0.05;
    if (stagePositionMm + kPositionToleranceMm < windowEnd)
      return false;
  }

  return true;
}

bool hf::capture::CapturePanelController::shouldAcceptWhiteReferenceFrame(
    const std::size_t cameraIndex) const {
  if (captureScanPhase_ != CaptureScanPhase::WhiteReferenceScan &&
      captureScanPhase_ != CaptureScanPhase::CombinedRecordScan) {
    return false;
  }

  const LumoCameraUi &ui =
      cameraIndex == 0 ? host_->camera1Ui_ : host_->camera2Ui_;
  const std::size_t stageCameraIndex = stageCameraIndexForUi(ui, cameraIndex);
  return shouldRecordWhiteReferenceFrameForCamera(stageCameraIndex, cameraIndex,
                                                  currentStageScanPositionMm());
}

double
hf::capture::CapturePanelController::whiteReferenceScanDistanceMmForCamera(
    const LumoCameraUi &ui, const CaptureScanPlan &plan) const {
  const CameraSettings settings = host_->cameraPanel()->buildSettings(ui);
  const double frameRateHz = std::max(1.0, settings.frameRateHz);
  const double durationSec =
      static_cast<double>(plan.whiteReferenceFrameCount) / frameRateHz;
  constexpr double kMarginFactor = 1.25;
  constexpr double kMarginMm = 2.0;
  return std::max(1.0,
                  durationSec * plan.recordScanSpeedMmPerSec * kMarginFactor +
                      kMarginMm);
}

void hf::capture::CapturePanelController::beginCaptureSampleScan() {
  if (captureScanTimer_ != nullptr)
    captureScanTimer_->stop();
  captureRelativeScanTimerActiveToken_ = 0;

  captureSampleRecordingActive_ = false;
  captureSampleFramesCollected_ = {0, 0};
  captureSampleScanTimerExtendCount_ = 0;
  captureStagePositionKnown_ = false;
  captureScanPhase_ = CaptureScanPhase::MoveToSampleScanOrigin;
  host_->appendLog(
      QStringLiteral(
          "%1 (%2): moving to sample scan origin %3 mm @ %4 mm/s\u2026")
          .arg(captureSequenceLogPrefix())
          .arg(captureIlluminationFolderName(captureRecordingIlluminationMode_))
          .arg(captureScanPlan_.sampleScanOriginMm, 0, 'f', 2)
          .arg(captureScanPlan_.operationSpeedMmPerSec, 0, 'f', 1));
  requestCaptureAbsoluteMove(captureScanPlan_.sampleScanOriginMm,
                             CaptureScanPhase::MoveToSampleScanOrigin,
                             captureScanPlan_.operationSpeedMmPerSec);
  updateRecorderStatus();
}

void hf::capture::CapturePanelController::verifySampleScanOriginAndStartScan() {
  if (captureRecorderMode_ == CaptureRecorderMode::Idle ||
      host_->stageWorker() == nullptr)
    return;

  host_->stageWorker()->requestPrimaryPosition([this](const double positionMm,
                                                      const bool ok) {
    QMetaObject::invokeMethod(
        this,
        [this, positionMm, ok]() {
          if (captureRecorderMode_ == CaptureRecorderMode::Idle ||
              captureScanPhase_ != CaptureScanPhase::MoveToSampleScanOrigin) {
            return;
          }

          if (!ok) {
            failCaptureSequence(
                QStringLiteral(
                    "%1: could not read stage position at sample scan origin.")
                    .arg(captureSequenceLogPrefix()));
            return;
          }

          constexpr double kOriginToleranceMm = 5.0;
          const double expectedOrigin = captureScanPlan_.sampleScanOriginMm;
          if (std::abs(positionMm - expectedOrigin) > kOriginToleranceMm) {
            failCaptureSequence(
                QStringLiteral("%1: sample scan origin mismatch (read %2 mm, "
                               "expected %3 mm).")
                    .arg(captureSequenceLogPrefix())
                    .arg(positionMm, 0, 'f', 2)
                    .arg(expectedOrigin, 0, 'f', 2));
            return;
          }

          startCaptureRelativeScan(captureScanPlan_.sampleScanTotalDistanceMm,
                                   captureScanPlan_.recordScanSpeedMmPerSec,
                                   CaptureScanPhase::SampleScan);
        },
        Qt::QueuedConnection);
  });
}

void hf::capture::CapturePanelController::scheduleRelativeScanTimer(
    const double distanceMm, const double speedMmPerSec) {
  if (captureScanTimer_ == nullptr || speedMmPerSec <= 0.0 || distanceMm <= 0.0)
    return;

  captureScanTimer_->stop();
  const int durationMs =
      static_cast<int>(std::ceil((distanceMm / speedMmPerSec) * 1000.0)) + 750;
  captureRelativeScanTimerActiveToken_ = ++captureRelativeScanTimerToken_;
  captureScanTimer_->start(std::max(durationMs, 500));
}

void hf::capture::CapturePanelController::extendSampleScanTimer() {
  if (captureScanTimer_ == nullptr ||
      captureScanPlan_.recordScanSpeedMmPerSec <= 0.0)
    return;

  const double elapsedSec =
      static_cast<double>(captureScanElapsed_.elapsed()) / 1000.0;
  const double scanDistanceMm =
      captureScanPhase_ == CaptureScanPhase::CombinedRecordScan
          ? captureScanPlan_.recordScanTotalDistanceMm
          : captureScanPlan_.sampleScanTotalDistanceMm;
  const double totalSec =
      scanDistanceMm / captureScanPlan_.recordScanSpeedMmPerSec;
  constexpr double kGraceSec = 2.0;

  if (elapsedSec >= totalSec + kGraceSec) {
    if (canCompleteSampleScan()) {
      onCaptureSampleScanComplete();
      return;
    }

    const std::optional<double> stagePositionMm = knownStageScanPositionMm();
    failCaptureSequence(
        QStringLiteral("%1: sample scan timed out (entered=%2, frames=%3/%4, "
                       "stage=%5 mm).")
            .arg(captureSequenceLogPrefix())
            .arg(selectedCamerasEnteredSampleWindow() ? QStringLiteral("yes")
                                                      : QStringLiteral("no"))
            .arg(captureSampleFramesCollected_[0])
            .arg(captureSampleFramesCollected_[1])
            .arg(stagePositionMm ? QString::number(*stagePositionMm, 'f', 2)
                                 : QStringLiteral("unknown")));
    return;
  }

  const double remainingSec = std::max(1.0, totalSec - elapsedSec + 0.75);
  captureScanTimer_->stop();
  captureRelativeScanTimerActiveToken_ = ++captureRelativeScanTimerToken_;
  captureScanTimer_->start(static_cast<int>(std::ceil(remainingSec * 1000.0)));

  ++captureSampleScanTimerExtendCount_;
  if (captureSampleScanTimerExtendCount_ == 1 ||
      captureSampleScanTimerExtendCount_ % 5 == 0) {
    host_->appendLog(QStringLiteral("%1: sample scan still in progress "
                                    "(entered=%2, frames=%3/%4) \u2014 "
                                    "extending scan timer.")
                         .arg(captureSequenceLogPrefix())
                         .arg(selectedCamerasEnteredSampleWindow()
                                  ? QStringLiteral("yes")
                                  : QStringLiteral("no"))
                         .arg(captureSampleFramesCollected_[0])
                         .arg(captureSampleFramesCollected_[1]));
  }
}

void hf::capture::CapturePanelController::startCaptureRelativeScan(
    const double distanceMm, const double speedMmPerSec,
    const CaptureScanPhase capturePhaseOnMoveStart) {
  host_->stageWorker()->requestPrimaryPosition([this, distanceMm, speedMmPerSec,
                                                capturePhaseOnMoveStart](
                                                   const double positionMm,
                                                   const bool ok) {
    QMetaObject::invokeMethod(
        this,
        [this, positionMm, ok, distanceMm, speedMmPerSec,
         capturePhaseOnMoveStart]() {
          if (captureRecorderMode_ == CaptureRecorderMode::Idle)
            return;

          if (!ok) {
            failCaptureSequence(
                QStringLiteral("%1: could not read stage position.")
                    .arg(captureSequenceLogPrefix()));
            return;
          }

          const double endMm = positionMm + distanceMm;
          if (endMm > zaber_stage::kTravelLengthMm) {
            failCaptureSequence(
                QStringLiteral(
                    "%1: scan would pass %2 mm (current %3 mm + %4 mm).")
                    .arg(captureSequenceLogPrefix())
                    .arg(zaber_stage::kTravelLengthMm, 0, 'f', 0)
                    .arg(positionMm, 0, 'f', 2)
                    .arg(distanceMm, 0, 'f', 2));
            return;
          }

          if (capturePhaseOnMoveStart == CaptureScanPhase::SampleScan ||
              capturePhaseOnMoveStart == CaptureScanPhase::CombinedRecordScan) {
            constexpr double kOriginToleranceMm = 5.0;
            const double expectedOrigin = captureScanPlan_.recordScanOriginMm;
            if (std::abs(positionMm - expectedOrigin) > kOriginToleranceMm) {
              failCaptureSequence(
                  QStringLiteral("%1: record scan origin mismatch (read %2 mm, "
                                 "expected %3 mm).")
                      .arg(captureSequenceLogPrefix())
                      .arg(positionMm, 0, 'f', 2)
                      .arg(expectedOrigin, 0, 'f', 2));
              return;
            }
          }

          host_->stageWorker()->requestMoveRelativeMm(distanceMm,
                                                      speedMmPerSec);
          if (capturePhaseOnMoveStart == CaptureScanPhase::SampleScan ||
              capturePhaseOnMoveStart == CaptureScanPhase::CombinedRecordScan) {
            captureScanOriginPositionMm_ = captureScanPlan_.recordScanOriginMm;
            captureSampleWindowComplete_ = {false, false};
            captureSampleWindowEntered_ = {false, false};
            captureSampleRecordingActive_ = true;
            captureSampleScanTimerExtendCount_ = 0;
          } else {
            captureScanOriginPositionMm_ = positionMm;
          }
          captureLastKnownStagePositionMm_ = positionMm;
          captureStagePositionKnown_ = true;
          captureActiveScanDistanceMm_ = distanceMm;
          captureScanElapsed_.restart();
          captureScanTimingActive_ = true;
          if (capturePhaseOnMoveStart == CaptureScanPhase::WhiteReferenceScan ||
              capturePhaseOnMoveStart == CaptureScanPhase::CombinedRecordScan) {
            captureWhiteRefWindowComplete_ = {false, false};
          }
          if (capturePhaseOnMoveStart != CaptureScanPhase::Idle)
            captureScanPhase_ = capturePhaseOnMoveStart;

          updateRecorderStatus();

          scheduleRelativeScanTimer(distanceMm, speedMmPerSec);
        },
        Qt::QueuedConnection);
  });
}

void hf::capture::CapturePanelController::onCaptureRelativeScanComplete() {
  if (captureRecorderMode_ == CaptureRecorderMode::Idle)
    return;

  if (captureRelativeScanTimerActiveToken_ != captureRelativeScanTimerToken_)
    return;

  if (captureScanPhase_ == CaptureScanPhase::SampleScan) {
    if (!canCompleteSampleScan()) {
      extendSampleScanTimer();
      return;
    }

    onCaptureSampleScanComplete();
    return;
  }

  if (captureScanPhase_ == CaptureScanPhase::CombinedRecordScan) {
    if (!canCompleteCombinedRecordScan()) {
      extendSampleScanTimer();
      return;
    }

    onCaptureCombinedRecordScanComplete();
    return;
  }

  if (host_->stageWorker() != nullptr)
    host_->stageWorker()->requestStopMotion();

  captureScanTimingActive_ = false;

  if (captureScanPhase_ == CaptureScanPhase::WhiteReferenceScan) {
    if (selectedCamerasReachedWhiteReferenceTarget()) {
      onCaptureWhiteReferenceSequenceComplete();
      return;
    }

    failCaptureSequence(
        QStringLiteral(
            "%1: white/bright reference incomplete \u2014 not all cameras "
            "reached %2 frames.")
            .arg(captureSequenceLogPrefix())
            .arg(captureScanPlan_.whiteReferenceFrameCount));
    return;
  }
}

void hf::capture::CapturePanelController::onCaptureSampleScanComplete() {
  if (captureScanPhase_ != CaptureScanPhase::SampleScan)
    return;

  if (captureScanTimer_ != nullptr)
    captureScanTimer_->stop();
  captureRelativeScanTimerActiveToken_ = 0;

  if (host_->stageWorker() != nullptr)
    host_->stageWorker()->requestStopMotion();

  captureScanTimingActive_ = false;
  captureStagePositionKnown_ = false;

  if (captureRecorderMode_ == CaptureRecorderMode::Record &&
      !selectedCamerasHaveSampleFrames()) {
    failCaptureSequence(
        QStringLiteral("%1: sample scan finished with no sample frames \u2014 "
                       "check stage position, "
                       "scan speed, and camera streaming.")
            .arg(captureSequenceLogPrefix()));
    return;
  }

  if (captureRecorderMode_ == CaptureRecorderMode::Record) {
    std::vector<std::size_t> selectedCameras;
    if (selectedCaptureCameraIndices(selectedCameras)) {
      for (const std::size_t cameraIndex : selectedCameras) {
        const LumoCameraUi &ui =
            cameraIndex == 0 ? host_->camera1Ui_ : host_->camera2Ui_;
        host_->appendLog(
            QStringLiteral("%1: sample capture complete for %2 (%3 frames).")
                .arg(captureSequenceLogPrefix())
                .arg(host_->cameraPanel()->profileTabNameForUi(ui))
                .arg(captureSampleFramesCollected_[cameraIndex]));
      }
    }
  }

  completeCaptureModeSequence();
}

void hf::capture::CapturePanelController::
    onCaptureCombinedRecordScanComplete() {
  if (captureScanPhase_ != CaptureScanPhase::CombinedRecordScan)
    return;

  if (captureScanTimer_ != nullptr)
    captureScanTimer_->stop();
  captureRelativeScanTimerActiveToken_ = 0;

  if (host_->stageWorker() != nullptr)
    host_->stageWorker()->requestStopMotion();

  captureScanTimingActive_ = false;
  captureStagePositionKnown_ = false;
  captureSampleRecordingActive_ = false;

  if (!selectedCamerasReachedWhiteReferenceTarget()) {
    failCaptureSequence(
        QStringLiteral(
            "%1: record scan finished with incomplete white/bright reference.")
            .arg(captureSequenceLogPrefix()));
    return;
  }

  if (captureRecorderMode_ == CaptureRecorderMode::Record &&
      !selectedCamerasHaveSampleFrames()) {
    failCaptureSequence(
        QStringLiteral("%1: record scan finished with no sample frames \u2014 "
                       "check stage position, "
                       "scan speed, and camera streaming.")
            .arg(captureSequenceLogPrefix()));
    return;
  }

  if (captureRecorderMode_ == CaptureRecorderMode::Record) {
    std::vector<std::size_t> selectedCameras;
    if (selectedCaptureCameraIndices(selectedCameras)) {
      const QString refLabel = captureRecordingIlluminationMode_ ==
                                       CaptureIlluminationMode::Reflectance
                                   ? QStringLiteral("White reference")
                                   : QStringLiteral("Bright reference");

      for (const std::size_t cameraIndex : selectedCameras) {
        const LumoCameraUi &ui =
            cameraIndex == 0 ? host_->camera1Ui_ : host_->camera2Ui_;
        host_->appendLog(
            QStringLiteral("%1: %2 complete for %3 (%4 frames).")
                .arg(captureSequenceLogPrefix())
                .arg(refLabel)
                .arg(host_->cameraPanel()->profileTabNameForUi(ui))
                .arg(captureWhiteRefFramesCollected_[cameraIndex]));
        host_->appendLog(
            QStringLiteral("%1: sample capture complete for %2 (%3 frames).")
                .arg(captureSequenceLogPrefix())
                .arg(host_->cameraPanel()->profileTabNameForUi(ui))
                .arg(captureSampleFramesCollected_[cameraIndex]));
      }
    }
  }

  completeCaptureModeSequence();
}

void hf::capture::CapturePanelController::failCaptureSequence(
    const QString &message) {
  host_->appendLog(message);
  restoreReflectanceExposuresAfterCapture();
  stopRecorder();
}

void hf::capture::CapturePanelController::completeCaptureModeSequence() {
  if (captureRecorderMode_ == CaptureRecorderMode::Preview) {
    for (std::size_t cameraIndex = 0; cameraIndex < 2; ++cameraIndex)
      lastPreviewCompleteSampleFrames_[cameraIndex] +=
          captureSampleFramesCollected_[cameraIndex];

    std::vector<std::size_t> selectedCameras;
    if (selectedCaptureCameraIndices(selectedCameras)) {
      for (const std::size_t cameraIndex : selectedCameras) {
        if (std::find(lastPreviewCompleteCameraIndices_.begin(),
                      lastPreviewCompleteCameraIndices_.end(),
                      cameraIndex) == lastPreviewCompleteCameraIndices_.end())
          lastPreviewCompleteCameraIndices_.push_back(cameraIndex);
      }
    }
  }

  resetCaptureSequenceState();

  const CaptureIlluminationMode completedMode =
      captureRecordingIlluminationMode_;
  host_->appendLog(QStringLiteral("%1: %2 mode finished.")
                       .arg(captureSequenceLogPrefix())
                       .arg(captureIlluminationFolderName(completedMode)));

  ++captureCurrentModeIndex_;
  if (captureCurrentModeIndex_ < capturePendingIlluminationModes_.size()) {
    captureRecordingIlluminationMode_ =
        capturePendingIlluminationModes_[captureCurrentModeIndex_];

    host_->appendLog(QStringLiteral("%1: starting %2 mode (%3 of %4)\u2026")
                         .arg(captureSequenceLogPrefix())
                         .arg(captureIlluminationFolderName(
                             captureRecordingIlluminationMode_))
                         .arg(captureCurrentModeIndex_ + 1)
                         .arg(capturePendingIlluminationModes_.size()));

    if (completedMode == CaptureIlluminationMode::Reflectance &&
        captureRecordingIlluminationMode_ ==
            CaptureIlluminationMode::Transmittance) {
      beginCaptureMoveToTempStopPosition();
      return;
    }

    if (!confirmCaptureHoodPreparation(captureRecordingIlluminationMode_,
                                       false)) {
      failCaptureSequence(
          QStringLiteral("%1: cancelled by operator at hood preparation.")
              .arg(captureSequenceLogPrefix()));
      return;
    }

    beginCaptureModeMotion();
    return;
  }

  restoreReflectanceExposuresAfterCapture();
  completeCaptureSequence();
}

void hf::capture::CapturePanelController::completeCaptureSequence() {
  if (captureScanTimer_ != nullptr)
    captureScanTimer_->stop();
  captureRelativeScanTimerActiveToken_ = 0;

  restoreReflectanceExposuresAfterCapture();

  const bool wasRecord = captureRecorderMode_ == CaptureRecorderMode::Record;
  const bool wasPreview = captureRecorderMode_ == CaptureRecorderMode::Preview;
  QStringList completedPreviewModeFolders;
  for (const CaptureIlluminationMode mode : capturePendingIlluminationModes_)
    completedPreviewModeFolders.push_back(captureIlluminationFolderName(mode));

  resetCaptureSequenceState();
  capturePendingIlluminationModes_.clear();
  captureCurrentModeIndex_ = 0;

  if (wasRecord && captureMultiviewPending_ && canRunMultiviewRgbCapture()) {
    // Close HSI dump now so preprocessing can run while stage moves / Multiview proceeds.
    if (captureMultiviewSessionDirectory_.isEmpty() && captureWriterWorker_ != nullptr)
      captureMultiviewSessionDirectory_ = captureWriterWorker_->sessionDirectory();
    endCaptureRawDumpSession();
    runCapturePostProcessingIfEnabled();

    host_->appendLog(QStringLiteral(
        "Capture record: HSI finished — moving stage to Multiview apex pose, then confirm…"));
    captureMultiviewAwaitingOperatorConfirm_ = true;
    beginMultiviewCapturePhase();
    return;
  }

  if (isStageRecordingEnabledInUi() && host_->stageWorker() != nullptr)
    host_->stageWorker()->requestStopMotion();

  captureRecorderMode_ = CaptureRecorderMode::Idle;
  captureMultiviewPending_ = false;
  captureMultiviewOnlySession_ = false;
  updateRecorderControls();
  host_->lightPanel()->updateConnectionDisplay();
  host_->lightPanel()->updateControlsEnabled();

  if (wasRecord) {
    if (!captureMultiviewOnlySession_)
      endCaptureRawDumpSession();
    runCapturePostProcessingIfEnabled();
    host_->appendLog(QStringLiteral("Capture record: scan sequence finished."));
    homeStageAfterCapture();
    beginRecordCompleteNotify();
  } else if (wasPreview) {
    host_->appendLog(
        QStringLiteral("Capture preview: scan sequence finished."));
    lastPreviewCompleteModeFolders_ = std::move(completedPreviewModeFolders);
    homeStageAfterCapture();
    beginPreviewCompleteNotify();
  }
}

void hf::capture::CapturePanelController::finishCaptureSequenceAfterOptionalMultiview() {
  const bool wasRecord = captureRecorderMode_ == CaptureRecorderMode::Record;
  const bool multiviewOnly = captureMultiviewOnlySession_;
  const QString multiviewSessionDir = captureMultiviewSessionDirectory_;
  captureMultiviewInProgress_ = false;
  captureMultiviewPending_ = false;
  captureMultiviewOnlySession_ = false;
  captureMultiviewSessionDirectory_.clear();

  if (isStageRecordingEnabledInUi() && host_->stageWorker() != nullptr)
    host_->stageWorker()->requestStopMotion();

  captureRecorderMode_ = CaptureRecorderMode::Idle;
  resetCaptureSequenceState();
  updateRecorderControls();
  host_->lightPanel()->updateConnectionDisplay();
  host_->lightPanel()->updateControlsEnabled();

  if (wasRecord) {
    if (!multiviewOnly)
      endCaptureRawDumpSession(); // no-op if already closed after HSI
    else if (!multiviewSessionDir.isEmpty())
      lastEndedCaptureSessionSummary_.sessionDirectory = multiviewSessionDir;
    // HSI+Multiview already started post-process at HSI end; Multiview-only starts it here.
    if (!capturePostProcessStartedForSession_)
      runCapturePostProcessingIfEnabled();
    host_->appendLog(QStringLiteral("Capture record: scan sequence finished."));
    homeStageAfterCapture();
    beginRecordCompleteNotify();
  }
}

hf::processing::CapturePostProcessOptions
hf::capture::CapturePanelController::capturePostProcessOptionsFromUi() const
{
  hf::processing::CapturePostProcessOptions options;
  options.saveFfcImage = host_->captureSaveFfcImageCheck_ != nullptr &&
                         host_->captureSaveFfcImageCheck_->isChecked();
  options.runGsamSegmentation = host_->captureRunGsamCheck_ != nullptr &&
                                host_->captureRunGsamCheck_->isChecked();
  options.runHfFusion = host_->captureRunHfFusionCheck_ != nullptr &&
                        host_->captureRunHfFusionCheck_->isChecked();
  if (host_->captureGsamPromptEdit_ != nullptr)
    options.gsamPrompt = host_->captureGsamPromptEdit_->text().trimmed();
  if (host_->captureGsamSampleCountSpin_ != nullptr)
    options.gsamSampleCount = host_->captureGsamSampleCountSpin_->value();
  if (host_->captureGsamPlanCombo_ != nullptr)
    options.gsamPlanPath = host_->captureGsamPlanCombo_->currentData().toString();
  if (gsam2ServerManager_ != nullptr)
    options.gsamServerUrl = gsam2ServerManager_->serverUrl();
  if (!options.gsamPlanPath.isEmpty())
    options.runGsamSegmentation = true;
  if (options.runGsamSegmentation)
    options.saveFfcImage = true;
  return options;
}

void hf::capture::CapturePanelController::runCapturePostProcessingIfEnabled() {
  if (capturePostProcessStartedForSession_)
    return;

  if (host_->capturePreprocessAfterScanCheck_ == nullptr ||
      !host_->capturePreprocessAfterScanCheck_->isChecked())
    return;

  if (!useStageForCapture())
    return;

  if (lastEndedCaptureSessionSummary_.sessionDirectory.isEmpty())
    return;

  if (capturePostProcessorWorker_ == nullptr)
    return;

  const hf::processing::CapturePostProcessOptions options = capturePostProcessOptionsFromUi();

  QString planNote;
  if (!options.gsamPlanPath.isEmpty())
  {
    hf::processing::GsamPlan plan;
    if (hf::processing::loadGsamPlan(options.gsamPlanPath, &plan, nullptr))
      planNote = hf::processing::gsamPlanComboLabel(
          plan, QFileInfo(options.gsamPlanPath).completeBaseName());
    else
      planNote = QFileInfo(options.gsamPlanPath).completeBaseName();
  }

  if (options.runGsamSegmentation && gsam2ServerManager_ != nullptr) {
    const auto state = gsam2ServerManager_->state();
    if (state != hf::processing::Gsam2ServerManager::State::Running) {
      host_->appendLog(
          QStringLiteral("Capture post-process: GSAM segmentation enabled but "
                         "server is not running \u2014 "
                         "start the GSAM server or disable segmentation."));
    }
  }

  if (options.runHfFusion && !options.runGsamSegmentation) {
    host_->appendLog(
        QStringLiteral("Capture post-process: spectral fusion enabled but GSAM "
                       "segmentation is off \u2014 fusion needs chip masks on both cameras."));
  }

  host_->appendLog(
      planNote.isEmpty()
          ? QStringLiteral("Capture post-process: started in background\u2026")
          : QStringLiteral("Capture post-process: started in background (%1)\u2026")
                .arg(planNote));

  capturePostProcessStartedForSession_ = true;
  capturePostProcessInFlight_ = true;

  capturePostProcessorWorker_->requestProcess(
      lastEndedCaptureSessionSummary_, options,
      [this](const hf::processing::CapturePostProcessResult &result) {
        QMetaObject::invokeMethod(
            this,
            [this, result]() {
              for (const QString &line : result.logLines)
                host_->appendLog(line);
              capturePostProcessInFlight_ = false;
              captureRecordCompletePostProcessPending_ = false;
              updateRecorderControls();
              tryNotifyRecordComplete();
            },
            Qt::QueuedConnection);
      });
}

void hf::capture::CapturePanelController::startCaptureSequence() {
  applyDualCameraScanSync();

  QString errorMessage;
  if (!buildCaptureScanPlan(captureScanPlan_, errorMessage)) {
    host_->appendLog(
        QStringLiteral("%1: %2").arg(captureSequenceLogPrefix(), errorMessage));
    if (captureRecorderMode_ == CaptureRecorderMode::Record)
      stopRecorder();
    else if (captureRecorderMode_ == CaptureRecorderMode::Preview)
      stopRecorder();
    return;
  }

  initializeCaptureModeQueue();

  if (host_->lighthouseWorker() != nullptr &&
      host_->isLighthouseSessionActive())
    host_->lighthouseWorker()->requestPollControllerPowerStatus();

  const LighthouseControllerPowerStatus powerStatus =
      host_->lighthouseWorker() != nullptr
          ? host_->lighthouseWorker()->currentControllerPowerStatus()
          : LighthouseControllerPowerStatus{};

  if (!confirmCaptureStart(powerStatus)) {
    host_->appendLog(QStringLiteral("%1: cancelled by operator.")
                         .arg(captureSequenceLogPrefix()));
    stopRecorder();
    return;
  }

  startCurrentCaptureMode();
}

void hf::capture::CapturePanelController::startPreview() {
  if (captureRecorderMode_ != CaptureRecorderMode::Idle)
    return;

  if (isMultiviewRgbCaptureSelected() && canRunMultiviewRgbCapture()) {
    if (host_->ur3ePanel() == nullptr)
      return;
    host_->appendLog(QStringLiteral(
        "Capture preview: Multiview RGB enabled — running UR3e scan execute (motion only)…"));
    if (!host_->ur3ePanel()->startHemisphereScanExecute({}))
      host_->appendLog(QStringLiteral("Capture preview: UR3e scan execute rejected."));
    return;
  }

  applyDualCameraScanSync();

  QString errorMessage;
  CaptureScanPlan plan;
  if (!buildCaptureScanPlan(plan, errorMessage)) {
    host_->appendLog(QStringLiteral("Capture preview: %1").arg(errorMessage));
    return;
  }

  captureRecorderMode_ = CaptureRecorderMode::Preview;
  lastPreviewCompleteSampleFrames_ = {0, 0};
  lastPreviewCompleteCameraIndices_.clear();
  resetCaptureSequenceState();
  updateRecorderControls();

  host_->appendLog(QStringLiteral("Capture preview: per-camera reference "
                                  "positions from hyperfusion.cfg, "
                                  "sample origin %1 mm, total scan %2 mm, "
                                  "target length %3 mm @ %4 mm/s "
                                  "(no save)\u2026")
                       .arg(plan.sampleScanOriginMm, 0, 'f', 2)
                       .arg(plan.sampleScanTotalDistanceMm, 0, 'f', 2)
                       .arg(plan.sampleScanLengthMm, 0, 'f', 2)
                       .arg(plan.recordScanSpeedMmPerSec, 0, 'f', 1));

  homeStageBeforeCapture();
}

bool hf::capture::CapturePanelController::validateCaptureRecordMetadata(
    QString &errorMessage) const {
  const QString saveFolder =
      host_->captureSaveFolderEdit_ != nullptr
          ? host_->captureSaveFolderEdit_->text().trimmed()
          : QString();
  const QString dataset = host_->captureDatasetEdit_ != nullptr
                              ? host_->captureDatasetEdit_->text().trimmed()
                              : QString();

  if (dataset.isEmpty()) {
    errorMessage = QStringLiteral("Enter a dataset name in Metadata.");
    return false;
  }

  if (saveFolder.isEmpty()) {
    errorMessage = QStringLiteral("Choose a save folder in Metadata.");
    return false;
  }

  const QFileInfo folderInfo(saveFolder);
  if (!folderInfo.exists() || !folderInfo.isDir()) {
    errorMessage =
        QStringLiteral("Save folder does not exist: %1").arg(saveFolder);
    return false;
  }

  const QString sessionDirectory = QDir(saveFolder).filePath(dataset);
  if (QDir(sessionDirectory).exists()) {
    errorMessage =
        QStringLiteral("Dataset folder already exists — choose a different "
                       "dataset name:\n%1")
            .arg(sessionDirectory);
    return false;
  }

  return true;
}

void hf::capture::CapturePanelController::notifyRecordBlocked(
    const QString &message) const {
  host_->appendLog(QStringLiteral("Capture record: %1").arg(message));
  QMessageBox::warning(host_, tr("Cannot start recording"), message);
}

bool hf::capture::CapturePanelController::selectedCaptureCameraIndices(
    std::vector<std::size_t> &cameraIndices) const {
  cameraIndices.clear();

  const auto isConnected = [](const LumoCameraUi &ui) {
    return ui.state != CameraState::Disconnected &&
           ui.state != CameraState::Fault;
  };

  const auto consider = [&cameraIndices, &isConnected](
                            const QCheckBox *checkbox, const LumoCameraUi &ui,
                            const std::size_t index) {
    if (checkbox != nullptr && isConnected(ui) && checkbox->isChecked())
      cameraIndices.push_back(index);
  };

  consider(host_->captureCamera1Check_, host_->camera1Ui_, 0);
  consider(host_->captureCamera2Check_, host_->camera2Ui_, 1);
  return !cameraIndices.empty();
}

std::size_t hf::capture::CapturePanelController::stageCameraIndexForUi(
    const LumoCameraUi &ui, const std::size_t cameraIndex) const {
  if (ui.sensorKind == LumoSensorKind::Fx10ePleora)
    return 0;
  if (ui.sensorKind == LumoSensorKind::Swir3Ni)
    return 1;
  return cameraIndex;
}

double hf::capture::CapturePanelController::whiteRefStartMmForStageCamera(
    const CaptureScanPlan &plan, const CaptureIlluminationMode mode,
    const std::size_t stageCameraIndex) const {
  if (stageCameraIndex >= 2)
    return 0.0;

  if (mode == CaptureIlluminationMode::Transmittance)
    return plan.brightRefStartMm[stageCameraIndex];

  return plan.whiteRefStartMm[stageCameraIndex];
}

double
hf::capture::CapturePanelController::estimatedStageScanPositionMm() const {
  if (!captureScanTimingActive_)
    return 0.0;

  const double elapsedSec =
      static_cast<double>(captureScanElapsed_.elapsed()) / 1000.0;
  const double traveledMm =
      std::min(elapsedSec * captureScanPlan_.recordScanSpeedMmPerSec,
               captureActiveScanDistanceMm_);
  return captureScanOriginPositionMm_ + traveledMm;
}

double hf::capture::CapturePanelController::currentStageScanPositionMm() const {
  if (captureScanTimingActive_ && captureStagePositionKnown_)
    return captureLastKnownStagePositionMm_;

  return estimatedStageScanPositionMm();
}

std::optional<double>
hf::capture::CapturePanelController::knownStageScanPositionMm() const {
  if (!captureScanTimingActive_ || !captureStagePositionKnown_)
    return std::nullopt;

  return captureLastKnownStagePositionMm_;
}

double
hf::capture::CapturePanelController::maxPlausibleStageScanPositionMm() const {
  constexpr double kSlackMm = 3.0;

  if (!captureScanTimingActive_)
    return captureScanOriginPositionMm_;

  const double elapsedSec =
      static_cast<double>(captureScanElapsed_.elapsed()) / 1000.0;
  const double traveledMm =
      std::max(0.0, elapsedSec * captureScanPlan_.recordScanSpeedMmPerSec);
  return captureScanOriginPositionMm_ + traveledMm + kSlackMm;
}

bool hf::capture::CapturePanelController::isStageScanPositionTrustworthy(
    const double positionMm) const {
  if (!captureScanTimingActive_ || !captureStagePositionKnown_)
    return false;

  return positionMm <= maxPlausibleStageScanPositionMm() &&
         hasStageScanElapsedForPosition(positionMm);
}

bool hf::capture::CapturePanelController::hasStageScanElapsedForPosition(
    const double positionMm) const {
  if (!captureScanTimingActive_)
    return false;

  const double speedMmPerSec = captureScanPlan_.recordScanSpeedMmPerSec;
  if (speedMmPerSec <= 0.0)
    return false;

  constexpr double kSlackMm = 2.0;
  const double travelNeededMm =
      std::max(0.0, positionMm - captureScanOriginPositionMm_ - kSlackMm);
  const double minElapsedSec = travelNeededMm / speedMmPerSec;
  const double elapsedSec =
      static_cast<double>(captureScanElapsed_.elapsed()) / 1000.0;
  return elapsedSec >= minElapsedSec;
}

bool hf::capture::CapturePanelController::isStagePositionWithinScanWindow(
    const double positionMm, const double windowStartMm,
    const double windowLengthMm) const {
  constexpr double kMarginMm = 0.25;
  return positionMm + kMarginMm >= windowStartMm &&
         positionMm - kMarginMm <= windowStartMm + windowLengthMm;
}

bool hf::capture::CapturePanelController::isStagePositionWithinSampleWindow(
    const double positionMm, const double windowStartMm,
    const double windowLengthMm) const {
  constexpr double kPositionToleranceMm = 0.05;
  return positionMm + kPositionToleranceMm >= windowStartMm &&
         positionMm - kPositionToleranceMm <= windowStartMm + windowLengthMm;
}

bool hf::capture::CapturePanelController::allSelectedCamerasPastSampleWindow(
    const double stagePositionMm) const {
  std::vector<std::size_t> selectedCameras;
  if (!selectedCaptureCameraIndices(selectedCameras))
    return true;

  for (const std::size_t cameraIndex : selectedCameras) {
    const LumoCameraUi &ui =
        cameraIndex == 0 ? host_->camera1Ui_ : host_->camera2Ui_;
    const std::size_t stageCameraIndex = stageCameraIndexForUi(ui, cameraIndex);
    const double windowEnd =
        captureScanPlan_.sampleScanStartMm[stageCameraIndex] +
        captureScanPlan_.sampleScanLengthMm;
    constexpr double kPositionToleranceMm = 0.05;
    if (stagePositionMm + kPositionToleranceMm < windowEnd)
      return false;
  }

  return true;
}

bool hf::capture::CapturePanelController::selectedCamerasEnteredSampleWindow()
    const {
  std::vector<std::size_t> selectedCameras;
  if (!selectedCaptureCameraIndices(selectedCameras))
    return false;

  for (const std::size_t cameraIndex : selectedCameras) {
    const LumoCameraUi &ui =
        cameraIndex == 0 ? host_->camera1Ui_ : host_->camera2Ui_;
    const std::size_t stageCameraIndex = stageCameraIndexForUi(ui, cameraIndex);
    if (stageCameraIndex >= captureSampleWindowEntered_.size() ||
        !captureSampleWindowEntered_[stageCameraIndex]) {
      return false;
    }
  }

  return true;
}

bool hf::capture::CapturePanelController::selectedCamerasHaveSampleFrames()
    const {
  std::vector<std::size_t> selectedCameras;
  if (!selectedCaptureCameraIndices(selectedCameras))
    return false;

  for (const std::size_t cameraIndex : selectedCameras) {
    if (captureSampleFramesCollected_[cameraIndex] <= 0)
      return false;
  }

  return true;
}

bool hf::capture::CapturePanelController::canCompleteSampleScan() const {
  if (captureScanPhase_ != CaptureScanPhase::SampleScan ||
      !captureSampleRecordingActive_)
    return false;

  if (!selectedCamerasEnteredSampleWindow())
    return false;

  if (captureRecorderMode_ == CaptureRecorderMode::Record &&
      !selectedCamerasHaveSampleFrames())
    return false;

  if (captureScanPlan_.recordScanSpeedMmPerSec <= 0.0)
    return false;

  const double elapsedSec =
      static_cast<double>(captureScanElapsed_.elapsed()) / 1000.0;
  const double expectedSec = captureScanPlan_.sampleScanTotalDistanceMm /
                             captureScanPlan_.recordScanSpeedMmPerSec;
  constexpr double kMinimumElapsedFraction = 0.85;
  if (elapsedSec + 0.05 < expectedSec * kMinimumElapsedFraction)
    return false;

  const std::optional<double> stagePositionMm = knownStageScanPositionMm();
  if (!stagePositionMm)
    return false;

  return allSelectedCamerasPastSampleWindow(*stagePositionMm);
}

void hf::capture::CapturePanelController::updateSampleScanWindowProgress(
    const double stagePositionMm) {
  if ((captureScanPhase_ != CaptureScanPhase::SampleScan &&
       captureScanPhase_ != CaptureScanPhase::CombinedRecordScan) ||
      !captureSampleRecordingActive_) {
    return;
  }

  if (!isStageScanPositionTrustworthy(stagePositionMm))
    return;

  std::vector<std::size_t> selectedCameras;
  if (!selectedCaptureCameraIndices(selectedCameras))
    return;

  for (const std::size_t cameraIndex : selectedCameras) {
    const LumoCameraUi &ui =
        cameraIndex == 0 ? host_->camera1Ui_ : host_->camera2Ui_;
    const std::size_t stageCameraIndex = stageCameraIndexForUi(ui, cameraIndex);
    if (stageCameraIndex >= captureSampleWindowEntered_.size())
      continue;

    const double windowStart =
        captureScanPlan_.sampleScanStartMm[stageCameraIndex];
    const double windowLength = captureScanPlan_.sampleScanLengthMm;
    if (isStagePositionWithinSampleWindow(stagePositionMm, windowStart,
                                          windowLength))
      captureSampleWindowEntered_[stageCameraIndex] = true;

    const double windowEnd = windowStart + windowLength;
    if (stagePositionMm + 0.05 >= windowEnd)
      captureSampleWindowComplete_[stageCameraIndex] = true;
  }
}

bool hf::capture::CapturePanelController::shouldRecordSampleFrameForCamera(
    const std::size_t stageCameraIndex, const double stagePositionMm) const {
  if (!captureSampleRecordingActive_)
    return false;

  if (!isStageScanPositionTrustworthy(stagePositionMm))
    return false;

  if (stageCameraIndex >= captureSampleWindowComplete_.size() ||
      captureSampleWindowComplete_[stageCameraIndex]) {
    return false;
  }

  const double windowStart =
      captureScanPlan_.sampleScanStartMm[stageCameraIndex];
  const double windowEnd = windowStart + captureScanPlan_.sampleScanLengthMm;

  if (stagePositionMm - 0.05 > windowEnd)
    return false;

  if (!isStagePositionWithinSampleWindow(stagePositionMm, windowStart,
                                         captureScanPlan_.sampleScanLengthMm))
    return false;

  if (captureScanPhase_ == CaptureScanPhase::CombinedRecordScan) {
    for (std::size_t cameraIndex = 0; cameraIndex < 2; ++cameraIndex) {
      const LumoCameraUi &ui =
          cameraIndex == 0 ? host_->camera1Ui_ : host_->camera2Ui_;
      if (stageCameraIndexForUi(ui, cameraIndex) != stageCameraIndex)
        continue;

      if (captureWhiteRefFramesCollected_[cameraIndex] <
          captureScanPlan_.whiteReferenceFrameCount) {
        return false;
      }
      break;
    }
  }

  return true;
}

bool hf::capture::CapturePanelController::canCompleteCombinedRecordScan()
    const {
  if (captureScanPhase_ != CaptureScanPhase::CombinedRecordScan ||
      !captureSampleRecordingActive_)
    return false;

  if (!selectedCamerasReachedWhiteReferenceTarget())
    return false;

  if (!selectedCamerasEnteredSampleWindow())
    return false;

  if (captureRecorderMode_ == CaptureRecorderMode::Record &&
      !selectedCamerasHaveSampleFrames())
    return false;

  if (captureScanPlan_.recordScanSpeedMmPerSec <= 0.0)
    return false;

  const double elapsedSec =
      static_cast<double>(captureScanElapsed_.elapsed()) / 1000.0;
  const double expectedSec = captureScanPlan_.recordScanTotalDistanceMm /
                             captureScanPlan_.recordScanSpeedMmPerSec;
  constexpr double kMinimumElapsedFraction = 0.85;
  if (elapsedSec + 0.05 < expectedSec * kMinimumElapsedFraction)
    return false;

  const std::optional<double> stagePositionMm = knownStageScanPositionMm();
  if (!stagePositionMm)
    return false;

  return allSelectedCamerasPastSampleWindow(*stagePositionMm);
}

QString hf::capture::CapturePanelController::recorderCameraStatusText(
    const std::size_t cameraIndex) const {
  if (cameraIndex >= 2)
    return QString();

  const LumoCameraUi &ui =
      cameraIndex == 0 ? host_->camera1Ui_ : host_->camera2Ui_;
  const QString cameraName = host_->cameraPanel()->profileTabNameForUi(ui);

  if (captureRecorderMode_ == CaptureRecorderMode::Idle)
    return QStringLiteral("%1: %2").arg(cameraName, tr("Ready"));

  if (!useStageForCapture()) {
    return QStringLiteral("%1: %2 (%3 frames)")
        .arg(cameraName)
        .arg(tr("sample"))
        .arg(captureSampleFramesCollected_[cameraIndex]);
  }

  switch (captureScanPhase_) {
  case CaptureScanPhase::BlackReference:
    if (!captureBlackReferenceCollectionActive_) {
      return QStringLiteral("%1: %2").arg(cameraName, tr("closing shutter\u2026"));
    }
    return QStringLiteral("%1: %2 (%3/%4)")
        .arg(cameraName)
        .arg(tr("black reference"))
        .arg(captureBlackRefFramesCollected_[cameraIndex])
        .arg(captureScanPlan_.blackReferenceFrameCount);
  case CaptureScanPhase::CombinedRecordScan: {
    const QString refLabel = captureRecordingIlluminationMode_ ==
                                     CaptureIlluminationMode::Reflectance
                                 ? tr("white ref")
                                 : tr("bright ref");
    return QStringLiteral("%1: %2 %3/%4, %5 %6")
        .arg(cameraName)
        .arg(refLabel)
        .arg(captureWhiteRefFramesCollected_[cameraIndex])
        .arg(captureScanPlan_.whiteReferenceFrameCount)
        .arg(tr("sample"))
        .arg(captureSampleFramesCollected_[cameraIndex]);
  }
  case CaptureScanPhase::WhiteReferenceScan: {
    const QString refLabel = captureRecordingIlluminationMode_ ==
                                     CaptureIlluminationMode::Reflectance
                                 ? tr("white reference")
                                 : tr("bright reference");
    return QStringLiteral("%1: %2 (%3/%4)")
        .arg(cameraName)
        .arg(refLabel)
        .arg(captureWhiteRefFramesCollected_[cameraIndex])
        .arg(captureScanPlan_.whiteReferenceFrameCount);
  }
  case CaptureScanPhase::SampleScan:
    return QStringLiteral("%1: %2 (%3 frames)")
        .arg(cameraName)
        .arg(tr("sample scan"))
        .arg(captureSampleFramesCollected_[cameraIndex]);
  default:
    break;
  }

  return QStringLiteral("%1: %2").arg(cameraName, tr("waiting\u2026"));
}

bool hf::capture::CapturePanelController::selectedCaptureCameraStreaming(
    QString &errorMessage) const {
  std::vector<std::size_t> cameraIndices;
  if (!selectedCaptureCameraIndices(cameraIndices)) {
    errorMessage = QStringLiteral(
        "Select at least one connected camera in the Cameras list.");
    return false;
  }

  const LumoCameraUi *uis[] = {&host_->camera1Ui_, &host_->camera2Ui_};
  for (const std::size_t index : cameraIndices) {
    const LumoCameraUi &ui = *uis[index];
    if (ui.state != CameraState::Streaming && ui.state != CameraState::Armed &&
        ui.state != CameraState::Configured &&
        ui.state != CameraState::Initialized) {
      errorMessage =
          QStringLiteral(
              "%1 is not streaming \u2014 connect and wait for preview first.")
              .arg(host_->cameraPanel()->profileTabNameForUi(ui));
      return false;
    }
  }

  return true;
}

bool hf::capture::CapturePanelController::selectedCaptureIlluminationModes(
    std::vector<CaptureIlluminationMode> &modes) const {
  modes.clear();

  if (host_->captureReflectanceCheck_ != nullptr &&
      host_->captureReflectanceCheck_->isChecked())
    modes.push_back(CaptureIlluminationMode::Reflectance);
  if (host_->captureTransmittanceCheck_ != nullptr &&
      host_->captureTransmittanceCheck_->isChecked())
    modes.push_back(CaptureIlluminationMode::Transmittance);

  return !modes.empty();
}

bool hf::capture::CapturePanelController::isCaptureStageConnected() const {
  return host_->stageWorker() != nullptr &&
         host_->stageWorker()->currentState() == StageState::Connected;
}

bool hf::capture::CapturePanelController::isCaptureStagePresent() const {
  if (host_->stageWorker() == nullptr)
    return false;

  const StageState state = host_->stageWorker()->currentState();
  return state != StageState::Disconnected && state != StageState::Fault;
}

bool hf::capture::CapturePanelController::isStageRecordingEnabledInUi() const {
  return host_->captureUseStageForRecordingCheck_ != nullptr &&
         host_->captureUseStageForRecordingCheck_->isChecked();
}

bool hf::capture::CapturePanelController::useStageForCapture() const {
  if (host_->captureUseStageForRecordingCheck_ == nullptr ||
      !host_->captureUseStageForRecordingCheck_->isChecked())
    return false;

  // Keep staged routing during homing/moves \u2014 stage state is Homing, not
  // Connected.
  if (captureStageSequenceActive_)
    return true;

  return isCaptureStageConnected();
}

bool hf::capture::CapturePanelController::effectiveCaptureIlluminationModes(
    std::vector<CaptureIlluminationMode> &modes) const {
  if (!useStageForCapture()) {
    modes = {CaptureIlluminationMode::Reflectance};
    return true;
  }

  return selectedCaptureIlluminationModes(modes);
}

QString hf::capture::CapturePanelController::captureIlluminationFolderName(
    const CaptureIlluminationMode mode) const {
  return mode == CaptureIlluminationMode::Reflectance
             ? QStringLiteral("reflectance")
             : QStringLiteral("transmittance");
}

QString hf::capture::CapturePanelController::captureCameraFolderName(
    const LumoCameraUi &ui) const {
  if (ui.sensorKind == LumoSensorKind::Fx10ePleora)
    return QStringLiteral("fx10e");
  if (ui.sensorKind == LumoSensorKind::Swir3Ni)
    return QStringLiteral("swir3");

  QString slug =
      host_->cameraPanel()->profileTabNameForUi(ui).trimmed().toLower();
  slug.replace(QLatin1Char(' '), QLatin1Char('_'));
  return slug;
}

QString hf::capture::CapturePanelController::captureStreamRelativeRoot(
    const CaptureIlluminationMode mode, const LumoCameraUi &ui) const {
  const QString modeFolder = useStageForCapture()
                                 ? captureIlluminationFolderName(mode)
                                 : QStringLiteral("recording");
  return modeFolder + QLatin1Char('/') + captureCameraFolderName(ui);
}

bool hf::capture::CapturePanelController::
    resolveCaptureRecordingIlluminationMode(CaptureIlluminationMode &mode,
                                            QString &errorMessage) const {
  std::vector<CaptureIlluminationMode> modes;
  if (!effectiveCaptureIlluminationModes(modes)) {
    errorMessage =
        QStringLiteral("Select reflectance and/or transmittance mode.");
    return false;
  }

  mode = modes.front();
  return true;
}

bool hf::capture::CapturePanelController::buildCaptureWriterSessionConfig(
    CaptureWriterSessionConfig &config, QString &errorMessage) const {
  QString metadataError;
  if (!validateCaptureRecordMetadata(metadataError)) {
    errorMessage = metadataError;
    return false;
  }

  QString streamingError;
  if (!selectedCaptureCameraStreaming(streamingError)) {
    errorMessage = streamingError;
    return false;
  }

  config.saveFolder = host_->captureSaveFolderEdit_->text().trimmed();
  config.datasetName = host_->captureDatasetEdit_->text().trimmed();
  config.operatorName = host_->captureOperatorEdit_ != nullptr
                            ? host_->captureOperatorEdit_->text().trimmed()
                            : QString();
  config.description =
      host_->captureDescriptionEdit_ != nullptr
          ? host_->captureDescriptionEdit_->toPlainText().trimmed()
          : QString();
  config.streams.clear();

  std::vector<CaptureIlluminationMode> illuminationModes;
  if (!effectiveCaptureIlluminationModes(illuminationModes)) {
    errorMessage =
        QStringLiteral("Select reflectance and/or transmittance mode.");
    return false;
  }

  std::vector<std::size_t> cameraIndices;
  if (!selectedCaptureCameraIndices(cameraIndices)) {
    errorMessage = QStringLiteral(
        "Select at least one connected camera in the Cameras list.");
    return false;
  }

  const LumoCameraUi *uis[] = {&host_->camera1Ui_, &host_->camera2Ui_};
  for (const CaptureIlluminationMode illuminationMode : illuminationModes) {
    for (const std::size_t cameraIndex : cameraIndices) {
      const LumoCameraUi &ui = *uis[cameraIndex];
      CaptureWriterStreamConfig stream;
      stream.source = cameraIndex == 0 ? CameraBackendId::Camera1
                                       : CameraBackendId::Camera2;
      stream.illuminationMode = illuminationMode;
      stream.relativeRoot = captureStreamRelativeRoot(illuminationMode, ui);
      stream.streamName = host_->cameraPanel()->profileTabNameForUi(ui);
      stream.settings = host_->cameraPanel()->buildSettings(ui);
      stream.spectralBands = ui.spectralBands;
      stream.calibrationPackPath =
          hf::camera::CameraPanelController::calibrationPackPath(ui);
      config.streams.push_back(std::move(stream));
    }
  }

  return true;
}

bool hf::capture::CapturePanelController::beginCaptureRawDumpSession(
    QString &errorMessage) {
  if (captureWriterWorker_ == nullptr) {
    errorMessage = QStringLiteral("Capture writer is not available.");
    return false;
  }

  CaptureWriterSessionConfig config;
  if (!buildCaptureWriterSessionConfig(config, errorMessage))
    return false;

  if (!captureWriterWorker_->beginSessionSync(config, &errorMessage))
    return false;

  return true;
}

void hf::capture::CapturePanelController::endCaptureRawDumpSession() {
  if (captureWriterWorker_ == nullptr || !captureWriterWorker_->isActive())
    return;

  const CaptureWriterSessionSummary summary =
      captureWriterWorker_->endSessionSync();
  lastEndedCaptureSessionSummary_ = summary;

  if (summary.sessionDirectory.isEmpty())
    return;

  host_->appendLog(QStringLiteral("Capture record: saved to %1")
                       .arg(summary.sessionDirectory));
  for (auto it = summary.streams.cbegin(); it != summary.streams.cend(); ++it) {
    host_->appendLog(
        QStringLiteral("  %1 - %2: %3 frames (%4 bytes) -> %5")
            .arg(it->relativeRoot.isEmpty() ? it.key() : it->relativeRoot)
            .arg(it->baseName)
            .arg(it->frameCount)
            .arg(it->bytesWritten)
            .arg(QFileInfo(it->rawPath).fileName()));
    if (!it->blackReferenceRawPath.isEmpty() &&
        it->blackReferenceFrameCount > 0) {
      host_->appendLog(
          QStringLiteral("    DARKREF - %1 frames -> %2")
              .arg(it->blackReferenceFrameCount)
              .arg(QFileInfo(it->blackReferenceRawPath).fileName()));
    }
    if (!it->whiteReferenceRawPath.isEmpty() &&
        it->whiteReferenceFrameCount > 0) {
      host_->appendLog(
          QStringLiteral("    WHITEREF - %1 frames -> %2")
              .arg(it->whiteReferenceFrameCount)
              .arg(QFileInfo(it->whiteReferenceRawPath).fileName()));
    }
  }
}

void hf::capture::CapturePanelController::appendCaptureRecordFrame(
    const FramePacket &frame) {
  if (captureRecorderMode_ != CaptureRecorderMode::Record ||
      captureWriterWorker_ == nullptr || !captureWriterWorker_->isActive())
    return;

  std::vector<std::size_t> selected;
  if (!selectedCaptureCameraIndices(selected))
    return;

  const std::size_t cameraIndex =
      frame.source == CameraBackendId::Camera1 ? 0 : 1;
  if (std::find(selected.begin(), selected.end(), cameraIndex) ==
      selected.end())
    return;

  const LumoCameraUi &cameraUi =
      cameraIndex == 0 ? host_->camera1Ui_ : host_->camera2Ui_;
  FramePacket routed = frame;
  routed.captureStreamKey =
      captureStreamRelativeRoot(captureRecordingIlluminationMode_, cameraUi);

  // Reflectance-only continuous dump when stage scanning is disabled.
  if (!useStageForCapture()) {
    routed.captureDestination = CaptureFrameDestination::Sample;
    captureWriterWorker_->submitFrame(std::move(routed));
    ++captureSampleFramesCollected_[cameraIndex];
    return;
  }

  if (captureScanPhase_ == CaptureScanPhase::BlackReference) {
    if (!shouldAcceptBlackReferenceFrame(cameraIndex))
      return;

    routed.captureDestination = CaptureFrameDestination::BlackReference;
    captureWriterWorker_->submitFrame(std::move(routed));
    ++captureBlackRefFramesCollected_[cameraIndex];

    if (selectedCamerasReachedBlackReferenceTarget())
      onCaptureBlackReferenceComplete();
    else
      updateRecorderStatus();
    return;
  }

  if (captureScanPhase_ == CaptureScanPhase::WhiteReferenceScan) {
    if (!shouldAcceptWhiteReferenceFrame(cameraIndex))
      return;

    if (captureRecorderMode_ == CaptureRecorderMode::Record) {
      routed.captureDestination = CaptureFrameDestination::WhiteReference;
      captureWriterWorker_->submitFrame(std::move(routed));
    }

    const std::size_t stageCameraIndex =
        stageCameraIndexForUi(cameraUi, cameraIndex);
    const double stagePositionMm = currentStageScanPositionMm();
    const double windowEnd =
        whiteRefStartMmForStageCamera(captureScanPlan_,
                                      captureRecordingIlluminationMode_,
                                      stageCameraIndex) +
        captureScanPlan_.whiteRefScanDistanceMm[stageCameraIndex];
    if (stagePositionMm + 0.05 >= windowEnd)
      captureWhiteRefWindowComplete_[stageCameraIndex] = true;

    onWhiteReferenceFrameCollected(cameraIndex);
    return;
  }

  if (captureScanPhase_ == CaptureScanPhase::CombinedRecordScan) {
    if (!captureSampleRecordingActive_)
      return;

    const std::size_t stageCameraIndex =
        stageCameraIndexForUi(cameraUi, cameraIndex);

    if (shouldRecordWhiteReferenceFrameForCamera(
            stageCameraIndex, cameraIndex, currentStageScanPositionMm())) {
      routed.captureDestination = CaptureFrameDestination::WhiteReference;
      captureWriterWorker_->submitFrame(std::move(routed));

      const double windowEnd =
          whiteRefStartMmForStageCamera(captureScanPlan_,
                                        captureRecordingIlluminationMode_,
                                        stageCameraIndex) +
          captureScanPlan_.whiteRefScanDistanceMm[stageCameraIndex];
      if (currentStageScanPositionMm() + 0.05 >= windowEnd)
        captureWhiteRefWindowComplete_[stageCameraIndex] = true;

      onWhiteReferenceFrameCollected(cameraIndex);
      return;
    }

    const std::optional<double> stagePositionMm = knownStageScanPositionMm();
    if (!stagePositionMm)
      return;

    updateSampleScanWindowProgress(*stagePositionMm);

    if (!shouldRecordSampleFrameForCamera(stageCameraIndex, *stagePositionMm))
      return;

    captureSampleWindowEntered_[stageCameraIndex] = true;

    routed.captureDestination = CaptureFrameDestination::Sample;
    captureWriterWorker_->submitFrame(std::move(routed));
    ++captureSampleFramesCollected_[cameraIndex];

    const double windowEnd =
        captureScanPlan_.sampleScanStartMm[stageCameraIndex] +
        captureScanPlan_.sampleScanLengthMm;
    if (*stagePositionMm + 0.05 >= windowEnd)
      captureSampleWindowComplete_[stageCameraIndex] = true;

    updateRecorderStatus();

    if (allSelectedCamerasPastSampleWindow(*stagePositionMm) &&
        canCompleteCombinedRecordScan())
      onCaptureCombinedRecordScanComplete();
    return;
  }

  if (captureScanPhase_ == CaptureScanPhase::SampleScan) {
    if (!captureSampleRecordingActive_)
      return;

    const std::size_t stageCameraIndex =
        stageCameraIndexForUi(cameraUi, cameraIndex);
    const std::optional<double> stagePositionMm = knownStageScanPositionMm();
    if (!stagePositionMm)
      return;

    updateSampleScanWindowProgress(*stagePositionMm);

    if (!shouldRecordSampleFrameForCamera(stageCameraIndex, *stagePositionMm)) {
      return;
    }

    captureSampleWindowEntered_[stageCameraIndex] = true;

    routed.captureDestination = CaptureFrameDestination::Sample;
    captureWriterWorker_->submitFrame(std::move(routed));
    ++captureSampleFramesCollected_[cameraIndex];

    const double windowEnd =
        captureScanPlan_.sampleScanStartMm[stageCameraIndex] +
        captureScanPlan_.sampleScanLengthMm;
    if (*stagePositionMm + 0.05 >= windowEnd)
      captureSampleWindowComplete_[stageCameraIndex] = true;

    if (allSelectedCamerasPastSampleWindow(*stagePositionMm) &&
        canCompleteSampleScan())
      onCaptureSampleScanComplete();
    return;
  }

  // Discard frames during homing, stage moves, and shutter transitions.
}

void hf::capture::CapturePanelController::startRecord() {
  if (captureRecorderMode_ != CaptureRecorderMode::Idle)
    return;

  QString errorMessage;

  if (capturePostProcessorWorker_ != nullptr &&
      capturePostProcessorWorker_->queueStatus().outstandingTotal() > 0) {
    notifyRecordBlocked(
        tr("Post-processing is still running. Wait for it to finish before "
           "starting a new recording."));
    return;
  }

  const bool wantMultiview = isMultiviewRgbCaptureSelected() && canRunMultiviewRgbCapture();
  const bool wantHsi = hasHsiCaptureSelection();

  if (!wantMultiview && !wantHsi) {
    notifyRecordBlocked(
        tr("Select reflectance/transmittance with a connected HSI camera, "
           "and/or enable Multiview RGB with a planned UR3e route."));
    return;
  }

  if (!validateCaptureRecordMetadata(errorMessage)) {
    notifyRecordBlocked(errorMessage);
    return;
  }

  captureMultiviewPending_ = wantMultiview;
  captureMultiviewOnlySession_ = wantMultiview && !wantHsi;
  captureMultiviewSessionDirectory_.clear();
  lastCaptureMultiviewSummaryText_.clear();
  captureMultiviewAwaitingOperatorConfirm_ = false;
  capturePostProcessStartedForSession_ = false;
  capturePostProcessInFlight_ = false;

  if (!wantHsi) {
    if (!beginMultiviewOnlyDatasetSession(errorMessage)) {
      captureMultiviewPending_ = false;
      captureMultiviewOnlySession_ = false;
      notifyRecordBlocked(errorMessage);
      return;
    }

    resetCaptureSequenceState();
    captureRecorderMode_ = CaptureRecorderMode::Record;
    updateRecorderControls();

    QMessageBox box(host_);
    box.setIcon(QMessageBox::Information);
    box.setWindowTitle(tr("Scanning procedure"));
    box.setText(tr("Ready to start Multiview-only recording?"));
    box.setInformativeText(buildScanningProcedureSummary());
    box.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
    box.setDefaultButton(QMessageBox::Ok);
    if (QAbstractButton *continueButton = box.button(QMessageBox::Ok))
      continueButton->setText(QStringLiteral("Continue"));
    if (box.exec() != QMessageBox::Ok) {
      captureMultiviewPending_ = false;
      captureMultiviewOnlySession_ = false;
      captureRecorderMode_ = CaptureRecorderMode::Idle;
      updateRecorderControls();
      host_->appendLog(QStringLiteral("Capture record: cancelled by operator."));
      return;
    }

    host_->appendLog(
        QStringLiteral("Capture record: Multiview-only session %1")
            .arg(captureMultiviewSessionDirectory_));
    beginMultiviewCapturePhase();
    return;
  }

  std::vector<std::size_t> selectedCameras;
  if (!selectedCaptureCameraIndices(selectedCameras)) {
    notifyRecordBlocked(
        tr("Select at least one connected camera in the Cameras list."));
    return;
  }

  if (!selectedCaptureCameraStreaming(errorMessage)) {
    notifyRecordBlocked(errorMessage);
    return;
  }

  std::vector<CaptureIlluminationMode> illuminationModes;
  if (!effectiveCaptureIlluminationModes(illuminationModes)) {
    notifyRecordBlocked(
        tr("Select reflectance and/or transmittance mode in Position."));
    return;
  }

  const bool useStage = useStageForCapture();
  if (!useStage && !confirmContinuousCaptureWithoutStage()) {
    host_->appendLog(QStringLiteral("Capture record: cancelled by operator."));
    return;
  }

  if (useStage) {
    applyDualCameraScanSync();
    if (!buildCaptureScanPlan(captureScanPlan_, errorMessage)) {
      notifyRecordBlocked(errorMessage);
      return;
    }
  }

  if (!beginCaptureRawDumpSession(errorMessage)) {
    notifyRecordBlocked(errorMessage);
    return;
  }

  if (wantMultiview && captureWriterWorker_ != nullptr)
    captureMultiviewSessionDirectory_ = captureWriterWorker_->sessionDirectory();

  resetCaptureSequenceState();
  captureRecorderMode_ = CaptureRecorderMode::Record;
  updateRecorderControls();

  initializeCaptureModeQueue();

  QStringList modeFolders;
  for (const CaptureIlluminationMode mode : illuminationModes)
    modeFolders.push_back(captureIlluminationFolderName(mode));

  if (!useStage) {
    const QString reason = isCaptureStageConnected()
                               ? QStringLiteral("stage scanning disabled")
                               : QStringLiteral("stage not connected");
    host_->appendLog(
        QStringLiteral("Capture record: reflectance only \u2192 %1 (%2). "
                       "Press Stop when finished.")
            .arg(captureWriterWorker_->sessionDirectory(), reason));
    return;
  }

  host_->appendLog(QStringLiteral("Capture record: illumination folders \u2014 "
                                  "%1 (reflectance first, then "
                                  "transmittance when both selected)")
                       .arg(modeFolders.join(QStringLiteral(", "))));

  if (wantMultiview)
    host_->appendLog(QStringLiteral(
        "Capture record: after HSI, stage → apex (%1 mm) then MVS stop (%2 mm), then "
        "Continue/Skip Multiview. Preprocessing can start as soon as HSI is saved.")
                         .arg(hf::hardwareConfig().sampleMultiviewApexPositionMm, 0, 'f', 0)
                         .arg(hf::hardwareConfig().sampleMultiviewPositionMm, 0, 'f', 0));

  host_->appendLog(QStringLiteral("Capture record: session %1 \u2014 "
                                  "per-camera white/bright ref from cfg, "
                                  "sample origin %2 mm, total scan %3 mm, "
                                  "target length %4 mm @ %5 mm/s.")
                       .arg(captureWriterWorker_->sessionDirectory())
                       .arg(captureScanPlan_.sampleScanOriginMm, 0, 'f', 2)
                       .arg(captureScanPlan_.sampleScanTotalDistanceMm, 0, 'f', 2)
                       .arg(captureScanPlan_.sampleScanLengthMm, 0, 'f', 2)
                       .arg(captureScanPlan_.recordScanSpeedMmPerSec, 0, 'f', 1));

  homeStageBeforeCapture();
}

void hf::capture::CapturePanelController::homeStageBeforeCapture() {
  if (host_->stageWorker() == nullptr ||
      host_->stageWorker()->currentState() != StageState::Connected) {
    failCaptureSequence(QStringLiteral("%1: stage is not connected.")
                            .arg(captureSequenceLogPrefix()));
    return;
  }

  captureStageSequenceActive_ = true;

  if (host_->stageWorker()->currentTopology().axesHomed) {
    host_->appendLog(
        QStringLiteral("%1: stage already referenced \u2014 starting from "
                       "current position (scanning home).")
            .arg(captureSequenceLogPrefix()));
    onStageHomedForCapture();
    return;
  }

  host_->stageHomingKind_ = MainWindow::StageHomingKind::BeforeCapture;
  host_->stageWorker()->requestHome();
}

void hf::capture::CapturePanelController::stopRecorder() {
  if (captureRecorderMode_ == CaptureRecorderMode::Idle && !captureMultiviewInProgress_)
    return;

  if (captureMultiviewInProgress_ && host_->ur3ePanel() != nullptr)
    host_->ur3ePanel()->requestStopMotion();

  if (captureScanTimer_ != nullptr)
    captureScanTimer_->stop();
  captureRelativeScanTimerActiveToken_ = 0;

  if (isStageRecordingEnabledInUi() && host_->stageWorker() != nullptr)
    host_->stageWorker()->requestStopMotion();

  if (host_->stageHomingKind_ == MainWindow::StageHomingKind::BeforeCapture)
    host_->stageHomingKind_ = MainWindow::StageHomingKind::None;

  if (captureScanPhase_ == CaptureScanPhase::BlackReference)
    setSelectedCameraShutters(true);

  restoreReflectanceExposuresAfterCapture();

  const bool wasRecord = captureRecorderMode_ == CaptureRecorderMode::Record;
  const bool wasPreview = captureRecorderMode_ == CaptureRecorderMode::Preview;
  const bool multiviewOnly = captureMultiviewOnlySession_;
  captureRecorderMode_ = CaptureRecorderMode::Idle;
  captureStageSequenceActive_ = false;
  captureMultiviewInProgress_ = false;
  captureMultiviewPending_ = false;
  captureMultiviewOnlySession_ = false;
  captureMultiviewAwaitingOperatorConfirm_ = false;
  captureMultiviewSessionDirectory_.clear();
  resetCaptureSequenceState();
  capturePendingIlluminationModes_.clear();
  captureCurrentModeIndex_ = 0;
  updateRecorderControls();
  host_->lightPanel()->updateConnectionDisplay();
  host_->lightPanel()->updateControlsEnabled();

  if (wasRecord) {
    if (!multiviewOnly)
      endCaptureRawDumpSession();
    host_->appendLog(QStringLiteral("Capture record: stopped."));
    homeStageAfterCapture();
  } else if (wasPreview) {
    host_->appendLog(QStringLiteral("Capture preview: stopped."));
    homeStageAfterCapture();
  }
}

double hf::capture::CapturePanelController::scanningHomePositionMm() const {
  if (host_->captureScanningHomeSpin_ != nullptr)
    return host_->captureScanningHomeSpin_->value();
  return 50.0;
}

void hf::capture::CapturePanelController::homeStageAfterCapture() {
  if (host_->performingGracefulShutdown_)
    return;

  if (!isStageRecordingEnabledInUi())
    return;

  if (host_->stageWorker() == nullptr ||
      host_->stageWorker()->currentState() != StageState::Connected)
    return;

  const double parkMm =
      qBound(zaber_stage::kTravelMinimumMm, scanningHomePositionMm(),
             zaber_stage::kTravelLengthMm);
  const double configuredSpeed = hf::hardwareConfig().operationScanningSpeedMmPerSec;
  const double speedMmPerSec =
      configuredSpeed > 0.0 ? configuredSpeed : zaber_stage::kMaxSpeedMmPerSec;

  host_->appendLog(
      QStringLiteral("Capture: parking at scanning home %1 mm @ %2 mm/s\u2026")
          .arg(parkMm, 0, 'f', 2)
          .arg(speedMmPerSec, 0, 'f', 1));

  host_->stageWorker()->requestMoveAbsoluteMm(
      parkMm, speedMmPerSec, true, [this](const bool success) {
        QMetaObject::invokeMethod(
            this,
            [this, success]() {
              if (success)
                host_->appendLog(
                    QStringLiteral("Capture: parked at scanning home."));
              else
                host_->appendLog(
                    QStringLiteral("Capture: scanning home move failed."));
              onStageHomedAfterCapture();
            },
            Qt::QueuedConnection);
      });
}

void hf::capture::CapturePanelController::finishScan() {
  onCaptureRelativeScanComplete();
}

void hf::capture::CapturePanelController::updateCamerasList() {
  const auto isConnected = [](const LumoCameraUi &ui) {
    return ui.state != CameraState::Disconnected &&
           ui.state != CameraState::Fault;
  };

  const auto updateCheckbox = [this, &isConnected](LumoCameraUi &ui,
                                                   QCheckBox *checkbox) {
    if (checkbox == nullptr)
      return;

    if (!isConnected(ui)) {
      checkbox->hide();
      return;
    }

    const bool firstShow = checkbox->isHidden();
    checkbox->setText(host_->cameraPanel()->profileTabNameForUi(ui));
    if (firstShow)
      checkbox->setChecked(true);
    checkbox->show();
  };

  updateCheckbox(host_->camera1Ui_, host_->captureCamera1Check_);
  updateCheckbox(host_->camera2Ui_, host_->captureCamera2Check_);

  const bool bfsConnected =
      host_->bfsPanel() != nullptr && host_->bfsPanel()->isCameraConnected();
  if (host_->captureBfsCheck_ != nullptr) {
    if (!bfsConnected) {
      const QSignalBlocker blocker(host_->captureBfsCheck_);
      host_->captureBfsCheck_->setChecked(false);
      host_->captureBfsCheck_->hide();
    } else {
      const bool firstShow = host_->captureBfsCheck_->isHidden();
      host_->captureBfsCheck_->setText(QStringLiteral("BFS"));
      if (firstShow) {
        const QSignalBlocker blocker(host_->captureBfsCheck_);
        host_->captureBfsCheck_->setChecked(true);
      }
      host_->captureBfsCheck_->show();
    }
  }

  updateDualCameraSyncControls();
  syncBfsAndMultiviewRgbCaptureControls();

  if (host_->captureCamerasEmptyLabel_ != nullptr) {
    const bool anyConnected = isConnected(host_->camera1Ui_) ||
                              isConnected(host_->camera2Ui_) || bfsConnected;
    host_->captureCamerasEmptyLabel_->setHidden(anyConnected);
  }

  updateCaptureStreamLayout();
}

bool hf::capture::CapturePanelController::isBfsCaptureSelected() const {
  return host_->captureBfsCheck_ != nullptr &&
         !host_->captureBfsCheck_->isHidden() &&
         host_->captureBfsCheck_->isChecked();
}

bool hf::capture::CapturePanelController::isMultiviewRgbCaptureSelected() const {
  return host_->captureMultiviewRgbCheck_ != nullptr &&
         host_->captureMultiviewRgbCheck_->isEnabled() &&
         host_->captureMultiviewRgbCheck_->isChecked();
}

bool hf::capture::CapturePanelController::canRunMultiviewRgbCapture() const {
  return host_->ur3ePanel() != nullptr && host_->ur3ePanel()->isRobotConnected()
         && host_->ur3ePanel()->isScanPlanReady() && isCaptureStageConnected()
         && host_->bfsPanel() != nullptr && host_->bfsPanel()->isCameraConnected();
}

bool hf::capture::CapturePanelController::hasHsiCaptureSelection() const {
  std::vector<std::size_t> cameras;
  if (!selectedCaptureCameraIndices(cameras))
    return false;
  std::vector<CaptureIlluminationMode> modes;
  return selectedCaptureIlluminationModes(modes) && !modes.empty();
}

QString hf::capture::CapturePanelController::captureMultiviewOutputDir() const {
  QString sessionDir = captureMultiviewSessionDirectory_;
  if (sessionDir.isEmpty() && captureWriterWorker_ != nullptr)
    sessionDir = captureWriterWorker_->sessionDirectory();
  if (sessionDir.isEmpty())
    return {};
  return QDir(sessionDir).filePath(QStringLiteral("multiview"));
}

bool hf::capture::CapturePanelController::beginMultiviewOnlyDatasetSession(
    QString &errorMessage) {
  if (!validateCaptureRecordMetadata(errorMessage))
    return false;

  const QString saveFolder = host_->captureSaveFolderEdit_->text().trimmed();
  const QString dataset = host_->captureDatasetEdit_->text().trimmed();
  const QString sessionDir = QDir(saveFolder).filePath(dataset);
  if (!QDir().mkpath(sessionDir)) {
    errorMessage =
        QStringLiteral("Could not create dataset folder:\n%1").arg(sessionDir);
    return false;
  }
  const QString multiviewDir = QDir(sessionDir).filePath(QStringLiteral("multiview"));
  if (!QDir().mkpath(multiviewDir)) {
    errorMessage =
        QStringLiteral("Could not create multiview folder:\n%1").arg(multiviewDir);
    return false;
  }

  captureMultiviewSessionDirectory_ = sessionDir;
  lastEndedCaptureSessionSummary_ = {};
  lastEndedCaptureSessionSummary_.sessionDirectory = sessionDir;
  return true;
}

void hf::capture::CapturePanelController::beginMultiviewCapturePhase() {
  if (host_->stageWorker() == nullptr
      || host_->stageWorker()->currentState() != StageState::Connected) {
    captureMultiviewAwaitingOperatorConfirm_ = false;
    failCaptureSequence(QStringLiteral("%1: stage is not connected for Multiview.")
                            .arg(captureSequenceLogPrefix()));
    return;
  }

  if (captureMultiviewSessionDirectory_.isEmpty() && captureWriterWorker_ != nullptr)
    captureMultiviewSessionDirectory_ = captureWriterWorker_->sessionDirectory();

  const auto &hw = hf::hardwareConfig();
  captureMultiviewNeedRingsLeg_ = false;
  captureMultiviewFramesSoFar_ = 0;
  captureMultiviewPinsSoFar_ = 0;
  captureMultiviewElapsedMsSoFar_ = 0;
  const double targetMm = hw.sampleMultiviewApexPositionMm;
  captureScanPhase_ = CaptureScanPhase::MoveToMultiviewPosition;
  host_->appendLog(
      QStringLiteral("%1: moving stage to Multiview apex pose %2 mm…")
          .arg(captureSequenceLogPrefix())
          .arg(targetMm, 0, 'f', 2));
  requestCaptureAbsoluteMove(targetMm, CaptureScanPhase::MoveToMultiviewPosition);
  updateRecorderStatus();
}

void hf::capture::CapturePanelController::startHemisphereMultiviewCaptureAfterStageMove() {
  if (host_->ur3ePanel() == nullptr) {
    failCaptureSequence(QStringLiteral("%1: UR3e panel unavailable for Multiview.")
                            .arg(captureSequenceLogPrefix()));
    return;
  }

  const QString outDir = captureMultiviewOutputDir();
  if (outDir.isEmpty()) {
    failCaptureSequence(QStringLiteral("%1: Multiview output directory is empty.")
                            .arg(captureSequenceLogPrefix()));
    return;
  }
  QDir().mkpath(outDir);

  const auto &hw = hf::hardwareConfig();
  const bool twoStage = hw.sampleMultiviewTwoStage();
  hf::ur3e::HemisphereScanExecuteOptions opts;
  opts.captureOutputDir = outDir;
  opts.stabilizeMs = hw.ur3e.scanCaptureStabilizeMs;
  opts.suppressUiSummary = true;
  if (twoStage && !captureMultiviewNeedRingsLeg_)
  {
    opts.pinSet = hf::ur3e::HemisphereScanPinSet::ApexOnly;
    opts.applyApexStageOutputShift = true;
  }
  else if (twoStage && captureMultiviewNeedRingsLeg_)
  {
    opts.pinSet = hf::ur3e::HemisphereScanPinSet::RingsOnly;
    opts.startFrameIndex = captureMultiviewFramesSoFar_;
    opts.appendTransformsJson = true;
  }
  captureMultiviewInProgress_ = true;
  lastCaptureMultiviewSummaryText_.clear();
  const QString leg =
      !twoStage ? QStringLiteral("full hemisphere")
                : (captureMultiviewNeedRingsLeg_ ? QStringLiteral("MVS rings")
                                                 : QStringLiteral("apex"));
  host_->appendLog(
      QStringLiteral("%1: %2 Multiview capture → %3 (%4 ms settle per pose; wrist sweep if enabled)…")
          .arg(captureSequenceLogPrefix(), leg, outDir)
          .arg(opts.stabilizeMs));

  connect(host_->ur3ePanel(),
          &hf::ur3e::Ur3ePanelController::hemisphereScanExecuteFinished,
          this,
          &CapturePanelController::onMultiviewCaptureFinished,
          Qt::UniqueConnection);

  if (!host_->ur3ePanel()->startHemisphereScanExecute(opts)) {
    captureMultiviewInProgress_ = false;
    failCaptureSequence(
        QStringLiteral("%1: could not start UR3e Multiview execute.")
            .arg(captureSequenceLogPrefix()));
  }
}

void hf::capture::CapturePanelController::onMultiviewCaptureFinished(
    const bool ok,
    const QString &detail,
    const int capturedFrameCount,
    const int successfulPins,
    const qint64 elapsedMs) {
  if (!captureMultiviewInProgress_)
    return;

  host_->appendLog(
      QStringLiteral("%1: Multiview %2 (%3 frame(s)) — %4")
          .arg(captureSequenceLogPrefix())
          .arg(ok ? QStringLiteral("finished") : QStringLiteral("failed"))
          .arg(capturedFrameCount)
          .arg(detail));

  if (!ok)
    lastCaptureMultiviewSummaryText_.clear();

  if (!ok && captureRecorderMode_ == CaptureRecorderMode::Record) {
    captureMultiviewInProgress_ = false;
    failCaptureSequence(
        QStringLiteral("%1: Multiview failed — %2")
            .arg(captureSequenceLogPrefix(), detail));
    return;
  }

  captureMultiviewFramesSoFar_ = capturedFrameCount;
  captureMultiviewPinsSoFar_ += successfulPins;
  captureMultiviewElapsedMsSoFar_ += elapsedMs;

  const auto &hw = hf::hardwareConfig();
  bool haveSpinRings = true;
  if (host_->ur3eHemisphereScanSettings_ != nullptr
      && host_->ur3eHemisphereScanSettings_->scanExecuteMode()
             == hf::ur3e::Ur3eScanExecuteMode::SemiFixed)
  {
    const auto semi = host_->ur3eHemisphereScanSettings_->semiFixedRoute();
    haveSpinRings = false;
    for (const auto &ring : semi.rings)
    {
      if (!ring.noPan && std::abs(ring.thetaDeg) >= 0.75)
      {
        haveSpinRings = true;
        break;
      }
    }
  }
  if (ok && hw.sampleMultiviewTwoStage() && !captureMultiviewNeedRingsLeg_
      && haveSpinRings) {
    captureMultiviewInProgress_ = false;
    captureMultiviewNeedRingsLeg_ = true;
    captureScanPhase_ = CaptureScanPhase::MoveToMultiviewPosition;
    host_->appendLog(
        QStringLiteral("%1: apex stills done (%2 frame(s)) — moving stage to MVS stop %3 mm…")
            .arg(captureSequenceLogPrefix())
            .arg(capturedFrameCount)
            .arg(hw.sampleMultiviewPositionMm, 0, 'f', 2));
    requestCaptureAbsoluteMove(hw.sampleMultiviewPositionMm,
                               CaptureScanPhase::MoveToMultiviewPosition);
    updateRecorderStatus();
    return;
  }

  if (ok) {
    const qint64 totalMs = captureMultiviewElapsedMsSoFar_ > 0
                               ? captureMultiviewElapsedMsSoFar_
                               : elapsedMs;
    const qint64 totalSec = totalMs / 1000;
    lastCaptureMultiviewSummaryText_ =
        QStringLiteral("Multiview\n"
                       "Successful pins: %1\n"
                       "Multiview images recorded: %2\n"
                       "Scanning time: %3:%4")
            .arg(captureMultiviewPinsSoFar_ > 0 ? captureMultiviewPinsSoFar_
                                                : successfulPins)
            .arg(captureMultiviewFramesSoFar_ > 0 ? captureMultiviewFramesSoFar_
                                                  : capturedFrameCount)
            .arg(totalSec / 60)
            .arg(totalSec % 60, 2, 10, QLatin1Char('0'));
  }

  finishCaptureSequenceAfterOptionalMultiview();
}

void hf::capture::CapturePanelController::syncBfsAndMultiviewRgbCaptureControls(
    QObject *source) {
  if (host_ == nullptr || syncingBfsMultiviewSelection_)
    return;

  const bool planReady =
      host_->ur3ePanel() != nullptr && host_->ur3ePanel()->isScanPlanReady();
  const bool robotConnected =
      host_->ur3ePanel() != nullptr && host_->ur3ePanel()->isRobotConnected();
  const bool stageConnected = isCaptureStageConnected();
  const bool bfsVisible = host_->captureBfsCheck_ != nullptr &&
                          !host_->captureBfsCheck_->isHidden();
  const bool multiviewReady =
      planReady && robotConnected && stageConnected && bfsVisible;

  if (host_->captureMultiviewRgbCheck_ != nullptr) {
    host_->captureMultiviewRgbCheck_->setEnabled(multiviewReady);
    if (!multiviewReady) {
      const QSignalBlocker blocker(host_->captureMultiviewRgbCheck_);
      host_->captureMultiviewRgbCheck_->setChecked(false);
      if (!bfsVisible) {
        host_->captureMultiviewRgbCheck_->setToolTip(
            tr("Connect the BFS camera to enable Multiview RGB capture."));
      } else if (!robotConnected) {
        host_->captureMultiviewRgbCheck_->setToolTip(
            tr("Connect the UR3e robot to enable Multiview RGB capture."));
      } else if (!stageConnected) {
        host_->captureMultiviewRgbCheck_->setToolTip(
            tr("Connect the stage to enable Multiview RGB capture."));
      } else {
        host_->captureMultiviewRgbCheck_->setToolTip(
            tr("BFS RGB for Multiview. Plan a UR3e scan route with reachable "
               "poses first."));
      }
    } else {
      host_->captureMultiviewRgbCheck_->setToolTip(
          tr("BFS RGB for Multiview (synced with the BFS camera checkbox). "
             "Preview = UR3e Execute (stage apex→MVS if connected). "
             "Record = stage → apex then MVS stop, then stills."));
    }
  }

  syncingBfsMultiviewSelection_ = true;

  if (source == host_->captureBfsCheck_ && host_->captureMultiviewRgbCheck_ != nullptr) {
    if (isBfsCaptureSelected() && multiviewReady)
      host_->captureMultiviewRgbCheck_->setChecked(true);
    else
      host_->captureMultiviewRgbCheck_->setChecked(false);
  } else if (source == host_->captureMultiviewRgbCheck_ &&
             host_->captureBfsCheck_ != nullptr && bfsVisible) {
    host_->captureBfsCheck_->setChecked(
        host_->captureMultiviewRgbCheck_ != nullptr &&
        host_->captureMultiviewRgbCheck_->isChecked());
  } else if (multiviewReady && isBfsCaptureSelected() &&
             host_->captureMultiviewRgbCheck_ != nullptr &&
             !host_->captureMultiviewRgbCheck_->isChecked()) {
    host_->captureMultiviewRgbCheck_->setChecked(true);
  }

  syncingBfsMultiviewSelection_ = false;
}

LumoCameraUi *hf::capture::CapturePanelController::cameraUiForIndex(
    const std::size_t cameraIndex) {
  if (cameraIndex == 0)
    return &host_->camera1Ui_;
  if (cameraIndex == 1)
    return &host_->camera2Ui_;
  return nullptr;
}

bool hf::capture::CapturePanelController::isCaptureStreamWaterfallVisible(
    const std::size_t cameraIndex) const {
  if (cameraIndex >= 2)
    return false;

  std::vector<std::size_t> selected;
  if (!selectedCaptureCameraIndices(selected))
    return false;

  return std::find(selected.begin(), selected.end(), cameraIndex) !=
         selected.end();
}

void hf::capture::CapturePanelController::updateCaptureStreamLayout() {
  if (host_->captureStreamGrid_ == nullptr)
    return;

  while (QLayoutItem *item = host_->captureStreamGrid_->takeAt(0))
    delete item;

  for (QGroupBox *pane : host_->captureWaterfallPanes_) {
    if (pane != nullptr)
      pane->hide();
  }
  if (host_->captureBfsPreviewPane_ != nullptr)
    host_->captureBfsPreviewPane_->hide();

  std::vector<std::size_t> selectedHsi;
  const bool hasHsi = selectedCaptureCameraIndices(selectedHsi);
  const bool showBfs = isBfsCaptureSelected();

  if (!hasHsi && !showBfs) {
    if (host_->captureStreamEmptyLabel_ != nullptr)
      host_->captureStreamEmptyLabel_->show();
    if (host_->cameraPanel() != nullptr)
      host_->cameraPanel()->refreshWaterfallDisplayTargets();
    return;
  }

  if (host_->captureStreamEmptyLabel_ != nullptr)
    host_->captureStreamEmptyLabel_->hide();

  std::vector<QGroupBox *> visiblePanes;
  visiblePanes.reserve(3);

  for (const std::size_t cameraIndex : selectedHsi) {
    if (cameraIndex >= 2 ||
        host_->captureWaterfallPanes_[cameraIndex] == nullptr)
      continue;

    LumoCameraUi *ui = cameraUiForIndex(cameraIndex);
    if (ui != nullptr) {
      host_->captureWaterfallPanes_[cameraIndex]->setTitle(
          host_->cameraPanel()->profileTabNameForUi(*ui) +
          QStringLiteral(" waterfall"));
    }
    host_->captureWaterfallPanes_[cameraIndex]->show();
    visiblePanes.push_back(host_->captureWaterfallPanes_[cameraIndex]);
  }

  if (showBfs && host_->captureBfsPreviewPane_ != nullptr) {
    host_->captureBfsPreviewPane_->setTitle(QStringLiteral("BFS"));
    host_->captureBfsPreviewPane_->show();
    visiblePanes.push_back(host_->captureBfsPreviewPane_);
  }

  const int count = static_cast<int>(visiblePanes.size());
  if (count == 1) {
    host_->captureStreamGrid_->addWidget(visiblePanes[0], 0, 0, 1, 1);
  } else if (count == 2) {
    host_->captureStreamGrid_->addWidget(visiblePanes[0], 0, 0);
    host_->captureStreamGrid_->addWidget(visiblePanes[1], 0, 1);
  } else if (count >= 3) {
    host_->captureStreamGrid_->addWidget(visiblePanes[0], 0, 0);
    host_->captureStreamGrid_->addWidget(visiblePanes[1], 0, 1);
    host_->captureStreamGrid_->addWidget(visiblePanes[2], 0, 2);
  }

  host_->captureStreamGrid_->setColumnStretch(0, 1);
  host_->captureStreamGrid_->setColumnStretch(1, count >= 2 ? 1 : 0);
  host_->captureStreamGrid_->setColumnStretch(2, count >= 3 ? 1 : 0);
  host_->captureStreamGrid_->setRowStretch(0, 1);
  host_->captureStreamGrid_->setRowStretch(1, 0);

  if (host_->cameraPanel() != nullptr)
    host_->cameraPanel()->refreshWaterfallDisplayTargets();
}

bool hf::capture::CapturePanelController::
    hasReflectanceAndTransmittanceCaptureModes() const {
  bool hasReflectance = false;
  bool hasTransmittance = false;
  for (const CaptureIlluminationMode mode : capturePendingIlluminationModes_) {
    if (mode == CaptureIlluminationMode::Reflectance)
      hasReflectance = true;
    else if (mode == CaptureIlluminationMode::Transmittance)
      hasTransmittance = true;
  }
  return hasReflectance && hasTransmittance;
}

void hf::capture::CapturePanelController::
    saveReflectanceExposuresForDualModeCapture() {
  for (std::size_t cameraIndex = 0; cameraIndex < 2; ++cameraIndex) {
    LumoCameraUi &ui = cameraIndex == 0 ? host_->camera1Ui_ : host_->camera2Ui_;
    savedReflectanceExposureMs_[cameraIndex] =
        ui.exposureSpin != nullptr ? ui.exposureSpin->value() : 0.0;
  }

  host_->appendLog(
      QStringLiteral(
          "%1: saved reflectance exposures \u2014 %2: %3 ms, %4: %5 ms")
          .arg(captureSequenceLogPrefix())
          .arg(host_->cameraPanel()->profileTabNameForUi(host_->camera1Ui_))
          .arg(savedReflectanceExposureMs_[0], 0, 'f', 2)
          .arg(host_->cameraPanel()->profileTabNameForUi(host_->camera2Ui_))
          .arg(savedReflectanceExposureMs_[1], 0, 'f', 2));
}

void hf::capture::CapturePanelController::applyCaptureExposureForCamera(
    const std::size_t cameraIndex, const double exposureMs) {
  LumoCameraUi &ui = cameraIndex == 0 ? host_->camera1Ui_ : host_->camera2Ui_;
  if (ui.exposureSpin != nullptr) {
    QSignalBlocker blocker(ui.exposureSpin);
    ui.exposureSpin->setValue(exposureMs);
  }

  if (host_->coordinator() == nullptr ||
      !host_->isCameraSessionActive(ui.state))
    return;

  CameraSettings settings = host_->cameraPanel()->buildSettings(ui);
  settings.exposureMs = exposureMs;
  host_->coordinator()->applySettings(cameraIndex, settings);
}

void hf::capture::CapturePanelController::
    applyTransmittanceExposuresFromConfig() {
  const hf::HardwareConfig &hw = hf::hardwareConfig();
  std::vector<std::size_t> selectedCameras;
  if (!selectedCaptureCameraIndices(selectedCameras))
    return;

  captureIlluminationExposureSwitchActive_ = true;

  for (const std::size_t cameraIndex : selectedCameras) {
    const double exposureMs = hw.transmittanceExposureMs[cameraIndex];
    applyCaptureExposureForCamera(cameraIndex, exposureMs);
    host_->appendLog(
        QStringLiteral("%1: transmittance exposure \u2014 %2 set to %3 ms "
                       "(from hyperfusion.cfg)")
            .arg(captureSequenceLogPrefix())
            .arg(host_->cameraPanel()->profileTabNameForUi(
                cameraIndex == 0 ? host_->camera1Ui_ : host_->camera2Ui_))
            .arg(exposureMs, 0, 'f', 2));
  }
}

void hf::capture::CapturePanelController::
    restoreReflectanceExposuresAfterCapture() {
  if (!captureUsingTransmittanceExposure_) {
    dualModeExposureSwitchEnabled_ = false;
    captureIlluminationExposureSwitchActive_ = false;
    return;
  }

  captureIlluminationExposureSwitchActive_ = true;
  std::vector<std::size_t> selectedCameras;
  if (selectedCaptureCameraIndices(selectedCameras)) {
    for (const std::size_t cameraIndex : selectedCameras) {
      applyCaptureExposureForCamera(cameraIndex,
                                    savedReflectanceExposureMs_[cameraIndex]);
      host_->appendLog(
          QStringLiteral(
              "%1: restored reflectance exposure \u2014 %2 set to %3 ms")
              .arg(captureSequenceLogPrefix())
              .arg(host_->cameraPanel()->profileTabNameForUi(
                  cameraIndex == 0 ? host_->camera1Ui_ : host_->camera2Ui_))
              .arg(savedReflectanceExposureMs_[cameraIndex], 0, 'f', 2));
    }
  }

  captureUsingTransmittanceExposure_ = false;
  dualModeExposureSwitchEnabled_ = false;
  captureIlluminationExposureSwitchActive_ = false;
}

QString
hf::capture::CapturePanelController::buildTransmittanceExposureChangeNotice()
    const {
  const hf::HardwareConfig &hw = hf::hardwareConfig();
  QStringList lines;
  lines << QStringLiteral(
      "Camera exposure updated for transmittance (from hyperfusion.cfg):");

  std::vector<std::size_t> selectedCameras;
  if (!selectedCaptureCameraIndices(selectedCameras))
    return lines.join(QStringLiteral("\n"));

  for (const std::size_t cameraIndex : selectedCameras) {
    const LumoCameraUi &ui =
        cameraIndex == 0 ? host_->camera1Ui_ : host_->camera2Ui_;
    lines << QStringLiteral("  %1: %2 ms \u2192 %3 ms")
                 .arg(host_->cameraPanel()->profileTabNameForUi(ui))
                 .arg(savedReflectanceExposureMs_[cameraIndex], 0, 'f', 2)
                 .arg(hw.transmittanceExposureMs[cameraIndex], 0, 'f', 2);
  }

  return lines.join(QStringLiteral("\n"));
}
