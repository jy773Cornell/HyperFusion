// Capture tab orchestration: recorder, stage scan sequence, writer, and post-processing.
// UI widgets remain on MainWindow; this controller owns capture state and sequence logic.
#pragma once

#include "backend/camera/CaptureWriterTypes.hpp"
#include "backend/camera/CameraTypes.hpp"
#include "backend/camera/processing/CapturePostProcessor.hpp"

#include <QElapsedTimer>
#include <QObject>
#include <QString>

#include <array>
#include <cstddef>
#include <memory>
#include <optional>
#include <vector>

class CapturePostProcessorWorker;
class CaptureWriterWorker;
class HfFusionWorker;
class MainWindow;
class OperationWaitDialog;
class QWidget;

struct LumoCameraUi;
struct LighthouseControllerPowerStatus;
enum class CaptureIlluminationMode;
enum class StageState;

namespace hf::processing
{
class Gsam2ServerManager;
}

namespace hf::capture
{
class CapturePanelController : public QObject
{
    Q_OBJECT

public:
    struct CaptureScanPlan
    {
        double whiteRefStartMm[2] = {0.0, 0.0};
        double brightRefStartMm[2] = {0.0, 0.0};
        double sampleScanStartMm[2] = {0.0, 0.0};
        double sampleScanOriginMm = 0.0;
        double sampleScanTotalDistanceMm = 0.0;
        double sampleScanLengthMm = 0.0;
        double whiteRefScanOriginMm = 0.0;
        double whiteRefScanTotalDistanceMm = 0.0;
        double whiteRefScanDistanceMm[2] = {0.0, 0.0};
        /// One-pass record scan: origin/end span all white-ref and sample windows.
        double recordScanOriginMm = 0.0;
        double recordScanTotalDistanceMm = 0.0;
        double operationSpeedMmPerSec = 0.0;
        double recordScanSpeedMmPerSec = 0.0;
        int whiteReferenceFrameCount = 0;
        int blackReferenceFrameCount = 0;
        int blackReferenceShutterSettleMs = 1500;
    };

    enum class CaptureScanPhase
    {
        Idle,
        MoveToFirstRefPosition,
        MoveToTempStopPosition,
        MoveToWhiteRefScanOrigin,
        MoveToSampleScanOrigin,
        BlackReference,
        WhiteReferenceScan,
        SampleScan,
        /// Single stage pass: white ref then sample windows per camera (dual or single).
        CombinedRecordScan,
        /// Absolute move to sample_multiview_position_mm before hemisphere BFS capture.
        MoveToMultiviewPosition,
    };

    enum class CaptureRecorderMode
    {
        Idle,
        Preview,
        Record,
    };

    explicit CapturePanelController(MainWindow *host, QObject *parent = nullptr);

    void initializeWorkers();
    void shutdownWorkers();

    [[nodiscard]] bool isSessionActive() const;
    [[nodiscard]] CaptureRecorderMode recorderMode() const;

    QWidget *createStreamTab();
    void onStagePosition(double positionMm);
    void onStageHomedForCapture();
    void onStageHomedAfterCapture();
    void onCaptureStageHomingFailed(const QString &message);
    void onStreamFrame(const SharedFramePacket &frame);
    void updatePositionControls(StageState state);
    void updateCamerasList();
    void updateRecorderControls();
    void updateGsamServerUi();
    void refreshGsamPlanCombo(const QString &preferredPlanId = {});
    void tryAutoStartGsamServer();
    void wireSettingsTabConnections();
    void updateDualCameraSyncControls();
    void updateScanningSpeedControls();
    void updateCaptureStreamLayout();
    /// @param source Checkbox that changed (BFS or Multiview RGB), or nullptr for availability refresh.
    void syncBfsAndMultiviewRgbCaptureControls(QObject *source = nullptr);
    [[nodiscard]] bool isBfsCaptureSelected() const;
    [[nodiscard]] bool isCaptureStreamWaterfallVisible(std::size_t cameraIndex) const;
    [[nodiscard]] bool dualCameraScanSyncActive() const;
    [[nodiscard]] bool shouldLockSwir3ForDualSync() const;
    [[nodiscard]] bool isApplyingDualCameraScanSync() const { return applyingDualCameraScanSync_; }

    void applyDualCameraScanSync(bool applyToHardware = true);
    void onCaptureStreamTabActivated();
    void resetDualCameraScanSyncHardwareState();
    void maybeApplyInitialDualCameraSync();

    [[nodiscard]] bool isDualCameraSyncHardwareApplyPending() const;
    [[nodiscard]] bool shouldSuppressCameraTimingDialogs() const;
    void notifyDualCameraSyncSettingsApplied(const CameraSettingsApplyReport &report);
    void notifyDualCameraSyncApplyFailed(const QString &message);
    void dismissDualCameraSyncWaitDialog();

public slots:
    void startPreview();
    void startRecord();
    void stopRecorder();
    void finishScan();

private:
    double autoRecordScanSpeedMmPerSec() const;
    bool bothFx10eAndSwir3CaptureCamerasConnected() const;
    [[nodiscard]] bool dualCameraScanSyncReadyForHardware() const;
    void updateSessionUiLock();
    void syncCaptureIlluminationModeControls();
    void updateRecorderStatus();
    void clearRecorderCameraStatusLabels();
    void notifyRecordComplete();
    void beginRecordCompleteNotify();
    void tryNotifyRecordComplete();
    void notifyPreviewComplete();
    void beginPreviewCompleteNotify();
    void tryNotifyPreviewComplete();
    bool buildCaptureScanPlan(CaptureScanPlan &plan, QString &errorMessage) const;
    QString captureSequenceLogPrefix() const;
    void resetCaptureSequenceState();
    void initializeCaptureModeQueue();
    bool confirmCaptureStart(const LighthouseControllerPowerStatus &powerStatus) const;
    bool confirmCaptureHoodPreparation(CaptureIlluminationMode mode,
                                       bool betweenReflectanceAndTransmittance);
    bool confirmContinuousCaptureWithoutStage() const;
    [[nodiscard]] bool confirmContinueWithMultiviewAfterHsi() const;
    [[nodiscard]] QString buildScanningProcedureSummary(
        const LighthouseControllerPowerStatus *powerStatus = nullptr) const;
    void finishRecordAfterSkippingMultiview();
    void startHemisphereMultiviewCaptureAfterStageMove();
    void startCurrentCaptureMode();
    void beginCaptureModeMotion();
    void beginCaptureMoveToTempStopPosition();
    void completeCaptureModeSequence();
    void startCaptureSequence();
    void requestCaptureAbsoluteMove(double positionMm,
                                    CaptureScanPhase expectedPhaseOnComplete,
                                    double speedMmPerSec = 0.0);
    void onCaptureAbsoluteMoveComplete(bool success);
    void beginCaptureMoveToFirstRefPosition();
    void beginCaptureBlackReference();
    void scheduleBlackReferenceShutterSettleCheck();
    void activateBlackReferenceCollection(const QString &logLine);
    bool selectedCaptureCameraShuttersReportedClosed() const;
    void onCaptureBlackReferenceComplete();
    void beginCaptureRecordScanSequence();
    void beginCaptureMoveToWhiteRefScanOrigin();
    void beginCombinedRecordScan();
    void onCaptureCombinedRecordScanComplete();
    void onCaptureWhiteReferenceSequenceComplete();
    void onCaptureSampleScanComplete();
    void onWhiteReferenceFrameCollected(std::size_t cameraIndex);
    void updateCombinedRecordScanGeometry();
    bool selectedCamerasReachedWhiteReferenceTarget() const;
    bool shouldAcceptWhiteReferenceFrame(std::size_t cameraIndex) const;
    bool shouldRecordWhiteReferenceFrameForCamera(std::size_t stageCameraIndex,
                                                  std::size_t cameraIndex,
                                                  double stagePositionMm) const;
    bool allSelectedCamerasPastWhiteReferenceWindow(double stagePositionMm) const;
    double whiteReferenceScanDistanceMmForCamera(const LumoCameraUi &ui,
                                                 const CaptureScanPlan &plan) const;
    void beginCaptureSampleScan();
    void verifySampleScanOriginAndStartScan();
    void startCaptureRelativeScan(double distanceMm,
                                  double speedMmPerSec,
                                  CaptureScanPhase capturePhaseOnMoveStart = CaptureScanPhase::Idle);
    void onCaptureRelativeScanComplete();
    void completeCaptureSequence();
    void beginMultiviewCapturePhase();
    void onMultiviewCaptureFinished(bool ok,
                                     const QString &detail,
                                     int capturedFrameCount,
                                     int successfulPins,
                                     qint64 elapsedMs);
    void finishCaptureSequenceAfterOptionalMultiview();
    [[nodiscard]] bool isMultiviewRgbCaptureSelected() const;
    [[nodiscard]] bool canRunMultiviewRgbCapture() const;
    [[nodiscard]] bool hasHsiCaptureSelection() const;
    [[nodiscard]] QString captureMultiviewOutputDir() const;
    bool beginMultiviewOnlyDatasetSession(QString &errorMessage);
    void runCapturePostProcessingIfEnabled();
    hf::processing::CapturePostProcessOptions capturePostProcessOptionsFromUi() const;
    void failCaptureSequence(const QString &message);
    void setSelectedCameraShutters(bool open);
    bool selectedCamerasReachedBlackReferenceTarget() const;
    bool shouldAcceptBlackReferenceFrame(std::size_t cameraIndex) const;
    bool validateCaptureRecordMetadata(QString &errorMessage) const;
    void notifyRecordBlocked(const QString &message) const;
    bool selectedCaptureCameraIndices(std::vector<std::size_t> &cameraIndices) const;
    std::size_t stageCameraIndexForUi(const LumoCameraUi &ui, std::size_t cameraIndex) const;
    double whiteRefStartMmForStageCamera(const CaptureScanPlan &plan,
                                         CaptureIlluminationMode mode,
                                         std::size_t stageCameraIndex) const;
    double estimatedStageScanPositionMm() const;
    double currentStageScanPositionMm() const;
    std::optional<double> knownStageScanPositionMm() const;
    double maxPlausibleStageScanPositionMm() const;
    bool isStageScanPositionTrustworthy(double positionMm) const;
    bool hasStageScanElapsedForPosition(double positionMm) const;
    bool isStagePositionWithinScanWindow(double positionMm,
                                         double windowStartMm,
                                         double windowLengthMm) const;
    bool isStagePositionWithinSampleWindow(double positionMm,
                                           double windowStartMm,
                                           double windowLengthMm) const;
    bool allSelectedCamerasPastSampleWindow(double stagePositionMm) const;
    bool selectedCamerasEnteredSampleWindow() const;
    bool selectedCamerasHaveSampleFrames() const;
    bool canCompleteSampleScan() const;
    bool canCompleteCombinedRecordScan() const;
    [[nodiscard]] QString recorderCameraStatusText(std::size_t cameraIndex) const;
    void updateSampleScanWindowProgress(double stagePositionMm);
    void scheduleRelativeScanTimer(double distanceMm, double speedMmPerSec);
    void extendSampleScanTimer();
    bool shouldRecordSampleFrameForCamera(std::size_t stageCameraIndex,
                                          double stagePositionMm) const;
    bool selectedCaptureIlluminationModes(std::vector<CaptureIlluminationMode> &modes) const;
    bool isCaptureStageConnected() const;
    bool isCaptureStagePresent() const;
    bool isStageRecordingEnabledInUi() const;
    bool useStageForCapture() const;
    bool effectiveCaptureIlluminationModes(std::vector<CaptureIlluminationMode> &modes) const;
    QString captureIlluminationFolderName(CaptureIlluminationMode mode) const;
    QString captureCameraFolderName(const LumoCameraUi &ui) const;
    QString captureStreamRelativeRoot(CaptureIlluminationMode mode, const LumoCameraUi &ui) const;
    bool resolveCaptureRecordingIlluminationMode(CaptureIlluminationMode &mode,
                                                 QString &errorMessage) const;
    bool selectedCaptureCameraStreaming(QString &errorMessage) const;
    bool buildCaptureWriterSessionConfig(CaptureWriterSessionConfig &config,
                                         QString &errorMessage) const;
    bool beginCaptureRawDumpSession(QString &errorMessage);
    void endCaptureRawDumpSession();
    void appendCaptureRecordFrame(const FramePacket &frame);
    void homeStageBeforeCapture();
    void homeStageAfterCapture();
    [[nodiscard]] double scanningHomePositionMm() const;
    void handleCapturePreviewFrame(const SharedFramePacket &frame);
    LumoCameraUi *cameraUiForIndex(std::size_t cameraIndex);
    [[nodiscard]] bool hasReflectanceAndTransmittanceCaptureModes() const;
    void saveReflectanceExposuresForDualModeCapture();
    void applyTransmittanceExposuresFromConfig();
    void restoreReflectanceExposuresAfterCapture();
    void applyCaptureExposureForCamera(std::size_t cameraIndex, double exposureMs);
    QString buildTransmittanceExposureChangeNotice() const;

    MainWindow *host_ = nullptr;

    CaptureRecorderMode captureRecorderMode_ = CaptureRecorderMode::Idle;
    CaptureScanPhase captureScanPhase_ = CaptureScanPhase::Idle;
    CaptureScanPlan captureScanPlan_;
    CaptureScanPhase captureMoveCompletePhase_ = CaptureScanPhase::Idle;
    std::array<bool, 2> captureWhiteRefWindowComplete_ = {false, false};
    double captureScanOriginPositionMm_ = 0.0;
    double captureActiveScanDistanceMm_ = 0.0;
    double captureLastKnownStagePositionMm_ = 0.0;
    bool captureScanTimingActive_ = false;
    bool captureStagePositionKnown_ = false;
    std::array<bool, 2> captureSampleWindowComplete_ = {false, false};
    std::array<bool, 2> captureSampleWindowEntered_ = {false, false};
    bool captureSampleRecordingActive_ = false;
    bool captureStageSequenceActive_ = false;
    QElapsedTimer captureScanElapsed_;
    CaptureIlluminationMode captureRecordingIlluminationMode_ = CaptureIlluminationMode::Reflectance;
    std::vector<CaptureIlluminationMode> capturePendingIlluminationModes_;
    std::size_t captureCurrentModeIndex_ = 0;
    std::array<int, 2> captureBlackRefFramesCollected_ = {0, 0};
    bool captureBlackReferenceCollectionActive_ = false;
    QElapsedTimer captureBlackRefShutterSettleTimer_;
    std::array<int, 2> captureWhiteRefFramesCollected_ = {0, 0};
    std::array<int, 2> captureSampleFramesCollected_ = {0, 0};
    quint64 captureRelativeScanTimerToken_ = 0;
    quint64 captureRelativeScanTimerActiveToken_ = 0;
    int captureSampleScanTimerExtendCount_ = 0;
    bool captureStageWasPresent_ = false;
    bool pendingCaptureRecordCompleteNotify_ = false;
    bool captureRecordCompleteHomingPending_ = false;
    bool captureRecordCompletePostProcessPending_ = false;
    bool pendingCapturePreviewCompleteNotify_ = false;
    bool capturePreviewCompleteHomingPending_ = false;
    QStringList lastPreviewCompleteModeFolders_;
    std::array<int, 2> lastPreviewCompleteSampleFrames_ = {0, 0};
    std::vector<std::size_t> lastPreviewCompleteCameraIndices_;
    bool applyingDualCameraScanSync_ = false;
    bool syncingBfsMultiviewSelection_ = false;
    bool captureMultiviewPending_ = false;
    bool captureMultiviewOnlySession_ = false;
    bool captureMultiviewInProgress_ = false;
    /// After HSI: stage moves to Multiview pose first; confirm dialog runs when that move completes.
    bool captureMultiviewAwaitingOperatorConfirm_ = false;
    /// Filled after Capture-driven Multiview; merged into Recording complete (empty if Multiview skipped).
    QString lastCaptureMultiviewSummaryText_;
    /// True after HSI dump ended and post-process was queued for this session (may run during Multiview).
    bool capturePostProcessStartedForSession_ = false;
    bool capturePostProcessInFlight_ = false;
    QString captureMultiviewSessionDirectory_;
    bool dualCameraScanSyncHardwareApplied_ = false;
    bool dualCameraSyncHardwareApplyPending_ = false;
    double lastAppliedSwir3SyncFrameRateHz_ = -1.0;
    OperationWaitDialog *dualCameraSyncWaitDialog_ = nullptr;
    bool dualModeExposureSwitchEnabled_ = false;
    bool captureUsingTransmittanceExposure_ = false;
    bool captureIlluminationExposureSwitchActive_ = false;
    std::array<double, 2> savedReflectanceExposureMs_ = {0.0, 0.0};

    struct DualCameraSyncSummaryContext
    {
        double fx10eFrameRateHz = 0.0;
        double fx10eExposureMs = 0.0;
        double fx10eSpatialMmPerPixel = 0.0;
        double swirSpatialMmPerPixel = 0.0;
        double requestedSwirFrameRateHz = 0.0;
        bool valid = false;
    };

    DualCameraSyncSummaryContext pendingDualSyncSummary_{};
    void showDualCameraSyncWaitDialog();
    void showDualCameraSyncSummaryDialog(const CameraSettingsApplyReport &report);
    class QTimer *captureScanTimer_ = nullptr;
    class QTimer *captureRecorderStatusTimer_ = nullptr;
    std::unique_ptr<CaptureWriterWorker> captureWriterWorker_;
    std::unique_ptr<CapturePostProcessorWorker> capturePostProcessorWorker_;
    std::unique_ptr<HfFusionWorker> hfFusionWorker_;
    CaptureWriterSessionSummary lastEndedCaptureSessionSummary_;
    std::unique_ptr<hf::processing::Gsam2ServerManager> gsam2ServerManager_;
};
} // namespace hf::capture
