// Main application window UI layout and control wiring.
#pragma once

#include <QElapsedTimer>
#include <QImage>
#include <QMainWindow>

#include <array>
#include <cstddef>
#include <functional>
#include <memory>
#include <vector>

#include "adapters/lumo/LumoDeviceTypes.hpp"
#include "adapters/lumo/CalpackBandCatalog.hpp"
#include "backend/CameraTypes.hpp"
#include "backend/LighthouseTypes.hpp"
#include "backend/StageTypes.hpp"
#include "backend/CameraCoordinator.hpp"
#include "backend/CaptureWriterTypes.hpp"
#include "frontend/processing/ProfileProcessor.hpp"
#include "frontend/processing/WaterfallProcessor.hpp"

class CaptureWriterWorker;
class CapturePostProcessorWorker;
class LumoCamera;
class QLabel;
class QGroupBox;
class Swir3NiCamera;

class LighthouseWorker;
class OperationWaitDialog;
class StageWorker;
class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QGroupBox;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QTabWidget;
class QTimer;
class QToolButton;
class QWidget;

namespace ui
{
class DetectorCrosshairWidget;
class IntensityBarWidget;
class ProfilePlotWidget;
class StageAxisWidget;
} // namespace ui

struct LumoCameraUi
{
    std::shared_ptr<LumoCamera> camera;
    LumoSensorKind sensorKind = LumoSensorKind::Fx10ePleora;
    std::size_t cameraIndex = 0;
    CameraState state = CameraState::Disconnected;
    bool autoStreamStarted = false;
    bool connectAttemptActive = false;
    bool shutterReportedOpen = false;

    QComboBox *deviceCombo = nullptr;
    QLineEdit *calibrationPackEdit = nullptr;
    QPushButton *calibrationPackBrowseBtn = nullptr;
    QPushButton *connectBtn = nullptr;
    QPushButton *applyBtn = nullptr;
    QDoubleSpinBox *exposureSpin = nullptr;
    int frameWidth = 0;
    int frameHeight = 0;

    QGroupBox *detectorPane = nullptr;
    QGroupBox *waterfallPane = nullptr;
    QGroupBox *wavelengthPane = nullptr;
    QGroupBox *pixelStreamPane = nullptr;
    ui::DetectorCrosshairWidget *detectorView = nullptr;
    QLabel *waterfallView = nullptr;
    ui::ProfilePlotWidget *wavelengthView = nullptr;
    ui::ProfilePlotWidget *pixelStreamView = nullptr;
    QDoubleSpinBox *frameRateSpin = nullptr;
    QComboBox *spectralBinningCombo = nullptr;
    QComboBox *spatialBinningCombo = nullptr;
    QLabel *shutterIndicator = nullptr;
    QLabel *shutterStatusLabel = nullptr;
    QPushButton *shutterToggleBtn = nullptr;
    QComboBox *triggerCombo = nullptr;
    QComboBox *redBandCombo = nullptr;
    QComboBox *greenBandCombo = nullptr;
    QComboBox *blueBandCombo = nullptr;
    std::vector<SpectralBand> spectralBands;
};

struct LighthouseRowUi
{
    QLabel *nameLabel = nullptr;
    QLabel *powerIndicator = nullptr;
    QLabel *powerStatusLabel = nullptr;
    ui::IntensityBarWidget *bar = nullptr;
    QCheckBox *onOffSwitch = nullptr;
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

protected:
    void closeEvent(QCloseEvent *event) override;
    void performGracefulShutdown();
    static void waitWithBusyDialog(OperationWaitDialog &dialog, const std::function<void()> &work);
    static bool isCameraSessionActive(CameraState state);
    bool anyCameraSessionActive() const;
    bool isStageSessionActive() const;
    bool isLighthouseSessionActive() const;

private:
    QWidget *createStreamTabsPanel();
    QWidget *createSettingsPanel();
    QWidget *createCameraSettingsTab();
    QWidget *createStageSettingsTab();
    QWidget *createLightSettingsTab();
    QWidget *createUr3eSettingsTab();
    QWidget *createCaptureSettingsTab();
    QWidget *createStreamTabPage(const QString &cameraName, LumoCameraUi &cameraUi);
    QWidget *createRgbUr3eStreamTab();
    QGroupBox *createPreviewPane(const QString &title, QLabel *&labelOut);
    QGroupBox *createStreamPane(const QString &title, QWidget *contentWidget);

    QWidget *createLumoCameraGroup(QWidget *parent, LumoCameraUi &ui, LumoSensorKind sensorKind);
    CameraSettings buildCameraSettings(const LumoCameraUi &ui) const;
    static QString shortProfileTabName(const QString &profileName);
    QString profileTabNameForUi(const LumoCameraUi &ui) const;
    void updateCameraTabLabel(const LumoCameraUi &ui);
    void refreshBandCombos(LumoCameraUi &ui);
    static void selectBandComboIndex(QComboBox *combo, int bandIndex);
    static QString resolveBundledCalibrationPackPath(const QString &fileName);
    static QString defaultFx10eCalibrationPackPath();
    static QString defaultSwir3CalibrationPackPath();
    static QString defaultCalibrationPackPathForProfile(const QString &profileName,
                                                        LumoSensorKind sensorKind);
    static QString calibrationPackPath(const LumoCameraUi &ui);
    static void setCalibrationPackDisplay(QLineEdit *edit, const QString &fullPath);
    void syncCalibrationPackToSelectedProfile(LumoCameraUi &ui);

    void appendLog(const QString &message);
    void refreshLumoDeviceLists();
    void refreshStageComPortList();
    QString selectedStagePortName() const;
    void setupStageWorker();
    void setupLighthouseWorker();
    void updateStageConnectionControls(StageState state);
    void updateStageDeviceDisplay(const StageTopology &topology);
    void clearStageDeviceDisplay();
    void onStageStateChanged(StageState state);
    void onStageTopologyChanged(const StageTopology &topology);
    void onStageError(const StageError &error);
    void onLighthouseStateChanged(LighthouseState state);
    void onLighthouseDeviceInfoChanged(const LighthouseDeviceInfo &info);
    void onLighthouseSettingsChanged(const LighthouseSettings &settings);
    void onLighthousePowerStatusChanged(const LighthouseControllerPowerStatus &status);
    void onLighthouseError(const LighthouseError &error);
    void updateStageMotionControls(StageState state);
    void updateStagePositionDisplay(double positionMm);
    void pollStagePosition();
    void updateCaptureCamerasList();
    void updateCaptureCameraPositionRows();
    void updateCapturePositionControls(StageState state);
    void updateCaptureRecorderControls();
    void updateCaptureScanningSpeedControls();
    struct CaptureScanPlan
    {
        double whiteRefStartMm[2] = {0.0, 0.0};
        double brightRefStartMm[2] = {0.0, 0.0};
        double sampleScanStartMm[2] = {0.0, 0.0};
        double sampleScanOriginMm = 0.0;
        double sampleScanTotalDistanceMm = 0.0;
        double sampleScanLengthMm = 0.0;
        double whiteReferenceScanLengthMm = 0.0;
        double operationSpeedMmPerSec = 0.0;
        double recordScanSpeedMmPerSec = 0.0;
        int blackReferenceFrameCount = 0;
    };
    enum class CaptureScanPhase
    {
        Idle,
        MoveToWhiteRefCamera,
        MoveToSampleScanOrigin,
        BlackReference,
        WhiteReferenceScan,
        SampleScan,
    };
    bool buildCaptureScanPlan(CaptureScanPlan &plan, QString &errorMessage) const;
    QString captureSequenceLogPrefix() const;
    void resetCaptureSequenceState();
    void initializeCaptureModeQueue();
    bool confirmCaptureStart(const LighthouseControllerPowerStatus &powerStatus) const;
    void saveCaptureRecordLighthouseBaseline();
    void applyCaptureRecordLighthouseIntensitiesForMode(CaptureIlluminationMode mode);
    void restoreCaptureRecordLighthouseIntensities();
    void startCurrentCaptureMode();
    void beginCaptureModeMotion();
    void completeCaptureModeSequence();
    void startCaptureSequence();
    void requestCaptureAbsoluteMove(double positionMm,
                                    CaptureScanPhase expectedPhaseOnComplete,
                                    double speedMmPerSec = 0.0);
    void onCaptureAbsoluteMoveComplete(bool success);
    void beginCaptureBlackReference();
    void onCaptureBlackReferenceComplete();
    void beginCaptureWhiteReferenceSequence();
    void beginCaptureWhiteReferenceScanForCurrentCamera();
    void beginCaptureWhiteReferenceScan();
    void beginCaptureSampleScan();
    void startCaptureRelativeScan(double distanceMm,
                                  double speedMmPerSec,
                                  CaptureScanPhase capturePhaseOnMoveStart = CaptureScanPhase::Idle);
    void onCaptureRelativeScanComplete();
    void completeCaptureSequence();
    void runCapturePostProcessingIfEnabled();
    void failCaptureSequence(const QString &message);
    void setSelectedCameraShutters(bool open);
    bool selectedCamerasReachedBlackReferenceTarget() const;
    bool validateCaptureRecordMetadata(QString &errorMessage) const;
    bool selectedCaptureCameraIndices(std::vector<std::size_t> &cameraIndices) const;
    std::size_t stageCameraIndexForUi(const LumoCameraUi &ui, std::size_t cameraIndex) const;
    double whiteRefStartMmForStageCamera(const CaptureScanPlan &plan,
                                           CaptureIlluminationMode mode,
                                           std::size_t stageCameraIndex) const;
    double estimatedStageScanPositionMm() const;
    bool isStagePositionWithinScanWindow(double positionMm,
                                         double windowStartMm,
                                         double windowLengthMm) const;
    bool shouldRecordSampleFrameForCamera(std::size_t stageCameraIndex,
                                          double stagePositionMm) const;
    bool shouldRecordWhiteReferenceFrameForCamera(std::size_t stageCameraIndex) const;
    bool selectedCaptureIlluminationModes(std::vector<CaptureIlluminationMode> &modes) const;
    bool isCaptureStageConnected() const;
    bool useStageForCapture() const;
    bool effectiveCaptureIlluminationModes(std::vector<CaptureIlluminationMode> &modes) const;
    QString captureIlluminationFolderName(CaptureIlluminationMode mode) const;
    QString captureCameraFolderName(const LumoCameraUi &ui) const;
    QString captureStreamRelativeRoot(CaptureIlluminationMode mode, const LumoCameraUi &ui) const;
    bool resolveCaptureRecordingIlluminationMode(CaptureIlluminationMode &mode,
                                                 QString &errorMessage) const;
    bool selectedCaptureCameraStreaming(QString &errorMessage) const;
    double closestSelectedCaptureCameraPositionMm(bool *hasSelection) const;
    double closestCaptureCameraPositionMm(bool *hasPosition) const;
    bool buildCaptureWriterSessionConfig(CaptureWriterSessionConfig &config, QString &errorMessage) const;
    bool beginCaptureRawDumpSession(QString &errorMessage);
    void endCaptureRawDumpSession();
    void appendCaptureRecordFrame(const FramePacket &frame);
    void startCapturePreview();
    void startCaptureRecord();
    void stopCaptureRecorder();
    void finishCaptureScan();
    void homeStageAfterCapture();
    void homeStageBeforeCapture();
    void loadHardwareConfig();
    void applyHardwareConfigToUi();
    void loadPersistedUiSettings();
    void savePersistedUiSettings();
    void schedulePersistedUiSettingsSave();
    void applyPersistedCameraProfilesAndBands();
    bool selectDeviceProfileByName(QComboBox *combo, const QString &profileName) const;
    void applyPersistedCameraUiValues(LumoCameraUi &ui);
    void applyPersistedCameraProfileSelection(LumoCameraUi &ui);
    void savePersistedCameraSettings(const LumoCameraUi &ui);
    void connectPersistedSettingsAutosave();
    void updateCameraControls(LumoCameraUi &ui, CameraState state);
    void updateShutterDisplay(LumoCameraUi &ui, bool isOpen);
    void onCameraStateChanged(LumoCameraUi &ui, CameraState state);
    void onShutterStateChanged(LumoCameraUi &ui, bool isOpen);
    void onCameraError(LumoCameraUi &ui, const CameraError &error);
    void onSettingsApplied(LumoCameraUi &ui, const CameraSettingsApplyReport &report);
    void updateDetectorFrame(const FramePacket &frame);
    void onStreamFrame(const FramePacket &frame);
    void updateWaterfallView(LumoCameraUi &ui, const QImage &image);
    void syncWaterfallBands(LumoCameraUi &ui);
    ui::WaterfallProcessor *waterfallProcessorFor(const LumoCameraUi &ui);
    void updateStreamPaneTitles(LumoCameraUi &ui);
    void updateProfilePaneTitles(LumoCameraUi &ui);
    void clearDetectorView(LumoCameraUi &ui);
    void setupWaterfallProcessors();
    void setupProfileProcessors();
    ui::ProfileProcessor *profileProcessorFor(const LumoCameraUi &ui);
    void syncProfileRgbMarkers(LumoCameraUi &ui);
    void onProfileLinesChanged(LumoCameraUi &ui, int spatialIndex, int bandIndex);
    void updateProfilePlots(LumoCameraUi &ui, const ui::ProfileExtraction &profiles);

    QPlainTextEdit *logOutput_ = nullptr;
    QTabWidget *settingsTabs_ = nullptr;
    QTabWidget *streamTabs_ = nullptr;
    QTabWidget *cameraSettingsTabs_ = nullptr;

    static constexpr int kSettingsTabCamera = 0;
    static constexpr int kSettingsTabStage = 1;
    static constexpr int kSettingsTabLight = 2;
    static constexpr int kSettingsTabUr3e = 3;
    static constexpr int kSettingsTabCapture = 4;

    QLabel *lightDaqStatusIndicator_ = nullptr;
    QLabel *lightDaqStatusLabel_ = nullptr;
    QPlainTextEdit *lightDaqInfoDisplay_ = nullptr;
    QPushButton *lightConnectBtn_ = nullptr;
    QPushButton *lightDisconnectBtn_ = nullptr;
    QPushButton *lightRefreshBtn_ = nullptr;
    QGroupBox *lightLightingBox_ = nullptr;
    LighthouseRowUi lighthouseRows_[4];
    QTimer *lightPowerPollTimer_ = nullptr;

    void updateLightConnectionDisplay();
    void updateLightControlsEnabled();
    void updateLighthousePowerDisplay(const LighthouseControllerPowerStatus &status);
    void syncLightUiFromBackend();
    void applyLighthouseSettingsToUi(const LighthouseSettings &settings);
    void applyPersistedLighthouseUiValues();
    LighthouseSettings buildLighthouseConnectDefaults() const;
    void savePersistedLighthouseSettings() const;
    static int lighthousePartnerIndex(int rowIndex);
    void setLighthouseRowIntensity(int rowIndex, int percent);

    QComboBox *stagePortCombo_ = nullptr;
    QComboBox *stageBaudCombo_ = nullptr;
    QPushButton *stageConnectBtn_ = nullptr;
    QPushButton *stageDisconnectBtn_ = nullptr;
    QPlainTextEdit *stageDeviceDisplay_ = nullptr;
    QGroupBox *stageControlBox_ = nullptr;
    QToolButton *stageHomeBtn_ = nullptr;
    QToolButton *stageToStartBtn_ = nullptr;
    QToolButton *stageBackBtn_ = nullptr;
    QToolButton *stageStopBtn_ = nullptr;
    QToolButton *stageForwardBtn_ = nullptr;
    QToolButton *stageToEndBtn_ = nullptr;
    QDoubleSpinBox *stageAbsolutePositionSpin_ = nullptr;
    QToolButton *stageAbsoluteMoveBtn_ = nullptr;
    ui::StageAxisWidget *stageAxisWidget_ = nullptr;
    QTimer *stagePositionTimer_ = nullptr;
    bool stagePositionPollInFlight_ = false;

    enum class StageHomingKind
    {
        None,
        BeforeDisconnect,
        Localization,
        Simple,
        BeforeCapture,
        AfterCapture,
    };
    StageHomingKind stageHomingKind_ = StageHomingKind::None;
    std::unique_ptr<StageWorker> stageWorker_;
    std::unique_ptr<LighthouseWorker> lighthouseWorker_;

    QLineEdit *captureDatasetEdit_ = nullptr;
    QLineEdit *captureSaveFolderEdit_ = nullptr;
    QPushButton *captureSaveFolderBrowseBtn_ = nullptr;
    QLineEdit *captureOperatorEdit_ = nullptr;
    QPlainTextEdit *captureDescriptionEdit_ = nullptr;
    enum class CaptureRecorderMode
    {
        Idle,
        Preview,
        Record,
    };
    CaptureRecorderMode captureRecorderMode_ = CaptureRecorderMode::Idle;
    CaptureScanPhase captureScanPhase_ = CaptureScanPhase::Idle;
    CaptureScanPlan captureScanPlan_;
    CaptureScanPhase captureMoveCompletePhase_ = CaptureScanPhase::Idle;
    std::vector<std::size_t> captureWhiteRefCameraQueue_;
    std::size_t captureWhiteRefQueueIndex_ = 0;
    double captureScanOriginPositionMm_ = 0.0;
    double captureActiveScanDistanceMm_ = 0.0;
    bool captureScanTimingActive_ = false;
    QElapsedTimer captureScanElapsed_;
    CaptureIlluminationMode captureRecordingIlluminationMode_ = CaptureIlluminationMode::Reflectance;
    std::vector<CaptureIlluminationMode> capturePendingIlluminationModes_;
    std::size_t captureCurrentModeIndex_ = 0;
    bool captureLighthouseBaselineSaved_ = false;
    int captureSavedReflectancePercent_ = 0;
    int captureSavedTransmittancePercent_ = 0;
    std::array<int, 2> captureBlackRefFramesCollected_ = {0, 0};
    QTimer *captureScanTimer_ = nullptr;
    std::unique_ptr<CaptureWriterWorker> captureWriterWorker_;
    std::unique_ptr<CapturePostProcessorWorker> capturePostProcessorWorker_;
    CaptureWriterSessionSummary lastEndedCaptureSessionSummary_;
    QPushButton *captureRecorderStopBtn_ = nullptr;
    QPushButton *captureRecorderPreviewBtn_ = nullptr;
    QPushButton *captureRecorderRecordBtn_ = nullptr;
    QCheckBox *captureReflectanceCheck_ = nullptr;
    QCheckBox *captureTransmittanceCheck_ = nullptr;
    QGroupBox *captureCamerasBox_ = nullptr;
    QLabel *captureCamerasEmptyLabel_ = nullptr;
    QCheckBox *captureCamera1Check_ = nullptr;
    QCheckBox *captureCamera2Check_ = nullptr;
    QWidget *captureCameraPositionRows_[2] = {nullptr, nullptr};
    QDoubleSpinBox *captureCameraPositionSpins_[2] = {nullptr, nullptr};
    QDoubleSpinBox *captureTargetLengthSpin_ = nullptr;
    QDoubleSpinBox *captureScanningSpeedSpin_ = nullptr;
    QCheckBox *captureScanningSpeedAutoCheck_ = nullptr;
    QCheckBox *captureUseStageForRecordingCheck_ = nullptr;
    QGroupBox *capturePreprocessingBox_ = nullptr;
    QCheckBox *capturePreprocessAfterScanCheck_ = nullptr;
    QCheckBox *captureSaveFfcImageCheck_ = nullptr;
    QWidget *capturePositionContent_ = nullptr;
    QTimer *settingsSaveTimer_ = nullptr;
    QString persistedStagePort_;
    std::unique_ptr<CameraCoordinator> coordinator_;
    std::unique_ptr<ui::WaterfallProcessor> waterfallProcessor1_;
    std::unique_ptr<ui::WaterfallProcessor> waterfallProcessor2_;
    std::unique_ptr<ui::ProfileProcessor> profileProcessor1_;
    std::unique_ptr<ui::ProfileProcessor> profileProcessor2_;

    LumoCameraUi camera1Ui_;
    LumoCameraUi camera2Ui_;
    bool gracefulShutdownDone_ = false;
    bool performingGracefulShutdown_ = false;

private slots:
    void onCameraSettingsTabChanged(int index);
    void onSettingsTabChanged(int index);
    void onStreamTabChanged(int index);
};
