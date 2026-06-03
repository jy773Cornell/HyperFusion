// Main application window UI layout and control wiring.
#pragma once

#include <QImage>
#include <QMainWindow>

#include <memory>

#include "adapters/lumo/LumoDeviceTypes.hpp"
#include "adapters/lumo/CalpackBandCatalog.hpp"
#include "core/CameraTypes.hpp"
#include "core/StageTypes.hpp"
#include "orchestrator/CameraCoordinator.hpp"
#include "ui/ProfileProcessor.hpp"
#include "ui/WaterfallProcessor.hpp"

class LumoCamera;
class Swir3NiCamera;
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

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

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
    static QString defaultFx10eCalibrationPackPath();
    static QString calibrationPackPath(const LumoCameraUi &ui);
    static void setCalibrationPackDisplay(QLineEdit *edit, const QString &fullPath);

    void appendLog(const QString &message);
    void refreshLumoDeviceLists();
    void refreshStageComPortList();
    QString selectedStagePortName() const;
    void setupStageWorker();
    void updateStageConnectionControls(StageState state);
    void updateStageDeviceDisplay(const StageTopology &topology);
    void clearStageDeviceDisplay();
    void onStageStateChanged(StageState state);
    void onStageTopologyChanged(const StageTopology &topology);
    void onStageError(const StageError &error);
    void updateStageMotionControls(StageState state);
    void updateStagePositionDisplay(double positionMm);
    void pollStagePosition();
    void updateCaptureCamerasList();
    void updateCaptureCameraPositionRows();
    void updateCapturePositionControls(StageState state);
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
    static constexpr int kSettingsTabUr3e = 3;
    static constexpr int kSettingsTabCapture = 4;

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
        Localization,
        Simple,
    };
    StageHomingKind stageHomingKind_ = StageHomingKind::None;
    std::unique_ptr<StageWorker> stageWorker_;

    QLineEdit *captureDatasetEdit_ = nullptr;
    QLineEdit *captureSaveFolderEdit_ = nullptr;
    QPushButton *captureSaveFolderBrowseBtn_ = nullptr;
    QLineEdit *captureOperatorEdit_ = nullptr;
    QPlainTextEdit *captureDescriptionEdit_ = nullptr;
    QPushButton *captureRecorderStopBtn_ = nullptr;
    QPushButton *captureRecorderPreviewBtn_ = nullptr;
    QPushButton *captureRecorderRecordBtn_ = nullptr;
    QGroupBox *captureCamerasBox_ = nullptr;
    QLabel *captureCamerasEmptyLabel_ = nullptr;
    QCheckBox *captureCamera1Check_ = nullptr;
    QCheckBox *captureCamera2Check_ = nullptr;
    QWidget *captureCameraPositionRows_[2] = {nullptr, nullptr};
    QDoubleSpinBox *captureCameraPositionSpins_[2] = {nullptr, nullptr};
    QDoubleSpinBox *captureTargetLengthSpin_ = nullptr;
    QCheckBox *captureStageConnectedCheck_ = nullptr;
    QWidget *capturePositionContent_ = nullptr;
    std::unique_ptr<CameraCoordinator> coordinator_;
    std::unique_ptr<ui::WaterfallProcessor> waterfallProcessor1_;
    std::unique_ptr<ui::WaterfallProcessor> waterfallProcessor2_;
    std::unique_ptr<ui::ProfileProcessor> profileProcessor1_;
    std::unique_ptr<ui::ProfileProcessor> profileProcessor2_;

    LumoCameraUi camera1Ui_;
    LumoCameraUi camera2Ui_;

private slots:
    void onCameraSettingsTabChanged(int index);
    void onSettingsTabChanged(int index);
    void onStreamTabChanged(int index);
};
