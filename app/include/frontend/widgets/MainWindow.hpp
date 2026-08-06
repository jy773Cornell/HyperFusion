// Main application window UI layout and control wiring.
#pragma once

#include "adapters/lumo/LumoDeviceTypes.hpp"
#include "backend/camera/CameraTypes.hpp"
#include "backend/light/LighthouseTypes.hpp"
#include "backend/stage/StageTypes.hpp"
#include "frontend/controllers/CapturePanelController.hpp"
#include "frontend/logging/AppLogSession.hpp"
#include "frontend/widgets/LumoCameraUi.hpp"

#include <QImage>
#include <QMainWindow>

#include <array>
#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

namespace hf::bfs
{
class BfsPanelController;
}

namespace hf::camera
{
class CameraPanelController;
}

namespace hf::light
{
class LightPanelController;
}

namespace hf::settings
{
class UiSettingsController;
}

namespace hf::stage
{
class StagePanelController;
}

namespace hf::ur3e
{
class Ur3ePanelController;
}

class CameraCoordinator;
class LighthouseWorker;
class OperationWaitDialog;
class StageWorker;
class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QGridLayout;
class QGroupBox;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QSpinBox;
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
class BfsCameraSettingsWidget;
class Ur3eJointBarWidget;
class Ur3eScanRoutePlanWidget;
class Ur3eHemisphereScanSettingsWidget;
class WaterfallDisplayWidget;
} // namespace ui

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

    friend class hf::capture::CapturePanelController;
    friend class hf::stage::StagePanelController;
    friend class hf::ur3e::Ur3ePanelController;
    friend class hf::bfs::BfsPanelController;
    friend class hf::light::LightPanelController;
    friend class hf::camera::CameraPanelController;
    friend class hf::settings::UiSettingsController;

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

    [[nodiscard]] hf::capture::CapturePanelController *capturePanel() const;
    [[nodiscard]] hf::stage::StagePanelController *stagePanel() const;
    [[nodiscard]] hf::ur3e::Ur3ePanelController *ur3ePanel() const;
    [[nodiscard]] hf::bfs::BfsPanelController *bfsPanel() const;
    [[nodiscard]] hf::light::LightPanelController *lightPanel() const;
    [[nodiscard]] hf::camera::CameraPanelController *cameraPanel() const;
    [[nodiscard]] hf::settings::UiSettingsController *settingsPanel() const;

    [[nodiscard]] StageWorker *stageWorker() const;
    [[nodiscard]] LighthouseWorker *lighthouseWorker() const;
    [[nodiscard]] CameraCoordinator *coordinator() const;

    [[nodiscard]] bool isCaptureSessionActive() const;

    void appendLog(const QString &message);
    void appendLog(hf::log::Channel channel, const QString &message);

protected:
    void closeEvent(QCloseEvent *event) override;
    /// Returns false when shutdown is aborted (e.g. user cancels UR3e home move).
    [[nodiscard]] bool performGracefulShutdown();
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
    QWidget *createUr3eStreamTab();
    QWidget *createCaptureSettingsTab();
    QWidget *createLumoCameraGroup(QWidget *parent, LumoCameraUi &ui, LumoSensorKind sensorKind);

    // Core layout
    QTabWidget *logTabs_ = nullptr;
    std::array<QPlainTextEdit *, hf::log::channelCount()> logOutputs_{};
    hf::log::SessionLogWriter sessionLog_;
    QTabWidget *settingsTabs_ = nullptr;
    QTabWidget *streamTabs_ = nullptr;
    QTabWidget *cameraSettingsTabs_ = nullptr;

    static constexpr int kSettingsTabCamera = 0;
    static constexpr int kSettingsTabStage = 1;
    static constexpr int kSettingsTabLight = 2;
    static constexpr int kSettingsTabCaptureWhenUr3eEnabled = 4;

    static constexpr int kStreamTabCamera1 = 0;
    static constexpr int kStreamTabCamera2 = 1;
    static constexpr int kStreamTabCaptureWhenUr3eEnabled = 3;

    bool use3dScanning_ = true;
    int ur3eSettingsTabIndex_ = -1;
    int ur3eStreamTabIndex_ = -1;
    int captureSettingsTabIndex_ = kSettingsTabCaptureWhenUr3eEnabled;
    int captureStreamTabIndex_ = kStreamTabCaptureWhenUr3eEnabled;

    [[nodiscard]] bool use3dScanningEnabled() const { return use3dScanning_; }
    [[nodiscard]] int captureStreamTabIndex() const { return captureStreamTabIndex_; }

    // UR3e stream tab
    QWidget *ur3eStreamPage_ = nullptr;
    ui::Ur3eScanRoutePlanWidget *ur3eScanRoutePlanWidget_ = nullptr;
    QLabel *ur3eRgbPreviewLabel_ = nullptr;

    // Capture stream tab
    QWidget *captureStreamPage_ = nullptr;
    QGridLayout *captureStreamGrid_ = nullptr;
    QLabel *captureStreamEmptyLabel_ = nullptr;
    QGroupBox *captureWaterfallPanes_[2] = {nullptr, nullptr};
    ui::WaterfallDisplayWidget *captureWaterfallViews_[2] = {nullptr, nullptr};
    QGroupBox *captureBfsPreviewPane_ = nullptr;
    QLabel *captureBfsPreviewLabel_ = nullptr;

    // Light settings tab
    QLabel *lightDaqStatusIndicator_ = nullptr;
    QLabel *lightDaqStatusLabel_ = nullptr;
    QPlainTextEdit *lightDaqInfoDisplay_ = nullptr;
    QPushButton *lightConnectBtn_ = nullptr;
    QPushButton *lightDisconnectBtn_ = nullptr;
    QPushButton *lightRefreshBtn_ = nullptr;
    QGroupBox *lightConnectionBox_ = nullptr;
    QGroupBox *lightLightingBox_ = nullptr;
    LighthouseRowUi lighthouseRows_[4];

    // Stage settings tab
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

    // Capture settings tab
    QLineEdit *captureDatasetEdit_ = nullptr;
    QLineEdit *captureSaveFolderEdit_ = nullptr;
    QPushButton *captureSaveFolderBrowseBtn_ = nullptr;
    QLineEdit *captureOperatorEdit_ = nullptr;
    QPlainTextEdit *captureDescriptionEdit_ = nullptr;
    QPushButton *captureRecorderStopBtn_ = nullptr;
    QPushButton *captureRecorderPreviewBtn_ = nullptr;
    QPushButton *captureRecorderRecordBtn_ = nullptr;
    QLabel *captureRecorderStatusIndicator_ = nullptr;
    QLabel *captureRecorderStatusLabel_ = nullptr;
    QLabel *captureRecorderCameraStatusLabels_[2] = {nullptr, nullptr};
    QTimer *captureRecorderStatusTimer_ = nullptr;
    QCheckBox *captureReflectanceCheck_ = nullptr;
    QCheckBox *captureTransmittanceCheck_ = nullptr;
    QGroupBox *captureModesBox_ = nullptr;
    QGroupBox *captureMetadataBox_ = nullptr;
    QGroupBox *capturePositionBox_ = nullptr;
    QGroupBox *captureCamerasBox_ = nullptr;
    QTabWidget *scanningSettingsTabs_ = nullptr;
    ui::BfsCameraSettingsWidget *bfsCameraSettings_ = nullptr;
    QWidget *ur3eSettingsPage_ = nullptr;
    QLineEdit *ur3eRobotIpEdit_ = nullptr;
    QPushButton *ur3eConnectBtn_ = nullptr;
    QPushButton *ur3eDisconnectBtn_ = nullptr;
    static constexpr int kUr3eJointCount = 6;
    ui::Ur3eJointBarWidget *ur3eJointBars_[kUr3eJointCount] = {nullptr, nullptr, nullptr,
                                                               nullptr, nullptr, nullptr};
    QPushButton *ur3eMoveBtn_ = nullptr;
    QPushButton *ur3eStopMotionBtn_ = nullptr;
    QPushButton *ur3eSyncJointsBtn_ = nullptr;
    QPushButton *ur3eStartRvizBtn_ = nullptr;
    QPushButton *ur3eStartMoveItBtn_ = nullptr;
    ui::Ur3eHemisphereScanSettingsWidget *ur3eHemisphereScanSettings_ = nullptr;
    QTimer *ur3ePosePollTimer_ = nullptr;
    QLabel *captureCamerasEmptyLabel_ = nullptr;
    QCheckBox *captureCamera1Check_ = nullptr;
    QCheckBox *captureCamera2Check_ = nullptr;
    QCheckBox *captureBfsCheck_ = nullptr;
    QCheckBox *capture3dRgbCheck_ = nullptr;
    QCheckBox *captureDualCameraAutoCheck_ = nullptr;
    QDoubleSpinBox *captureTargetLengthSpin_ = nullptr;
    QDoubleSpinBox *captureScanningSpeedSpin_ = nullptr;
    QCheckBox *captureScanningSpeedAutoCheck_ = nullptr;
    QCheckBox *captureUseStageForRecordingCheck_ = nullptr;
    QGroupBox *capturePreprocessingBox_ = nullptr;
    QCheckBox *capturePreprocessAfterScanCheck_ = nullptr;
    QCheckBox *captureSaveFfcImageCheck_ = nullptr;
    QCheckBox *captureRunGsamCheck_ = nullptr;
    QCheckBox *captureRunHfFusionCheck_ = nullptr;
    QLabel *captureGsamServerStatusLabel_ = nullptr;
    QLineEdit *captureGsamPromptEdit_ = nullptr;
    QSpinBox *captureGsamSampleCountSpin_ = nullptr;
    QWidget *capturePositionContent_ = nullptr;
    QString persistedStagePort_;

    // Camera UI state
    LumoCameraUi camera1Ui_;
    LumoCameraUi camera2Ui_;

    bool gracefulShutdownDone_ = false;
    bool performingGracefulShutdown_ = false;

    std::unique_ptr<hf::stage::StagePanelController> stagePanel_;
    std::unique_ptr<hf::ur3e::Ur3ePanelController> ur3ePanel_;
    std::unique_ptr<hf::bfs::BfsPanelController> bfsPanel_;
    std::unique_ptr<hf::light::LightPanelController> lightPanel_;
    std::unique_ptr<hf::camera::CameraPanelController> cameraPanel_;
    std::unique_ptr<hf::settings::UiSettingsController> settingsPanel_;
    std::unique_ptr<hf::capture::CapturePanelController> capturePanel_;

private slots:
    void onSettingsTabChanged(int index);
};
