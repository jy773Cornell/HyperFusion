// Camera tab orchestration: coordinator, stream pipeline, profiles, and live display.
#pragma once

#include "adapters/lumo/LumoDeviceTypes.hpp"
#include "backend/CameraTypes.hpp"
#include "backend/SdkLifecycleRunner.hpp"
#include "frontend/processing/ProfileProcessor.hpp"
#include "frontend/processing/WaterfallProcessor.hpp"
#include "frontend/streaming/CameraStreamPipeline.hpp"
#include "frontend/streaming/StreamFpsTracker.hpp"
#include "frontend/widgets/LumoCameraUi.hpp"

#include <QImage>
#include <QObject>
#include <QString>

#include <QElapsedTimer>

#include <array>
#include <memory>

class CameraCoordinator;
class MainWindow;
class OperationWaitDialog;
class QTimer;

namespace hf::camera
{
enum class CameraWaitOperation
{
    Connecting,
    ApplyingSettings,
    Disconnecting,
};

class CameraPanelController : public QObject
{
    Q_OBJECT

public:
    static constexpr int kFx10eConnectWaitTimeoutMs = 30000;
    static constexpr int kSwir3ConnectWaitTimeoutMs = 90000;
    static constexpr int kFx10eAcquisitionTimeoutMs = 5000;
    static constexpr int kSwir3AcquisitionTimeoutMs = 60000;

    explicit CameraPanelController(MainWindow *host, QObject *parent = nullptr);

    void initializeCameras();
    void shutdownCoordinatorSync();
    void stopStreamPipeline();

    [[nodiscard]] CameraCoordinator *coordinator() const;
    static bool isSessionActive(CameraState state);

    CameraSettings buildSettings(const LumoCameraUi &ui) const;
    QString profileTabNameForUi(const LumoCameraUi &ui) const;
    static QString shortProfileTabName(const QString &profileName);
    static QString calibrationPackPath(const LumoCameraUi &ui);
    static void setCalibrationPackDisplay(class QLineEdit *edit, const QString &fullPath);
    static QString defaultCalibrationPackPathForProfile(const QString &profileName,
                                                        LumoSensorKind sensorKind);
    static QString defaultFx10eCalibrationPackPath();
    static QString defaultSwir3CalibrationPackPath();
    /// Resolves UI/saved path and updates the line edit when a bundled default is used.
    QString ensureCalibrationPackResolved(LumoCameraUi &ui);
    static void selectBandComboIndex(class QComboBox *combo, int bandIndex);

    void updateTabLabel(const LumoCameraUi &ui);
    void refreshBandCombos(LumoCameraUi &ui);
    void syncCalibrationPackToSelectedProfile(LumoCameraUi &ui);
    void refreshDeviceLists();
    void updateCameraControls(LumoCameraUi &ui, CameraState state);
    void updateShutterDisplay(LumoCameraUi &ui, bool isOpen);

    void setupStreamPipeline();
    void onStreamFrame(const SharedFramePacket &frame);
    ui::WaterfallProcessor *waterfallProcessorFor(const LumoCameraUi &ui);
    ui::ProfileProcessor *profileProcessorFor(const LumoCameraUi &ui);
    void syncWaterfallBands(LumoCameraUi &ui);
    void syncWaterfallMaxLines(LumoCameraUi &ui);
    void wireWaterfallPaneResizeHandlers();
    void syncProfileRgbMarkers(LumoCameraUi &ui);
    void onProfileLinesChanged(LumoCameraUi &ui, int spatialIndex, int bandIndex);
    void applyDetectorDisplay(LumoCameraUi &ui, const QImage &image);
    void applyWaterfallDisplay(LumoCameraUi &ui, QImage image, ui::WaterfallDisplayTarget target);
    void applyProfileDisplay(LumoCameraUi &ui, const ui::ProfileExtraction &profiles);
    void refreshWaterfallDisplayTargets();
    void updateStreamPaneTitles(LumoCameraUi &ui);
    void updateProfilePaneTitles(LumoCameraUi &ui);
    void clearDetectorView(LumoCameraUi &ui);

    void showCameraOperationWait(std::size_t cameraIndex, CameraWaitOperation operation);
    void dismissCameraOperationWait(std::size_t cameraIndex);
    void dismissAllCameraOperationWaits();

    [[nodiscard]] bool anySessionActive() const;

public slots:
    void onCameraStateChanged(LumoCameraUi &ui, CameraState state);
    void onShutterStateChanged(LumoCameraUi &ui, bool isOpen);
    void onCameraError(LumoCameraUi &ui, const CameraError &error);
    void onSettingsApplied(LumoCameraUi &ui, const CameraSettingsApplyReport &report);

private:
    struct CameraOperationWait
    {
        OperationWaitDialog *dialog = nullptr;
        CameraWaitOperation operation = CameraWaitOperation::Connecting;
        bool active = false;
    };

    void updateProfilePlots(LumoCameraUi &ui, const ui::ProfileExtraction &profiles);
    void updateWaterfallView(LumoCameraUi &ui, const QImage &image, ui::WaterfallDisplayTarget target);
    void publishWaterfallToAllViews(std::size_t cameraIndex, std::shared_ptr<const QImage> image);
    void republishGlobalWaterfallViews(std::size_t cameraIndex);
    void onCameraOperationWaitTimedOut(std::size_t cameraIndex);
    void finishAutoStreamStartup(std::size_t cameraIndex);
    void setConnectDisplayPaused(bool paused);
    LumoCameraUi *cameraUiForIndex(std::size_t cameraIndex);
    void noteStreamFrame(const FramePacket &frame);
    void refreshAcquisitionFpsOverlays();
    void refreshSessionUptimeLabels();
    void syncSessionUptimeClock(LumoCameraUi &ui, CameraState state);
    [[nodiscard]] bool anyCameraOperationWaitActive() const;
    void pollSdkFrameRates();
    void syncStreamDisplayLoad();

    MainWindow *host_ = nullptr;
    std::unique_ptr<CameraCoordinator> coordinator_;
    std::unique_ptr<SdkLifecycleRunner> sdkLifecycleRunner_;
    std::unique_ptr<ui::CameraStreamPipeline> streamPipeline_;
    ui::StreamFpsTracker streamFpsTrackers_[2];
    std::array<double, 2> sdkFrameRateHz_{0.0, 0.0};
    std::array<QElapsedTimer, 2> sessionUptimeTimers_{};
    std::array<bool, 2> sessionUptimeActive_{false, false};
    std::array<std::shared_ptr<const QImage>, 2> globalWaterfallImages_{};
    std::array<CameraOperationWait, 2> operationWaits_{};
    QTimer *connectTimeoutTimer_ = nullptr;
    std::size_t connectTimeoutCameraIndex_ = 0;
    int connectDisplayPauseDepth_ = 0;
    QTimer *fpsOverlayTimer_ = nullptr;
    QTimer *sdkFrameRatePollTimer_ = nullptr;
};
} // namespace hf::camera
