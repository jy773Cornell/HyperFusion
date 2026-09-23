// UR3e tab orchestration: WSL sidecar lifecycle, robot connect, pose polling, motion.
#pragma once

#include <QObject>
#include <QString>
#include <QStringList>
#include <QVariant>

#include "backend/multiview/Ur3eHemisphereScanReachability.hpp"
#include "backend/multiview/Ur3eServerManager.hpp"
#include "backend/multiview/Ur3eClient.hpp"
#include "backend/multiview/Ur3eCameraTransforms.hpp"

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

class MainWindow;
class QTimer;
class Ur3eExternalControlWaitDialog;

namespace hf::ur3e
{
class Ur3eMoveItManager;
class Ur3eRvizManager;

enum class HemisphereScanPinSet
{
    All,
    ApexOnly,
    RingsOnly,
};

struct HemisphereScanExecuteOptions
{
    /// Empty = motion-only (UR3e Execute / Capture Preview). Non-empty = save BFS stills.
    QString captureOutputDir;
    /// Dwell after arriving at each pin before optional capture (ms).
    int stabilizeMs = 2000;
    /// When true (Capture-driven Multiview), skip the standalone UR3e summary dialog on success —
    /// Capture merges the scan stats into Recording complete.
    bool suppressUiSummary = false;
    HemisphereScanPinSet pinSet = HemisphereScanPinSet::All;
    int startFrameIndex = 0;
    bool appendTransformsJson = false;
};

class Ur3ePanelController : public QObject
{
    Q_OBJECT

public:
    explicit Ur3ePanelController(MainWindow *host, QObject *parent = nullptr);
    ~Ur3ePanelController() override;

    void wireSettingsTabConnections();
    void applyHardwareConfigToUi();
    void startSidecarOnLaunch();
    /// Returns false when the user cancels shutdown (e.g. home positioning dialog).
    [[nodiscard]] bool shutdownSync();
    void refreshUi();

    [[nodiscard]] bool isRobotConnected() const { return robotConnected_; }
    [[nodiscard]] bool isSidecarRunning() const;
    /// True when a hemisphere scan plan exists with at least one reachable pose.
    [[nodiscard]] bool isScanPlanReady() const;
    [[nodiscard]] bool isScanExecuting() const { return scanExecuting_; }

    /// Live BFS camera optical in base_link (tool0 ⊗ tool_tcp_*), even when
    /// MoveIt tip hyperfusion_tcp is the DLP (scan_tcp=dlp).
    /// *calibOut* receives flange tool0 + joints when the sidecar provided them.
    [[nodiscard]] bool tryGetLiveOpticalTcpPose(Ur3eScanTcpPose *out,
                                                QString *errorMessage = nullptr,
                                                CalibrationCaptureExtras *calibOut = nullptr) const;

    /// HDMI 26-frame FPP at the current pose (no robot/stage motion).
    /// Call from a worker thread — BFS stills use BlockingQueuedConnection.
    [[nodiscard]] bool captureStationaryFppBurst(const QString &captureDir,
                                                 QString *errorMessage = nullptr);

    /// Start hemisphere execute. Returns false if rejected (busy / no plan / already running).
    bool startHemisphereScanExecute(const HemisphereScanExecuteOptions &options = {});
    void requestStopMotion();

signals:
    void hemisphereScanExecuteFinished(bool ok,
                                       const QString &detail,
                                       int capturedFrameCount,
                                       int successfulPins,
                                       qint64 elapsedMs);

public slots:
    void onSidecarStateChanged(Ur3eServerManager::State state, const QString &detail);

private:
    enum class HomeEnsureContext
    {
        AfterConnect,
        BeforeScanExecute,
        BeforeDisconnect,
        BeforeShutdown,
    };

    enum class HomeEnsurePromptChoice
    {
        Retry,
        ContinueWithoutHoming,
        ProceedAnyway,
        Cancel,
    };

    struct HomeEnsureOutcome
    {
        bool success = false;
        bool alreadyAtHome = false;
        bool cancelled = false;
        bool atHomeVerified = false;
    };

    [[nodiscard]] HomeEnsureOutcome ensureRobotAtHomeSync(HomeEnsureContext context);
    [[nodiscard]] HomeEnsurePromptChoice promptManualHomePositioning(const QString &reason,
                                                                     HomeEnsureContext context);
    [[nodiscard]] HomeEnsurePromptChoice showManualHomePositioningDialog(const QString &reason,
                                                                         HomeEnsureContext context);
    void finishHomeEnsureAfterConnect(const HomeEnsureOutcome &outcome);

    void updateRobotUi();
    void pollJoints();
    void pollJointsSync();
    void applyJointPositions(const std::vector<double> &positionsRad,
                             const QStringList &names,
                             bool syncTargets);
    void joinJointPollThread();
    void applyJointTargets(const std::vector<double> &positionsRad,
                           const QStringList &names = QStringList());
    void setJointPollIntervalMs(int intervalMs);
    void applyConfiguredInitialJointTargets();
    void applyScanHomeJointTargets();
    void beginHomeMotionUi();
    void endHomeMotionUi();
    void scheduleManualTargetPreview();
    void pushManualTargetPreview();
    void syncWorkspaceBoundaryPreview();
    void pushWorkspaceBoundaryToMoveIt();
    void syncTargetsFromCurrent();
    void setBusy(bool busy);

    void onConnectRequested();
    void onDisconnectRequested();
    void onSyncJointsRequested();
    void onMoveRequested();
    void onStopMotionRequested();
    void onStartRvizRequested();
    void onStartMoveItRequested();
    void onPlanHemisphereScanRequested();
    void onExecuteHemisphereScanRequested();
    void onAddSemiFixedRingRequested();
    void refreshSemiFixedPreview();
    bool startSemiFixedScanExecute(const HemisphereScanExecuteOptions &options);
    void onMoveItStateChanged(bool running, const QString &detail);
    void onRvizStateChanged(bool running, const QString &detail);

private slots:
    void scanExecuteSetActivePoint(int pointIndex);
    void scanExecuteMarkCompleted(int pointIndex);
    void scanExecuteMarkFailed(int pointIndex);
    void scanExecuteLogMoving(int step,
                              int total,
                              int pointIndex,
                              const QString &tcpSummary,
                              const QString &targetSummary,
                              const QVariantList &targetPositionsRad);
    void scanExecuteLogArrived(int step,
                               int total,
                               int pointIndex,
                               const QString &arrivedPose,
                               const QString &arrivedJoints,
                               const QVariantList &positionsRad,
                               const QStringList &names);
    void scanExecuteFinish(bool ok,
                           const QString &errorMessage,
                           int executedCount,
                           bool stopped,
                           int capturedFrameCount = 0,
                           qint64 elapsedMs = 0);
    void applyPolledJoints(const QVariantList &positionsRad, const QStringList &names);
    void syncHomeJointTargetSliders();

private:
    void finishConnect(bool ok, const QString &detail);
    void finishDisconnect(bool ok, const QString &detail);
    void finishMove(bool ok, const QString &detail);
    void finishStop(bool ok, const QString &detail);
    void finishScanPlan(const Ur3eHemisphereScanPlan &plan, const QString &errorMessage);
    void finishSemiScanPlan(const Ur3eHemisphereScanPlan &plan,
                            const QString &errorMessage,
                            double intervalDeg,
                            int panDirection,
                            const Ur3eHemisphereScanParams &scanParams);
    void finishScanExecute(bool ok,
                           const QString &detail,
                           int capturedFrameCount,
                           int successfulPins = 0,
                           qint64 elapsedMs = 0,
                           bool stopped = false);
    void tryLoadCachedScanPlan();
    void saveCachedScanPlan();
    void onLoadScanRouteRequested(const QString &routePath);
    void onLoadPlannedRouteAsSemiFixedRequested(const QString &routePath);

    void dismissConnectWaitDialog();
    void applyConnectAsyncStatus(const Ur3eConnectAsyncStatus &status);
    void updateConnectDialogFromSidecarLine(const QString &line);
    void onConnectPollTick();
    void onConnectCountdownTick();
    void onConnectDialogCancelled();
    void onConnectTimedOut();
    void pollDriverPrestartReady();
    void onBoundarySyncTick();

    static constexpr int kPosePollIntervalMs = 500;
    static constexpr int kMotionPollIntervalMs = 100;
    static constexpr int kConnectPollIntervalMs = 1500;
    static constexpr int kDriverReadyPollIntervalMs = 2000;
    // Backup only — sidecar keepalive owns steady-state. Was 5s and thrashed logs.
    static constexpr int kBoundarySyncIntervalMs = 30000;
    static constexpr int kScanCaptureStabilizeMs = 500;

    MainWindow *host_ = nullptr;
    std::unique_ptr<Ur3eServerManager> serverManager_;
    std::unique_ptr<Ur3eMoveItManager> moveItManager_;
    std::unique_ptr<Ur3eRvizManager> rvizManager_;
    bool robotConnected_ = false;
    bool busy_ = false;
    bool scanPlanReady_ = false;
    bool scanExecuting_ = false;
    bool scanExecuteSuppressUiSummary_ = false;
    /// True while post-scan / post-stop retreat-to-home is running — ignore extra Stop presses
    /// so they cannot cancel the home motion mid-flight.
    std::atomic<bool> scanReturningHome_{false};
    bool scanPlanning_ = false;
    bool motionInProgress_ = false;
    Ur3eHemisphereScanPlan plannedScanPlan_;
    std::atomic<bool> stopRequested_{false};
    std::atomic<bool> shutdownRequested_{false};
    std::atomic<int> scanExecuteSessionId_{0};
    std::mutex scanExecuteThreadMutex_;
    std::thread scanExecuteThread_;
    std::atomic<bool> jointPollInFlight_{false};
    std::mutex jointPollThreadMutex_;
    std::thread jointPollThread_;
    Ur3eServerManager::State lastLoggedSidecarState_ = Ur3eServerManager::State::Stopped;

    Ur3eExternalControlWaitDialog *connectWaitDialog_ = nullptr;
    QTimer *connectPollTimer_ = nullptr;
    QTimer *connectCountdownTimer_ = nullptr;
    qint64 connectDeadlineMs_ = 0;
    std::atomic<int> connectSessionId_{0};
    bool connectInProgress_ = false;
    std::atomic<bool> connectPollInFlight_{false};
    QString lastConnectStatusPhase_;
    QString lastConnectStatusMessage_;

    bool driverPrestartReady_ = false;
    QTimer *driverReadyPollTimer_ = nullptr;
    std::atomic<bool> driverReadyPollInFlight_{false};
    std::atomic<bool> manualTargetPreviewInFlight_{false};
    std::atomic<bool> manualTargetPreviewPending_{false};
    std::atomic<bool> boundarySyncInFlight_{false};
    QTimer *boundarySyncTimer_ = nullptr;
};

} // namespace hf::ur3e
