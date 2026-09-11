// BFS settings tab orchestration: Spinnaker worker, connect, RGB preview.
#pragma once

#include "backend/multiview/BfsCameraTypes.hpp"
#include "frontend/streaming/StreamFpsTracker.hpp"

#include <QObject>
#include <QString>
#include <QTimer>

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

class MainWindow;

namespace hf::bfs
{
class BfsCameraWorker;

class BfsPanelController : public QObject
{
    Q_OBJECT

public:
    explicit BfsPanelController(MainWindow *host, QObject *parent = nullptr);
    ~BfsPanelController() override;

    void initializeWorker();
    void shutdownSync();
    void wireSettingsTabConnections();
    [[nodiscard]] bool isCameraConnected() const;

    /// Thread-safe copy of the latest streamed RGB frame (for Capture Multiview stills).
    [[nodiscard]] bool tryCopyLastFrame(BfsRgbFrame &out) const;
    /// Latest grab counter (0 if none). Safe from the scan worker thread.
    [[nodiscard]] std::uint64_t lastFrameIndex() const;
    /// Block until lastFrame_.frameIndex >= afterIndex + minNewFrames. Safe from a worker thread.
    [[nodiscard]] bool waitForNewerFrame(std::uint64_t afterIndex,
                                         int minNewFrames,
                                         int timeoutMs,
                                         BfsRgbFrame *out = nullptr) const;

private:
    void onRefreshClicked();
    void onConnectClicked();
    void onDisconnectClicked();
    void onCaptureClicked();
    void startStationaryFppBurst(const QString &captureDir);
    void finishStationaryFppBurst(bool ok, const QString &detail);
    void onSettingsEdited();
    void applySettingsFromUi();
    void onStateChanged(BfsCameraState state);
    void onError(const BfsError &error);
    void onDevices(const std::vector<BfsDeviceInfo> &devices);
    void queueFrame(BfsRgbFrame frame);
    void flushPendingFrame();
    void showFrameOnPreview(const BfsRgbFrame &frame);
    [[nodiscard]] BfsCameraSettings settingsFromUi() const;
    void updatePreviewDisconnected();

    MainWindow *host_ = nullptr;
    std::unique_ptr<BfsCameraWorker> worker_;
    bool applyingDevices_ = false;

    std::mutex pendingFrameMutex_;
    std::optional<BfsRgbFrame> pendingFrame_;
    bool frameFlushQueued_ = false;
    mutable std::mutex lastFrameMutex_;
    std::optional<BfsRgbFrame> lastFrame_;
    ui::StreamFpsTracker streamFps_;
    QTimer *settingsApplyTimer_ = nullptr;
    std::thread fppBurstThread_;
    bool fppBurstBusy_ = false;
};
} // namespace hf::bfs
