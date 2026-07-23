// BFS settings tab orchestration: Spinnaker worker, connect, RGB preview.
#pragma once

#include "backend/3dscanning/BfsCameraTypes.hpp"

#include <QObject>

#include <memory>
#include <mutex>
#include <optional>

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

private:
    void onRefreshClicked();
    void onConnectClicked();
    void onDisconnectClicked();
    void onCaptureClicked();
    void onSettingsEdited();
    void onStateChanged(BfsCameraState state);
    void onError(const BfsError &error);
    void onDevices(const std::vector<BfsDeviceInfo> &devices);
    void queueFrame(BfsRgbFrame frame);
    void flushPendingFrame();
    void showFrameOnPreview(const BfsRgbFrame &frame);
    [[nodiscard]] BfsCameraSettings settingsFromUi() const;
    void updatePreviewDisconnected();
    void focusBfsStreamTab();

    MainWindow *host_ = nullptr;
    std::unique_ptr<BfsCameraWorker> worker_;
    bool applyingDevices_ = false;

    std::mutex pendingFrameMutex_;
    std::optional<BfsRgbFrame> pendingFrame_;
    bool frameFlushQueued_ = false;
    std::optional<BfsRgbFrame> lastFrame_;
};
} // namespace hf::bfs
