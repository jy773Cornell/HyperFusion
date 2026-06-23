// Coalesced dual-camera stream display pipeline: frame ingress, worker dispatch, capped GUI refresh.
#pragma once

#include "backend/CameraTypes.hpp"
#include "frontend/processing/BilProfileExtractor.hpp"
#include "frontend/processing/DetectorFrameProcessor.hpp"
#include "frontend/processing/ProfileProcessor.hpp"
#include "frontend/processing/WaterfallProcessor.hpp"
#include "frontend/streaming/FrameCoalescer.hpp"

#include <QImage>
#include <QObject>
#include <QTimer>

#include <array>
#include <atomic>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <vector>

struct LumoCameraUi;

namespace ui
{
enum class WaterfallDisplayTarget
{
    None,
    StreamTab,
    CaptureTab,
};

struct CameraStreamDisplayHooks
{
    std::function<void(LumoCameraUi &, const QImage &detectorImage)> applyDetectorImage;
    std::function<void(LumoCameraUi &, QImage waterfallImage, WaterfallDisplayTarget target)>
        applyWaterfallImage;
    std::function<void(LumoCameraUi &, const ProfileExtraction &profiles)> applyProfiles;
    /// FX10e/SWIR stream tab active for this camera (detector/profile panes).
    std::function<bool(std::size_t cameraIndex)> isStreamTabVisible;
};

class CameraStreamPipeline final : public QObject
{
    Q_OBJECT

public:
    explicit CameraStreamPipeline(QObject *parent = nullptr);

    void setCameraUi(std::size_t cameraIndex, LumoCameraUi *ui);
    void setDisplayHooks(CameraStreamDisplayHooks hooks);

    WaterfallProcessor *waterfallProcessor(std::size_t cameraIndex);
    ProfileProcessor *profileProcessor(std::size_t cameraIndex);

    void start();
    void stop();

    /// Called from the camera stream thread. Waterfall stacks every frame; GUI paint is ~30 Hz.
    void ingestFrame(FramePacket frame);

    /// Detector/profile display refresh interval (default ~30 Hz).
    void setDisplayIntervalMs(int intervalMs);

    /// When true, GUI paint flushes are skipped (e.g. during blocking camera connect).
    void setDisplayPaused(bool paused);

private slots:
    void dispatchPendingFrames();
    void flushDisplay();
    void flushWaterfallDisplay();
    void applyWaterfallDisplay(std::size_t cameraIndex,
                               QImage image,
                               WaterfallDisplayTarget target);

private:
    struct PendingDisplay
    {
        QImage detectorImage;
        QImage waterfallImage;
        ProfileExtraction profiles;
        bool detectorDirty = false;
        bool waterfallDirty = false;
        bool profileDirty = false;
        std::vector<double> wavelengthAxisCache;
        int wavelengthAxisBandCount = -1;
    };

    static std::size_t cameraIndexFor(const FramePacket &frame);

    void wireProcessors(std::size_t cameraIndex);
    void markDispatchNeeded();

    CameraStreamDisplayHooks hooks_;
    FrameCoalescer coalescer_;
    std::array<LumoCameraUi *, 2> cameraUi_{nullptr, nullptr};

    std::unique_ptr<WaterfallProcessor> waterfallProcessors_[2];
    std::unique_ptr<ProfileProcessor> profileProcessors_[2];
    std::unique_ptr<DetectorFrameProcessor> detectorProcessors_[2];

    QTimer displayTimer_;
    QTimer waterfallDisplayTimer_;
    std::mutex pendingMutex_;
    std::array<PendingDisplay, 2> pending_{};
    std::atomic<bool> dispatchScheduled_{false};
    std::atomic<bool> displayPaused_{false};
};
} // namespace ui
