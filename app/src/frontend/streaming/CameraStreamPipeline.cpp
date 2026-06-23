// Dual-camera stream pipeline: coalesced ingress, background workers, throttled GUI refresh.
#include "frontend/streaming/CameraStreamPipeline.hpp"

#include "frontend/widgets/LumoCameraUi.hpp"

#include <QMetaObject>

#include <algorithm>

namespace ui
{
namespace
{
constexpr int kDefaultDisplayIntervalMs = 33;
constexpr int kWaterfallDisplayIntervalMs = 33;
} // namespace

CameraStreamPipeline::CameraStreamPipeline(QObject *parent) : QObject(parent)
{
    for (std::size_t i = 0; i < 2; ++i)
    {
        waterfallProcessors_[i] = std::make_unique<WaterfallProcessor>();
        profileProcessors_[i] = std::make_unique<ProfileProcessor>();
        detectorProcessors_[i] = std::make_unique<DetectorFrameProcessor>();
    }

    displayTimer_.setInterval(kDefaultDisplayIntervalMs);
    connect(&displayTimer_, &QTimer::timeout, this, &CameraStreamPipeline::flushDisplay);

    waterfallDisplayTimer_.setInterval(kWaterfallDisplayIntervalMs);
    connect(&waterfallDisplayTimer_, &QTimer::timeout, this, &CameraStreamPipeline::flushWaterfallDisplay);
}

void CameraStreamPipeline::setCameraUi(const std::size_t cameraIndex, LumoCameraUi *ui)
{
    if (cameraIndex < cameraUi_.size())
        cameraUi_[cameraIndex] = ui;
}

void CameraStreamPipeline::setDisplayHooks(const CameraStreamDisplayHooks hooks)
{
    hooks_ = std::move(hooks);
}

WaterfallProcessor *CameraStreamPipeline::waterfallProcessor(const std::size_t cameraIndex)
{
    return cameraIndex < 2 ? waterfallProcessors_[cameraIndex].get() : nullptr;
}

ProfileProcessor *CameraStreamPipeline::profileProcessor(const std::size_t cameraIndex)
{
    return cameraIndex < 2 ? profileProcessors_[cameraIndex].get() : nullptr;
}

void CameraStreamPipeline::start()
{
    for (std::size_t i = 0; i < 2; ++i)
    {
        wireProcessors(i);
        waterfallProcessors_[i]->start();
        profileProcessors_[i]->start();
        detectorProcessors_[i]->start();
    }

    displayTimer_.start();
    waterfallDisplayTimer_.start();
}

void CameraStreamPipeline::stop()
{
    displayTimer_.stop();
    waterfallDisplayTimer_.stop();

    for (std::size_t i = 0; i < 2; ++i)
    {
        detectorProcessors_[i]->stop();
        profileProcessors_[i]->stop();
        waterfallProcessors_[i]->stop();
    }
}

void CameraStreamPipeline::setDisplayIntervalMs(const int intervalMs)
{
    const int clamped = std::max(16, intervalMs);
    displayTimer_.setInterval(clamped);
    waterfallDisplayTimer_.setInterval(clamped);
}

void CameraStreamPipeline::setDisplayPaused(const bool paused)
{
    displayPaused_.store(paused, std::memory_order_release);
}

std::size_t CameraStreamPipeline::cameraIndexFor(const FramePacket &frame)
{
    return frame.source == CameraBackendId::Camera1 ? 0U : 1U;
}

void CameraStreamPipeline::wireProcessors(const std::size_t cameraIndex)
{
    LumoCameraUi *ui = cameraUi_[cameraIndex];
    if (ui == nullptr)
        return;

    detectorProcessors_[cameraIndex]->setImageReadyCallback(
        [this, cameraIndex](QImage image) {
            std::lock_guard<std::mutex> lock(pendingMutex_);
            pending_[cameraIndex].detectorImage = std::move(image);
            pending_[cameraIndex].detectorDirty = true;
        });

    waterfallProcessors_[cameraIndex]->setImageReadyCallback(
        [this, cameraIndex](QImage image) {
            std::lock_guard<std::mutex> lock(pendingMutex_);
            pending_[cameraIndex].waterfallImage = std::move(image);
            pending_[cameraIndex].waterfallDirty = true;
        });

    profileProcessors_[cameraIndex]->setProfilesReadyCallback(
        [this, cameraIndex](ProfileExtraction profiles) {
            std::lock_guard<std::mutex> lock(pendingMutex_);
            pending_[cameraIndex].profiles = std::move(profiles);
            pending_[cameraIndex].profileDirty = true;
        });
}

void CameraStreamPipeline::markDispatchNeeded()
{
    if (dispatchScheduled_.exchange(true, std::memory_order_acq_rel))
        return;

    QMetaObject::invokeMethod(this, &CameraStreamPipeline::dispatchPendingFrames, Qt::QueuedConnection);
}

void CameraStreamPipeline::ingestFrame(FramePacket frame)
{
    const auto packet = std::make_shared<FramePacket>(std::move(frame));
    const std::size_t cameraIndex = cameraIndexFor(*packet);

    if (cameraIndex < 2)
        waterfallProcessors_[cameraIndex]->submitFrame(packet);

    if (coalescer_.submit(packet))
        markDispatchNeeded();
}

void CameraStreamPipeline::dispatchPendingFrames()
{
    dispatchScheduled_.store(false, std::memory_order_release);

    const std::uint32_t dirtyMask = coalescer_.takeDirtyMask();
    for (std::size_t cameraIndex = 0; cameraIndex < 2; ++cameraIndex)
    {
        if ((dirtyMask & (1U << cameraIndex)) == 0U)
            continue;

        SharedFramePacket latest = coalescer_.takeLatest(cameraIndex);
        if (!latest)
            continue;

        detectorProcessors_[cameraIndex]->submitFrame(latest);
        profileProcessors_[cameraIndex]->submitFrame(latest);
    }
}

void CameraStreamPipeline::applyWaterfallDisplay(const std::size_t cameraIndex,
                                                 QImage image,
                                                 const WaterfallDisplayTarget target)
{
    Q_UNUSED(target);
    LumoCameraUi *ui = cameraIndex < cameraUi_.size() ? cameraUi_[cameraIndex] : nullptr;
    if (ui == nullptr || image.isNull() || !hooks_.applyWaterfallImage)
        return;

    hooks_.applyWaterfallImage(*ui, std::move(image), WaterfallDisplayTarget::StreamTab);
}

void CameraStreamPipeline::flushWaterfallDisplay()
{
    if (displayPaused_.load(std::memory_order_acquire))
        return;

    std::array<QImage, 2> localImages{};
    std::array<bool, 2> localDirty{};

    {
        std::lock_guard<std::mutex> lock(pendingMutex_);
        for (std::size_t cameraIndex = 0; cameraIndex < 2; ++cameraIndex)
        {
            if (!pending_[cameraIndex].waterfallDirty)
                continue;

            localImages[cameraIndex] = std::move(pending_[cameraIndex].waterfallImage);
            pending_[cameraIndex].waterfallDirty = false;
            localDirty[cameraIndex] = true;
        }
    }

    for (std::size_t cameraIndex = 0; cameraIndex < 2; ++cameraIndex)
    {
        if (!localDirty[cameraIndex] || localImages[cameraIndex].isNull())
            continue;

        applyWaterfallDisplay(cameraIndex, std::move(localImages[cameraIndex]), WaterfallDisplayTarget::StreamTab);
    }
}

void CameraStreamPipeline::flushDisplay()
{
    if (displayPaused_.load(std::memory_order_acquire))
        return;

    std::array<PendingDisplay, 2> localPending{};
    {
        std::lock_guard<std::mutex> lock(pendingMutex_);
        localPending = pending_;
        for (auto &slot : pending_)
        {
            slot.detectorDirty = false;
            slot.profileDirty = false;
        }
    }

    for (std::size_t cameraIndex = 0; cameraIndex < 2; ++cameraIndex)
    {
        LumoCameraUi *ui = cameraUi_[cameraIndex];
        if (ui == nullptr)
            continue;

        if (hooks_.isStreamTabVisible && !hooks_.isStreamTabVisible(cameraIndex))
            continue;

        const PendingDisplay &slot = localPending[cameraIndex];

        if (slot.detectorDirty && hooks_.applyDetectorImage && !slot.detectorImage.isNull())
            hooks_.applyDetectorImage(*ui, slot.detectorImage);

        if (slot.profileDirty && hooks_.applyProfiles && slot.profiles.valid)
            hooks_.applyProfiles(*ui, slot.profiles);
    }
}
} // namespace ui
