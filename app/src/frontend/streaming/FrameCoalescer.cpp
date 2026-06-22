// Latest-wins frame slot implementation.
#include "frontend/streaming/FrameCoalescer.hpp"

namespace ui
{
std::size_t FrameCoalescer::cameraIndexFor(const FramePacket &frame)
{
    return frame.source == CameraBackendId::Camera1 ? 0U : 1U;
}

bool FrameCoalescer::submit(const SharedFramePacket &frame)
{
    if (!frame)
        return false;

    const std::size_t index = cameraIndexFor(*frame);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_[index] = frame;
    }
    dirtyMask_.fetch_or(1U << index, std::memory_order_release);
    return true;
}

SharedFramePacket FrameCoalescer::takeLatest(const std::size_t cameraIndex)
{
    if (cameraIndex >= latest_.size())
        return {};

    std::lock_guard<std::mutex> lock(mutex_);
    return std::move(latest_[cameraIndex]);
}

std::uint32_t FrameCoalescer::takeDirtyMask()
{
    return dirtyMask_.exchange(0U, std::memory_order_acq_rel);
}
} // namespace ui
