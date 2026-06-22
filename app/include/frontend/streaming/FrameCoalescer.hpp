// Latest-wins frame slots per camera for detector/profile preview (waterfall bypasses this).
#pragma once

#include "backend/CameraTypes.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <mutex>

namespace ui
{
class FrameCoalescer
{
public:
    /// Stores the latest frame for the camera (detector/profile path only).
    bool submit(const SharedFramePacket &frame);

    /// Returns and clears the latest frame for a camera index (0 or 1).
    SharedFramePacket takeLatest(std::size_t cameraIndex);

    /// Bit mask of cameras with pending frames since last clear (bit 0 = cam1, bit 1 = cam2).
    std::uint32_t takeDirtyMask();

private:
    static std::size_t cameraIndexFor(const FramePacket &frame);

    std::mutex mutex_;
    std::array<SharedFramePacket, 2> latest_{};
    std::atomic<std::uint32_t> dirtyMask_{0};
};
} // namespace ui
