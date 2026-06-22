// Thread-safe measured frame-rate tracker for camera stream ingress (not GUI refresh).
#pragma once

#include <chrono>
#include <mutex>

namespace ui
{
class StreamFpsTracker
{
public:
    /// Count one delivered camera frame (call from the camera worker thread).
    void noteFrame();

    /// Last measured rate over a ~500 ms window; 0 when idle.
    [[nodiscard]] double fps() const;

    void reset();

private:
    mutable std::mutex mutex_;
    int framesInWindow_ = 0;
    std::chrono::steady_clock::time_point windowStart_{};
    double fps_ = 0.0;
    static constexpr int kWindowMs = 500;
};

inline void StreamFpsTracker::noteFrame()
{
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = std::chrono::steady_clock::now();
    if (framesInWindow_ == 0)
        windowStart_ = now;

    ++framesInWindow_;

    const auto elapsedMs =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - windowStart_).count();
    if (elapsedMs >= kWindowMs && elapsedMs > 0)
    {
        fps_ = static_cast<double>(framesInWindow_) * 1000.0 / static_cast<double>(elapsedMs);
        framesInWindow_ = 0;
        windowStart_ = now;
    }
}

inline double StreamFpsTracker::fps() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return fps_;
}

inline void StreamFpsTracker::reset()
{
    std::lock_guard<std::mutex> lock(mutex_);
    framesInWindow_ = 0;
    windowStart_ = {};
    fps_ = 0.0;
}
} // namespace ui
