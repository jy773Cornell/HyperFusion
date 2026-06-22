// Background worker: converts streamed BIL frames to detector QImages off the GUI thread.
#pragma once

#include "backend/CameraTypes.hpp"

#include <QImage>

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>

namespace ui
{
class DetectorFrameProcessor
{
public:
    using ImageReadyCallback = std::function<void(QImage image)>;

    DetectorFrameProcessor();
    ~DetectorFrameProcessor();

    DetectorFrameProcessor(const DetectorFrameProcessor &) = delete;
    DetectorFrameProcessor &operator=(const DetectorFrameProcessor &) = delete;

    void start();
    void stop();

    void setImageReadyCallback(ImageReadyCallback callback);

    /// Latest-wins queue (depth 1).
    void submitFrame(SharedFramePacket frame);

private:
    void threadLoop();

    ImageReadyCallback imageReadyCallback_;
    std::mutex callbackMutex_;

    std::mutex queueMutex_;
    std::condition_variable queueCv_;
    SharedFramePacket pendingFrame_;
    QImage reuseImage_;

    std::thread workerThread_;
    std::atomic<bool> running_{false};
};
} // namespace ui
