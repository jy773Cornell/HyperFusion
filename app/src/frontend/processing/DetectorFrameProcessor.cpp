// Detector frame conversion worker implementation.
#include "frontend/processing/DetectorFrameProcessor.hpp"

#include "frontend/processing/DetectorFrameConverter.hpp"

namespace ui
{
DetectorFrameProcessor::DetectorFrameProcessor() = default;

DetectorFrameProcessor::~DetectorFrameProcessor()
{
    stop();
}

void DetectorFrameProcessor::start()
{
    if (running_.exchange(true))
        return;

    workerThread_ = std::thread(&DetectorFrameProcessor::threadLoop, this);
}

void DetectorFrameProcessor::stop()
{
    if (!running_.exchange(false))
        return;

    queueCv_.notify_all();
    if (workerThread_.joinable())
        workerThread_.join();

    std::lock_guard<std::mutex> lock(queueMutex_);
    pendingFrame_.reset();
}

void DetectorFrameProcessor::setImageReadyCallback(ImageReadyCallback callback)
{
    std::lock_guard<std::mutex> lock(callbackMutex_);
    imageReadyCallback_ = std::move(callback);
}

void DetectorFrameProcessor::submitFrame(SharedFramePacket frame)
{
    if (!running_.load() || !frame)
        return;

    {
        std::lock_guard<std::mutex> lock(queueMutex_);
        pendingFrame_ = std::move(frame);
    }

    queueCv_.notify_one();
}

void DetectorFrameProcessor::threadLoop()
{
    while (running_.load())
    {
        SharedFramePacket frame;
        {
            std::unique_lock<std::mutex> lock(queueMutex_);
            queueCv_.wait(lock, [this]() { return !running_.load() || pendingFrame_ != nullptr; });
            if (!running_.load() && !pendingFrame_)
                break;
            if (!pendingFrame_)
                continue;

            frame = std::move(pendingFrame_);
        }

        const QImage image = framePacketToQImage(*frame, reuseImage_);
        if (image.isNull())
            continue;

        ImageReadyCallback callback;
        {
            std::lock_guard<std::mutex> lock(callbackMutex_);
            callback = imageReadyCallback_;
        }

        if (callback)
            callback(image);
    }
}
} // namespace ui
