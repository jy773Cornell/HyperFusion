// Background waterfall builder: RGB lines from streamed frames stacked for Qt display.
#pragma once

#include "core/CameraTypes.hpp"
#include "ui/RgbBandExtractor.hpp"

#include <QImage>

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>

namespace ui
{
class WaterfallProcessor
{
public:
    using ImageReadyCallback = std::function<void(QImage image)>;

    WaterfallProcessor();
    ~WaterfallProcessor();

    WaterfallProcessor(const WaterfallProcessor &) = delete;
    WaterfallProcessor &operator=(const WaterfallProcessor &) = delete;

    void start();
    void stop();

    void reset();
    void setMaxLines(int maxLines);
    void setBandIndices(const RgbBandIndices &bands);
    void setImageReadyCallback(ImageReadyCallback callback);

    /// Enqueues a frame copy for processing (drops oldest pending if queue is full).
    void submitFrame(FramePacket frame);

private:
    struct PendingFrame
    {
        FramePacket packet;
        RgbBandIndices bands;
    };

    void threadLoop();
    void appendRgbLine(const std::vector<std::uint8_t> &rgbRow, int width);
    void publishImage();

    ImageReadyCallback imageReadyCallback_;
    std::mutex callbackMutex_;

    std::mutex queueMutex_;
    std::condition_variable queueCv_;
    std::deque<PendingFrame> queue_;

    std::mutex stateMutex_;
    RgbBandIndices bands_;
    QImage waterfallImage_;
    int lineCount_ = 0;
    int maxLines_ = 512;

    std::thread workerThread_;
    std::atomic<bool> running_{false};
};
} // namespace ui
