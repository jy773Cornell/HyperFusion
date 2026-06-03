// Waterfall processor thread: RGB line extract and vertical stack.
#include "ui/WaterfallProcessor.hpp"

#include "ui/RgbBandExtractor.hpp"

#include <algorithm>
#include <cstring>

namespace ui
{
namespace
{
constexpr int kDefaultMaxLines = 512;
constexpr std::size_t kMaxQueueDepth = 2;
} // namespace

WaterfallProcessor::WaterfallProcessor() = default;

WaterfallProcessor::~WaterfallProcessor()
{
    stop();
}

void WaterfallProcessor::start()
{
    if (running_.exchange(true))
        return;

    workerThread_ = std::thread(&WaterfallProcessor::threadLoop, this);
}

void WaterfallProcessor::stop()
{
    if (!running_.exchange(false))
        return;

    queueCv_.notify_all();
    if (workerThread_.joinable())
        workerThread_.join();

    {
        std::lock_guard<std::mutex> queueLock(queueMutex_);
        queue_.clear();
    }
}

void WaterfallProcessor::reset()
{
    std::lock_guard<std::mutex> stateLock(stateMutex_);
    waterfallImage_ = QImage();
    lineCount_ = 0;
}

void WaterfallProcessor::setMaxLines(const int maxLines)
{
    std::lock_guard<std::mutex> stateLock(stateMutex_);
    maxLines_ = std::max(16, maxLines);
    if (lineCount_ > maxLines_)
        lineCount_ = maxLines_;
}

void WaterfallProcessor::setBandIndices(const RgbBandIndices &bands)
{
    std::lock_guard<std::mutex> stateLock(stateMutex_);
    bands_ = bands;
}

void WaterfallProcessor::setImageReadyCallback(ImageReadyCallback callback)
{
    std::lock_guard<std::mutex> lock(callbackMutex_);
    imageReadyCallback_ = std::move(callback);
}

void WaterfallProcessor::submitFrame(FramePacket frame)
{
    if (!running_.load())
        return;

    PendingFrame pending;
    pending.packet = std::move(frame);
    {
        std::lock_guard<std::mutex> stateLock(stateMutex_);
        pending.bands = bands_;
    }

    {
        std::lock_guard<std::mutex> queueLock(queueMutex_);
        while (queue_.size() >= kMaxQueueDepth)
            queue_.pop_front();
        queue_.push_back(std::move(pending));
    }

    queueCv_.notify_one();
}

void WaterfallProcessor::appendRgbLine(const std::vector<std::uint8_t> &rgbRow, const int width)
{
    if (width <= 0 || rgbRow.size() < static_cast<std::size_t>(width) * 3)
        return;

    if (waterfallImage_.isNull() || waterfallImage_.width() != width)
    {
        waterfallImage_ = QImage(width, maxLines_, QImage::Format_RGB888);
        waterfallImage_.fill(qRgb(0, 0, 0));
        lineCount_ = 0;
    }

    if (lineCount_ >= maxLines_)
    {
        for (int y = 1; y < maxLines_; ++y)
        {
            std::memcpy(waterfallImage_.scanLine(y - 1),
                        waterfallImage_.scanLine(y),
                        static_cast<std::size_t>(width) * 3);
        }
        lineCount_ = maxLines_;
    }
    else
    {
        ++lineCount_;
    }

    const int targetRow = lineCount_ - 1;
    auto *scanLine = waterfallImage_.scanLine(targetRow);
    std::memcpy(scanLine, rgbRow.data(), static_cast<std::size_t>(width) * 3);
}

void WaterfallProcessor::publishImage()
{
    QImage snapshot;
    int activeLines = 0;
    {
        std::lock_guard<std::mutex> stateLock(stateMutex_);
        if (waterfallImage_.isNull() || lineCount_ <= 0)
            return;

        activeLines = lineCount_;
        snapshot = waterfallImage_.copy(0, 0, waterfallImage_.width(), activeLines);
    }

    ImageReadyCallback callback;
    {
        std::lock_guard<std::mutex> lock(callbackMutex_);
        callback = imageReadyCallback_;
    }

    if (callback)
        callback(std::move(snapshot));
}

void WaterfallProcessor::threadLoop()
{
    while (running_.load())
    {
        PendingFrame pending;
        {
            std::unique_lock<std::mutex> queueLock(queueMutex_);
            queueCv_.wait(queueLock, [this]() { return !running_.load() || !queue_.empty(); });
            if (!running_.load() && queue_.empty())
                break;
            if (queue_.empty())
                continue;

            pending = std::move(queue_.front());
            queue_.pop_front();
        }

        std::vector<std::uint8_t> rgbRow;
        if (!extractRgbLineFromBilFrame(pending.packet, pending.bands, rgbRow))
            continue;

        {
            std::lock_guard<std::mutex> stateLock(stateMutex_);
            appendRgbLine(rgbRow, pending.packet.width);
        }

        publishImage();
    }
}
} // namespace ui
