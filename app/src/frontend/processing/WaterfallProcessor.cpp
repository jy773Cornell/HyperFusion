// Waterfall processor thread: RGB line extract and vertical stack.
#include "frontend/processing/WaterfallProcessor.hpp"

#include "frontend/processing/RgbBandExtractor.hpp"

#include <algorithm>
#include <cstring>

namespace ui
{
namespace
{
constexpr int kDefaultMaxLines = 512;
constexpr std::size_t kMaxQueueDepth = 256;
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
    publishSnapshot_ = QImage();
    lineCount_ = 0;
    displayDirty_ = false;
    lastPublishAt_ = {};
}

void WaterfallProcessor::setMaxLines(const int maxLines)
{
    std::lock_guard<std::mutex> stateLock(stateMutex_);
    const int newMax = std::max(16, maxLines);
    if (lineCount_ > newMax)
        lineCount_ = newMax;

    if (waterfallImage_.isNull())
    {
        maxLines_ = newMax;
        return;
    }

    const int width = waterfallImage_.width();
    if (newMax != maxLines_ || waterfallImage_.height() != newMax)
    {
        QImage expanded(width, newMax, QImage::Format_RGB888);
        expanded.fill(qRgb(0, 0, 0));
        const int copyLines = std::min(lineCount_, newMax);
        for (int y = 0; y < copyLines; ++y)
        {
            std::memcpy(expanded.scanLine(y),
                        waterfallImage_.scanLine(y),
                        static_cast<std::size_t>(width) * 3);
        }
        waterfallImage_ = std::move(expanded);
        displayDirty_ = true;
    }

    maxLines_ = newMax;
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

void WaterfallProcessor::setPublishIntervalMs(const int intervalMs)
{
    {
        std::lock_guard<std::mutex> stateLock(stateMutex_);
        publishInterval_ = std::chrono::milliseconds(std::max(16, intervalMs));
    }
    forcePublish_.store(true, std::memory_order_release);
    queueCv_.notify_one();
}

void WaterfallProcessor::submitFrame(SharedFramePacket frame)
{
    if (!running_.load() || !frame)
        return;

    PendingFrame pending;
    pending.packet = std::move(frame);
    {
        std::lock_guard<std::mutex> stateLock(stateMutex_);
        pending.bands = bands_;
    }

    {
        std::lock_guard<std::mutex> queueLock(queueMutex_);
        if (queue_.size() >= kMaxQueueDepth)
            queue_.pop_front();
        queue_.push_back(std::move(pending));
    }

    queueCv_.notify_one();
}

void WaterfallProcessor::appendRgbLine(const std::vector<std::uint8_t> &rgbRow, const int width)
{
    if (width <= 0 || rgbRow.size() < static_cast<std::size_t>(width) * 3)
        return;

    if (waterfallImage_.isNull() || waterfallImage_.width() != width || waterfallImage_.height() != maxLines_)
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
    displayDirty_ = true;
}

void WaterfallProcessor::publishImage()
{
    int activeLines = 0;
    int width = 0;
    {
        std::lock_guard<std::mutex> stateLock(stateMutex_);
        if (waterfallImage_.isNull() || lineCount_ <= 0)
            return;

        activeLines = lineCount_;
        width = waterfallImage_.width();
        if (publishSnapshot_.width() != width || publishSnapshot_.height() != activeLines
            || publishSnapshot_.format() != QImage::Format_RGB888)
        {
            publishSnapshot_ = QImage(width, activeLines, QImage::Format_RGB888);
        }

        for (int y = 0; y < activeLines; ++y)
        {
            std::memcpy(publishSnapshot_.scanLine(y),
                        waterfallImage_.scanLine(y),
                        static_cast<std::size_t>(width) * 3);
        }

        displayDirty_ = false;
        lastPublishAt_ = std::chrono::steady_clock::now();
    }

    ImageReadyCallback callback;
    {
        std::lock_guard<std::mutex> lock(callbackMutex_);
        callback = imageReadyCallback_;
    }

    if (callback)
        callback(publishSnapshot_);
}

void WaterfallProcessor::maybePublishImage()
{
    const bool force = forcePublish_.exchange(false, std::memory_order_acq_rel);
    bool shouldPublish = false;
    {
        std::lock_guard<std::mutex> stateLock(stateMutex_);
        if (!displayDirty_)
            return;

        if (force)
        {
            shouldPublish = true;
        }
        else
        {
            const auto now = std::chrono::steady_clock::now();
            if (lastPublishAt_ == std::chrono::steady_clock::time_point{}
                || now - lastPublishAt_ >= publishInterval_)
            {
                shouldPublish = true;
            }
        }
    }

    if (shouldPublish)
        publishImage();
}

void WaterfallProcessor::threadLoop()
{
    while (running_.load())
    {
        PendingFrame pending;
        bool hasFrame = false;
        {
            std::unique_lock<std::mutex> queueLock(queueMutex_);
            queueCv_.wait(queueLock, [this]() {
                return !running_.load() || !queue_.empty()
                       || forcePublish_.load(std::memory_order_acquire);
            });
            if (!running_.load() && queue_.empty()
                && !forcePublish_.load(std::memory_order_acquire))
                break;

            if (!queue_.empty())
            {
                pending = std::move(queue_.front());
                queue_.pop_front();
                hasFrame = true;
            }
        }

        if (!hasFrame)
        {
            maybePublishImage();
            continue;
        }

        std::vector<std::uint8_t> rgbRow;
        if (!extractRgbLineFromBilFrame(*pending.packet, pending.bands, rgbRow))
            continue;

        {
            std::lock_guard<std::mutex> stateLock(stateMutex_);
            appendRgbLine(rgbRow, pending.packet->width);
        }

        maybePublishImage();
    }
}
} // namespace ui
