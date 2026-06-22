// Profile processor worker thread implementation.
#include "frontend/processing/ProfileProcessor.hpp"

namespace ui
{
namespace
{
constexpr std::size_t kMaxQueueDepth = 2;
} // namespace

ProfileProcessor::ProfileProcessor() = default;

ProfileProcessor::~ProfileProcessor()
{
    stop();
}

void ProfileProcessor::start()
{
    if (running_.exchange(true))
        return;

    workerThread_ = std::thread(&ProfileProcessor::threadLoop, this);
}

void ProfileProcessor::stop()
{
    if (!running_.exchange(false))
        return;

    queueCv_.notify_all();
    if (workerThread_.joinable())
        workerThread_.join();

    std::lock_guard<std::mutex> queueLock(queueMutex_);
    queue_.clear();
}

void ProfileProcessor::reset()
{
    std::lock_guard<std::mutex> stateLock(stateMutex_);
    latestFrame_.reset();
    hasLatestFrame_ = false;
    cursor_ = ProfileCursor{};
}

void ProfileProcessor::setCursor(const ProfileCursor &cursor)
{
    std::lock_guard<std::mutex> stateLock(stateMutex_);
    cursor_ = cursor;
}

void ProfileProcessor::setProfilesReadyCallback(ProfilesReadyCallback callback)
{
    std::lock_guard<std::mutex> lock(callbackMutex_);
    profilesReadyCallback_ = std::move(callback);
}

void ProfileProcessor::enqueueJob(const bool withFrame, SharedFramePacket frame)
{
    PendingJob job;
    {
        std::lock_guard<std::mutex> stateLock(stateMutex_);
        if (withFrame && frame)
        {
            latestFrame_ = std::move(frame);
            hasLatestFrame_ = true;
        }
        if (!hasLatestFrame_ || !latestFrame_)
            return;

        job.frame = latestFrame_;
        job.cursor = cursor_;
        job.hasFrame = true;
    }

    {
        std::lock_guard<std::mutex> queueLock(queueMutex_);
        while (queue_.size() >= kMaxQueueDepth)
            queue_.pop_front();
        queue_.push_back(std::move(job));
    }

    queueCv_.notify_one();
}

void ProfileProcessor::submitFrame(SharedFramePacket frame)
{
    if (!running_.load() || !frame)
        return;

    enqueueJob(true, std::move(frame));
}

void ProfileProcessor::requestRefresh()
{
    if (!running_.load())
        return;

    enqueueJob(false, {});
}

void ProfileProcessor::threadLoop()
{
    while (running_.load())
    {
        PendingJob job;
        {
            std::unique_lock<std::mutex> queueLock(queueMutex_);
            queueCv_.wait(queueLock, [this]() { return !running_.load() || !queue_.empty(); });
            if (!running_.load() && queue_.empty())
                break;
            if (queue_.empty())
                continue;

            job = std::move(queue_.front());
            queue_.pop_front();
        }

        if (!job.frame)
            continue;

        ProfileExtraction profiles;
        if (!extractProfilesFromBilFrame(*job.frame, job.cursor, profiles))
            continue;

        ProfilesReadyCallback callback;
        {
            std::lock_guard<std::mutex> lock(callbackMutex_);
            callback = profilesReadyCallback_;
        }

        if (callback)
            callback(std::move(profiles));
    }
}
} // namespace ui
