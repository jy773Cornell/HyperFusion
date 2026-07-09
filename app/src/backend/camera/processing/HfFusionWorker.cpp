// Background worker for manual/offline hf_fusion session runs (backend/offline).
#include "backend/camera/processing/HfFusionWorker.hpp"

HfFusionWorker::HfFusionWorker() = default;

HfFusionWorker::~HfFusionWorker()
{
    stop();
}

void HfFusionWorker::start()
{
    if (running_.exchange(true))
        return;

    workerThread_ = std::thread(&HfFusionWorker::workerLoop, this);
}

void HfFusionWorker::stop()
{
    if (!running_.exchange(false))
        return;

    jobCv_.notify_all();
    if (workerThread_.joinable())
        workerThread_.join();

    {
        std::lock_guard lock(jobMutex_);
        jobQueue_.clear();
    }

    busy_ = false;
    notifyStatus();
}

bool HfFusionWorker::isBusy() const
{
    std::lock_guard lock(jobMutex_);
    return busy_.load() || !jobQueue_.empty();
}

void HfFusionWorker::setStatusListener(StatusListener listener)
{
    std::lock_guard lock(jobMutex_);
    statusListener_ = std::move(listener);
}

void HfFusionWorker::notifyStatus()
{
    StatusListener listener;
    {
        std::lock_guard lock(jobMutex_);
        listener = statusListener_;
    }

    if (listener)
        listener();
}

void HfFusionWorker::requestFusion(const QString &sessionDirectory,
                                   const QStringList &modes,
                                   CompletionCallback onComplete)
{
    if (!running_.load())
        start();

    FusionJob job;
    job.sessionDirectory = sessionDirectory;
    job.modes = modes;
    job.callback = std::move(onComplete);

    {
        std::lock_guard lock(jobMutex_);
        jobQueue_.push_back(std::move(job));
    }

    notifyStatus();
    jobCv_.notify_one();
}

void HfFusionWorker::workerLoop()
{
    while (running_.load())
    {
        FusionJob job;
        {
            std::unique_lock lock(jobMutex_);
            jobCv_.wait(lock, [this]() { return !running_.load() || !jobQueue_.empty(); });
            if (!running_.load() && jobQueue_.empty())
                break;
            if (jobQueue_.empty())
                continue;

            job = std::move(jobQueue_.front());
            jobQueue_.pop_front();
        }

        busy_ = true;
        notifyStatus();

        const hf::processing::HfFusionSessionResult result =
            hf::processing::runSessionFusion(job.sessionDirectory, job.modes);

        busy_ = false;
        notifyStatus();

        if (job.callback)
            job.callback(result);
    }
}
