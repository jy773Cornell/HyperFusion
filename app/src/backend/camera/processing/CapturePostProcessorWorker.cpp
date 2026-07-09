#include "backend/camera/processing/CapturePostProcessorWorker.hpp"

CapturePostProcessorWorker::CapturePostProcessorWorker() = default;

CapturePostProcessorWorker::~CapturePostProcessorWorker()
{
    stop();
}

void CapturePostProcessorWorker::start()
{
    if (running_.exchange(true))
        return;

    workerThread_ = std::thread(&CapturePostProcessorWorker::workerLoop, this);
}

void CapturePostProcessorWorker::stop()
{
    if (!running_.exchange(false))
        return;

    jobCv_.notify_all();
    if (workerThread_.joinable())
        workerThread_.join();

    {
        std::lock_guard lock(jobMutex_);
        jobQueue_.clear();
        activeSessionDirectory_.clear();
    }

    busy_ = false;
    completedCount_ = 0;
    notifyStatus();
}

bool CapturePostProcessorWorker::isBusy() const
{
    return busy_.load();
}

void CapturePostProcessorWorker::setStatusListener(StatusListener listener)
{
    std::lock_guard lock(jobMutex_);
    statusListener_ = std::move(listener);
}

CapturePostProcessorWorker::QueueStatus CapturePostProcessorWorker::queueStatus() const
{
    QueueStatus status;
    status.processing = busy_.load();
    status.completedCount = completedCount_.load();

    {
        std::lock_guard lock(jobMutex_);
        status.queuedCount = static_cast<int>(jobQueue_.size());
        status.activeSessionDirectory = activeSessionDirectory_;
    }

    return status;
}

void CapturePostProcessorWorker::notifyStatus()
{
    StatusListener listener;
    {
        std::lock_guard lock(jobMutex_);
        listener = statusListener_;
    }

    if (listener)
        listener();
}

void CapturePostProcessorWorker::requestProcess(const CaptureWriterSessionSummary &summary,
                                                const hf::processing::CapturePostProcessOptions &options,
                                                CompletionCallback onComplete)
{
    if (!running_.load())
        start();

    ProcessJob job;
    job.summary = summary;
    job.options = options;
    job.callback = std::move(onComplete);

    {
        std::lock_guard lock(jobMutex_);
        jobQueue_.push_back(std::move(job));
    }

    notifyStatus();
    jobCv_.notify_one();
}

void CapturePostProcessorWorker::workerLoop()
{
    while (running_.load())
    {
        ProcessJob job;
        {
            std::unique_lock lock(jobMutex_);
            jobCv_.wait(lock, [this]() { return !running_.load() || !jobQueue_.empty(); });
            if (!running_.load() && jobQueue_.empty())
                break;
            if (jobQueue_.empty())
                continue;

            job = std::move(jobQueue_.front());
            jobQueue_.pop_front();
            activeSessionDirectory_ = job.summary.sessionDirectory;
        }

        busy_ = true;
        notifyStatus();

        const hf::processing::CapturePostProcessResult result =
            hf::processing::processCaptureSession(job.summary, job.options);

        busy_ = false;
        completedCount_.fetch_add(1);

        {
            std::lock_guard lock(jobMutex_);
            activeSessionDirectory_.clear();
            if (jobQueue_.empty())
                completedCount_ = 0;
        }

        notifyStatus();

        if (job.callback)
            job.callback(result);
    }
}
