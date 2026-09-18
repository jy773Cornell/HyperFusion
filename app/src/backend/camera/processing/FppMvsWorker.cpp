// Background worker for FPP MVS decode→fusion after scan (backend/offline).
#include "backend/camera/processing/FppMvsWorker.hpp"

FppMvsWorker::FppMvsWorker() = default;

FppMvsWorker::~FppMvsWorker()
{
    stop();
}

void FppMvsWorker::start()
{
    if (running_.exchange(true))
        return;

    workerThread_ = std::thread(&FppMvsWorker::workerLoop, this);
}

void FppMvsWorker::stop()
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

bool FppMvsWorker::isBusy() const
{
    std::lock_guard lock(jobMutex_);
    return busy_.load() || !jobQueue_.empty();
}

void FppMvsWorker::setStatusListener(StatusListener listener)
{
    std::lock_guard lock(jobMutex_);
    statusListener_ = std::move(listener);
}

void FppMvsWorker::notifyStatus()
{
    StatusListener listener;
    {
        std::lock_guard lock(jobMutex_);
        listener = statusListener_;
    }

    if (listener)
        listener();
}

void FppMvsWorker::requestProcess(const hf::processing::FppMvsRunRequest &request,
                                  CompletionCallback onComplete)
{
    if (!running_.load())
        start();

    Job job;
    job.request = request;
    job.callback = std::move(onComplete);

    {
        std::lock_guard lock(jobMutex_);
        jobQueue_.push_back(std::move(job));
    }

    notifyStatus();
    jobCv_.notify_one();
}

void FppMvsWorker::workerLoop()
{
    while (running_.load())
    {
        Job job;
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

        hf::processing::FppMvsRunResult result = hf::processing::runFppMvsPipeline(job.request);

        if (job.callback)
            job.callback(std::move(result));

        busy_ = false;
        notifyStatus();
    }
}
