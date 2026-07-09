#include "backend/camera/CaptureWriterWorker.hpp"

#include "backend/camera/LumoDatasetWriter.hpp"

CaptureWriterWorker::CaptureWriterWorker()
    : writer_(std::make_unique<LumoDatasetWriter>())
{
}

CaptureWriterWorker::~CaptureWriterWorker()
{
    stop();
}

void CaptureWriterWorker::start()
{
    if (running_.exchange(true))
        return;

    writerThread_ = std::thread(&CaptureWriterWorker::writerLoop, this);
}

void CaptureWriterWorker::stop()
{
    if (!running_.load())
    {
        writer_->end();
        return;
    }

    endSessionSync();

    running_ = false;
    jobCv_.notify_all();

    if (writerThread_.joinable())
        writerThread_.join();

    {
        std::lock_guard lock(stateMutex_);
        sessionDirectory_.clear();
    }
}

bool CaptureWriterWorker::beginSessionSync(const CaptureWriterSessionConfig &config,
                                           QString *errorMessage)
{
    if (!running_.load())
        start();

    std::promise<bool> done;
    auto future = done.get_future();

    WriterJob job;
    job.kind = JobKind::BeginSession;
    job.beginConfig = config;
    job.beginPromise = &done;
    job.beginErrorMessage = errorMessage;
    enqueueJob(std::move(job));

    return future.get();
}

void CaptureWriterWorker::submitFrame(FramePacket frame)
{
    if (!running_.load() || !writer_->isActive())
        return;

    WriterJob job;
    job.kind = JobKind::Frame;
    job.frame = std::move(frame);
    enqueueJob(std::move(job));
}

void CaptureWriterWorker::requestEndSession(SessionSummaryCallback onComplete)
{
    WriterJob job;
    job.kind = JobKind::EndSession;
    job.endCallback = std::move(onComplete);
    enqueueJob(std::move(job));
}

CaptureWriterSessionSummary CaptureWriterWorker::endSessionSync()
{
    std::promise<CaptureWriterSessionSummary> done;
    auto future = done.get_future();

    WriterJob job;
    job.kind = JobKind::EndSession;
    job.endPromise = &done;
    enqueueJob(std::move(job));

    return future.get();
}

bool CaptureWriterWorker::isActive() const
{
    return writer_ != nullptr && writer_->isActive();
}

QString CaptureWriterWorker::sessionDirectory() const
{
    std::lock_guard lock(stateMutex_);
    return sessionDirectory_;
}

void CaptureWriterWorker::setErrorCallback(ErrorCallback callback)
{
    std::lock_guard lock(callbackMutex_);
    errorCallback_ = std::move(callback);
}

void CaptureWriterWorker::enqueueJob(WriterJob job)
{
    {
        std::lock_guard lock(jobMutex_);
        jobQueue_.push_back(std::move(job));
    }
    jobCv_.notify_one();
}

bool CaptureWriterWorker::processBeginSession(const CaptureWriterSessionConfig &config,
                                              QString *errorMessage)
{
    if (!writer_->begin(config, errorMessage))
        return false;

    {
        std::lock_guard lock(stateMutex_);
        sessionDirectory_ = writer_->sessionDirectory();
    }
    return true;
}

CaptureWriterSessionSummary CaptureWriterWorker::processEndSession()
{
    CaptureWriterSessionSummary summary = writer_->end();
    {
        std::lock_guard lock(stateMutex_);
        sessionDirectory_.clear();
    }
    return summary;
}

void CaptureWriterWorker::writerLoop()
{
    while (true)
    {
        WriterJob job;
        {
            std::unique_lock lock(jobMutex_);
            jobCv_.wait(lock, [this]() { return !jobQueue_.empty() || !running_.load(); });

            if (jobQueue_.empty())
            {
                if (!running_.load())
                    break;
                continue;
            }

            job = std::move(jobQueue_.front());
            jobQueue_.pop_front();
        }

        switch (job.kind)
        {
        case JobKind::BeginSession:
        {
            const bool ok = processBeginSession(job.beginConfig, job.beginErrorMessage);
            if (job.beginPromise != nullptr)
                job.beginPromise->set_value(ok);
            if (!ok && job.beginErrorMessage != nullptr && !job.beginErrorMessage->isEmpty())
                notifyError(*job.beginErrorMessage);
            break;
        }
        case JobKind::Frame:
        {
            QString errorMessage;
            if (!writer_->appendFrame(job.frame, &errorMessage))
                notifyError(errorMessage);
            break;
        }
        case JobKind::EndSession:
        {
            CaptureWriterSessionSummary summary = processEndSession();
            if (job.endPromise != nullptr)
                job.endPromise->set_value(summary);
            if (job.endCallback)
                job.endCallback(summary);
            break;
        }
        }
    }
}

void CaptureWriterWorker::notifyError(const QString &message)
{
    if (message.isEmpty())
        return;

    ErrorCallback callback;
    {
        std::lock_guard lock(callbackMutex_);
        callback = errorCallback_;
    }
    if (callback)
        callback(message);
}
