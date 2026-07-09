#pragma once

#include "backend/CaptureWriterTypes.hpp"

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <thread>

class LumoDatasetWriter;

class CaptureWriterWorker
{
public:
    using ErrorCallback = std::function<void(const QString &)>;
    using SessionSummaryCallback = std::function<void(const CaptureWriterSessionSummary &)>;

    CaptureWriterWorker();
    ~CaptureWriterWorker();

    void start();
    void stop();

    bool beginSessionSync(const CaptureWriterSessionConfig &config, QString *errorMessage = nullptr);
    void submitFrame(FramePacket frame);
    void requestEndSession(SessionSummaryCallback onComplete = nullptr);
    CaptureWriterSessionSummary endSessionSync();

    bool isActive() const;
    QString sessionDirectory() const;

    void setErrorCallback(ErrorCallback callback);

private:
    enum class JobKind
    {
        BeginSession,
        Frame,
        EndSession,
    };

    struct WriterJob
    {
        JobKind kind = JobKind::Frame;
        CaptureWriterSessionConfig beginConfig;
        FramePacket frame;
        SessionSummaryCallback endCallback;
        std::promise<bool> *beginPromise = nullptr;
        QString *beginErrorMessage = nullptr;
        std::promise<CaptureWriterSessionSummary> *endPromise = nullptr;
    };

    void enqueueJob(WriterJob job);
    void writerLoop();
    void notifyError(const QString &message);
    bool processBeginSession(const CaptureWriterSessionConfig &config, QString *errorMessage);
    CaptureWriterSessionSummary processEndSession();

    std::unique_ptr<LumoDatasetWriter> writer_;

    mutable std::mutex stateMutex_;
    QString sessionDirectory_;

    mutable std::mutex callbackMutex_;
    ErrorCallback errorCallback_;

    mutable std::mutex jobMutex_;
    std::condition_variable jobCv_;
    std::deque<WriterJob> jobQueue_;

    std::atomic<bool> running_{false};
    std::thread writerThread_;
};
