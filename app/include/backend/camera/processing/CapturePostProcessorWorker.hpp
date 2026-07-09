#pragma once

#include "backend/CaptureWriterTypes.hpp"
#include "backend/processing/CapturePostProcessor.hpp"

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>

class CapturePostProcessorWorker
{
public:
    using CompletionCallback = std::function<void(hf::processing::CapturePostProcessResult)>;
    using StatusListener = std::function<void()>;

    struct QueueStatus
    {
        bool processing = false;
        int queuedCount = 0;
        int completedCount = 0;
        QString activeSessionDirectory;

        int outstandingTotal() const { return queuedCount + (processing ? 1 : 0); }

        int currentJobNumber() const
        {
            if (!processing)
                return 0;
            return completedCount + 1;
        }
    };

    CapturePostProcessorWorker();
    ~CapturePostProcessorWorker();

    void start();
    void stop();

    void requestProcess(const CaptureWriterSessionSummary &summary,
                        const hf::processing::CapturePostProcessOptions &options,
                        CompletionCallback onComplete = nullptr);

    void setStatusListener(StatusListener listener);

    bool isBusy() const;
    QueueStatus queueStatus() const;

private:
    struct ProcessJob
    {
        CaptureWriterSessionSummary summary;
        hf::processing::CapturePostProcessOptions options;
        CompletionCallback callback;
    };

    void workerLoop();
    void notifyStatus();

    mutable std::mutex jobMutex_;
    std::condition_variable jobCv_;
    std::deque<ProcessJob> jobQueue_;

    std::atomic<bool> running_{false};
    std::atomic<bool> busy_{false};
    std::atomic<int> completedCount_{0};
    QString activeSessionDirectory_;
    StatusListener statusListener_;
    std::thread workerThread_;
};
