// Background worker for FPP MVS decode→fusion after scan (backend/offline).
#pragma once

#include "backend/camera/processing/FppMvsRunner.hpp"

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>

class FppMvsWorker
{
public:
    using CompletionCallback = std::function<void(hf::processing::FppMvsRunResult)>;
    using StatusListener = std::function<void()>;

    FppMvsWorker();
    ~FppMvsWorker();

    void start();
    void stop();

    void requestProcess(const hf::processing::FppMvsRunRequest &request,
                        CompletionCallback onComplete = nullptr);

    void setStatusListener(StatusListener listener);

    bool isBusy() const;

private:
    struct Job
    {
        hf::processing::FppMvsRunRequest request;
        CompletionCallback callback;
    };

    void workerLoop();
    void notifyStatus();

    mutable std::mutex jobMutex_;
    std::condition_variable jobCv_;
    std::deque<Job> jobQueue_;

    std::atomic<bool> running_{false};
    std::atomic<bool> busy_{false};
    StatusListener statusListener_;
    std::thread workerThread_;
};
