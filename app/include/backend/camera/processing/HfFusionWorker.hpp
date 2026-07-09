// Background worker for manual/offline hf_fusion session runs (backend/offline).
#pragma once

#include "backend/camera/processing/HfFusionRunner.hpp"

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>

class HfFusionWorker
{
public:
    using CompletionCallback = std::function<void(hf::processing::HfFusionSessionResult)>;
    using StatusListener = std::function<void()>;

    HfFusionWorker();
    ~HfFusionWorker();

    void start();
    void stop();

    void requestFusion(const QString &sessionDirectory,
                       const QStringList &modes = {},
                       CompletionCallback onComplete = nullptr);

    void setStatusListener(StatusListener listener);

    bool isBusy() const;

private:
    struct FusionJob
    {
        QString sessionDirectory;
        QStringList modes;
        CompletionCallback callback;
    };

    void workerLoop();
    void notifyStatus();

    mutable std::mutex jobMutex_;
    std::condition_variable jobCv_;
    std::deque<FusionJob> jobQueue_;

    std::atomic<bool> running_{false};
    std::atomic<bool> busy_{false};
    StatusListener statusListener_;
    std::thread workerThread_;
};
