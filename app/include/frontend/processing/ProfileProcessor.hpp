// Background worker: spectral (wavelength) and spatial (pixel) DN profiles from streamed frames.
#pragma once

#include "frontend/processing/BilProfileExtractor.hpp"

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>

#include "backend/CameraTypes.hpp"

namespace ui
{
class ProfileProcessor
{
public:
    using ProfilesReadyCallback = std::function<void(ProfileExtraction profiles)>;

    ProfileProcessor();
    ~ProfileProcessor();

    ProfileProcessor(const ProfileProcessor &) = delete;
    ProfileProcessor &operator=(const ProfileProcessor &) = delete;

    void start();
    void stop();
    void reset();

    void setCursor(const ProfileCursor &cursor);
    void setProfilesReadyCallback(ProfilesReadyCallback callback);

    void submitFrame(SharedFramePacket frame);
    void requestRefresh();

private:
    struct PendingJob
    {
        SharedFramePacket frame;
        ProfileCursor cursor;
        bool hasFrame = false;
    };

    void threadLoop();
    void enqueueJob(bool withFrame, SharedFramePacket frame);

    ProfilesReadyCallback profilesReadyCallback_;
    std::mutex callbackMutex_;

    std::mutex queueMutex_;
    std::condition_variable queueCv_;
    std::deque<PendingJob> queue_;

    std::mutex stateMutex_;
    SharedFramePacket latestFrame_;
    ProfileCursor cursor_;
    bool hasLatestFrame_ = false;

    std::thread workerThread_;
    std::atomic<bool> running_{false};
};
} // namespace ui
