// Blackfly S Spinnaker adapter (backend/3dscanning). No acquisition until connect+start.
#pragma once

#include "backend/3dscanning/BfsCameraTypes.hpp"

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace hf::bfs
{
class BfsSpinnakerCamera
{
public:
    BfsSpinnakerCamera();
    ~BfsSpinnakerCamera();

    BfsSpinnakerCamera(const BfsSpinnakerCamera &) = delete;
    BfsSpinnakerCamera &operator=(const BfsSpinnakerCamera &) = delete;

    [[nodiscard]] static bool sdkAvailable();
    [[nodiscard]] static std::vector<BfsDeviceInfo> enumerateDevices(BfsError *error = nullptr);

    [[nodiscard]] BfsCameraState state() const;
    [[nodiscard]] std::string connectedSerial() const;

    bool connect(const QString &cameraId, BfsError &error);
    bool applySettings(const BfsCameraSettings &settings, BfsError &error);
    bool startStreaming(BfsError &error);
    bool pollFrame(BfsRgbFrame &frame, std::uint32_t timeoutMs, BfsError &error);
    void stopStreaming();
    void disconnect();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    mutable std::mutex mutex_;
};
} // namespace hf::bfs
