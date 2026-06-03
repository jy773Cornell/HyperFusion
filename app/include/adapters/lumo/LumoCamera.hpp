// Generic Specim Lumo Sensor SDK camera instance (SSP selected via device index).
#pragma once

#include "adapters/lumo/LumoDeviceTypes.hpp"
#include "core/ICameraController.hpp"

#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

class LumoCamera : public ICameraController
{
public:
    explicit LumoCamera(CameraBackendId backendId,
                        std::string instanceLabel,
                        LumoSensorKind sensorKind = LumoSensorKind::Fx10ePleora);
    ~LumoCamera() override;

    void prepareConnection(const CameraSettings &settings);

    /// Lists SSP sensors (SI_SYSTEM DeviceName). No sensor handle may be open on this instance.
    static bool enumerateDevices(const CameraSettings &prep,
                                 std::vector<LumoDeviceEntry> &devices,
                                 CameraError &error);

    std::string name() const override;
    CameraBackendId backendId() const override;
    LumoSensorKind sensorKind() const override;
    bool requiresGuiThreadForSdkLifecycle() const override;

    bool connect(CameraError &error) override;
    bool initialize(CameraError &error) override;
    bool applySettings(const CameraSettings &settings,
                       CameraError &error,
                       CameraTimingApplyResult *timingOut = nullptr) override;
    bool openShutter(CameraError &error) override;
    bool closeShutter(CameraError &error) override;
    bool shutterIsOpen(bool &isOpen, CameraError &error) override;
    bool arm(CameraError &error) override;
    bool start(CameraError &error) override;
    void stop() override;
    void disconnect() override;

    CameraState state() const override;
    bool pollFrame(FramePacket &frame, std::uint32_t timeoutMs, CameraError &error) override;

private:
    std::string tag() const;

    bool expectState(CameraState expected, CameraError &error);
    void setFault(const CameraError &error);

#if defined(HF_HAVE_LUMO_SDK)
    friend int lumoDataCallbackEntry(std::uint8_t *buffer,
                                     std::int64_t frameSize,
                                     std::int64_t frameNumber,
                                     void *context);

    bool ensureSdkLoaded(CameraError &error);
    void releaseSdkLoad();
    bool checkSi(int code, const char *operation, CameraError &error);
    void rollbackOpenConnection();
    bool refreshImageGeometry(CameraError &error);
    bool applyCameraTiming(void *handle,
                           const CameraSettings &requested,
                           CameraError &error,
                           CameraTimingApplyResult *timingOut);
    bool registerDataCallback(CameraError &error);
    void unregisterDataCallback();
    void onFrame(const std::uint8_t *buffer, std::int64_t frameSize, std::int64_t frameNumber);
    /// Stop acquisition and wake any blocked pollFrame (must not hold mutex_ while waiting on frameMutex_).
    void haltAcquisition();
#endif

    const CameraBackendId backendId_;
    const std::string instanceLabel_;
    const LumoSensorKind sensorKind_;

    mutable std::mutex mutex_;
    CameraState state_ = CameraState::Disconnected;
    CameraSettings settings_;

#if defined(HF_HAVE_LUMO_SDK)
    void *handle_ = nullptr;
    bool callbackRegistered_ = false;
#endif

    int imageWidth_ = 0;
    int imageHeight_ = 0;
    std::int64_t imageSizeBytes_ = 0;

    std::mutex frameMutex_;
    std::condition_variable frameCv_;
    std::vector<std::uint8_t> latestFrameBytes_;
    std::int64_t latestFrameNumber_ = 0;
    bool frameReady_ = false;
};
