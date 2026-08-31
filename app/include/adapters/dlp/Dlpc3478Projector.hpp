// DLPC3478 / DLP3010EVM-LC adapter (adapters/dlp).
// 64-bit GUI talks to Win32 hf_dlpc_bridge.exe (TI Cypress lib is x86). Control thread only.
#pragma once

#include "backend/fpp/DlpTypes.hpp"

#include <memory>
#include <mutex>
#include <string>
#include <vector>

class QProcess;

namespace hf::dlp
{
class Dlpc3478Projector
{
public:
    Dlpc3478Projector();
    ~Dlpc3478Projector();

    Dlpc3478Projector(const Dlpc3478Projector &) = delete;
    Dlpc3478Projector &operator=(const Dlpc3478Projector &) = delete;

    [[nodiscard]] static bool sdkAvailable();
    [[nodiscard]] static bool hardwareBackendAvailable();
    [[nodiscard]] static std::vector<DlpDeviceInfo> enumerateDevices(DlpError *error = nullptr);

    [[nodiscard]] DlpProjectorState state() const;
    [[nodiscard]] std::string connectedId() const;
    [[nodiscard]] bool hardwareOutputEnabled() const;

    bool connect(const QString &deviceId, DlpError &error);
    bool arm(const DlpProjectorSettings &settings, DlpError &error);
    bool blank(DlpError &error);
    bool showTestPattern(const QString &patternName, DlpError &error);
    bool applyLedCurrents(const DlpProjectorSettings &settings, DlpError &error);
    std::vector<DlpDeviceInfo> listDevices(DlpError *error);
    void disconnect();

private:
    bool requireConnected(DlpError &error) const;
    void teardownLocked();
#if defined(HF_HAVE_DLPC_API)
    bool ensureBridgeLocked(DlpError &error);
    void restartBridgeLocked();
    bool transactLocked(const QString &request, QStringList *extraLines, DlpError &error,
                        int timeoutMs);
    bool commandOkLocked(const QString &request, DlpError &error, int timeoutMs);
#endif

    mutable std::mutex mutex_;
    DlpProjectorState state_ = DlpProjectorState::Disconnected;
    std::string connectedId_;
    bool ledsEnabled_ = false;
#if defined(HF_HAVE_DLPC_API)
    std::unique_ptr<QProcess> bridge_;
#endif
};
} // namespace hf::dlp
