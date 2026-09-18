// Semi-fixed UR3e ring-scan execute (backend orchestration helpers).
// MoveIt only for home↔ring / ring↔ring; shoulder_pan stepped spin without MoveIt.
// Works for mock and real robot (same C++ path; sidecar hardware joint move).

#pragma once

#include "backend/multiview/Ur3eClient.hpp"
#include "backend/multiview/Ur3eHemisphereScan.hpp"
#include "backend/multiview/Ur3eSemiFixedScan.hpp"

#include <QString>

#include <atomic>
#include <functional>

namespace hf::ur3e
{

struct SemiFixedScanExecuteInput
{
    QString serverUrl;
    Ur3eSemiFixedRoute route;
    QString captureDir; ///< Empty = motion-only.
    int stabilizeMs = 500;
    int sessionId = 0;
    Ur3eWristSweepParams wristSweep{};
    /// All = top then rings (default). ApexOnly = top only. RingsOnly = skip top.
    bool skipTop = false;
    bool skipRings = false;
    /// GUI Multiview / FPP sample-stage stop (mm).
    double stageMm = 1600.0;
};

struct SemiFixedScanExecuteHost
{
    std::function<bool()> sessionActive;
    std::function<bool()> stopRequested;
    std::function<void()> clearStopRequested;
    std::function<void(bool)> setReturningHome;
    /// Blocking: ensure at home before scan. Return false if cancelled.
    std::function<bool()> ensureHomeBeforeScan;
    std::function<void()> syncHomeSliders;
    std::function<void(const QString &)> log;
    std::function<void(int ringIndex)> setActiveRing;
    std::function<void(int ringIndex)> markRingCompleted;
    std::function<void(int ringIndex)> markRingFailed;
    /// One imaging pose finished (center or wrist-sweep sample) — progress bar, not ring color.
    std::function<void()> markPinCompleted;
    std::function<void()> markPinFailed;
    /// Capture one still. ringIndex=-1 = top/apex pose; sampleIndex=0 there.
    std::function<bool(const Ur3eScanTcpPose &plannedTcp, int ringIndex, int sampleIndex)>
        captureStill;
    /// Optional full-scan stage move. Empty = ignore stage.
    /// Return false on failed/aborted move. Do not install when the stage is disconnected.
    std::function<bool(double targetMm, const QString &label, QString *errorOut)> moveStage;
    std::function<void(bool ok,
                       const QString &error,
                       int executedSamples,
                       bool stopped,
                       int capturedCount,
                       qint64 elapsedMs)>
        finish;
};

/// Blocking worker-thread entry. Does not start its own thread.
void runSemiFixedScanExecute(const SemiFixedScanExecuteInput &input,
                             SemiFixedScanExecuteHost &host);

} // namespace hf::ur3e
