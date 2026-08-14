// Semi-fixed ring-scan execute: MoveIt hops + stepped shoulder_pan spin (backend).

#include "backend/3dscanning/Ur3eSemiFixedScanExecute.hpp"

#include "backend/HyperFusionConfig.hpp"

#include <QString>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>
#include <vector>

namespace hf::ur3e
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

bool hardwareMoveExact(const QString &serverUrl,
                       const std::vector<double> &joints,
                       const QString &label,
                       QString *errorOut,
                       bool *stoppedOut);

std::vector<double> wristSweepOffsetsRad(const int stepsEachWay, const double stepDeg)
{
    // Non-zero offsets only — center still is separate (same as Auto: (2N)^k + 1).
    std::vector<double> offsets;
    if (stepsEachWay <= 0 || !(stepDeg > 0.0))
        return offsets;
    const double stepRad = stepDeg * kDegToRad;
    for (int i = 1; i <= stepsEachWay; ++i)
    {
        offsets.push_back(static_cast<double>(i) * stepRad);
        offsets.push_back(-static_cast<double>(i) * stepRad);
    }
    return offsets;
}

/// After center still: product of wrist offsets (same policy as Auto), then return to base.
bool runWristSweepAtPose(const QString &serverUrl,
                         const std::vector<double> &baseJoints,
                         const Ur3eWristSweepParams &wrist,
                         const Ur3eScanTcpPose &plannedTcp,
                         const int ringIndex,
                         const int sampleIndex,
                         const int stabilizeMs,
                         SemiFixedScanExecuteHost &host,
                         QString *errorOut,
                         bool *stoppedOut)
{
    if (errorOut != nullptr)
        *errorOut = QString();
    if (stoppedOut != nullptr)
        *stoppedOut = false;
    if (!wrist.enabled || wrist.enabledAxisCount() <= 0 || baseJoints.size() != 6)
        return true;

    if (host.log)
    {
        host.log(QStringLiteral(
                     "UR3e semi-fixed: wrist sweep at ring %1 sample %2 "
                     "(step %3°, ±%4, axes w1=%5 w2=%6 w3=%7)…")
                     .arg(ringIndex)
                     .arg(sampleIndex)
                     .arg(wrist.stepDeg, 0, 'f', 1)
                     .arg(wrist.stepsEachWay)
                     .arg(wrist.wrist1 ? QStringLiteral("on") : QStringLiteral("off"))
                     .arg(wrist.wrist2 ? QStringLiteral("on") : QStringLiteral("off"))
                     .arg(wrist.wrist3 ? QStringLiteral("on") : QStringLiteral("off")));
    }

    const std::vector<double> offsets =
        wristSweepOffsetsRad(wrist.stepsEachWay, wrist.stepDeg);
    if (offsets.empty())
        return true;
    const std::vector<double> axis1 = wrist.wrist1 ? offsets : std::vector<double>{0.0};
    const std::vector<double> axis2 = wrist.wrist2 ? offsets : std::vector<double>{0.0};
    const std::vector<double> axis3 = wrist.wrist3 ? offsets : std::vector<double>{0.0};

    for (const double d1 : axis1)
    {
        for (const double d2 : axis2)
        {
            for (const double d3 : axis3)
            {
                // Disabled axes contribute 0; all-zero only if every axis disabled (guarded above).
                if (d1 == 0.0 && d2 == 0.0 && d3 == 0.0)
                    continue;
                if (host.stopRequested && host.stopRequested())
                {
                    if (stoppedOut != nullptr)
                        *stoppedOut = true;
                    return false;
                }
                std::vector<double> joints = baseJoints;
                joints[3] += d1;
                joints[4] += d2;
                joints[5] += d3;
                // Hardware spin (same as pan / return-to-pin): MoveIt direct_only fails
                // after multi-turn ring entry even for ~4° wrist deltas (OMPL 99999).
                QString moveErr;
                bool moveStopped = false;
                if (!hardwareMoveExact(
                        serverUrl,
                        joints,
                        QStringLiteral("semi-fixed wrist sweep offset"),
                        &moveErr,
                        &moveStopped))
                {
                    if (moveStopped)
                    {
                        if (stoppedOut != nullptr)
                            *stoppedOut = true;
                        if (errorOut != nullptr)
                            *errorOut = moveErr;
                        return false;
                    }
                    if (host.markPinFailed)
                        host.markPinFailed();
                    if (host.log)
                    {
                        host.log(QStringLiteral(
                                     "UR3e semi-fixed: wrist sweep skip — %1")
                                     .arg(moveErr.isEmpty()
                                              ? QStringLiteral("hardware move failed")
                                              : moveErr));
                    }
                    continue;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(stabilizeMs));
                if (host.captureStill && !host.captureStill(plannedTcp, ringIndex, sampleIndex))
                {
                    // Leave wrists at nominal before aborting so retreat/home is not offset.
                    hardwareMoveExact(serverUrl,
                                      baseJoints,
                                      QStringLiteral("semi-fixed return-to-pin after sweep abort"),
                                      nullptr,
                                      stoppedOut);
                    if (errorOut != nullptr)
                        *errorOut = QStringLiteral("wrist sweep capture failed");
                    return false;
                }
                if (host.markPinCompleted)
                    host.markPinCompleted();
            }
        }
    }

    // Always restore nominal pin joints before next sample/pin (hardware — wrist-only deltas).
    if (host.log)
    {
        host.log(QStringLiteral(
                     "UR3e semi-fixed: wrist sweep ring %1 sample %2 — return to nominal…")
                     .arg(ringIndex)
                     .arg(sampleIndex));
    }
    QString returnErr;
    bool returnStopped = false;
    if (!hardwareMoveExact(serverUrl,
                           baseJoints,
                           QStringLiteral("semi-fixed return-to-pin after wrist sweep"),
                           &returnErr,
                           &returnStopped))
    {
        if (stoppedOut != nullptr)
            *stoppedOut = returnStopped;
        if (errorOut != nullptr)
        {
            *errorOut = returnErr.isEmpty()
                            ? QStringLiteral("return to nominal pin after wrist sweep failed")
                            : returnErr;
        }
        return false;
    }
    if (stabilizeMs > 0)
        std::this_thread::sleep_for(std::chrono::milliseconds(stabilizeMs));
    return true;
}

bool moveItToJoints(const QString &serverUrl,
                    const std::vector<double> &joints,
                    const Ur3eScanTcpPose *tcp,
                    QString *errorOut,
                    bool *stoppedOut,
                    const bool allowPinPoseCone = false)
{
    const Ur3eScanWaypointMoveResult move =
        ur3eExecuteScanWaypoint(serverUrl, joints, tcp, nullptr, false, false, allowPinPoseCone);
    if (stoppedOut != nullptr)
        *stoppedOut = move.stopped;
    if (move.stopped)
    {
        if (errorOut != nullptr)
            *errorOut = move.errorMessage.isEmpty() ? QStringLiteral("stopped") : move.errorMessage;
        return false;
    }
    if (move.skipped || !move.ok)
    {
        if (errorOut != nullptr)
            *errorOut = move.errorMessage.isEmpty() ? QStringLiteral("no collision-free path")
                                                    : move.errorMessage;
        return false;
    }
    return true;
}

/// Place *raw* on the continuous branch of *reference* (shortest wrapped delta).
double unwrapContinuous(const double reference, const double raw)
{
    double delta = raw - reference;
    delta = std::atan2(std::sin(delta), std::cos(delta));
    return reference + delta;
}

std::vector<double> unwrapJointsOntoLive(const std::vector<double> &live,
                                         const std::vector<double> &target)
{
    std::vector<double> out = target;
    if (live.size() != 6 || target.size() != 6)
        return out;
    for (int i = 0; i < 6; ++i)
        out[static_cast<std::size_t>(i)] =
            unwrapContinuous(live[static_cast<std::size_t>(i)],
                             target[static_cast<std::size_t>(i)]);
    return out;
}

/// MoveIt / URDF shoulder_pan planning limit (±360°). Soft margin keeps edge
/// off the hard stop (controller −4 / protective stop).
constexpr double kShoulderPanLimitRad = 2.0 * kPi;
constexpr double kShoulderPanLimitMarginRad = 5.0 * kDegToRad;

/// Pick continuous entry pan so [edge → entry] along panDir stays inside ±360°.
/// Principalized entry in (−π, π] can make edge = entry − (N−1)Δ go past −360°
/// (e.g. entry −14.6° → edge −364.6°). Prefer entry±2π when needed.
bool pickPanSweepBranch(const double principalPanRad,
                        const int panDir,
                        const int samplesPerRing,
                        const double intervalRad,
                        double *entryOut,
                        double *edgeOut)
{
    if (entryOut == nullptr || edgeOut == nullptr || samplesPerRing <= 0)
        return false;
    const int dir = panDir >= 0 ? 1 : -1;
    const double span =
        static_cast<double>(std::max(0, samplesPerRing - 1)) * intervalRad;
    const double loLimit = -kShoulderPanLimitRad + kShoulderPanLimitMarginRad;
    const double hiLimit = kShoulderPanLimitRad - kShoulderPanLimitMarginRad;

    const double candidates[3] = {
        principalPanRad,
        principalPanRad + 2.0 * kPi,
        principalPanRad - 2.0 * kPi,
    };

    double bestEntry = principalPanRad;
    double bestEdge = principalPanRad - static_cast<double>(dir) * span;
    bool found = false;
    double bestAbs = 1.0e99;
    for (const double entryCand : candidates)
    {
        const double edgeCand = entryCand - static_cast<double>(dir) * span;
        const double lo = std::min(entryCand, edgeCand);
        const double hi = std::max(entryCand, edgeCand);
        if (lo < loLimit - 1.0e-9 || hi > hiLimit + 1.0e-9)
            continue;
        const double score = std::abs(entryCand);
        if (!found || score < bestAbs - 1.0e-9)
        {
            found = true;
            bestAbs = score;
            bestEntry = entryCand;
            bestEdge = edgeCand;
        }
    }
    if (!found)
        return false;
    *entryOut = bestEntry;
    *edgeOut = bestEdge;
    return true;
}

std::vector<double> configuredHomeJointsRad()
{
    std::vector<double> home(6, 0.0);
    const auto &deg = hf::hardwareConfig().ur3e.homeJointsDeg;
    for (int i = 0; i < 6; ++i)
        home[static_cast<std::size_t>(i)] = deg[static_cast<std::size_t>(i)] * kDegToRad;
    return home;
}

bool hardwareMoveExact(const QString &serverUrl,
                       const std::vector<double> &joints,
                       const QString &label,
                       QString *errorOut,
                       bool *stoppedOut)
{
    const Ur3eScanWaypointMoveResult move =
        ur3eExecuteHardwareJointMove(serverUrl, joints, true, nullptr, label);
    if (stoppedOut != nullptr)
        *stoppedOut = move.stopped;
    if (move.stopped)
    {
        if (errorOut != nullptr)
            *errorOut = move.errorMessage.isEmpty() ? QStringLiteral("stopped") : move.errorMessage;
        return false;
    }
    if (!move.ok)
    {
        if (errorOut != nullptr)
            *errorOut = move.errorMessage.isEmpty() ? QStringLiteral("hardware move failed")
                                                    : move.errorMessage;
        return false;
    }
    return true;
}

bool readLiveJointsRad(const QString &serverUrl, std::vector<double> *out)
{
    if (out == nullptr)
        return false;
    for (int attempt = 0; attempt < 15; ++attempt)
    {
        const Ur3eJointsState live = ur3eGetJoints(serverUrl);
        if (live.ok && live.positionsRad.size() == 6)
        {
            *out = live.positionsRad;
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
}

/// At verified home, clear any continuous wrist_3 turn before another MoveIt hop.
/// The sidecar validates the unwind in small collision-checked samples.
bool rewindWrist3AtHome(const QString &serverUrl,
                        SemiFixedScanExecuteHost &host,
                        const QString &whenLabel,
                        QString *errorOut,
                        bool *stoppedOut)
{
    if (errorOut != nullptr)
        *errorOut = QString();
    if (stoppedOut != nullptr)
        *stoppedOut = false;

    const Ur3eWrist3RewindResult rewind = ur3eRewindWrist3Cable(serverUrl);
    if (rewind.stopped)
    {
        if (stoppedOut != nullptr)
            *stoppedOut = true;
        return false;
    }
    if (!rewind.ok)
    {
        if (errorOut != nullptr)
        {
            *errorOut = rewind.errorMessage.isEmpty()
                            ? QStringLiteral("wrist_3 cable rewind failed")
                            : rewind.errorMessage;
        }
        return false;
    }

    if (rewind.rewound)
    {
        if (host.log)
        {
            host.log(QStringLiteral(
                         "UR3e semi-fixed: wrist_3 cable rewind %1 turn(s) %2.")
                         .arg(rewind.turns > 0 ? QStringLiteral("+%1").arg(rewind.turns)
                                               : QString::number(rewind.turns))
                         .arg(whenLabel));
        }
        if (host.syncHomeSliders)
            host.syncHomeSliders();
    }
    return true;
}

/// If shoulder_pan is outside (−π, π], hardware-spin by ±2π onto the principal branch.
/// Plan JSON / MoveIt hops use this branch; RTDE can be +360° after execute unwrap.
bool principalizeShoulderPan(const QString &serverUrl,
                             SemiFixedScanExecuteHost &host,
                             QString *errorOut,
                             bool *stoppedOut)
{
    std::vector<double> live;
    if (!readLiveJointsRad(serverUrl, &live))
    {
        if (errorOut != nullptr)
            *errorOut = QStringLiteral("could not read joints to principalize pan");
        return false;
    }
    const double pan = live[0];
    const double principal = std::atan2(std::sin(pan), std::cos(pan));
    if (std::abs(principal - pan) < (0.5 * kDegToRad))
        return true;

    if (host.log)
    {
        host.log(QStringLiteral(
                     "UR3e semi-fixed: principalize shoulder_pan %1° → %2° (±360° unwrap)…")
                     .arg(pan * 180.0 / kPi, 0, 'f', 1)
                     .arg(principal * 180.0 / kPi, 0, 'f', 1));
    }
    std::vector<double> target = live;
    target[0] = principal;
    return hardwareMoveExact(serverUrl,
                             target,
                             QStringLiteral("semi-fixed principalize shoulder_pan"),
                             errorOut,
                             stoppedOut);
}

/// Prep continuous joint branch (required entry unwind + verify), then MoveIt
/// collision-aware home from the plan-validated entry — not post-pan wound state.
/// No wrist_3 unwind. Hardware interpolate to home is not used.
/// Returns true when MoveIt home (or joint-hop fallback) reports ok.
bool retreatToScanHome(const QString &serverUrl,
                       SemiFixedScanExecuteHost &host,
                       const bool userStopped,
                       const std::vector<double> *unwindEntryBranch)
{
    constexpr double kEntryVerifyTolRad = 0.08; // ~4.6° wrap-aware joint L2

    if (host.setReturningHome)
        host.setReturningHome(true);
    if (host.clearStopRequested)
        host.clearStopRequested();

    if (host.log)
    {
        host.log(userStopped ? QStringLiteral(
                                   "UR3e semi-fixed: stop — returning to home pose…")
                             : QStringLiteral(
                                   "UR3e semi-fixed: returning to home pose…"));
    }
    if (host.syncHomeSliders)
        host.syncHomeSliders();

    // 1) Hard-require unwind to the exact continuous entry captured after MoveIt
    //    (plan-validated pin↔home start) — not the wound look-alike (entry+360°).
    if (unwindEntryBranch != nullptr && unwindEntryBranch->size() == 6)
    {
        if (host.log)
        {
            host.log(QStringLiteral(
                "UR3e semi-fixed: unwinding to ring-entry branch before MoveIt home…"));
        }
        QString unwindErr;
        bool stopped = false;
        if (!hardwareMoveExact(serverUrl,
                               *unwindEntryBranch,
                               QStringLiteral("semi-fixed unwind to entry branch"),
                               &unwindErr,
                               &stopped))
        {
            if (host.setReturningHome)
                host.setReturningHome(false);
            if (stopped)
                return false;
            if (host.log)
            {
                host.log(QStringLiteral(
                             "UR3e semi-fixed: abort MoveIt home — entry unwind failed: %1")
                             .arg(unwindErr.isEmpty()
                                      ? QStringLiteral("hardware move failed")
                                      : unwindErr));
            }
            return false;
        }

        std::vector<double> live;
        if (!readLiveJointsRad(serverUrl, &live)
            || ur3eJointDistanceRad(*unwindEntryBranch, live) > kEntryVerifyTolRad)
        {
            if (host.setReturningHome)
                host.setReturningHome(false);
            const double dist =
                live.size() == 6 ? ur3eJointDistanceRad(*unwindEntryBranch, live)
                                 : -1.0;
            if (host.log)
            {
                host.log(
                    QStringLiteral(
                        "UR3e semi-fixed: abort MoveIt home — live joints not near "
                        "validated entry (dist=%1 rad, tol=%2)")
                        .arg(dist, 0, 'f', 3)
                        .arg(kEntryVerifyTolRad, 0, 'f', 3));
            }
            return false;
        }
        if (host.log)
        {
            host.log(QStringLiteral(
                "UR3e semi-fixed: at validated entry — MoveIt home from this branch…"));
        }
    }

    // 2) Collision-aware MoveIt to scan home (not hardware interpolate).
    if (host.log)
        host.log(QStringLiteral("UR3e semi-fixed: MoveIt → scan home (collision-aware)…"));
    const Ur3eScanWaypointMoveResult postHome = ur3eExecuteMoveHome(serverUrl);
    if (host.setReturningHome)
        host.setReturningHome(false);

    if (postHome.ok)
    {
        if (host.syncHomeSliders)
            host.syncHomeSliders();
        if (host.log)
            host.log(QStringLiteral("UR3e semi-fixed: at scan home pose."));
        return true;
    }

    // Last resort: if MoveIt rejects the start state, try a MoveIt joint hop to home.
    if (host.log)
    {
        host.log(QStringLiteral(
                     "UR3e semi-fixed: MoveIt home failed (%1) — retrying as joint waypoint…")
                     .arg(postHome.errorMessage.isEmpty()
                              ? QStringLiteral("unknown")
                              : postHome.errorMessage));
    }
    {
        QString hopErr;
        bool hopStopped = false;
        if (moveItToJoints(serverUrl, configuredHomeJointsRad(), nullptr, &hopErr, &hopStopped,
                           false))
        {
            if (host.syncHomeSliders)
                host.syncHomeSliders();
            if (host.log)
                host.log(QStringLiteral("UR3e semi-fixed: at scan home pose (joint hop)."));
            return true;
        }
        if (hopStopped)
            return false;
        if (host.log)
        {
            host.log(QStringLiteral("UR3e semi-fixed warning: %1")
                         .arg(hopErr.isEmpty() ? QStringLiteral("could not return to home")
                                               : hopErr));
        }
    }
    return false;
}

/// Always MoveIt home first (from validated entry when known), then approach
/// with **plan** entry joints only — no live IK / pin-pose cone (avoids wound wrist branch).
bool moveItToRingEntryViaHome(const QString &serverUrl,
                              const std::vector<double> &joints,
                              const Ur3eScanTcpPose *tcp,
                              SemiFixedScanExecuteHost &host,
                              const std::vector<double> *unwindEntryBranch,
                              QString *errorOut,
                              bool *stoppedOut)
{
    if (errorOut != nullptr)
        *errorOut = QString();
    if (stoppedOut != nullptr)
        *stoppedOut = false;

    if (host.log)
    {
        host.log(QStringLiteral(
            "UR3e semi-fixed: home first, then MoveIt → plan ring entry (no live IK)…"));
    }

    if (!retreatToScanHome(serverUrl, host, false, unwindEntryBranch))
    {
        if (errorOut != nullptr)
            *errorOut = QStringLiteral("could not return to home before ring entry");
        return false;
    }

    // Plan joints only — allowPinPoseCone=false disables cone + live IK refresh.
    if (moveItToJoints(serverUrl, joints, tcp, errorOut, stoppedOut, false))
        return true;

    if (stoppedOut != nullptr && *stoppedOut)
        return false;
    if (errorOut != nullptr && errorOut->isEmpty())
        *errorOut = QStringLiteral("no collision-free path from home to plan entry");
    return false;
}

} // namespace

void runSemiFixedScanExecute(const SemiFixedScanExecuteInput &input, SemiFixedScanExecuteHost &host)
{
    int executed = 0;
    QString errorMessage;
    bool ok = true;
    // Always retreat home on Stop and on normal completion (same policy as Auto).
    bool returnHomeAfterScan = true;
    const auto scanStartedAt = std::chrono::steady_clock::now();

    // capturedCount is owned by the host (BFS stills / transforms); pass 0 here.
    const auto finishNow = [&](const bool finishOk, const bool stopped) {
        const qint64 elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                     std::chrono::steady_clock::now() - scanStartedAt)
                                     .count();
        if (host.finish)
            host.finish(finishOk, errorMessage, executed, stopped, 0, elapsedMs);
    };

    const auto sessionOk = [&]() {
        return host.sessionActive && host.sessionActive();
    };
    const auto stopRequested = [&]() {
        return host.stopRequested && host.stopRequested();
    };

    if (host.log)
        host.log(QStringLiteral("UR3e semi-fixed: verifying scan home before scan…"));
    if (host.syncHomeSliders)
        host.syncHomeSliders();

    if (host.ensureHomeBeforeScan && !host.ensureHomeBeforeScan())
    {
        errorMessage = QStringLiteral("Scan aborted — homing cancelled.");
        finishNow(false, false);
        return;
    }

    if (host.syncHomeSliders)
        host.syncHomeSliders();

    // Home verification is wrap-aware, so a physically wound wrist_3 (for example
    // -449° vs the -90° home reference) can still count as "already home". Clear
    // that continuous turn before planning the top pose.
    {
        QString rewindErr;
        bool rewindStopped = false;
        if (!rewindWrist3AtHome(input.serverUrl,
                                host,
                                QStringLiteral("before top still"),
                                &rewindErr,
                                &rewindStopped))
        {
            if (rewindStopped)
            {
                if (stopRequested())
                    ur3eStopMotion(input.serverUrl);
                finishNow(false, true);
                return;
            }
            errorMessage =
                QStringLiteral("Scan aborted — could not unwind wrist_3 at home (%1).")
                    .arg(rewindErr);
            finishNow(false, false);
            return;
        }
    }

    const int ringCount = input.route.rings.size();
    const double intervalDeg =
        input.route.intervalDeg > 0.0 ? input.route.intervalDeg : 10.0;
    const int samplesPerRing = semiFixedSampleCount(intervalDeg);
    const double intervalRad = intervalDeg * kDegToRad;
    const int panDir = input.route.panDirection >= 0 ? 1 : -1;
    const int stabilizeMs =
        input.stabilizeMs > 0
            ? input.stabilizeMs
            : (input.route.stabilizeMs > 0 ? input.route.stabilizeMs : 500);
    const bool captureStills = !input.captureDir.trimmed().isEmpty();

    const auto markPinDone = [&host]() {
        if (host.markPinCompleted)
            host.markPinCompleted();
    };
    const auto markPinFail = [&host]() {
        if (host.markPinFailed)
            host.markPinFailed();
    };

    if (host.log)
    {
        host.log(QStringLiteral(
                     "UR3e semi-fixed execute: top still + %1 ring(s), pan %2° every %3° "
                     "(%4 samples/ring%5)…")
                     .arg(ringCount)
                     .arg(panDir > 0 ? QStringLiteral("+360") : QStringLiteral("−360"))
                     .arg(intervalDeg, 0, 'f', 1)
                     .arg(samplesPerRing)
                     .arg(captureStills ? QStringLiteral(", BFS stills → ") + input.captureDir
                                        : QStringLiteral(", motion-only")));
    }

    // Always: MoveIt → top (θ=0 look-down) → photo, then rings.
    // Preview stores rings at [0..N-1] and top at index N (see inferSemiFixedPreviewRings).
    const int topPreviewIndex = ringCount;
    std::vector<double> lastEntryBranch;
    bool hasLastEntryBranch = false;
    {
        Ur3eSemiFixedRoute routeCopy = input.route;
        ensureSemiFixedTopPose(routeCopy);
        const Ur3eSemiFixedRing &top = routeCopy.topPose;

        if (host.setActiveRing)
            host.setActiveRing(topPreviewIndex);
        if (host.log)
            host.log(QStringLiteral("UR3e semi-fixed: MoveIt → top (θ=0) for first still…"));

        QString moveErr;
        bool stopped = false;
        const Ur3eScanTcpPose *tcpPtr = top.hasEntryTcp ? &top.entryTcp : nullptr;
        // Top / θ=0: exact look-down — no pin-pose cone.
        if (!moveItToJoints(input.serverUrl, top.entryJointsRad, tcpPtr, &moveErr, &stopped,
                            false))
        {
            if (stopped)
            {
                if (stopRequested())
                    ur3eStopMotion(input.serverUrl);
                else
                {
                    ok = false;
                    errorMessage = moveErr;
                }
            }
            else
            {
                ok = false;
                errorMessage = moveErr.isEmpty() ? QStringLiteral("top pose MoveIt failed")
                                                 : moveErr;
            }
            if (host.markRingFailed)
                host.markRingFailed(topPreviewIndex);
            markPinFail();
            goto semi_fixed_done;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(stabilizeMs));

        if (!sessionOk() || stopRequested())
        {
            ur3eStopMotion(input.serverUrl);
            goto semi_fixed_done;
        }

        if (captureStills && host.captureStill)
        {
            Ur3eScanTcpPose planned = top.hasEntryTcp ? top.entryTcp : Ur3eScanTcpPose{};
            if (!host.captureStill(planned, -1, 0))
            {
                ok = false;
                errorMessage = QStringLiteral("top capture failed");
                if (host.markRingFailed)
                    host.markRingFailed(topPreviewIndex);
                markPinFail();
                goto semi_fixed_done;
            }
        }
        markPinDone(); // center pose (top)

        // Wrist sweep whenever enabled (not gated on capture session).
        {
            Ur3eScanTcpPose planned = top.hasEntryTcp ? top.entryTcp : Ur3eScanTcpPose{};
            QString wristErr;
            bool wristStopped = false;
            if (!runWristSweepAtPose(input.serverUrl,
                                     top.entryJointsRad,
                                     input.wristSweep,
                                     planned,
                                     -1,
                                     0,
                                     stabilizeMs,
                                     host,
                                     &wristErr,
                                     &wristStopped))
            {
                if (wristStopped)
                {
                    if (stopRequested())
                        ur3eStopMotion(input.serverUrl);
                    else
                    {
                        ok = false;
                        errorMessage = wristErr;
                    }
                    goto semi_fixed_done;
                }
                ok = false;
                errorMessage = wristErr.isEmpty() ? QStringLiteral("top wrist sweep failed")
                                                 : wristErr;
                markPinFail();
                goto semi_fixed_done;
            }
        }
        ++executed;
        if (host.markRingCompleted)
            host.markRingCompleted(topPreviewIndex);
        if (host.log)
            host.log(QStringLiteral("UR3e semi-fixed: top still done."));
    }

    for (int ringIndex = 0; ringIndex < ringCount; ++ringIndex)
    {
        if (!sessionOk() || stopRequested())
        {
            ur3eStopMotion(input.serverUrl);
            break;
        }

        const Ur3eSemiFixedRing &ring = input.route.rings[ringIndex];
        if (ring.entryJointsRad.size() != 6)
        {
            ok = false;
            errorMessage = QStringLiteral("Ring %1 has invalid joints").arg(ringIndex);
            if (host.markRingFailed)
                host.markRingFailed(ringIndex);
            markPinFail();
            break;
        }

        if (host.setActiveRing)
            host.setActiveRing(ringIndex);
        if (host.log)
        {
            host.log(QStringLiteral("UR3e semi-fixed [%1/%2] MoveIt → ring “%3”…")
                         .arg(ringIndex + 1)
                         .arg(ringCount)
                         .arg(ring.displayName));
        }

        QString moveErr;
        bool stopped = false;
        const Ur3eScanTcpPose *tcpPtr = ring.hasEntryTcp ? &ring.entryTcp : nullptr;
        // Always home first, then plan entry joints (no live IK / cone).
        const std::vector<double> *unwindPtr =
            hasLastEntryBranch ? &lastEntryBranch : nullptr;
        if (!moveItToRingEntryViaHome(input.serverUrl,
                                      ring.entryJointsRad,
                                      tcpPtr,
                                      host,
                                      unwindPtr,
                                      &moveErr,
                                      &stopped))
        {
            if (stopped)
            {
                if (stopRequested())
                    ur3eStopMotion(input.serverUrl);
                else
                {
                    ok = false;
                    errorMessage = moveErr;
                }
            }
            else
            {
                ok = false;
                errorMessage = moveErr;
                if (host.markRingFailed)
                    host.markRingFailed(ringIndex);
                markPinFail();
            }
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(stabilizeMs));

        // MoveIt/plan joints are principal (−π, π]; RTDE may report the same pose ±360°.
        // Unwrap onto the plan branch BEFORE capturing entry / spinning, or the first
        // pan step and the next MoveIt hop leave that branch (controller −4).
        {
            QString prinErr;
            bool prinStopped = false;
            if (!principalizeShoulderPan(input.serverUrl, host, &prinErr, &prinStopped))
            {
                if (prinStopped)
                {
                    if (stopRequested())
                        ur3eStopMotion(input.serverUrl);
                    else
                    {
                        ok = false;
                        errorMessage = prinErr;
                    }
                    goto semi_fixed_done;
                }
                ok = false;
                errorMessage = prinErr.isEmpty()
                                   ? QStringLiteral("principalize shoulder_pan failed")
                                   : prinErr;
                if (host.markRingFailed)
                    host.markRingFailed(ringIndex);
                markPinFail();
                break;
            }
        }

        // Continuous RTDE joints after principalize — never the planned JSON alone.
        std::vector<double> entryBranch;
        if (!readLiveJointsRad(input.serverUrl, &entryBranch))
        {
            ok = false;
            errorMessage = QStringLiteral(
                "Ring %1: could not read live joints after MoveIt entry").arg(ringIndex);
            if (host.markRingFailed)
                host.markRingFailed(ringIndex);
            markPinFail();
            break;
        }
        lastEntryBranch = entryBranch;
        hasLastEntryBranch = true;
        if (host.log)
        {
            host.log(QStringLiteral(
                         "UR3e semi-fixed ring %1: entry branch pan=%2° wrist_3=%3° "
                         "(live RTDE, principal)")
                         .arg(ringIndex)
                         .arg(entryBranch[0] * 180.0 / kPi, 0, 'f', 1)
                         .arg(entryBranch[5] * 180.0 / kPi, 0, 'f', 1));
        }

        // Imaging start edge: opposite the scan direction so interval steps only
        // advance in panDir (avoids mid-ring reverse when RTDE wraps) and the last
        // sample lands on entry. +dir → edge ≈ entry−(N−1)Δ; −dir → entry+(N−1)Δ.
        // Choose continuous entry±2π so the whole sweep stays inside MoveIt
        // shoulder_pan ±360° (principal entry alone can put edge past −360°).
        double sweepEntryPan = entryBranch[0];
        double edgePan = entryBranch[0];
        if (!pickPanSweepBranch(entryBranch[0],
                                panDir,
                                samplesPerRing,
                                intervalRad,
                                &sweepEntryPan,
                                &edgePan))
        {
            ok = false;
            errorMessage = QStringLiteral(
                "Ring %1: pan sweep would exceed shoulder_pan ±360° "
                "(entry %2°)")
                               .arg(ringIndex)
                               .arg(entryBranch[0] * 180.0 / kPi, 0, 'f', 1);
            if (host.markRingFailed)
                host.markRingFailed(ringIndex);
            markPinFail();
            break;
        }
        if (std::abs(sweepEntryPan - entryBranch[0]) >= (0.5 * kDegToRad))
        {
            if (host.log)
            {
                host.log(QStringLiteral(
                             "UR3e semi-fixed ring %1: shift entry pan %2° → %3° "
                             "so init sweep stays within ±360°…")
                             .arg(ringIndex)
                             .arg(entryBranch[0] * 180.0 / kPi, 0, 'f', 1)
                             .arg(sweepEntryPan * 180.0 / kPi, 0, 'f', 1));
            }
            std::vector<double> shifted = entryBranch;
            shifted[0] = sweepEntryPan;
            QString shiftErr;
            bool shiftStopped = false;
            if (!hardwareMoveExact(input.serverUrl,
                                   shifted,
                                   QStringLiteral("semi-fixed shift pan for ±360° sweep"),
                                   &shiftErr,
                                   &shiftStopped))
            {
                if (shiftStopped)
                {
                    if (stopRequested())
                        ur3eStopMotion(input.serverUrl);
                    else
                    {
                        ok = false;
                        errorMessage = shiftErr;
                    }
                    goto semi_fixed_done;
                }
                ok = false;
                errorMessage = shiftErr.isEmpty()
                                   ? QStringLiteral("pan branch shift for sweep failed")
                                   : shiftErr;
                if (host.markRingFailed)
                    host.markRingFailed(ringIndex);
                markPinFail();
                break;
            }
            entryBranch = shifted;
            lastEntryBranch = entryBranch;
            std::this_thread::sleep_for(std::chrono::milliseconds(stabilizeMs));
        }
        if (std::abs(edgePan - entryBranch[0]) >= (0.5 * kDegToRad))
        {
            if (host.log)
            {
                host.log(QStringLiteral(
                             "UR3e semi-fixed ring %1: pan to scan edge %2° "
                             "(from entry %3°, dir %4)…")
                             .arg(ringIndex)
                             .arg(edgePan * 180.0 / kPi, 0, 'f', 1)
                             .arg(entryBranch[0] * 180.0 / kPi, 0, 'f', 1)
                             .arg(panDir > 0 ? QStringLiteral("+") : QStringLiteral("−")));
            }
            std::vector<double> liveForEdge;
            if (!readLiveJointsRad(input.serverUrl, &liveForEdge) || liveForEdge.size() != 6)
            {
                ok = false;
                errorMessage = QStringLiteral("could not read joints before pan to edge");
                if (host.markRingFailed)
                    host.markRingFailed(ringIndex);
                markPinFail();
                goto semi_fixed_done;
            }
            // Pan-only: keep live wrists/elbow (do not yank wrist_3 back to entryBranch).
            std::vector<double> edgeJoints = liveForEdge;
            edgeJoints[0] = edgePan;
            QString edgeErr;
            bool edgeStopped = false;
            if (!hardwareMoveExact(input.serverUrl,
                                   edgeJoints,
                                   QStringLiteral("semi-fixed pan to scan edge"),
                                   &edgeErr,
                                   &edgeStopped))
            {
                if (edgeStopped)
                {
                    if (stopRequested())
                        ur3eStopMotion(input.serverUrl);
                    else
                    {
                        ok = false;
                        errorMessage = edgeErr;
                    }
                    goto semi_fixed_done;
                }
                ok = false;
                errorMessage =
                    edgeErr.isEmpty() ? QStringLiteral("pan to scan edge failed") : edgeErr;
                if (host.markRingFailed)
                    host.markRingFailed(ringIndex);
                markPinFail();
                goto semi_fixed_done;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(stabilizeMs));
        }

        // Absolute continuous targets from edge (never live+Δ — wrapped live caused
        // mid-ring reverse sweeps in the logs, e.g. 270° → −89° → −71°).
        for (int sample = 0; sample < samplesPerRing; ++sample)
        {
            if (!sessionOk() || stopRequested())
            {
                ur3eStopMotion(input.serverUrl);
                goto semi_fixed_done;
            }

            std::vector<double> sampleJoints = entryBranch;
            sampleJoints[0] =
                edgePan
                + static_cast<double>(sample) * intervalRad * static_cast<double>(panDir);

            if (sample > 0)
            {
                if (host.log)
                {
                    host.log(QStringLiteral(
                                 "UR3e semi-fixed ring %1: pan sample %2/%3 → %4° "
                                 "(step %+5°)…")
                                 .arg(ringIndex)
                                 .arg(sample + 1)
                                 .arg(samplesPerRing)
                                 .arg(sampleJoints[0] * 180.0 / kPi, 0, 'f', 1)
                                 .arg(intervalDeg * panDir, 0, 'f', 1));
                }

                const Ur3eScanWaypointMoveResult spin = ur3eExecuteHardwareJointMove(
                    input.serverUrl,
                    sampleJoints,
                    true, // skip collision — operator guarantees clear ring
                    nullptr,
                    QStringLiteral("semi-fixed shoulder_pan step"));
                if (spin.stopped)
                {
                    if (stopRequested())
                        ur3eStopMotion(input.serverUrl);
                    else
                    {
                        ok = false;
                        errorMessage = spin.errorMessage.isEmpty() ? QStringLiteral("spin stopped")
                                                                   : spin.errorMessage;
                    }
                    goto semi_fixed_done;
                }
                if (!spin.ok)
                {
                    ok = false;
                    errorMessage = spin.errorMessage.isEmpty()
                                       ? QStringLiteral("shoulder_pan step failed")
                                       : spin.errorMessage;
                    if (host.markRingFailed)
                        host.markRingFailed(ringIndex);
                    markPinFail();
                    goto semi_fixed_done;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(stabilizeMs));
            }
            else if (host.log)
            {
                host.log(QStringLiteral(
                             "UR3e semi-fixed ring %1: pan sample 1/%2 at edge %3°")
                             .arg(ringIndex)
                             .arg(samplesPerRing)
                             .arg(sampleJoints[0] * 180.0 / kPi, 0, 'f', 1));
            }

            if (captureStills && host.captureStill)
            {
                Ur3eScanTcpPose planned = ring.hasEntryTcp ? ring.entryTcp : Ur3eScanTcpPose{};
                if (!host.captureStill(planned, ringIndex, sample))
                {
                    ok = false;
                    errorMessage = QStringLiteral("capture failed");
                    markPinFail();
                    goto semi_fixed_done;
                }
            }
            markPinDone(); // center pose (pan sample)

            // Wrist sweep whenever enabled (not gated on capture session).
            {
                Ur3eScanTcpPose planned = ring.hasEntryTcp ? ring.entryTcp : Ur3eScanTcpPose{};
                QString wristErr;
                bool wristStopped = false;
                if (!runWristSweepAtPose(input.serverUrl,
                                         sampleJoints,
                                         input.wristSweep,
                                         planned,
                                         ringIndex,
                                         sample,
                                         stabilizeMs,
                                         host,
                                         &wristErr,
                                         &wristStopped))
                {
                    if (wristStopped)
                    {
                        if (stopRequested())
                            ur3eStopMotion(input.serverUrl);
                        else
                        {
                            ok = false;
                            errorMessage = wristErr;
                        }
                        goto semi_fixed_done;
                    }
                    ok = false;
                    errorMessage =
                        wristErr.isEmpty() ? QStringLiteral("wrist sweep failed") : wristErr;
                    markPinFail();
                    goto semi_fixed_done;
                }
            }

            ++executed;
        }

        // Return to entry (last sample should already be there for a full ring).
        if (sessionOk() && !stopRequested())
        {
            if (host.log)
            {
                host.log(QStringLiteral(
                             "UR3e semi-fixed ring %1: return to entry branch "
                             "(pan %2°)…")
                             .arg(ringIndex)
                             .arg(entryBranch[0] * 180.0 / kPi, 0, 'f', 1));
            }
            QString backErr;
            bool backStopped = false;
            if (!hardwareMoveExact(input.serverUrl,
                                   entryBranch,
                                   QStringLiteral("semi-fixed return to ring entry"),
                                   &backErr,
                                   &backStopped))
            {
                if (backStopped)
                {
                    if (stopRequested())
                        ur3eStopMotion(input.serverUrl);
                    goto semi_fixed_done;
                }
                if (host.log)
                {
                    host.log(QStringLiteral(
                                 "UR3e semi-fixed ring %1: return-to-entry warning — %2")
                                 .arg(ringIndex)
                                 .arg(backErr));
                }
            }

            // Stay on principal pan so the next ring MoveIt matches planned joints.
            {
                QString prinErr;
                bool prinStopped = false;
                if (!principalizeShoulderPan(input.serverUrl, host, &prinErr, &prinStopped))
                {
                    if (prinStopped)
                    {
                        if (stopRequested())
                            ur3eStopMotion(input.serverUrl);
                        goto semi_fixed_done;
                    }
                    if (host.log && !prinErr.isEmpty())
                    {
                        host.log(QStringLiteral(
                                     "UR3e semi-fixed ring %1: principalize warning — %2")
                                     .arg(ringIndex)
                                     .arg(prinErr));
                    }
                }
                std::vector<double> afterPrin;
                if (readLiveJointsRad(input.serverUrl, &afterPrin))
                {
                    lastEntryBranch = afterPrin;
                    hasLastEntryBranch = true;
                }
            }
        }

        if (host.markRingCompleted)
            host.markRingCompleted(ringIndex);
    }

semi_fixed_done:
    if (!sessionOk())
        return;

    const bool userStopped = stopRequested();
    returnHomeAfterScan = true;

    bool homeOk = false;
    if (returnHomeAfterScan)
    {
        const std::vector<double> *entryPtr =
            hasLastEntryBranch ? &lastEntryBranch : nullptr;
        homeOk = retreatToScanHome(input.serverUrl, host, userStopped, entryPtr);
    }

    // Leave the robot on the configured continuous home branch. Without this,
    // wrap-aware homing can finish at wrist_3 home±360° and poison the next scan.
    if (homeOk && !userStopped && !stopRequested())
    {
        QString rewindErr;
        bool rewindStopped = false;
        if (!rewindWrist3AtHome(input.serverUrl,
                                host,
                                QStringLiteral("after scan"),
                                &rewindErr,
                                &rewindStopped))
        {
            if (rewindStopped)
            {
                if (stopRequested())
                    ur3eStopMotion(input.serverUrl);
            }
            else if (ok)
            {
                ok = false;
                errorMessage =
                    QStringLiteral("Scan motion completed, but wrist_3 home unwind failed (%1).")
                        .arg(rewindErr);
            }
            else if (host.log)
            {
                host.log(QStringLiteral(
                             "UR3e semi-fixed warning: wrist_3 home unwind failed — %1")
                             .arg(rewindErr));
            }
        }
    }

    // MoveIt pin-hop errors often say "could not retreat to home" even when our
    // post-scan home path actually succeeded — clarify the finish message.
    if (!ok && homeOk && errorMessage.contains(QStringLiteral("retreat to home"),
                                               Qt::CaseInsensitive))
    {
        errorMessage = QStringLiteral(
                           "ring MoveIt hop failed after partial scan; returned to home. "
                           "(%1)")
                           .arg(errorMessage);
    }
    else if (!ok && homeOk)
    {
        errorMessage = QStringLiteral("%1 (returned to scan home).").arg(errorMessage);
    }

    finishNow(ok, userStopped || stopRequested());
}

} // namespace hf::ur3e
