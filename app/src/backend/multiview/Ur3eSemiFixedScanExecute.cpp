// Semi-fixed ring-scan execute: MoveIt hops + stepped shoulder_pan spin (backend).

#include "backend/multiview/Ur3eSemiFixedScanExecute.hpp"

#include "backend/HyperFusionConfig.hpp"
#include "backend/multiview/Ur3eHemisphereScanReachability.hpp"
#include "backend/multiview/Ur3eWorkspaceBoundary.hpp"

#include <QJsonArray>
#include <QJsonObject>
#include <QString>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <thread>
#include <vector>

namespace hf::ur3e
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

/// Joint-space PTP through an elbow sign change folds the payload through the arm.
bool sameElbowFamily(const std::vector<double> &a, const std::vector<double> &b)
{
    if (a.size() < 3 || b.size() < 3)
        return true;
    const double ea = a[2];
    const double eb = b[2];
    constexpr double kNearZeroRad = 5.0 * kDegToRad;
    if (std::abs(ea) < kNearZeroRad || std::abs(eb) < kNearZeroRad)
        return true;
    return (ea * eb) > 0.0;
}

bool jointsNearEqual(const std::vector<double> &a, const std::vector<double> &b,
                     const double tolRad = 2.0 * kDegToRad)
{
    if (a.size() != 6 || b.size() != 6)
        return false;
    for (int i = 0; i < 6; ++i)
    {
        const double d = std::atan2(std::sin(a[static_cast<std::size_t>(i)]
                                             - b[static_cast<std::size_t>(i)]),
                                    std::cos(a[static_cast<std::size_t>(i)]
                                             - b[static_cast<std::size_t>(i)]));
        if (std::abs(d) > tolRad)
            return false;
    }
    return true;
}

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
                if (host.stopRequested && host.stopRequested())
                {
                    if (stoppedOut != nullptr)
                        *stoppedOut = true;
                    hardwareMoveExact(serverUrl,
                                      baseJoints,
                                      QStringLiteral("semi-fixed return-to-pin after sweep abort"),
                                      nullptr,
                                      stoppedOut);
                    return false;
                }
                if (host.captureStill && !host.captureStill(plannedTcp, ringIndex, sampleIndex))
                {
                    // Leave wrists at nominal before aborting so retreat/home is not offset.
                    hardwareMoveExact(serverUrl,
                                      baseJoints,
                                      QStringLiteral("semi-fixed return-to-pin after sweep abort"),
                                      nullptr,
                                      stoppedOut);
                    if (host.stopRequested && host.stopRequested())
                    {
                        if (stoppedOut != nullptr)
                            *stoppedOut = true;
                        return false;
                    }
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

/// Prefer an elbow branch that does not cross 0° from *live* (fold-safe PTP).
double unwrapElbowAvoidFold(const double liveElbow, const double targetPrincipal)
{
    constexpr double kNearZero = 5.0 * kDegToRad;
    if (std::abs(liveElbow) < kNearZero || std::abs(targetPrincipal) < kNearZero)
        return unwrapContinuous(liveElbow, targetPrincipal);

    double best = unwrapContinuous(liveElbow, targetPrincipal);
    double bestAbs = std::abs(best - liveElbow);
    bool bestCrosses = (liveElbow < 0.0 && best > 0.0) || (liveElbow > 0.0 && best < 0.0);

    for (int k = -2; k <= 2; ++k)
    {
        const double cand = targetPrincipal + static_cast<double>(k) * 2.0 * kPi;
        const double absDelta = std::abs(cand - liveElbow);
        const bool crosses = (liveElbow < 0.0 && cand > 0.0) || (liveElbow > 0.0 && cand < 0.0);
        if (crosses)
            continue;
        if (bestCrosses || absDelta + 1.0e-9 < bestAbs)
        {
            best = cand;
            bestAbs = absDelta;
            bestCrosses = false;
        }
    }
    return best;
}

std::vector<double> unwrapJointsOntoLive(const std::vector<double> &live,
                                         const std::vector<double> &target)
{
    std::vector<double> out = target;
    if (live.size() != 6 || target.size() != 6)
        return out;
    for (int i = 0; i < 6; ++i)
    {
        if (i == 2)
        {
            out[static_cast<std::size_t>(i)] =
                unwrapElbowAvoidFold(live[static_cast<std::size_t>(i)],
                                     target[static_cast<std::size_t>(i)]);
        }
        else
        {
            out[static_cast<std::size_t>(i)] =
                unwrapContinuous(live[static_cast<std::size_t>(i)],
                                 target[static_cast<std::size_t>(i)]);
        }
    }
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

int nearestPanMaskBin(const std::vector<std::uint8_t> &mask, const double panRad)
{
    const int n = static_cast<int>(mask.size());
    if (n <= 0)
        return -1;
    const double wrapped = std::atan2(std::sin(panRad), std::cos(panRad));
    int best = 0;
    double bestAbs = 1.0e99;
    for (int i = 0; i < n; ++i)
    {
        const double binPan = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(n);
        const double binW = std::atan2(std::sin(binPan), std::cos(binPan));
        const double d = std::abs(std::atan2(std::sin(binW - wrapped),
                                             std::cos(binW - wrapped)));
        if (d < bestAbs)
        {
            bestAbs = d;
            best = i;
        }
    }
    return best;
}

/// Contiguous valid abs-pan run containing the entry, as continuous radians.
std::vector<double> backupContiguousPans(const double entryPan,
                                         const std::vector<std::uint8_t> &mask,
                                         const int panDir,
                                         const double intervalDeg)
{
    std::vector<double> pans;
    const int n = static_cast<int>(mask.size());
    if (n <= 0)
    {
        pans.push_back(entryPan);
        return pans;
    }
    int entryBin = nearestPanMaskBin(mask, entryPan);
    if (entryBin < 0)
    {
        pans.push_back(entryPan);
        return pans;
    }
    if (mask[static_cast<std::size_t>(entryBin)] == 0)
    {
        int found = -1;
        for (int d = 1; d < n; ++d)
        {
            const int a = (entryBin + d) % n;
            const int b = (entryBin - d + n) % n;
            if (mask[static_cast<std::size_t>(a)] != 0)
            {
                found = a;
                break;
            }
            if (mask[static_cast<std::size_t>(b)] != 0)
            {
                found = b;
                break;
            }
        }
        if (found < 0)
        {
            pans.push_back(entryPan);
            return pans;
        }
        entryBin = found;
    }

    int start = entryBin;
    for (int k = 0; k < n - 1; ++k)
    {
        const int prev = (start - 1 + n) % n;
        if (mask[static_cast<std::size_t>(prev)] == 0)
            break;
        if (prev == entryBin)
            break;
        start = prev;
    }

    std::vector<int> bins;
    int i = start;
    for (int k = 0; k < n; ++k)
    {
        if (mask[static_cast<std::size_t>(i)] == 0)
            break;
        bins.push_back(i);
        i = (i + 1) % n;
        if (i == start)
            break;
    }
    if (bins.empty())
    {
        pans.push_back(entryPan);
        return pans;
    }
    if (panDir < 0)
        std::reverse(bins.begin(), bins.end());

    auto wrapAbs = [n](const int bin) {
        const double raw = 2.0 * kPi * static_cast<double>(bin) / static_cast<double>(n);
        return std::atan2(std::sin(raw), std::cos(raw));
    };

    double prev = unwrapContinuous(entryPan, wrapAbs(bins.front()));
    const double loLimit = -kShoulderPanLimitRad + kShoulderPanLimitMarginRad;
    const double hiLimit = kShoulderPanLimitRad - kShoulderPanLimitMarginRad;
    for (const int shift : {0, 1, -1})
    {
        const double trial = prev + static_cast<double>(shift) * 2.0 * kPi;
        if (trial >= loLimit - 1.0e-9 && trial <= hiLimit + 1.0e-9)
        {
            prev = trial;
            break;
        }
    }

    pans.reserve(bins.size());
    for (std::size_t k = 0; k < bins.size(); ++k)
    {
        const double absPan = wrapAbs(bins[k]);
        const double cont = unwrapContinuous(k == 0 ? prev : pans.back(), absPan);
        pans.push_back(cont);
    }
    if (pans.empty())
        return pans;

    const double stepDeg = intervalDeg > 0.0 ? intervalDeg : 10.0;
    const double arcDeg =
        360.0 * static_cast<double>(bins.size()) / static_cast<double>(n);
    const int samples = std::max(1, static_cast<int>(std::lround(arcDeg / stepDeg)));
    const double existingDir =
        (pans.size() >= 2 && pans[1] + 1.0e-9 < pans[0]) || panDir < 0 ? -1.0
                                                                       : 1.0;
    const double arcStart = pans.front();
    const double stepRad = stepDeg * kDegToRad * existingDir;
    std::vector<double> sampled;
    sampled.reserve(static_cast<std::size_t>(samples));
    const double maxSpan = arcDeg * kDegToRad + 1.0e-9;
    for (int i = 0; i < samples; ++i)
    {
        const double p = arcStart + static_cast<double>(i) * stepRad;
        if (std::abs(p - arcStart) > maxSpan)
            break;
        sampled.push_back(p);
    }
    if (sampled.empty())
        sampled.push_back(entryPan);
    return sampled;
}

bool sameRouteRingTheta(const Ur3eSemiFixedRing &a, const Ur3eSemiFixedRing &b)
{
    if (std::abs(a.thetaDeg) < 0.75 || std::abs(b.thetaDeg) < 0.75)
        return false;
    return std::lround(a.thetaDeg * 2.0) == std::lround(b.thetaDeg * 2.0);
}

std::vector<double> configuredHomeJointsRad()
{
    std::vector<double> home(6, 0.0);
    const auto &deg = hf::hardwareConfig().ur3e.homeJointsDeg;
    for (int i = 0; i < 6; ++i)
        home[static_cast<std::size_t>(i)] = deg[static_cast<std::size_t>(i)] * kDegToRad;
    return home;
}

/// First non-RGB ring entry — FPP scan hub (start/end) instead of cfg home.
std::vector<double> fppSweepHomeJointsRad(const Ur3eSemiFixedRoute &route)
{
    for (const Ur3eSemiFixedRing &ring : route.rings)
    {
        if (ring.captureKind.trimmed().compare(QStringLiteral("rgb"), Qt::CaseInsensitive)
            == 0)
            continue;
        if (ring.entryJointsRad.size() == 6)
            return ring.entryJointsRad;
    }
    return {};
}

bool planApexJointsOnRingSphere(const QString &serverUrl,
                                Ur3eSemiFixedRing &top,
                                QString *errorOut)
{
    if (!top.hasEntryTcp)
    {
        if (errorOut != nullptr)
            *errorOut = QStringLiteral("apex TCP missing");
        return false;
    }

    Ur3eHemisphereScanPoint grid;
    grid.thetaDeg = 0.0;
    grid.phiDeg = 0.0;
    grid.xM = top.entryTcp.xM;
    grid.yM = top.entryTcp.yM;
    grid.zM = top.entryTcp.zM;
    const Ur3eScanTcpPose tcp = tcpPoseForHemispherePoint(grid);
    top.entryTcp = tcp;
    top.hasEntryTcp = true;

    QJsonObject pose;
    pose.insert(QStringLiteral("index"), 0);
    const Ur3eScanTcpPose ee = tcp;
    pose.insert(QStringLiteral("x"), ee.xM);
    pose.insert(QStringLiteral("y"), ee.yM);
    pose.insert(QStringLiteral("z"), ee.zM);
    pose.insert(QStringLiteral("rx"), ee.rxRad);
    pose.insert(QStringLiteral("ry"), ee.ryRad);
    pose.insert(QStringLiteral("rz"), ee.rzRad);
    pose.insert(QStringLiteral("tool_z_x"), ee.toolZMx);
    pose.insert(QStringLiteral("tool_z_y"), ee.toolZMy);
    pose.insert(QStringLiteral("tool_z_z"), ee.toolZMz);
    pose.insert(QStringLiteral("require_perpendicular"), true);
    double upX = 0.0;
    double upY = 0.0;
    double upZ = 1.0;
    homeApexCameraUpWorld(upX, upY, upZ);
    pose.insert(QStringLiteral("camera_up_x"), upX);
    pose.insert(QStringLiteral("camera_up_y"), upY);
    pose.insert(QStringLiteral("camera_up_z"), upZ);

    const Ur3eWorkspaceBoundary boundary =
        workspaceBoundaryFromConfig(hf::hardwareConfig().ur3e);
    QJsonObject workspace;
    workspace.insert(QStringLiteral("enabled"), boundary.enabled);
    workspace.insert(QStringLiteral("length_m"), boundary.lengthM());
    workspace.insert(QStringLiteral("width_m"), boundary.widthM());
    workspace.insert(QStringLiteral("height_m"), boundary.heightM());
    workspace.insert(QStringLiteral("mount_height_m"), boundary.mountHeightM());
    workspace.insert(QStringLiteral("ceiling_clearance_m"), boundary.ceilingClearanceM());

    QJsonArray poses;
    poses.append(pose);
    QJsonObject body;
    body.insert(QStringLiteral("poses"), poses);
    body.insert(QStringLiteral("workspace"), workspace);
    body.insert(QStringLiteral("pin_pose_tolerance_deg"), 0.0);
    body.insert(QStringLiteral("scan_camera_up_world_z"),
                hf::hardwareConfig().ur3e.scanCameraUpWorldZ);
    appendUr3eScanHomeJointsToJson(body);

    QString planErr;
    const int planTimeoutMs = std::max(60000, hf::hardwareConfig().ur3e.planTimeoutMs);
    const QJsonObject response =
        ur3ePostJsonRequest(serverUrl, QStringLiteral("/plan_hemisphere_scan"), body,
                            planTimeoutMs, &planErr);
    if (response.isEmpty() || !response.value(QStringLiteral("ok")).toBool(false))
    {
        if (errorOut != nullptr)
            *errorOut = planErr.isEmpty() ? QStringLiteral("apex plan failed") : planErr;
        return false;
    }

    const QJsonArray results = response.value(QStringLiteral("results")).toArray();
    if (results.isEmpty())
    {
        if (errorOut != nullptr)
            *errorOut = QStringLiteral("apex plan returned no results");
        return false;
    }
    const QJsonObject entry = results.at(0).toObject();
    if (!entry.value(QStringLiteral("reachable")).toBool(false)
        || !entry.value(QStringLiteral("home_path_ok")).toBool(true))
    {
        if (errorOut != nullptr)
        {
            const QString err = entry.value(QStringLiteral("error")).toString();
            *errorOut = err.isEmpty() ? QStringLiteral("apex unreachable at ring R") : err;
        }
        return false;
    }
    const QJsonArray joints = entry.value(QStringLiteral("joints")).toArray();
    if (joints.size() != 6)
    {
        if (errorOut != nullptr)
            *errorOut = QStringLiteral("apex plan missing joints");
        return false;
    }
    top.entryJointsRad.clear();
    for (const QJsonValue &j : joints)
        top.entryJointsRad.push_back(j.toDouble());
    top.reachable = true;
    top.homePathOk = true;
    top.reachabilityKnown = true;
    return true;
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
                       const std::vector<double> *unwindEntryBranch,
                       const std::vector<double> *homeJointsOverride = nullptr)
{
    constexpr double kEntryVerifyTolRad = 0.08; // ~4.6° wrap-aware joint L2

    if (host.setReturningHome)
        host.setReturningHome(true);
    if (host.clearStopRequested)
        host.clearStopRequested();

    const bool fppSweepHome =
        homeJointsOverride != nullptr && homeJointsOverride->size() == 6;

    if (host.log)
    {
        host.log(userStopped
                     ? (fppSweepHome
                            ? QStringLiteral("UR3e FPP: stop — returning to sweep home…")
                            : QStringLiteral(
                                  "UR3e semi-fixed: stop — returning to home pose…"))
                     : (fppSweepHome
                            ? QStringLiteral("UR3e FPP: returning to sweep home…")
                            : QStringLiteral(
                                  "UR3e semi-fixed: returning to home pose…")));
    }
    if (!fppSweepHome && host.syncHomeSliders)
        host.syncHomeSliders();

    // 1) Unwind to the continuous entry branch (plan-validated), then go home.
    // FPP hub case: ring-entry joints == sweep home. Mid-pan live ≠ home — do NOT
    // treat "entry == home" as already home; skip the redundant unwind and fall
    // through to the live check + hardware move to sweep home below.
    if (unwindEntryBranch != nullptr && unwindEntryBranch->size() == 6)
    {
        const bool entryIsSweepHome =
            fppSweepHome && jointsNearEqual(*unwindEntryBranch, *homeJointsOverride);
        if (!entryIsSweepHome)
        {
            if (host.log)
            {
                host.log(fppSweepHome
                             ? QStringLiteral(
                                   "UR3e FPP: unwinding to ring-entry before sweep home…")
                             : QStringLiteral(
                                   "UR3e semi-fixed: unwinding to ring-entry branch before "
                                   "MoveIt home…"));
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
                                 "UR3e semi-fixed: abort home — entry unwind failed: %1")
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
                            "UR3e semi-fixed: abort home — live joints not near "
                            "validated entry (dist=%1 rad, tol=%2)")
                            .arg(dist, 0, 'f', 3)
                            .arg(kEntryVerifyTolRad, 0, 'f', 3));
                }
                return false;
            }
        }
        else if (host.log)
        {
            host.log(QStringLiteral(
                "UR3e FPP: ring entry is sweep home — recovering from live pan…"));
        }
    }

    if (fppSweepHome)
    {
        std::vector<double> live;
        if (readLiveJointsRad(serverUrl, &live)
            && jointsNearEqual(live, *homeJointsOverride))
        {
            if (host.setReturningHome)
                host.setReturningHome(false);
            if (host.log)
                host.log(QStringLiteral("UR3e FPP: already at sweep home."));
            return true;
        }
        if (host.log)
            host.log(QStringLiteral("UR3e FPP: hardware → sweep home…"));
        QString homeErr;
        bool homeStopped = false;
        const bool okMove = hardwareMoveExact(serverUrl,
                                              *homeJointsOverride,
                                              QStringLiteral("FPP retreat to sweep home"),
                                              &homeErr,
                                              &homeStopped);
        if (host.setReturningHome)
            host.setReturningHome(false);
        if (homeStopped)
            return false;
        if (!okMove)
        {
            if (host.log)
            {
                host.log(QStringLiteral("UR3e FPP: sweep home failed — %1")
                             .arg(homeErr.isEmpty()
                                      ? QStringLiteral("hardware move failed")
                                      : homeErr));
            }
            return false;
        }
        if (host.log)
            host.log(QStringLiteral("UR3e FPP: at sweep home."));
        return true;
    }

    // 2) Collision-aware MoveIt to configured scan home (Semi hub).
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

    // FPP hub = plan sweep pose (first non-RGB ring), not cfg home_joints_deg.
    const std::vector<double> fppSweepHome =
        input.route.isFppPlan ? fppSweepHomeJointsRad(input.route) : std::vector<double>{};
    const bool useFppSweepHome = fppSweepHome.size() == 6;

    if (host.log)
    {
        host.log(useFppSweepHome
                     ? QStringLiteral("UR3e FPP: verifying sweep home before scan…")
                     : QStringLiteral("UR3e semi-fixed: verifying scan home before scan…"));
    }
    if (!useFppSweepHome && host.syncHomeSliders)
        host.syncHomeSliders();

    if (useFppSweepHome)
    {
        std::vector<double> live;
        const bool atSweep = readLiveJointsRad(input.serverUrl, &live)
                             && jointsNearEqual(live, fppSweepHome);
        if (!atSweep)
        {
            if (host.log)
                host.log(QStringLiteral("UR3e FPP: hardware → sweep home before scan…"));
            QString homeErr;
            bool homeStopped = false;
            if (!hardwareMoveExact(input.serverUrl,
                                   fppSweepHome,
                                   QStringLiteral("FPP move to sweep home"),
                                   &homeErr,
                                   &homeStopped))
            {
                if (homeStopped && stopRequested())
                    ur3eStopMotion(input.serverUrl);
                errorMessage =
                    homeErr.isEmpty()
                        ? QStringLiteral("Scan aborted — could not reach FPP sweep home.")
                        : QStringLiteral("Scan aborted — FPP sweep home failed: %1")
                              .arg(homeErr);
                finishNow(false, homeStopped && stopRequested());
                return;
            }
        }
        else if (host.log)
        {
            host.log(QStringLiteral("UR3e FPP: already at sweep home."));
        }
    }
    else if (host.ensureHomeBeforeScan && !host.ensureHomeBeforeScan())
    {
        errorMessage = QStringLiteral("Scan aborted — homing cancelled.");
        finishNow(false, false);
        return;
    }

    if (!useFppSweepHome && host.syncHomeSliders)
        host.syncHomeSliders();

    // Clear wound wrist_3 at cfg home. Skip for FPP (rewinder targets cfg home).
    if (!useFppSweepHome)
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

    Ur3eSemiFixedRoute route = input.route;
    pruneSemiFixedRedundantFullSpinPins(route);
    // Fold RGB sweep after prune so same-θ FPP+RGB both run.
    foldRgbEntriesIntoRings(route);

    const int ringCount = route.rings.size();
    const bool apexRingOnly =
        ringCount == 1
        && (route.rings[0].noPan || std::abs(route.rings[0].thetaDeg) < 0.75);
    bool doTop = !input.skipTop && !(apexRingOnly && !input.skipRings);
    // Sweep-only FPP plans omit top_pose — do not invent a home still.
    if (route.isFppPlan && !route.hasTopPose)
        doTop = false;
    if (doTop)
    {
        Ur3eSemiFixedRoute apexProbe = route;
        ensureSemiFixedTopPose(apexProbe);
        if (apexProbe.topPose.entryJointsRad.size() != 6)
        {
            if (route.isFppPlan)
            {
                // Explicit top_pose without joints is invalid for FPP; skip apex.
                doTop = false;
                if (host.log)
                    host.log(QStringLiteral(
                        "UR3e FPP: top_pose missing joints — skipping apex still."));
            }
            else
            {
                if (host.log)
                    host.log(QStringLiteral(
                        "UR3e semi-fixed: no saved apex joints — planning home pose at Z=R…"));
                QString apexErr;
                if (!planApexJointsOnRingSphere(input.serverUrl, apexProbe.topPose, &apexErr))
                {
                    if (host.log)
                        host.log(QStringLiteral("UR3e semi-fixed: apex plan failed — %1")
                                     .arg(apexErr));
                    doTop = false;
                }
                else
                {
                    route.topPose = apexProbe.topPose;
                    route.hasTopPose = true;
                }
            }
        }
    }
    const double intervalDeg =
        route.intervalDeg > 0.0 ? route.intervalDeg : 10.0;
    const double panRangeDeg =
        route.panRangeDeg >= 0.0 ? std::min(360.0, route.panRangeDeg) : 360.0;
    const int samplesPerRing = semiFixedSampleCount(intervalDeg, panRangeDeg);
    const double intervalRad = intervalDeg * kDegToRad;
    const int panDir = route.panDirection >= 0 ? 1 : -1;
    const int stabilizeMs =
        input.stabilizeMs > 0
            ? input.stabilizeMs
            : (route.stabilizeMs > 0 ? route.stabilizeMs : 500);
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
        const bool announceTop = doTop;
        host.log(QStringLiteral(
                     "UR3e semi-fixed execute: %1%2 ring entry(ies), "
                     "full-spin latitudes use one pin, backup pairs go "
                     "pin1 → home → pin2, pan %3%4° every %5° "
                     "(%6 samples/spin%7)…")
                     .arg(announceTop ? QStringLiteral("top still + ") : QString())
                     .arg(ringCount)
                     .arg(panDir > 0 ? QStringLiteral("+") : QStringLiteral("−"))
                     .arg(panRangeDeg, 0, 'f', 0)
                     .arg(intervalDeg, 0, 'f', 1)
                     .arg(samplesPerRing)
                     .arg(captureStills ? QStringLiteral(", BFS stills → ") + input.captureDir
                                        : QStringLiteral(", motion-only")));
    }

    // Always: MoveIt → top (θ=0 look-down) → photo, then rings.
    // Preview stores rings at [0..N-1] and top at index N (see inferSemiFixedPreviewRings).
    const double stageMm = input.stageMm;
    if (host.moveStage)
    {
        QString stageErr;
        if (!host.moveStage(stageMm, QStringLiteral("stage"), &stageErr))
        {
            ok = false;
            errorMessage = stageErr.isEmpty() ? QStringLiteral("Stage move failed")
                                              : stageErr;
            finishNow(false, stopRequested());
            return;
        }
    }

    int topPreviewIndex = ringCount;
    for (int i = 0; i < ringCount; ++i)
    {
        if (route.rings[i].noPan || std::abs(route.rings[i].thetaDeg) < 0.75)
        {
            topPreviewIndex = i;
            break;
        }
    }
    std::vector<double> lastEntryBranch;
    bool hasLastEntryBranch = false;
    bool rgbPromptDone = false;
    if (doTop)
    {
        Ur3eSemiFixedRoute routeCopy = route;
        ensureSemiFixedTopPose(routeCopy);
        Ur3eSemiFixedRing top = routeCopy.topPose;

        if (host.setActiveRing)
            host.setActiveRing(topPreviewIndex);
        if (host.log)
        {
            host.log(route.isFppPlan
                         ? QStringLiteral(
                               "UR3e FPP: MoveIt → apex (home XY, %1 mm, no pan)…")
                               .arg(route.apexHeightM > 0.01 ? route.apexHeightM * 1000.0
                                                             : 450.0,
                                    0, 'f', 0)
                         : QStringLiteral("UR3e semi-fixed: MoveIt → apex at ring R (θ=0)…"));
        }

        QString moveErr;
        bool stopped = false;
        const Ur3eScanTcpPose *tcpPtr = top.hasEntryTcp ? &top.entryTcp : nullptr;
        // Top / θ=0: FPP = home XY at working distance; Semi = home XY, Z = ring R. No pin-pose cone.
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
                if (stopRequested())
                {
                    ur3eStopMotion(input.serverUrl);
                    goto semi_fixed_done;
                }
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

    if (!input.skipRings)
    for (int ringIndex = 0; ringIndex < ringCount; ++ringIndex)
    {
        if (!sessionOk() || stopRequested())
        {
            ur3eStopMotion(input.serverUrl);
            break;
        }

        const Ur3eSemiFixedRing &ring = route.rings[ringIndex];
        if (ring.entryJointsRad.size() != 6)
        {
            ok = false;
            errorMessage = QStringLiteral("Ring %1 has invalid joints").arg(ringIndex);
            if (host.markRingFailed)
                host.markRingFailed(ringIndex);
            markPinFail();
            break;
        }

        const bool isRgbRing =
            ring.captureKind.trimmed().compare(QStringLiteral("rgb"), Qt::CaseInsensitive)
            == 0;
        if (route.isFppPlan && isRgbRing && !rgbPromptDone)
        {
            rgbPromptDone = true;
            // First RGB ring: blank DLP, then ask the operator whether to continue.
            if (host.blankProjector)
            {
                QString blankErr;
                if (!host.blankProjector(&blankErr))
                {
                    if (host.log)
                    {
                        host.log(QStringLiteral("UR3e FPP: DLP blank before RGB failed — %1")
                                     .arg(blankErr.isEmpty() ? QStringLiteral("unknown")
                                                             : blankErr));
                    }
                }
                else if (host.log)
                {
                    host.log(QStringLiteral(
                        "UR3e FPP: DLP blanked after FPP — waiting for RGB confirm…"));
                }
            }
            if (host.confirmContinueRgb)
            {
                if (!host.confirmContinueRgb())
                {
                    if (host.log)
                        host.log(QStringLiteral("UR3e FPP: RGB scan skipped by operator."));
                    break;
                }
                if (host.log)
                    host.log(QStringLiteral("UR3e FPP: continuing with RGB sweep…"));
            }
        }

        if (host.setActiveRing)
            host.setActiveRing(ringIndex);
        if (host.log)
        {
            const bool prevSame =
                ringIndex > 0
                && sameRouteRingTheta(route.rings[ringIndex - 1], ring);
            const bool nextSame =
                ringIndex + 1 < ringCount
                && sameRouteRingTheta(ring, route.rings[ringIndex + 1]);
            QString hopNote;
            const bool elbowFlipVsHome =
                !sameElbowFamily(configuredHomeJointsRad(), ring.entryJointsRad);
            if (prevSame)
                hopNote = QStringLiteral(" (backup 2-pin: home → pin2)");
            else if (nextSame)
                hopNote = QStringLiteral(" (backup 2-pin: pin1, then home → pin2)");
            else if (elbowFlipVsHome)
                hopNote = QStringLiteral(" (via-home: elbow family ≠ home)");
            else if (ring.baseSweepOk && !hasLastEntryBranch)
                hopNote = QStringLiteral(" (simple path)");
            if (route.isFppPlan)
            {
                host.log(QStringLiteral(
                             "UR3e FPP [%1/%2] MoveIt hop → ring “%3” (taught pose)…")
                             .arg(ringIndex + 1)
                             .arg(ringCount)
                             .arg(ring.displayName));
            }
            else
            {
                host.log(QStringLiteral("UR3e semi-fixed [%1/%2] MoveIt → ring “%3”%4…")
                             .arg(ringIndex + 1)
                             .arg(ringCount)
                             .arg(ring.displayName)
                             .arg(hopNote));
            }
        }

        QString moveErr;
        bool stopped = false;
        const Ur3eScanTcpPose *tcpPtr = ring.hasEntryTcp ? &ring.entryTcp : nullptr;
        const bool backupHop =
            ringIndex > 0 && sameRouteRingTheta(route.rings[ringIndex - 1], ring);
        const bool elbowFlipVsHome =
            !sameElbowFamily(configuredHomeJointsRad(), ring.entryJointsRad);
        const bool simple360 =
            ring.baseSweepOk && !ring.noPan && !backupHop && !hasLastEntryBranch
            && !elbowFlipVsHome;
        const std::vector<double> *unwindPtr =
            hasLastEntryBranch ? &lastEntryBranch : nullptr;

        // Already at the taught entry.
        bool hopOk = true;
        if (hasLastEntryBranch && jointsNearEqual(lastEntryBranch, ring.entryJointsRad))
        {
            if (host.log)
            {
                host.log(QStringLiteral(
                             "UR3e semi-fixed ring %1: already at entry — skip hop")
                             .arg(ringIndex));
            }
        }
        else if (route.isFppPlan)
        {
            // Hand-taught FPP rings: MoveIt plans the entry hop (handles elbow-family
            // flips that hardware PTP would fold through 0°). Pan spins stay hardware.
            if (host.log)
            {
                const bool elbowFlip =
                    hasLastEntryBranch
                        ? !sameElbowFamily(lastEntryBranch, ring.entryJointsRad)
                        : !sameElbowFamily(configuredHomeJointsRad(), ring.entryJointsRad);
                if (elbowFlip)
                {
                    host.log(QStringLiteral(
                        "UR3e FPP: elbow family change — relying on MoveIt path…"));
                }
            }
            hopOk = moveItToJoints(input.serverUrl, ring.entryJointsRad, tcpPtr, &moveErr,
                                   &stopped, false);
            if (!hopOk && !stopped)
            {
                if (host.log)
                {
                    host.log(QStringLiteral(
                                 "UR3e FPP: MoveIt direct failed (%1) — retry via home…")
                                 .arg(moveErr));
                }
                hopOk = moveItToRingEntryViaHome(input.serverUrl,
                                                 ring.entryJointsRad,
                                                 tcpPtr,
                                                 host,
                                                 unwindPtr,
                                                 &moveErr,
                                                 &stopped);
            }
        }
        else if (simple360)
        {
            hopOk = moveItToJoints(input.serverUrl, ring.entryJointsRad, tcpPtr, &moveErr,
                                   &stopped, false);
        }
        else
        {
            hopOk = moveItToRingEntryViaHome(input.serverUrl,
                                             ring.entryJointsRad,
                                             tcpPtr,
                                             host,
                                             unwindPtr,
                                             &moveErr,
                                             &stopped);
        }
        if (!hopOk)
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

        // Imaging pans: full 360° when the pin is sweep-OK; backup 2-pin rings
        // only visit the contiguous valid run (or the pin itself if no mask).
        std::vector<double> samplePans;
        const bool pinOnly = ring.noPan || std::abs(ring.thetaDeg) < 0.75;
        const bool backupPartial = !pinOnly && ring.backupCoverageOk && !ring.baseSweepOk;
        const double ringIntervalDeg = resolveRingIntervalDeg(ring, intervalDeg);
        const double ringPanRangeDeg = resolveRingPanRangeDeg(ring, panRangeDeg);
        const int ringSamplesPerRing =
            semiFixedSampleCount(ringIntervalDeg, ringPanRangeDeg);
        const double ringIntervalRad = ringIntervalDeg * kDegToRad;
        if (pinOnly)
        {
            samplePans.push_back(entryBranch[0]);
            if (host.log)
            {
                host.log(QStringLiteral(
                             "UR3e semi-fixed ring %1: apex pin — one still, no pan")
                             .arg(ringIndex));
            }
        }
        else if (backupPartial)
        {
            samplePans = backupContiguousPans(entryBranch[0], ring.panMask, panDir,
                                             ringIntervalDeg);
            if (samplePans.empty())
                samplePans.push_back(entryBranch[0]);
            if (host.log)
            {
                if (ring.panMask.empty())
                {
                    host.log(QStringLiteral(
                                 "UR3e semi-fixed ring %1: backup pin — no pan_mask, "
                                 "still at entry only (union %2°)")
                                 .arg(ringIndex)
                                 .arg(ring.backupUnionDeg, 0, 'f', 1));
                }
                else
                {
                    host.log(QStringLiteral(
                                 "UR3e semi-fixed ring %1: backup pan %2 sample(s) "
                                 "(union %3°, not a full 360°)")
                                 .arg(ringIndex)
                                 .arg(samplePans.size())
                                 .arg(ring.backupUnionDeg, 0, 'f', 1));
                }
            }
        }
        else
        {
            double sweepEntryPan = entryBranch[0];
            double edgePan = entryBranch[0];
            if (!pickPanSweepBranch(entryBranch[0],
                                    panDir,
                                    ringSamplesPerRing,
                                    ringIntervalRad,
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
            samplePans.reserve(static_cast<std::size_t>(ringSamplesPerRing));
            for (int i = 0; i < ringSamplesPerRing; ++i)
            {
                samplePans.push_back(
                    edgePan
                    + static_cast<double>(i) * ringIntervalRad * static_cast<double>(panDir));
            }
        }

        const int samplesThisRing = static_cast<int>(samplePans.size());
        const double edgePan = samplePans.empty() ? entryBranch[0] : samplePans.front();
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

        // Absolute continuous targets (never live+Δ — wrapped live caused reverse sweeps).
        for (int sample = 0; sample < samplesThisRing; ++sample)
        {
            if (!sessionOk() || stopRequested())
            {
                ur3eStopMotion(input.serverUrl);
                goto semi_fixed_done;
            }

            std::vector<double> sampleJoints = entryBranch;
            sampleJoints[0] = samplePans[static_cast<std::size_t>(sample)];

            if (sample > 0)
            {
                if (host.log)
                {
                    host.log(QStringLiteral(
                                 "UR3e semi-fixed ring %1: pan sample %2/%3 → %4° "
                                 "(step %+5°)…")
                                 .arg(ringIndex)
                                 .arg(sample + 1)
                                 .arg(samplesThisRing)
                                 .arg(sampleJoints[0] * 180.0 / kPi, 0, 'f', 1)
                                 .arg(ringIntervalDeg * panDir, 0, 'f', 1));
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
                             .arg(samplesThisRing)
                             .arg(sampleJoints[0] * 180.0 / kPi, 0, 'f', 1));
            }

            if (captureStills && host.captureStill)
            {
                Ur3eScanTcpPose planned = ring.hasEntryTcp ? ring.entryTcp : Ur3eScanTcpPose{};
                if (!host.captureStill(planned, ringIndex, sample))
                {
                    if (stopRequested())
                    {
                        ur3eStopMotion(input.serverUrl);
                        goto semi_fixed_done;
                    }
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
        const std::vector<double> *homeOverride =
            useFppSweepHome ? &fppSweepHome : nullptr;
        homeOk = retreatToScanHome(input.serverUrl, host, userStopped, entryPtr,
                                   homeOverride);
    }

    // Leave the robot on the configured continuous home branch. Without this,
    // wrap-aware homing can finish at wrist_3 home±360° and poison the next scan.
    // FPP uses sweep pose as hub — skip cfg-home wrist rewind.
    if (homeOk && !userStopped && !stopRequested() && !useFppSweepHome)
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
