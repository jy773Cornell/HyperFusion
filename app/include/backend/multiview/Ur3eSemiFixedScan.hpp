// Semi-fixed UR3e ring routes: MoveIt only for home↔ring / ring↔ring; pan spin without MoveIt.
// Backend layer. Works for mock and real hardware (same route JSON; cfg fingerprint ignores mock).

#pragma once

#include "backend/multiview/Ur3eHemisphereScanReachability.hpp"
#include "backend/HyperFusionConfig.hpp"

#include <QString>
#include <QVector>

#include <cstdint>
#include <vector>

namespace hf::ur3e
{

struct Ur3eSemiFixedRing
{
    QString id;
    QString displayName;
    /// Entry joints (radians), length 6: pan, lift, elbow, w1, w2, w3.
    std::vector<double> entryJointsRad;
    /// TCP at entry (metres) — used for preview ring inference + stills metadata.
    Ur3eScanTcpPose entryTcp{};
    bool hasEntryTcp = false;
    /// Set when ring/top was taken from an Auto plan pin (preview coloring).
    bool reachabilityKnown = false;
    bool reachable = false;
    bool homePathOk = true;
    /// Full 360° shoulder_pan at this entry is plan-proven.
    bool baseSweepOk = true;
    /// 2-pin backup pair (union coverage, not a full spin).
    bool backupCoverageOk = false;
    double backupUnionDeg = 0.0;
    /// Abs 0..360° validity bins for backup spins. Empty = unknown.
    std::vector<std::uint8_t> panMask;
    /// Grid φ of the source plan pin (display + pin1/pin2 order).
    double phiDeg = 0.0;
    double thetaDeg = 0.0;
    /// One still at entry (apex). No shoulder_pan spin.
    bool noPan = false;
};

struct Ur3eSemiFixedRoute
{
    QString id;
    QString displayName;
    QString robotCfgFingerprint;
    /// Photo every this many degrees around shoulder_pan (+360°).
    double intervalDeg = 10.0;
    /// +1 = positive pan direction, −1 = opposite.
    int panDirection = 1;
    int stabilizeMs = 500;
    /// Always captured first (θ=0, home pose with Z = ring R). Filled if missing.
    Ur3eSemiFixedRing topPose{};
    bool hasTopPose = false;
    QVector<Ur3eSemiFixedRing> rings;
};

struct Ur3eSemiFixedRouteInfo
{
    QString id;
    QString displayName;
    QString path;
    QString robotCfgFingerprint;
    int ringCount = 0;
    double intervalDeg = 10.0;
};

/// Apex TCP: home XY + home orientation, Z = *sphereRadiusM*.
[[nodiscard]] Ur3eSemiFixedRing apexTopPoseOnRingSphere(double sphereRadiusM);

/// Legacy wrapper — 200 mm leftover. Prefer apexTopPoseOnRingSphere(ring R).
[[nodiscard]] Ur3eSemiFixedRing defaultSemiFixedTopPose();

/// Sphere radius implied by planned ring TCPs (0 if unknown).
[[nodiscard]] double inferSemiFixedSphereRadiusM(const Ur3eSemiFixedRoute &route);

/// If the route has rings, install / replace a non-matching apex with home XY and Z = R.
void ensureSemiFixedTopPose(Ur3eSemiFixedRoute &route);

/// Named Semi plans beside app.exe (`mvs_semi_scan_plans`, copied from app/preset).
[[nodiscard]] QString defaultUr3eSemiScanRoutesDir();

[[nodiscard]] bool saveUr3eSemiFixedRoute(const QString &path,
                                          const Ur3eSemiFixedRoute &route,
                                          QString *errorMessage = nullptr);

[[nodiscard]] bool loadUr3eSemiFixedRoute(const QString &path,
                                          const QString &expectedRobotCfgFingerprint,
                                          Ur3eSemiFixedRoute &routeOut,
                                          QString *errorMessage = nullptr);

[[nodiscard]] QVector<Ur3eSemiFixedRouteInfo>
listUr3eSemiFixedRoutesMatchingCfg(const QString &dir,
                                   const QString &expectedRobotCfgFingerprint);

/// Build semi-fixed rings from a Semi (or Auto) hemisphere plan.
/// One pin per latitude when any pin can spin a full 360°. Two pins only for
/// backup-union pairs (neither pin is sweep-OK); execute is pin1 → home → pin2.
[[nodiscard]] Ur3eSemiFixedRoute
semiFixedRouteFromHemispherePlan(const Ur3eHemisphereScanPlan &plan,
                                 const QString &robotCfgFingerprint,
                                 const QString &displayName = QString(),
                                 double intervalDeg = 10.0,
                                 int panDirection = 1);

/// Drop extra same-latitude pins once a full-360° (`baseSweepOk`) pin exists.
/// Backup-only pairs are left as two entries.
void pruneSemiFixedRedundantFullSpinPins(Ur3eSemiFixedRoute &route);

/// Infer a horizontal preview ring from entry TCP and tray scan center.
struct Ur3eSemiFixedPreviewRing
{
    double centerXM = 0.0;
    double centerYM = 0.0;
    double centerZM = 0.0;
    double radiusM = 0.05;
    QString displayName;
    /// Apex / top still (θ≈0) — drawn as a pin instead of a spin ring.
    bool isTopPose = false;
    bool noPan = false;
    bool reachabilityKnown = false;
    bool reachable = false;
    bool homePathOk = true;
};

[[nodiscard]] Ur3eSemiFixedPreviewRing
inferSemiFixedPreviewRing(const Ur3eSemiFixedRing &ring);

[[nodiscard]] QVector<Ur3eSemiFixedPreviewRing>
inferSemiFixedPreviewRings(const Ur3eSemiFixedRoute &route);

/// Geometric Semi rings from scan params (layers × θ) — grey preview before / during Plan.
[[nodiscard]] QVector<Ur3eSemiFixedPreviewRing>
previewSemiFixedRingsFromScanParams(const Ur3eHemisphereScanParams &params);

/// One preview ring per latitude in a Semi/Auto plan (includes unreachable → blue).
[[nodiscard]] QVector<Ur3eSemiFixedPreviewRing>
previewSemiFixedRingsFromHemispherePlan(const Ur3eHemisphereScanPlan &plan);

/// Fill Layer (verticalPoints) + θ range from plan latitudes (skips apex).
void syncHemisphereParamsFromPlanLatitudes(const Ur3eHemisphereScanPlan &plan,
                                           Ur3eHemisphereScanParams &paramsInOut);

[[nodiscard]] int semiFixedSampleCount(double intervalDeg);

/// Imaging samples for one ring: full 360° / interval, or backup arc / interval.
[[nodiscard]] int semiFixedRingSampleCount(const Ur3eSemiFixedRing &ring, double intervalDeg);

} // namespace hf::ur3e
