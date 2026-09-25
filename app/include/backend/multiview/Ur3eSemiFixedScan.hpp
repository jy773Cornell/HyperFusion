// Semi-fixed UR3e ring routes: MoveIt only for home↔ring / ring↔ring; pan spin without MoveIt.
// Backend layer. Works for mock and real hardware (same route JSON; cfg fingerprint ignores mock).

#pragma once

#include "backend/multiview/Ur3eHemisphereScanReachability.hpp"
#include "backend/HyperFusionConfig.hpp"

#include <QString>
#include <QVector>

#include <cstdint>
#include <array>
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
    /// ``rgb`` = one color still per pan sample. Empty = FPP burst when the plan is FPP.
    QString captureKind;
    /// 0 = inherit route ``intervalDeg``. Per-ring override (RGB Range / Interval).
    double intervalDeg = 0.0;
    /// ``< 0`` = inherit route ``panRangeDeg``. Per-ring override (RGB Range / Interval).
    double panRangeDeg = -1.0;
};

struct Ur3eSemiFixedRoute
{
    QString id;
    QString displayName;
    QString robotCfgFingerprint;
    /// Photo every this many degrees around shoulder_pan.
    double intervalDeg = 10.0;
    /// Pan arc length in degrees (0…360). 360 = full spin (legacy Semi default).
    double panRangeDeg = 360.0;
    /// +1 = positive pan direction, −1 = opposite.
    int panDirection = 1;
    int stabilizeMs = 500;
    /// Optional plan-specific MoveIt hub, radians. FPP uses this between disjoint sweeps.
    std::vector<double> homeJointsRad;
    /// Absolute shoulder_pan ranges in degrees. Empty keeps the legacy contiguous arc.
    std::vector<std::array<double, 2>> panSweepRangesDeg;
    /// JSON kind ``fpp`` (vs ``ur3e_semi_fixed_route`` for Semi).
    bool isFppPlan = false;
    /// FPP plans historically carried stage mm in JSON; stage is GUI-owned now.
    bool haveStagePositions = false;
    double stageHomeMm = 0.0;
    double stageDlpMm = 0.0;
    /// FPP apex working distance (metres above sample / z=0). 0 = default 0.45.
    double apexHeightM = 0.0;
    /// Captured first when present. FPP omits this for sweep-only plans. Semi: home XY at ring R.
    Ur3eSemiFixedRing topPose{};
    bool hasTopPose = false;
    QVector<Ur3eSemiFixedRing> rings;
    /// Optional RGB sweep. Same spin as a ring; one color still per sample, not an FPP burst.
    Ur3eSemiFixedRing rgbRing{};
    bool hasRgbRing = false;
    /// Legacy optional RGB home still. Execute is sweep-only and does not fold this in.
    Ur3eSemiFixedRing rgbHome{};
    bool hasRgbHome = false;
    /// Optional BFS exposure for FPP/DLP bursts (µs). Unset → use BFS GUI Exposure Time.
    double dlpExposureUs = 0.0;
    bool hasDlpExposure = false;
    /// Optional BFS exposure for RGB sweep stills (µs). Unset → use BFS GUI Exposure Time.
    double rgbExposureUs = 0.0;
    bool hasRgbExposure = false;
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

/// Apex TCP: home XY + home orientation, Z = *zM* (FPP working distance or Semi ring R).
[[nodiscard]] Ur3eSemiFixedRing apexTopPoseOnRingSphere(double zM);

/// Legacy wrapper — 200 mm leftover. Prefer apexTopPoseOnRingSphere(ring R).
[[nodiscard]] Ur3eSemiFixedRing defaultSemiFixedTopPose();

/// Sphere radius implied by planned ring TCPs (0 if unknown).
[[nodiscard]] double inferSemiFixedSphereRadiusM(const Ur3eSemiFixedRoute &route);

/// FPP: keep an explicit top_pose only. Sweep-only plans stay without a home still.
/// Semi: if rings exist, install / replace a non-matching apex with home XY and Z = R.
void ensureSemiFixedTopPose(Ur3eSemiFixedRoute &route);

/// Named Semi plans beside app.exe (`mvs_scan_plans/semi`, from app/preset).
/// Prefer ``defaultUr3eSemiScanPlansDir()``; this alias remains for call sites.
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
    /// FPP: pin at centerXM/YM/ZM in tray preview frame (Z up from tray).
    /// Taught entry_tcp is base_link; inferFppPreviewPins converts mount→tray.
    bool drawAsPin = false;
    /// Unit approach direction for pin stick (tool +Z / look axis).
    double tipDirX = 0.0;
    double tipDirY = 0.0;
    double tipDirZ = 1.0;
    /// Maps to Semi/FPP execute ringIndex (top uses rings.size() when separate).
    int executeIndex = 0;
    bool noPan = false;
    /// FPP RGB home / sweep pins (distinct color from fringe pins).
    bool isRgb = false;
    bool reachabilityKnown = false;
    bool reachable = false;
    bool homePathOk = true;
};

[[nodiscard]] Ur3eSemiFixedPreviewRing
inferSemiFixedPreviewRing(const Ur3eSemiFixedRing &ring);

[[nodiscard]] QVector<Ur3eSemiFixedPreviewRing>
inferSemiFixedPreviewRings(const Ur3eSemiFixedRoute &route);

/// FPP: discrete pins at scan-TCP pan samples (GUI interval/range).
/// Converts taught base_link entry_tcp into tray-frame preview XYZ (Z up).
[[nodiscard]] QVector<Ur3eSemiFixedPreviewRing>
inferFppPreviewPins(const Ur3eSemiFixedRoute &route);

/// Geometric Semi rings from scan params (layers × θ) — grey preview before / during Plan.
[[nodiscard]] QVector<Ur3eSemiFixedPreviewRing>
previewSemiFixedRingsFromScanParams(const Ur3eHemisphereScanParams &params);

/// One preview ring per latitude in a Semi/Auto plan (includes unreachable → blue).
[[nodiscard]] QVector<Ur3eSemiFixedPreviewRing>
previewSemiFixedRingsFromHemispherePlan(const Ur3eHemisphereScanPlan &plan);

/// Fill Layer (verticalPoints) + θ range from plan latitudes (skips apex).
void syncHemisphereParamsFromPlanLatitudes(const Ur3eHemisphereScanPlan &plan,
                                           Ur3eHemisphereScanParams &paramsInOut);

[[nodiscard]] int semiFixedSampleCount(double intervalDeg, double rangeDeg = 360.0);

[[nodiscard]] int semiFixedSweepRangeSampleCount(
    const std::vector<std::array<double, 2>> &rangesDeg, double intervalDeg);

[[nodiscard]] double semiFixedSweepRangeTotalDeg(
    const std::vector<std::array<double, 2>> &rangesDeg);

/// Imaging samples for one ring: panRange / interval, or backup arc / interval.
[[nodiscard]] int semiFixedRingSampleCount(const Ur3eSemiFixedRing &ring,
                                           double intervalDeg,
                                           double rangeDeg = 360.0);

/// Resolve per-ring spin spacing (0 / ``<0`` on the ring inherit the route defaults).
[[nodiscard]] double resolveRingIntervalDeg(const Ur3eSemiFixedRing &ring,
                                            double routeIntervalDeg);
[[nodiscard]] double resolveRingPanRangeDeg(const Ur3eSemiFixedRing &ring,
                                            double routePanRangeDeg);

/// Append ``rgbRing`` onto ``route.rings`` for execute / capture index (sweep-only; no rgb_home).
void foldRgbEntriesIntoRings(Ur3eSemiFixedRoute &route);

} // namespace hf::ur3e
