// Semi-fixed UR3e ring routes: MoveIt only for home↔ring / ring↔ring; pan spin without MoveIt.
// Backend layer. Works for mock and real hardware (same route JSON; cfg fingerprint ignores mock).

#pragma once

#include "backend/3dscanning/Ur3eHemisphereScanReachability.hpp"
#include "backend/HyperFusionConfig.hpp"

#include <QString>
#include <QVector>

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
    /// Always captured first (MoveIt look-down / θ=0). Filled with default if missing.
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

/// Default apex look-down pose used for every semi-fixed scan (before rings).
[[nodiscard]] Ur3eSemiFixedRing defaultSemiFixedTopPose();

/// Ensure route.hasTopPose; if missing, install defaultSemiFixedTopPose().
void ensureSemiFixedTopPose(Ur3eSemiFixedRoute &route);

/// Named Semi plans beside app.exe (`ur3e_semi_scan_routes`).
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

/// Build semi-fixed rings from a Semi (or Auto) hemisphere plan: one entry per
/// latitude from sweep-OK pins when present (else reachable); prefer home→pin /
/// nearest-to-home among candidates.
[[nodiscard]] Ur3eSemiFixedRoute
semiFixedRouteFromHemispherePlan(const Ur3eHemisphereScanPlan &plan,
                                 const QString &robotCfgFingerprint,
                                 const QString &displayName = QString(),
                                 double intervalDeg = 10.0,
                                 int panDirection = 1);

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

} // namespace hf::ur3e
