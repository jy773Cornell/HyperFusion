// MoveIt-backed hemisphere scan planning for UR3e (backend layer).

#pragma once



#include "backend/multiview/Ur3eHemisphereScan.hpp"

#include "backend/multiview/Ur3eClient.hpp"

#include "backend/multiview/Ur3eWorkspaceBoundary.hpp"

#include <QJsonObject>
#include <QString>
#include <cstdint>
#include <vector>



namespace hf::ur3e

{



/// World-frame TCP pose for a scan grid point (metres, tray-centred frame).

struct Ur3eScanTcpPose

{

    double xM = 0.0;

    double yM = 0.0;

    double zM = 0.0;

    /// UR rotation vector (axis-angle, radians).

    double rxRad = 0.0;

    double ryRad = 0.0;

    double rzRad = 0.0;

    /// Unit vector: tool +Z should align with this (toward sphere centre).

    double toolZMx = 0.0;

    double toolZMy = 0.0;

    double toolZMz = 1.0;

};



struct Ur3ePlannedScanPoint

{

    Ur3eHemisphereScanPoint gridPoint;

    Ur3eScanTcpPose tcp;

    bool reachable = false;

    /// True when MoveIt verified a home→pin path; false = previous→pin chain only.
    bool homePathOk = false;

    /// Semi plan: full shoulder_pan circle at this pin is collision-free (plan-time).
    bool baseSweepOk = false;

    /// Semi backup: one of a 2-pin pair whose pan union covers the backup threshold.
    bool backupCoverageOk = false;

    double backupUnionDeg = 0.0;

    /// Abs 0..360° validity bins (1 = collision-free). Empty = unknown / full sweep.
    std::vector<std::uint8_t> panMask;

    std::vector<double> jointPositionsRad;

    QString planningError;

};



struct Ur3eHemisphereScanPlan

{

    std::vector<Ur3ePlannedScanPoint> points;

    int reachableCount = 0;

    int unreachableCount = 0;

    /// Subset of reachableCount with a verified home→pin path.
    int homePathOkCount = 0;

    /// reachableCount − homePathOkCount (previous→pin only).
    int chainOnlyCount = 0;

    bool moveItUsed = false;

    QString errorMessage;

};



/// TCP pose with inward-facing tool axis for a dome grid point.
/// Apex keeps home XY/orientation and sets Z = R; rings use the active scan tip.

[[nodiscard]] Ur3eScanTcpPose tcpPoseForHemispherePoint(const Ur3eHemisphereScanPoint &gridPoint);

/// Legacy no-op: apex is already the active scan TCP (home pose, Z = R).

[[nodiscard]] Ur3eScanTcpPose retargetApexCameraTcpToMoveItTip(Ur3eScanTcpPose cameraTcp);



/// Convert scan TCP to UR sidecar pose (fills rotation vector from tool +Z when unset).

[[nodiscard]] Ur3eTcpPose urTcpPoseFromScanTcp(const Ur3eScanTcpPose &scanTcp);



/// Build grid poses locally, then evaluate reachability via MoveIt in the WSL sidecar.

[[nodiscard]] Ur3eHemisphereScanPlan evaluateHemisphereScanPlanMoveIt(

    const QString &serverUrl,

    const Ur3eHemisphereScanParams &params,

    const Ur3eWorkspaceBoundary &boundary,

    QString *errorMessage = nullptr);

/// Semi plan: same MoveIt pin IK as Auto, plus base-link pan-circle check.
/// Keeps at most *maxSweepOkPerRing* sweep-OK pins per latitude (default 2).
[[nodiscard]] Ur3eHemisphereScanPlan evaluateSemiHemisphereScanPlanMoveIt(
    const QString &serverUrl,
    const Ur3eHemisphereScanParams &params,
    const Ur3eWorkspaceBoundary &boundary,
    int maxSweepOkPerRing = 2,
    QString *errorMessage = nullptr);

/// Reachable plan indices: top θ ring first (home-nearest entry), then downward ring-by-ring φ sweep.

[[nodiscard]] std::vector<int> buildHemisphereScanExecutionOrder(

    const Ur3eHemisphereScanPlan &plan);

/// Same latitude (not apex). Used so a 2-pin ring visits pin1 → home → pin2.
[[nodiscard]] bool sameHemisphereScanRing(const Ur3ePlannedScanPoint &a,
                                          const Ur3ePlannedScanPoint &b);

/// Reachable pins on the same latitude as *ref* (0 if *ref* is apex).
[[nodiscard]] int reachablePinsOnSameRing(const Ur3eHemisphereScanPlan &plan,
                                          const Ur3ePlannedScanPoint &ref);



/// Home pose from hyperfusion.cfg `[multiview] home_joints_deg` (radians).

[[nodiscard]] std::vector<double> ur3eScanHomeJointsRadFromConfig();

/// Wrapped joint-space distance (matches MoveIt scan_planner `_joint_distance_rad`).
[[nodiscard]] double ur3eJointDistanceRad(const std::vector<double> &referenceRad,
                                            const std::vector<double> &candidateRad);

[[nodiscard]] bool ur3eIsNearScanHomeJoints(const std::vector<double> &currentRad,
                                              double toleranceRad = 0.05);

void appendUr3eScanHomeJointsToJson(QJsonObject &body);

} // namespace hf::ur3e

