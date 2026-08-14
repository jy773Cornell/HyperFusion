// MoveIt-backed hemisphere scan planning for UR3e (backend layer).

#pragma once



#include "backend/3dscanning/Ur3eHemisphereScan.hpp"

#include "backend/3dscanning/Ur3eClient.hpp"

#include "backend/3dscanning/Ur3eWorkspaceBoundary.hpp"

#include <QJsonObject>
#include <QString>
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

[[nodiscard]] Ur3eScanTcpPose tcpPoseForHemispherePoint(const Ur3eHemisphereScanPoint &gridPoint);



/// Convert scan TCP to UR sidecar pose (fills rotation vector from tool +Z when unset).

[[nodiscard]] Ur3eTcpPose urTcpPoseFromScanTcp(const Ur3eScanTcpPose &scanTcp);



/// Build grid poses locally, then evaluate reachability via MoveIt in the WSL sidecar.

[[nodiscard]] Ur3eHemisphereScanPlan evaluateHemisphereScanPlanMoveIt(

    const QString &serverUrl,

    const Ur3eHemisphereScanParams &params,

    const Ur3eWorkspaceBoundary &boundary,

    QString *errorMessage = nullptr);

/// Semi plan: same MoveIt pin IK as Auto, plus base-link pan-circle check.
/// Keeps at most *maxSweepOkPerRing* sweep-OK pins per latitude (default 3).
[[nodiscard]] Ur3eHemisphereScanPlan evaluateSemiHemisphereScanPlanMoveIt(
    const QString &serverUrl,
    const Ur3eHemisphereScanParams &params,
    const Ur3eWorkspaceBoundary &boundary,
    int maxSweepOkPerRing = 3,
    QString *errorMessage = nullptr);

/// Reachable plan indices: top θ ring first (home-nearest entry), then downward ring-by-ring φ sweep.

[[nodiscard]] std::vector<int> buildHemisphereScanExecutionOrder(

    const Ur3eHemisphereScanPlan &plan);



/// Home pose from hyperfusion.cfg `[3d scanning] home_joints_deg` (radians).

[[nodiscard]] std::vector<double> ur3eScanHomeJointsRadFromConfig();

/// Wrapped joint-space distance (matches MoveIt scan_planner `_joint_distance_rad`).
[[nodiscard]] double ur3eJointDistanceRad(const std::vector<double> &referenceRad,
                                            const std::vector<double> &candidateRad);

[[nodiscard]] bool ur3eIsNearScanHomeJoints(const std::vector<double> &currentRad,
                                              double toleranceRad = 0.05);

void appendUr3eScanHomeJointsToJson(QJsonObject &body);

} // namespace hf::ur3e

