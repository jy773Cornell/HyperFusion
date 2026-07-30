// MoveIt-backed hemisphere scan planning for UR3e (backend layer).

#include "backend/3dscanning/Ur3eHemisphereScanReachability.hpp"



#include "backend/HyperFusionConfig.hpp"
#include "backend/3dscanning/Ur3eMountTransform.hpp"

#include <QJsonArray>

#include <QJsonDocument>

#include <QJsonObject>



#include <algorithm>

#include <array>

#include <cmath>
#include <limits>
#include <map>



namespace hf::ur3e

{

Ur3eScanTcpPose tcpPoseForHemispherePoint(const Ur3eHemisphereScanPoint &gridPoint)

{

    Ur3eScanTcpPose tcp;

    tcp.xM = gridPoint.xM;

    tcp.yM = gridPoint.yM;

    tcp.zM = gridPoint.zM + kSampleTrayHeightM;



    const Ur3eMountTransform mount =
        Ur3eMountTransform::sceneAlignFromConfig(hf::hardwareConfig().ur3e);
    mount.transformPoint(tcp.xM, tcp.yM, tcp.zM);

    // Dome axis / floor-circle center (home optical-TCP projected onto tray).
    double centerXM = 0.0;
    double centerYM = 0.0;
    scanCenterOffsetM(centerXM, centerYM);
    double centerZM = kSampleTrayHeightM;
    mount.transformPoint(centerXM, centerYM, centerZM);

    tcp.toolZMx = centerXM - tcp.xM;
    tcp.toolZMy = centerYM - tcp.yM;
    tcp.toolZMz = centerZM - tcp.zM;

    // Apex (θ=0, look-down): image-up / TCP upper face → world +X.
    // Other pins: image-up ≈ tray/world +Z when scan_camera_up_world_z is enabled.
    const bool isApexPin = std::abs(gridPoint.thetaDeg) <= 1.0e-9;
    double upX = isApexPin ? 1.0 : 0.0;
    double upY = 0.0;
    double upZ = isApexPin ? 0.0 : 1.0;
    mount.transformVector(upX, upY, upZ);
    const bool lockUp =
        isApexPin || hf::hardwareConfig().ur3e.scanCameraUpWorldZ;
    orientScanTcpFromToolZ(tcp, lockUp, upX, upY, upZ);

    return tcp;

}



Ur3eTcpPose urTcpPoseFromScanTcp(const Ur3eScanTcpPose &scanTcp)

{

    Ur3eTcpPose pose;

    pose.x = scanTcp.xM;

    pose.y = scanTcp.yM;

    pose.z = scanTcp.zM;

    pose.rx = scanTcp.rxRad;

    pose.ry = scanTcp.ryRad;

    pose.rz = scanTcp.rzRad;

    return pose;

}



Ur3eHemisphereScanPlan evaluateHemisphereScanPlanMoveIt(const QString &serverUrl,

                                                        const Ur3eHemisphereScanParams &params,

                                                        const Ur3eWorkspaceBoundary &boundary,

                                                        QString *errorMessage)

{

    Ur3eHemisphereScanPlan plan;

    const std::vector<Ur3eHemisphereScanPoint> gridPoints = generateHemisphereScanPoints(params);

    if (gridPoints.empty())

        return plan;



    QJsonArray poses;

    for (std::size_t index = 0; index < gridPoints.size(); ++index)

    {

        const Ur3eScanTcpPose tcp = tcpPoseForHemispherePoint(gridPoints[index]);

        QJsonObject pose;

        pose.insert(QStringLiteral("index"), static_cast<int>(index));

        pose.insert(QStringLiteral("x"), tcp.xM);

        pose.insert(QStringLiteral("y"), tcp.yM);

        pose.insert(QStringLiteral("z"), tcp.zM);

        pose.insert(QStringLiteral("rx"), tcp.rxRad);

        pose.insert(QStringLiteral("ry"), tcp.ryRad);

        pose.insert(QStringLiteral("rz"), tcp.rzRad);

        pose.insert(QStringLiteral("tool_z_x"), tcp.toolZMx);

        pose.insert(QStringLiteral("tool_z_y"), tcp.toolZMy);

        pose.insert(QStringLiteral("tool_z_z"), tcp.toolZMz);

        // Camera-up preference for MoveIt cone re-rolls (apex uses world +X).
        const bool isApexPin =
            std::abs(gridPoints[index].thetaDeg) <= 1.0e-9;
        pose.insert(QStringLiteral("camera_up_x"), isApexPin ? 1.0 : 0.0);
        pose.insert(QStringLiteral("camera_up_y"), 0.0);
        pose.insert(QStringLiteral("camera_up_z"), isApexPin ? 0.0 : 1.0);
        pose.insert(QStringLiteral("require_perpendicular"), isApexPin);

        poses.append(pose);

    }



    QJsonObject workspace;

    workspace.insert(QStringLiteral("enabled"), boundary.enabled);

    workspace.insert(QStringLiteral("length_m"), boundary.lengthM());

    workspace.insert(QStringLiteral("width_m"), boundary.widthM());

    workspace.insert(QStringLiteral("height_m"), boundary.heightM());

    workspace.insert(QStringLiteral("mount_height_m"), boundary.mountHeightM());

    workspace.insert(QStringLiteral("ceiling_clearance_m"), boundary.ceilingClearanceM());

    QJsonObject body;

    body.insert(QStringLiteral("poses"), poses);

    body.insert(QStringLiteral("workspace"), workspace);

    body.insert(QStringLiteral("pin_pose_tolerance_deg"),
                hf::hardwareConfig().ur3e.pinPoseToleranceDeg);
    body.insert(QStringLiteral("scan_camera_up_world_z"),
                hf::hardwareConfig().ur3e.scanCameraUpWorldZ);

    appendUr3eScanHomeJointsToJson(body);

    // Large grids + home→pin path checks can exceed 15 min; keep curl alive until MoveIt finishes.
    const int planTimeoutMs = std::max(60000, hf::hardwareConfig().ur3e.planTimeoutMs);
    const QJsonObject response =
        ur3ePostJsonRequest(serverUrl, QStringLiteral("/plan_hemisphere_scan"), body,
                            planTimeoutMs, errorMessage);

    if (response.isEmpty() || !response.value(QStringLiteral("ok")).toBool(false))

    {

        if (errorMessage != nullptr && errorMessage->isEmpty())

            *errorMessage = QStringLiteral("MoveIt hemisphere scan planning failed.");

        plan.errorMessage = errorMessage != nullptr ? *errorMessage : QString();

        return plan;

    }



    const QJsonArray results = response.value(QStringLiteral("results")).toArray();

    plan.points.reserve(gridPoints.size());

    for (std::size_t index = 0; index < gridPoints.size(); ++index)

    {

        Ur3ePlannedScanPoint planned;

        planned.gridPoint = gridPoints[index];

        planned.tcp = tcpPoseForHemispherePoint(gridPoints[index]);

        plan.points.push_back(planned);

    }



    for (const QJsonValue &value : results)

    {

        const QJsonObject entry = value.toObject();

        const int pointIndex = entry.value(QStringLiteral("index")).toInt(-1);

        if (pointIndex < 0 || pointIndex >= static_cast<int>(plan.points.size()))

            continue;



        Ur3ePlannedScanPoint &planned = plan.points[static_cast<std::size_t>(pointIndex)];

        planned.reachable = entry.value(QStringLiteral("reachable")).toBool(false);

        // Default true for older sidecar payloads that omit the field.
        planned.homePathOk =
            planned.reachable
            && entry.value(QStringLiteral("home_path_ok")).toBool(true);

        planned.planningError = entry.value(QStringLiteral("error")).toString();



        const QJsonArray joints = entry.value(QStringLiteral("joints")).toArray();

        planned.jointPositionsRad.clear();

        planned.jointPositionsRad.reserve(6);

        for (const QJsonValue &jointValue : joints)

            planned.jointPositionsRad.push_back(jointValue.toDouble(0.0));

        const QJsonObject tcpObj = entry.value(QStringLiteral("tcp")).toObject();
        if (!tcpObj.isEmpty())
        {
            planned.tcp.xM = tcpObj.value(QStringLiteral("x")).toDouble(planned.tcp.xM);
            planned.tcp.yM = tcpObj.value(QStringLiteral("y")).toDouble(planned.tcp.yM);
            planned.tcp.zM = tcpObj.value(QStringLiteral("z")).toDouble(planned.tcp.zM);
            planned.tcp.rxRad = tcpObj.value(QStringLiteral("rx")).toDouble(planned.tcp.rxRad);
            planned.tcp.ryRad = tcpObj.value(QStringLiteral("ry")).toDouble(planned.tcp.ryRad);
            planned.tcp.rzRad = tcpObj.value(QStringLiteral("rz")).toDouble(planned.tcp.rzRad);
            planned.tcp.toolZMx =
                tcpObj.value(QStringLiteral("tool_z_x")).toDouble(planned.tcp.toolZMx);
            planned.tcp.toolZMy =
                tcpObj.value(QStringLiteral("tool_z_y")).toDouble(planned.tcp.toolZMy);
            planned.tcp.toolZMz =
                tcpObj.value(QStringLiteral("tool_z_z")).toDouble(planned.tcp.toolZMz);
        }

        if (planned.reachable)
        {
            ++plan.reachableCount;
            if (planned.homePathOk)
                ++plan.homePathOkCount;
            else
                ++plan.chainOnlyCount;
        }
        else
        {
            ++plan.unreachableCount;
        }

    }



    plan.moveItUsed = true;

    return plan;

}

std::vector<double> ur3eScanHomeJointsRadFromConfig()
{
    constexpr double kPi = 3.14159265358979323846;
    const std::array<double, 6> &homeDeg = hf::hardwareConfig().ur3e.homeJointsDeg;
    std::vector<double> joints;
    joints.reserve(6);
    for (const double deg : homeDeg)
        joints.push_back(deg * kPi / 180.0);
    return joints;
}

namespace
{
double jointDeltaRad(const double referenceRad, const double candidateRad)
{
    // Shortest signed delta in (-π, π]; matches Python joint_angles.joint_delta_rad.
    return std::atan2(std::sin(candidateRad - referenceRad),
                      std::cos(candidateRad - referenceRad));
}
} // namespace

double ur3eJointDistanceRad(const std::vector<double> &referenceRad,
                            const std::vector<double> &candidateRad)
{
    if (referenceRad.size() != 6 || candidateRad.size() != 6)
        return std::numeric_limits<double>::infinity();

    double totalSq = 0.0;
    for (int jointIndex = 0; jointIndex < 6; ++jointIndex)
    {
        const double delta =
            jointDeltaRad(referenceRad[static_cast<std::size_t>(jointIndex)],
                          candidateRad[static_cast<std::size_t>(jointIndex)]);
        totalSq += delta * delta;
    }
    return std::sqrt(totalSq);
}

bool ur3eIsNearScanHomeJoints(const std::vector<double> &currentRad, const double toleranceRad)
{
    const std::vector<double> homeRad = ur3eScanHomeJointsRadFromConfig();
    return ur3eJointDistanceRad(homeRad, currentRad) <= toleranceRad;
}

void appendUr3eScanHomeJointsToJson(QJsonObject &body)
{
    QJsonArray homeJointsDeg;
    for (const double deg : hf::hardwareConfig().ur3e.homeJointsDeg)
        homeJointsDeg.append(deg);
    body.insert(QStringLiteral("home_joints_deg"), homeJointsDeg);
}

namespace
{

[[nodiscard]] int findClosestIndexToJoints(const Ur3eHemisphereScanPlan &plan,
                                         const std::vector<int> &candidates,
                                         const std::vector<double> &referenceRad)
{
    int bestIndex = -1;
    double bestDistance = std::numeric_limits<double>::infinity();
    for (const int index : candidates)
    {
        const std::vector<double> &joints =
            plan.points[static_cast<std::size_t>(index)].jointPositionsRad;
        const double distance = ur3eJointDistanceRad(referenceRad, joints);
        if (distance < bestDistance)
        {
            bestDistance = distance;
            bestIndex = index;
        }
    }
    return bestIndex;
}

void appendRingPhiSweep(std::vector<int> &order,
                        const Ur3eHemisphereScanPlan &plan,
                        const std::vector<int> &ringIndices,
                        const int entryIndex)
{
    // Sweep decreasing φ from entry (clockwise about world +Z when looking down on tray).
    if (ringIndices.empty() || entryIndex < 0)
        return;

    std::vector<int> sorted = ringIndices;
    std::sort(sorted.begin(), sorted.end(), [&plan](const int lhs, const int rhs) {
        const Ur3eHemisphereScanPoint &a = plan.points[static_cast<std::size_t>(lhs)].gridPoint;
        const Ur3eHemisphereScanPoint &b = plan.points[static_cast<std::size_t>(rhs)].gridPoint;
        if (a.phiDeg != b.phiDeg)
            return a.phiDeg < b.phiDeg;
        return lhs < rhs;
    });

    const auto entryIt =
        std::find(sorted.begin(), sorted.end(), entryIndex);
    if (entryIt == sorted.end())
    {
        // No entry pin: still walk clockwise (descending φ).
        order.insert(order.end(), sorted.rbegin(), sorted.rend());
        return;
    }

    // entry → lower φ … → first, then wrap from last → … → just above entry.
    for (auto it = entryIt;; )
    {
        order.push_back(*it);
        if (it == sorted.begin())
            break;
        --it;
    }
    for (auto it = sorted.end(); it != std::next(entryIt); )
    {
        --it;
        order.push_back(*it);
    }
}

void buildTopRingFirstExecutionOrder(std::vector<int> &order,
                                     const Ur3eHemisphereScanPlan &plan,
                                     const std::vector<int> &reachableIndices,
                                     const std::vector<double> &homeRad)
{
    if (reachableIndices.empty())
        return;

    std::map<double, std::vector<int>> ringsByTheta;
    for (const int index : reachableIndices)
    {
        const double theta =
            plan.points[static_cast<std::size_t>(index)].gridPoint.thetaDeg;
        ringsByTheta[theta].push_back(index);
    }

    std::vector<double> ringThetas;
    ringThetas.reserve(ringsByTheta.size());
    for (const auto &entry : ringsByTheta)
        ringThetas.push_back(entry.first);
    // θ = 0° at dome apex → increasing θ walks top ring toward tray rim.
    std::sort(ringThetas.begin(), ringThetas.end());

    std::vector<double> referenceRad = homeRad;
    bool firstRing = true;

    for (const double theta : ringThetas)
    {
        const std::vector<int> &ringIndices = ringsByTheta[theta];
        if (ringIndices.empty())
            continue;

        const int entryIndex =
            firstRing ? findClosestIndexToJoints(plan, ringIndices, homeRad)
                      : findClosestIndexToJoints(plan, ringIndices, referenceRad);
        firstRing = false;
        if (entryIndex < 0)
            continue;

        const std::size_t orderBefore = order.size();
        appendRingPhiSweep(order, plan, ringIndices, entryIndex);
        if (order.size() > orderBefore)
        {
            referenceRad =
                plan.points[static_cast<std::size_t>(order.back())].jointPositionsRad;
        }
    }
}

} // namespace

std::vector<int> buildHemisphereScanExecutionOrder(const Ur3eHemisphereScanPlan &plan)
{
    std::vector<int> indices;
    indices.reserve(static_cast<std::size_t>(plan.reachableCount));
    for (int index = 0; index < static_cast<int>(plan.points.size()); ++index)
    {
        const Ur3ePlannedScanPoint &point = plan.points[static_cast<std::size_t>(index)];
        if (!point.reachable || point.jointPositionsRad.size() != 6)
            continue;
        indices.push_back(index);
    }

    std::vector<int> order;
    order.reserve(indices.size());
    buildTopRingFirstExecutionOrder(
        order, plan, indices, ur3eScanHomeJointsRadFromConfig());
    if (order.empty())
        order = indices;
    return order;
}

} // namespace hf::ur3e

