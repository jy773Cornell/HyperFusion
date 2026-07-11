// MoveIt-backed hemisphere scan planning for UR3e (backend layer).

#include "backend/ur3e/Ur3eHemisphereScanReachability.hpp"



#include "backend/HyperFusionConfig.hpp"
#include "backend/ur3e/Ur3eMountTransform.hpp"

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

namespace

{

constexpr double kEpsilon = 1.0e-9;



double vectorLength(const double x, const double y, const double z)

{

    return std::sqrt(x * x + y * y + z * z);

}



bool normalizeVector(double &x, double &y, double &z)

{

    const double length = vectorLength(x, y, z);

    if (length <= kEpsilon)

        return false;

    x /= length;

    y /= length;

    z /= length;

    return true;

}



void rotationMatrixToRotVec(const double m00,

                            const double m01,

                            const double m02,

                            const double m10,

                            const double m11,

                            const double m12,

                            const double m20,

                            const double m21,

                            const double m22,

                            double &rx,

                            double &ry,

                            double &rz)

{

    const double trace = m00 + m11 + m22;

    const double angle = std::acos(std::clamp((trace - 1.0) * 0.5, -1.0, 1.0));

    if (angle <= kEpsilon)

    {

        rx = ry = rz = 0.0;

        return;

    }



    const double sinAngle = std::sin(angle);

    rx = (m21 - m12) / (2.0 * sinAngle) * angle;

    ry = (m02 - m20) / (2.0 * sinAngle) * angle;

    rz = (m10 - m01) / (2.0 * sinAngle) * angle;

}

void buildTcpOrientationFromToolZ(const double toolZX,
                                  const double toolZY,
                                  const double toolZZ,
                                  Ur3eScanTcpPose &tcp)
{
    double zx = toolZX;
    double zy = toolZY;
    double zz = toolZZ;
    if (!normalizeVector(zx, zy, zz))
    {
        zx = 0.0;
        zy = 0.0;
        zz = -1.0;
    }

    tcp.toolZMx = zx;
    tcp.toolZMy = zy;
    tcp.toolZMz = zz;

    double refX = std::abs(zx) < 0.9 ? 1.0 : 0.0;
    double refY = std::abs(zx) < 0.9 ? 0.0 : 1.0;
    const double refZ = 0.0;

    double yX = zy * refZ - zz * refY;
    double yY = zz * refX - zx * refZ;
    double yZ = zx * refY - zy * refX;
    normalizeVector(yX, yY, yZ);

    const double xX = yY * zz - yZ * zy;
    const double xY = yZ * zx - yX * zz;
    const double xZ = yX * zy - yY * zx;

    rotationMatrixToRotVec(xX, yX, zx, xY, yY, zy, xZ, yZ, zz, tcp.rxRad, tcp.ryRad, tcp.rzRad);
}

} // namespace



Ur3eScanTcpPose tcpPoseForHemispherePoint(const Ur3eHemisphereScanPoint &gridPoint)

{

    Ur3eScanTcpPose tcp;

    tcp.xM = gridPoint.xM;

    tcp.yM = gridPoint.yM;

    tcp.zM = gridPoint.zM + kSampleTrayHeightM;



    const Ur3eMountTransform mount =
        Ur3eMountTransform::sceneAlignFromConfig(hf::hardwareConfig().ur3e);
    mount.transformPoint(tcp.xM, tcp.yM, tcp.zM);

    // Dome axis / floor-circle center (tray origin projected through sphere center).
    double centerXM = 0.0;
    double centerYM = 0.0;
    double centerZM = kSampleTrayHeightM;
    mount.transformPoint(centerXM, centerYM, centerZM);

    const double toolZX = centerXM - tcp.xM;
    const double toolZY = centerYM - tcp.yM;
    const double toolZZ = centerZM - tcp.zM;

    buildTcpOrientationFromToolZ(toolZX, toolZY, toolZZ, tcp);

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

        poses.append(pose);

    }



    QJsonObject workspace;

    workspace.insert(QStringLiteral("enabled"), boundary.enabled);

    workspace.insert(QStringLiteral("length_m"), boundary.lengthM());

    workspace.insert(QStringLiteral("width_m"), boundary.widthM());

    workspace.insert(QStringLiteral("height_m"), boundary.heightM());

    workspace.insert(QStringLiteral("mount_height_m"), boundary.mountHeightM());

    QJsonObject body;

    body.insert(QStringLiteral("poses"), poses);

    body.insert(QStringLiteral("workspace"), workspace);

    appendUr3eScanHomeJointsToJson(body);



    const QJsonObject response =

        ur3ePostJsonRequest(serverUrl, QStringLiteral("/plan_hemisphere_scan"), body, 300000, errorMessage);

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

        planned.planningError = entry.value(QStringLiteral("error")).toString();



        const QJsonArray joints = entry.value(QStringLiteral("joints")).toArray();

        planned.jointPositionsRad.clear();

        planned.jointPositionsRad.reserve(6);

        for (const QJsonValue &jointValue : joints)

            planned.jointPositionsRad.push_back(jointValue.toDouble(0.0));



        if (planned.reachable)

            ++plan.reachableCount;

        else

            ++plan.unreachableCount;

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
        order.insert(order.end(), sorted.begin(), sorted.end());
        return;
    }

    order.insert(order.end(), entryIt, sorted.end());
    order.insert(order.end(), sorted.begin(), entryIt);
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

