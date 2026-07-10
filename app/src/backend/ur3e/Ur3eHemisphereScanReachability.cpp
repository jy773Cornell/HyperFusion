// MoveIt-backed hemisphere scan planning for UR3e (backend layer).

#include "backend/ur3e/Ur3eHemisphereScanReachability.hpp"



#include "backend/HyperFusionConfig.hpp"

#include <QJsonArray>

#include <QJsonDocument>

#include <QJsonObject>



#include <algorithm>

#include <array>

#include <cmath>
#include <limits>



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

} // namespace



Ur3eScanTcpPose tcpPoseForHemispherePoint(const Ur3eHemisphereScanPoint &gridPoint)

{

    Ur3eScanTcpPose tcp;

    tcp.xM = gridPoint.xM;

    tcp.yM = gridPoint.yM;

    tcp.zM = gridPoint.zM + kSampleTrayHeightM;



    const double centerXM = 0.0;

    const double centerYM = 0.0;

    const double centerZM = kSampleTrayHeightM;



    double toolZX = centerXM - tcp.xM;

    double toolZY = centerYM - tcp.yM;

    double toolZZ = centerZM - tcp.zM;

    if (!normalizeVector(toolZX, toolZY, toolZZ))

    {

        toolZX = 0.0;

        toolZY = 0.0;

        toolZZ = -1.0;

    }



    tcp.toolZMx = toolZX;

    tcp.toolZMy = toolZY;

    tcp.toolZMz = toolZZ;



    double refX = std::abs(toolZX) < 0.9 ? 1.0 : 0.0;

    double refY = std::abs(toolZX) < 0.9 ? 0.0 : 1.0;

    double refZ = 0.0;



    double yX = toolZY * refZ - toolZZ * refY;

    double yY = toolZZ * refX - toolZX * refZ;

    double yZ = toolZX * refY - toolZY * refX;

    normalizeVector(yX, yY, yZ);



    const double xX = yY * toolZZ - yZ * toolZY;

    const double xY = yZ * toolZX - yX * toolZZ;

    const double xZ = yX * toolZY - yY * toolZX;



    rotationMatrixToRotVec(xX, yX, toolZX, xY, yY, toolZY, xZ, yZ, toolZZ, tcp.rxRad, tcp.ryRad,

                           tcp.rzRad);

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
    double delta = candidateRad - referenceRad;
    while (delta > M_PI)
        delta -= 2.0 * M_PI;
    while (delta <= -M_PI)
        delta += 2.0 * M_PI;
    return delta;
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

void sortTopToBottomRingOrder(std::vector<int> &indices, const Ur3eHemisphereScanPlan &plan)
{
    std::sort(indices.begin(), indices.end(), [&plan](const int lhs, const int rhs) {
        const Ur3eHemisphereScanPoint &a = plan.points[static_cast<std::size_t>(lhs)].gridPoint;
        const Ur3eHemisphereScanPoint &b = plan.points[static_cast<std::size_t>(rhs)].gridPoint;
        if (a.thetaDeg != b.thetaDeg)
            return a.thetaDeg < b.thetaDeg;
        if (a.phiDeg != b.phiDeg)
            return a.phiDeg < b.phiDeg;
        return lhs < rhs;
    });
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

    sortTopToBottomRingOrder(indices, plan);
    return indices;
}

} // namespace hf::ur3e

