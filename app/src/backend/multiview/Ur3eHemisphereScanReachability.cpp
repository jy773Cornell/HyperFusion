// MoveIt-backed hemisphere scan planning for UR3e (backend layer).

#include "backend/multiview/Ur3eHemisphereScanReachability.hpp"



#include "backend/HyperFusionConfig.hpp"
#include "backend/multiview/Ur3eMountTransform.hpp"

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
constexpr double kPi = 3.14159265358979323846;

struct Mat4
{
    double m[4][4]{};

    static Mat4 identity()
    {
        Mat4 t;
        t.m[0][0] = t.m[1][1] = t.m[2][2] = t.m[3][3] = 1.0;
        return t;
    }

    static Mat4 fromRpyXyz(const double roll,
                           const double pitch,
                           const double yaw,
                           const double x,
                           const double y,
                           const double z)
    {
        const double cr = std::cos(roll);
        const double sr = std::sin(roll);
        const double cp = std::cos(pitch);
        const double sp = std::sin(pitch);
        const double cy = std::cos(yaw);
        const double sy = std::sin(yaw);
        Mat4 t = identity();
        t.m[0][0] = cy * cp;
        t.m[0][1] = cy * sp * sr - sy * cr;
        t.m[0][2] = cy * sp * cr + sy * sr;
        t.m[0][3] = x;
        t.m[1][0] = sy * cp;
        t.m[1][1] = sy * sp * sr + cy * cr;
        t.m[1][2] = sy * sp * cr - cy * sr;
        t.m[1][3] = y;
        t.m[2][0] = -sp;
        t.m[2][1] = cp * sr;
        t.m[2][2] = cp * cr;
        t.m[2][3] = z;
        return t;
    }

    Mat4 operator*(const Mat4 &rhs) const
    {
        Mat4 out;
        for (int row = 0; row < 4; ++row)
        {
            for (int col = 0; col < 4; ++col)
            {
                out.m[row][col] = m[row][0] * rhs.m[0][col] + m[row][1] * rhs.m[1][col]
                                  + m[row][2] * rhs.m[2][col] + m[row][3] * rhs.m[3][col];
            }
        }
        return out;
    }
};

Mat4 inverseRigid(const Mat4 &t)
{
    Mat4 out = Mat4::identity();
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
            out.m[i][j] = t.m[j][i];
    }
    const double px = t.m[0][3];
    const double py = t.m[1][3];
    const double pz = t.m[2][3];
    out.m[0][3] = -(out.m[0][0] * px + out.m[0][1] * py + out.m[0][2] * pz);
    out.m[1][3] = -(out.m[1][0] * px + out.m[1][1] * py + out.m[1][2] * pz);
    out.m[2][3] = -(out.m[2][0] * px + out.m[2][1] * py + out.m[2][2] * pz);
    return out;
}

Mat4 rotvecXyz(const double x,
               const double y,
               const double z,
               const double rx,
               const double ry,
               const double rz)
{
    Mat4 t = Mat4::identity();
    t.m[0][3] = x;
    t.m[1][3] = y;
    t.m[2][3] = z;
    const double ang = std::sqrt(rx * rx + ry * ry + rz * rz);
    if (ang <= 1.0e-12)
        return t;
    const double ax = rx / ang;
    const double ay = ry / ang;
    const double az = rz / ang;
    const double c = std::cos(ang);
    const double s = std::sin(ang);
    const double k = 1.0 - c;
    t.m[0][0] = c + ax * ax * k;
    t.m[0][1] = ax * ay * k - az * s;
    t.m[0][2] = ax * az * k + ay * s;
    t.m[1][0] = ay * ax * k + az * s;
    t.m[1][1] = c + ay * ay * k;
    t.m[1][2] = ay * az * k - ax * s;
    t.m[2][0] = az * ax * k - ay * s;
    t.m[2][1] = az * ay * k + ax * s;
    t.m[2][2] = c + az * az * k;
    return t;
}

void rotvecFromMat(const Mat4 &t, double &rx, double &ry, double &rz)
{
    const double m00 = t.m[0][0];
    const double m01 = t.m[0][1];
    const double m02 = t.m[0][2];
    const double m10 = t.m[1][0];
    const double m11 = t.m[1][1];
    const double m12 = t.m[1][2];
    const double m20 = t.m[2][0];
    const double m21 = t.m[2][1];
    const double m22 = t.m[2][2];
    const double trace = m00 + m11 + m22;
    const double cosAngle = std::clamp((trace - 1.0) * 0.5, -1.0, 1.0);
    const double angle = std::acos(cosAngle);
    if (angle <= 1.0e-12)
    {
        rx = ry = rz = 0.0;
        return;
    }
    if (angle > kPi - 1.0e-6 || std::abs(std::sin(angle)) < 1.0e-6)
    {
        double ax = std::sqrt(std::max(0.0, (m00 + 1.0) * 0.5));
        double ay = std::sqrt(std::max(0.0, (m11 + 1.0) * 0.5));
        double az = std::sqrt(std::max(0.0, (m22 + 1.0) * 0.5));
        if (ax >= ay && ax >= az)
        {
            ay = std::copysign(ay, m10 + m01);
            az = std::copysign(az, m20 + m02);
        }
        else if (ay >= az)
        {
            ax = std::copysign(ax, m10 + m01);
            az = std::copysign(az, m21 + m12);
        }
        else
        {
            ax = std::copysign(ax, m20 + m02);
            ay = std::copysign(ay, m21 + m12);
        }
        const double len = std::sqrt(ax * ax + ay * ay + az * az);
        if (len <= 1.0e-12)
        {
            rx = kPi;
            ry = 0.0;
            rz = 0.0;
            return;
        }
        rx = ax / len * kPi;
        ry = ay / len * kPi;
        rz = az / len * kPi;
        return;
    }
    const double inv = 0.5 / std::sin(angle);
    rx = (m21 - m12) * inv * angle;
    ry = (m02 - m20) * inv * angle;
    rz = (m10 - m01) * inv * angle;
}

Mat4 toolTcpMmToMat(const hf::HardwareConfig::Ur3eConfig::ToolTcpMm &tcp)
{
    return Mat4::fromRpyXyz(tcp.rollDeg * kPi / 180.0,
                            tcp.pitchDeg * kPi / 180.0,
                            tcp.yawDeg * kPi / 180.0,
                            tcp.xMm * 0.001,
                            tcp.yMm * 0.001,
                            tcp.zMm * 0.001);
}

void insertMoveItPoseJson(QJsonObject &pose, const Ur3eScanTcpPose &optical, const bool isApex)
{
    (void)isApex;
    const Ur3eScanTcpPose ee = optical;
    pose.insert(QStringLiteral("x"), ee.xM);
    pose.insert(QStringLiteral("y"), ee.yM);
    pose.insert(QStringLiteral("z"), ee.zM);
    pose.insert(QStringLiteral("rx"), ee.rxRad);
    pose.insert(QStringLiteral("ry"), ee.ryRad);
    pose.insert(QStringLiteral("rz"), ee.rzRad);
    pose.insert(QStringLiteral("tool_z_x"), ee.toolZMx);
    pose.insert(QStringLiteral("tool_z_y"), ee.toolZMy);
    pose.insert(QStringLiteral("tool_z_z"), ee.toolZMz);
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
    const bool isApexPin = std::abs(gridPoint.thetaDeg) <= 1.0e-9;
    if (isApexPin)
    {
        // Home XY + home orientation; Z = ring radius from the grid.
        double hx = 0.0;
        double hy = 0.0;
        double hz = 0.0;
        homeActiveTcpBaseLink(hx, hy, hz, tcp.rxRad, tcp.ryRad, tcp.rzRad,
                              tcp.toolZMx, tcp.toolZMy, tcp.toolZMz);
        (void)hz;
        tcp.xM = hx;
        tcp.yM = hy;
        mount.transformPoint(tcp.xM, tcp.yM, tcp.zM);
        return tcp;
    }
    mount.transformPoint(tcp.xM, tcp.yM, tcp.zM);
    double centerXM = 0.0;
    double centerYM = 0.0;
    scanCenterOffsetM(centerXM, centerYM);
    double centerZM = kSampleTrayHeightM;
    mount.transformPoint(centerXM, centerYM, centerZM);

    tcp.toolZMx = centerXM - tcp.xM;
    tcp.toolZMy = centerYM - tcp.yM;
    tcp.toolZMz = centerZM - tcp.zM;

    // Rings: image-up = world −Z.
    double upX = 0.0;
    double upY = 0.0;
    double upZ = -1.0;
    if (isApexPin)
        homeApexCameraUpWorld(upX, upY, upZ);
    mount.transformVector(upX, upY, upZ);

    // Nominal tool +Z = look at scan-center. pin_tcp_tilt_deg is the tip angle of that
    // optical axis away from nominal (TCP pose orientation), not a single-wrist turn.
    // + tips toward camera-up (ring up = world −Z → more look-down). Cone centers on tilted +Z.
    const double tiltDeg = hf::hardwareConfig().ur3e.pinTcpTiltDeg;
    if (!isApexPin && std::abs(tiltDeg) > 1.0e-9)
    {
        double zx = tcp.toolZMx;
        double zy = tcp.toolZMy;
        double zz = tcp.toolZMz;
        const double zLen = std::sqrt(zx * zx + zy * zy + zz * zz);
        double ux = upX;
        double uy = upY;
        double uz = upZ;
        const double uLen = std::sqrt(ux * ux + uy * uy + uz * uz);
        if (zLen > 1.0e-12 && uLen > 1.0e-12)
        {
            zx /= zLen;
            zy /= zLen;
            zz /= zLen;
            ux /= uLen;
            uy /= uLen;
            uz /= uLen;
            // Pitch axis ⊥ nominal look-at and camera-up; rotate tool +Z by tiltDeg.
            double ax = zy * uz - zz * uy;
            double ay = zz * ux - zx * uz;
            double az = zx * uy - zy * ux;
            const double aLen = std::sqrt(ax * ax + ay * ay + az * az);
            if (aLen > 1.0e-8)
            {
                ax /= aLen;
                ay /= aLen;
                az /= aLen;
                const double ang = tiltDeg * (3.14159265358979323846 / 180.0);
                const double c = std::cos(ang);
                const double s = std::sin(ang);
                const double dot = ax * zx + ay * zy + az * zz;
                tcp.toolZMx = zx * c + (ay * zz - az * zy) * s + ax * dot * (1.0 - c);
                tcp.toolZMy = zy * c + (az * zx - ax * zz) * s + ay * dot * (1.0 - c);
                tcp.toolZMz = zz * c + (ax * zy - ay * zx) * s + az * dot * (1.0 - c);
            }
        }
    }

    // Rebuild full TCP rotvec (rx,ry,rz) from tilted tool +Z + camera-up.
    const bool lockUp =
        isApexPin || hf::hardwareConfig().ur3e.scanCameraUpWorldZ;
    orientScanTcpFromToolZ(tcp, lockUp, upX, upY, upZ);

    return tcp;

}

Ur3eScanTcpPose retargetApexCameraTcpToMoveItTip(Ur3eScanTcpPose cameraTcp)
{
    return cameraTcp;
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

        const bool isApexPin =
            std::abs(gridPoints[index].thetaDeg) <= 1.0e-9;
        insertMoveItPoseJson(pose, tcp, isApexPin);
        double apexUpX = 1.0;
        double apexUpY = 0.0;
        double apexUpZ = 0.0;
        if (isApexPin)
            homeApexCameraUpWorld(apexUpX, apexUpY, apexUpZ);
        pose.insert(QStringLiteral("camera_up_x"), isApexPin ? apexUpX : 0.0);
        pose.insert(QStringLiteral("camera_up_y"), isApexPin ? apexUpY : 0.0);
        pose.insert(QStringLiteral("camera_up_z"), isApexPin ? apexUpZ : -1.0);
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

Ur3eHemisphereScanPlan evaluateSemiHemisphereScanPlanMoveIt(
    const QString &serverUrl,
    const Ur3eHemisphereScanParams &params,
    const Ur3eWorkspaceBoundary &boundary,
    const int maxSweepOkPerRing,
    QString *errorMessage)
{
    Ur3eHemisphereScanPlan plan;
    const int searchCandidates = hf::hardwareConfig().ur3e.semiRingSearchCandidates;
    const std::vector<Ur3eHemisphereScanPoint> gridPoints =
        generateSemiHemisphereScanPoints(params, searchCandidates);
    if (gridPoints.empty())
        return plan;

    QJsonArray poses;
    for (std::size_t index = 0; index < gridPoints.size(); ++index)
    {
        const Ur3eScanTcpPose tcp = tcpPoseForHemispherePoint(gridPoints[index]);
        QJsonObject pose;
        pose.insert(QStringLiteral("index"), static_cast<int>(index));
        const bool isApexPin = std::abs(gridPoints[index].thetaDeg) <= 1.0e-9;
        insertMoveItPoseJson(pose, tcp, isApexPin);
        double apexUpX = 1.0;
        double apexUpY = 0.0;
        double apexUpZ = 0.0;
        if (isApexPin)
            homeApexCameraUpWorld(apexUpX, apexUpY, apexUpZ);
        pose.insert(QStringLiteral("camera_up_x"), isApexPin ? apexUpX : 0.0);
        pose.insert(QStringLiteral("camera_up_y"), isApexPin ? apexUpY : 0.0);
        pose.insert(QStringLiteral("camera_up_z"), isApexPin ? apexUpZ : -1.0);
        pose.insert(QStringLiteral("require_perpendicular"), isApexPin);
        pose.insert(QStringLiteral("theta_deg"), gridPoints[index].thetaDeg);
        pose.insert(QStringLiteral("phi_deg"), gridPoints[index].phiDeg);
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
    body.insert(QStringLiteral("semi_ring_sweep"), true);
    body.insert(QStringLiteral("semi_max_sweep_ok_per_ring"),
                std::max(1, maxSweepOkPerRing));
    body.insert(QStringLiteral("semi_ring_search_candidates"),
                std::max(1, searchCandidates));
    appendUr3eScanHomeJointsToJson(body);

    const int planTimeoutMs = std::max(60000, hf::hardwareConfig().ur3e.planTimeoutMs);
    const QJsonObject response =
        ur3ePostJsonRequest(serverUrl, QStringLiteral("/plan_hemisphere_scan"), body,
                            planTimeoutMs, errorMessage);

    if (response.isEmpty() || !response.value(QStringLiteral("ok")).toBool(false))
    {
        if (errorMessage != nullptr && errorMessage->isEmpty())
            *errorMessage = QStringLiteral("MoveIt semi hemisphere scan planning failed.");
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
        planned.homePathOk =
            planned.reachable && entry.value(QStringLiteral("home_path_ok")).toBool(true);
        planned.baseSweepOk =
            planned.reachable && entry.value(QStringLiteral("base_sweep_ok")).toBool(false);
        planned.backupCoverageOk =
            planned.reachable && entry.value(QStringLiteral("backup_coverage_ok")).toBool(false);
        planned.backupUnionDeg = entry.value(QStringLiteral("backup_union_deg")).toDouble(0.0);
        planned.panMask.clear();
        if (entry.contains(QStringLiteral("pan_mask")) && entry.value(QStringLiteral("pan_mask")).isArray())
        {
            const QJsonArray mask = entry.value(QStringLiteral("pan_mask")).toArray();
            planned.panMask.reserve(mask.size());
            for (const QJsonValue &bit : mask)
                planned.panMask.push_back(bit.toBool(false) || bit.toInt(0) != 0 ? 1 : 0);
        }
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

bool sameHemisphereScanRing(const Ur3ePlannedScanPoint &a, const Ur3ePlannedScanPoint &b)
{
    if (std::abs(a.gridPoint.thetaDeg) < 0.75 || std::abs(b.gridPoint.thetaDeg) < 0.75)
        return false;
    return std::lround(a.gridPoint.thetaDeg * 2.0) == std::lround(b.gridPoint.thetaDeg * 2.0);
}

int reachablePinsOnSameRing(const Ur3eHemisphereScanPlan &plan,
                            const Ur3ePlannedScanPoint &ref)
{
    if (std::abs(ref.gridPoint.thetaDeg) < 0.75)
        return 0;
    int n = 0;
    for (const Ur3ePlannedScanPoint &pt : plan.points)
    {
        if (!pt.reachable || pt.jointPositionsRad.size() != 6)
            continue;
        if (sameHemisphereScanRing(ref, pt))
            ++n;
    }
    return n;
}

} // namespace hf::ur3e

