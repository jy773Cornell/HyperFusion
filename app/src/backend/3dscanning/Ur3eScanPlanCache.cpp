// Persist last hemisphere scan Plan for auto-load when cfg + UI params match.

#include "backend/3dscanning/Ur3eScanPlanCache.hpp"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

namespace hf::ur3e
{
namespace
{

constexpr int kCacheSchemaVersion = 4; // home_path_ok / chain-only marking

QJsonObject matPoseToJson(const Ur3eScanTcpPose &tcp)
{
    QJsonObject o;
    o.insert(QStringLiteral("x_m"), tcp.xM);
    o.insert(QStringLiteral("y_m"), tcp.yM);
    o.insert(QStringLiteral("z_m"), tcp.zM);
    o.insert(QStringLiteral("rx"), tcp.rxRad);
    o.insert(QStringLiteral("ry"), tcp.ryRad);
    o.insert(QStringLiteral("rz"), tcp.rzRad);
    o.insert(QStringLiteral("tool_z_x"), tcp.toolZMx);
    o.insert(QStringLiteral("tool_z_y"), tcp.toolZMy);
    o.insert(QStringLiteral("tool_z_z"), tcp.toolZMz);
    return o;
}

bool matPoseFromJson(const QJsonObject &o, Ur3eScanTcpPose &tcp)
{
    tcp.xM = o.value(QStringLiteral("x_m")).toDouble();
    tcp.yM = o.value(QStringLiteral("y_m")).toDouble();
    tcp.zM = o.value(QStringLiteral("z_m")).toDouble();
    tcp.rxRad = o.value(QStringLiteral("rx")).toDouble();
    tcp.ryRad = o.value(QStringLiteral("ry")).toDouble();
    tcp.rzRad = o.value(QStringLiteral("rz")).toDouble();
    tcp.toolZMx = o.value(QStringLiteral("tool_z_x")).toDouble(0.0);
    tcp.toolZMy = o.value(QStringLiteral("tool_z_y")).toDouble(0.0);
    tcp.toolZMz = o.value(QStringLiteral("tool_z_z")).toDouble(1.0);
    return true;
}

} // namespace

QString ur3eScanPlanFingerprint(const hf::HardwareConfig::Ur3eConfig &ur3e,
                                const Ur3eHemisphereScanParams &params)
{
    QJsonObject fp;
    fp.insert(QStringLiteral("schema"), kCacheSchemaVersion);

    // Scan UI / grid
    fp.insert(QStringLiteral("sphere_radius_m"), params.sphereRadiusM);
    fp.insert(QStringLiteral("horizontal_points"), params.horizontalPoints);
    fp.insert(QStringLiteral("vertical_points"), params.verticalPoints);
    fp.insert(QStringLiteral("theta_min_deg"), params.thetaMinDeg);
    fp.insert(QStringLiteral("theta_max_deg"), params.thetaMaxDeg);
    fp.insert(QStringLiteral("always_apex_pin"), true);
    fp.insert(QStringLiteral("apex_camera_up_world_x"), true);
    fp.insert(QStringLiteral("apex_no_cone"), true);

    // Robot geometry from hyperfusion.cfg [3d scanning]
    fp.insert(QStringLiteral("ur_type"), ur3e.urType);
    fp.insert(QStringLiteral("use_mock_hardware"), ur3e.useMockHardware);
    fp.insert(QStringLiteral("tool_tcp_x_mm"), ur3e.toolTcpXMm);
    fp.insert(QStringLiteral("tool_tcp_y_mm"), ur3e.toolTcpYMm);
    fp.insert(QStringLiteral("tool_tcp_z_mm"), ur3e.toolTcpZMm);
    fp.insert(QStringLiteral("tool_payload_shape"), ur3e.toolPayloadShape);
    fp.insert(QStringLiteral("tool_payload_mesh"), ur3e.toolPayloadMesh);
    fp.insert(QStringLiteral("tool_payload_radius_mm"), ur3e.toolPayloadRadiusMm);
    fp.insert(QStringLiteral("ceiling_mount_height_mm"), ur3e.ceilingMountHeightMm);
    fp.insert(QStringLiteral("workspace_boundary_enabled"), ur3e.workspaceBoundaryEnabled);
    fp.insert(QStringLiteral("workspace_length_mm"), ur3e.workspaceLengthMm);
    fp.insert(QStringLiteral("workspace_width_mm"), ur3e.workspaceWidthMm);
    fp.insert(QStringLiteral("workspace_height_mm"), ur3e.workspaceHeightMm);
    fp.insert(QStringLiteral("workspace_ceiling_clearance_mm"),
              ur3e.workspaceCeilingClearanceMm);
    fp.insert(QStringLiteral("mount_roll_deg"), ur3e.mountRollDeg);
    fp.insert(QStringLiteral("mount_pitch_deg"), ur3e.mountPitchDeg);
    fp.insert(QStringLiteral("mount_yaw_deg"), ur3e.mountYawDeg);
    fp.insert(QStringLiteral("mount_offset_x_mm"), ur3e.mountOffsetXMm);
    fp.insert(QStringLiteral("mount_offset_y_mm"), ur3e.mountOffsetYMm);
    fp.insert(QStringLiteral("scan_center_from_home_tcp"), true);

    QJsonArray home;
    for (const double deg : ur3e.homeJointsDeg)
        home.append(deg);
    fp.insert(QStringLiteral("home_joints_deg"), home);
    fp.insert(QStringLiteral("scan_camera_up_world_z"), ur3e.scanCameraUpWorldZ);
    fp.insert(QStringLiteral("pin_pose_tolerance_deg"), ur3e.pinPoseToleranceDeg);

    return QString::fromUtf8(
        QJsonDocument(fp).toJson(QJsonDocument::Compact));
}

QString defaultUr3eScanPlanCachePath()
{
    return QDir(QCoreApplication::applicationDirPath())
        .filePath(QStringLiteral("ur3e_last_scan_plan.json"));
}

bool saveUr3eScanPlanCache(const QString &path,
                           const QString &fingerprint,
                           const Ur3eHemisphereScanPlan &plan,
                           QString *errorMessage)
{
    if (path.isEmpty())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("scan plan cache path is empty.");
        return false;
    }

    QJsonObject root;
    root.insert(QStringLiteral("schema"), kCacheSchemaVersion);
    root.insert(QStringLiteral("fingerprint"), fingerprint);
    root.insert(QStringLiteral("reachable_count"), plan.reachableCount);
    root.insert(QStringLiteral("unreachable_count"), plan.unreachableCount);
    root.insert(QStringLiteral("home_path_ok_count"), plan.homePathOkCount);
    root.insert(QStringLiteral("chain_only_count"), plan.chainOnlyCount);
    root.insert(QStringLiteral("moveit_used"), plan.moveItUsed);
    root.insert(QStringLiteral("error_message"), plan.errorMessage);

    QJsonArray points;
    for (const Ur3ePlannedScanPoint &pt : plan.points)
    {
        QJsonObject entry;
        QJsonObject grid;
        grid.insert(QStringLiteral("phi_deg"), pt.gridPoint.phiDeg);
        grid.insert(QStringLiteral("theta_deg"), pt.gridPoint.thetaDeg);
        grid.insert(QStringLiteral("x_m"), pt.gridPoint.xM);
        grid.insert(QStringLiteral("y_m"), pt.gridPoint.yM);
        grid.insert(QStringLiteral("z_m"), pt.gridPoint.zM);
        entry.insert(QStringLiteral("grid"), grid);
        entry.insert(QStringLiteral("tcp"), matPoseToJson(pt.tcp));
        entry.insert(QStringLiteral("reachable"), pt.reachable);
        entry.insert(QStringLiteral("home_path_ok"), pt.homePathOk);
        entry.insert(QStringLiteral("planning_error"), pt.planningError);

        QJsonArray joints;
        for (const double rad : pt.jointPositionsRad)
            joints.append(rad);
        entry.insert(QStringLiteral("joints_rad"), joints);
        points.append(entry);
    }
    root.insert(QStringLiteral("points"), points);

    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text))
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Could not write %1").arg(path);
        return false;
    }

    const QByteArray payload = QJsonDocument(root).toJson(QJsonDocument::Indented);
    if (file.write(payload) != payload.size())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Incomplete write: %1").arg(path);
        return false;
    }
    return true;
}

bool loadUr3eScanPlanCache(const QString &path,
                           const QString &expectedFingerprint,
                           Ur3eHemisphereScanPlan &planOut,
                           QString *errorMessage)
{
    planOut = {};
    if (path.isEmpty() || !QFile::exists(path))
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("No cached scan plan file.");
        return false;
    }

    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Could not read %1").arg(path);
        return false;
    }

    QJsonParseError parseError;
    const QJsonDocument doc = QJsonDocument::fromJson(file.readAll(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !doc.isObject())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Invalid scan plan cache JSON.");
        return false;
    }

    const QJsonObject root = doc.object();
    if (root.value(QStringLiteral("schema")).toInt() != kCacheSchemaVersion)
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Scan plan cache schema mismatch.");
        return false;
    }

    const QString fingerprint = root.value(QStringLiteral("fingerprint")).toString();
    if (fingerprint.isEmpty() || fingerprint != expectedFingerprint)
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral(
                "Cached plan does not match current robot cfg / scan parameters.");
        return false;
    }

    planOut.reachableCount = root.value(QStringLiteral("reachable_count")).toInt();
    planOut.unreachableCount = root.value(QStringLiteral("unreachable_count")).toInt();
    planOut.homePathOkCount = root.value(QStringLiteral("home_path_ok_count")).toInt(-1);
    planOut.chainOnlyCount = root.value(QStringLiteral("chain_only_count")).toInt(-1);
    planOut.moveItUsed = root.value(QStringLiteral("moveit_used")).toBool(true);
    planOut.errorMessage = root.value(QStringLiteral("error_message")).toString();

    const QJsonArray points = root.value(QStringLiteral("points")).toArray();
    planOut.points.reserve(points.size());
    int recomputedHomeOk = 0;
    int recomputedChain = 0;
    for (const QJsonValue &value : points)
    {
        if (!value.isObject())
            continue;
        const QJsonObject entry = value.toObject();
        Ur3ePlannedScanPoint pt;
        const QJsonObject grid = entry.value(QStringLiteral("grid")).toObject();
        pt.gridPoint.phiDeg = grid.value(QStringLiteral("phi_deg")).toDouble();
        pt.gridPoint.thetaDeg = grid.value(QStringLiteral("theta_deg")).toDouble();
        pt.gridPoint.xM = grid.value(QStringLiteral("x_m")).toDouble();
        pt.gridPoint.yM = grid.value(QStringLiteral("y_m")).toDouble();
        pt.gridPoint.zM = grid.value(QStringLiteral("z_m")).toDouble();
        matPoseFromJson(entry.value(QStringLiteral("tcp")).toObject(), pt.tcp);
        pt.reachable = entry.value(QStringLiteral("reachable")).toBool();
        // Schema 4+: explicit field. Older caches default true when reachable.
        pt.homePathOk =
            pt.reachable && entry.value(QStringLiteral("home_path_ok")).toBool(true);
        pt.planningError = entry.value(QStringLiteral("planning_error")).toString();
        const QJsonArray joints = entry.value(QStringLiteral("joints_rad")).toArray();
        pt.jointPositionsRad.reserve(joints.size());
        for (const QJsonValue &j : joints)
            pt.jointPositionsRad.push_back(j.toDouble());
        if (pt.reachable)
        {
            if (pt.homePathOk)
                ++recomputedHomeOk;
            else
                ++recomputedChain;
        }
        planOut.points.push_back(std::move(pt));
    }

    if (planOut.homePathOkCount < 0)
        planOut.homePathOkCount = recomputedHomeOk;
    if (planOut.chainOnlyCount < 0)
        planOut.chainOnlyCount = recomputedChain;

    if (planOut.points.empty())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Cached scan plan has no points.");
        return false;
    }
    return true;
}

} // namespace hf::ur3e
