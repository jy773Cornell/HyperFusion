// Persist hemisphere scan plans: last-plan cache + named routes library.

#include "backend/multiview/Ur3eScanPlanCache.hpp"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

namespace hf::ur3e
{
namespace
{

constexpr int kCacheSchemaVersion = 4; // home_path_ok / chain-only marking

/// Compare robot-cfg fingerprints ignoring use_mock_hardware (sim ↔ real).
bool robotCfgFingerprintsMatch(const QString &a, const QString &b)
{
    if (a == b)
        return true;
    if (a.isEmpty() || b.isEmpty())
        return false;

    auto stripMock = [](const QString &raw) -> QString {
        QJsonParseError err;
        const QJsonDocument doc = QJsonDocument::fromJson(raw.toUtf8(), &err);
        if (err.error != QJsonParseError::NoError || !doc.isObject())
            return raw;
        QJsonObject o = doc.object();
        o.remove(QStringLiteral("use_mock_hardware"));
        return QString::fromUtf8(QJsonDocument(o).toJson(QJsonDocument::Compact));
    };

    return stripMock(a) == stripMock(b);
}

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

QJsonObject robotCfgFingerprintObject(const hf::HardwareConfig::Ur3eConfig &ur3e)
{
    QJsonObject fp;
    fp.insert(QStringLiteral("schema"), kCacheSchemaVersion);
    fp.insert(QStringLiteral("ur_type"), ur3e.urType);
    // Intentionally omit use_mock_hardware — sim plans must load on real robot (same geometry).
    fp.insert(QStringLiteral("tool_tcp_x_mm"), ur3e.toolTcpXMm);
    fp.insert(QStringLiteral("tool_tcp_y_mm"), ur3e.toolTcpYMm);
    fp.insert(QStringLiteral("tool_tcp_z_mm"), ur3e.toolTcpZMm);
    fp.insert(QStringLiteral("tool_tcp_roll_deg"), ur3e.toolTcpRollDeg);
    fp.insert(QStringLiteral("tool_tcp_pitch_deg"), ur3e.toolTcpPitchDeg);
    fp.insert(QStringLiteral("tool_tcp_yaw_deg"), ur3e.toolTcpYawDeg);
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
    fp.insert(QStringLiteral("scan_center_from_home_tcp"), false);
    fp.insert(QStringLiteral("scan_center_base_xy"), true);
    fp.insert(QStringLiteral("apex_over_home_tcp_xy"), true);

    QJsonArray home;
    for (const double deg : ur3e.homeJointsDeg)
        home.append(deg);
    fp.insert(QStringLiteral("home_joints_deg"), home);
    fp.insert(QStringLiteral("scan_camera_up_world_z"), ur3e.scanCameraUpWorldZ);
    fp.insert(QStringLiteral("pin_pose_tolerance_deg"), ur3e.pinPoseToleranceDeg);
    fp.insert(QStringLiteral("pin_pose_tolerance_vertical_only"), true);
    fp.insert(QStringLiteral("pin_tcp_tilt_deg"), ur3e.pinTcpTiltDeg);
    fp.insert(QStringLiteral("semi_ring_search_candidates"), ur3e.semiRingSearchCandidates);
    return fp;
}

QJsonObject scanParamsToJson(const Ur3eHemisphereScanParams &params)
{
    QJsonObject o;
    o.insert(QStringLiteral("sphere_radius_m"), params.sphereRadiusM);
    o.insert(QStringLiteral("horizontal_points"), params.horizontalPoints);
    o.insert(QStringLiteral("vertical_points"), params.verticalPoints);
    o.insert(QStringLiteral("theta_min_deg"), params.thetaMinDeg);
    o.insert(QStringLiteral("theta_max_deg"), params.thetaMaxDeg);
    return o;
}

Ur3eHemisphereScanParams scanParamsFromJson(const QJsonObject &o)
{
    Ur3eHemisphereScanParams params;
    params.sphereRadiusM = o.value(QStringLiteral("sphere_radius_m")).toDouble(0.5);
    params.horizontalPoints = o.value(QStringLiteral("horizontal_points")).toInt(12);
    params.verticalPoints = o.value(QStringLiteral("vertical_points")).toInt(5);
    params.thetaMinDeg = o.value(QStringLiteral("theta_min_deg")).toDouble(30.0);
    params.thetaMaxDeg = o.value(QStringLiteral("theta_max_deg")).toDouble(90.0);
    normalizeHemisphereScanParams(params);
    return params;
}

QJsonObject planPointsToJson(const Ur3eHemisphereScanPlan &plan)
{
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
        entry.insert(QStringLiteral("base_sweep_ok"), pt.baseSweepOk);
        entry.insert(QStringLiteral("planning_error"), pt.planningError);

        QJsonArray joints;
        for (const double rad : pt.jointPositionsRad)
            joints.append(rad);
        entry.insert(QStringLiteral("joints_rad"), joints);
        points.append(entry);
    }
    QJsonObject meta;
    meta.insert(QStringLiteral("points"), points);
    meta.insert(QStringLiteral("reachable_count"), plan.reachableCount);
    meta.insert(QStringLiteral("unreachable_count"), plan.unreachableCount);
    meta.insert(QStringLiteral("home_path_ok_count"), plan.homePathOkCount);
    meta.insert(QStringLiteral("chain_only_count"), plan.chainOnlyCount);
    meta.insert(QStringLiteral("moveit_used"), plan.moveItUsed);
    meta.insert(QStringLiteral("error_message"), plan.errorMessage);
    return meta;
}

bool planFromJsonRoot(const QJsonObject &root, Ur3eHemisphereScanPlan &planOut, QString *errorMessage)
{
    planOut = {};
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
        pt.homePathOk =
            pt.reachable && entry.value(QStringLiteral("home_path_ok")).toBool(true);
        pt.baseSweepOk =
            pt.reachable && entry.value(QStringLiteral("base_sweep_ok")).toBool(false);
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
            *errorMessage = QStringLiteral("Scan plan has no points.");
        return false;
    }
    return true;
}

bool writeJsonFile(const QString &path, const QJsonObject &root, QString *errorMessage)
{
    QFileInfo info(path);
    if (!info.dir().exists() && !QDir().mkpath(info.absolutePath()))
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Could not create directory for %1").arg(path);
        return false;
    }

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

} // namespace

QString ur3eScanRobotCfgFingerprint(const hf::HardwareConfig::Ur3eConfig &ur3e)
{
    return QString::fromUtf8(
        QJsonDocument(robotCfgFingerprintObject(ur3e)).toJson(QJsonDocument::Compact));
}

QString ur3eScanPlanFingerprint(const hf::HardwareConfig::Ur3eConfig &ur3e,
                                const Ur3eHemisphereScanParams &params)
{
    QJsonObject fp = robotCfgFingerprintObject(ur3e);
    // Scan UI / grid
    fp.insert(QStringLiteral("sphere_radius_m"), params.sphereRadiusM);
    fp.insert(QStringLiteral("horizontal_points"), params.horizontalPoints);
    fp.insert(QStringLiteral("vertical_points"), params.verticalPoints);
    fp.insert(QStringLiteral("theta_min_deg"), params.thetaMinDeg);
    fp.insert(QStringLiteral("theta_max_deg"), params.thetaMaxDeg);
    fp.insert(QStringLiteral("always_apex_pin"), true);
    fp.insert(QStringLiteral("apex_camera_up_world_x"), true);
    fp.insert(QStringLiteral("apex_no_cone"), true);
    return QString::fromUtf8(QJsonDocument(fp).toJson(QJsonDocument::Compact));
}

QString defaultUr3eScanPlanCachePath()
{
    return QDir(QCoreApplication::applicationDirPath())
        .filePath(QStringLiteral("ur3e_last_scan_plan.json"));
}

QString defaultUr3eScanRoutesDir()
{
    return QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("ur3e_scan_routes"));
}

QString defaultUr3eScanRouteDisplayName(const Ur3eHemisphereScanParams &params)
{
    // Keep short for the settings combo (long labels force the panel wider than the splitter).
    return QStringLiteral("R%1 %2×%3 θ%4–%5")
        .arg(qRound(params.sphereRadiusM * 1000.0))
        .arg(params.horizontalPoints)
        .arg(params.verticalPoints)
        .arg(params.thetaMinDeg, 0, 'f', 0)
        .arg(params.thetaMaxDeg, 0, 'f', 0);
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

    QJsonObject root = planPointsToJson(plan);
    root.insert(QStringLiteral("schema"), kCacheSchemaVersion);
    root.insert(QStringLiteral("fingerprint"), fingerprint);
    return writeJsonFile(path, root, errorMessage);
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
    if (fingerprint.isEmpty() || !robotCfgFingerprintsMatch(fingerprint, expectedFingerprint))
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral(
                "Cached plan does not match current robot cfg / scan parameters.");
        return false;
    }

    return planFromJsonRoot(root, planOut, errorMessage);
}

bool saveUr3eNamedScanRoute(const QString &path,
                            const QString &displayName,
                            const QString &fingerprint,
                            const QString &robotCfgFingerprint,
                            const Ur3eHemisphereScanParams &params,
                            const Ur3eHemisphereScanPlan &plan,
                            QString *errorMessage)
{
    if (path.isEmpty())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("scan route path is empty.");
        return false;
    }

    QJsonObject root = planPointsToJson(plan);
    root.insert(QStringLiteral("schema"), kCacheSchemaVersion);
    root.insert(QStringLiteral("name"), displayName);
    root.insert(QStringLiteral("fingerprint"), fingerprint);
    root.insert(QStringLiteral("robot_cfg_fingerprint"), robotCfgFingerprint);
    root.insert(QStringLiteral("scan_params"), scanParamsToJson(params));
    return writeJsonFile(path, root, errorMessage);
}

QVector<Ur3eScanRouteInfo>
listUr3eScanRoutesMatchingCfg(const QString &dir,
                              const QString &expectedRobotCfgFingerprint)
{
    QVector<Ur3eScanRouteInfo> out;
    if (dir.isEmpty() || expectedRobotCfgFingerprint.isEmpty())
        return out;

    QDir routesDir(dir);
    if (!routesDir.exists())
        return out;

    const QFileInfoList files =
        routesDir.entryInfoList({QStringLiteral("*.json")}, QDir::Files, QDir::Name);
    for (const QFileInfo &info : files)
    {
        QFile file(info.absoluteFilePath());
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
            continue;

        QJsonParseError parseError;
        const QJsonDocument doc = QJsonDocument::fromJson(file.readAll(), &parseError);
        if (parseError.error != QJsonParseError::NoError || !doc.isObject())
            continue;

        const QJsonObject root = doc.object();
        if (root.value(QStringLiteral("schema")).toInt() != kCacheSchemaVersion)
            continue;

        const QString robotFp = root.value(QStringLiteral("robot_cfg_fingerprint")).toString();
        if (robotFp.isEmpty() || !robotCfgFingerprintsMatch(robotFp, expectedRobotCfgFingerprint))
            continue;

        Ur3eScanRouteInfo entry;
        entry.path = info.absoluteFilePath();
        entry.id = info.completeBaseName();
        entry.displayName = root.value(QStringLiteral("name")).toString();
        if (entry.displayName.isEmpty())
            entry.displayName = entry.id;
        entry.fingerprint = root.value(QStringLiteral("fingerprint")).toString();
        entry.robotCfgFingerprint = robotFp;
        entry.params = scanParamsFromJson(root.value(QStringLiteral("scan_params")).toObject());
        entry.pointCount = root.value(QStringLiteral("points")).toArray().size();
        entry.reachableCount = root.value(QStringLiteral("reachable_count")).toInt();
        out.push_back(std::move(entry));
    }
    return out;
}

bool loadUr3eNamedScanRoute(const QString &path,
                            const QString &expectedRobotCfgFingerprint,
                            Ur3eHemisphereScanPlan &planOut,
                            Ur3eHemisphereScanParams *paramsOut,
                            QString *displayNameOut,
                            QString *errorMessage)
{
    planOut = {};
    if (path.isEmpty() || !QFile::exists(path))
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Scan route file not found.");
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
            *errorMessage = QStringLiteral("Invalid scan route JSON.");
        return false;
    }

    const QJsonObject root = doc.object();
    if (root.value(QStringLiteral("schema")).toInt() != kCacheSchemaVersion)
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Scan route schema mismatch.");
        return false;
    }

    const QString robotFp = root.value(QStringLiteral("robot_cfg_fingerprint")).toString();
    if (robotFp.isEmpty() || !robotCfgFingerprintsMatch(robotFp, expectedRobotCfgFingerprint))
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral(
                "Scan route does not match current robot cfg (hyperfusion.cfg).");
        return false;
    }

    if (paramsOut != nullptr)
        *paramsOut = scanParamsFromJson(root.value(QStringLiteral("scan_params")).toObject());
    if (displayNameOut != nullptr)
    {
        *displayNameOut = root.value(QStringLiteral("name")).toString();
        if (displayNameOut->isEmpty())
            *displayNameOut = QFileInfo(path).completeBaseName();
    }

    return planFromJsonRoot(root, planOut, errorMessage);
}

} // namespace hf::ur3e
