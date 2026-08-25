// Semi-fixed ring route persistence + preview ring inference (backend).

#include "backend/multiview/Ur3eSemiFixedScan.hpp"

#include "backend/multiview/Ur3eHemisphereScan.hpp"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QHash>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include <algorithm>
#include <cmath>

namespace hf::ur3e
{
namespace
{

constexpr int kSchemaVersion = 1;

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

QJsonObject tcpToJson(const Ur3eScanTcpPose &tcp)
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

Ur3eScanTcpPose tcpFromJson(const QJsonObject &o)
{
    Ur3eScanTcpPose tcp;
    tcp.xM = o.value(QStringLiteral("x_m")).toDouble();
    tcp.yM = o.value(QStringLiteral("y_m")).toDouble();
    tcp.zM = o.value(QStringLiteral("z_m")).toDouble();
    tcp.rxRad = o.value(QStringLiteral("rx")).toDouble();
    tcp.ryRad = o.value(QStringLiteral("ry")).toDouble();
    tcp.rzRad = o.value(QStringLiteral("rz")).toDouble();
    tcp.toolZMx = o.value(QStringLiteral("tool_z_x")).toDouble(0.0);
    tcp.toolZMy = o.value(QStringLiteral("tool_z_y")).toDouble(0.0);
    tcp.toolZMz = o.value(QStringLiteral("tool_z_z")).toDouble(1.0);
    return tcp;
}

QJsonArray jointsToJson(const std::vector<double> &joints)
{
    QJsonArray a;
    for (const double v : joints)
        a.append(v);
    return a;
}

std::vector<double> jointsFromJson(const QJsonArray &a)
{
    std::vector<double> out;
    out.reserve(static_cast<std::size_t>(a.size()));
    for (const QJsonValue &v : a)
        out.push_back(v.toDouble());
    return out;
}

} // namespace

Ur3eSemiFixedRing defaultSemiFixedTopPose()
{
    // Fixed apex look-down (θ=0) — always first still on semi-fixed execute.
    Ur3eSemiFixedRing top;
    top.id = QStringLiteral("top");
    top.displayName = QStringLiteral("Top (θ=0)");
    top.entryJointsRad = {
        1.320915979459176,
        -3.345937397893044,
        2.4709152402327033,
        -0.6957748605175326,
        1.5707963267948966,
        -1.3209159800161308,
    };
    top.entryTcp.xM = -0.07501500004324846;
    top.entryTcp.yM = -0.016443060582171126;
    top.entryTcp.zM = 0.2;
    top.entryTcp.rxRad = -2.221441296101876;
    top.entryTcp.ryRad = 2.2214412956462506;
    top.entryTcp.rzRad = 3.852205944410706e-07;
    top.entryTcp.toolZMx = 0.0;
    top.entryTcp.toolZMy = 0.0;
    top.entryTcp.toolZMz = -1.0;
    top.hasEntryTcp = true;
    // Known-good planned apex pose — show green in preview.
    top.reachabilityKnown = true;
    top.reachable = true;
    top.homePathOk = true;
    return top;
}

void ensureSemiFixedTopPose(Ur3eSemiFixedRoute &route)
{
    if (route.hasTopPose && route.topPose.entryJointsRad.size() == 6)
        return;
    route.topPose = defaultSemiFixedTopPose();
    route.hasTopPose = true;
}

QString defaultUr3eSemiScanRoutesDir()
{
    return QDir(QCoreApplication::applicationDirPath())
        .filePath(QStringLiteral("ur3e_semi_scan_routes"));
}

bool saveUr3eSemiFixedRoute(const QString &path,
                            const Ur3eSemiFixedRoute &route,
                            QString *errorMessage)
{
    QDir().mkpath(QFileInfo(path).absolutePath());

    Ur3eSemiFixedRoute toSave = route;
    ensureSemiFixedTopPose(toSave);

    QJsonObject root;
    root.insert(QStringLiteral("schema"), kSchemaVersion);
    root.insert(QStringLiteral("kind"), QStringLiteral("ur3e_semi_fixed_route"));
    root.insert(QStringLiteral("id"), toSave.id);
    root.insert(QStringLiteral("display_name"), toSave.displayName);
    root.insert(QStringLiteral("robot_cfg_fingerprint"), toSave.robotCfgFingerprint);
    root.insert(QStringLiteral("interval_deg"), toSave.intervalDeg);
    root.insert(QStringLiteral("pan_direction"), toSave.panDirection);
    root.insert(QStringLiteral("stabilize_ms"), toSave.stabilizeMs);

    {
        QJsonObject top;
        top.insert(QStringLiteral("id"), toSave.topPose.id);
        top.insert(QStringLiteral("display_name"), toSave.topPose.displayName);
        top.insert(QStringLiteral("entry_joints_rad"),
                   jointsToJson(toSave.topPose.entryJointsRad));
        if (toSave.topPose.hasEntryTcp)
            top.insert(QStringLiteral("entry_tcp"), tcpToJson(toSave.topPose.entryTcp));
        if (toSave.topPose.reachabilityKnown)
        {
            top.insert(QStringLiteral("reachable"), toSave.topPose.reachable);
            top.insert(QStringLiteral("home_path_ok"), toSave.topPose.homePathOk);
        }
        root.insert(QStringLiteral("top_pose"), top);
    }

    QJsonArray rings;
    for (const Ur3eSemiFixedRing &ring : toSave.rings)
    {
        QJsonObject o;
        o.insert(QStringLiteral("id"), ring.id);
        o.insert(QStringLiteral("display_name"), ring.displayName);
        o.insert(QStringLiteral("entry_joints_rad"), jointsToJson(ring.entryJointsRad));
        if (ring.hasEntryTcp)
            o.insert(QStringLiteral("entry_tcp"), tcpToJson(ring.entryTcp));
        if (ring.reachabilityKnown)
        {
            o.insert(QStringLiteral("reachable"), ring.reachable);
            o.insert(QStringLiteral("home_path_ok"), ring.homePathOk);
        }
        rings.append(o);
    }
    root.insert(QStringLiteral("rings"), rings);

    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text))
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Cannot write %1").arg(path);
        return false;
    }
    file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    return true;
}

bool loadUr3eSemiFixedRoute(const QString &path,
                            const QString &expectedRobotCfgFingerprint,
                            Ur3eSemiFixedRoute &routeOut,
                            QString *errorMessage)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Cannot read %1").arg(path);
        return false;
    }

    QJsonParseError err;
    const QJsonDocument doc = QJsonDocument::fromJson(file.readAll(), &err);
    if (err.error != QJsonParseError::NoError || !doc.isObject())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Invalid JSON in %1").arg(path);
        return false;
    }

    const QJsonObject root = doc.object();
    const QString fp = root.value(QStringLiteral("robot_cfg_fingerprint")).toString();
    // Empty fingerprint = hand-authored / shared route — skip cfg gate.
    if (!fp.isEmpty() && !expectedRobotCfgFingerprint.isEmpty()
        && !robotCfgFingerprintsMatch(fp, expectedRobotCfgFingerprint))
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Route robot cfg fingerprint mismatch");
        return false;
    }

    routeOut = {};
    routeOut.id = root.value(QStringLiteral("id")).toString(QFileInfo(path).completeBaseName());
    routeOut.displayName =
        root.value(QStringLiteral("display_name")).toString(routeOut.id);
    routeOut.robotCfgFingerprint = fp;
    routeOut.intervalDeg = root.value(QStringLiteral("interval_deg")).toDouble(10.0);
    routeOut.panDirection = root.value(QStringLiteral("pan_direction")).toInt(1);
    if (routeOut.panDirection >= 0)
        routeOut.panDirection = 1;
    else
        routeOut.panDirection = -1;
    routeOut.stabilizeMs = root.value(QStringLiteral("stabilize_ms")).toInt(500);

    if (root.contains(QStringLiteral("top_pose")) && root.value(QStringLiteral("top_pose")).isObject())
    {
        const QJsonObject top = root.value(QStringLiteral("top_pose")).toObject();
        Ur3eSemiFixedRing pose;
        pose.id = top.value(QStringLiteral("id")).toString(QStringLiteral("top"));
        pose.displayName =
            top.value(QStringLiteral("display_name")).toString(QStringLiteral("Top (θ=0)"));
        pose.entryJointsRad =
            jointsFromJson(top.value(QStringLiteral("entry_joints_rad")).toArray());
        if (pose.entryJointsRad.size() == 6)
        {
            if (top.contains(QStringLiteral("entry_tcp")))
            {
                pose.entryTcp = tcpFromJson(top.value(QStringLiteral("entry_tcp")).toObject());
                pose.hasEntryTcp = true;
            }
            // Saved top is an intentional init pose — color like a reachable plan pin.
            pose.reachabilityKnown = true;
            pose.reachable = top.value(QStringLiteral("reachable")).toBool(true);
            pose.homePathOk = top.value(QStringLiteral("home_path_ok")).toBool(true);
            routeOut.topPose = pose;
            routeOut.hasTopPose = true;
        }
    }
    ensureSemiFixedTopPose(routeOut);

    const QJsonArray rings = root.value(QStringLiteral("rings")).toArray();
    for (const QJsonValue &v : rings)
    {
        if (!v.isObject())
            continue;
        const QJsonObject o = v.toObject();
        Ur3eSemiFixedRing ring;
        ring.id = o.value(QStringLiteral("id")).toString();
        ring.displayName = o.value(QStringLiteral("display_name")).toString(ring.id);
        ring.entryJointsRad = jointsFromJson(o.value(QStringLiteral("entry_joints_rad")).toArray());
        if (ring.entryJointsRad.size() != 6)
            continue;
        if (o.contains(QStringLiteral("entry_tcp")))
        {
            ring.entryTcp = tcpFromJson(o.value(QStringLiteral("entry_tcp")).toObject());
            ring.hasEntryTcp = true;
        }
        if (o.contains(QStringLiteral("reachable")))
        {
            ring.reachabilityKnown = true;
            ring.reachable = o.value(QStringLiteral("reachable")).toBool(false);
            ring.homePathOk = o.value(QStringLiteral("home_path_ok")).toBool(true);
        }
        if (ring.id.isEmpty())
            ring.id = QStringLiteral("ring_%1").arg(routeOut.rings.size() + 1);
        if (ring.displayName.isEmpty())
            ring.displayName = ring.id;
        routeOut.rings.push_back(ring);
    }

    if (routeOut.rings.isEmpty())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Route has no valid rings");
        return false;
    }
    return true;
}

QVector<Ur3eSemiFixedRouteInfo>
listUr3eSemiFixedRoutesMatchingCfg(const QString &dir,
                                   const QString &expectedRobotCfgFingerprint)
{
    QVector<Ur3eSemiFixedRouteInfo> out;
    QDir routesDir(dir);
    if (!routesDir.exists())
        return out;

    const QFileInfoList files =
        routesDir.entryInfoList({QStringLiteral("*.json")}, QDir::Files, QDir::Name);
    for (const QFileInfo &info : files)
    {
        Ur3eSemiFixedRoute route;
        QString err;
        if (!loadUr3eSemiFixedRoute(info.absoluteFilePath(), expectedRobotCfgFingerprint, route,
                                    &err))
            continue;
        Ur3eSemiFixedRouteInfo entry;
        entry.id = route.id;
        entry.displayName = route.displayName;
        entry.path = info.absoluteFilePath();
        entry.robotCfgFingerprint = route.robotCfgFingerprint;
        entry.ringCount = route.rings.size();
        entry.intervalDeg = route.intervalDeg;
        out.push_back(entry);
    }
    return out;
}

Ur3eSemiFixedPreviewRing inferSemiFixedPreviewRing(const Ur3eSemiFixedRing &ring)
{
    Ur3eSemiFixedPreviewRing preview;
    preview.displayName = ring.displayName;
    preview.reachabilityKnown = ring.reachabilityKnown;
    preview.reachable = ring.reachable;
    preview.homePathOk = ring.homePathOk;
    double cx = 0.0;
    double cy = 0.0;
    scanCenterOffsetM(cx, cy);
    preview.centerXM = cx;
    preview.centerYM = cy;

    if (ring.hasEntryTcp)
    {
        preview.centerZM = ring.entryTcp.zM;
        const double dx = ring.entryTcp.xM - cx;
        const double dy = ring.entryTcp.yM - cy;
        preview.radiusM = std::hypot(dx, dy);
        if (preview.radiusM < 0.02)
            preview.radiusM = 0.05;
    }
    else
    {
        preview.centerZM = 0.25;
        preview.radiusM = 0.15;
    }
    return preview;
}

QVector<Ur3eSemiFixedPreviewRing> inferSemiFixedPreviewRings(const Ur3eSemiFixedRoute &route)
{
    QVector<Ur3eSemiFixedPreviewRing> out;
    out.reserve(route.rings.size() + (route.hasTopPose ? 1 : 0));
    // Rings first so execute ringIndex 0..N-1 still maps 1:1.
    for (const Ur3eSemiFixedRing &ring : route.rings)
        out.push_back(inferSemiFixedPreviewRing(ring));
    if (route.hasTopPose)
    {
        Ur3eSemiFixedPreviewRing top = inferSemiFixedPreviewRing(route.topPose);
        top.isTopPose = true;
        if (top.displayName.isEmpty())
            top.displayName = QStringLiteral("Top");
        // Apex look-down: draw as a pin at TCP, not a spin ring.
        if (route.topPose.hasEntryTcp)
        {
            top.centerXM = route.topPose.entryTcp.xM;
            top.centerYM = route.topPose.entryTcp.yM;
            top.centerZM = route.topPose.entryTcp.zM;
            top.radiusM = 0.0;
        }
        out.push_back(top);
    }
    return out;
}

Ur3eSemiFixedRoute semiFixedRouteFromHemispherePlan(const Ur3eHemisphereScanPlan &plan,
                                                    const QString &robotCfgFingerprint,
                                                    const QString &displayName,
                                                    const double intervalDeg,
                                                    const int panDirection)
{
    // One ring entry per latitude. Prefer base_sweep_ok + home↔pin, then
    // nearest joints to configured home.
    struct Candidate
    {
        int index = -1;
        double thetaDeg = 0.0;
        double homeDist = 1.0e9;
        bool baseSweepOk = false;
        bool homePathOk = false;
    };

    const std::vector<double> homeJoints = ur3eScanHomeJointsRadFromConfig();

    QHash<int, Candidate> bestByThetaKey;
    int apexIndex = -1;
    for (int i = 0; i < static_cast<int>(plan.points.size()); ++i)
    {
        const Ur3ePlannedScanPoint &pt = plan.points[static_cast<std::size_t>(i)];
        if (!pt.reachable || pt.jointPositionsRad.size() != 6)
            continue;

        if (std::abs(pt.gridPoint.thetaDeg) < 0.75)
        {
            // Top/apex: require verified return-home (home_path_ok).
            if (!pt.homePathOk)
                continue;
            if (apexIndex < 0
                || ur3eJointDistanceRad(homeJoints, pt.jointPositionsRad)
                       < ur3eJointDistanceRad(
                             homeJoints,
                             plan.points[static_cast<std::size_t>(apexIndex)].jointPositionsRad)
                           - 1e-9)
                apexIndex = i;
            continue;
        }

        // Semi plans: base-sweep OK and pin↔home (home_path_ok) required.
        if (!pt.baseSweepOk || !pt.homePathOk)
            continue;

        const int thetaKey = static_cast<int>(std::lround(pt.gridPoint.thetaDeg * 2.0));
        Candidate cand;
        cand.index = i;
        cand.thetaDeg = pt.gridPoint.thetaDeg;
        cand.homeDist = ur3eJointDistanceRad(homeJoints, pt.jointPositionsRad);
        cand.baseSweepOk = pt.baseSweepOk;
        cand.homePathOk = pt.homePathOk;

        const auto it = bestByThetaKey.constFind(thetaKey);
        if (it == bestByThetaKey.cend())
        {
            bestByThetaKey.insert(thetaKey, cand);
            continue;
        }

        const Candidate &cur = it.value();
        const bool betterDist = cand.homeDist < cur.homeDist - 1e-9;
        const bool betterIndex = std::abs(cand.homeDist - cur.homeDist) <= 1e-9
                                 && cand.index < cur.index;
        if (betterDist || betterIndex)
            bestByThetaKey.insert(thetaKey, cand);
    }

    // Fallback: if plan has no base_sweep_ok markers (legacy), use reachable
    // pins that still have home_path_ok (return-home proven).
    if (bestByThetaKey.isEmpty())
    {
        for (int i = 0; i < static_cast<int>(plan.points.size()); ++i)
        {
            const Ur3ePlannedScanPoint &pt = plan.points[static_cast<std::size_t>(i)];
            if (!pt.reachable || pt.jointPositionsRad.size() != 6)
                continue;
            if (!pt.homePathOk)
                continue;
            if (std::abs(pt.gridPoint.thetaDeg) < 0.75)
                continue;
            const int thetaKey = static_cast<int>(std::lround(pt.gridPoint.thetaDeg * 2.0));
            Candidate cand;
            cand.index = i;
            cand.thetaDeg = pt.gridPoint.thetaDeg;
            cand.homeDist = ur3eJointDistanceRad(homeJoints, pt.jointPositionsRad);
            cand.homePathOk = pt.homePathOk;
            const auto it = bestByThetaKey.constFind(thetaKey);
            if (it == bestByThetaKey.cend()
                || cand.homeDist < it.value().homeDist)
            {
                bestByThetaKey.insert(thetaKey, cand);
            }
        }
    }

    QVector<Candidate> ordered;
    ordered.reserve(bestByThetaKey.size());
    for (auto it = bestByThetaKey.cbegin(); it != bestByThetaKey.cend(); ++it)
        ordered.push_back(it.value());
    std::sort(ordered.begin(), ordered.end(),
              [](const Candidate &a, const Candidate &b) { return a.thetaDeg < b.thetaDeg; });

    Ur3eSemiFixedRoute route;
    route.id = QStringLiteral("from_semi_plan");
    route.displayName =
        displayName.isEmpty() ? QStringLiteral("Semi ring-sweep plan") : displayName;
    route.robotCfgFingerprint = robotCfgFingerprint;
    route.intervalDeg = intervalDeg > 0.0 ? intervalDeg : 10.0;
    route.panDirection = panDirection >= 0 ? 1 : -1;
    route.stabilizeMs = 500;

    if (apexIndex >= 0)
    {
        const Ur3ePlannedScanPoint &apex =
            plan.points[static_cast<std::size_t>(apexIndex)];
        route.topPose.id = QStringLiteral("top");
        route.topPose.displayName = QStringLiteral("Top (θ=0)");
        route.topPose.entryJointsRad = apex.jointPositionsRad;
        route.topPose.entryTcp = apex.tcp;
        route.topPose.hasEntryTcp = true;
        route.topPose.reachabilityKnown = true;
        route.topPose.reachable = apex.reachable;
        route.topPose.homePathOk = apex.homePathOk;
        route.hasTopPose = true;
    }
    else
    {
        // No plan-proven apex — leave empty top (do not invent home_path_ok=true).
        route.hasTopPose = false;
    }

    for (const Candidate &cand : ordered)
    {
        const Ur3ePlannedScanPoint &pt =
            plan.points[static_cast<std::size_t>(cand.index)];
        Ur3eSemiFixedRing ring;
        ring.id = QStringLiteral("theta_%1").arg(cand.thetaDeg, 0, 'f', 1);
        ring.displayName = QStringLiteral("θ=%1° (plan)")
                               .arg(cand.thetaDeg, 0, 'f', 1);
        ring.entryJointsRad = pt.jointPositionsRad;
        ring.entryTcp = pt.tcp;
        ring.hasEntryTcp = true;
        ring.reachabilityKnown = true;
        ring.reachable = pt.reachable;
        ring.homePathOk = pt.homePathOk;
        route.rings.push_back(ring);
    }

    return route;
}

QVector<Ur3eSemiFixedPreviewRing>
previewSemiFixedRingsFromScanParams(const Ur3eHemisphereScanParams &paramsIn)
{
    Ur3eHemisphereScanParams params = paramsIn;
    normalizeHemisphereScanParams(params);
    const std::vector<Ur3eHemisphereScanPoint> points = generateHemisphereScanPoints(params);

    QVector<Ur3eSemiFixedPreviewRing> out;
    QHash<int, bool> seenTheta;
    double cx = 0.0;
    double cy = 0.0;
    scanCenterOffsetM(cx, cy);

    for (const Ur3eHemisphereScanPoint &pt : points)
    {
        if (std::abs(pt.thetaDeg) < 0.75)
        {
            Ur3eSemiFixedPreviewRing top;
            top.displayName = QStringLiteral("Top (θ=0)");
            top.isTopPose = true;
            top.centerXM = pt.xM;
            top.centerYM = pt.yM;
            top.centerZM = kSemiFixedApexRadiusM;
            top.radiusM = 0.0;
            top.reachabilityKnown = false;
            out.push_back(top);
            continue;
        }
        const int key = static_cast<int>(std::lround(pt.thetaDeg * 2.0));
        if (seenTheta.contains(key))
            continue;
        seenTheta.insert(key, true);

        Ur3eSemiFixedPreviewRing ring;
        ring.displayName = QStringLiteral("θ=%1°").arg(pt.thetaDeg, 0, 'f', 1);
        ring.centerXM = cx;
        ring.centerYM = cy;
        ring.centerZM = pt.zM;
        ring.radiusM = std::hypot(pt.xM - cx, pt.yM - cy);
        if (ring.radiusM < 0.02)
            ring.radiusM = std::max(0.05, params.sphereRadiusM * std::sin(pt.thetaDeg * 3.14159265358979323846 / 180.0));
        ring.reachabilityKnown = false;
        out.push_back(ring);
    }

    // Rings first, top last (matches inferSemiFixedPreviewRings / execute indices).
    QVector<Ur3eSemiFixedPreviewRing> ordered;
    ordered.reserve(out.size());
    Ur3eSemiFixedPreviewRing topRing{};
    bool hasTop = false;
    for (const Ur3eSemiFixedPreviewRing &r : out)
    {
        if (r.isTopPose)
        {
            topRing = r;
            hasTop = true;
        }
        else
            ordered.push_back(r);
    }
    if (hasTop)
        ordered.push_back(topRing);
    return ordered;
}

QVector<Ur3eSemiFixedPreviewRing>
previewSemiFixedRingsFromHemispherePlan(const Ur3eHemisphereScanPlan &plan)
{
    struct LatitudeAgg
    {
        double thetaDeg = 0.0;
        double centerXM = 0.0;
        double centerYM = 0.0;
        double centerZM = 0.0;
        double radiusM = 0.05;
        bool hasGeom = false;
        bool anyReachable = false;
        bool anySweepOk = false;
        bool anyHomePathOk = false;
        bool isTop = false;
    };

    double cx = 0.0;
    double cy = 0.0;
    scanCenterOffsetM(cx, cy);

    QHash<int, LatitudeAgg> byTheta;
    LatitudeAgg apex;
    bool hasApex = false;

    const auto applyGeom = [&](LatitudeAgg &agg, const Ur3ePlannedScanPoint &pt,
                               const bool prefer) {
        const double x = pt.tcp.xM;
        const double y = pt.tcp.yM;
        const double z = pt.tcp.zM;
        const bool tcpOk = std::isfinite(x) && std::isfinite(y) && std::isfinite(z)
                           && (std::abs(x) + std::abs(y) + std::abs(z) > 1.0e-6);
        const double gx = tcpOk ? x : pt.gridPoint.xM;
        const double gy = tcpOk ? y : pt.gridPoint.yM;
        const double gz = tcpOk ? z : pt.gridPoint.zM;
        if (!agg.hasGeom || prefer)
        {
            if (agg.isTop)
            {
                agg.centerXM = gx;
                agg.centerYM = gy;
                agg.centerZM = gz;
                agg.radiusM = 0.0;
            }
            else
            {
                agg.centerXM = cx;
                agg.centerYM = cy;
                agg.centerZM = gz;
                agg.radiusM = std::hypot(gx - cx, gy - cy);
                if (agg.radiusM < 0.02)
                {
                    const double th = pt.gridPoint.thetaDeg * 3.14159265358979323846 / 180.0;
                    agg.radiusM = std::max(0.05, std::hypot(gx - cx, gy - cy));
                    if (agg.radiusM < 0.02)
                        agg.radiusM = std::max(0.05, std::abs(std::sin(th)) * 0.2);
                }
            }
            agg.hasGeom = true;
        }
    };

    for (const Ur3ePlannedScanPoint &pt : plan.points)
    {
        if (std::abs(pt.gridPoint.thetaDeg) < 0.75)
        {
            hasApex = true;
            apex.isTop = true;
            apex.thetaDeg = 0.0;
            const bool prefer = pt.reachable && (!apex.anyReachable || pt.homePathOk);
            applyGeom(apex, pt, prefer || !apex.hasGeom);
            if (pt.reachable)
            {
                apex.anyReachable = true;
                apex.anySweepOk = true; // top has no base sweep requirement
                if (pt.homePathOk)
                    apex.anyHomePathOk = true;
            }
            continue;
        }

        const int key = static_cast<int>(std::lround(pt.gridPoint.thetaDeg * 2.0));
        LatitudeAgg &agg = byTheta[key];
        agg.thetaDeg = pt.gridPoint.thetaDeg;
        const bool prefer = pt.reachable && pt.baseSweepOk
                            && (!agg.anySweepOk || (pt.homePathOk && !agg.anyHomePathOk));
        applyGeom(agg, pt, prefer || !agg.hasGeom);
        if (pt.reachable)
        {
            agg.anyReachable = true;
            if (pt.baseSweepOk)
            {
                agg.anySweepOk = true;
                if (pt.homePathOk)
                    agg.anyHomePathOk = true;
            }
        }
    }

    QVector<LatitudeAgg> ordered;
    ordered.reserve(byTheta.size());
    for (auto it = byTheta.cbegin(); it != byTheta.cend(); ++it)
        ordered.push_back(it.value());
    std::sort(ordered.begin(), ordered.end(),
              [](const LatitudeAgg &a, const LatitudeAgg &b) { return a.thetaDeg < b.thetaDeg; });

    QVector<Ur3eSemiFixedPreviewRing> out;
    out.reserve(ordered.size() + (hasApex ? 1 : 0));
    for (const LatitudeAgg &agg : ordered)
    {
        Ur3eSemiFixedPreviewRing ring;
        ring.displayName = QStringLiteral("θ=%1°").arg(agg.thetaDeg, 0, 'f', 1);
        ring.centerXM = agg.centerXM;
        ring.centerYM = agg.centerYM;
        ring.centerZM = agg.centerZM;
        ring.radiusM = agg.radiusM > 0.02 ? agg.radiusM : 0.05;
        ring.reachabilityKnown = true;
        // Semi-executable ring = at least one base-sweep OK pin; else blue unreachable.
        ring.reachable = agg.anySweepOk;
        ring.homePathOk = agg.anySweepOk && agg.anyHomePathOk;
        out.push_back(ring);
    }
    if (hasApex)
    {
        Ur3eSemiFixedPreviewRing top;
        top.displayName = QStringLiteral("Top (θ=0)");
        top.isTopPose = true;
        top.centerXM = apex.centerXM;
        top.centerYM = apex.centerYM;
        top.centerZM = apex.centerZM;
        top.radiusM = 0.0;
        top.reachabilityKnown = true;
        top.reachable = apex.anyReachable;
        top.homePathOk = apex.anyHomePathOk;
        out.push_back(top);
    }
    else if (!out.isEmpty())
    {
        // Ring-only plans (one θ per file) carry no θ=0 point, but execute always
        // runs the built-in apex still first — preview it so both agree.
        const Ur3eSemiFixedRing fallback = defaultSemiFixedTopPose();
        Ur3eSemiFixedPreviewRing top = inferSemiFixedPreviewRing(fallback);
        top.displayName = QStringLiteral("Top (θ=0)");
        top.isTopPose = true;
        if (fallback.hasEntryTcp)
        {
            top.centerXM = fallback.entryTcp.xM;
            top.centerYM = fallback.entryTcp.yM;
            top.centerZM = fallback.entryTcp.zM;
        }
        top.radiusM = 0.0;
        out.push_back(top);
    }
    return out;
}

void syncHemisphereParamsFromPlanLatitudes(const Ur3eHemisphereScanPlan &plan,
                                           Ur3eHemisphereScanParams &paramsInOut)
{
    double tMin = 1.0e100;
    double tMax = -1.0e100;
    QHash<int, bool> latitudeKeys;
    for (const Ur3ePlannedScanPoint &pt : plan.points)
    {
        const double th = pt.gridPoint.thetaDeg;
        if (std::abs(th) < 0.75)
            continue;
        tMin = std::min(tMin, th);
        tMax = std::max(tMax, th);
        latitudeKeys.insert(static_cast<int>(std::lround(th * 2.0)), true);
    }
    if (latitudeKeys.isEmpty())
        return;
    paramsInOut.thetaMinDeg = tMin;
    paramsInOut.thetaMaxDeg = tMax;
    paramsInOut.verticalPoints = std::max(1, static_cast<int>(latitudeKeys.size()));
    normalizeHemisphereScanParams(paramsInOut);
}

int semiFixedSampleCount(const double intervalDeg)
{
    if (!(intervalDeg > 0.0))
        return 1;
    const int n = static_cast<int>(std::lround(360.0 / intervalDeg));
    return std::max(1, n);
}

} // namespace hf::ur3e
