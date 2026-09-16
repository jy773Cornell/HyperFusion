// Semi-fixed ring route persistence + preview ring inference (backend).

#include "backend/multiview/Ur3eSemiFixedScan.hpp"

#include "backend/HyperFusionConfig.hpp"
#include "backend/multiview/Ur3eHemisphereScan.hpp"
#include "backend/multiview/Ur3eScanPlanCache.hpp"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QHash>
#include <QVector>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include <algorithm>
#include <cmath>
#include <cstdint>

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

    QJsonParseError errA;
    QJsonParseError errB;
    const QJsonDocument docA = QJsonDocument::fromJson(a.toUtf8(), &errA);
    const QJsonDocument docB = QJsonDocument::fromJson(b.toUtf8(), &errB);
    if (errA.error != QJsonParseError::NoError || errB.error != QJsonParseError::NoError
        || !docA.isObject() || !docB.isObject())
        return false;

    QJsonObject oa = docA.object();
    QJsonObject ob = docB.object();
    oa.remove(QStringLiteral("use_mock_hardware"));
    ob.remove(QStringLiteral("use_mock_hardware"));
    if (oa.size() != ob.size())
        return false;

    constexpr double kEps = 1.0e-6;
    const QStringList keys = oa.keys();
    for (const QString &key : keys)
    {
        if (!ob.contains(key))
            return false;
        const QJsonValue va = oa.value(key);
        const QJsonValue vb = ob.value(key);
        if (va.isDouble() && vb.isDouble())
        {
            if (std::abs(va.toDouble() - vb.toDouble()) > kEps)
                return false;
            continue;
        }
        if (va.isArray() && vb.isArray())
        {
            const QJsonArray aa = va.toArray();
            const QJsonArray ab = vb.toArray();
            if (aa.size() != ab.size())
                return false;
            for (int i = 0; i < aa.size(); ++i)
            {
                if (aa.at(i).isDouble() && ab.at(i).isDouble())
                {
                    if (std::abs(aa.at(i).toDouble() - ab.at(i).toDouble()) > kEps)
                        return false;
                }
                else if (aa.at(i) != ab.at(i))
                {
                    return false;
                }
            }
            continue;
        }
        if (va != vb)
            return false;
    }
    return true;
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

void snapApexPreviewToRingSphere(QVector<Ur3eSemiFixedPreviewRing> &rings,
                                 const double fallbackRadiusM)
{
    double cx = 0.0;
    double cy = 0.0;
    scanCenterOffsetM(cx, cy);

    double sphereR = 0.0;
    for (const Ur3eSemiFixedPreviewRing &p : rings)
    {
        if (p.isTopPose)
            continue;
        sphereR = std::max(sphereR, std::hypot(p.radiusM, p.centerZM));
    }
    if (sphereR < 0.01)
        sphereR = fallbackRadiusM;
    if (sphereR < 0.01)
        return;

    for (Ur3eSemiFixedPreviewRing &p : rings)
    {
        if (!p.isTopPose)
            continue;
        p.centerXM = cx;
        p.centerYM = cy;
        p.centerZM = sphereR;
        p.radiusM = 0.0;
    }
}

} // namespace

Ur3eSemiFixedRing apexTopPoseOnRingSphere(const double sphereRadiusM)
{
    Ur3eSemiFixedRing top;
    top.id = QStringLiteral("top");
    top.displayName = QStringLiteral("Top (θ=0)");
    top.noPan = true;
    top.thetaDeg = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    homeTcpScanCenterOffsetM(cx, cy);
    Ur3eHemisphereScanPoint grid;
    grid.phiDeg = 0.0;
    grid.thetaDeg = 0.0;
    grid.xM = cx;
    grid.yM = cy;
    grid.zM = std::max(0.01, sphereRadiusM);
    top.entryTcp = tcpPoseForHemispherePoint(grid);
    top.hasEntryTcp = true;
    top.reachabilityKnown = false;
    top.reachable = false;
    top.homePathOk = false;
    return top;
}

Ur3eSemiFixedRing defaultSemiFixedTopPose()
{
    return apexTopPoseOnRingSphere(kSemiFixedApexRadiusM);
}

double inferSemiFixedSphereRadiusM(const Ur3eSemiFixedRoute &route)
{
    double cx = 0.0;
    double cy = 0.0;
    scanCenterOffsetM(cx, cy);
    double r = 0.0;
    for (const Ur3eSemiFixedRing &ring : route.rings)
    {
        if (ring.noPan || std::abs(ring.thetaDeg) < 0.75)
            continue;
        if (!ring.hasEntryTcp)
            continue;
        const double horiz = std::hypot(ring.entryTcp.xM - cx, ring.entryTcp.yM - cy);
        r = std::max(r, std::hypot(horiz, ring.entryTcp.zM));
    }
    return r;
}

void ensureSemiFixedTopPose(Ur3eSemiFixedRoute &route)
{
    const double ringR = inferSemiFixedSphereRadiusM(route);
    if (route.hasTopPose && route.topPose.entryJointsRad.size() == 6
        && route.topPose.hasEntryTcp)
    {
        if (ringR < 0.01 || std::abs(route.topPose.entryTcp.zM - ringR) <= 0.02)
            return;
    }
    if (ringR < 0.01)
        return;
    std::vector<double> keepJoints = route.topPose.entryJointsRad;
    route.topPose = apexTopPoseOnRingSphere(ringR);
    if (keepJoints.size() == 6)
        route.topPose.entryJointsRad = std::move(keepJoints);
    // Apex is home pose with Z=R; joints are filled at execute when missing.
    // Keep preview green (not gray/blue) for deferred IK.
    route.topPose.reachabilityKnown = true;
    route.topPose.reachable = true;
    route.topPose.homePathOk = true;
    route.hasTopPose = true;
}

QString defaultUr3eSemiScanRoutesDir()
{
    return defaultUr3eSemiScanPlansDir();
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
    root.insert(QStringLiteral("kind"),
                toSave.isFppPlan ? QStringLiteral("fpp")
                                 : QStringLiteral("ur3e_semi_fixed_route"));
    root.insert(QStringLiteral("id"), toSave.id);
    root.insert(QStringLiteral("display_name"), toSave.displayName);
    root.insert(QStringLiteral("robot_cfg_fingerprint"), toSave.robotCfgFingerprint);
    root.insert(QStringLiteral("interval_deg"), toSave.intervalDeg);
    root.insert(QStringLiteral("pan_range_deg"), toSave.panRangeDeg);
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
        top.insert(QStringLiteral("base_sweep_ok"), toSave.topPose.baseSweepOk);
        top.insert(QStringLiteral("backup_coverage_ok"), toSave.topPose.backupCoverageOk);
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
        o.insert(QStringLiteral("base_sweep_ok"), ring.baseSweepOk);
        o.insert(QStringLiteral("backup_coverage_ok"), ring.backupCoverageOk);
        o.insert(QStringLiteral("backup_union_deg"), ring.backupUnionDeg);
        o.insert(QStringLiteral("phi_deg"), ring.phiDeg);
        o.insert(QStringLiteral("theta_deg"), ring.thetaDeg);
        o.insert(QStringLiteral("no_pan"), ring.noPan);
        if (!ring.panMask.empty())
        {
            QJsonArray mask;
            for (const std::uint8_t bit : ring.panMask)
                mask.append(static_cast<int>(bit != 0 ? 1 : 0));
            o.insert(QStringLiteral("pan_mask"), mask);
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
    const QString kind = root.value(QStringLiteral("kind")).toString();
    if (!kind.isEmpty() && kind != QStringLiteral("fpp")
        && kind != QStringLiteral("ur3e_semi_fixed_route"))
    {
        if (errorMessage != nullptr)
            *errorMessage =
                QStringLiteral("Unsupported route kind \"%1\" in %2").arg(kind, path);
        return false;
    }

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
    routeOut.isFppPlan = (kind == QStringLiteral("fpp"));
    routeOut.id = root.value(QStringLiteral("id")).toString(QFileInfo(path).completeBaseName());
    routeOut.displayName =
        root.value(QStringLiteral("display_name")).toString(routeOut.id);
    routeOut.robotCfgFingerprint = fp;
    routeOut.intervalDeg = root.value(QStringLiteral("interval_deg")).toDouble(10.0);
    routeOut.panRangeDeg = root.value(QStringLiteral("pan_range_deg")).toDouble(360.0);
    if (!(routeOut.panRangeDeg >= 0.0))
        routeOut.panRangeDeg = 0.0;
    if (routeOut.panRangeDeg > 360.0)
        routeOut.panRangeDeg = 360.0;
    routeOut.panDirection = root.value(QStringLiteral("pan_direction")).toInt(1);
    if (routeOut.panDirection >= 0)
        routeOut.panDirection = 1;
    else
        routeOut.panDirection = -1;
    routeOut.stabilizeMs = root.value(QStringLiteral("stabilize_ms")).toInt(500);

    // Stage stops are GUI-owned (Multiview Stage 1 / 2). Ignore legacy JSON stage blocks.
    routeOut.haveStagePositions = false;
    routeOut.stageHomeMm = 0.0;
    routeOut.stageDlpMm = 0.0;

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
        // Legacy routes had no flags — they were full-spin entries.
        ring.baseSweepOk = o.value(QStringLiteral("base_sweep_ok")).toBool(true);
        ring.backupCoverageOk = o.value(QStringLiteral("backup_coverage_ok")).toBool(false);
        ring.backupUnionDeg = o.value(QStringLiteral("backup_union_deg")).toDouble(0.0);
        ring.phiDeg = o.value(QStringLiteral("phi_deg")).toDouble(0.0);
        ring.thetaDeg = o.value(QStringLiteral("theta_deg")).toDouble(0.0);
        ring.noPan = o.value(QStringLiteral("no_pan")).toBool(false)
                     || std::abs(ring.thetaDeg) < 0.75;
        ring.panMask.clear();
        if (o.contains(QStringLiteral("pan_mask")) && o.value(QStringLiteral("pan_mask")).isArray())
        {
            const QJsonArray mask = o.value(QStringLiteral("pan_mask")).toArray();
            ring.panMask.reserve(mask.size());
            for (const QJsonValue &bit : mask)
                ring.panMask.push_back(bit.toBool(false) || bit.toInt(0) != 0 ? 1 : 0);
        }
        if (ring.id.isEmpty())
            ring.id = QStringLiteral("ring_%1").arg(routeOut.rings.size() + 1);
        if (ring.displayName.isEmpty())
            ring.displayName = ring.id;
        routeOut.rings.push_back(ring);
    }

    if (routeOut.rings.isEmpty() && !routeOut.hasTopPose)
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Route has no valid rings or apex");
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
    preview.noPan = ring.noPan;
    if (ring.noPan || std::abs(ring.thetaDeg) < 0.75)
        preview.isTopPose = true;
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
        if (preview.isTopPose)
            preview.radiusM = 0.0;
        else if (preview.radiusM < 0.02)
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
    bool haveApexRing = false;
    for (const Ur3eSemiFixedPreviewRing &p : out)
    {
        if (p.isTopPose || p.noPan)
            haveApexRing = true;
    }
    if (route.hasTopPose && !haveApexRing)
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
    snapApexPreviewToRingSphere(out, 0.0);
    return out;
}

namespace
{
void fillPinTipDir(Ur3eSemiFixedPreviewRing &pin, const Ur3eSemiFixedRing &ring)
{
    if (ring.hasEntryTcp)
    {
        const double lx = ring.entryTcp.toolZMx;
        const double ly = ring.entryTcp.toolZMy;
        const double lz = ring.entryTcp.toolZMz;
        const double len = std::sqrt(lx * lx + ly * ly + lz * lz);
        if (len > 1.0e-9)
        {
            pin.tipDirX = lx / len;
            pin.tipDirY = ly / len;
            pin.tipDirZ = lz / len;
            return;
        }
    }
    double cx = 0.0;
    double cy = 0.0;
    scanCenterOffsetM(cx, cy);
    const double dx = cx - pin.centerXM;
    const double dy = cy - pin.centerYM;
    const double dz = 0.0 - pin.centerZM;
    const double len = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (len > 1.0e-9)
    {
        pin.tipDirX = dx / len;
        pin.tipDirY = dy / len;
        pin.tipDirZ = dz / len;
    }
    else
    {
        pin.tipDirX = 0.0;
        pin.tipDirY = 0.0;
        pin.tipDirZ = 1.0;
    }
}

Ur3eSemiFixedPreviewRing makeFppPin(const Ur3eSemiFixedRing &ring,
                                    const double xM,
                                    const double yM,
                                    const double zM,
                                    const int executeIndex,
                                    const QString &label)
{
    Ur3eSemiFixedPreviewRing pin;
    pin.displayName = label;
    pin.centerXM = xM;
    pin.centerYM = yM;
    pin.centerZM = zM;
    pin.radiusM = 0.0;
    pin.isTopPose = false;
    pin.drawAsPin = true;
    pin.noPan = ring.noPan;
    pin.reachabilityKnown = ring.reachabilityKnown;
    pin.reachable = ring.reachable;
    pin.homePathOk = ring.homePathOk;
    pin.executeIndex = executeIndex;
    fillPinTipDir(pin, ring);
    return pin;
}
} // namespace

QVector<Ur3eSemiFixedPreviewRing> inferFppPreviewPins(const Ur3eSemiFixedRoute &route)
{
    QVector<Ur3eSemiFixedPreviewRing> out;
    double cx = 0.0;
    double cy = 0.0;
    scanCenterOffsetM(cx, cy);

    const double intervalDeg = route.intervalDeg > 0.0 ? route.intervalDeg : 10.0;
    const double rangeDeg =
        route.panRangeDeg >= 0.0 ? std::min(360.0, route.panRangeDeg) : 360.0;
    const int panDir = route.panDirection >= 0 ? 1 : -1;
    const int ringCount = route.rings.size();

    // Rings first so execute ringIndex 0..N-1 still maps via executeIndex.
    for (int ri = 0; ri < ringCount; ++ri)
    {
        const Ur3eSemiFixedRing &ring = route.rings[ri];
        if (!ring.hasEntryTcp)
            continue;

        const bool pinOnly = ring.noPan || std::abs(ring.thetaDeg) < 0.75;
        if (pinOnly)
        {
            out.push_back(makeFppPin(ring,
                                     ring.entryTcp.xM,
                                     ring.entryTcp.yM,
                                     ring.entryTcp.zM,
                                     ri,
                                     ring.displayName.isEmpty()
                                         ? QStringLiteral("pin %1").arg(ri + 1)
                                         : ring.displayName));
            continue;
        }

        const int samples = semiFixedSampleCount(intervalDeg, rangeDeg);
        const double dx0 = ring.entryTcp.xM - cx;
        const double dy0 = ring.entryTcp.yM - cy;
        for (int s = 0; s < samples; ++s)
        {
            const double ang =
                static_cast<double>(panDir) * static_cast<double>(s) * intervalDeg
                * (3.14159265358979323846 / 180.0);
            const double c = std::cos(ang);
            const double sn = std::sin(ang);
            const double x = cx + dx0 * c - dy0 * sn;
            const double y = cy + dx0 * sn + dy0 * c;
            const QString label =
                samples <= 1
                    ? ring.displayName
                    : QStringLiteral("%1 @%2°")
                          .arg(ring.displayName)
                          .arg(static_cast<double>(panDir) * static_cast<double>(s)
                                   * intervalDeg,
                               0, 'f', 0);
            out.push_back(makeFppPin(ring, x, y, ring.entryTcp.zM, ri, label));
        }
    }

    if (route.hasTopPose && route.topPose.entryJointsRad.size() == 6)
    {
        Ur3eSemiFixedRing top = route.topPose;
        const double x = top.hasEntryTcp ? top.entryTcp.xM : cx;
        const double y = top.hasEntryTcp ? top.entryTcp.yM : cy;
        const double z = top.hasEntryTcp ? top.entryTcp.zM : 0.25;
        Ur3eSemiFixedPreviewRing home =
            makeFppPin(top,
                       x,
                       y,
                       z,
                       ringCount,
                       top.displayName.isEmpty() ? QStringLiteral("Home") : top.displayName);
        home.isTopPose = true;
        out.push_back(home);
    }
    return out;
}

Ur3eSemiFixedRoute semiFixedRouteFromHemispherePlan(const Ur3eHemisphereScanPlan &plan,
                                                    const QString &robotCfgFingerprint,
                                                    const QString &displayName,
                                                    const double intervalDeg,
                                                    const int panDirection)
{
    // One full-spin pin per latitude. Two pins only when both are backup-only.
    struct Candidate
    {
        int index = -1;
        double thetaDeg = 0.0;
        double phiDeg = 0.0;
        bool baseSweepOk = false;
        bool backupCoverageOk = false;
        bool homePathOk = false;
    };

    const std::vector<double> homeJoints = ur3eScanHomeJointsRadFromConfig();

    QHash<int, QVector<Candidate>> pinsByThetaKey;
    int apexIndex = -1;
    auto pinAccepted = [](const Ur3ePlannedScanPoint &pt) {
        return pt.homePathOk && (pt.baseSweepOk || pt.backupCoverageOk);
    };

    for (int i = 0; i < static_cast<int>(plan.points.size()); ++i)
    {
        const Ur3ePlannedScanPoint &pt = plan.points[static_cast<std::size_t>(i)];
        const bool isApex = std::abs(pt.gridPoint.thetaDeg) < 0.75;
        if (!isApex && (!pt.reachable || pt.jointPositionsRad.size() != 6))
            continue;

        if (isApex)
        {
            const bool haveJoints = pt.jointPositionsRad.size() == 6;
            if (haveJoints && !pt.homePathOk)
                continue;
            if (!haveJoints)
            {
                if (apexIndex < 0)
                    apexIndex = i;
                continue;
            }
            if (apexIndex < 0
                || plan.points[static_cast<std::size_t>(apexIndex)].jointPositionsRad.size() != 6
                || ur3eJointDistanceRad(homeJoints, pt.jointPositionsRad)
                       < ur3eJointDistanceRad(
                             homeJoints,
                             plan.points[static_cast<std::size_t>(apexIndex)].jointPositionsRad)
                           - 1e-9)
                apexIndex = i;
            continue;
        }

        if (!pinAccepted(pt))
            continue;

        const int thetaKey = static_cast<int>(std::lround(pt.gridPoint.thetaDeg * 2.0));
        Candidate cand;
        cand.index = i;
        cand.thetaDeg = pt.gridPoint.thetaDeg;
        cand.phiDeg = pt.gridPoint.phiDeg;
        cand.baseSweepOk = pt.baseSweepOk;
        cand.backupCoverageOk = pt.backupCoverageOk;
        cand.homePathOk = pt.homePathOk;
        QVector<Candidate> &row = pinsByThetaKey[thetaKey];
        const bool rowHasSweep = std::any_of(
            row.cbegin(), row.cend(), [](const Candidate &c) { return c.baseSweepOk; });
        if (rowHasSweep)
            continue;
        if (cand.baseSweepOk)
        {
            row.clear();
            row.push_back(cand);
            continue;
        }
        if (row.size() >= 2)
            continue;
        row.push_back(cand);
    }

    // Fallback: if plan has no sweep/backup markers (legacy), one nearest-home pin.
    if (pinsByThetaKey.isEmpty())
    {
        QHash<int, Candidate> bestByThetaKey;
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
            cand.phiDeg = pt.gridPoint.phiDeg;
            cand.homePathOk = pt.homePathOk;
            cand.baseSweepOk = true;
            const auto it = bestByThetaKey.constFind(thetaKey);
            const double homeDist = ur3eJointDistanceRad(homeJoints, pt.jointPositionsRad);
            if (it == bestByThetaKey.cend()
                || homeDist < ur3eJointDistanceRad(
                                  homeJoints,
                                  plan.points[static_cast<std::size_t>(it.value().index)]
                                      .jointPositionsRad))
            {
                bestByThetaKey.insert(thetaKey, cand);
            }
        }
        for (auto it = bestByThetaKey.cbegin(); it != bestByThetaKey.cend(); ++it)
            pinsByThetaKey[it.key()].push_back(it.value());
    }

    QVector<int> thetaKeys;
    thetaKeys.reserve(pinsByThetaKey.size());
    for (auto it = pinsByThetaKey.cbegin(); it != pinsByThetaKey.cend(); ++it)
        thetaKeys.push_back(it.key());
    std::sort(thetaKeys.begin(), thetaKeys.end());

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
        // Saved plans often leave apex joints empty (planned at execute as home@Z=R).
        const bool deferredApex = apex.jointPositionsRad.size() != 6;
        route.topPose.reachabilityKnown = true;
        route.topPose.reachable = apex.reachable || deferredApex;
        route.topPose.homePathOk = apex.homePathOk || deferredApex;
        route.topPose.baseSweepOk = true;
        route.topPose.thetaDeg = 0.0;
        route.topPose.noPan = true;
        route.hasTopPose = true;
    }
    else
    {
        route.hasTopPose = false;
    }

    for (const int thetaKey : thetaKeys)
    {
        const QVector<Candidate> &row = pinsByThetaKey.value(thetaKey);
        const int nPins = row.size();
        for (int pin = 0; pin < nPins; ++pin)
        {
            const Candidate &cand = row[pin];
            const Ur3ePlannedScanPoint &pt =
                plan.points[static_cast<std::size_t>(cand.index)];
            Ur3eSemiFixedRing ring;
            ring.thetaDeg = cand.thetaDeg;
            ring.phiDeg = cand.phiDeg;
            ring.id = nPins >= 2
                          ? QStringLiteral("theta_%1_pin%2")
                                .arg(cand.thetaDeg, 0, 'f', 1)
                                .arg(pin + 1)
                          : QStringLiteral("theta_%1").arg(cand.thetaDeg, 0, 'f', 1);
            if (nPins >= 2)
            {
                ring.displayName = QStringLiteral("θ=%1° pin%2 (φ=%3°)")
                                       .arg(cand.thetaDeg, 0, 'f', 1)
                                       .arg(pin + 1)
                                       .arg(cand.phiDeg, 0, 'f', 1);
                if (cand.backupCoverageOk && !cand.baseSweepOk)
                    ring.displayName += QStringLiteral(" backup");
            }
            else
            {
                ring.displayName = QStringLiteral("θ=%1° (plan)")
                                       .arg(cand.thetaDeg, 0, 'f', 1);
            }
            ring.entryJointsRad = pt.jointPositionsRad;
            ring.entryTcp = pt.tcp;
            ring.hasEntryTcp = true;
            ring.reachabilityKnown = true;
            ring.reachable = pt.reachable;
            ring.homePathOk = pt.homePathOk;
            ring.baseSweepOk = pt.baseSweepOk || !pt.backupCoverageOk;
            ring.backupCoverageOk = pt.backupCoverageOk;
            ring.backupUnionDeg = pt.backupUnionDeg;
            ring.panMask = pt.panMask;
            ring.noPan = false;
            route.rings.push_back(ring);
        }
    }

    // Apex-only plans: one pin-only hop so Execute has a route entry (no 360° spin).
    if (route.hasTopPose && route.rings.isEmpty())
    {
        Ur3eSemiFixedRing apex = route.topPose;
        apex.id = QStringLiteral("apex");
        if (apex.displayName.isEmpty() || apex.displayName == QStringLiteral("Top (θ=0)"))
            apex.displayName = QStringLiteral("Apex (θ=0)");
        apex.noPan = true;
        apex.baseSweepOk = false;
        apex.thetaDeg = 0.0;
        route.rings.push_back(apex);
    }

    pruneSemiFixedRedundantFullSpinPins(route);
    return route;
}

void pruneSemiFixedRedundantFullSpinPins(Ur3eSemiFixedRoute &route)
{
    const auto sameTheta = [](const Ur3eSemiFixedRing &a, const Ur3eSemiFixedRing &b) {
        if (std::abs(a.thetaDeg) < 0.75 || std::abs(b.thetaDeg) < 0.75)
            return false;
        return std::lround(a.thetaDeg * 2.0) == std::lround(b.thetaDeg * 2.0);
    };

    QVector<Ur3eSemiFixedRing> kept;
    kept.reserve(route.rings.size());
    for (const Ur3eSemiFixedRing &ring : route.rings)
    {
        bool drop = false;
        for (int i = 0; i < kept.size(); ++i)
        {
            Ur3eSemiFixedRing &prev = kept[i];
            if (!sameTheta(prev, ring))
                continue;
            if (prev.baseSweepOk)
            {
                drop = true;
                break;
            }
            if (ring.baseSweepOk)
            {
                prev = ring;
                drop = true;
                break;
            }
        }
        if (!drop)
            kept.push_back(ring);
    }
    route.rings = std::move(kept);
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
            top.centerZM = pt.zM;
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
    snapApexPreviewToRingSphere(ordered, params.sphereRadiusM);
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
            const bool deferredApex = pt.jointPositionsRad.size() != 6;
            const bool prefer =
                (pt.reachable || deferredApex) && (!apex.anyReachable || pt.homePathOk);
            applyGeom(apex, pt, prefer || !apex.hasGeom);
            // Deferred apex (empty joints) is still executable — show green in preview.
            if (pt.reachable || deferredApex)
            {
                apex.anyReachable = true;
                apex.anySweepOk = true; // top has no base sweep requirement
                if (pt.homePathOk || deferredApex)
                    apex.anyHomePathOk = true;
            }
            continue;
        }

        const int key = static_cast<int>(std::lround(pt.gridPoint.thetaDeg * 2.0));
        LatitudeAgg &agg = byTheta[key];
        agg.thetaDeg = pt.gridPoint.thetaDeg;
        const bool sweepOrBackup = pt.baseSweepOk || pt.backupCoverageOk;
        const bool prefer = pt.reachable && sweepOrBackup
                            && (!agg.anySweepOk || (pt.homePathOk && !agg.anyHomePathOk));
        applyGeom(agg, pt, prefer || !agg.hasGeom);
        if (pt.reachable)
        {
            agg.anyReachable = true;
            if (sweepOrBackup)
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
        // Semi-executable ring = sweep-OK or backup-coverage pin; else blue.
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
    snapApexPreviewToRingSphere(out, 0.0);
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

int semiFixedSampleCount(const double intervalDeg, const double rangeDeg)
{
    const double range = std::clamp(rangeDeg, 0.0, 360.0);
    if (!(intervalDeg > 0.0) || !(range > 0.0))
        return 1;
    const int n = static_cast<int>(std::lround(range / intervalDeg));
    return std::max(1, n);
}

namespace
{
int nearestPanMaskBin(const std::vector<std::uint8_t> &mask, const double panRad)
{
    const int n = static_cast<int>(mask.size());
    if (n <= 0)
        return -1;
    const double wrapped = std::atan2(std::sin(panRad), std::cos(panRad));
    int best = 0;
    double bestAbs = 1.0e99;
    for (int i = 0; i < n; ++i)
    {
        const double binPan =
            std::atan2(std::sin(2.0 * 3.14159265358979323846 * static_cast<double>(i)
                                / static_cast<double>(n)),
                       std::cos(2.0 * 3.14159265358979323846 * static_cast<double>(i)
                                / static_cast<double>(n)));
        const double d = std::abs(std::atan2(std::sin(binPan - wrapped),
                                             std::cos(binPan - wrapped)));
        if (d < bestAbs)
        {
            bestAbs = d;
            best = i;
        }
    }
    return best;
}

int contiguousMaskCountFromEntry(const std::vector<std::uint8_t> &mask, const double entryPan)
{
    const int n = static_cast<int>(mask.size());
    if (n <= 0)
        return 0;
    int entryBin = nearestPanMaskBin(mask, entryPan);
    if (entryBin < 0)
        return 0;
    if (mask[static_cast<std::size_t>(entryBin)] == 0)
    {
        int found = -1;
        for (int d = 1; d < n; ++d)
        {
            const int a = (entryBin + d) % n;
            const int b = (entryBin - d + n) % n;
            if (mask[static_cast<std::size_t>(a)] != 0)
            {
                found = a;
                break;
            }
            if (mask[static_cast<std::size_t>(b)] != 0)
            {
                found = b;
                break;
            }
        }
        if (found < 0)
            return 0;
        entryBin = found;
    }
    int start = entryBin;
    for (int k = 0; k < n - 1; ++k)
    {
        const int prev = (start - 1 + n) % n;
        if (mask[static_cast<std::size_t>(prev)] == 0)
            break;
        if (prev == entryBin)
            break;
        start = prev;
    }
    int count = 0;
    int i = start;
    for (int k = 0; k < n; ++k)
    {
        if (mask[static_cast<std::size_t>(i)] == 0)
            break;
        ++count;
        i = (i + 1) % n;
        if (i == start)
            break;
    }
    return count;
}
} // namespace

int semiFixedRingSampleCount(const Ur3eSemiFixedRing &ring,
                             const double intervalDeg,
                             const double rangeDeg)
{
    if (ring.noPan || std::abs(ring.thetaDeg) < 0.75)
        return 1;
    if (ring.backupCoverageOk && !ring.baseSweepOk)
    {
        if (ring.panMask.empty())
            return 1;
        const double entryPan =
            ring.entryJointsRad.size() == 6 ? ring.entryJointsRad[0] : 0.0;
        const int bins = contiguousMaskCountFromEntry(ring.panMask, entryPan);
        const int n = static_cast<int>(ring.panMask.size());
        if (bins <= 0 || n <= 0)
            return 1;
        const double arcDeg = 360.0 * static_cast<double>(bins) / static_cast<double>(n);
        const double step = intervalDeg > 0.0 ? intervalDeg : 10.0;
        const double capped = std::min(arcDeg, std::clamp(rangeDeg, 0.0, 360.0));
        return std::max(1, static_cast<int>(std::lround(capped / step)));
    }
    return semiFixedSampleCount(intervalDeg, rangeDeg);
}

} // namespace hf::ur3e
