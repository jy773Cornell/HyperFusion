// HTTP client for the UR3e WSL sidecar server (via wsl curl).
#include "backend/3dscanning/Ur3eClient.hpp"

#include "backend/HyperFusionConfig.hpp"
#include "backend/camera/processing/GsamWslPathUtil.hpp"
#include "backend/3dscanning/Ur3eHemisphereScanReachability.hpp"
#include "backend/3dscanning/Ur3eWorkspaceBoundary.hpp"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QProcess>
#include <QTemporaryFile>
#include <QUrl>

#include <cmath>

namespace hf::ur3e
{
namespace
{
QString wslDistroArgument()
{
    return hf::hardwareConfig().ur3e.wslDistro.trimmed();
}

int serverPortFromUrl(const QString &serverUrl)
{
    const QUrl url(serverUrl);
    if (url.port() > 0)
        return url.port();
    return hf::hardwareConfig().ur3e.serverPort;
}

QString wslLocalEndpoint(const QString &serverUrl, const QString &path)
{
    return QStringLiteral("http://127.0.0.1:%1%2")
        .arg(serverPortFromUrl(serverUrl))
        .arg(path.startsWith(QLatin1Char('/')) ? path : QStringLiteral("/") + path);
}

QByteArray runWslCurl(const QStringList &curlArgs,
                      const int timeoutMs,
                      QString *errorMessage,
                      const bool allowHttpErrorBody);

QByteArray runWslCurl(const QStringList &curlArgs, const int timeoutMs, QString *errorMessage)
{
    return runWslCurl(curlArgs, timeoutMs, errorMessage, false);
}

QByteArray runWslCurl(const QStringList &curlArgs,
                      const int timeoutMs,
                      QString *errorMessage,
                      const bool allowHttpErrorBody)
{
    QStringList wslArgs;
    const QString distro = wslDistroArgument();
    if (!distro.isEmpty())
        wslArgs << QStringLiteral("-d") << distro;
    wslArgs << QStringLiteral("--") << QStringLiteral("curl") << curlArgs;

    QProcess process;
    process.setProgram(QStringLiteral("wsl.exe"));
    process.setArguments(wslArgs);
    process.start();

    if (!process.waitForStarted(10000))
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Failed to start wsl.exe: %1").arg(process.errorString());
        return {};
    }

    if (!process.waitForFinished(timeoutMs + 5000))
    {
        process.kill();
        process.waitForFinished(2000);
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("UR3e WSL request timed out.");
        return {};
    }

    const QByteArray stdoutPayload = process.readAllStandardOutput();
    const QByteArray stderrPayload = process.readAllStandardError();

    if (process.exitStatus() != QProcess::NormalExit
        || (process.exitCode() != 0 && !(allowHttpErrorBody && !stdoutPayload.trimmed().isEmpty())))
    {
        if (errorMessage != nullptr)
        {
            const QString detail = QString::fromUtf8(stderrPayload.trimmed());
            if (!detail.isEmpty())
                *errorMessage = detail;
            else
                *errorMessage = QStringLiteral("UR3e WSL curl failed (exit %1).").arg(process.exitCode());
        }
        return {};
    }

    return stdoutPayload;
}

QJsonObject getJson(const QString &serverUrl,
                    const QString &path,
                    const int timeoutMs,
                    QString *errorMessage)
{
    const QStringList curlArgs = {
        QStringLiteral("-sfS"),
        QStringLiteral("-m"),
        QString::number(qMax(1, timeoutMs / 1000)),
        wslLocalEndpoint(serverUrl, path),
    };

    const QByteArray payload = runWslCurl(curlArgs, timeoutMs, errorMessage);
    if (payload.isEmpty() && (errorMessage == nullptr || !errorMessage->isEmpty()))
        return {};

    const QJsonDocument doc = QJsonDocument::fromJson(payload);
    if (!doc.isObject())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("UR3e server returned invalid JSON.");
        return {};
    }

    return doc.object();
}

QJsonObject postJson(const QString &serverUrl,
                     const QString &path,
                     const QJsonObject &body,
                     const int timeoutMs,
                     QString *errorMessage)
{
    QTemporaryFile tempFile;
    tempFile.setAutoRemove(true);
    if (!tempFile.open())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Could not create temp file for UR3e request.");
        return {};
    }

    const QByteArray jsonBody = QJsonDocument(body).toJson(QJsonDocument::Compact);
    if (tempFile.write(jsonBody) != jsonBody.size())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Could not write UR3e request body.");
        return {};
    }
    tempFile.close();

    const QString wslBodyPath = hf::processing::windowsPathToWsl(tempFile.fileName());
    const QStringList curlArgs = {
        QStringLiteral("-sS"),
        QStringLiteral("-m"),
        QString::number(qMax(1, timeoutMs / 1000)),
        QStringLiteral("-X"),
        QStringLiteral("POST"),
        QStringLiteral("-H"),
        QStringLiteral("Content-Type: application/json"),
        QStringLiteral("-d"),
        QStringLiteral("@%1").arg(wslBodyPath),
        wslLocalEndpoint(serverUrl, path),
    };

    const QByteArray payload = runWslCurl(curlArgs, timeoutMs, errorMessage, true);
    if (payload.isEmpty() && (errorMessage == nullptr || !errorMessage->isEmpty()))
        return {};

    const QJsonDocument doc = QJsonDocument::fromJson(payload);
    if (!doc.isObject())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("UR3e server returned invalid JSON.");
        return {};
    }

    const QJsonObject response = doc.object();
    if (!response.value(QStringLiteral("ok")).toBool(false))
    {
        const QString serverError = response.value(QStringLiteral("error")).toString();
        if (!serverError.isEmpty() && errorMessage != nullptr)
            *errorMessage = serverError;
    }

    return response;
}

Ur3eTcpPose poseFromJsonArray(const QJsonArray &values)
{
    Ur3eTcpPose pose;
    if (values.size() < 6)
        return pose;

    pose.x = values.at(0).toDouble();
    pose.y = values.at(1).toDouble();
    pose.z = values.at(2).toDouble();
    pose.rx = values.at(3).toDouble();
    pose.ry = values.at(4).toDouble();
    pose.rz = values.at(5).toDouble();
    return pose;
}

QJsonArray poseToJsonArray(const Ur3eTcpPose &pose)
{
    return QJsonArray{
        pose.x,
        pose.y,
        pose.z,
        pose.rx,
        pose.ry,
        pose.rz,
    };
}

std::vector<double> positionsFromJsonArray(const QJsonArray &values)
{
    std::vector<double> positions;
    positions.reserve(static_cast<std::size_t>(values.size()));
    for (const QJsonValue &value : values)
        positions.push_back(value.toDouble());
    return positions;
}

QJsonArray positionsToJsonArray(const std::vector<double> &positions)
{
    QJsonArray array;
    for (const double value : positions)
        array.append(value);
    return array;
}

Ur3eScanWaypointMoveResult parseScanMotionResponse(const QJsonObject &response, const QString &localError)
{
    Ur3eScanWaypointMoveResult result;
    if (response.isEmpty())
    {
        result.errorMessage = localError;
        return result;
    }

    if (response.value(QStringLiteral("stopped")).toBool(false))
    {
        result.stopped = true;
        result.errorMessage = response.value(QStringLiteral("error")).toString(
            QStringLiteral("Motion stopped."));
        return result;
    }

    if (response.value(QStringLiteral("skipped")).toBool(false))
    {
        result.skipped = true;
        result.errorMessage = response.value(QStringLiteral("error")).toString(
            QStringLiteral("no collision-free path"));
        return result;
    }

    if (!response.value(QStringLiteral("ok")).toBool(false))
    {
        result.errorMessage = response.value(QStringLiteral("error")).toString(localError);
        return result;
    }

    result.ok = true;
    result.alreadyAtHome = response.value(QStringLiteral("already_at_home")).toBool(false);
    return result;
}

QJsonObject buildScanMotionRequestBody()
{
    const Ur3eWorkspaceBoundary boundary =
        workspaceBoundaryFromConfig(hf::hardwareConfig().ur3e);
    QJsonObject workspace;
    workspace.insert(QStringLiteral("enabled"), boundary.enabled);
    workspace.insert(QStringLiteral("length_m"), boundary.lengthM());
    workspace.insert(QStringLiteral("width_m"), boundary.widthM());
    workspace.insert(QStringLiteral("height_m"), boundary.heightM());
    workspace.insert(QStringLiteral("mount_height_m"), boundary.mountHeightM());
    workspace.insert(QStringLiteral("ceiling_clearance_m"), boundary.ceilingClearanceM());

    QJsonObject body;
    body.insert(QStringLiteral("workspace"), workspace);
    appendUr3eScanHomeJointsToJson(body);
    body.insert(QStringLiteral("pin_pose_tolerance_deg"),
                hf::hardwareConfig().ur3e.pinPoseToleranceDeg);
    body.insert(QStringLiteral("scan_camera_up_world_z"),
                hf::hardwareConfig().ur3e.scanCameraUpWorldZ);
    return body;
}

void fillHealthStatus(const QJsonObject &response, Ur3eHealthStatus *status)
{
    if (status == nullptr)
        return;

    status->ok = response.value(QStringLiteral("status")).toString() == QStringLiteral("ok");
    status->useMockHardware = response.value(QStringLiteral("use_mock_hardware")).toBool(true);
    status->robotConnected = response.value(QStringLiteral("robot_connected")).toBool(false);
    status->driverReady = response.value(QStringLiteral("driver_ready")).toBool(false);
    status->driverState = response.value(QStringLiteral("driver_state")).toString();
    status->robotIp = response.value(QStringLiteral("robot_ip")).toString();
    status->fault = response.value(QStringLiteral("fault")).toString();
}
} // namespace

bool ur3eServerHealthCheck(const QString &serverUrl,
                           Ur3eHealthStatus *status,
                           QString *errorMessage,
                           const int timeoutMs)
{
    const QJsonObject response =
        getJson(serverUrl, QStringLiteral("/health"), qMax(3000, timeoutMs), errorMessage);
    if (response.isEmpty())
        return false;

    if (response.value(QStringLiteral("status")).toString() != QStringLiteral("ok"))
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("UR3e server health check failed.");
        return false;
    }

    fillHealthStatus(response, status);
    return true;
}

bool ur3eServerShutdown(const QString &serverUrl, QString *errorMessage)
{
    const QJsonObject response =
        postJson(serverUrl, QStringLiteral("/shutdown"), QJsonObject(), 5000, errorMessage);
    return !response.isEmpty() && response.value(QStringLiteral("ok")).toBool(false);
}

void fillUr3eConnectAsyncStatus(const QJsonObject &response, Ur3eConnectAsyncStatus *status)
{
    if (status == nullptr)
        return;

    const hf::HardwareConfig::Ur3eConfig &cfg = hf::hardwareConfig().ur3e;
    status->ok = response.value(QStringLiteral("ok")).toBool(false);
    status->inProgress = response.value(QStringLiteral("in_progress")).toBool(false);
    status->alreadyConnected = response.value(QStringLiteral("already_connected")).toBool(false);
    status->phase = response.value(QStringLiteral("phase")).toString();
    status->message = response.value(QStringLiteral("message")).toString();
    status->errorMessage = response.value(QStringLiteral("error")).toString();
    status->driverState = response.value(QStringLiteral("driver_state")).toString();
    status->robotIp = response.value(QStringLiteral("robot_ip")).toString();
    status->reverseIp = response.value(QStringLiteral("reverse_ip")).toString(cfg.reverseIp);
    status->reverseConnected = response.value(QStringLiteral("reverse_connected")).toBool(false);
    status->scriptPortListening =
        response.value(QStringLiteral("script_port_listening")).toBool(false);
    status->robotConnected = response.value(QStringLiteral("connected")).toBool(false);
    if (response.contains(QStringLiteral("use_mock_hardware")))
        status->useMockHardware = response.value(QStringLiteral("use_mock_hardware")).toBool(cfg.useMockHardware);
    else
        status->useMockHardware = cfg.useMockHardware;
}

bool ur3eConnectStart(const QString &serverUrl,
                      const QString &robotIp,
                      Ur3eConnectAsyncStatus *status,
                      QString *errorMessage)
{
    QJsonObject body;
    if (!robotIp.trimmed().isEmpty())
        body.insert(QStringLiteral("ip"), robotIp.trimmed());

    QString localError;
    const QJsonObject response =
        postJson(serverUrl, QStringLiteral("/connect/start"), body, 15000, &localError);
    if (response.isEmpty())
    {
        if (errorMessage != nullptr)
            *errorMessage = localError;
        return false;
    }

    if (!response.value(QStringLiteral("ok")).toBool(false)
        && !response.value(QStringLiteral("already_connected")).toBool(false)
        && !response.value(QStringLiteral("in_progress")).toBool(false))
    {
        const QString err = response.value(QStringLiteral("error")).toString(localError);
        if (errorMessage != nullptr)
            *errorMessage = err;
        return false;
    }

    fillUr3eConnectAsyncStatus(response, status);
    return true;
}

bool ur3eConnectStatus(const QString &serverUrl,
                       Ur3eConnectAsyncStatus *status,
                       QString *errorMessage)
{
    if (status == nullptr)
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("status output is required.");
        return false;
    }

    QString localError;
    const QJsonObject response =
        getJson(serverUrl, QStringLiteral("/connect/status"), 5000, &localError);
    if (response.isEmpty())
    {
        if (errorMessage != nullptr)
            *errorMessage = localError;
        return false;
    }

    fillUr3eConnectAsyncStatus(response, status);
    return true;
}

bool ur3eConnectCancel(const QString &serverUrl, QString *errorMessage)
{
    QString localError;
    const QJsonObject response =
        postJson(serverUrl, QStringLiteral("/connect/cancel"), QJsonObject(), 8000, &localError);
    if (response.isEmpty())
    {
        if (errorMessage != nullptr)
            *errorMessage = localError;
        return false;
    }

    return response.value(QStringLiteral("ok")).toBool(false);
}

bool ur3eSidecarSupportsAsyncConnect(const QString &serverUrl, QString *errorMessage)
{
    QString localError;
    const QJsonObject response =
        getJson(serverUrl, QStringLiteral("/connect/status"), 3000, &localError);
    if (response.isEmpty())
    {
        if (errorMessage != nullptr)
            *errorMessage = localError;
        return false;
    }

    return response.contains(QStringLiteral("phase"));
}

Ur3eConnectResult ur3eConnectRobot(const QString &serverUrl,
                                     const QString &robotIp,
                                     QString *errorMessage)
{
    Ur3eConnectResult result;
    QJsonObject body;
    if (!robotIp.trimmed().isEmpty())
        body.insert(QStringLiteral("ip"), robotIp.trimmed());

    QString localError;
    const int connectTimeoutMs = qMax(30000, hf::hardwareConfig().ur3e.connectTimeoutMs);
    const QJsonObject response =
        postJson(serverUrl, QStringLiteral("/connect"), body, connectTimeoutMs, &localError);
    if (response.isEmpty())
    {
        result.errorMessage = localError;
        if (errorMessage != nullptr)
            *errorMessage = localError;
        return result;
    }

    if (!response.value(QStringLiteral("ok")).toBool(false))
    {
        result.errorMessage = response.value(QStringLiteral("error")).toString(localError);
        if (errorMessage != nullptr)
            *errorMessage = result.errorMessage;
        return result;
    }

    result.ok = true;
    const hf::HardwareConfig::Ur3eConfig &cfg = hf::hardwareConfig().ur3e;
    if (response.contains(QStringLiteral("use_mock_hardware")))
        result.useMockHardware = response.value(QStringLiteral("use_mock_hardware")).toBool(cfg.useMockHardware);
    else if (response.value(QStringLiteral("mode")).toString() == QStringLiteral("simulation"))
        result.useMockHardware = true;
    else if (response.value(QStringLiteral("mode")).toString() == QStringLiteral("hardware"))
        result.useMockHardware = false;
    else
        result.useMockHardware = cfg.useMockHardware;
    result.driverState = response.value(QStringLiteral("driver_state")).toString();
    return result;
}

Ur3eConnectResult ur3eDisconnectRobot(const QString &serverUrl, QString *errorMessage)
{
    Ur3eConnectResult result;
    QString localError;
    const QJsonObject response =
        postJson(serverUrl, QStringLiteral("/disconnect"), QJsonObject(), 15000, &localError);
    if (response.isEmpty())
    {
        result.errorMessage = localError;
        if (errorMessage != nullptr)
            *errorMessage = localError;
        return result;
    }

    result.ok = response.value(QStringLiteral("ok")).toBool(false);
    if (!result.ok)
    {
        result.errorMessage = response.value(QStringLiteral("error")).toString(localError);
        if (errorMessage != nullptr)
            *errorMessage = result.errorMessage;
    }
    result.driverState = response.value(QStringLiteral("driver_state")).toString();
    return result;
}

Ur3ePoseResult ur3eGetTcpPose(const QString &serverUrl, QString *errorMessage)
{
    Ur3ePoseResult result;
    QString localError;
    const QJsonObject response = getJson(serverUrl, QStringLiteral("/pose"), 5000, &localError);
    if (response.isEmpty())
    {
        result.errorMessage = localError;
        if (errorMessage != nullptr)
            *errorMessage = localError;
        return result;
    }

    if (!response.value(QStringLiteral("ok")).toBool(false))
    {
        result.errorMessage = response.value(QStringLiteral("error")).toString(localError);
        if (errorMessage != nullptr)
            *errorMessage = result.errorMessage;
        return result;
    }

    result.ok = true;
    result.pose = poseFromJsonArray(response.value(QStringLiteral("pose")).toArray());
    result.driverState = response.value(QStringLiteral("driver_state")).toString();
    return result;
}

Ur3eMoveResult ur3eMoveLinear(const QString &serverUrl,
                              const Ur3eTcpPose &pose,
                              const double speedMPerS,
                              const double accelMPerS2,
                              const bool waitUntilDone,
                              QString *errorMessage)
{
    Ur3eMoveResult result;
    QJsonObject body;
    body.insert(QStringLiteral("pose"), poseToJsonArray(pose));
    body.insert(QStringLiteral("speed"), speedMPerS);
    body.insert(QStringLiteral("accel"), accelMPerS2);
    body.insert(QStringLiteral("wait"), waitUntilDone);

    QString localError;
    const QJsonObject response =
        postJson(serverUrl, QStringLiteral("/move_l"), body, 120000, &localError);
    if (response.isEmpty())
    {
        result.errorMessage = localError;
        if (errorMessage != nullptr)
            *errorMessage = localError;
        return result;
    }

    if (!response.value(QStringLiteral("ok")).toBool(false))
    {
        result.errorMessage = response.value(QStringLiteral("error")).toString(localError);
        if (errorMessage != nullptr)
            *errorMessage = result.errorMessage;
        return result;
    }

    result.ok = true;
    result.pose = poseFromJsonArray(response.value(QStringLiteral("pose")).toArray());
    return result;
}

Ur3eJointsState ur3eGetJoints(const QString &serverUrl, QString *errorMessage)
{
    Ur3eJointsState result;
    QString localError;
    const QJsonObject response = getJson(serverUrl, QStringLiteral("/joints"), 5000, &localError);
    if (response.isEmpty())
    {
        result.errorMessage = localError;
        if (errorMessage != nullptr)
            *errorMessage = localError;
        return result;
    }

    if (!response.value(QStringLiteral("ok")).toBool(false))
    {
        result.errorMessage = response.value(QStringLiteral("error")).toString(localError);
        if (errorMessage != nullptr)
            *errorMessage = result.errorMessage;
        return result;
    }

    result.ok = true;
    result.names = response.value(QStringLiteral("names")).toVariant().toStringList();
    result.positionsRad = positionsFromJsonArray(response.value(QStringLiteral("positions")).toArray());
    result.driverState = response.value(QStringLiteral("driver_state")).toString();
    return result;
}

Ur3eJointsMoveResult ur3eMoveJoints(const QString &serverUrl,
                                    const std::vector<double> &positionsRad,
                                    const bool waitUntilDone,
                                    QString *errorMessage)
{
    Ur3eJointsMoveResult result;
    QJsonObject body;
    body.insert(QStringLiteral("positions"), positionsToJsonArray(positionsRad));
    body.insert(QStringLiteral("wait"), waitUntilDone);

    QString localError;
    const QJsonObject response =
        postJson(serverUrl, QStringLiteral("/move_j"), body, 120000, &localError);
    if (response.isEmpty())
    {
        result.errorMessage = localError;
        if (errorMessage != nullptr)
            *errorMessage = localError;
        return result;
    }

    if (!response.value(QStringLiteral("ok")).toBool(false))
    {
        result.errorMessage = response.value(QStringLiteral("error")).toString(localError);
        if (errorMessage != nullptr)
            *errorMessage = result.errorMessage;
        return result;
    }

    result.ok = true;
    result.positionsRad =
        positionsFromJsonArray(response.value(QStringLiteral("positions")).toArray());
    return result;
}

Ur3eScanWaypointMoveResult ur3eExecuteScanWaypoint(const QString &serverUrl,
                                                   const std::vector<double> &positionsRad,
                                                   const Ur3eScanTcpPose *tcpPose,
                                                   QString *errorMessage,
                                                   const bool requireHomeFirst,
                                                   const bool directOnly,
                                                   const bool allowPinPoseCone)
{
    QJsonObject body = buildScanMotionRequestBody();
    body.insert(QStringLiteral("joints"), positionsToJsonArray(positionsRad));
    if (requireHomeFirst)
        body.insert(QStringLiteral("require_home_first"), true);
    if (directOnly)
        body.insert(QStringLiteral("direct_only"), true);
    if (allowPinPoseCone)
        body.insert(QStringLiteral("allow_pin_pose_cone"), true);
    if (tcpPose != nullptr)
    {
        QJsonObject tcp;
        tcp.insert(QStringLiteral("x_m"), tcpPose->xM);
        tcp.insert(QStringLiteral("y_m"), tcpPose->yM);
        tcp.insert(QStringLiteral("z_m"), tcpPose->zM);
        tcp.insert(QStringLiteral("rx"), tcpPose->rxRad);
        tcp.insert(QStringLiteral("ry"), tcpPose->ryRad);
        tcp.insert(QStringLiteral("rz"), tcpPose->rzRad);
        tcp.insert(QStringLiteral("tool_z_x"), tcpPose->toolZMx);
        tcp.insert(QStringLiteral("tool_z_y"), tcpPose->toolZMy);
        tcp.insert(QStringLiteral("tool_z_z"), tcpPose->toolZMz);
        // Apex look-down: keep exact perpendicular (no pin-pose cone) + camera-up +X.
        // Semi-fixed ring entries force cone (require_perpendicular=false).
        const bool apexLookDown = tcpPose->toolZMz < -0.98
                                  && std::abs(tcpPose->toolZMx) < 0.15
                                  && std::abs(tcpPose->toolZMy) < 0.15;
        if (allowPinPoseCone)
        {
            tcp.insert(QStringLiteral("require_perpendicular"), false);
            tcp.insert(QStringLiteral("camera_up_x"), 0.0);
            tcp.insert(QStringLiteral("camera_up_y"), 0.0);
            tcp.insert(QStringLiteral("camera_up_z"), 1.0);
        }
        else if (apexLookDown)
        {
            tcp.insert(QStringLiteral("require_perpendicular"), true);
            tcp.insert(QStringLiteral("camera_up_x"), 1.0);
            tcp.insert(QStringLiteral("camera_up_y"), 0.0);
            tcp.insert(QStringLiteral("camera_up_z"), 0.0);
        }
        body.insert(QStringLiteral("tcp"), tcp);
    }

    QString localError;
    const QJsonObject response = postJson(
        serverUrl, QStringLiteral("/execute_scan_waypoint"), body, 360000, &localError);
    Ur3eScanWaypointMoveResult result = parseScanMotionResponse(response, localError);
    if (errorMessage != nullptr && !result.ok && !result.stopped && !result.skipped)
        *errorMessage = result.errorMessage;
    else if (errorMessage != nullptr && (result.stopped || result.skipped))
        *errorMessage = result.errorMessage;
    return result;
}

Ur3eScanWaypointMoveResult ur3eExecuteMoveHome(const QString &serverUrl, QString *errorMessage)
{
    QString localError;
    QJsonObject body = buildScanMotionRequestBody();
    // After Stop / scan end, ignore racing stop latch and retry briefly so home completes.
    body.insert(QStringLiteral("post_scan_home"), true);
    body.insert(QStringLiteral("ignore_stop"), true);
    const QJsonObject response = postJson(
        serverUrl,
        QStringLiteral("/execute_move_home"),
        body,
        180000,
        &localError);
    Ur3eScanWaypointMoveResult result = parseScanMotionResponse(response, localError);
    if (errorMessage != nullptr && !result.ok)
        *errorMessage = result.errorMessage;
    return result;
}

Ur3eScanWaypointMoveResult ur3eExecuteHardwareJointMove(const QString &serverUrl,
                                                        const std::vector<double> &positionsRad,
                                                        const bool skipCollisionCheck,
                                                        QString *errorMessage,
                                                        const QString &label)
{
    QJsonObject body = buildScanMotionRequestBody();
    body.insert(QStringLiteral("joints"), positionsToJsonArray(positionsRad));
    body.insert(QStringLiteral("skip_collision_check"), skipCollisionCheck);
    if (!label.isEmpty())
        body.insert(QStringLiteral("label"), label);

    QString localError;
    const QJsonObject response = postJson(
        serverUrl, QStringLiteral("/execute_hardware_joint_move"), body, 360000, &localError);
    Ur3eScanWaypointMoveResult result = parseScanMotionResponse(response, localError);
    if (errorMessage != nullptr && !result.ok && !result.stopped)
        *errorMessage = result.errorMessage;
    else if (errorMessage != nullptr && result.stopped)
        *errorMessage = result.errorMessage;
    return result;
}

Ur3eWrist3RewindResult ur3eRewindWrist3Cable(const QString &serverUrl, QString *errorMessage)
{
    Ur3eWrist3RewindResult result;
    QString localError;
    const QJsonObject response = postJson(
        serverUrl,
        QStringLiteral("/rewind_wrist3_cable"),
        buildScanMotionRequestBody(),
        180000,
        &localError);
    if (response.isEmpty())
    {
        result.errorMessage = localError;
        if (errorMessage != nullptr)
            *errorMessage = localError;
        return result;
    }

    if (response.value(QStringLiteral("stopped")).toBool(false))
    {
        result.stopped = true;
        result.errorMessage = response.value(QStringLiteral("error")).toString(
            QStringLiteral("Motion stopped."));
        if (errorMessage != nullptr)
            *errorMessage = result.errorMessage;
        return result;
    }

    result.rewound = response.value(QStringLiteral("rewound")).toBool(false);
    result.turns = response.value(QStringLiteral("turns")).toInt(0);

    if (!response.value(QStringLiteral("ok")).toBool(false))
    {
        result.errorMessage = response.value(QStringLiteral("error")).toString(localError);
        if (errorMessage != nullptr)
            *errorMessage = result.errorMessage;
        return result;
    }

    result.ok = true;
    return result;
}

bool ur3eStopMotion(const QString &serverUrl, QString *errorMessage)
{
    const QJsonObject response =
        postJson(serverUrl, QStringLiteral("/stop"), QJsonObject(), 10000, errorMessage);
    return !response.isEmpty() && response.value(QStringLiteral("ok")).toBool(false);
}

bool ur3ePreviewManualTarget(const QString &serverUrl,
                             const std::vector<double> &positionsRad,
                             QString *errorMessage)
{
    QJsonObject body = buildScanMotionRequestBody();
    body.insert(QStringLiteral("joints"), positionsToJsonArray(positionsRad));

    QString localError;
    const QJsonObject response =
        postJson(serverUrl, QStringLiteral("/preview_manual_target"), body, 120000, &localError);
    if (response.isEmpty() || !response.value(QStringLiteral("ok")).toBool(false))
    {
        if (errorMessage != nullptr)
            *errorMessage = response.value(QStringLiteral("error")).toString(localError);
        return false;
    }
    return true;
}

bool ur3eSyncWorkspaceBoundary(const QString &serverUrl, QString *errorMessage)
{
    QString localError;
    const QJsonObject response = postJson(
        serverUrl,
        QStringLiteral("/sync_workspace_boundary"),
        buildScanMotionRequestBody(),
        30000,
        &localError);
    if (response.isEmpty() || !response.value(QStringLiteral("ok")).toBool(false))
    {
        if (errorMessage != nullptr)
            *errorMessage = response.value(QStringLiteral("error")).toString(localError);
        return false;
    }
    return true;
}

QJsonObject ur3ePostJsonRequest(const QString &serverUrl,
                                const QString &path,
                                const QJsonObject &body,
                                const int timeoutMs,
                                QString *errorMessage)
{
    return postJson(serverUrl, path, body, timeoutMs, errorMessage);
}

Ur3eHemisphereScanExecuteResult ur3eExecuteHemisphereScan(
    const QString &serverUrl,
    const std::vector<std::vector<double>> &waypointsRad,
    QString *errorMessage)
{
    Ur3eHemisphereScanExecuteResult result;
    QJsonArray waypoints;
    for (const std::vector<double> &joints : waypointsRad)
    {
        QJsonObject entry;
        entry.insert(QStringLiteral("joints"), positionsToJsonArray(joints));
        waypoints.append(entry);
    }

    QJsonObject body;
    body.insert(QStringLiteral("waypoints"), waypoints);

    QString localError;
    const QJsonObject response = postJson(
        serverUrl, QStringLiteral("/execute_hemisphere_scan"), body, 3600000, &localError);
    if (response.isEmpty())
    {
        result.errorMessage = localError;
        if (errorMessage != nullptr)
            *errorMessage = localError;
        return result;
    }

    if (!response.value(QStringLiteral("ok")).toBool(false))
    {
        result.errorMessage = response.value(QStringLiteral("error")).toString(localError);
        if (errorMessage != nullptr)
            *errorMessage = result.errorMessage;
        return result;
    }

    result.ok = true;
    result.executedCount = response.value(QStringLiteral("executed")).toInt(0);
    result.stopped = response.value(QStringLiteral("stopped")).toBool(false);
    return result;
}

} // namespace hf::ur3e
