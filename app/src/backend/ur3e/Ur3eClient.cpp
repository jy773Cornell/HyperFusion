// HTTP client for the UR3e WSL sidecar server (via wsl curl).
#include "backend/ur3e/Ur3eClient.hpp"

#include "backend/HyperFusionConfig.hpp"
#include "backend/processing/GsamWslPathUtil.hpp"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QProcess>
#include <QTemporaryFile>
#include <QUrl>

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

QByteArray runWslCurl(const QStringList &curlArgs, const int timeoutMs, QString *errorMessage)
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

    if (process.exitStatus() != QProcess::NormalExit || process.exitCode() != 0)
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
        QStringLiteral("-sfS"),
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

void fillHealthStatus(const QJsonObject &response, Ur3eHealthStatus *status)
{
    if (status == nullptr)
        return;

    status->ok = response.value(QStringLiteral("status")).toString() == QStringLiteral("ok");
    status->useMockHardware = response.value(QStringLiteral("use_mock_hardware")).toBool(true);
    status->robotConnected = response.value(QStringLiteral("robot_connected")).toBool(false);
    status->driverState = response.value(QStringLiteral("driver_state")).toString();
    status->robotIp = response.value(QStringLiteral("robot_ip")).toString();
    status->fault = response.value(QStringLiteral("fault")).toString();
}
} // namespace

bool ur3eServerHealthCheck(const QString &serverUrl,
                           Ur3eHealthStatus *status,
                           QString *errorMessage)
{
    const QJsonObject response = getJson(serverUrl, QStringLiteral("/health"), 3000, errorMessage);
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

Ur3eConnectResult ur3eConnectRobot(const QString &serverUrl,
                                     const QString &robotIp,
                                     QString *errorMessage)
{
    Ur3eConnectResult result;
    QJsonObject body;
    if (!robotIp.trimmed().isEmpty())
        body.insert(QStringLiteral("ip"), robotIp.trimmed());

    QString localError;
    const QJsonObject response =
        postJson(serverUrl, QStringLiteral("/connect"), body, 240000, &localError);
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
    result.useMockHardware = response.value(QStringLiteral("use_mock_hardware")).toBool(
        response.value(QStringLiteral("mode")).toString() == QStringLiteral("simulation"));
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

bool ur3eStopMotion(const QString &serverUrl, QString *errorMessage)
{
    const QJsonObject response =
        postJson(serverUrl, QStringLiteral("/stop"), QJsonObject(), 10000, errorMessage);
    return !response.isEmpty() && response.value(QStringLiteral("ok")).toBool(false);
}

} // namespace hf::ur3e
