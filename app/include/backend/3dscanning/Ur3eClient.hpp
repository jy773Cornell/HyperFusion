// HTTP client for the UR3e WSL sidecar server.
#pragma once

#include <QString>
#include <QStringList>
#include <QJsonObject>
#include <vector>

namespace hf::ur3e
{
struct Ur3eScanTcpPose;
struct Ur3eTcpPose
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double rx = 0.0;
    double ry = 0.0;
    double rz = 0.0;
};

struct Ur3eHealthStatus
{
    bool ok = false;
    bool useMockHardware = true;
    bool robotConnected = false;
    bool driverReady = false;
    QString driverState;
    QString robotIp;
    QString fault;
};

struct Ur3eConnectResult
{
    bool ok = false;
    QString errorMessage;
    bool useMockHardware = true;
    QString driverState;
};

struct Ur3eConnectAsyncStatus
{
    bool ok = false;
    bool inProgress = false;
    bool alreadyConnected = false;
    bool useMockHardware = true;
    bool reverseConnected = false;
    bool scriptPortListening = false;
    bool robotConnected = false;
    QString phase;
    QString message;
    QString errorMessage;
    QString driverState;
    QString robotIp;
    QString reverseIp;
};

struct Ur3ePoseResult
{
    bool ok = false;
    QString errorMessage;
    Ur3eTcpPose pose;
    QString driverState;
};

struct Ur3eMoveResult
{
    bool ok = false;
    QString errorMessage;
    Ur3eTcpPose pose;
};

struct Ur3eJointsState
{
    bool ok = false;
    QString errorMessage;
    QStringList names;
    std::vector<double> positionsRad;
    QString driverState;
};

struct Ur3eJointsMoveResult
{
    bool ok = false;
    QString errorMessage;
    std::vector<double> positionsRad;
};

struct Ur3eScanWaypointMoveResult
{
    bool ok = false;
    QString errorMessage;
    bool stopped = false;
    bool skipped = false;
    bool alreadyAtHome = false;
};

bool ur3eServerHealthCheck(const QString &serverUrl,
                           Ur3eHealthStatus *status = nullptr,
                           QString *errorMessage = nullptr);

bool ur3eServerShutdown(const QString &serverUrl, QString *errorMessage = nullptr);

Ur3eConnectResult ur3eConnectRobot(const QString &serverUrl,
                                   const QString &robotIp,
                                   QString *errorMessage = nullptr);

bool ur3eConnectStart(const QString &serverUrl,
                      const QString &robotIp,
                      Ur3eConnectAsyncStatus *status = nullptr,
                      QString *errorMessage = nullptr);

bool ur3eConnectStatus(const QString &serverUrl,
                       Ur3eConnectAsyncStatus *status,
                       QString *errorMessage = nullptr);

bool ur3eConnectCancel(const QString &serverUrl, QString *errorMessage = nullptr);

bool ur3eSidecarSupportsAsyncConnect(const QString &serverUrl, QString *errorMessage = nullptr);

Ur3eConnectResult ur3eDisconnectRobot(const QString &serverUrl, QString *errorMessage = nullptr);

Ur3ePoseResult ur3eGetTcpPose(const QString &serverUrl, QString *errorMessage = nullptr);

Ur3eMoveResult ur3eMoveLinear(const QString &serverUrl,
                              const Ur3eTcpPose &pose,
                              double speedMPerS,
                              double accelMPerS2,
                              bool waitUntilDone,
                              QString *errorMessage = nullptr);

Ur3eJointsState ur3eGetJoints(const QString &serverUrl, QString *errorMessage = nullptr);

Ur3eJointsMoveResult ur3eMoveJoints(const QString &serverUrl,
                                    const std::vector<double> &positionsRad,
                                    bool waitUntilDone,
                                    QString *errorMessage = nullptr);

Ur3eScanWaypointMoveResult ur3eExecuteScanWaypoint(const QString &serverUrl,
                                                   const std::vector<double> &positionsRad,
                                                   const Ur3eScanTcpPose *tcpPose = nullptr,
                                                   QString *errorMessage = nullptr,
                                                   const bool requireHomeFirst = false,
                                                   const bool directOnly = false);

Ur3eScanWaypointMoveResult ur3eExecuteMoveHome(const QString &serverUrl,
                                               QString *errorMessage = nullptr);

bool ur3eStopMotion(const QString &serverUrl, QString *errorMessage = nullptr);

bool ur3ePreviewManualTarget(const QString &serverUrl,
                             const std::vector<double> &positionsRad,
                             QString *errorMessage = nullptr);

bool ur3eSyncWorkspaceBoundary(const QString &serverUrl, QString *errorMessage = nullptr);

QJsonObject ur3ePostJsonRequest(const QString &serverUrl,
                                const QString &path,
                                const QJsonObject &body,
                                int timeoutMs,
                                QString *errorMessage = nullptr);

struct Ur3eHemisphereScanExecuteResult
{
    bool ok = false;
    QString errorMessage;
    int executedCount = 0;
    bool stopped = false;
};

Ur3eHemisphereScanExecuteResult ur3eExecuteHemisphereScan(
    const QString &serverUrl,
    const std::vector<std::vector<double>> &waypointsRad,
    QString *errorMessage = nullptr);

} // namespace hf::ur3e
