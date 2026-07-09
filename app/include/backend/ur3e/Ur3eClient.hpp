// HTTP client for the UR3e WSL sidecar server.
#pragma once

#include <QString>
#include <QStringList>
#include <vector>

namespace hf::ur3e
{
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

bool ur3eServerHealthCheck(const QString &serverUrl,
                           Ur3eHealthStatus *status = nullptr,
                           QString *errorMessage = nullptr);

bool ur3eServerShutdown(const QString &serverUrl, QString *errorMessage = nullptr);

Ur3eConnectResult ur3eConnectRobot(const QString &serverUrl,
                                   const QString &robotIp,
                                   QString *errorMessage = nullptr);

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

bool ur3eStopMotion(const QString &serverUrl, QString *errorMessage = nullptr);

} // namespace hf::ur3e
