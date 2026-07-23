// Launches MoveIt 2 + RViz for UR3e in WSL (separate window).
#include "backend/3dscanning/Ur3eMoveItManager.hpp"

#include "backend/HyperFusionConfig.hpp"
#include "backend/3dscanning/Ur3eWslLogUtil.hpp"
#include "backend/3dscanning/Ur3eWslPathUtil.hpp"

#include <QProcess>

namespace hf::ur3e
{
Ur3eMoveItManager::Ur3eMoveItManager(QObject *parent)
    : QObject(parent)
{
    connect(&process_, &QProcess::readyReadStandardError, this, [this]() {
        const QString text = sanitizeWslProcessOutput(process_.readAllStandardError());
        if (!text.isEmpty())
            emit stateChanged(running_, text);
    });

    connect(&process_, &QProcess::readyReadStandardOutput, this, [this]() {
        const QString text = sanitizeWslProcessOutput(process_.readAllStandardOutput());
        if (!text.isEmpty())
            emit stateChanged(running_, text);
    });

    connect(
        &process_,
        QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
        this,
        [this](const int exitCode, const QProcess::ExitStatus status) {
            markStopped(QStringLiteral("MoveIt exited (code=%1, status=%2)")
                            .arg(exitCode)
                            .arg(status == QProcess::NormalExit ? QStringLiteral("normal")
                                                                : QStringLiteral("crash")));
        });
}

void Ur3eMoveItManager::markStopped(const QString &detail)
{
    if (!running_)
        return;

    running_ = false;
    emit stateChanged(false, detail);
}

bool Ur3eMoveItManager::isRunning() const
{
    return running_;
}

QString Ur3eMoveItManager::buildLaunchCommand() const
{
    const hf::HardwareConfig::Ur3eConfig &cfg = hf::hardwareConfig().ur3e;
    const QString rosDistro = cfg.rosDistro.trimmed().isEmpty() ? QStringLiteral("jazzy")
                                                                : cfg.rosDistro.trimmed();
    const QString urType =
        cfg.urType.trimmed().isEmpty() ? QStringLiteral("ur3e") : cfg.urType.trimmed();

    const QString repoLinux = resolveUr3eRepoLinuxPath();
    const QString scriptPath =
        repoLinux.isEmpty()
            ? QString()
            : QStringLiteral("%1/scripts/launch_moveit.sh").arg(repoLinux);

    if (scriptPath.isEmpty())
    {
        return QStringLiteral("source /opt/ros/%1/setup.bash && exec ros2 launch ur_moveit_config "
                              "ur_moveit.launch.py ur_type:=%2 launch_rviz:=true launch_servo:=false")
            .arg(rosDistro, urType);
    }

    return QStringLiteral(
               "export ROS_LOCALHOST_ONLY=1 && "
               "export HYPERFUSION_UR3E_REPO='%1' && "
               "export HYPERFUSION_UR3E_SERVER_PORT='%6' && "
               "%7"
               "%8"
               "export HYPERFUSION_USE_MOCK_HARDWARE='%5' && "
               "pkill -f 'moveit_ros_move_group/[m]ove_group' 2>/dev/null || true; "
               "sleep 1; "
               "sed 's/\\r$//' '%2' | bash -s %3 %4")
        .arg(repoLinux,
             scriptPath,
             rosDistro,
             urType,
             cfg.useMockHardware ? QStringLiteral("true") : QStringLiteral("false"),
             QString::number(cfg.serverPort),
             buildMountEnvExports(cfg),
             buildToolPayloadEnvExports(cfg));
}

void Ur3eMoveItManager::start()
{
    if (isRunning())
        return;

    if (process_.state() != QProcess::NotRunning)
    {
        process_.kill();
        process_.waitForFinished(3000);
    }

    const hf::HardwareConfig::Ur3eConfig &cfg = hf::hardwareConfig().ur3e;
    const QString inner = buildLaunchCommand();

    QStringList arguments;
    if (!cfg.wslDistro.trimmed().isEmpty())
        arguments << QStringLiteral("-d") << cfg.wslDistro.trimmed();
    arguments << QStringLiteral("--") << QStringLiteral("bash") << QStringLiteral("-lc") << inner;

    process_.setProgram(QStringLiteral("wsl.exe"));
    process_.setArguments(arguments);
    process_.start();

    if (!process_.waitForStarted(10000))
    {
        markStopped(QStringLiteral("Failed to start MoveIt via wsl.exe: %1")
                        .arg(process_.errorString()));
        return;
    }

    running_ = true;
    emit stateChanged(true, QStringLiteral("MoveIt + RViz launching in WSL\u2026"));
}

void Ur3eMoveItManager::stop()
{
    if (!running_ && process_.state() == QProcess::NotRunning)
        return;

    if (process_.state() != QProcess::NotRunning)
    {
        process_.terminate();
        if (!process_.waitForFinished(5000))
            process_.kill();
    }

    markStopped(QStringLiteral("MoveIt stopped."));
}

} // namespace hf::ur3e
