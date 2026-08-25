// Launches RViz2 for UR3e visualization only (no MoveIt) in WSL (separate window).
#include "backend/multiview/Ur3eRvizManager.hpp"

#include "backend/HyperFusionConfig.hpp"
#include "backend/multiview/Ur3eWslLogUtil.hpp"
#include "backend/multiview/Ur3eWslPathUtil.hpp"

#include <QProcess>

namespace hf::ur3e
{
Ur3eRvizManager::Ur3eRvizManager(QObject *parent)
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
            markStopped(QStringLiteral("RViz exited (code=%1, status=%2)")
                            .arg(exitCode)
                            .arg(status == QProcess::NormalExit ? QStringLiteral("normal")
                                                                : QStringLiteral("crash")));
        });
}

void Ur3eRvizManager::markStopped(const QString &detail)
{
    if (!running_)
        return;

    running_ = false;
    emit stateChanged(false, detail);
}

bool Ur3eRvizManager::isRunning() const
{
    return running_;
}

QString Ur3eRvizManager::buildLaunchCommand() const
{
    const hf::HardwareConfig::Ur3eConfig &cfg = hf::hardwareConfig().ur3e;
    const QString rosDistro = cfg.rosDistro.trimmed().isEmpty() ? QStringLiteral("jazzy")
                                                                : cfg.rosDistro.trimmed();
    const QString urType =
        cfg.urType.trimmed().isEmpty() ? QStringLiteral("ur3e") : cfg.urType.trimmed();

    const QString repoLinux = resolveUr3eRepoLinuxPath();
    const QString scriptPath =
        repoLinux.isEmpty() ? QString()
                            : QStringLiteral("%1/scripts/launch_rviz.sh").arg(repoLinux);

    if (scriptPath.isEmpty())
    {
        return QStringLiteral("source /opt/ros/%1/setup.bash && exec rviz2").arg(rosDistro);
    }

    return QStringLiteral(
               "export ROS_LOCALHOST_ONLY=1 && "
               "export HYPERFUSION_UR3E_REPO='%1' && "
               "export HYPERFUSION_UR3E_SERVER_PORT='%5' && "
               "%6"
               "%7"
               "sed 's/\\r$//' '%2' | bash -s %3 %4")
        .arg(repoLinux,
             scriptPath,
             rosDistro,
             urType,
             QString::number(cfg.serverPort),
             buildMountEnvExports(cfg),
             buildToolPayloadEnvExports(cfg));
}

void Ur3eRvizManager::start()
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
        markStopped(QStringLiteral("Failed to start RViz via wsl.exe: %1")
                        .arg(process_.errorString()));
        return;
    }

    running_ = true;
    emit stateChanged(true, QStringLiteral("RViz launching in WSL\u2026"));
}

void Ur3eRvizManager::stop()
{
    if (!running_ && process_.state() == QProcess::NotRunning)
        return;

    if (process_.state() != QProcess::NotRunning)
    {
        process_.terminate();
        if (!process_.waitForFinished(5000))
            process_.kill();
    }

    markStopped(QStringLiteral("RViz stopped."));
}

} // namespace hf::ur3e
