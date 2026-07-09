// WSL UR3e HTTP server lifecycle manager (QProcess sidecar).
#include "backend/ur3e/Ur3eServerManager.hpp"

#include "backend/HyperFusionConfig.hpp"
#include "backend/ur3e/Ur3eClient.hpp"
#include "backend/ur3e/Ur3eWslLogUtil.hpp"
#include "backend/ur3e/Ur3eWslPathUtil.hpp"

#include <QMetaObject>
#include <QTimer>

#include <thread>

namespace hf::ur3e
{
namespace
{
QStringList wslBashArguments(const QString &script)
{
    const hf::HardwareConfig::Ur3eConfig &cfg = hf::hardwareConfig().ur3e;
    QStringList arguments;
    if (!cfg.wslDistro.trimmed().isEmpty())
        arguments << QStringLiteral("-d") << cfg.wslDistro.trimmed();
    arguments << QStringLiteral("--") << QStringLiteral("bash") << QStringLiteral("-lc") << script;
    return arguments;
}
} // namespace

Ur3eServerManager::Ur3eServerManager(QObject *parent)
    : QObject(parent)
{
    connect(&process_, &QProcess::readyReadStandardError, this, [this]() {
        if (silentMode_ && state_ == State::Starting)
            return;

        const QByteArray chunk = process_.readAllStandardError();
        if (chunk.trimmed().isEmpty())
            return;

        const QString text = sanitizeWslProcessOutput(chunk);
        if (text.isEmpty() || text.contains(QStringLiteral("GET /health")))
            return;

        lastDetail_ = text;
        emit stateChanged(state_, lastDetail_);
    });

    connect(&process_, &QProcess::readyReadStandardOutput, this, [this]() {
        if (silentMode_ && state_ == State::Starting)
            return;

        const QByteArray chunk = process_.readAllStandardOutput();
        if (chunk.trimmed().isEmpty())
            return;

        const QString text = sanitizeWslProcessOutput(chunk);
        if (text.contains(QStringLiteral("GET /health")))
            return;

        lastDetail_ = text;
        emit stateChanged(state_, lastDetail_);
    });

    connect(
        &process_,
        QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
        this,
        [this](const int exitCode, const QProcess::ExitStatus status) {
            if (state_ == State::Starting || state_ == State::Running)
            {
                if (silentMode_)
                {
                    markUnavailable();
                    return;
                }

                const QString detail =
                    QStringLiteral("UR3e server exited (code=%1, status=%2)")
                        .arg(exitCode)
                        .arg(status == QProcess::NormalExit ? QStringLiteral("normal")
                                                            : QStringLiteral("crash"));
                setState(State::Failed, detail);
            }
            else
            {
                setState(State::Stopped);
            }
        });

    connect(&process_, &QProcess::started, this, [this]() {
        if (state_ != State::Starting)
            return;

        healthPollAttempts_ = 0;
        scheduleNextHealthPoll(2000);
    });

    connect(&process_, &QProcess::errorOccurred, this, [this](const QProcess::ProcessError) {
        if (state_ != State::Starting)
            return;

        if (silentMode_)
            markUnavailable();
        else
            setState(State::Failed,
                     QStringLiteral("Failed to start wsl.exe: %1").arg(process_.errorString()));
    });

    connect(
        &cleanupProcess_,
        QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
        this,
        &Ur3eServerManager::onCleanupFinished);

    connect(&cleanupProcess_, &QProcess::errorOccurred, this, [this](const QProcess::ProcessError) {
        if (cleanupBeforeLaunch_ && state_ == State::Starting)
            onCleanupFinished();
    });
}

QString Ur3eServerManager::statusText() const
{
    switch (state_)
    {
    case State::Unavailable:
        return lastDetail_.isEmpty() ? QStringLiteral("Not available")
                                     : QStringLiteral("Not available — %1").arg(lastDetail_);
    case State::Stopped:
        return QStringLiteral("Stopped");
    case State::Starting:
        return QStringLiteral("Connecting\u2026");
    case State::Running:
        return lastDetail_.isEmpty() ? QStringLiteral("Connected") : lastDetail_;
    case State::Failed:
        return lastDetail_.isEmpty() ? QStringLiteral("Failed") : lastDetail_;
    }
    return QStringLiteral("Unknown");
}

QString Ur3eServerManager::serverUrl() const
{
    const hf::HardwareConfig::Ur3eConfig &cfg = hf::hardwareConfig().ur3e;
    return QStringLiteral("http://127.0.0.1:%1").arg(cfg.serverPort);
}

void Ur3eServerManager::setState(const State state, const QString &detail)
{
    const State previous = state_;
    state_ = state;
    if (!detail.isEmpty())
        lastDetail_ = detail;

    if (state == State::Running || state == State::Unavailable || state == State::Stopped)
        silentMode_ = false;

    if (previous != state)
        emit stateChanged(state_, lastDetail_);
}

void Ur3eServerManager::markUnavailable()
{
    silentMode_ = false;
    if (lastDetail_.isEmpty())
        lastDetail_ = QStringLiteral("Sidecar unavailable (check WSL, resources/ur3e, Log tab)");
    state_ = State::Unavailable;
    emit stateChanged(state_, lastDetail_);
}

QString Ur3eServerManager::buildLaunchCommand() const
{
    const hf::HardwareConfig::Ur3eConfig &cfg = hf::hardwareConfig().ur3e;
    const QString repoLinux = resolveUr3eRepoLinuxPath();

    QString serverArgs =
        QStringLiteral("./venv/bin/ur3e_server --host 0.0.0.0 --port %1")
            .arg(cfg.serverPort);
    serverArgs += QStringLiteral(" --robot-ip %1").arg(cfg.robotIp);
    serverArgs += QStringLiteral(" --dashboard-port %1").arg(cfg.dashboardPort);
    serverArgs += QStringLiteral(" --rtde-port %1").arg(cfg.rtdePort);
    serverArgs += QStringLiteral(" --max-linear-speed %1").arg(cfg.maxLinearSpeedMPerS, 0, 'g', 6);
    serverArgs += QStringLiteral(" --max-linear-accel %1").arg(cfg.maxLinearAccelMPerS2, 0, 'g', 6);
    if (cfg.useMockHardware)
        serverArgs += QStringLiteral(" --use-mock-hardware");
    else
        serverArgs += QStringLiteral(" --no-use-mock-hardware");
    serverArgs += QStringLiteral(" --ros-distro %1").arg(cfg.rosDistro);
    serverArgs += QStringLiteral(" --ur-type %1").arg(cfg.urType);
    if (cfg.prestartDriver)
        serverArgs += QStringLiteral(" --prestart-driver");

    const QString extraShell = cfg.wslBashCommand.trimmed();
    const QString rosDistro = cfg.rosDistro.trimmed().isEmpty() ? QStringLiteral("jazzy")
                                                                : cfg.rosDistro.trimmed();
    const QString rosSource = QStringLiteral("source /opt/ros/%1/setup.bash").arg(rosDistro);

    QString inner;
    if (extraShell.isEmpty())
        inner = QStringLiteral("cd '%1' && %2 && %3").arg(repoLinux, rosSource, serverArgs);
    else
        inner = QStringLiteral("cd '%1' && %2 && %3 && %4").arg(repoLinux, rosSource, extraShell, serverArgs);
    return inner;
}

void Ur3eServerManager::beginAsyncCleanup()
{
    if (cleanupProcess_.state() != QProcess::NotRunning)
    {
        cleanupProcess_.kill();
        cleanupProcess_.waitForFinished(500);
    }

    const QString cleanup = QStringLiteral(
        "pkill -f ur_control.launch.py 2>/dev/null || true; "
        "pkill -f ur3e_server 2>/dev/null || true; "
        "pkill -f hyperfusion_ur3e.sidecar.server 2>/dev/null || true");

    cleanupProcess_.setProgram(QStringLiteral("wsl.exe"));
    cleanupProcess_.setArguments(wslBashArguments(cleanup));
    cleanupProcess_.start();

    if (cleanupProcess_.state() == QProcess::NotRunning)
        QTimer::singleShot(0, this, &Ur3eServerManager::onCleanupFinished);
}

void Ur3eServerManager::onCleanupFinished()
{
    if (state_ != State::Starting || !cleanupBeforeLaunch_)
        return;

    cleanupBeforeLaunch_ = false;
    checkHealthThenLaunch();
}

void Ur3eServerManager::checkHealthThenLaunch()
{
    const int generation = startupGeneration_.load(std::memory_order_acquire);
    const QString url = serverUrl();

    std::thread([this, generation, url]() {
        Ur3eHealthStatus health;
        const bool alreadyRunning = ur3eServerHealthCheck(url, &health);

        QMetaObject::invokeMethod(
            this,
            [this, generation, alreadyRunning, health]() {
                if (generation != startupGeneration_.load(std::memory_order_acquire)
                    || state_ != State::Starting)
                {
                    return;
                }

                if (alreadyRunning)
                {
                    const QString detail =
                        health.useMockHardware ? QStringLiteral("Connected (simulation)")
                                               : QStringLiteral("Connected (robot %1)")
                                                     .arg(health.robotIp);
                    setState(State::Running, detail);
                    return;
                }

                launchServerProcess();
            },
            Qt::QueuedConnection);
    }).detach();
}

void Ur3eServerManager::launchServerProcess()
{
    if (state_ != State::Starting)
        return;

    const QString repoLinux = resolveUr3eRepoLinuxPath();
    if (repoLinux.isEmpty())
    {
        const QString detail = QStringLiteral("Could not locate resources/ur3e for WSL.");
        if (silentMode_)
        {
            lastDetail_ = detail;
            markUnavailable();
        }
        else
            setState(State::Failed, detail);
        return;
    }

    if (process_.state() != QProcess::NotRunning)
    {
        process_.kill();
        process_.waitForFinished(500);
    }

    const QString inner = buildLaunchCommand();
    process_.setProgram(QStringLiteral("wsl.exe"));
    process_.setArguments(wslBashArguments(inner));
    process_.start();
}

void Ur3eServerManager::tryAutoStart()
{
    if (state_ == State::Starting || state_ == State::Running)
        return;

    silentMode_ = true;
    cleanupBeforeLaunch_ = true;
    ++startupGeneration_;
    healthPollInFlight_.store(false, std::memory_order_release);

    setState(State::Starting, QStringLiteral("Cleaning stale WSL processes\u2026"));
    beginAsyncCleanup();
}

void Ur3eServerManager::startServer()
{
    if (state_ == State::Starting || state_ == State::Running)
        return;

    silentMode_ = false;
    cleanupBeforeLaunch_ = false;
    ++startupGeneration_;
    healthPollInFlight_.store(false, std::memory_order_release);

    setState(State::Starting, QStringLiteral("Launching WSL UR3e server\u2026"));
    checkHealthThenLaunch();
}

void Ur3eServerManager::scheduleNextHealthPoll(const int delayMs)
{
    QTimer::singleShot(delayMs, this, &Ur3eServerManager::pollHealth);
}

void Ur3eServerManager::handleHealthPollResult(const bool /*ok*/, const QString &error)
{
    healthPollInFlight_.store(false, std::memory_order_release);

    if (state_ != State::Starting)
        return;

    ++healthPollAttempts_;
    if (healthPollAttempts_ >= 120)
    {
        if (silentMode_)
            markUnavailable();
        else
            setState(State::Failed,
                     error.isEmpty() ? QStringLiteral("UR3e server did not become ready.") : error);
        return;
    }

    if (process_.state() == QProcess::NotRunning)
    {
        if (silentMode_)
            markUnavailable();
        else
        {
            setState(State::Failed,
                     lastDetail_.isEmpty() ? QStringLiteral("UR3e server process exited during startup.")
                                           : lastDetail_);
        }
        return;
    }

    scheduleNextHealthPoll(1000);
}

void Ur3eServerManager::pollHealth()
{
    if (state_ != State::Starting)
        return;

    if (healthPollInFlight_.exchange(true, std::memory_order_acq_rel))
        return;

    const int generation = startupGeneration_.load(std::memory_order_acquire);
    const QString url = serverUrl();

    std::thread([this, generation, url]() {
        Ur3eHealthStatus health;
        QString error;
        const bool ok = ur3eServerHealthCheck(url, &health, &error);

        QMetaObject::invokeMethod(
            this,
            [this, generation, ok, error, health]() {
                if (generation != startupGeneration_.load(std::memory_order_acquire))
                {
                    healthPollInFlight_.store(false, std::memory_order_release);
                    return;
                }

                if (ok)
                {
                    healthPollInFlight_.store(false, std::memory_order_release);
                    const QString detail =
                        health.useMockHardware ? QStringLiteral("Connected (simulation)")
                                               : QStringLiteral("Connected (hardware)");
                    setState(State::Running, detail);
                    return;
                }

                handleHealthPollResult(false, error);
            },
            Qt::QueuedConnection);
    }).detach();
}

void Ur3eServerManager::stopServer()
{
    silentMode_ = false;
    ++startupGeneration_;
    healthPollInFlight_.store(false, std::memory_order_release);

    if (cleanupProcess_.state() != QProcess::NotRunning)
    {
        cleanupProcess_.kill();
        cleanupProcess_.waitForFinished(500);
    }

    if (state_ == State::Running || state_ == State::Starting)
    {
        QString shutdownError;
        ur3eServerShutdown(serverUrl(), &shutdownError);
    }

    if (process_.state() != QProcess::NotRunning)
    {
        process_.terminate();
        if (!process_.waitForFinished(5000))
            process_.kill();
    }

    setState(State::Stopped);
}

} // namespace hf::ur3e
