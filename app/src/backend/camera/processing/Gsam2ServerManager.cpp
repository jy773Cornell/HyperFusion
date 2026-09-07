// WSL GSAM2 HTTP server lifecycle manager (QProcess sidecar).
#include "backend/camera/processing/Gsam2ServerManager.hpp"

#include "backend/HyperFusionConfig.hpp"
#include "backend/camera/processing/Gsam2SegmentationClient.hpp"
#include "backend/camera/processing/GsamWslPathUtil.hpp"

#include <QMetaObject>
#include <QTimer>

#include <thread>

namespace hf::processing
{
Gsam2ServerManager::Gsam2ServerManager(QObject *parent)
    : QObject(parent)
{
    connect(&process_, &QProcess::readyReadStandardError, this, [this]() {
        if (silentMode_ && state_ == State::Starting)
            return;

        const QByteArray chunk = process_.readAllStandardError();
        if (!chunk.trimmed().isEmpty())
            setState(state_, QString::fromUtf8(chunk.trimmed()));
    });

    connect(&process_, &QProcess::readyReadStandardOutput, this, [this]() {
        if (silentMode_ && state_ == State::Starting)
            return;

        const QByteArray chunk = process_.readAllStandardOutput();
        if (!chunk.trimmed().isEmpty())
            setState(state_, QString::fromUtf8(chunk.trimmed()));
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
                    QStringLiteral("GSAM2 server exited (code=%1, status=%2)")
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
}

QString Gsam2ServerManager::statusText() const
{
    switch (state_)
    {
    case State::Unavailable:
        return QStringLiteral("Not available");
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

QString Gsam2ServerManager::serverUrl() const
{
    const hf::HardwareConfig::SegmentationConfig &cfg = hf::hardwareConfig().segmentation;
    return QStringLiteral("http://127.0.0.1:%1").arg(cfg.serverPort);
}

void Gsam2ServerManager::setState(const State state, const QString &detail)
{
    state_ = state;
    if (!detail.isEmpty())
        lastDetail_ = detail;

    if (state == State::Running || state == State::Unavailable || state == State::Stopped)
        silentMode_ = false;

    emit stateChanged(state_, lastDetail_);
}

void Gsam2ServerManager::markUnavailable()
{
    silentMode_ = false;
    lastDetail_.clear();
    state_ = State::Unavailable;
    emit stateChanged(state_, lastDetail_);
}

QString Gsam2ServerManager::buildLaunchCommand() const
{
    const hf::HardwareConfig::SegmentationConfig &cfg = hf::hardwareConfig().segmentation;
    const QString repoLinux = resolveSam2RepoLinuxPath();
    const QString warmupFlag = cfg.warmupOnStart ? QStringLiteral(" --warmup") : QString();

    QString serverArgs = QStringLiteral("./venv/bin/python gsam2_server.py --host 0.0.0.0 --port %1")
                             .arg(cfg.serverPort);
    serverArgs += QStringLiteral(" --box-threshold %1").arg(cfg.boxThreshold, 0, 'g', 6);
    serverArgs += QStringLiteral(" --hf-model-id %1").arg(cfg.hfModelId);
    serverArgs += QStringLiteral(" --sam2-config %1").arg(cfg.sam2Config);
    serverArgs += QStringLiteral(" --sam2-checkpoint %1").arg(cfg.sam2Checkpoint);
    serverArgs += QStringLiteral(" --detector-device %1").arg(cfg.detectorDevice);
    serverArgs += QStringLiteral(" --sam2-device %1").arg(cfg.sam2Device);
    if (cfg.multimaskOutput)
        serverArgs += QStringLiteral(" --multimask-output");
    serverArgs += warmupFlag;

    const QString extraShell = cfg.wslBashCommand.trimmed();
    const bool skipExtraShell =
        extraShell.isEmpty()
        || extraShell == QStringLiteral("source ./venv/bin/activate")
        || extraShell == QStringLiteral("source ~/venvs/gsam2/bin/activate");

    QString inner;
    if (skipExtraShell)
        inner = QStringLiteral("cd '%1' && %2").arg(repoLinux, serverArgs);
    else
        inner = QStringLiteral("cd '%1' && %2 && %3").arg(repoLinux, extraShell, serverArgs);
    return inner;
}

void Gsam2ServerManager::tryAutoStart()
{
    silentMode_ = true;
    startServer();

    if (state_ == State::Failed)
        markUnavailable();
}

void Gsam2ServerManager::startServer()
{
    if (state_ == State::Starting || state_ == State::Running)
        return;

    const QString repoLinux = resolveSam2RepoLinuxPath();
    if (repoLinux.isEmpty())
    {
        if (silentMode_)
            markUnavailable();
        else
            setState(State::Failed, QStringLiteral("Could not locate app/sidecars/gsam2 for WSL."));
        return;
    }

    ++startupGeneration_;
    healthPollInFlight_.store(false, std::memory_order_release);
    healthPollAttempts_ = 0;

    if (!silentMode_)
        setState(State::Starting, QStringLiteral("Launching WSL GSAM2 server\u2026"));
    else
        setState(State::Starting);

    checkHealthThenLaunch();
}

void Gsam2ServerManager::checkHealthThenLaunch()
{
    const int generation = startupGeneration_.load(std::memory_order_acquire);
    const QString url = serverUrl();

    std::thread([this, generation, url]() {
        bool modelLoaded = false;
        const bool alreadyRunning = gsam2ServerHealthCheck(url, &modelLoaded);

        QMetaObject::invokeMethod(
            this,
            [this, generation, alreadyRunning, modelLoaded]() {
                if (generation != startupGeneration_.load(std::memory_order_acquire)
                    || state_ != State::Starting)
                {
                    return;
                }

                if (alreadyRunning)
                {
                    setState(State::Running,
                             modelLoaded ? QStringLiteral("Connected (models loaded)")
                                         : QStringLiteral("Connected"));
                    return;
                }

                launchServerProcess();
            },
            Qt::QueuedConnection);
    }).detach();
}

void Gsam2ServerManager::launchServerProcess()
{
    if (state_ != State::Starting)
        return;

    const hf::HardwareConfig::SegmentationConfig &cfg = hf::hardwareConfig().segmentation;
    const QString inner = buildLaunchCommand();

    QStringList arguments;
    if (!cfg.wslDistro.trimmed().isEmpty())
        arguments << QStringLiteral("-d") << cfg.wslDistro.trimmed();
    arguments << QStringLiteral("--") << QStringLiteral("bash") << QStringLiteral("-lc") << inner;

    if (process_.state() != QProcess::NotRunning)
    {
        process_.kill();
        process_.waitForFinished(500);
    }

    process_.setProgram(QStringLiteral("wsl.exe"));
    process_.setArguments(arguments);
    process_.start();
}

void Gsam2ServerManager::scheduleNextHealthPoll(const int delayMs)
{
    QTimer::singleShot(delayMs, this, &Gsam2ServerManager::pollHealth);
}

void Gsam2ServerManager::handleHealthPollResult(const bool /*ok*/, const QString &error)
{
    healthPollInFlight_.store(false, std::memory_order_release);

    if (state_ != State::Starting)
        return;

    ++healthPollAttempts_;
    if (healthPollAttempts_ >= 600)
    {
        if (silentMode_)
            markUnavailable();
        else
        {
            setState(State::Failed,
                     error.isEmpty() ? QStringLiteral("GSAM2 server did not become ready.") : error);
        }
        return;
    }

    if (process_.state() == QProcess::NotRunning)
    {
        if (silentMode_)
            markUnavailable();
        else
        {
            setState(State::Failed,
                     lastDetail_.isEmpty() ? QStringLiteral("GSAM2 server process exited during startup.")
                                           : lastDetail_);
        }
        return;
    }

    scheduleNextHealthPoll(1000);
}

void Gsam2ServerManager::pollHealth()
{
    if (state_ != State::Starting)
        return;

    if (healthPollInFlight_.exchange(true, std::memory_order_acq_rel))
        return;

    const int generation = startupGeneration_.load(std::memory_order_acquire);
    const QString url = serverUrl();

    std::thread([this, generation, url]() {
        bool modelLoaded = false;
        QString error;
        const bool ok = gsam2ServerHealthCheck(url, &modelLoaded, &error);

        QMetaObject::invokeMethod(
            this,
            [this, generation, ok, error, modelLoaded]() {
                if (generation != startupGeneration_.load(std::memory_order_acquire))
                {
                    healthPollInFlight_.store(false, std::memory_order_release);
                    return;
                }

                if (ok)
                {
                    healthPollInFlight_.store(false, std::memory_order_release);
                    setState(State::Running,
                             modelLoaded ? QStringLiteral("Connected (models loaded)")
                                         : QStringLiteral("Connected (warming up on first request)"));
                    return;
                }

                handleHealthPollResult(false, error);
            },
            Qt::QueuedConnection);
    }).detach();
}

void Gsam2ServerManager::stopServer()
{
    silentMode_ = false;
    ++startupGeneration_;
    healthPollInFlight_.store(false, std::memory_order_release);

    if (state_ == State::Running || state_ == State::Starting)
    {
        QString shutdownError;
        gsam2ServerShutdown(serverUrl(), &shutdownError);
    }

    if (process_.state() != QProcess::NotRunning)
    {
        process_.terminate();
        if (!process_.waitForFinished(5000))
            process_.kill();
    }

    setState(State::Stopped);
}

} // namespace hf::processing
