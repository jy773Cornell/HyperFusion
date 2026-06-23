// WSL GSAM2 HTTP server lifecycle manager (QProcess sidecar).
#include "backend/processing/Gsam2ServerManager.hpp"

#include "backend/HyperFusionConfig.hpp"
#include "backend/processing/Gsam2SegmentationClient.hpp"
#include "backend/processing/GsamWslPathUtil.hpp"

#include <QTimer>

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
    const QString activate = cfg.wslBashCommand.trimmed();
    const QString warmupFlag = cfg.warmupOnStart ? QStringLiteral(" --warmup") : QString();

    QString serverArgs = QStringLiteral("python gsam2_server.py --host 0.0.0.0 --port %1")
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

    QString inner = QStringLiteral("cd '%1' && %2 && %3")
                        .arg(repoLinux, activate, serverArgs);
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
            setState(State::Failed, QStringLiteral("Could not locate resources/sam2 for WSL."));
        return;
    }

    bool modelLoaded = false;
    if (gsam2ServerHealthCheck(serverUrl(), &modelLoaded))
    {
        setState(State::Running,
                 modelLoaded ? QStringLiteral("Connected (models loaded)")
                             : QStringLiteral("Connected"));
        return;
    }

    const hf::HardwareConfig::SegmentationConfig &cfg = hf::hardwareConfig().segmentation;
    const QString inner = buildLaunchCommand();

    QStringList arguments;
    if (!cfg.wslDistro.trimmed().isEmpty())
        arguments << QStringLiteral("-d") << cfg.wslDistro.trimmed();
    arguments << QStringLiteral("--") << QStringLiteral("bash") << QStringLiteral("-lc") << inner;

    if (!silentMode_)
        setState(State::Starting, QStringLiteral("Launching WSL GSAM2 server\u2026"));
    else
        setState(State::Starting);

    healthPollAttempts_ = 0;
    process_.setProgram(QStringLiteral("wsl.exe"));
    process_.setArguments(arguments);
    process_.start();

    if (!process_.waitForStarted(10000))
    {
        if (silentMode_)
            markUnavailable();
        else
            setState(State::Failed, QStringLiteral("Failed to start wsl.exe: %1").arg(process_.errorString()));
        return;
    }

    QTimer::singleShot(2000, this, &Gsam2ServerManager::pollHealth);
}

void Gsam2ServerManager::pollHealth()
{
    if (state_ != State::Starting)
        return;

    bool modelLoaded = false;
    QString error;
    if (gsam2ServerHealthCheck(serverUrl(), &modelLoaded, &error))
    {
        setState(State::Running,
                 modelLoaded ? QStringLiteral("Connected (models loaded)")
                             : QStringLiteral("Connected (warming up on first request)"));
        return;
    }

    ++healthPollAttempts_;
    if (healthPollAttempts_ >= 600)
    {
        if (silentMode_)
            markUnavailable();
        else
        {
            setState(State::Failed,
                     error.isEmpty() ? QStringLiteral("GSAM2 server did not become ready.")
                                     : error);
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

    QTimer::singleShot(1000, this, &Gsam2ServerManager::pollHealth);
}

void Gsam2ServerManager::stopServer()
{
    silentMode_ = false;

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
