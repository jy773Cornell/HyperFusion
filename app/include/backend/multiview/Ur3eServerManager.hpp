// WSL UR3e HTTP server lifecycle manager (QProcess sidecar).
#pragma once

#include <QObject>
#include <QProcess>
#include <QString>

#include <atomic>

namespace hf::ur3e
{
class Ur3eServerManager : public QObject
{
    Q_OBJECT

public:
    enum class State
    {
        Unavailable,
        Stopped,
        Starting,
        Running,
        Failed,
    };

    explicit Ur3eServerManager(QObject *parent = nullptr);

    State state() const { return state_; }
    bool isServerConnected() const { return state_ == State::Running; }
    QString statusText() const;
    QString serverUrl() const;

    void tryAutoStart();
    void startServer();
    void stopServer();

public slots:
    void pollHealth();

signals:
    void stateChanged(hf::ur3e::Ur3eServerManager::State state, const QString &detail);

private:
    void setState(State state, const QString &detail = QString());
    QString buildLaunchCommand() const;
    void markUnavailable();
    QString launchFailureDetail(const QString &prefix) const;
    void ingestProcessOutput(const QByteArray &chunk, const bool emitToUi);
    void beginAsyncCleanup();
    void onCleanupFinished();
    void checkHealthThenLaunch();
    void launchServerProcess();
    void handleHealthPollResult(const bool ok, const QString &error);
    void scheduleNextHealthPoll(const int delayMs);

    QProcess process_;
    QProcess cleanupProcess_;
    State state_ = State::Unavailable;
    QString lastDetail_;
    QString lastLaunchOutput_;
    int healthPollAttempts_ = 0;
    bool silentMode_ = false;
    bool cleanupBeforeLaunch_ = false;
    std::atomic<int> startupGeneration_{0};
    std::atomic<bool> healthPollInFlight_{false};
};

} // namespace hf::ur3e
