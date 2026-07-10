// WSL GSAM2 HTTP server lifecycle manager (QProcess sidecar).
#pragma once

#include <QObject>
#include <QProcess>
#include <QString>

#include <atomic>

namespace hf::processing
{
class Gsam2ServerManager : public QObject
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

    explicit Gsam2ServerManager(QObject *parent = nullptr);

    State state() const { return state_; }
    bool isServerConnected() const { return state_ == State::Running; }
    QString statusText() const;
    QString serverUrl() const;

    /// Best-effort launch on app start; missing WSL/resources leave state Unavailable (no error UI).
    void tryAutoStart();
    void startServer();
    void stopServer();

public slots:
    void pollHealth();

signals:
    void stateChanged(hf::processing::Gsam2ServerManager::State state, const QString &detail);

private:
    void setState(State state, const QString &detail = QString());
    QString buildLaunchCommand() const;
    void markUnavailable();
    void checkHealthThenLaunch();
    void launchServerProcess();
    void scheduleNextHealthPoll(int delayMs);
    void handleHealthPollResult(bool ok, const QString &error);

    QProcess process_;
    State state_ = State::Unavailable;
    QString lastDetail_;
    int healthPollAttempts_ = 0;
    bool silentMode_ = false;
    std::atomic<int> startupGeneration_{0};
    std::atomic<bool> healthPollInFlight_{false};
};

} // namespace hf::processing
