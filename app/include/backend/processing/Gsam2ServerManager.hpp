// WSL GSAM2 HTTP server lifecycle manager (QProcess sidecar).
#pragma once

#include <QObject>
#include <QProcess>
#include <QString>

namespace hf::processing
{
class Gsam2ServerManager : public QObject
{
    Q_OBJECT

public:
    enum class State
    {
        Stopped,
        Starting,
        Running,
        Failed,
    };

    explicit Gsam2ServerManager(QObject *parent = nullptr);

    State state() const { return state_; }
    QString statusText() const;
    QString serverUrl() const;

    void startServer();
    void stopServer();

public slots:
    void pollHealth();

signals:
    void stateChanged(hf::processing::Gsam2ServerManager::State state, const QString &detail);

private:
    void setState(State state, const QString &detail = QString());
    QString buildLaunchCommand() const;

    QProcess process_;
    State state_ = State::Stopped;
    QString lastDetail_;
    int healthPollAttempts_ = 0;
};

} // namespace hf::processing
