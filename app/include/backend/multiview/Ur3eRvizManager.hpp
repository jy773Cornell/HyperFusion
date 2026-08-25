// Launches RViz2 for UR3e visualization only (no MoveIt) in WSL (separate window).
#pragma once

#include <QObject>
#include <QProcess>
#include <QString>

namespace hf::ur3e
{
class Ur3eRvizManager : public QObject
{
    Q_OBJECT

public:
    explicit Ur3eRvizManager(QObject *parent = nullptr);

    [[nodiscard]] bool isRunning() const;
    void start();
    void stop();

signals:
    void stateChanged(bool running, const QString &detail);

private:
    QString buildLaunchCommand() const;
    void markStopped(const QString &detail);

    QProcess process_;
    bool running_ = false;
};

} // namespace hf::ur3e
