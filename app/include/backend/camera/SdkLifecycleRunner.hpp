// Dedicated Qt thread for Lumo/NI SDK connect, init, and disconnect (not main GUI or camera control).
#pragma once

#include <QMutex>
#include <QThread>
#include <QWaitCondition>

#include <functional>

class QObject;

class SdkLifecycleRunner final : public QThread
{
    Q_OBJECT

public:
    explicit SdkLifecycleRunner(QObject *parent = nullptr);
    ~SdkLifecycleRunner() override;

    SdkLifecycleRunner(const SdkLifecycleRunner &) = delete;
    SdkLifecycleRunner &operator=(const SdkLifecycleRunner &) = delete;

    /// Runs task on this thread's event loop; blocks the caller until finished.
    void runSync(std::function<void()> task);

protected:
    void run() override;

private:
    QObject *workerAnchor_ = nullptr;
    QMutex anchorMutex_;
    QWaitCondition anchorReady_;
    bool anchorReadyFlag_ = false;
};
