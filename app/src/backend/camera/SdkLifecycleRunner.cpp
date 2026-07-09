// Dedicated Qt thread implementation for blocking Lumo/NI SDK lifecycle calls.
#include "backend/SdkLifecycleRunner.hpp"

#include <QMetaObject>
#include <QMutexLocker>
#include <QObject>

SdkLifecycleRunner::SdkLifecycleRunner(QObject *parent) : QThread(parent)
{
    start();
    QMutexLocker lock(&anchorMutex_);
    while (!anchorReadyFlag_)
        anchorReady_.wait(&anchorMutex_);
}

SdkLifecycleRunner::~SdkLifecycleRunner()
{
    quit();
    wait(15000);
}

void SdkLifecycleRunner::run()
{
    workerAnchor_ = new QObject();

    {
        QMutexLocker lock(&anchorMutex_);
        anchorReadyFlag_ = true;
        anchorReady_.wakeAll();
    }

    exec();

    delete workerAnchor_;
    workerAnchor_ = nullptr;
}

void SdkLifecycleRunner::runSync(std::function<void()> task)
{
    if (!task)
        return;

    if (QThread::currentThread() == this)
    {
        task();
        return;
    }

    QObject *anchor = nullptr;
    {
        QMutexLocker lock(&anchorMutex_);
        anchor = workerAnchor_;
    }
    if (anchor == nullptr)
        return;

    QMetaObject::invokeMethod(
        anchor,
        [t = std::move(task)]() mutable { t(); },
        Qt::BlockingQueuedConnection);
}
