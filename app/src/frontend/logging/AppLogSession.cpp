// Per-run session log file writer (logs/ beside app.exe).
#include "frontend/logging/AppLogSession.hpp"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QMutexLocker>
#include <QTextStream>
#include <QStringConverter>

namespace hf::log
{
QString channelFileTag(const Channel channel)
{
    switch (channel)
    {
    case Channel::App:
        return QStringLiteral("app");
    case Channel::Fx10e:
        return QStringLiteral("fx10e");
    case Channel::Swir:
        return QStringLiteral("swir");
    case Channel::Stage:
        return QStringLiteral("stage");
    case Channel::Light:
        return QStringLiteral("light");
    case Channel::Ur3e:
        return QStringLiteral("ur3e");
    case Channel::Capture:
        return QStringLiteral("capture");
    }
    return QStringLiteral("app");
}

SessionLogWriter::~SessionLogWriter()
{
    close();
}

bool SessionLogWriter::begin(const QString &applicationDirectory)
{
    QMutexLocker lock(&mutex_);
    if (file_ != nullptr)
        return true;

    const QString logsDir = QDir(applicationDirectory).filePath(QStringLiteral("logs"));
    if (!QDir().mkpath(logsDir))
        return false;

    const QString stamp =
        QDateTime::currentDateTime().toString(QStringLiteral("yyyy-MM-dd_HHmmss"));
    filePath_ = QDir(logsDir).filePath(QStringLiteral("hyperfusion_%1.log").arg(stamp));

    auto *opened = new QFile(filePath_);
    if (!opened->open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text))
    {
        delete opened;
        filePath_.clear();
        return false;
    }

    file_ = opened;
    QTextStream stream(file_);
    stream.setEncoding(QStringConverter::Utf8);
    stream << QStringLiteral("# HyperFusion session log %1\n")
                  .arg(QDateTime::currentDateTime().toString(Qt::ISODate));
    stream.flush();
    return true;
}

void SessionLogWriter::write(const Channel channel, const QString &message)
{
    QMutexLocker lock(&mutex_);
    if (file_ == nullptr || !file_->isOpen())
        return;

    const QString ts = QDateTime::currentDateTime().toString(QStringLiteral("yyyy-MM-dd hh:mm:ss"));
    QTextStream stream(file_);
    stream.setEncoding(QStringConverter::Utf8);
    stream << QStringLiteral("[%1] %2 %3\n").arg(channelFileTag(channel), ts, message);
    stream.flush();
}

void SessionLogWriter::close()
{
    QMutexLocker lock(&mutex_);
    if (file_ == nullptr)
        return;

    if (file_->isOpen())
    {
        QTextStream stream(file_);
        stream.setEncoding(QStringConverter::Utf8);
        stream << QStringLiteral("# session ended %1\n")
                      .arg(QDateTime::currentDateTime().toString(Qt::ISODate));
        stream.flush();
        file_->close();
    }

    delete file_;
    file_ = nullptr;
}

bool SessionLogWriter::isOpen() const
{
    QMutexLocker lock(&mutex_);
    return file_ != nullptr && file_->isOpen();
}

QString SessionLogWriter::filePath() const
{
    QMutexLocker lock(&mutex_);
    return filePath_;
}

} // namespace hf::log
