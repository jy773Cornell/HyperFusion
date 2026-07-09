// Per-run session log file writer (logs/ beside app.exe).
#pragma once

#include "frontend/logging/AppLog.hpp"

#include <QMutex>
#include <QString>

class QFile;

namespace hf::log
{
QString channelFileTag(Channel channel);

class SessionLogWriter
{
public:
    SessionLogWriter() = default;
    ~SessionLogWriter();

    SessionLogWriter(const SessionLogWriter &) = delete;
    SessionLogWriter &operator=(const SessionLogWriter &) = delete;

    bool begin(const QString &applicationDirectory);
    void write(Channel channel, const QString &message);
    void close();

    [[nodiscard]] bool isOpen() const;
    [[nodiscard]] QString filePath() const;

private:
    mutable QMutex mutex_;
    QFile *file_ = nullptr;
    QString filePath_;
};

} // namespace hf::log
