// Application log channel labels and message routing.
#include "frontend/logging/AppLog.hpp"

namespace hf::log
{
namespace
{
bool startsWithInsensitive(const QString &text, const QString &prefix)
{
    return text.size() >= prefix.size()
           && text.left(prefix.size()).compare(prefix, Qt::CaseInsensitive) == 0;
}
} // namespace

QString channelLabel(Channel channel)
{
    switch (channel)
    {
    case Channel::App:
        return QStringLiteral("App");
    case Channel::Fx10e:
        return QStringLiteral("FX10e");
    case Channel::Swir:
        return QStringLiteral("SWIR");
    case Channel::Stage:
        return QStringLiteral("Stage");
    case Channel::Light:
        return QStringLiteral("Light");
    case Channel::Ur3e:
        return QStringLiteral("3D Scanning");
    case Channel::Capture:
        return QStringLiteral("Capture");
    }
    return QStringLiteral("Log");
}

QString channelTabTitle(Channel channel)
{
    return channelLabel(channel);
}

Channel classifyMessage(const QString &message)
{
    if (message.startsWith(QStringLiteral("  ")))
        return Channel::App;

    const QString trimmed = message.trimmed();
    if (trimmed.isEmpty())
        return Channel::App;

    if (startsWithInsensitive(trimmed, QStringLiteral("UR3e")))
        return Channel::Ur3e;

    if (startsWithInsensitive(trimmed, QStringLiteral("Light:"))
        || startsWithInsensitive(trimmed, QStringLiteral("Light error")))
        return Channel::Light;

    if (startsWithInsensitive(trimmed, QStringLiteral("Capture"))
        || startsWithInsensitive(trimmed, QStringLiteral("GSAM2"))
        || startsWithInsensitive(trimmed, QStringLiteral("Dual-camera")))
        return Channel::Capture;

    if (startsWithInsensitive(trimmed, QStringLiteral("Hardware config")))
        return Channel::App;

    if (startsWithInsensitive(trimmed, QStringLiteral("Stage:"))
        || startsWithInsensitive(trimmed, QStringLiteral("Stage error")))
        return Channel::Stage;

    if (startsWithInsensitive(trimmed, QStringLiteral("SWIR3"))
        || trimmed.contains(QStringLiteral("SWIR3 NI"), Qt::CaseInsensitive)
        || (trimmed.contains(QStringLiteral("swir3"), Qt::CaseInsensitive)
            && !trimmed.contains(QStringLiteral("fx10e"), Qt::CaseInsensitive)))
        return Channel::Swir;

    if (startsWithInsensitive(trimmed, QStringLiteral("FX10e"))
        || (trimmed.contains(QStringLiteral("fx10e"), Qt::CaseInsensitive)
            && !trimmed.contains(QStringLiteral("swir3"), Qt::CaseInsensitive)))
        return Channel::Fx10e;

    if (trimmed.contains(QStringLiteral("lighthouse"), Qt::CaseInsensitive))
        return Channel::Light;

    if (startsWithInsensitive(trimmed, QStringLiteral("Lumo:"))
        || startsWithInsensitive(trimmed, QStringLiteral("HyperFusion UI"))
        || startsWithInsensitive(trimmed, QStringLiteral("Camera coordinator"))
        || startsWithInsensitive(trimmed, QStringLiteral("Session log"))
        || startsWithInsensitive(trimmed, QStringLiteral("Camera 1:"))
        || startsWithInsensitive(trimmed, QStringLiteral("Camera 2:")))
        return Channel::App;

    return Channel::App;
}

} // namespace hf::log
