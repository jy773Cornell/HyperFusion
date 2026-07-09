// Application log channels and message routing for the tabbed log panel.
#pragma once

#include <QString>

namespace hf::log
{
enum class Channel
{
    App = 0,
    Fx10e,
    Swir,
    Stage,
    Light,
    Ur3e,
    Capture,
};

constexpr int channelCount()
{
    return 7;
}

QString channelLabel(Channel channel);
QString channelTabTitle(Channel channel);

/// Lowercase tag used in session log files, e.g. [fx10e].
QString channelFileTag(Channel channel);

/// Route a log line to the correct tab based on message prefix / keywords.
Channel classifyMessage(const QString &message);

} // namespace hf::log
