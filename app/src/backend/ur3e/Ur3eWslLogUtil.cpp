// Sanitizes WSL subprocess log text for HyperFusion UI (backend/ur3e layer).
#include "backend/ur3e/Ur3eWslLogUtil.hpp"

#include <QRegularExpression>

namespace hf::ur3e
{
namespace
{
QRegularExpression ansiEscapePattern()
{
    static const QRegularExpression pattern(
        QStringLiteral("\\x1B(?:[@-Z\\\\\\-_]|\\[[0-?]*[ -/]*[@-~])"));
    return pattern;
}

QRegularExpression orphanSgrPattern()
{
    static const QRegularExpression pattern(QStringLiteral("\\[[0-9;]*m"));
    return pattern;
}
} // namespace

QString stripAnsiEscapes(QString text)
{
    text.remove(ansiEscapePattern());
    text.remove(orphanSgrPattern());
    return text.trimmed();
}

QString sanitizeWslProcessOutput(const QByteArray &raw)
{
    QString text = QString::fromUtf8(raw);
    text.replace(QStringLiteral("\r\n"), QStringLiteral("\n"));
    text.replace(QLatin1Char('\r'), QLatin1Char('\n'));
    return stripAnsiEscapes(text);
}

} // namespace hf::ur3e
