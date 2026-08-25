// Sanitizes WSL subprocess log text for HyperFusion UI (backend/multiview layer).
#pragma once

#include <QByteArray>
#include <QString>

namespace hf::ur3e
{
QString sanitizeWslProcessOutput(const QByteArray &raw);
QString stripAnsiEscapes(QString text);
} // namespace hf::ur3e
