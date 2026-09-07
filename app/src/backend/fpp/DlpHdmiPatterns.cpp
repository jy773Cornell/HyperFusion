// HDMI PSP pattern directory lookup (backend/fpp). Walks up from app.exe to calibration.
#include "backend/fpp/DlpHdmiPatterns.hpp"

#include "backend/HyperFusionConfig.hpp"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>

namespace hf::dlp
{
QString resolveHdmiPspPatternDir()
{
    const QString configured = hf::hardwareConfig().dlp.hdmiPatternDir.trimmed();
    if (!configured.isEmpty())
    {
        const QFileInfo info(configured);
        if (info.isDir())
            return info.absoluteFilePath();
    }

    QDir dir(QCoreApplication::applicationDirPath());
    for (int depth = 0; depth < 8; ++depth)
    {
        const QString candidate = dir.filePath(QStringLiteral("calibration/multiview/patterns/psp"));
        if (QFileInfo::exists(QDir(candidate).filePath(QStringLiteral("sine_1_0.png"))))
            return QFileInfo(candidate).absoluteFilePath();
        if (!dir.cdUp())
            break;
    }
    return {};
}

QString hdmiPspPatternFile(const char *fileName)
{
    if (fileName == nullptr || fileName[0] == '\0')
        return {};
    const QString dir = resolveHdmiPspPatternDir();
    if (dir.isEmpty())
        return {};
    return QDir(dir).filePath(QString::fromUtf8(fileName));
}
} // namespace hf::dlp
