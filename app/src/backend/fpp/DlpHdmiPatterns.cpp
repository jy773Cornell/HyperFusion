// HDMI PSP pattern directory lookup (backend/fpp). Walks up from app.exe to calibration.
#include "backend/fpp/DlpHdmiPatterns.hpp"

#include "backend/HyperFusionConfig.hpp"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>

namespace hf::dlp
{
namespace
{
bool looksLikePspDir(const QString &candidate)
{
    return QFileInfo::exists(QDir(candidate).filePath(QStringLiteral("sine_1_0.png")));
}
} // namespace

QString resolveHdmiPspPatternDir()
{
    const QString configured = hf::hardwareConfig().dlp.hdmiPatternDir.trimmed();
    if (!configured.isEmpty())
    {
        const QFileInfo info(configured);
        if (info.isDir())
            return info.absoluteFilePath();
    }

    const QStringList relativeCandidates = {
        QStringLiteral("calibration/multiview/dlp_cal/patterns/psp"),
        QStringLiteral("calibration/multiview/fpp_cal/patterns/psp"), // legacy name
    };

    QDir dir(QCoreApplication::applicationDirPath());
    for (int depth = 0; depth < 8; ++depth)
    {
        for (const QString &rel : relativeCandidates)
        {
            const QString candidate = dir.filePath(rel);
            if (looksLikePspDir(candidate))
                return QFileInfo(candidate).absoluteFilePath();
        }
        if (!dir.cdUp())
            break;
    }

#ifdef HF_APP_SOURCE_DIR
    {
        const QDir src(QString::fromUtf8(HF_APP_SOURCE_DIR));
        for (const QString &rel : relativeCandidates)
        {
            const QString candidate = src.filePath(rel);
            if (looksLikePspDir(candidate))
                return QFileInfo(candidate).absoluteFilePath();
        }
    }
#endif

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
