// Windows ↔ WSL path helpers for GSAM2 sidecar integration.
#include "backend/camera/processing/GsamWslPathUtil.hpp"

#include "backend/HyperFusionConfig.hpp"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>

namespace hf::processing
{
QString windowsPathToWsl(const QString &windowsPath)
{
    const QString native = QDir::fromNativeSeparators(windowsPath.trimmed());
    if (native.size() >= 2 && native.at(1) == QLatin1Char(':'))
    {
        const QChar drive = native.at(0).toLower();
        QString rest = native.mid(2);
        while (rest.startsWith(QLatin1Char('/')))
            rest = rest.mid(1);
        return QStringLiteral("/mnt/%1/%2").arg(drive, rest);
    }

    return native;
}

QString resolveSam2ResourcesWindowsPath()
{
    QDir dir(QCoreApplication::applicationDirPath());
    for (int depth = 0; depth < 8; ++depth)
    {
        const QString candidate = dir.filePath(QStringLiteral("resources/gsam2"));
        if (QFileInfo::exists(candidate))
            return QFileInfo(candidate).absoluteFilePath();

        const QString alt = dir.filePath(QStringLiteral("../resources/gsam2"));
        if (QFileInfo::exists(alt))
            return QFileInfo(alt).absoluteFilePath();

        if (!dir.cdUp())
            break;
    }

    return {};
}

QString resolveSam2RepoLinuxPath()
{
    const hf::HardwareConfig::SegmentationConfig &cfg = hf::hardwareConfig().segmentation;
    if (!cfg.sam2RepoLinux.trimmed().isEmpty())
        return cfg.sam2RepoLinux.trimmed();

    const QString windowsPath = resolveSam2ResourcesWindowsPath();
    if (windowsPath.isEmpty())
        return {};

    return windowsPathToWsl(windowsPath);
}

} // namespace hf::processing
