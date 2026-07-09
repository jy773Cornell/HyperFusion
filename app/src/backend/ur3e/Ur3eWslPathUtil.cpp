// Windows ↔ WSL path helpers for UR3e sidecar integration.
#include "backend/ur3e/Ur3eWslPathUtil.hpp"

#include "backend/HyperFusionConfig.hpp"
#include "backend/camera/processing/GsamWslPathUtil.hpp"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>

namespace hf::ur3e
{
QString resolveUr3eResourcesWindowsPath()
{
    QDir dir(QCoreApplication::applicationDirPath());
    for (int depth = 0; depth < 8; ++depth)
    {
        const QString candidate = dir.filePath(QStringLiteral("resources/ur3e"));
        if (QFileInfo::exists(candidate))
            return QFileInfo(candidate).absoluteFilePath();

        const QString alt = dir.filePath(QStringLiteral("../resources/ur3e"));
        if (QFileInfo::exists(alt))
            return QFileInfo(alt).absoluteFilePath();

        if (!dir.cdUp())
            break;
    }

    return {};
}

QString resolveUr3eRepoLinuxPath()
{
    const hf::HardwareConfig::Ur3eConfig &cfg = hf::hardwareConfig().ur3e;
    if (!cfg.ur3eRepoLinux.trimmed().isEmpty())
        return cfg.ur3eRepoLinux.trimmed();

    const QString windowsPath = resolveUr3eResourcesWindowsPath();
    if (windowsPath.isEmpty())
        return {};

    return hf::processing::windowsPathToWsl(windowsPath);
}

} // namespace hf::ur3e
