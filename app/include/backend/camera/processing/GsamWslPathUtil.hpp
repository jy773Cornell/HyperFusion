// Windows ↔ WSL path helpers for GSAM2 sidecar integration.
#pragma once

#include <QString>

namespace hf::processing
{
QString windowsPathToWsl(const QString &windowsPath);
QString resolveSam2ResourcesWindowsPath();
QString resolveSam2RepoLinuxPath();

} // namespace hf::processing
