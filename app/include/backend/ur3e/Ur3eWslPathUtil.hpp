// Windows ↔ WSL path helpers for UR3e sidecar integration.
#pragma once

#include <QString>

namespace hf::ur3e
{
QString resolveUr3eResourcesWindowsPath();
QString resolveUr3eRepoLinuxPath();

} // namespace hf::ur3e
