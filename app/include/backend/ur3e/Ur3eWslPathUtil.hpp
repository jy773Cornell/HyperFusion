// Windows ↔ WSL path helpers for UR3e sidecar integration.
#pragma once

#include "backend/HyperFusionConfig.hpp"

#include <QString>

namespace hf::ur3e
{
QString resolveUr3eResourcesWindowsPath();
QString resolveUr3eRepoLinuxPath();
QString buildMountEnvExports(const hf::HardwareConfig::Ur3eConfig &cfg);
QString buildToolPayloadEnvExports(const hf::HardwareConfig::Ur3eConfig &cfg);

} // namespace hf::ur3e
