// Persist last hemisphere scan Plan for auto-load when cfg + UI params match.
// Backend layer: JSON beside app.exe; no hardware side effects.

#pragma once

#include "backend/3dscanning/Ur3eHemisphereScan.hpp"
#include "backend/3dscanning/Ur3eHemisphereScanReachability.hpp"
#include "backend/HyperFusionConfig.hpp"

#include <QString>

namespace hf::ur3e
{

/// Stable fingerprint of robot geometry + scan grid that must match to reload a cached plan.
[[nodiscard]] QString ur3eScanPlanFingerprint(const hf::HardwareConfig::Ur3eConfig &ur3e,
                                              const Ur3eHemisphereScanParams &params);

[[nodiscard]] QString defaultUr3eScanPlanCachePath();

[[nodiscard]] bool saveUr3eScanPlanCache(const QString &path,
                                         const QString &fingerprint,
                                         const Ur3eHemisphereScanPlan &plan,
                                         QString *errorMessage = nullptr);

/// Loads only when the file fingerprint equals *expectedFingerprint*.
[[nodiscard]] bool loadUr3eScanPlanCache(const QString &path,
                                         const QString &expectedFingerprint,
                                         Ur3eHemisphereScanPlan &planOut,
                                         QString *errorMessage = nullptr);

} // namespace hf::ur3e
