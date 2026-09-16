// Auto hemisphere UR3e scan execute (MoveIt pins + optional wrist sweep).
// Kept in a separate TU from Ur3eSemiFixedScanExecute. Session start remains
// Ur3ePanelController::startHemisphereScanExecute (auto branch) so Capture/BFS
// hooks stay intact; this module owns mode helpers + readiness checks.

#pragma once

#include "backend/multiview/Ur3eHemisphereScanReachability.hpp"

#include <QString>

namespace hf::ur3e
{

enum class Ur3eScanExecuteMode
{
    AutoHemisphere = 0,
    SemiFixed = 1,
    Fpp = 2,
};

[[nodiscard]] inline QString ur3eScanExecuteModeLabel(const Ur3eScanExecuteMode mode)
{
    switch (mode)
    {
    case Ur3eScanExecuteMode::SemiFixed:
        return QStringLiteral("Semi-fixed");
    case Ur3eScanExecuteMode::Fpp:
        return QStringLiteral("FPP");
    case Ur3eScanExecuteMode::AutoHemisphere:
    default:
        return QStringLiteral("Auto planning");
    }
}

/// Semi-fixed and FPP execute saved ring plans under ``mvs_scan_plans/semi`` and
/// ``mvs_scan_plans/fpp`` respectively.
[[nodiscard]] inline bool isSavedRingRouteMode(const Ur3eScanExecuteMode mode)
{
    return mode == Ur3eScanExecuteMode::SemiFixed || mode == Ur3eScanExecuteMode::Fpp;
}

[[nodiscard]] inline bool autoHemisphereScanPlanReady(const Ur3eHemisphereScanPlan &plan)
{
    return plan.reachableCount > 0;
}

} // namespace hf::ur3e
