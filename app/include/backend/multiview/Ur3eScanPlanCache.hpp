// Persist hemisphere scan plans: last-plan cache + named routes library.
// Backend layer: JSON beside app.exe; load only when robot cfg fingerprint matches.

#pragma once

#include "backend/multiview/Ur3eHemisphereScan.hpp"
#include "backend/multiview/Ur3eHemisphereScanReachability.hpp"
#include "backend/HyperFusionConfig.hpp"

#include <QString>
#include <QVector>

namespace hf::ur3e
{

struct Ur3eScanRouteInfo
{
    QString id;          ///< File stem / stable id
    QString displayName; ///< UI label
    QString path;        ///< Absolute JSON path
    QString fingerprint; ///< Full cfg + scan-grid fingerprint
    QString robotCfgFingerprint;
    Ur3eHemisphereScanParams params{};
    int pointCount = 0;
    int reachableCount = 0;
};

/// Full fingerprint: robot geometry + scan grid (must match to reload last-plan cache).
[[nodiscard]] QString ur3eScanPlanFingerprint(const hf::HardwareConfig::Ur3eConfig &ur3e,
                                              const Ur3eHemisphereScanParams &params);

/// Robot / hyperfusion.cfg geometry only — used to list loadable named routes.
/// Ignores use_mock_hardware so simulation plans load on the real robot.
[[nodiscard]] QString ur3eScanRobotCfgFingerprint(const hf::HardwareConfig::Ur3eConfig &ur3e);

[[nodiscard]] QString defaultUr3eScanPlanCachePath();
/// Root: ``mvs_scan_plans`` beside app.exe (from preset).
[[nodiscard]] QString defaultUr3eMvsScanPlansRootDir();
/// Auto planning library: ``mvs_scan_plans/auto``.
[[nodiscard]] QString defaultUr3eScanRoutesDir();
/// Semi-fixed library: ``mvs_scan_plans/semi``.
[[nodiscard]] QString defaultUr3eSemiScanPlansDir();
/// FPP library: ``mvs_scan_plans/fpp``.
[[nodiscard]] QString defaultUr3eFppScanPlansDir();

[[nodiscard]] QString defaultUr3eScanRouteDisplayName(const Ur3eHemisphereScanParams &params);

[[nodiscard]] bool saveUr3eScanPlanCache(const QString &path,
                                         const QString &fingerprint,
                                         const Ur3eHemisphereScanPlan &plan,
                                         QString *errorMessage = nullptr);

/// Loads only when the file fingerprint equals *expectedFingerprint*.
[[nodiscard]] bool loadUr3eScanPlanCache(const QString &path,
                                         const QString &expectedFingerprint,
                                         Ur3eHemisphereScanPlan &planOut,
                                         QString *errorMessage = nullptr);

/// Save a named route (robot-cfg gated library entry).
[[nodiscard]] bool saveUr3eNamedScanRoute(const QString &path,
                                          const QString &displayName,
                                          const QString &fingerprint,
                                          const QString &robotCfgFingerprint,
                                          const Ur3eHemisphereScanParams &params,
                                          const Ur3eHemisphereScanPlan &plan,
                                          QString *errorMessage = nullptr);

/// List routes whose robot_cfg_fingerprint matches *expectedRobotCfgFingerprint*.
[[nodiscard]] QVector<Ur3eScanRouteInfo>
listUr3eScanRoutesMatchingCfg(const QString &dir,
                              const QString &expectedRobotCfgFingerprint);

/// Load route plan; requires robot cfg match. Optionally returns stored scan params.
[[nodiscard]] bool loadUr3eNamedScanRoute(const QString &path,
                                          const QString &expectedRobotCfgFingerprint,
                                          Ur3eHemisphereScanPlan &planOut,
                                          Ur3eHemisphereScanParams *paramsOut = nullptr,
                                          QString *displayNameOut = nullptr,
                                          QString *errorMessage = nullptr);

} // namespace hf::ur3e
