// Ceiling-mounted UR3e workspace boundary cube (backend layer).
#pragma once

#include "backend/HyperFusionConfig.hpp"

#include "backend/3dscanning/Ur3eHemisphereScan.hpp"

#include <vector>

namespace hf::ur3e
{

/// Sample tray footprint (metres), centred at origin on the workspace floor.
constexpr double kSampleTrayLengthM = 0.54;
constexpr double kSampleTrayWidthM = 0.49;
constexpr double kSampleTrayHeightM = 0.02;

struct Ur3eWorkspaceBoundary
{
    bool enabled = true;
    /// X extent (mm), tray centered at origin.
    double lengthMm = 1200.0;
    /// Y extent (mm), tray centered at origin.
    double widthMm = 1200.0;
    /// Vertical extent (mm) below the mount plane (MoveIt collision box depth).
    double heightMm = 1000.0;
    /// Robot mount plane height in world frame (mm). Tray/sample stage stays at Z=0.
    double mountHeightMm = 1000.0;

    void normalize();
    [[nodiscard]] double lengthM() const;
    [[nodiscard]] double widthM() const;
    [[nodiscard]] double heightM() const;
    [[nodiscard]] double mountHeightM() const;
    [[nodiscard]] double floorZM() const;
    [[nodiscard]] double topZM() const;
    [[nodiscard]] double halfLengthM() const;
    [[nodiscard]] double halfWidthM() const;
    [[nodiscard]] bool containsPointM(double xM, double yM, double zM) const;
};

[[nodiscard]] Ur3eWorkspaceBoundary workspaceBoundaryFromConfig(
    const HardwareConfig::Ur3eConfig &config);

/// Largest hemisphere radius (m) that fits inside the workspace cube and sample tray.
[[nodiscard]] double maxHemisphereRadiusM(const Ur3eWorkspaceBoundary &boundary);
void clampHemisphereScanParamsToBoundary(Ur3eHemisphereScanParams &params,
                                         const Ur3eWorkspaceBoundary &boundary);

[[nodiscard]] int countScanPointsOutsideBoundary(
    const Ur3eWorkspaceBoundary &boundary,
    const std::vector<Ur3eHemisphereScanPoint> &points);

} // namespace hf::ur3e
