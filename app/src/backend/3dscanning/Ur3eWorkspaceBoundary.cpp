// Ceiling-mounted UR3e workspace boundary cube (backend layer).
#include "backend/3dscanning/Ur3eWorkspaceBoundary.hpp"

#include "backend/3dscanning/Ur3eHemisphereScan.hpp"

#include <algorithm>

namespace hf::ur3e
{
void Ur3eWorkspaceBoundary::normalize()
{
    lengthMm = std::max(100.0, lengthMm);
    widthMm = std::max(100.0, widthMm);
    heightMm = std::max(100.0, heightMm);
    mountHeightMm = std::max(100.0, mountHeightMm);
    ceilingClearanceMm = std::max(0.0, ceilingClearanceMm);
    // Keep a usable vertical span between floor and inset top.
    if (ceilingClearanceMm >= heightMm)
        ceilingClearanceMm = std::max(0.0, heightMm - 10.0);
}

double Ur3eWorkspaceBoundary::lengthM() const { return lengthMm * 0.001; }
double Ur3eWorkspaceBoundary::widthM() const { return widthMm * 0.001; }
double Ur3eWorkspaceBoundary::heightM() const { return heightMm * 0.001; }
double Ur3eWorkspaceBoundary::mountHeightM() const { return mountHeightMm * 0.001; }
double Ur3eWorkspaceBoundary::ceilingClearanceM() const { return ceilingClearanceMm * 0.001; }
double Ur3eWorkspaceBoundary::floorZM() const
{
    return std::max(0.0, mountHeightM() - heightM());
}
double Ur3eWorkspaceBoundary::topZM() const
{
    return std::max(floorZM(), mountHeightM() - ceilingClearanceM());
}
double Ur3eWorkspaceBoundary::halfLengthM() const { return lengthM() * 0.5; }
double Ur3eWorkspaceBoundary::halfWidthM() const { return widthM() * 0.5; }

bool Ur3eWorkspaceBoundary::containsPointM(const double xM, const double yM, const double zM) const
{
    if (!enabled)
        return true;
    return std::abs(xM) <= halfLengthM() && std::abs(yM) <= halfWidthM() && zM >= floorZM()
           && zM <= topZM();
}

Ur3eWorkspaceBoundary workspaceBoundaryFromConfig(const HardwareConfig::Ur3eConfig &config)
{
    Ur3eWorkspaceBoundary boundary;
    boundary.enabled = config.workspaceBoundaryEnabled;
    boundary.lengthMm = config.workspaceLengthMm;
    boundary.widthMm = config.workspaceWidthMm;
    boundary.heightMm = config.workspaceHeightMm;
    boundary.mountHeightMm = config.ceilingMountHeightMm;
    boundary.ceilingClearanceMm = config.workspaceCeilingClearanceMm;
    boundary.normalize();
    return boundary;
}

double maxHemisphereRadiusM(const Ur3eWorkspaceBoundary &boundary)
{
    if (!boundary.enabled)
        return 5.0;

    const double boundaryHorizontal =
        std::min(boundary.halfLengthM(), boundary.halfWidthM());
    const double trayHorizontal =
        std::min(kSampleTrayLengthM * 0.5, kSampleTrayWidthM * 0.5);
    const double horizontalLimit = std::min(boundaryHorizontal, trayHorizontal);
    const double verticalLimit =
        std::max(0.01, boundary.mountHeightM() - kSampleTrayHeightM);
    return std::max(0.01, std::min(horizontalLimit, verticalLimit));
}

void clampHemisphereScanParamsToBoundary(Ur3eHemisphereScanParams &params,
                                         const Ur3eWorkspaceBoundary &boundary)
{
    normalizeHemisphereScanParams(params);
    if (!boundary.enabled)
        return;

    const double maxRadius = maxHemisphereRadiusM(boundary);
    params.sphereRadiusM = std::min(params.sphereRadiusM, maxRadius);
}

int countScanPointsOutsideBoundary(
    const Ur3eWorkspaceBoundary &boundary,
    const std::vector<Ur3eHemisphereScanPoint> &points)
{
    if (!boundary.enabled)
        return 0;

    int outsideCount = 0;
    for (const Ur3eHemisphereScanPoint &point : points)
    {
        if (!boundary.containsPointM(point.xM, point.yM, point.zM + kSampleTrayHeightM))
            ++outsideCount;
    }
    return outsideCount;
}

} // namespace hf::ur3e
