// Hemisphere scan grid for UR3e sample-tray dome (backend layer).
#pragma once

#include <vector>

namespace hf::ur3e
{

struct Ur3eHemisphereScanParams
{
    double sphereRadiusM = 0.50;
    int horizontalPoints = 12;
    int verticalPoints = 5;
    /// Polar angle from dome apex (0°) to equator (90°). Hemisphere only — clamped to 0–90°.
    double thetaMinDeg = 30.0;
    double thetaMaxDeg = 90.0;
};

struct Ur3eHemisphereScanPoint
{
    double phiDeg = 0.0;
    double thetaDeg = 0.0;
    double xM = 0.0;
    double yM = 0.0;
    double zM = 0.0;
};

void normalizeHemisphereScanParams(Ur3eHemisphereScanParams &params);
[[nodiscard]] int hemisphereScanPointCount(const Ur3eHemisphereScanParams &params);
[[nodiscard]] std::vector<Ur3eHemisphereScanPoint>
generateHemisphereScanPoints(const Ur3eHemisphereScanParams &params);

} // namespace hf::ur3e
