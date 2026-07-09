// Hemisphere scan grid for UR3e sample-tray dome (backend layer).
#include "backend/ur3e/Ur3eHemisphereScan.hpp"

#include <algorithm>
#include <cmath>

namespace hf::ur3e
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kHemisphereMaxThetaDeg = 90.0;

double clampThetaDeg(const double degrees)
{
    return std::clamp(degrees, 0.0, kHemisphereMaxThetaDeg);
}

std::vector<double> linspaceInclusive(const double start, const double end, const int count)
{
    std::vector<double> values;
    if (count <= 0)
        return values;
    values.reserve(static_cast<std::size_t>(count));
    if (count == 1)
    {
        values.push_back(start);
        return values;
    }

    const double step = (end - start) / static_cast<double>(count - 1);
    for (int index = 0; index < count; ++index)
        values.push_back(start + step * static_cast<double>(index));
    return values;
}
} // namespace

void normalizeHemisphereScanParams(Ur3eHemisphereScanParams &params)
{
    params.sphereRadiusM = std::max(0.01, params.sphereRadiusM);
    params.horizontalPoints = std::max(1, params.horizontalPoints);
    params.verticalPoints = std::max(1, params.verticalPoints);
    params.thetaMinDeg = clampThetaDeg(params.thetaMinDeg);
    params.thetaMaxDeg = clampThetaDeg(params.thetaMaxDeg);
    if (params.thetaMinDeg > params.thetaMaxDeg)
        std::swap(params.thetaMinDeg, params.thetaMaxDeg);
}

int hemisphereScanPointCount(const Ur3eHemisphereScanParams &params)
{
    Ur3eHemisphereScanParams normalized = params;
    normalizeHemisphereScanParams(normalized);
    return normalized.horizontalPoints * normalized.verticalPoints;
}

std::vector<Ur3eHemisphereScanPoint>
generateHemisphereScanPoints(const Ur3eHemisphereScanParams &params)
{
    Ur3eHemisphereScanParams normalized = params;
    normalizeHemisphereScanParams(normalized);

    const std::vector<double> thetaSamples =
        linspaceInclusive(normalized.thetaMinDeg, normalized.thetaMaxDeg, normalized.verticalPoints);

    std::vector<Ur3eHemisphereScanPoint> points;
    points.reserve(static_cast<std::size_t>(normalized.horizontalPoints * normalized.verticalPoints));

    for (const double thetaDeg : thetaSamples)
    {
        for (int horizontalIndex = 0; horizontalIndex < normalized.horizontalPoints; ++horizontalIndex)
        {
            const double phiDeg =
                360.0 * static_cast<double>(horizontalIndex) / static_cast<double>(normalized.horizontalPoints);
            const double thetaRad = thetaDeg * kPi / 180.0;
            const double phiRad = phiDeg * kPi / 180.0;
            const double sinTheta = std::sin(thetaRad);

            Ur3eHemisphereScanPoint point;
            point.phiDeg = phiDeg;
            point.thetaDeg = thetaDeg;
            point.xM = normalized.sphereRadiusM * sinTheta * std::cos(phiRad);
            point.yM = normalized.sphereRadiusM * sinTheta * std::sin(phiRad);
            point.zM = normalized.sphereRadiusM * std::cos(thetaRad);
            points.push_back(point);
        }
    }

    return points;
}

} // namespace hf::ur3e
