// Hemisphere scan grid for UR3e sample-tray dome (backend layer).
#include "backend/3dscanning/Ur3eHemisphereScan.hpp"

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

bool isApexThetaDeg(const double thetaDeg)
{
    return std::abs(thetaDeg) <= 1e-9;
}

Ur3eHemisphereScanPoint makeApexScanPoint(const double sphereRadiusM,
                                          const double centerXM,
                                          const double centerYM)
{
    Ur3eHemisphereScanPoint apex;
    apex.phiDeg = 0.0;
    apex.thetaDeg = 0.0;
    apex.xM = centerXM;
    apex.yM = centerYM;
    apex.zM = sphereRadiusM;
    return apex;
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

    // Always one apex pin (θ=0). If the vertical grid already includes θ=0, that
    // ring collapses to the single apex instead of H coincident pins.
    if (isApexThetaDeg(normalized.thetaMinDeg))
        return 1 + std::max(0, normalized.verticalPoints - 1) * normalized.horizontalPoints;
    return 1 + normalized.horizontalPoints * normalized.verticalPoints;
}

std::vector<Ur3eHemisphereScanPoint>
generateHemisphereScanPoints(const Ur3eHemisphereScanParams &params)
{
    Ur3eHemisphereScanParams normalized = params;
    normalizeHemisphereScanParams(normalized);

    double centerXM = 0.0;
    double centerYM = 0.0;
    scanCenterOffsetM(centerXM, centerYM);

    double apexXM = 0.0;
    double apexYM = 0.0;
    homeTcpScanCenterOffsetM(apexXM, apexYM);

    const std::vector<double> thetaSamples =
        linspaceInclusive(normalized.thetaMinDeg, normalized.thetaMaxDeg, normalized.verticalPoints);

    std::vector<Ur3eHemisphereScanPoint> points;
    points.reserve(static_cast<std::size_t>(hemisphereScanPointCount(normalized)));

    // Apex over home TCP XY (reachable look-down); rings orbit base XY.
    points.push_back(makeApexScanPoint(normalized.sphereRadiusM, apexXM, apexYM));

    for (const double thetaDeg : thetaSamples)
    {
        if (isApexThetaDeg(thetaDeg))
            continue;

        for (int horizontalIndex = 0; horizontalIndex < normalized.horizontalPoints; ++horizontalIndex)
        {
            const double phiDeg =
                360.0 * static_cast<double>(horizontalIndex)
                / static_cast<double>(normalized.horizontalPoints);
            const double thetaRad = thetaDeg * kPi / 180.0;
            const double phiRad = phiDeg * kPi / 180.0;
            const double sinTheta = std::sin(thetaRad);

            Ur3eHemisphereScanPoint point;
            point.phiDeg = phiDeg;
            point.thetaDeg = thetaDeg;
            point.xM = centerXM + normalized.sphereRadiusM * sinTheta * std::cos(phiRad);
            point.yM = centerYM + normalized.sphereRadiusM * sinTheta * std::sin(phiRad);
            point.zM = normalized.sphereRadiusM * std::cos(thetaRad);
            points.push_back(point);
        }
    }

    return points;
}

std::vector<Ur3eHemisphereScanPoint>
generateSemiHemisphereScanPoints(const Ur3eHemisphereScanParams &params,
                                 const int searchCandidates)
{
    Ur3eHemisphereScanParams normalized = params;
    normalizeHemisphereScanParams(normalized);

    const int nPhi = std::max(
        1, std::max(normalized.horizontalPoints, std::clamp(searchCandidates, 1, 720)));

    double centerXM = 0.0;
    double centerYM = 0.0;
    scanCenterOffsetM(centerXM, centerYM);

    double apexXM = 0.0;
    double apexYM = 0.0;
    homeTcpScanCenterOffsetM(apexXM, apexYM);

    const std::vector<double> thetaSamples =
        linspaceInclusive(normalized.thetaMinDeg, normalized.thetaMaxDeg, normalized.verticalPoints);

    std::vector<Ur3eHemisphereScanPoint> points;
    points.reserve(static_cast<std::size_t>(1 + nPhi * normalized.verticalPoints));

    points.push_back(makeApexScanPoint(normalized.sphereRadiusM, apexXM, apexYM));

    for (const double thetaDeg : thetaSamples)
    {
        if (isApexThetaDeg(thetaDeg))
            continue;

        for (int horizontalIndex = 0; horizontalIndex < nPhi; ++horizontalIndex)
        {
            const double phiDeg =
                360.0 * static_cast<double>(horizontalIndex) / static_cast<double>(nPhi);
            const double thetaRad = thetaDeg * kPi / 180.0;
            const double phiRad = phiDeg * kPi / 180.0;
            const double sinTheta = std::sin(thetaRad);

            Ur3eHemisphereScanPoint point;
            point.phiDeg = phiDeg;
            point.thetaDeg = thetaDeg;
            point.xM = centerXM + normalized.sphereRadiusM * sinTheta * std::cos(phiRad);
            point.yM = centerYM + normalized.sphereRadiusM * sinTheta * std::sin(phiRad);
            point.zM = normalized.sphereRadiusM * std::cos(thetaRad);
            points.push_back(point);
        }
    }

    return points;
}

} // namespace hf::ur3e
