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
    /// Scan grids always include one extra apex pin (θ=0, look-down) regardless of thetaMinDeg.
    double thetaMinDeg = 30.0;
    double thetaMaxDeg = 90.0;
};

/// Per-pin wrist_1/2/3 photo grid used at Execute (not part of MoveIt Plan).
struct Ur3eWristSweepParams
{
    bool enabled = true;
    double stepDeg = 3.0;
    int stepsEachWay = 4;
    bool wrist1 = false;
    bool wrist2 = true;
    bool wrist3 = true;

    [[nodiscard]] int enabledAxisCount() const
    {
        return (wrist1 ? 1 : 0) + (wrist2 ? 1 : 0) + (wrist3 ? 1 : 0);
    }

    /// Center + non-zero offset product on enabled wrists: (2N)^k + 1.
    [[nodiscard]] int imagesPerPin() const
    {
        if (!enabled)
            return 1;
        const int axes = enabledAxisCount();
        if (axes <= 0 || stepsEachWay <= 0 || !(stepDeg > 0.0))
            return 1;
        const int offsets = 2 * stepsEachWay;
        int product = 1;
        for (int i = 0; i < axes; ++i)
            product *= offsets;
        return product + 1;
    }
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
/// Tray-frame scan / look-at center (metres): perpendicular projection of the
/// optical TCP at *home_joints_deg* onto the sample-tray surface.
void scanCenterOffsetM(double &xM, double &yM);
/// Grid pin count including the always-present apex (θ=0) pin.
[[nodiscard]] int hemisphereScanPointCount(const Ur3eHemisphereScanParams &params);
/// Latitude/longitude grid plus a fixed apex pin first. Apex TCP aims at dome center
/// (perpendicular look-down); camera-up / TCP upper face locked to world +X.
/// Other pins use scan_camera_up_world_z. Dome centered on home-TCP tray projection.
[[nodiscard]] std::vector<Ur3eHemisphereScanPoint>
generateHemisphereScanPoints(const Ur3eHemisphereScanParams &params);

} // namespace hf::ur3e
