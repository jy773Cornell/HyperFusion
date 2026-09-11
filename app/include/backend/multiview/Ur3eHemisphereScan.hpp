// Hemisphere scan grid for UR3e sample-tray dome (backend layer).
#pragma once

#include <vector>

namespace hf::ur3e
{

/// Legacy fallback only. Semi and Auto apex Z = the ring sphere radius.
constexpr double kSemiFixedApexRadiusM = 0.200;

struct Ur3eHemisphereScanParams
{
    double sphereRadiusM = 0.50;
    int horizontalPoints = 12;
    int verticalPoints = 5;
    /// Polar angle from dome apex (0°) to equator (90°). Hemisphere only — clamped to 0–90°.
    /// Scan grids always include one extra apex pin (θ=0) regardless of thetaMinDeg.
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
/// Ring-dome / look-at center on the tray (metres). Fixed at (0,0) — under base_link
/// when mount_offset_x/y_mm = 0 (Semi pans about the base).
void scanCenterOffsetM(double &xM, double &yM);
/// Home active-TCP XY in base_link (same frame as ring / GET /pose).
void homeTcpScanCenterOffsetM(double &xM, double &yM);
/// Home active-TCP image-up in base_link (tool −Y).
void homeApexCameraUpWorld(double &x, double &y, double &z);
/// Home active-TCP in base_link: XY + orientation. Z is the live home height.
void homeActiveTcpBaseLink(double &xM, double &yM, double &zM,
                           double &rx, double &ry, double &rz,
                           double &toolZX, double &toolZY, double &toolZZ);
/// Home active-TCP orientation in base_link.
void homeOpticalTcpOrientation(double &rx, double &ry, double &rz,
                               double &toolZX, double &toolZY, double &toolZZ);
/// Grid pin count including the always-present apex (θ=0) pin.
[[nodiscard]] int hemisphereScanPointCount(const Ur3eHemisphereScanParams &params);
/// Latitude/longitude grid plus a fixed apex pin first.
/// Apex = home XY and home orientation, Z = ring radius R (base_link).
/// Rings orbit the scan-center on scan_tcp; image-up → world −Z.
[[nodiscard]] std::vector<Ur3eHemisphereScanPoint>
generateHemisphereScanPoints(const Ur3eHemisphereScanParams &params);
/// Semi Plan grid: same θ layers + apex, but *searchCandidates* φ samples evenly over 360°.
/// Apex Z = params.sphereRadiusM (same sphere as the ring).
[[nodiscard]] std::vector<Ur3eHemisphereScanPoint>
generateSemiHemisphereScanPoints(const Ur3eHemisphereScanParams &params,
                                 int searchCandidates);

} // namespace hf::ur3e
