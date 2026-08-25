// Stage / scene alignment from hyperfusion.cfg mount keys (backend layer).
#include "backend/multiview/Ur3eMountTransform.hpp"

#include "backend/multiview/Ur3eHemisphereScanReachability.hpp"

#include <algorithm>
#include <cmath>

namespace hf::ur3e
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1.0e-9;

double degToRad(const double degrees)
{
    return degrees * kPi / 180.0;
}

bool normalizeVector(double &x, double &y, double &z)
{
    const double length = std::sqrt(x * x + y * y + z * z);
    if (length <= kEpsilon)
        return false;
    x /= length;
    y /= length;
    z /= length;
    return true;
}

void rotationMatrixToRotVec(const double m00,
                              const double m01,
                              const double m02,
                              const double m10,
                              const double m11,
                              const double m12,
                              const double m20,
                              const double m21,
                              const double m22,
                              double &rx,
                              double &ry,
                              double &rz)
{
    const double trace = m00 + m11 + m22;
    const double cosAngle = std::clamp((trace - 1.0) * 0.5, -1.0, 1.0);
    const double angle = std::acos(cosAngle);
    if (angle <= kEpsilon)
    {
        rx = ry = rz = 0.0;
        return;
    }

    // Near 180°: (R − Rᵀ)/(2 sinθ) is unstable — use diagonal of R.
    if (angle > kPi - 1.0e-6 || std::abs(std::sin(angle)) < 1.0e-6)
    {
        double ax = std::sqrt(std::max(0.0, (m00 + 1.0) * 0.5));
        double ay = std::sqrt(std::max(0.0, (m11 + 1.0) * 0.5));
        double az = std::sqrt(std::max(0.0, (m22 + 1.0) * 0.5));
        if (ax >= ay && ax >= az)
        {
            ay = std::copysign(ay, m10 + m01);
            az = std::copysign(az, m20 + m02);
        }
        else if (ay >= az)
        {
            ax = std::copysign(ax, m10 + m01);
            az = std::copysign(az, m21 + m12);
        }
        else
        {
            ax = std::copysign(ax, m20 + m02);
            ay = std::copysign(ay, m21 + m12);
        }
        const double len = std::sqrt(ax * ax + ay * ay + az * az);
        if (len <= kEpsilon)
        {
            // Fallback: rotate 180° about X (covers look-down tool +Z = −world Z).
            rx = kPi;
            ry = 0.0;
            rz = 0.0;
            return;
        }
        rx = ax / len * kPi;
        ry = ay / len * kPi;
        rz = az / len * kPi;
        return;
    }

    const double sinAngle = std::sin(angle);
    rx = (m21 - m12) / (2.0 * sinAngle) * angle;
    ry = (m02 - m20) / (2.0 * sinAngle) * angle;
    rz = (m10 - m01) / (2.0 * sinAngle) * angle;
}

void applyYawPitch(const double yawRad,
                   const double pitchRad,
                   const double xIn,
                   const double yIn,
                   const double zIn,
                   double &xOut,
                   double &yOut,
                   double &zOut)
{
    const double cy = std::cos(yawRad);
    const double sy = std::sin(yawRad);
    const double cp = std::cos(pitchRad);
    const double sp = std::sin(pitchRad);

    const double xYaw = cy * xIn - sy * yIn;
    const double yYaw = sy * xIn + cy * yIn;
    const double zYaw = zIn;

    xOut = cp * xYaw + sp * zYaw;
    yOut = yYaw;
    zOut = -sp * xYaw + cp * zYaw;
}
} // namespace

Ur3eMountTransform Ur3eMountTransform::sceneAlignFromConfig(
    const hf::HardwareConfig::Ur3eConfig &cfg)
{
    Ur3eMountTransform mount;
    mount.yawRad = degToRad(cfg.mountYawDeg);
    mount.pitchRad = degToRad(cfg.mountPitchDeg);
    mount.offsetXM = cfg.mountOffsetXMm / 1000.0;
    mount.offsetYM = cfg.mountOffsetYMm / 1000.0;
    return mount;
}

bool Ur3eMountTransform::isIdentity() const
{
    return std::abs(yawRad) <= kEpsilon && std::abs(pitchRad) <= kEpsilon
           && std::abs(offsetXM) <= kEpsilon && std::abs(offsetYM) <= kEpsilon;
}

void Ur3eMountTransform::transformPoint(double &xM, double &yM, double &zM) const
{
    if (isIdentity())
        return;

    double xRot = 0.0;
    double yRot = 0.0;
    double zRot = 0.0;
    applyYawPitch(yawRad, pitchRad, xM, yM, zM, xRot, yRot, zRot);
    xM = xRot + offsetXM;
    yM = yRot + offsetYM;
    zM = zRot;
}

void Ur3eMountTransform::transformVector(double &xM, double &yM, double &zM) const
{
    if (isIdentity())
        return;

    double xRot = 0.0;
    double yRot = 0.0;
    double zRot = 0.0;
    applyYawPitch(yawRad, pitchRad, xM, yM, zM, xRot, yRot, zRot);
    xM = xRot;
    yM = yRot;
    zM = zRot;
}

void Ur3eMountTransform::transformTcpPose(Ur3eScanTcpPose &tcp) const
{
    if (isIdentity())
        return;

    transformPoint(tcp.xM, tcp.yM, tcp.zM);
    transformVector(tcp.toolZMx, tcp.toolZMy, tcp.toolZMz);

    double upX = 0.0;
    double upY = 0.0;
    double upZ = 1.0;
    transformVector(upX, upY, upZ);
    orientScanTcpFromToolZ(tcp,
                           hf::hardwareConfig().ur3e.scanCameraUpWorldZ,
                           upX,
                           upY,
                           upZ);
}

void orientScanTcpFromToolZ(Ur3eScanTcpPose &tcp,
                            const bool lockCameraUpWorldZ,
                            double upX,
                            double upY,
                            double upZ)
{
    double zx = tcp.toolZMx;
    double zy = tcp.toolZMy;
    double zz = tcp.toolZMz;
    if (!normalizeVector(zx, zy, zz))
    {
        zx = 0.0;
        zy = 0.0;
        zz = -1.0;
    }
    tcp.toolZMx = zx;
    tcp.toolZMy = zy;
    tcp.toolZMz = zz;

    double yX = 0.0;
    double yY = 0.0;
    double yZ = 0.0;

    if (lockCameraUpWorldZ)
    {
        if (!normalizeVector(upX, upY, upZ))
        {
            upX = 0.0;
            upY = 0.0;
            upZ = 1.0;
        }
        // Project tray/world up onto plane ⊥ look-at; OpenCV image-up = −tool Y.
        const double upDotZ = upX * zx + upY * zy + upZ * zz;
        double px = upX - upDotZ * zx;
        double py = upY - upDotZ * zy;
        double pz = upZ - upDotZ * zz;
        if (!normalizeVector(px, py, pz))
        {
            // Look-at ≈ ±up: fall back to world X/Y heuristic.
            const double refX = std::abs(zx) < 0.9 ? 1.0 : 0.0;
            const double refY = std::abs(zx) < 0.9 ? 0.0 : 1.0;
            yX = zy * 0.0 - zz * refY;
            yY = zz * refX - zx * 0.0;
            yZ = zx * refY - zy * refX;
            normalizeVector(yX, yY, yZ);
        }
        else
        {
            yX = -px;
            yY = -py;
            yZ = -pz;
        }
    }
    else
    {
        const double refX = std::abs(zx) < 0.9 ? 1.0 : 0.0;
        const double refY = std::abs(zx) < 0.9 ? 0.0 : 1.0;
        yX = zy * 0.0 - zz * refY;
        yY = zz * refX - zx * 0.0;
        yZ = zx * refY - zy * refX;
        normalizeVector(yX, yY, yZ);
    }

    const double xX = yY * zz - yZ * zy;
    const double xY = yZ * zx - yX * zz;
    const double xZ = yX * zy - yY * zx;

    rotationMatrixToRotVec(xX, yX, zx, xY, yY, zy, xZ, yZ, zz, tcp.rxRad, tcp.ryRad, tcp.rzRad);
}

} // namespace hf::ur3e
