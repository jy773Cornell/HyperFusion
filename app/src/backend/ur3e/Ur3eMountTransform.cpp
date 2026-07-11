// Stage / scene alignment from hyperfusion.cfg mount keys (backend layer).
#include "backend/ur3e/Ur3eMountTransform.hpp"

#include "backend/ur3e/Ur3eHemisphereScanReachability.hpp"

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
    const double angle = std::acos(std::clamp((trace - 1.0) * 0.5, -1.0, 1.0));
    if (angle <= kEpsilon)
    {
        rx = ry = rz = 0.0;
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

    double toolZX = tcp.toolZMx;
    double toolZY = tcp.toolZMy;
    double toolZZ = tcp.toolZMz;
    if (!normalizeVector(toolZX, toolZY, toolZZ))
    {
        toolZX = 0.0;
        toolZY = 0.0;
        toolZZ = -1.0;
    }

    tcp.toolZMx = toolZX;
    tcp.toolZMy = toolZY;
    tcp.toolZMz = toolZZ;

    double refX = std::abs(toolZX) < 0.9 ? 1.0 : 0.0;
    double refY = std::abs(toolZX) < 0.9 ? 0.0 : 1.0;
    const double refZ = 0.0;

    double yX = toolZY * refZ - toolZZ * refY;
    double yY = toolZZ * refX - toolZX * refZ;
    double yZ = toolZX * refY - toolZY * refX;
    normalizeVector(yX, yY, yZ);

    const double xX = yY * toolZZ - yZ * toolZY;
    const double xY = yZ * toolZX - yX * toolZZ;
    const double xZ = yX * toolZY - yY * toolZX;

    rotationMatrixToRotVec(xX, yX, toolZX, xY, yY, toolZY, xZ, yZ, toolZZ, tcp.rxRad, tcp.ryRad,
                           tcp.rzRad);
}

} // namespace hf::ur3e
