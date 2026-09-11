// Tray scan centers + home active-TCP in base_link for apex (backend).
// Apex keeps home XY/orientation and only changes Z to the ring radius.
#include "backend/multiview/Ur3eHemisphereScan.hpp"

#include "backend/HyperFusionConfig.hpp"
#include "backend/multiview/Ur3eMountTransform.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace hf::ur3e
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

struct Mat4
{
    double m[4][4]{};

    static Mat4 identity()
    {
        Mat4 t;
        t.m[0][0] = t.m[1][1] = t.m[2][2] = t.m[3][3] = 1.0;
        return t;
    }

    static Mat4 translate(const double x, const double y, const double z)
    {
        Mat4 t = identity();
        t.m[0][3] = x;
        t.m[1][3] = y;
        t.m[2][3] = z;
        return t;
    }

    static Mat4 fromRpyXyz(const double roll,
                           const double pitch,
                           const double yaw,
                           const double x,
                           const double y,
                           const double z)
    {
        // URDF / tf2: R = Rz(yaw) * Ry(pitch) * Rx(roll)
        const double cr = std::cos(roll);
        const double sr = std::sin(roll);
        const double cp = std::cos(pitch);
        const double sp = std::sin(pitch);
        const double cy = std::cos(yaw);
        const double sy = std::sin(yaw);

        Mat4 t = identity();
        t.m[0][0] = cy * cp;
        t.m[0][1] = cy * sp * sr - sy * cr;
        t.m[0][2] = cy * sp * cr + sy * sr;
        t.m[0][3] = x;
        t.m[1][0] = sy * cp;
        t.m[1][1] = sy * sp * sr + cy * cr;
        t.m[1][2] = sy * sp * cr - cy * sr;
        t.m[1][3] = y;
        t.m[2][0] = -sp;
        t.m[2][1] = cp * sr;
        t.m[2][2] = cp * cr;
        t.m[2][3] = z;
        return t;
    }

    static Mat4 rotZ(const double angle)
    {
        const double c = std::cos(angle);
        const double s = std::sin(angle);
        Mat4 t = identity();
        t.m[0][0] = c;
        t.m[0][1] = -s;
        t.m[1][0] = s;
        t.m[1][1] = c;
        return t;
    }

    Mat4 operator*(const Mat4 &rhs) const
    {
        Mat4 out;
        for (int row = 0; row < 4; ++row)
        {
            for (int col = 0; col < 4; ++col)
            {
                out.m[row][col] = m[row][0] * rhs.m[0][col] + m[row][1] * rhs.m[1][col]
                                  + m[row][2] * rhs.m[2][col] + m[row][3] * rhs.m[3][col];
            }
        }
        return out;
    }

    void transformPoint(double &x, double &y, double &z) const
    {
        const double nx = m[0][0] * x + m[0][1] * y + m[0][2] * z + m[0][3];
        const double ny = m[1][0] * x + m[1][1] * y + m[1][2] * z + m[1][3];
        const double nz = m[2][0] * x + m[2][1] * y + m[2][2] * z + m[2][3];
        x = nx;
        y = ny;
        z = nz;
    }
};

void rotvecFromMat(const Mat4 &t, double &rx, double &ry, double &rz)
{
    const double m00 = t.m[0][0];
    const double m01 = t.m[0][1];
    const double m02 = t.m[0][2];
    const double m10 = t.m[1][0];
    const double m11 = t.m[1][1];
    const double m12 = t.m[1][2];
    const double m20 = t.m[2][0];
    const double m21 = t.m[2][1];
    const double m22 = t.m[2][2];
    const double trace = m00 + m11 + m22;
    const double cosAngle = std::clamp((trace - 1.0) * 0.5, -1.0, 1.0);
    const double angle = std::acos(cosAngle);
    constexpr double kEps = 1.0e-12;
    if (angle <= kEps)
    {
        rx = ry = rz = 0.0;
        return;
    }
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
        if (len <= kEps)
        {
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

// UR3e fixed joint origins from ur_description / HyperFusion materialized URDF.
Mat4 ur3eActiveTcpBaseFromJoints(const std::array<double, 6> &qRad,
                                 const HardwareConfig::Ur3eConfig &cfg)
{
    Mat4 t = Mat4::fromRpyXyz(0.0, 0.0, kPi, 0.0, 0.0, 0.0); // base_link → base_link_inertia
    t = t * Mat4::translate(0.0, 0.0, 0.15185) * Mat4::rotZ(qRad[0]);
    t = t * Mat4::fromRpyXyz(kPi * 0.5, 0.0, 0.0, 0.0, 0.0, 0.0) * Mat4::rotZ(qRad[1]);
    t = t * Mat4::translate(-0.24355, 0.0, 0.0) * Mat4::rotZ(qRad[2]);
    t = t * Mat4::translate(-0.2132, 0.0, 0.13105) * Mat4::rotZ(qRad[3]);
    t = t * Mat4::fromRpyXyz(kPi * 0.5, 0.0, 0.0, 0.0, -0.08535, 0.0) * Mat4::rotZ(qRad[4]);
    t = t
        * Mat4::fromRpyXyz(kPi * 0.5, kPi, kPi, 0.0, 0.0921, 0.0)
        * Mat4::rotZ(qRad[5]);
    // wrist_3 → flange → tool0 (ROS-Industrial)
    t = t * Mat4::fromRpyXyz(0.0, -kPi * 0.5, -kPi * 0.5, 0.0, 0.0, 0.0);
    t = t * Mat4::fromRpyXyz(kPi * 0.5, 0.0, kPi * 0.5, 0.0, 0.0, 0.0);
    const auto tcp = cfg.activeToolTcpMm();
    t = t
        * Mat4::fromRpyXyz(tcp.rollDeg * kPi / 180.0,
                           tcp.pitchDeg * kPi / 180.0,
                           tcp.yawDeg * kPi / 180.0,
                           tcp.xMm * 0.001,
                           tcp.yMm * 0.001,
                           tcp.zMm * 0.001);
    return t;
}

Mat4 ur3eOpticalTcpWorldFromJoints(const std::array<double, 6> &qRad,
                                   const HardwareConfig::Ur3eConfig &cfg)
{
    const double mountRoll = cfg.mountRollDeg * kPi / 180.0;
    const double mountPitch = cfg.mountPitchDeg * kPi / 180.0;
    const double mountYaw = cfg.mountYawDeg * kPi / 180.0;
    const double mountX = cfg.mountOffsetXMm * 0.001;
    const double mountY = cfg.mountOffsetYMm * 0.001;
    const double mountZ = cfg.ceilingMountHeightMm * 0.001;

    Mat4 t = Mat4::fromRpyXyz(mountRoll, mountPitch, mountYaw, mountX, mountY, mountZ);
    return t * ur3eActiveTcpBaseFromJoints(qRad, cfg);
}

void inverseSceneMountXy(double &xM, double &yM, const Ur3eMountTransform &mount)
{
    // Forward mount: yaw→pitch then +offset. Undo offset, then inverse pitch/yaw on XY (z=0).
    xM -= mount.offsetXM;
    yM -= mount.offsetYM;
    if (std::abs(mount.yawRad) <= 1.0e-12 && std::abs(mount.pitchRad) <= 1.0e-12)
        return;

    const double cy = std::cos(-mount.yawRad);
    const double sy = std::sin(-mount.yawRad);
    const double cp = std::cos(-mount.pitchRad);
    const double sp = std::sin(-mount.pitchRad);

    const double x1 = cp * xM - sp * 0.0;
    const double y1 = yM;
    const double z1 = sp * xM + cp * 0.0;
    xM = cy * x1 - sy * y1;
    yM = sy * x1 + cy * y1;
    (void)z1;
}

std::array<double, 6> homeJointsRad(const HardwareConfig::Ur3eConfig &cfg)
{
    std::array<double, 6> qRad{};
    for (int i = 0; i < 6; ++i)
        qRad[static_cast<std::size_t>(i)] =
            cfg.homeJointsDeg[static_cast<std::size_t>(i)] * kPi / 180.0;
    return qRad;
}

void fillHomeActiveTcpBase(const Mat4 &t,
                           double &xM,
                           double &yM,
                           double &zM,
                           double &rx,
                           double &ry,
                           double &rz,
                           double &toolZX,
                           double &toolZY,
                           double &toolZZ)
{
    xM = t.m[0][3];
    yM = t.m[1][3];
    zM = t.m[2][3];
    toolZX = t.m[0][2];
    toolZY = t.m[1][2];
    toolZZ = t.m[2][2];
    rotvecFromMat(t, rx, ry, rz);
}

void homeOpticalTcpTrayXyM(double &xM, double &yM)
{
    const auto &cfg = hf::hardwareConfig().ur3e;
    const Mat4 tcpBase = ur3eActiveTcpBaseFromJoints(homeJointsRad(cfg), cfg);
    xM = tcpBase.m[0][3];
    yM = tcpBase.m[1][3];
}
} // namespace

void scanCenterOffsetM(double &xM, double &yM)
{
    // Ring dome / look-at: under base_link when mount_offset_x/y_mm = 0.
    xM = 0.0;
    yM = 0.0;
}

void homeTcpScanCenterOffsetM(double &xM, double &yM)
{
    homeOpticalTcpTrayXyM(xM, yM);
}

void homeActiveTcpBaseLink(double &xM, double &yM, double &zM,
                           double &rx, double &ry, double &rz,
                           double &toolZX, double &toolZY, double &toolZZ)
{
    const auto &cfg = hf::hardwareConfig().ur3e;
    fillHomeActiveTcpBase(ur3eActiveTcpBaseFromJoints(homeJointsRad(cfg), cfg),
                          xM, yM, zM, rx, ry, rz, toolZX, toolZY, toolZZ);
}

void homeApexCameraUpWorld(double &x, double &y, double &z)
{
    const auto &cfg = hf::hardwareConfig().ur3e;
    const Mat4 t = ur3eActiveTcpBaseFromJoints(homeJointsRad(cfg), cfg);
    // OpenCV image-up ≈ tool −Y.
    x = -t.m[0][1];
    y = -t.m[1][1];
    z = -t.m[2][1];
}

void homeOpticalTcpOrientation(double &rx, double &ry, double &rz,
                               double &toolZX, double &toolZY, double &toolZZ)
{
    double xM = 0.0;
    double yM = 0.0;
    double zM = 0.0;
    homeActiveTcpBaseLink(xM, yM, zM, rx, ry, rz, toolZX, toolZY, toolZZ);
}

} // namespace hf::ur3e
