// Tray scan centers: base XY for ring dome; home optical-TCP XY for apex (backend).
// FK matches HyperFusion URDF chain (ceiling mount + UR3e + hyperfusion_tcp).
#include "backend/multiview/Ur3eHemisphereScan.hpp"

#include "backend/HyperFusionConfig.hpp"
#include "backend/multiview/Ur3eMountTransform.hpp"

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

// UR3e fixed joint origins from ur_description / HyperFusion materialized URDF.
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
    t = t * Mat4::fromRpyXyz(0.0, 0.0, kPi, 0.0, 0.0, 0.0); // base_link → base_link_inertia
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
    // tool0 → hyperfusion_tcp (URDF rpy: Rz(yaw) * Ry(pitch) * Rx(roll))
    t = t
        * Mat4::fromRpyXyz(cfg.toolTcpRollDeg * kPi / 180.0,
                           cfg.toolTcpPitchDeg * kPi / 180.0,
                           cfg.toolTcpYawDeg * kPi / 180.0,
                           cfg.toolTcpXMm * 0.001,
                           cfg.toolTcpYMm * 0.001,
                           cfg.toolTcpZMm * 0.001);
    return t;
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

void homeOpticalTcpTrayXyM(double &xM, double &yM)
{
    const auto &cfg = hf::hardwareConfig().ur3e;
    std::array<double, 6> qRad{};
    for (int i = 0; i < 6; ++i)
        qRad[static_cast<std::size_t>(i)] =
            cfg.homeJointsDeg[static_cast<std::size_t>(i)] * kPi / 180.0;

    const Mat4 tcpWorld = ur3eOpticalTcpWorldFromJoints(qRad, cfg);
    double wx = 0.0;
    double wy = 0.0;
    double wz = 0.0;
    tcpWorld.transformPoint(wx, wy, wz);

    // Perpendicular projection onto the sample-tray surface: keep world XY.
    xM = wx;
    yM = wy;

    // Grid is authored in pre-mount tray frame; C++ remounts poses for MoveIt.
    inverseSceneMountXy(xM, yM, Ur3eMountTransform::sceneAlignFromConfig(cfg));
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
    // Apex only: optical TCP XY at home_joints_deg (reachable look-down locus).
    homeOpticalTcpTrayXyM(xM, yM);
}

} // namespace hf::ur3e
