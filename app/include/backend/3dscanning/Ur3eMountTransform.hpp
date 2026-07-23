// Stage / scene alignment from hyperfusion.cfg mount keys (backend layer).
// Roll is robot ceiling-mount only; yaw, pitch, and XY offset rotate the sample stage preview
// and hemisphere pin targets to match the physical install vs RViz.
#pragma once

#include "backend/HyperFusionConfig.hpp"

namespace hf::ur3e
{

struct Ur3eScanTcpPose;

struct Ur3eMountTransform
{
    double yawRad = 0.0;
    double pitchRad = 0.0;
    double offsetXM = 0.0;
    double offsetYM = 0.0;

    [[nodiscard]] static Ur3eMountTransform sceneAlignFromConfig(
        const hf::HardwareConfig::Ur3eConfig &cfg);

    [[nodiscard]] bool isIdentity() const;

    void transformPoint(double &xM, double &yM, double &zM) const;
    void transformVector(double &xM, double &yM, double &zM) const;
    void transformTcpPose(Ur3eScanTcpPose &tcp) const;
};

} // namespace hf::ur3e
