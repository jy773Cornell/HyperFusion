// Optical-TCP pose → nerfstudio-style transforms.json (Capture 3D RGB).
// Backend layer: pure math + file I/O; no hardware side effects.

#pragma once

#include "backend/3dscanning/Ur3eHemisphereScanReachability.hpp"

#include <QString>
#include <array>
#include <vector>

namespace hf::ur3e
{

/// 4×4 row-major camera_to_world (OpenGL / nerfstudio).
using Mat4 = std::array<double, 16>;

/// Build OpenGL camera_to_world from optical TCP (UR rotvec + position).
[[nodiscard]] Mat4 cameraToWorldOpenGlFromTcp(const Ur3eScanTcpPose &tcp);

struct TransformsJsonFrame
{
    QString filePathStem; // e.g. "00000" (no extension)
    Mat4 transformMatrix{};
};

struct TransformsJsonDocument
{
    int width = 0;
    int height = 0;
    std::vector<TransformsJsonFrame> frames;
};

/// Write nerfstudio-compatible transforms.json beside the TIFF frames.
[[nodiscard]] bool writeTransformsJson(const QString &directory,
                                       const TransformsJsonDocument &doc,
                                       QString *errorMessage = nullptr);

/// Write one pose JSON next to a still (e.g. 00000.json beside 00000.tif).
[[nodiscard]] bool writeCameraPoseJson(const QString &jsonPath,
                                       const Ur3eScanTcpPose &tcp,
                                       const Mat4 &cameraToWorldOpenGl,
                                       const QString &imageFileName,
                                       int width,
                                       int height,
                                       QString *errorMessage = nullptr);

} // namespace hf::ur3e
