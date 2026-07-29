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

/// OpenCV camera intrinsics for multiview JSON (pixels).
struct CameraIntrinsics
{
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    int width = 0;
    int height = 0;
    /// Brown-Conrady coeffs (typically k1,k2,p1,p2,k3). Empty = undistorted / unknown.
    std::vector<double> distortion;
};

/// OpenCV camera-to-parent rotation R (3×3 row-major) and translation t (metres).
/// Capture JSON parent is base_link (optical tip hyperfusion_tcp).
struct CameraExtrinsicsRt
{
    double R[3][3]{};
    double t[3]{};
};

/// Build OpenGL camera_to_parent from optical TCP (UR rotvec + position).
[[nodiscard]] Mat4 cameraToWorldOpenGlFromTcp(const Ur3eScanTcpPose &tcp);

/// OpenCV camera-to-parent R and t from optical TCP (same pose as transform_matrix before GL flip).
[[nodiscard]] CameraExtrinsicsRt cameraExtrinsicsOpenCvFromTcp(const Ur3eScanTcpPose &tcp);

struct TransformsJsonFrame
{
    QString filePathStem; // e.g. "00000" (no extension)
    Mat4 transformMatrix{};
    CameraExtrinsicsRt extrinsics{};
};

struct TransformsJsonDocument
{
    CameraIntrinsics intrinsics{};
    std::vector<TransformsJsonFrame> frames;

    int width() const { return intrinsics.width; }
    int height() const { return intrinsics.height; }
};

/// Write nerfstudio-compatible transforms.json beside the TIFF frames.
[[nodiscard]] bool writeTransformsJson(const QString &directory,
                                       const TransformsJsonDocument &doc,
                                       QString *errorMessage = nullptr);

/// Write one pose JSON next to a still (e.g. 00000.json beside 00000.tif).
/// *tcp* is the pose used for extrinsics (optical TCP in base_link).
/// *plannedTcp* is the MoveIt world pin target (audit only).
[[nodiscard]] bool writeCameraPoseJson(const QString &jsonPath,
                                       const Ur3eScanTcpPose &tcp,
                                       const Mat4 &cameraToWorldOpenGl,
                                       const CameraExtrinsicsRt &extrinsics,
                                       const CameraIntrinsics &intrinsics,
                                       const QString &imageFileName,
                                       const QString &poseSource,
                                       const Ur3eScanTcpPose *plannedTcp = nullptr,
                                       QString *errorMessage = nullptr);

} // namespace hf::ur3e
