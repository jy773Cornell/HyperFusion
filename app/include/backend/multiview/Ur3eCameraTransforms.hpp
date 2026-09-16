// Optical-TCP pose → nerfstudio-style transforms.json (Capture Multiview RGB).
// Backend layer: pure math + file I/O; no hardware side effects.

#pragma once

#include "backend/HyperFusionConfig.hpp"
#include "backend/multiview/Ur3eClient.hpp"
#include "backend/multiview/Ur3eHemisphereScanReachability.hpp"

#include <QString>
#include <QStringList>
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
/// Capture JSON parent is base_link; tip is always BFS camera optical (cfg tool_tcp_*).
struct CameraExtrinsicsRt
{
    double R[3][3]{};
    double t[3]{};
};

/// BFS camera optical in base_link: ``base_T_tool0 · tool0_T_camera`` (cfg ``tool_tcp_*``).
/// Use this for capture JSON even when MoveIt tip ``hyperfusion_tcp`` is remapped to DLP.
[[nodiscard]] Ur3eScanTcpPose cameraOpticalTcpFromTool0(
    const Ur3eTcpPose &tool0,
    const hf::HardwareConfig::Ur3eConfig::ToolTcpMm &cameraTcp);

/// Build OpenGL camera_to_parent from optical TCP (UR rotvec + position).
[[nodiscard]] Mat4 cameraToWorldOpenGlFromTcp(const Ur3eScanTcpPose &tcp);

/// OpenCV camera-to-parent R and t from optical TCP (same pose as transform_matrix before GL flip).
[[nodiscard]] CameraExtrinsicsRt cameraExtrinsicsOpenCvFromTcp(const Ur3eScanTcpPose &tcp);

struct TransformsJsonFrame
{
    QString filePathStem; // e.g. "00000" (no extension)
    Mat4 transformMatrix{};
    CameraExtrinsicsRt extrinsics{};
    int fppStepIndex = -1;
    QString fppStepLabel;
    QString fppPattern;
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
                                       QString *errorMessage = nullptr,
                                       bool appendExisting = false);

/// Flange TF + joints for hand–eye (optional; optical TCP JSON still written).
struct CalibrationCaptureExtras
{
    bool haveFlange = false;
    double flangeX = 0.0;
    double flangeY = 0.0;
    double flangeZ = 0.0;
    double flangeRx = 0.0;
    double flangeRy = 0.0;
    double flangeRz = 0.0;
    QStringList jointNames;
    std::vector<double> jointsRad;
    /// FPP pin burst: step index in kFppScanningSteps, or -1 if not an FPP still.
    int fppStepIndex = -1;
    QString fppStepLabel;
    /// HDMI PSP pattern name shown on the DLP (empty when DLP was not used).
    QString fppPattern;
    /// DLP LED currents (mA) at capture — decode picks R/G/B vs luma from these.
    bool haveDlpLed = false;
    int dlpLedRedMa = 0;
    int dlpLedGreenMa = 0;
    int dlpLedBlueMa = 0;
    /// BFS capture settings at still time (UI / applied). Written as ``bfs_capture``.
    bool haveBfsCapture = false;
    QString bfsCameraId;
    QString bfsExposureMode;
    QString bfsExposureAuto;
    double bfsExposureTimeUs = 0.0;
    QString bfsGainAuto;
    double bfsGainDb = 0.0;
    bool bfsGammaEnable = false;
    double bfsGamma = 0.0;
    QString bfsBalanceWhiteAuto;
    QString bfsBalanceRatioSelector;
    double bfsBalanceRatio = 0.0;
    bool bfsAcquisitionFrameRateEnable = false;
    double bfsAcquisitionFrameRateHz = 0.0;
    int bfsDeviceLinkThroughputLimit = 0;
    double bfsBlackLevelPercent = 0.0;
    double bfsEvCompensation = 0.0;
    /// Apex stills: camera t is expressed at the MVS stage stop (sample treated static).
    bool haveOutputStageShift = false;
    double stageCapturePositionMm = 0.0;
    double stageOutputPositionMm = 0.0;
    double outputShiftXM = 0.0;
    double outputShiftYM = 0.0;
    double outputShiftZM = 0.0;
};

/// Write one pose JSON next to a still (e.g. 00000.json beside 00000.tif).
/// Always emits `base_T_flange` (live TF base_link→tool0) when *calib* has flange;
/// otherwise `base_T_flange: null` and `hand_eye_ready: false`.
/// *tcp* must be BFS camera optical (``tool_tcp_*``), never the DLP MoveIt tip.
[[nodiscard]] bool writeCameraPoseJson(const QString &jsonPath,
                                       const Ur3eScanTcpPose &tcp,
                                       const Mat4 &cameraToWorldOpenGl,
                                       const CameraExtrinsicsRt &extrinsics,
                                       const CameraIntrinsics &intrinsics,
                                       const QString &imageFileName,
                                       const QString &poseSource,
                                       const Ur3eScanTcpPose *plannedTcp = nullptr,
                                       QString *errorMessage = nullptr,
                                       const CalibrationCaptureExtras *calib = nullptr);

} // namespace hf::ur3e
