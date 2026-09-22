// UR3e tab orchestration implementation.
#include "frontend/controllers/Ur3ePanelController.hpp"

#include "backend/HyperFusionConfig.hpp"
#include "backend/multiview/Ur3eClient.hpp"
#include "backend/multiview/Ur3eHemisphereScan.hpp"
#include "backend/multiview/Ur3eHemisphereScanReachability.hpp"
#include "backend/multiview/Ur3eMountTransform.hpp"
#include "backend/multiview/Ur3eWorkspaceBoundary.hpp"
#include "backend/multiview/Ur3eMoveItManager.hpp"
#include "backend/multiview/Ur3eRvizManager.hpp"
#include "backend/multiview/Ur3eServerManager.hpp"
#include "backend/multiview/Ur3eWslSetup.hpp"
#include "backend/multiview/Ur3eCameraTransforms.hpp"
#include "backend/multiview/Ur3eScanPlanCache.hpp"
#include "backend/multiview/Ur3eAutoHemisphereScanExecute.hpp"
#include "backend/multiview/Ur3eSemiFixedScan.hpp"
#include "backend/multiview/Ur3eSemiFixedScanExecute.hpp"
#include "backend/multiview/BfsTiffIo.hpp"
#include "frontend/controllers/BfsPanelController.hpp"
#include "frontend/controllers/DlpPanelController.hpp"
#include "backend/fpp/DlpTypes.hpp"
#include "backend/stage/StageWorker.hpp"
#include "frontend/widgets/MainWindow.hpp"
#include "frontend/widgets/Ur3eExternalControlWaitDialog.hpp"
#include "frontend/widgets/Ur3eHemisphereScanSettingsWidget.hpp"
#include "frontend/widgets/Ur3eJointBarWidget.hpp"
#include "frontend/settings/AppSettingsStore.hpp"
#include "frontend/widgets/Ur3eScanRoutePlanWidget.hpp"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QFileDialog>
#include <QLineEdit>
#include <QDateTime>
#include <QMessageBox>
#include <QAbstractButton>
#include <QMetaObject>
#include <QPushButton>
#include <QThread>
#include <QTimer>
#include <QHash>
#include <QJsonDocument>
#include <QJsonObject>

#include <QMetaObject>
#include <QVariant>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

namespace hf::ur3e
{
namespace
{
constexpr int kUr3eJointCount = 6;
constexpr const char *kUr3eJointNames[kUr3eJointCount] = {
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
};

/// Wrist_2 / wrist_3 offsets in radians: −N…−1, +1…+N × step (center is separate).
std::vector<double> wristSweepOffsetsRad(const int stepsEachWay, const double stepDeg)
{
    std::vector<double> offsets;
    if (stepsEachWay <= 0 || !(stepDeg > 0.0))
        return offsets;
    const double stepRad = stepDeg * (3.14159265358979323846 / 180.0);
    offsets.reserve(static_cast<std::size_t>(2 * stepsEachWay));
    for (int i = stepsEachWay; i >= 1; --i)
        offsets.push_back(-static_cast<double>(i) * stepRad);
    for (int i = 1; i <= stepsEachWay; ++i)
        offsets.push_back(static_cast<double>(i) * stepRad);
    return offsets;
}

bool sameElbowFamily(const std::vector<double> &a, const std::vector<double> &b)
{
    if (a.size() < 3 || b.size() < 3)
        return true;
    const double ea = a[2];
    const double eb = b[2];
    constexpr double kNearZeroRad = 5.0 * 3.14159265358979323846 / 180.0;
    if (std::abs(ea) < kNearZeroRad || std::abs(eb) < kNearZeroRad)
        return true;
    return (ea * eb) > 0.0;
}

Ur3eScanTcpPose scanTcpFromLivePose(const Ur3eTcpPose &live, const Ur3eScanTcpPose &fallback)
{
    Ur3eScanTcpPose tcp = fallback;
    tcp.xM = live.x;
    tcp.yM = live.y;
    tcp.zM = live.z;
    tcp.rxRad = live.rx;
    tcp.ryRad = live.ry;
    tcp.rzRad = live.rz;
    // Optical +Z from live rotvec (OpenCV camera forward) — keep aligned with extrinsics R.
    const CameraExtrinsicsRt ext = cameraExtrinsicsOpenCvFromTcp(tcp);
    tcp.toolZMx = ext.R[0][2];
    tcp.toolZMy = ext.R[1][2];
    tcp.toolZMz = ext.R[2][2];
    return tcp;
}

CalibrationCaptureExtras extrasFromLivePose(const Ur3ePoseResult &live)
{
    CalibrationCaptureExtras extras;
    if (live.hasTool0)
    {
        extras.haveFlange = true;
        extras.flangeX = live.tool0.x;
        extras.flangeY = live.tool0.y;
        extras.flangeZ = live.tool0.z;
        extras.flangeRx = live.tool0.rx;
        extras.flangeRy = live.tool0.ry;
        extras.flangeRz = live.tool0.rz;
    }
    extras.jointNames = live.jointNames;
    extras.jointsRad = live.jointsRad;
    return extras;
}

void fillBfsCaptureExtras(CalibrationCaptureExtras &calib, MainWindow *host)
{
    if (host == nullptr || host->bfsPanel() == nullptr)
        return;
    const hf::bfs::BfsCameraSettings s = host->bfsPanel()->settingsFromUi();
    calib.haveBfsCapture = true;
    calib.bfsCameraId = s.cameraId;
    calib.bfsExposureMode = s.exposureMode;
    calib.bfsExposureAuto = s.exposureAuto;
    calib.bfsExposureTimeUs = s.exposureTimeUs;
    calib.bfsGainAuto = s.gainAuto;
    calib.bfsGainDb = s.gainDb;
    calib.bfsGammaEnable = s.gammaEnable;
    calib.bfsGamma = s.gamma;
    calib.bfsBalanceWhiteAuto = s.balanceWhiteAuto;
    calib.bfsBalanceRatioSelector = s.balanceRatioSelector;
    calib.bfsBalanceRatio = s.balanceRatio;
    calib.bfsAcquisitionFrameRateEnable = s.acquisitionFrameRateEnable;
    calib.bfsAcquisitionFrameRateHz = s.acquisitionFrameRateHz;
    calib.bfsDeviceLinkThroughputLimit = s.deviceLinkThroughputLimit;
    calib.bfsBlackLevelPercent = s.blackLevelPercent;
    calib.bfsEvCompensation = s.evCompensation;
}

/// Capture JSON / transforms always use BFS camera optical (tool_tcp_*), never the
/// MoveIt tip when scan_tcp=dlp remaps hyperfusion_tcp to the projector.
[[nodiscard]] bool resolveCameraOpticalTcp(const Ur3ePoseResult &live,
                                           const Ur3eScanTcpPose &fallback,
                                           Ur3eScanTcpPose *out,
                                           QString *poseSource,
                                           QString *warnMessage)
{
    if (out == nullptr)
        return false;
    const auto &ur3e = hf::hardwareConfig().ur3e;
    if (live.ok && live.hasTool0)
    {
        *out = cameraOpticalTcpFromTool0(live.tool0, ur3e.cameraToolTcpMm());
        if (poseSource != nullptr)
            *poseSource = QStringLiteral("live_tf_base_tool0_x_camera_tcp");
        return true;
    }
    if (live.ok && !ur3e.usesDlpScanTcp())
    {
        // hyperfusion_tcp == camera when scan_tcp=camera.
        *out = scanTcpFromLivePose(live.pose, fallback);
        if (poseSource != nullptr)
            *poseSource = QStringLiteral("live_tf_base_hyperfusion_tcp");
        if (warnMessage != nullptr)
        {
            *warnMessage = QStringLiteral(
                "live tool0 TF missing — using hyperfusion_tcp (scan_tcp=camera).");
        }
        return true;
    }
    if (live.ok && ur3e.usesDlpScanTcp())
    {
        // Live tip is DLP — cannot use it for camera depth / RGB poses.
        *out = fallback;
        if (poseSource != nullptr)
            *poseSource = QStringLiteral("planned_world_fallback");
        if (warnMessage != nullptr)
        {
            *warnMessage = QStringLiteral(
                "live tool0 TF missing while scan_tcp=dlp — camera optical unknown; "
                "wrote planned MoveIt tip (may be DLP). Reconnect so /pose includes tool0.");
        }
        return false;
    }
    *out = fallback;
    if (poseSource != nullptr)
        *poseSource = QStringLiteral("planned_world_fallback");
    if (warnMessage != nullptr)
        *warnMessage = QStringLiteral("live TCP unavailable — using planned pose.");
    return false;
}

struct OutputPoseShift
{
    bool apply = false;
    double xM = 0.0;
    double yM = 0.0;
    double zM = 0.0;
    double stageCaptureMm = 0.0;
    double stageOutputMm = 0.0;
};

[[nodiscard]] bool stageConnectedForScan(MainWindow *host)
{
    if (host == nullptr)
        return false;
    StageWorker *worker = host->stageWorker();
    return worker != nullptr && worker->currentState() == StageState::Connected;
}

/// Blocking absolute move. Caller only invokes this when the stage was connected at execute start.
[[nodiscard]] bool waitMoveStageAbsolute(MainWindow *host,
                                         const double targetMm,
                                         const QString &label,
                                         const std::function<bool()> &stillOk,
                                         QString *errorMessage)
{
    StageWorker *worker = host != nullptr ? host->stageWorker() : nullptr;
    if (worker == nullptr || worker->currentState() != StageState::Connected)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage =
                QStringLiteral("Stage disconnected during Multiview stage move (%1).").arg(label);
        }
        return false;
    }

    double speed = hf::hardwareConfig().operationScanningSpeedMmPerSec;
    if (!(speed > 0.0))
        speed = 80.0;

    if (host != nullptr)
    {
        QMetaObject::invokeMethod(
            host,
            [host, targetMm, label]() {
                host->appendLog(QStringLiteral("UR3e scan: stage → %1 (%2 mm)…")
                                    .arg(label)
                                    .arg(targetMm, 0, 'f', 1));
            },
            Qt::QueuedConnection);
    }

    std::mutex mu;
    std::condition_variable cv;
    bool done = false;
    bool ok = false;
    worker->requestMoveAbsoluteMm(targetMm, speed, true, [&](const bool success) {
        std::lock_guard<std::mutex> lock(mu);
        ok = success;
        done = true;
        cv.notify_one();
    });

    while (true)
    {
        {
            std::unique_lock<std::mutex> lock(mu);
            if (cv.wait_for(lock, std::chrono::milliseconds(50), [&]() { return done; }))
                break;
        }
        if (!stillOk || !stillOk())
        {
            worker->requestStopMotion();
            std::unique_lock<std::mutex> lock(mu);
            cv.wait_for(lock, std::chrono::seconds(30), [&]() { return done; });
            if (errorMessage != nullptr)
                *errorMessage = QStringLiteral("Stage move aborted (%1).").arg(label);
            return false;
        }
    }

    if (!ok && errorMessage != nullptr)
    {
        *errorMessage =
            QStringLiteral("Stage move to %1 mm failed (%2).").arg(targetMm, 0, 'f', 1).arg(label);
    }
    return ok;
}

/// Stay at the scan stage stop after execute (do not home the stage sensor).
void parkStageAtMvsAfterExecuteIfUsed(MainWindow *host,
                                      const bool useStage,
                                      const std::function<bool()> &stillOk,
                                      const double parkMm = -1.0)
{
    if (!useStage)
        return;
    const double targetMm =
        parkMm >= 0.0 ? parkMm : hf::hardwareConfig().sampleMultiviewPositionMm;
    QString stageErr;
    if (!waitMoveStageAbsolute(host,
                               targetMm,
                               QStringLiteral("MVS rings"),
                               stillOk,
                               &stageErr)
        && host != nullptr && !stageErr.isEmpty())
    {
        QMetaObject::invokeMethod(
            host,
            [host, stageErr]() {
                host->appendLog(QStringLiteral("UR3e scan warning: %1").arg(stageErr));
            },
            Qt::QueuedConnection);
    }
}

[[nodiscard]] OutputPoseShift makeApexStageOutputShift()
{
    // Legacy helper (unused for single-stage scans).
    OutputPoseShift shift;
    const auto &hw = hf::hardwareConfig();
    if (!hw.sampleMultiviewTwoStage())
        return shift;
    shift.apply = true;
    hw.sampleMultiviewApexOutputShiftM(shift.xM, shift.yM, shift.zM);
    shift.stageCaptureMm = hw.sampleMultiviewApexPositionMm;
    shift.stageOutputMm = hw.sampleMultiviewPositionMm;
    return shift;
}

/// Same axis convention as cfg, but with plan-overridden stage stops (FPP).
[[nodiscard]] OutputPoseShift makeStageOutputShiftMm(const double captureMm, const double outputMm)
{
    OutputPoseShift shift;
    if (std::abs(outputMm - captureMm) <= 0.5)
        return shift;
    shift.apply = true;
    shift.stageCaptureMm = captureMm;
    shift.stageOutputMm = outputMm;
    const double dM = (outputMm - captureMm) * 0.001;
    switch (hf::hardwareConfig().sampleMultiviewStageAxis)
    {
    case hf::HardwareConfig::SampleMultiviewStageAxis::PosX:
        shift.xM = dM;
        break;
    case hf::HardwareConfig::SampleMultiviewStageAxis::NegX:
        shift.xM = -dM;
        break;
    case hf::HardwareConfig::SampleMultiviewStageAxis::PosY:
        shift.yM = dM;
        break;
    case hf::HardwareConfig::SampleMultiviewStageAxis::NegY:
        shift.yM = -dM;
        break;
    }
    return shift;
}

/// One BFS TIFF + pose JSON. *fppStepIndex* < 0 = single still (DLP off).
bool saveOneBfsStillAtPin(Ur3ePanelController *controller,
                          MainWindow *host,
                          const QString &serverUrl,
                          const QString &captureDir,
                          const Ur3eScanTcpPose &plannedTcp,
                          int *captured,
                          TransformsJsonDocument *transformsDoc,
                          bool *ok,
                          QString *errorMessage,
                          const QString &skipContext,
                          const int fppStepIndex,
                          const QString &fppStepLabel,
                          const OutputPoseShift &outputShift = {})
{
    hf::bfs::BfsRgbFrame frame;
    bool gotFrame = false;
    QMetaObject::invokeMethod(
        controller,
        [host, &frame, &gotFrame]() {
            if (host->bfsPanel() != nullptr)
                gotFrame = host->bfsPanel()->tryCopyLastFrame(frame);
        },
        Qt::BlockingQueuedConnection);

    if (!gotFrame)
    {
        QMetaObject::invokeMethod(
            controller,
            [host, skipContext, fppStepLabel]() {
                if (fppStepLabel.isEmpty())
                {
                    host->appendLog(
                        QStringLiteral("UR3e scan capture: no BFS frame at %1 — skipping still.")
                            .arg(skipContext));
                }
                else
                {
                    host->appendLog(
                        QStringLiteral(
                            "UR3e scan capture: no BFS frame at %1 (%2) — skipping still.")
                            .arg(skipContext, fppStepLabel));
                }
            },
            Qt::QueuedConnection);
        return true;
    }

    Ur3eScanTcpPose tcpForPose = plannedTcp;
    QString poseSource = QStringLiteral("planned_world_fallback");
    const Ur3ePoseResult livePose = ur3eGetTcpPose(serverUrl);
    QString opticalWarn;
    const bool opticalOk =
        resolveCameraOpticalTcp(livePose, plannedTcp, &tcpForPose, &poseSource, &opticalWarn);
    if (!opticalOk && hf::hardwareConfig().ur3e.usesDlpScanTcp())
    {
        if (ok != nullptr)
            *ok = false;
        if (errorMessage != nullptr)
        {
            *errorMessage =
                QStringLiteral(
                    "Camera optical pose unavailable at %1 while scan_tcp=dlp "
                    "(%2). Need live /pose tool0.")
                    .arg(skipContext,
                         opticalWarn.isEmpty() ? QStringLiteral("no tool0 TF") : opticalWarn);
        }
        return false;
    }
    if (!opticalOk)
    {
        QMetaObject::invokeMethod(
            controller,
            [host, skipContext, opticalWarn]() {
                host->appendLog(
                    QStringLiteral("UR3e scan capture: camera optical unresolved at %1 — %2")
                        .arg(skipContext, opticalWarn));
            },
            Qt::QueuedConnection);
    }
    else if (!opticalWarn.isEmpty())
    {
        QMetaObject::invokeMethod(
            controller,
            [host, skipContext, opticalWarn]() {
                host->appendLog(
                    QStringLiteral("UR3e scan capture: %1 (%2)").arg(opticalWarn, skipContext));
            },
            Qt::QueuedConnection);
    }

    const QString stem = QStringLiteral("%1").arg(*captured, 5, 10, QLatin1Char('0'));
    const QString tiffPath = QDir(captureDir).filePath(stem + QStringLiteral(".tif"));
    const std::string saveError = hf::bfs::saveRgb8AsTiff(
        tiffPath, frame.width, frame.height, frame.rgb.data(), frame.rgb.size());
    if (!saveError.empty())
    {
        if (ok != nullptr)
            *ok = false;
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("Failed to save BFS TIFF %1: %2")
                                .arg(tiffPath, QString::fromStdString(saveError));
        }
        return false;
    }

    if (outputShift.apply)
    {
        tcpForPose.xM += outputShift.xM;
        tcpForPose.yM += outputShift.yM;
        tcpForPose.zM += outputShift.zM;
    }

    const auto &ur3eCfg = hf::hardwareConfig().ur3e;
    CameraIntrinsics intrinsics;
    intrinsics.fx = ur3eCfg.bfsCameraFx;
    intrinsics.fy = ur3eCfg.bfsCameraFy;
    intrinsics.width = frame.width;
    intrinsics.height = frame.height;
    intrinsics.cx = ur3eCfg.bfsCameraCx > 0.0
                        ? ur3eCfg.bfsCameraCx
                        : (frame.width > 0 ? 0.5 * static_cast<double>(frame.width) : 0.0);
    intrinsics.cy = ur3eCfg.bfsCameraCy > 0.0
                        ? ur3eCfg.bfsCameraCy
                        : (frame.height > 0 ? 0.5 * static_cast<double>(frame.height) : 0.0);
    intrinsics.distortion = ur3eCfg.bfsCameraDistortion;

    const Mat4 c2w = cameraToWorldOpenGlFromTcp(tcpForPose);
    const CameraExtrinsicsRt extrinsics = cameraExtrinsicsOpenCvFromTcp(tcpForPose);
    const QString imageName = stem + QStringLiteral(".tif");
    const QString poseJsonPath = QDir(captureDir).filePath(stem + QStringLiteral(".json"));
    QString poseError;
    CalibrationCaptureExtras calib = extrasFromLivePose(livePose);
    calib.fppStepLabel = fppStepLabel;
    if (fppStepIndex >= 0 && fppStepIndex < hf::dlp::kFppScanningStepCount)
    {
        const hf::dlp::FppScanStep &step = hf::dlp::kFppScanningSteps[fppStepIndex];
        // Sequence index matches decoder-facing PSP indices 0..25 (black..v80).
        calib.fppStepIndex = fppStepIndex;
        const char *patternName = step.patternName;
        calib.fppPattern = patternName != nullptr
                               ? QString::fromUtf8(patternName)
                               : QStringLiteral("Black");
        if (calib.fppStepLabel.isEmpty())
            calib.fppStepLabel = QString::fromUtf8(step.label);
        if (host != nullptr && host->dlpPanel() != nullptr)
        {
            const hf::dlp::DlpProjectorSettings led = host->dlpPanel()->currentSettings();
            calib.haveDlpLed = true;
            calib.dlpLedRedMa = led.ledRedMa;
            calib.dlpLedGreenMa = led.ledGreenMa;
            calib.dlpLedBlueMa = led.ledBlueMa;
        }
    }
    if (outputShift.apply)
    {
        calib.haveOutputStageShift = true;
        calib.stageCapturePositionMm = outputShift.stageCaptureMm;
        calib.stageOutputPositionMm = outputShift.stageOutputMm;
        calib.outputShiftXM = outputShift.xM;
        calib.outputShiftYM = outputShift.yM;
        calib.outputShiftZM = outputShift.zM;
    }
    fillBfsCaptureExtras(calib, host);
    if (!calib.haveFlange)
    {
        QMetaObject::invokeMethod(
            controller,
            [host, poseJsonPath]() {
                host->appendLog(
                    QStringLiteral(
                        "UR3e scan capture: no base_T_flange (live tool0 TF "
                        "missing) — %1 not usable for hand-eye.")
                        .arg(QFileInfo(poseJsonPath).fileName()));
            },
            Qt::QueuedConnection);
    }
    if (!writeCameraPoseJson(poseJsonPath,
                             tcpForPose,
                             c2w,
                             extrinsics,
                             intrinsics,
                             imageName,
                             poseSource,
                             &plannedTcp,
                             &poseError,
                             &calib))
    {
        if (ok != nullptr)
            *ok = false;
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("Failed to save pose JSON %1: %2")
                                .arg(poseJsonPath, poseError);
        }
        return false;
    }

    if (transformsDoc->intrinsics.width <= 0)
        transformsDoc->intrinsics = intrinsics;
    TransformsJsonFrame entry;
    entry.filePathStem = stem;
    entry.transformMatrix = c2w;
    entry.extrinsics = extrinsics;
    entry.fppStepIndex = calib.fppStepIndex;
    entry.fppStepLabel = calib.fppStepLabel;
    entry.fppPattern = calib.fppPattern;
    transformsDoc->frames.push_back(std::move(entry));
    ++(*captured);
    return true;
}

/// If DLP is connected: 26 PSP stills, then blank.
/// Otherwise one still (existing Multiview behavior).
/// Color RGB uses the plan RGB ring (rgb exposure / capture_kind), not a white FPP step.
bool capturePinStillsMaybeFpp(Ur3ePanelController *controller,
                              MainWindow *host,
                              const QString &serverUrl,
                              const QString &captureDir,
                              const Ur3eScanTcpPose &plannedTcp,
                              int *captured,
                              TransformsJsonDocument *transformsDoc,
                              bool *ok,
                              QString *errorMessage,
                              const QString &skipContext,
                              const std::function<bool()> &sessionActive,
                              const OutputPoseShift &outputShift = {},
                              bool rgbOnly = false)
{
    hf::dlp::DlpPanelController *dlp = host != nullptr ? host->dlpPanel() : nullptr;
    const bool fpp = !rgbOnly && dlp != nullptr && dlp->isConnected();
    if (!fpp)
    {
        return saveOneBfsStillAtPin(controller,
                                    host,
                                    serverUrl,
                                    captureDir,
                                    plannedTcp,
                                    captured,
                                    transformsDoc,
                                    ok,
                                    errorMessage,
                                    skipContext,
                                    -1,
                                    {},
                                    outputShift);
    }

    const int fppSteps = hf::dlp::kFppScanningStepCount;
    QMetaObject::invokeMethod(
        controller,
        [host, skipContext, fppSteps]() {
            host->appendLog(
                QStringLiteral(
                    "UR3e scan capture: FPP burst at %1 (%2 patterns, wait %3 new + %4 settle BFS frame(s)).")
                    .arg(skipContext)
                    .arg(fppSteps)
                    .arg(hf::dlp::kFppCaptureMinNewFrames)
                    .arg(hf::dlp::kFppCaptureStabilizeFrames));
        },
        Qt::QueuedConnection);

    hf::bfs::BfsPanelController *bfs = host->bfsPanel();
    if (bfs == nullptr)
    {
        if (ok != nullptr)
            *ok = false;
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("BFS panel unavailable for FPP capture.");
        return false;
    }

    bool burstOk = true;
    bool burstAborted = false;
    const auto keepGoing = [&]() {
        return !sessionActive || sessionActive();
    };
    const auto abortRequested = [&]() { return !keepGoing(); };

    for (int step = 0; step < fppSteps; ++step)
    {
        if (!keepGoing())
        {
            burstAborted = true;
            break;
        }

        QString dlpError;
        if (!dlp->showFppScanStepSync(step, &dlpError))
        {
            if (ok != nullptr)
                *ok = false;
            if (errorMessage != nullptr)
            {
                *errorMessage = QStringLiteral("DLP FPP step %1 failed at %2: %3")
                                    .arg(step)
                                    .arg(skipContext, dlpError);
            }
            burstOk = false;
            break;
        }

        if (!keepGoing())
        {
            burstAborted = true;
            break;
        }

        const std::uint64_t beforeIndex = bfs->lastFrameIndex();
        if (!bfs->waitForNewerFrame(beforeIndex,
                                    hf::dlp::kFppCaptureMinNewFrames,
                                    hf::dlp::kFppCaptureFrameWaitMs,
                                    nullptr,
                                    abortRequested))
        {
            if (!keepGoing())
            {
                burstAborted = true;
                break;
            }
            if (ok != nullptr)
                *ok = false;
            if (errorMessage != nullptr)
            {
                *errorMessage =
                    QStringLiteral(
                        "Timed out waiting for %1 new BFS frame(s) after FPP step %2 at %3 "
                        "(last frameIndex=%4).")
                        .arg(hf::dlp::kFppCaptureMinNewFrames)
                        .arg(step)
                        .arg(skipContext)
                        .arg(bfs->lastFrameIndex());
            }
            burstOk = false;
            break;
        }
        if (!bfs->waitForNewerFrame(bfs->lastFrameIndex(),
                                    hf::dlp::kFppCaptureStabilizeFrames,
                                    hf::dlp::kFppCaptureStabilizeWaitMs,
                                    nullptr,
                                    abortRequested))
        {
            if (!keepGoing())
            {
                burstAborted = true;
                break;
            }
            if (ok != nullptr)
                *ok = false;
            if (errorMessage != nullptr)
            {
                *errorMessage =
                    QStringLiteral(
                        "Timed out waiting for %1 settle BFS frame(s) after FPP step %2 at %3 "
                        "(last frameIndex=%4).")
                        .arg(hf::dlp::kFppCaptureStabilizeFrames)
                        .arg(step)
                        .arg(skipContext)
                        .arg(bfs->lastFrameIndex());
            }
            burstOk = false;
            break;
        }

        if (!keepGoing())
        {
            burstAborted = true;
            break;
        }

        const QString label = QString::fromUtf8(hf::dlp::kFppScanningSteps[step].label);
        if (!saveOneBfsStillAtPin(controller,
                                  host,
                                  serverUrl,
                                  captureDir,
                                  plannedTcp,
                                  captured,
                                  transformsDoc,
                                  ok,
                                  errorMessage,
                                  skipContext,
                                  step,
                                  label,
                                  outputShift))
        {
            burstOk = false;
            break;
        }
    }

    QString blankError;
    if (!dlp->blankSync(&blankError))
    {
        QMetaObject::invokeMethod(
            controller,
            [host, blankError]() {
                host->appendLog(
                    QStringLiteral("UR3e scan capture: DLP blank after FPP failed — %1")
                        .arg(blankError));
            },
            Qt::QueuedConnection);
        if (burstOk && !burstAborted)
        {
            if (ok != nullptr)
                *ok = false;
            if (errorMessage != nullptr)
                *errorMessage = QStringLiteral("DLP blank after FPP failed: %1").arg(blankError);
            burstOk = false;
        }
    }

    if (burstAborted)
    {
        QMetaObject::invokeMethod(
            controller,
            [host, skipContext]() {
                host->appendLog(
                    QStringLiteral("UR3e scan capture: FPP burst aborted at %1 (stop).")
                        .arg(skipContext));
            },
            Qt::QueuedConnection);
        return false;
    }
    return burstOk;
}

void appendScanPlanFailureReport(MainWindow *host, const Ur3eHemisphereScanPlan &plan)
{
    if (host == nullptr || plan.unreachableCount <= 0)
        return;

    QHash<QString, QStringList> grouped;
    for (int pointIndex = 0; pointIndex < static_cast<int>(plan.points.size()); ++pointIndex)
    {
        const Ur3ePlannedScanPoint &point = plan.points[static_cast<std::size_t>(pointIndex)];
        if (point.reachable)
            continue;

        QString reason = point.planningError.trimmed();
        if (reason.isEmpty())
            reason = QStringLiteral("unknown");
        grouped[reason].append(QString::number(pointIndex));
    }

    host->appendLog(QStringLiteral("UR3e scan plan failure breakdown:"));
    for (auto it = grouped.constBegin(); it != grouped.constEnd(); ++it)
    {
        host->appendLog(QStringLiteral("UR3e scan plan failure: %1 — %2 point(s): #%3")
                            .arg(it.key())
                            .arg(it.value().size())
                            .arg(it.value().join(QLatin1Char(','))));
    }

    for (int pointIndex = 0; pointIndex < static_cast<int>(plan.points.size()); ++pointIndex)
    {
        const Ur3ePlannedScanPoint &point = plan.points[static_cast<std::size_t>(pointIndex)];
        if (point.reachable)
            continue;

        host->appendLog(
            QStringLiteral("UR3e scan plan failure pt %1 tcp=(%2, %3, %4) m: %5")
                .arg(pointIndex)
                .arg(point.tcp.xM, 0, 'f', 3)
                .arg(point.tcp.yM, 0, 'f', 3)
                .arg(point.tcp.zM, 0, 'f', 3)
                .arg(point.planningError.trimmed().isEmpty() ? QStringLiteral("unknown")
                                                               : point.planningError.trimmed()));
    }
}

QString formatJointTargetsDeg(const std::vector<double> &targetRad)
{
    QStringList parts;
    const int count = qMin(static_cast<int>(targetRad.size()), kUr3eJointCount);
    for (int jointIndex = 0; jointIndex < count; ++jointIndex)
    {
        double rad = targetRad[static_cast<std::size_t>(jointIndex)];
        while (rad > M_PI)
            rad -= 2.0 * M_PI;
        while (rad <= -M_PI)
            rad += 2.0 * M_PI;
        const int degrees = static_cast<int>(std::lround(rad * 180.0 / M_PI));
        parts << QStringLiteral("%1=%2°")
                     .arg(QString::fromUtf8(kUr3eJointNames[jointIndex]))
                     .arg(degrees);
    }
    return parts.join(QStringLiteral(", "));
}

QString formatTcpPose(const Ur3eTcpPose &pose)
{
    return QStringLiteral("[%1, %2, %3, %4, %5, %6]")
        .arg(pose.x, 0, 'f', 3)
        .arg(pose.y, 0, 'f', 3)
        .arg(pose.z, 0, 'f', 3)
        .arg(pose.rx, 0, 'f', 3)
        .arg(pose.ry, 0, 'f', 3)
        .arg(pose.rz, 0, 'f', 3);
}

QString formatConfiguredHomeJointsDeg()
{
    const std::array<double, 6> &homeDeg = hf::hardwareConfig().ur3e.homeJointsDeg;
    QStringList parts;
    for (int jointIndex = 0; jointIndex < kUr3eJointCount; ++jointIndex)
    {
        parts << QStringLiteral("%1=%2°")
                     .arg(QString::fromUtf8(kUr3eJointNames[jointIndex]))
                     .arg(static_cast<int>(std::lround(homeDeg[static_cast<std::size_t>(jointIndex)])));
    }
    return parts.join(QStringLiteral(", "));
}
} // namespace

Ur3ePanelController::Ur3ePanelController(MainWindow *host, QObject *parent)
    : QObject(parent)
    , host_(host)
    , serverManager_(std::make_unique<Ur3eServerManager>(host))
    , moveItManager_(std::make_unique<Ur3eMoveItManager>(host))
    , rvizManager_(std::make_unique<Ur3eRvizManager>(host))
{
    connect(serverManager_.get(),
            &Ur3eServerManager::stateChanged,
            this,
            &Ur3ePanelController::onSidecarStateChanged);
    connect(moveItManager_.get(),
            &Ur3eMoveItManager::stateChanged,
            this,
            &Ur3ePanelController::onMoveItStateChanged);
    connect(rvizManager_.get(),
            &Ur3eRvizManager::stateChanged,
            this,
            &Ur3ePanelController::onRvizStateChanged);

    connectPollTimer_ = new QTimer(this);
    connectPollTimer_->setInterval(kConnectPollIntervalMs);
    connect(connectPollTimer_, &QTimer::timeout, this, &Ur3ePanelController::onConnectPollTick);

    connectCountdownTimer_ = new QTimer(this);
    connectCountdownTimer_->setInterval(1000);
    connect(connectCountdownTimer_, &QTimer::timeout, this, &Ur3ePanelController::onConnectCountdownTick);

    driverReadyPollTimer_ = new QTimer(this);
    driverReadyPollTimer_->setInterval(kDriverReadyPollIntervalMs);
    connect(driverReadyPollTimer_, &QTimer::timeout, this, &Ur3ePanelController::pollDriverPrestartReady);

    boundarySyncTimer_ = new QTimer(this);
    boundarySyncTimer_->setInterval(kBoundarySyncIntervalMs);
    connect(boundarySyncTimer_, &QTimer::timeout, this, &Ur3ePanelController::onBoundarySyncTick);

    driverPrestartReady_ = !hf::hardwareConfig().ur3e.prestartDriver;
}

Ur3ePanelController::~Ur3ePanelController()
{
    dismissConnectWaitDialog();
    (void)shutdownSync();
}

bool Ur3ePanelController::isSidecarRunning() const
{
    return serverManager_ != nullptr && serverManager_->isServerConnected();
}

bool Ur3ePanelController::isScanPlanReady() const
{
    if (host_ != nullptr && host_->ur3eHemisphereScanSettings_ != nullptr
        && isSavedRingRouteMode(host_->ur3eHemisphereScanSettings_->scanExecuteMode()))
        return host_->ur3eHemisphereScanSettings_->semiFixedRouteReady();
    return scanPlanReady_ && plannedScanPlan_.reachableCount > 0;
}

bool Ur3ePanelController::tryGetLiveOpticalTcpPose(Ur3eScanTcpPose *out,
                                                   QString *errorMessage,
                                                   CalibrationCaptureExtras *calibOut) const
{
    if (out == nullptr)
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("output pose is null");
        return false;
    }
    if (!robotConnected_ || serverManager_ == nullptr || !serverManager_->isServerConnected())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("robot not connected");
        return false;
    }

    const Ur3ePoseResult livePose = ur3eGetTcpPose(serverManager_->serverUrl(), errorMessage);
    if (!livePose.ok)
        return false;

    QString poseSource;
    QString warn;
    if (!resolveCameraOpticalTcp(livePose, Ur3eScanTcpPose{}, out, &poseSource, &warn))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = warn.isEmpty()
                                ? QStringLiteral("camera optical TCP unavailable")
                                : warn;
        }
        return false;
    }
    (void)poseSource;
    if (calibOut != nullptr)
        *calibOut = extrasFromLivePose(livePose);
    return true;
}

bool Ur3ePanelController::captureStationaryFppBurst(const QString &captureDir, QString *errorMessage)
{
    if (host_ == nullptr)
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("BFS / UR3e host unavailable.");
        return false;
    }
    if (host_->dlpPanel() == nullptr || !host_->dlpPanel()->isConnected())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("DLP not connected.");
        return false;
    }

    Ur3eScanTcpPose planned{};
    QString poseError;
    if (!tryGetLiveOpticalTcpPose(&planned, &poseError, nullptr))
    {
        QMetaObject::invokeMethod(
            this,
            [this, poseError]() {
                host_->appendLog(
                    QStringLiteral(
                        "BFS Capture FPP: no live TCP (%1) — writing stills without a planned pose.")
                        .arg(poseError));
            },
            Qt::QueuedConnection);
    }

    const QString serverUrl =
        serverManager_ != nullptr ? serverManager_->serverUrl() : QString();
    int captured = 0;
    TransformsJsonDocument transformsDoc;
    bool ok = true;
    QString err;
    const bool burstOk = capturePinStillsMaybeFpp(this,
                                                  host_,
                                                  serverUrl,
                                                  captureDir,
                                                  planned,
                                                  &captured,
                                                  &transformsDoc,
                                                  &ok,
                                                  &err,
                                                  QStringLiteral("BFS Capture (stationary)"),
                                                  []() { return true; },
                                                  {});
    if (!burstOk)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = err.isEmpty() ? QStringLiteral("FPP burst failed.") : err;
        }
        return false;
    }
    if (captured <= 0)
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("FPP burst wrote no stills.");
        return false;
    }
    QString writeError;
    if (!writeTransformsJson(captureDir, transformsDoc, &writeError, false))
    {
        if (errorMessage != nullptr)
            *errorMessage = writeError;
        return false;
    }
    return true;
}

void Ur3ePanelController::applyHardwareConfigToUi()
{
    const hf::HardwareConfig::Ur3eConfig &cfg = hf::hardwareConfig().ur3e;

    if (host_->ur3eRobotIpEdit_ != nullptr)
        host_->ur3eRobotIpEdit_->setText(cfg.robotIp);

    applyConfiguredInitialJointTargets();
    syncWorkspaceBoundaryPreview();
    if (host_->ur3eScanRoutePlanWidget_ != nullptr && host_->ur3eHemisphereScanSettings_ != nullptr)
        host_->ur3eScanRoutePlanWidget_->setScanParams(host_->ur3eHemisphereScanSettings_->params());
    updateRobotUi();
}

void Ur3ePanelController::startSidecarOnLaunch()
{
    if (serverManager_ == nullptr)
        return;

    const hf::HardwareConfig::Ur3eConfig &cfg = hf::hardwareConfig().ur3e;
    if (!cfg.useMultiview)
        return;

    host_->appendLog(QStringLiteral(
        "UR3e: preparing WSL (%1, prestart_driver=%2)\u2026")
                        .arg(cfg.useMockHardware ? QStringLiteral("simulation startup")
                                                 : QStringLiteral("network setup + cleanup"))
                        .arg(cfg.prestartDriver ? QStringLiteral("true") : QStringLiteral("false")));

    std::thread([this]() {
        QString setupDetail;
        (void)runUr3eStartupSetupOnce(&setupDetail);
        QMetaObject::invokeMethod(
            this,
            [this, setupDetail]() {
                if (!setupDetail.isEmpty())
                    host_->appendLog(QStringLiteral("UR3e: %1").arg(setupDetail));
                host_->appendLog(QStringLiteral("UR3e: starting sidecar\u2026"));
                serverManager_->tryAutoStart();
            },
            Qt::QueuedConnection);
    }).detach();
}

void Ur3ePanelController::refreshUi()
{
    updateRobotUi();
}

bool Ur3ePanelController::shutdownSync()
{
    if (!hf::hardwareConfig().ur3e.useMultiview)
        return true;

    shutdownRequested_.store(true, std::memory_order_release);
    stopRequested_.store(true, std::memory_order_release);
    ++scanExecuteSessionId_;

    if (connectInProgress_)
        onConnectDialogCancelled();

    scanExecuting_ = false;
    scanPlanning_ = false;
    motionInProgress_ = false;

    if (robotConnected_ && serverManager_ != nullptr && serverManager_->isServerConnected())
        ur3eStopMotion(serverManager_->serverUrl());

    {
        std::lock_guard<std::mutex> lock(scanExecuteThreadMutex_);
        if (scanExecuteThread_.joinable())
            scanExecuteThread_.join();
    }

    joinJointPollThread();

    if (host_->ur3ePosePollTimer_ != nullptr)
        host_->ur3ePosePollTimer_->stop();

    if (robotConnected_ && serverManager_ != nullptr && serverManager_->isServerConnected())
    {
        beginHomeMotionUi();
        const HomeEnsureOutcome homeOutcome =
            ensureRobotAtHomeSync(HomeEnsureContext::BeforeShutdown);
        endHomeMotionUi();
        if (homeOutcome.cancelled)
        {
            shutdownRequested_.store(false, std::memory_order_release);
            stopRequested_.store(false, std::memory_order_release);
            if (host_->ur3ePosePollTimer_ != nullptr)
                host_->ur3ePosePollTimer_->start(kPosePollIntervalMs);
            pollJointsSync();
            syncTargetsFromCurrent();
            return false;
        }
        pollJointsSync();
        if (homeOutcome.atHomeVerified)
            applyScanHomeJointTargets();
    }

    if (robotConnected_ && serverManager_ != nullptr && serverManager_->isServerConnected())
        ur3eDisconnectRobot(serverManager_->serverUrl());

    if (serverManager_ != nullptr)
        serverManager_->stopServer();

    if (moveItManager_ != nullptr)
        moveItManager_->stop();

    if (rvizManager_ != nullptr)
        rvizManager_->stop();

    robotConnected_ = false;
    return true;
}

void Ur3ePanelController::wireSettingsTabConnections()
{
    if (host_->ur3eConnectBtn_ != nullptr)
    {
        connect(host_->ur3eConnectBtn_, &QPushButton::clicked, this, [this]() {
            onConnectRequested();
        });
    }
    if (host_->ur3eDisconnectBtn_ != nullptr)
    {
        connect(host_->ur3eDisconnectBtn_, &QPushButton::clicked, this, [this]() {
            onDisconnectRequested();
        });
    }
    if (host_->ur3eMoveBtn_ != nullptr)
    {
        connect(host_->ur3eMoveBtn_, &QPushButton::clicked, this, [this]() {
            onMoveRequested();
        });
    }
    for (int jointIndex = 0; jointIndex < MainWindow::kUr3eJointCount; ++jointIndex)
    {
        ui::Ur3eJointBarWidget *bar = host_->ur3eJointBars_[jointIndex];
        if (bar == nullptr)
            continue;
        connect(bar,
                &ui::Ur3eJointBarWidget::targetChanged,
                this,
                [this](const double) { scheduleManualTargetPreview(); });
    }
    if (host_->ur3eStopMotionBtn_ != nullptr)
    {
        connect(host_->ur3eStopMotionBtn_, &QPushButton::clicked, this, [this]() {
            onStopMotionRequested();
        });
    }
    if (host_->ur3eSyncJointsBtn_ != nullptr)
    {
        connect(host_->ur3eSyncJointsBtn_, &QPushButton::clicked, this, [this]() {
            onSyncJointsRequested();
        });
    }
    if (host_->ur3eStartRvizBtn_ != nullptr)
    {
        connect(host_->ur3eStartRvizBtn_, &QPushButton::clicked, this, [this]() {
            onStartRvizRequested();
        });
    }
    if (host_->ur3eStartMoveItBtn_ != nullptr)
    {
        connect(host_->ur3eStartMoveItBtn_, &QPushButton::clicked, this, [this]() {
            onStartMoveItRequested();
        });
    }
    if (host_->ur3eHemisphereScanSettings_ != nullptr)
    {
        connect(host_->ur3eHemisphereScanSettings_,
                &ui::Ur3eHemisphereScanSettingsWidget::planScanRequested,
                this,
                [this]() { onPlanHemisphereScanRequested(); });
        connect(host_->ur3eHemisphereScanSettings_,
                &ui::Ur3eHemisphereScanSettingsWidget::executeScanRequested,
                this,
                [this]() { onExecuteHemisphereScanRequested(); });
        connect(host_->ur3eHemisphereScanSettings_,
                &ui::Ur3eHemisphereScanSettingsWidget::loadScanRouteRequested,
                this,
                &Ur3ePanelController::onLoadScanRouteRequested);
        connect(host_->ur3eHemisphereScanSettings_,
                &ui::Ur3eHemisphereScanSettingsWidget::loadPlannedRouteAsSemiFixedRequested,
                this,
                &Ur3ePanelController::onLoadPlannedRouteAsSemiFixedRequested);
        connect(host_->ur3eHemisphereScanSettings_,
                &ui::Ur3eHemisphereScanSettingsWidget::scanTcpChanged,
                this,
                [this]() {
                    const bool dlp = hf::hardwareConfig().ur3e.usesDlpScanTcp();
                    host_->appendLog(
                        QStringLiteral("UR3e: scan tip → %1. Disconnect / Connect to rematerialize "
                                       "hyperfusion_tcp.")
                            .arg(dlp ? QStringLiteral("DLP lens") : QStringLiteral("Camera lens")));
                });
        connect(host_->ur3eHemisphereScanSettings_,
                &ui::Ur3eHemisphereScanSettingsWidget::paramsChanged,
                this,
                [this]() {
                    if (host_->ur3eHemisphereScanSettings_ != nullptr
                        && host_->ur3eHemisphereScanSettings_->scanExecuteMode()
                               == Ur3eScanExecuteMode::SemiFixed)
                    {
                        // Params changed: drop planned rings so preview tracks Layer/θ/radius.
                        plannedScanPlan_ = Ur3eHemisphereScanPlan{};
                        scanPlanReady_ = false;
                        Ur3eSemiFixedRoute route =
                            host_->ur3eHemisphereScanSettings_->semiFixedRoute();
                        if (!route.rings.isEmpty())
                        {
                            route.rings.clear();
                            host_->ur3eHemisphereScanSettings_->setSemiFixedRoute(route);
                        }
                        else
                            refreshSemiFixedPreview();
                        updateRobotUi();
                        if (host_->capturePanel() != nullptr)
                            host_->capturePanel()->syncBfsAndMultiviewRgbCaptureControls();
                        return;
                    }
                    scanPlanReady_ = false;
                    if (host_->ur3eHemisphereScanSettings_ != nullptr)
                        host_->ur3eHemisphereScanSettings_->setPlannedReachablePins(-1);
                    if (host_->ur3eScanRoutePlanWidget_ != nullptr)
                    {
                        host_->ur3eScanRoutePlanWidget_->clearSemiFixedPreviewRings();
                        host_->ur3eScanRoutePlanWidget_->clearScanPlan();
                    }
                    if (host_->ur3eScanRoutePlanWidget_ != nullptr
                        && host_->ur3eHemisphereScanSettings_ != nullptr)
                    {
                        host_->ur3eScanRoutePlanWidget_->setScanParams(
                            host_->ur3eHemisphereScanSettings_->params());
                    }
                    scheduleManualTargetPreview();
                    updateRobotUi();
                    if (host_->capturePanel() != nullptr)
                        host_->capturePanel()->syncBfsAndMultiviewRgbCaptureControls();
                });
        connect(host_->ur3eHemisphereScanSettings_,
                &ui::Ur3eHemisphereScanSettingsWidget::scanModeChanged,
                this,
                [this]() {
                    if (host_->ur3eHemisphereScanSettings_ == nullptr)
                        return;
                    if (host_->ur3eHemisphereScanSettings_->scanExecuteMode()
                        == Ur3eScanExecuteMode::SemiFixed)
                        refreshSemiFixedPreview();
                    else if (host_->ur3eHemisphereScanSettings_->scanExecuteMode()
                             == Ur3eScanExecuteMode::Fpp)
                        refreshSemiFixedPreview();
                    else if (host_->ur3eScanRoutePlanWidget_ != nullptr)
                    {
                        host_->ur3eScanRoutePlanWidget_->clearSemiFixedPreviewRings();
                        if (scanPlanReady_)
                            host_->ur3eScanRoutePlanWidget_->setScanPlan(plannedScanPlan_);
                        else
                            host_->ur3eScanRoutePlanWidget_->clearScanPlan();
                    }
                    updateRobotUi();
                    if (host_->capturePanel() != nullptr)
                        host_->capturePanel()->syncBfsAndMultiviewRgbCaptureControls();
                });
        connect(host_->ur3eHemisphereScanSettings_,
                &ui::Ur3eHemisphereScanSettingsWidget::semiFixedRouteChanged,
                this,
                [this]() {
                    refreshSemiFixedPreview();
                    updateRobotUi();
                    if (host_->capturePanel() != nullptr)
                        host_->capturePanel()->syncBfsAndMultiviewRgbCaptureControls();
                });
        connect(host_->ur3eHemisphereScanSettings_,
                &ui::Ur3eHemisphereScanSettingsWidget::addSemiFixedRingRequested,
                this,
                &Ur3ePanelController::onAddSemiFixedRingRequested);
    }

    if (host_->ur3ePosePollTimer_ != nullptr)
    {
        connect(host_->ur3ePosePollTimer_, &QTimer::timeout, this, [this]() { pollJoints(); });
    }

    tryLoadCachedScanPlan();
}

void Ur3ePanelController::onSidecarStateChanged(const Ur3eServerManager::State state,
                                                  const QString &detail)
{
    bool sidecarStateChanged = false;
    if (serverManager_ != nullptr)
    {
        const bool stateChanged = state != lastLoggedSidecarState_;
        sidecarStateChanged = stateChanged;
        if (stateChanged)
        {
            lastLoggedSidecarState_ = state;
            if (state == Ur3eServerManager::State::Starting)
            {
                host_->appendLog(QStringLiteral("UR3e sidecar: starting (driver may take up to 2 min)\u2026"));
            }
            else if (state == Ur3eServerManager::State::Running
                || state == Ur3eServerManager::State::Failed
                || state == Ur3eServerManager::State::Unavailable)
            {
                host_->appendLog(
                    QStringLiteral("UR3e sidecar: %1").arg(serverManager_->statusText()));
            }
        }
        else if (!detail.isEmpty())
        {
            const QStringList lines = detail.split(QLatin1Char('\n'));
            for (const QString &line : lines)
            {
                const QString trimmed = line.trimmed();
                if (trimmed.isEmpty() || trimmed.contains(QStringLiteral("GET /health"), Qt::CaseInsensitive))
                    continue;
                if (connectInProgress_
                    && (trimmed.contains(QStringLiteral("connect phase="), Qt::CaseInsensitive)
                        || trimmed.contains(QStringLiteral("GET /connect/status"), Qt::CaseInsensitive)))
                {
                    updateConnectDialogFromSidecarLine(trimmed);
                    continue;
                }
                host_->appendLog(trimmed);
                updateConnectDialogFromSidecarLine(trimmed);
            }
        }
    }

    if (!isSidecarRunning())
    {
        robotConnected_ = false;
        driverPrestartReady_ = false;
        if (driverReadyPollTimer_ != nullptr)
            driverReadyPollTimer_->stop();
        if (boundarySyncTimer_ != nullptr)
            boundarySyncTimer_->stop();
        if (host_->ur3ePosePollTimer_ != nullptr)
            host_->ur3ePosePollTimer_->stop();
        updateRobotUi();
    }
    else
    {
        if (boundarySyncTimer_ != nullptr && !boundarySyncTimer_->isActive())
            boundarySyncTimer_->start();
        // Only (re)start the driver-ready poll on an actual transition into Running.
        if (state == Ur3eServerManager::State::Running && sidecarStateChanged)
        {
            pushWorkspaceBoundaryToMoveIt();
            const hf::HardwareConfig::Ur3eConfig &cfg = hf::hardwareConfig().ur3e;
            if (cfg.prestartDriver)
            {
                driverPrestartReady_ = false;
                if (driverReadyPollTimer_ != nullptr)
                    driverReadyPollTimer_->start();
                pollDriverPrestartReady();
            }
            else
            {
                driverPrestartReady_ = true;
            }
        }
        updateRobotUi();
    }
}

void Ur3ePanelController::setBusy(const bool busy)
{
    busy_ = busy;
    updateRobotUi();
}

void Ur3ePanelController::pollDriverPrestartReady()
{
    if (!hf::hardwareConfig().ur3e.prestartDriver || serverManager_ == nullptr
        || !serverManager_->isServerConnected())
    {
        if (driverReadyPollTimer_ != nullptr)
            driverReadyPollTimer_->stop();
        return;
    }

    if (driverPrestartReady_
        || driverReadyPollInFlight_.exchange(true, std::memory_order_acq_rel))
    {
        return;
    }

    const QString serverUrl = serverManager_->serverUrl();
    std::thread([this, serverUrl]() {
        Ur3eHealthStatus health;
        QString error;
        const bool ok = ur3eServerHealthCheck(serverUrl, &health, &error, 15000);
        QMetaObject::invokeMethod(
            this,
            [this, ok, health, error]() {
                driverReadyPollInFlight_.store(false, std::memory_order_release);
                if (!ok || !isSidecarRunning())
                {
                    if (!ok && !error.isEmpty() && host_ != nullptr)
                    {
                        // Rate-limit: only log when still waiting (timer still active).
                        static qint64 lastLogMs = 0;
                        const qint64 now = QDateTime::currentMSecsSinceEpoch();
                        if (now - lastLogMs > 10000)
                        {
                            lastLogMs = now;
                            host_->appendLog(
                                QStringLiteral("UR3e: waiting for driver ready (%1)…").arg(error));
                        }
                    }
                    return;
                }

                if (health.driverReady)
                {
                    if (!driverPrestartReady_)
                    {
                        host_->appendLog(
                            hf::hardwareConfig().ur3e.useMockHardware
                                ? QStringLiteral("UR3e: simulation driver ready.")
                                : QStringLiteral("UR3e: robot driver ready."));
                    }
                    driverPrestartReady_ = true;
                    if (driverReadyPollTimer_ != nullptr)
                        driverReadyPollTimer_->stop();
                    updateRobotUi();
                }
            },
            Qt::QueuedConnection);
    }).detach();
}

void Ur3ePanelController::updateRobotUi()
{
    const bool sidecarRunning = isSidecarRunning();
    const bool captureActive = host_->isCaptureSessionActive();
    const auto &urCfg = hf::hardwareConfig().ur3e;

    if (host_->ur3eRobotIpEdit_ != nullptr)
        host_->ur3eRobotIpEdit_->setEnabled(!robotConnected_ && !busy_ && !captureActive);

    // Connect only when the UR driver prestart reports ready (port 50002 / controllers).
    const bool driverReady = !urCfg.prestartDriver || driverPrestartReady_;

    if (host_->ur3eConnectBtn_ != nullptr)
    {
        host_->ur3eConnectBtn_->setEnabled(sidecarRunning && driverReady && !robotConnected_ && !busy_
                                            && !connectInProgress_ && !captureActive);
        if (!sidecarRunning)
        {
            host_->ur3eConnectBtn_->setToolTip(
                QStringLiteral("Waiting for UR3e sidecar (WSL). Check the Log tab for status."));
        }
        else if (!driverReady)
        {
            host_->ur3eConnectBtn_->setToolTip(
                urCfg.useMockHardware
                    ? QStringLiteral(
                          "Waiting for simulation driver warmup in WSL (up to ~2 min after sidecar starts).")
                    : QStringLiteral(
                          "Waiting for UR robot driver (reverse port 50002 / controller manager). "
                          "Check the Log tab."));
        }
        else
        {
            host_->ur3eConnectBtn_->setToolTip(QString());
        }
    }
    if (host_->ur3eDisconnectBtn_ != nullptr)
    {
        host_->ur3eDisconnectBtn_->setEnabled(sidecarRunning && robotConnected_ && !busy_
                                              && !captureActive);
    }

    const bool motionReady = sidecarRunning && robotConnected_ && !captureActive;
    const bool canStartMotion = motionReady && !busy_;

    if (host_->ur3eMoveBtn_ != nullptr)
        host_->ur3eMoveBtn_->setEnabled(canStartMotion);
    if (host_->ur3eStopMotionBtn_ != nullptr)
        host_->ur3eStopMotionBtn_->setEnabled(motionInProgress_ || scanExecuting_);
    if (host_->ur3eSyncJointsBtn_ != nullptr)
        host_->ur3eSyncJointsBtn_->setEnabled(canStartMotion);

    const bool vizUiEnabled =
        sidecarRunning && robotConnected_ && !busy_ && !captureActive;
    const bool rvizRunning = rvizManager_ != nullptr && rvizManager_->isRunning();
    if (host_->ur3eStartRvizBtn_ != nullptr)
    {
        host_->ur3eStartRvizBtn_->setEnabled(vizUiEnabled);
        host_->ur3eStartRvizBtn_->setText(rvizRunning ? QStringLiteral("Stop RViz")
                                                      : QStringLiteral("Start RViz"));
    }

    const bool moveItRunning =
        moveItManager_ != nullptr && moveItManager_->isRunning();
    if (host_->ur3eStartMoveItBtn_ != nullptr)
    {
        host_->ur3eStartMoveItBtn_->setEnabled(vizUiEnabled);
        host_->ur3eStartMoveItBtn_->setText(moveItRunning ? QStringLiteral("Stop MoveIt")
                                                          : QStringLiteral("Start MoveIt"));
    }

    for (int jointIndex = 0; jointIndex < MainWindow::kUr3eJointCount; ++jointIndex)
    {
        ui::Ur3eJointBarWidget *bar = host_->ur3eJointBars_[jointIndex];
        if (bar != nullptr)
            bar->setEnabled(canStartMotion);
    }

    if (host_->ur3eHemisphereScanSettings_ != nullptr)
    {
        host_->ur3eHemisphereScanSettings_->setPlanEnabled(canStartMotion && !busy_);
        const bool executeReady =
            isSavedRingRouteMode(host_->ur3eHemisphereScanSettings_->scanExecuteMode())
                ? host_->ur3eHemisphereScanSettings_->semiFixedRouteReady()
                : (scanPlanReady_ && plannedScanPlan_.reachableCount > 0);
        host_->ur3eHemisphereScanSettings_->setExecuteEnabled(canStartMotion && executeReady
                                                               && !scanExecuting_);
        host_->ur3eHemisphereScanSettings_->setParamsEnabled(!scanPlanning_ && !scanExecuting_);
    }
}

void Ur3ePanelController::applyJointTargets(const std::vector<double> &positionsRad,
                                            const QStringList &names)
{
    for (int uiIndex = 0; uiIndex < MainWindow::kUr3eJointCount; ++uiIndex)
    {
        ui::Ur3eJointBarWidget *bar = host_->ur3eJointBars_[uiIndex];
        if (bar == nullptr)
            continue;

        int sourceIndex = uiIndex;
        if (names.size() >= MainWindow::kUr3eJointCount)
        {
            const int namedIndex =
                names.indexOf(QString::fromUtf8(kUr3eJointNames[uiIndex]));
            if (namedIndex >= 0)
                sourceIndex = namedIndex;
        }

        if (sourceIndex < 0 || sourceIndex >= static_cast<int>(positionsRad.size()))
            continue;

        bar->setValueRadians(positionsRad[static_cast<std::size_t>(sourceIndex)]);
    }
}

void Ur3ePanelController::setJointPollIntervalMs(const int intervalMs)
{
    if (host_->ur3ePosePollTimer_ != nullptr)
        host_->ur3ePosePollTimer_->setInterval(qMax(50, intervalMs));
}

void Ur3ePanelController::applyJointPositions(const std::vector<double> &positionsRad,
                                              const QStringList &names,
                                              const bool syncTargets)
{
    for (int uiIndex = 0; uiIndex < MainWindow::kUr3eJointCount; ++uiIndex)
    {
        ui::Ur3eJointBarWidget *bar = host_->ur3eJointBars_[uiIndex];
        if (bar == nullptr)
            continue;

        int sourceIndex = uiIndex;
        if (names.size() >= MainWindow::kUr3eJointCount)
        {
            const int namedIndex =
                names.indexOf(QString::fromUtf8(kUr3eJointNames[uiIndex]));
            if (namedIndex >= 0)
                sourceIndex = namedIndex;
        }

        if (sourceIndex < 0 || sourceIndex >= static_cast<int>(positionsRad.size()))
            continue;

        const double radians = positionsRad[static_cast<std::size_t>(sourceIndex)];
        bar->setCurrentRadians(radians);
        if (syncTargets)
            bar->setValueRadians(radians);
    }
}

void Ur3ePanelController::syncWorkspaceBoundaryPreview()
{
    const Ur3eWorkspaceBoundary boundary =
        workspaceBoundaryFromConfig(hf::hardwareConfig().ur3e);

    if (host_->ur3eHemisphereScanSettings_ != nullptr)
        host_->ur3eHemisphereScanSettings_->applyBoundaryLimits(boundary);

    if (host_->ur3eScanRoutePlanWidget_ != nullptr)
    {
        host_->ur3eScanRoutePlanWidget_->setWorkspaceBoundary(boundary);
        host_->ur3eScanRoutePlanWidget_->setSceneMount(
            Ur3eMountTransform::sceneAlignFromConfig(hf::hardwareConfig().ur3e));
    }

    pushWorkspaceBoundaryToMoveIt();
    scheduleManualTargetPreview();
}

void Ur3ePanelController::pushWorkspaceBoundaryToMoveIt()
{
    if (serverManager_ == nullptr || !serverManager_->isServerConnected())
        return;

    if (boundarySyncInFlight_.exchange(true))
        return;

    const QString serverUrl = serverManager_->serverUrl();
    std::thread([this, serverUrl]() {
        QString error;
        (void)ur3eSyncWorkspaceBoundary(serverUrl, &error);
        boundarySyncInFlight_.store(false);
        Q_UNUSED(error);
    }).detach();
}

void Ur3ePanelController::onBoundarySyncTick()
{
    if (!isSidecarRunning())
        return;

    const bool moveItRunning = moveItManager_ != nullptr && moveItManager_->isRunning();
    if (robotConnected_ || moveItRunning)
        pushWorkspaceBoundaryToMoveIt();
}

void Ur3ePanelController::applyScanHomeJointTargets()
{
    const hf::HardwareConfig::Ur3eConfig &cfg = hf::hardwareConfig().ur3e;
    for (int jointIndex = 0; jointIndex < MainWindow::kUr3eJointCount; ++jointIndex)
    {
        ui::Ur3eJointBarWidget *bar = host_->ur3eJointBars_[jointIndex];
        if (bar == nullptr)
            continue;
        const double radians = cfg.homeJointsDeg[static_cast<std::size_t>(jointIndex)] * M_PI / 180.0;
        bar->setValueRadians(radians);
    }

    scheduleManualTargetPreview();
}

void Ur3ePanelController::syncHomeJointTargetSliders()
{
    applyScanHomeJointTargets();
}

void Ur3ePanelController::applyConfiguredInitialJointTargets()
{
    if (!hf::hardwareConfig().ur3e.useMockHardware)
        return;

    applyScanHomeJointTargets();
}

void Ur3ePanelController::beginHomeMotionUi()
{
    motionInProgress_ = true;
    setJointPollIntervalMs(kMotionPollIntervalMs);
    applyScanHomeJointTargets();
}

void Ur3ePanelController::endHomeMotionUi()
{
    motionInProgress_ = false;
    setJointPollIntervalMs(kPosePollIntervalMs);
}

void Ur3ePanelController::scheduleManualTargetPreview()
{
    if (!robotConnected_ || serverManager_ == nullptr || !serverManager_->isServerConnected()
        || shutdownRequested_.load(std::memory_order_acquire))
    {
        return;
    }

    if (manualTargetPreviewInFlight_.load(std::memory_order_acquire))
    {
        manualTargetPreviewPending_.store(true, std::memory_order_release);
        return;
    }

    pushManualTargetPreview();
}

void Ur3ePanelController::pushManualTargetPreview()
{
    if (!robotConnected_ || serverManager_ == nullptr || !serverManager_->isServerConnected()
        || shutdownRequested_.load(std::memory_order_acquire))
    {
        manualTargetPreviewPending_.store(false, std::memory_order_release);
        return;
    }

    if (manualTargetPreviewInFlight_.exchange(true, std::memory_order_acq_rel))
    {
        manualTargetPreviewPending_.store(true, std::memory_order_release);
        return;
    }

    manualTargetPreviewPending_.store(false, std::memory_order_release);

    std::vector<double> target;
    target.reserve(MainWindow::kUr3eJointCount);
    for (int jointIndex = 0; jointIndex < MainWindow::kUr3eJointCount; ++jointIndex)
    {
        ui::Ur3eJointBarWidget *bar = host_->ur3eJointBars_[jointIndex];
        target.push_back(bar != nullptr ? bar->valueRadians() : 0.0);
    }

    const QString serverUrl = serverManager_->serverUrl();
    std::thread([this, serverUrl, target]() {
        QString error;
        (void)ur3ePreviewManualTarget(serverUrl, target, &error);
        QMetaObject::invokeMethod(
            this,
            [this]() {
                manualTargetPreviewInFlight_.store(false, std::memory_order_release);
                if (manualTargetPreviewPending_.exchange(false, std::memory_order_acq_rel))
                    pushManualTargetPreview();
            },
            Qt::QueuedConnection);
    }).detach();
}

void Ur3ePanelController::syncTargetsFromCurrent()
{
    for (int jointIndex = 0; jointIndex < MainWindow::kUr3eJointCount; ++jointIndex)
    {
        ui::Ur3eJointBarWidget *bar = host_->ur3eJointBars_[jointIndex];
        if (bar != nullptr)
            bar->syncTargetFromCurrent();
    }
}

void Ur3ePanelController::joinJointPollThread()
{
    std::lock_guard<std::mutex> lock(jointPollThreadMutex_);
    if (jointPollThread_.joinable())
        jointPollThread_.join();
}

void Ur3ePanelController::applyPolledJoints(const QVariantList &positionsRad, const QStringList &names)
{
    if (positionsRad.size() < MainWindow::kUr3eJointCount)
        return;

    std::vector<double> positions;
    positions.reserve(static_cast<std::size_t>(positionsRad.size()));
    for (const QVariant &value : positionsRad)
        positions.push_back(value.toDouble());

    applyJointPositions(positions, names, false);
}

void Ur3ePanelController::pollJointsSync()
{
    if (!robotConnected_ || serverManager_ == nullptr || !serverManager_->isServerConnected()
        || shutdownRequested_.load(std::memory_order_acquire))
    {
        return;
    }

    const Ur3eJointsState result = ur3eGetJoints(serverManager_->serverUrl());
    if (!result.ok || result.positionsRad.size() < MainWindow::kUr3eJointCount)
        return;

    applyJointPositions(result.positionsRad, result.names, false);
}

void Ur3ePanelController::pollJoints()
{
    if (!robotConnected_ || serverManager_ == nullptr || !serverManager_->isServerConnected()
        || shutdownRequested_.load(std::memory_order_acquire) || connectInProgress_)
    {
        return;
    }

    if (busy_ && !scanExecuting_ && !motionInProgress_)
        return;

    // Skip this tick if the previous poll is still running so we never queue up
    // blocking wsl.exe/curl launches on the UI thread.
    if (jointPollInFlight_.exchange(true, std::memory_order_acq_rel))
        return;

    const QString serverUrl = serverManager_->serverUrl();

    std::lock_guard<std::mutex> lock(jointPollThreadMutex_);
    if (jointPollThread_.joinable())
        jointPollThread_.join();

    jointPollThread_ = std::thread([this, serverUrl]() {
        const Ur3eJointsState result = ur3eGetJoints(serverUrl);
        if (!shutdownRequested_.load(std::memory_order_acquire) && result.ok
            && result.positionsRad.size() >= static_cast<std::size_t>(MainWindow::kUr3eJointCount))
        {
            QVariantList positions;
            positions.reserve(static_cast<int>(result.positionsRad.size()));
            for (const double value : result.positionsRad)
                positions.append(value);

            QMetaObject::invokeMethod(
                this,
                "applyPolledJoints",
                Qt::QueuedConnection,
                Q_ARG(QVariantList, positions),
                Q_ARG(QStringList, result.names));
        }
        jointPollInFlight_.store(false, std::memory_order_release);
    });
}

void Ur3ePanelController::onSyncJointsRequested()
{
    if (serverManager_ == nullptr || busy_ || !robotConnected_)
        return;

    pollJointsSync();
    syncTargetsFromCurrent();
    host_->appendLog(QStringLiteral("UR3e: joint targets synced from current pose."));
}

void Ur3ePanelController::onMoveRequested()
{
    if (serverManager_ == nullptr || busy_ || !robotConnected_)
        return;

    const QString serverUrl = serverManager_->serverUrl();

    std::vector<double> target;
    target.reserve(MainWindow::kUr3eJointCount);
    for (int jointIndex = 0; jointIndex < MainWindow::kUr3eJointCount; ++jointIndex)
    {
        ui::Ur3eJointBarWidget *bar = host_->ur3eJointBars_[jointIndex];
        target.push_back(bar != nullptr ? bar->valueRadians() : 0.0);
    }

    const QString targetSummary = formatJointTargetsDeg(target);
    host_->appendLog(
        QStringLiteral("UR3e: Move (joint, MoveIt) requested — %1").arg(targetSummary));
    stopRequested_.store(false, std::memory_order_release);
    motionInProgress_ = true;
    setBusy(true);
    setJointPollIntervalMs(kMotionPollIntervalMs);
    std::thread([this, serverUrl, target, targetSummary]() {
        const Ur3eScanWaypointMoveResult result =
            ur3eExecuteScanWaypoint(serverUrl, target, nullptr, nullptr, false, true);
        const bool ok = result.ok;
        QString detail;
        if (ok)
            detail = QStringLiteral("MoveIt move complete — %1").arg(targetSummary);
        else if (result.stopped)
            detail = result.errorMessage.isEmpty() ? QStringLiteral("Motion stopped.") : result.errorMessage;
        else if (result.skipped)
            detail = result.errorMessage.isEmpty()
                         ? QStringLiteral("Move skipped — no collision-free path.")
                         : result.errorMessage;
        else
            detail = result.errorMessage.isEmpty() ? QStringLiteral("MoveIt motion failed.") : result.errorMessage;
        QMetaObject::invokeMethod(
            this,
            [this, ok, detail]() { finishMove(ok, detail); },
            Qt::QueuedConnection);
    }).detach();
}

void Ur3ePanelController::onStartMoveItRequested()
{
    if (moveItManager_ == nullptr || busy_)
        return;

    if (!isSidecarRunning())
    {
        host_->appendLog(QStringLiteral("UR3e: sidecar not ready — cannot start MoveIt."));
        return;
    }

    if (!robotConnected_)
    {
        host_->appendLog(QStringLiteral(
            "UR3e: connect the robot first (MoveIt needs the running UR driver)."));
        return;
    }

    if (moveItManager_->isRunning())
    {
        host_->appendLog(QStringLiteral("UR3e: stopping MoveIt\u2026"));
        moveItManager_->stop();
        updateRobotUi();
        return;
    }

    if (rvizManager_->isRunning())
    {
        host_->appendLog(QStringLiteral("UR3e: stopping RViz (MoveIt launches its own window)\u2026"));
        rvizManager_->stop();
    }

    const Ur3eJointsState joints = ur3eGetJoints(serverManager_->serverUrl());
    if (!joints.ok || joints.positionsRad.size() < MainWindow::kUr3eJointCount)
    {
        host_->appendLog(QStringLiteral(
            "UR3e: /joint_states not available — connect the robot and wait for the driver "
            "before starting MoveIt."));
        return;
    }

    host_->appendLog(QStringLiteral("UR3e: starting MoveIt 2 + RViz in WSL\u2026"));
    moveItManager_->start();
    pushWorkspaceBoundaryToMoveIt();
    updateRobotUi();
}

void Ur3ePanelController::onPlanHemisphereScanRequested()
{
    if (host_->ur3eHemisphereScanSettings_ == nullptr || busy_)
        return;

    if (!robotConnected_ || serverManager_ == nullptr)
    {
        host_->appendLog(
            QStringLiteral("UR3e scan plan: connect the robot first (MoveIt needs the UR driver)."));
        return;
    }

    const bool semiMode =
        host_->ur3eHemisphereScanSettings_->scanExecuteMode() == Ur3eScanExecuteMode::SemiFixed;
    const Ur3eHemisphereScanParams scanParams =
        semiMode ? host_->ur3eHemisphereScanSettings_->semiPlanParams()
                 : host_->ur3eHemisphereScanSettings_->params();
    const Ur3eWorkspaceBoundary boundary =
        workspaceBoundaryFromConfig(hf::hardwareConfig().ur3e);
    const QString serverUrl = serverManager_->serverUrl();
    const double intervalDeg =
        host_->ur3eHemisphereScanSettings_->semiFixedRoute().intervalDeg;
    const int panDirection =
        host_->ur3eHemisphereScanSettings_->semiFixedRoute().panDirection;

    host_->appendLog(
        semiMode
            ? QStringLiteral(
                  "UR3e semi plan: MoveIt IK + base-sweep (first 2 OK pins / ring, "
                  "%1 φ candidates/ring)…")
                  .arg(hf::hardwareConfig().ur3e.semiRingSearchCandidates)
            : QStringLiteral("UR3e scan plan: running MoveIt IK + collision check…"));
    scanPlanning_ = true;
    setBusy(true);

    if (semiMode)
    {
        // Show geometric layer rings immediately while MoveIt runs.
        plannedScanPlan_ = Ur3eHemisphereScanPlan{};
        scanPlanReady_ = false;
        if (host_->ur3eHemisphereScanSettings_ != nullptr)
        {
            Ur3eSemiFixedRoute route = host_->ur3eHemisphereScanSettings_->semiFixedRoute();
            route.rings.clear();
            host_->ur3eHemisphereScanSettings_->setSemiFixedRoute(route);
        }
        refreshSemiFixedPreview();
    }

    std::thread([this, scanParams, boundary, serverUrl, semiMode, intervalDeg,
                 panDirection]() {
        QString errorMessage;
        const Ur3eHemisphereScanPlan plan =
            semiMode ? evaluateSemiHemisphereScanPlanMoveIt(serverUrl, scanParams, boundary, 2,
                                                            &errorMessage)
                     : evaluateHemisphereScanPlanMoveIt(serverUrl, scanParams, boundary,
                                                        &errorMessage);
        QMetaObject::invokeMethod(
            this,
            [this, plan, errorMessage, semiMode, intervalDeg, panDirection, scanParams]() {
                if (semiMode)
                    finishSemiScanPlan(plan, errorMessage, intervalDeg, panDirection, scanParams);
                else
                    finishScanPlan(plan, errorMessage);
            },
            Qt::QueuedConnection);
    }).detach();
}

void Ur3ePanelController::finishScanPlan(const Ur3eHemisphereScanPlan &plan,
                                         const QString &errorMessage)
{
    scanPlanning_ = false;
    setBusy(false);

    if (!errorMessage.isEmpty() && plan.points.empty())
    {
        host_->appendLog(QStringLiteral("UR3e scan plan failed: %1").arg(errorMessage));
        scanPlanReady_ = false;
        updateRobotUi();
        if (host_->capturePanel() != nullptr)
            host_->capturePanel()->syncBfsAndMultiviewRgbCaptureControls();
        return;
    }

    plannedScanPlan_ = plan;
    scanPlanReady_ = !plan.points.empty();

    if (host_->ur3eHemisphereScanSettings_ != nullptr)
        host_->ur3eHemisphereScanSettings_->setPlannedReachablePins(plan.reachableCount);

    host_->appendLog(
        QStringLiteral("UR3e scan plan (MoveIt): %1 points — %2 reachable "
                       "(%3 home→pin, %4 chain-only), %5 unreachable"
                       " (pin tip ±%6° vertical).")
            .arg(plan.points.size())
            .arg(plan.reachableCount)
            .arg(plan.homePathOkCount)
            .arg(plan.chainOnlyCount)
            .arg(plan.unreachableCount)
            .arg(hf::hardwareConfig().ur3e.pinPoseToleranceDeg, 0, 'f', 1));

    if (!plan.errorMessage.isEmpty())
        host_->appendLog(QStringLiteral("UR3e scan plan: %1").arg(plan.errorMessage));

    if (plan.points.empty())
    {
        updateRobotUi();
        if (host_->capturePanel() != nullptr)
            host_->capturePanel()->syncBfsAndMultiviewRgbCaptureControls();
        return;
    }

    if (plan.reachableCount == 0)
    {
        host_->appendLog(QStringLiteral(
            "UR3e scan plan: no collision-free IK poses — adjust grid, radius, or workspace."));
        appendScanPlanFailureReport(host_, plan);
    }
    else if (plan.unreachableCount > 0)
    {
        appendScanPlanFailureReport(host_, plan);
    }

    if (host_->ur3eScanRoutePlanWidget_ != nullptr)
    {
        const Ur3eHemisphereScanParams scanParams = host_->ur3eHemisphereScanSettings_->params();
        host_->ur3eScanRoutePlanWidget_->setScanParams(scanParams);
        host_->ur3eScanRoutePlanWidget_->setScanPlan(plan);
    }

    saveCachedScanPlan();

    updateRobotUi();
    if (host_->capturePanel() != nullptr)
        host_->capturePanel()->syncBfsAndMultiviewRgbCaptureControls();
}

void Ur3ePanelController::finishSemiScanPlan(const Ur3eHemisphereScanPlan &plan,
                                             const QString &errorMessage,
                                             const double intervalDeg,
                                             const int panDirection,
                                             const Ur3eHemisphereScanParams &scanParams)
{
    scanPlanning_ = false;
    setBusy(false);

    if (!errorMessage.isEmpty() && plan.points.empty())
    {
        host_->appendLog(QStringLiteral("UR3e semi plan failed: %1").arg(errorMessage));
        plannedScanPlan_ = Ur3eHemisphereScanPlan{};
        scanPlanReady_ = false;
        updateRobotUi();
        return;
    }

    plannedScanPlan_ = plan;
    scanPlanReady_ = false; // Semi execute uses route rings, not Auto pin order.

    int sweepOk = 0;
    int backupOk = 0;
    for (const Ur3ePlannedScanPoint &pt : plan.points)
    {
        if (!pt.reachable)
            continue;
        if (pt.baseSweepOk)
            ++sweepOk;
        else if (pt.backupCoverageOk)
            ++backupOk;
    }

    host_->appendLog(
        QStringLiteral("UR3e semi plan (MoveIt): %1 candidates — %2 reachable, "
                       "%3 base-sweep OK, %4 backup-coverage "
                       "(pin tip ±%5° vertical).")
            .arg(plan.points.size())
            .arg(plan.reachableCount)
            .arg(sweepOk)
            .arg(backupOk)
            .arg(hf::hardwareConfig().ur3e.pinPoseToleranceDeg, 0, 'f', 1));
    if (!plan.errorMessage.isEmpty())
        host_->appendLog(QStringLiteral("UR3e semi plan: %1").arg(plan.errorMessage));

    const QString robotFp = ur3eScanRobotCfgFingerprint(hf::hardwareConfig().ur3e);
    const QString displayName = defaultUr3eScanRouteDisplayName(scanParams)
                                + QStringLiteral(" semi");
    Ur3eSemiFixedRoute route =
        semiFixedRouteFromHemispherePlan(plan, robotFp, displayName, intervalDeg, panDirection);
    if (host_->ur3eHemisphereScanSettings_ != nullptr)
    {
        host_->ur3eHemisphereScanSettings_->setSemiFixedRoute(route);
        host_->ur3eHemisphereScanSettings_->setPlannedReachablePins(sweepOk + backupOk);
    }

    // Persist into Semi-only folder.
    const QString fingerprint = ur3eScanPlanFingerprint(hf::hardwareConfig().ur3e, scanParams);
    const QString safeStem =
        QStringLiteral("S%1_%2x%3_i%4_t%5-%6")
            .arg(qRound(scanParams.sphereRadiusM * 1000.0))
            .arg(scanParams.horizontalPoints)
            .arg(scanParams.verticalPoints)
            .arg(qRound(intervalDeg))
            .arg(qRound(scanParams.thetaMinDeg))
            .arg(qRound(scanParams.thetaMaxDeg));
    const QString routeDir = defaultUr3eSemiScanRoutesDir();
    QDir().mkpath(routeDir);
    const QString routePath = QDir(routeDir).filePath(safeStem + QStringLiteral(".json"));
    QString routeError;
    if (!saveUr3eNamedScanRoute(routePath, displayName, fingerprint, robotFp, scanParams, plan,
                                &routeError))
    {
        host_->appendLog(QStringLiteral("UR3e semi plan: save failed — %1").arg(routeError));
    }
    else
    {
        // Tag as Semi plan + imaging interval for reload.
        QFile f(routePath);
        if (f.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            const QJsonDocument doc = QJsonDocument::fromJson(f.readAll());
            f.close();
            if (doc.isObject())
            {
                QJsonObject root = doc.object();
                root.insert(QStringLiteral("kind"), QStringLiteral("ur3e_semi_hemisphere_plan"));
                root.insert(QStringLiteral("imaging_interval_deg"), intervalDeg);
                root.insert(QStringLiteral("pan_direction"), panDirection >= 0 ? 1 : -1);
                if (f.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text))
                {
                    f.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
                    f.close();
                }
            }
        }
        host_->appendLog(QStringLiteral("UR3e semi plan: saved \"%1\" (%2 ring(s)).")
                             .arg(displayName)
                             .arg(route.rings.size()));
        if (host_->ur3eHemisphereScanSettings_ != nullptr)
        {
            host_->ur3eHemisphereScanSettings_->rememberSemiFixedPlanPath(routePath);
            host_->ur3eHemisphereScanSettings_->refreshAvailableRoutes();
        }
    }

    refreshSemiFixedPreview();
    updateRobotUi();
    if (host_->capturePanel() != nullptr)
        host_->capturePanel()->syncBfsAndMultiviewRgbCaptureControls();
}

void Ur3ePanelController::saveCachedScanPlan()
{
    if (host_ == nullptr || host_->ur3eHemisphereScanSettings_ == nullptr)
        return;
    if (plannedScanPlan_.points.empty())
        return;

    const auto &ur3eCfg = hf::hardwareConfig().ur3e;
    const Ur3eHemisphereScanParams params = host_->ur3eHemisphereScanSettings_->params();
    const QString fingerprint = ur3eScanPlanFingerprint(ur3eCfg, params);
    const QString robotFp = ur3eScanRobotCfgFingerprint(ur3eCfg);

    // Always persist last plan + named route after a successful Plan.
    QString error;
    if (!saveUr3eScanPlanCache(defaultUr3eScanPlanCachePath(), fingerprint, plannedScanPlan_,
                               &error))
    {
        host_->appendLog(QStringLiteral("UR3e scan plan cache: save failed — %1").arg(error));
    }
    else
    {
        host_->appendLog(QStringLiteral("UR3e scan plan cache: saved (%1 points).")
                             .arg(plannedScanPlan_.points.size()));
    }

    // Named library entry (loadable when robot cfg still matches).
    const QString displayName = defaultUr3eScanRouteDisplayName(params);
    const QString safeStem =
        QStringLiteral("R%1_%2x%3_t%4-%5")
            .arg(qRound(params.sphereRadiusM * 1000.0))
            .arg(params.horizontalPoints)
            .arg(params.verticalPoints)
            .arg(qRound(params.thetaMinDeg))
            .arg(qRound(params.thetaMaxDeg));
    const QString routePath =
        QDir(defaultUr3eScanRoutesDir()).filePath(safeStem + QStringLiteral(".json"));
    QString routeError;
    if (!saveUr3eNamedScanRoute(routePath,
                                displayName,
                                fingerprint,
                                robotFp,
                                params,
                                plannedScanPlan_,
                                &routeError))
    {
        host_->appendLog(QStringLiteral("UR3e scan route: save failed — %1").arg(routeError));
    }
    else
    {
        host_->appendLog(QStringLiteral("UR3e scan route: saved \"%1\".").arg(displayName));
        host_->ur3eHemisphereScanSettings_->refreshAvailableRoutes();
    }
}

void Ur3ePanelController::onLoadScanRouteRequested(const QString &routePath)
{
    if (host_ == nullptr || host_->ur3eHemisphereScanSettings_ == nullptr)
        return;
    if (routePath.trimmed().isEmpty())
        return;

    const QString robotFp = ur3eScanRobotCfgFingerprint(hf::hardwareConfig().ur3e);
    Ur3eHemisphereScanPlan plan;
    Ur3eHemisphereScanParams routeParams;
    QString displayName;
    QString error;
    if (!loadUr3eNamedScanRoute(routePath, robotFp, plan, &routeParams, &displayName, &error))
    {
        host_->appendLog(QStringLiteral("UR3e scan route: not loaded — %1").arg(error));
        return;
    }

    // Apply route grid to UI, then install planned joints.
    host_->ur3eHemisphereScanSettings_->setParams(routeParams);
    plannedScanPlan_ = plan;
    scanPlanReady_ = plan.reachableCount > 0;
    host_->ur3eHemisphereScanSettings_->setPlannedReachablePins(plan.reachableCount);
    host_->ur3eHemisphereScanSettings_->rememberAutoRoutePath(routePath);

    host_->appendLog(
        QStringLiteral("UR3e scan route: loaded \"%1\" — %2 points (%3 reachable: "
                       "%4 home→pin, %5 chain-only; %6 unreachable).")
            .arg(displayName)
            .arg(plan.points.size())
            .arg(plan.reachableCount)
            .arg(plan.homePathOkCount)
            .arg(plan.chainOnlyCount)
            .arg(plan.unreachableCount));
    if (host_->ur3eScanRoutePlanWidget_ != nullptr)
    {
        host_->ur3eScanRoutePlanWidget_->setScanParams(routeParams);
        host_->ur3eScanRoutePlanWidget_->setScanPlan(plan);
    }
    host_->ur3eHemisphereScanSettings_->refreshAvailableRoutes();
    updateRobotUi();
    if (host_->capturePanel() != nullptr)
        host_->capturePanel()->syncBfsAndMultiviewRgbCaptureControls();
}

void Ur3ePanelController::onLoadPlannedRouteAsSemiFixedRequested(const QString &routePath)
{
    if (host_ == nullptr || host_->ur3eHemisphereScanSettings_ == nullptr)
        return;
    if (routePath.trimmed().isEmpty())
        return;

    const QString robotFp = ur3eScanRobotCfgFingerprint(hf::hardwareConfig().ur3e);
    const bool fppMode =
        host_->ur3eHemisphereScanSettings_->scanExecuteMode() == Ur3eScanExecuteMode::Fpp;

    // FPP (kind "fpp") or Semi (kind "ur3e_semi_fixed_route") — load rings/top/stage.
    {
        Ur3eSemiFixedRoute hand;
        QString handErr;
        if (loadUr3eSemiFixedRoute(routePath, robotFp, hand, &handErr))
        {
            if (fppMode)
                hand.isFppPlan = true;
            host_->ur3eHemisphereScanSettings_->setSemiFixedRoute(hand);
            host_->ur3eHemisphereScanSettings_->setPlannedReachablePins(
                std::max(1, static_cast<int>(hand.rings.size()) + (hand.hasTopPose ? 1 : 0)));
            if (fppMode)
                host_->ur3eHemisphereScanSettings_->rememberFppPlanPath(routePath);
            else
                host_->ur3eHemisphereScanSettings_->rememberSemiFixedPlanPath(routePath);
            refreshSemiFixedPreview();
            host_->appendLog(
                QStringLiteral("UR3e %1: loaded \"%2\" → home + %3 ring(s) "
                               "(spin uses GUI Range/Interval/Direction; stage uses GUI Stage).")
                    .arg(fppMode ? QStringLiteral("FPP") : QStringLiteral("semi-fixed"))
                    .arg(hand.displayName)
                    .arg(hand.rings.size()));
            updateRobotUi();
            if (host_->capturePanel() != nullptr)
                host_->capturePanel()->syncBfsAndMultiviewRgbCaptureControls();
            return;
        }
        if (fppMode)
        {
            host_->appendLog(
                QStringLiteral("UR3e FPP: plan not loaded — %1").arg(handErr));
            return;
        }
    }

    Ur3eHemisphereScanPlan plan;
    Ur3eHemisphereScanParams routeParams;
    QString displayName;
    QString error;
    if (!loadUr3eNamedScanRoute(routePath, robotFp, plan, &routeParams, &displayName, &error))
    {
        host_->appendLog(
            QStringLiteral("UR3e semi-fixed: Semi plan not loaded — %1").arg(error));
        return;
    }

    double intervalDeg =
        host_->ur3eHemisphereScanSettings_->semiFixedRoute().intervalDeg;
    int panDir = host_->ur3eHemisphereScanSettings_->semiFixedRoute().panDirection;
    double panRangeDeg = host_->ur3eHemisphereScanSettings_->semiFixedRoute().panRangeDeg;
    QFile metaFile(routePath);
    if (metaFile.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        const QJsonDocument doc = QJsonDocument::fromJson(metaFile.readAll());
        metaFile.close();
        if (doc.isObject())
        {
            const QJsonObject root = doc.object();
            if (root.contains(QStringLiteral("imaging_interval_deg")))
                intervalDeg = root.value(QStringLiteral("imaging_interval_deg")).toDouble(intervalDeg);
            if (root.contains(QStringLiteral("pan_direction")))
                panDir = root.value(QStringLiteral("pan_direction")).toInt(panDir);
            if (root.contains(QStringLiteral("pan_range_deg")))
                panRangeDeg = root.value(QStringLiteral("pan_range_deg")).toDouble(panRangeDeg);
            else if (root.contains(QStringLiteral("interval_deg")))
                intervalDeg = root.value(QStringLiteral("interval_deg")).toDouble(intervalDeg);
        }
    }

    Ur3eSemiFixedRoute semi = semiFixedRouteFromHemispherePlan(
        plan, robotFp, displayName, intervalDeg, panDir);
    semi.panRangeDeg = panRangeDeg >= 0.0 ? std::min(360.0, panRangeDeg) : 360.0;

    // Layer + θ from plan latitudes (scan_params can be stale / defaults).
    syncHemisphereParamsFromPlanLatitudes(plan, routeParams);

    if (semi.rings.isEmpty() && !semi.hasTopPose)
    {
        host_->appendLog(QStringLiteral(
            "UR3e semi-fixed: Semi plan has no sweep-OK or backup-coverage ring entries."));
        // Still show all latitudes (unreachable in blue) when the plan has points.
        if (!plan.points.empty())
        {
            plannedScanPlan_ = plan;
            scanPlanReady_ = false;
            if (host_->ur3eHemisphereScanSettings_ != nullptr)
            {
                host_->ur3eHemisphereScanSettings_->applyLoadedSemiPlanSettings(
                    routeParams, intervalDeg, panDir, panRangeDeg);
            }
            refreshSemiFixedPreview();
        }
        return;
    }

    plannedScanPlan_ = plan;
    scanPlanReady_ = plan.reachableCount > 0;

    host_->ur3eHemisphereScanSettings_->applyLoadedSemiPlanSettings(routeParams, intervalDeg,
                                                                    panDir, panRangeDeg);
    host_->ur3eHemisphereScanSettings_->setSemiFixedRoute(semi);
    host_->ur3eHemisphereScanSettings_->setPlannedReachablePins(
        std::max(1, plan.reachableCount));
    if (host_->ur3eHemisphereScanSettings_->scanExecuteMode() == Ur3eScanExecuteMode::Fpp)
        host_->ur3eHemisphereScanSettings_->rememberFppPlanPath(routePath);
    else
        host_->ur3eHemisphereScanSettings_->rememberSemiFixedPlanPath(routePath);
    if (host_->ur3eScanRoutePlanWidget_ != nullptr)
        host_->ur3eScanRoutePlanWidget_->setScanParams(
            host_->ur3eHemisphereScanSettings_->semiPlanParams());
    refreshSemiFixedPreview();
    const bool apexOnly =
        semi.hasTopPose
        && (semi.rings.isEmpty()
            || (semi.rings.size() == 1
                && (semi.rings[0].noPan || std::abs(semi.rings[0].thetaDeg) < 0.75)));
    if (apexOnly)
    {
        host_->appendLog(
            QStringLiteral("UR3e semi-fixed: loaded Semi plan \"%1\" → apex only (executable).")
                .arg(displayName));
    }
    else
    {
        host_->appendLog(
            QStringLiteral("UR3e semi-fixed: loaded Semi plan \"%1\" → %2 ring(s) "
                           "(Layer %3, interval %4°, θ %5–%6°).")
                .arg(displayName)
                .arg(semi.rings.size())
                .arg(routeParams.verticalPoints)
                .arg(intervalDeg, 0, 'f', 1)
                .arg(routeParams.thetaMinDeg, 0, 'f', 0)
                .arg(routeParams.thetaMaxDeg, 0, 'f', 0));
    }
    updateRobotUi();
    if (host_->capturePanel() != nullptr)
        host_->capturePanel()->syncBfsAndMultiviewRgbCaptureControls();
}

void Ur3ePanelController::tryLoadCachedScanPlan()
{
    if (host_ == nullptr || host_->ur3eHemisphereScanSettings_ == nullptr)
        return;

    host_->ur3eHemisphereScanSettings_->refreshAvailableRoutes();

    // Restore last Semi-fixed GUI selection (mode already restored from QSettings).
    if (host_->ur3eHemisphereScanSettings_->rememberLastPlan()
        && isSavedRingRouteMode(host_->ur3eHemisphereScanSettings_->scanExecuteMode()))
    {
        const QString semiRoutePath =
            host_->ur3eHemisphereScanSettings_->rememberedSemiFixedRoutePath();
        if (!semiRoutePath.isEmpty() && QFile::exists(semiRoutePath))
        {
            Ur3eSemiFixedRoute route;
            QString err;
            const QString fp = ur3eScanRobotCfgFingerprint(hf::hardwareConfig().ur3e);
            if (loadUr3eSemiFixedRoute(semiRoutePath, fp, route, &err))
            {
                host_->ur3eHemisphereScanSettings_->setSemiFixedRoute(route);
                refreshSemiFixedPreview();
                host_->appendLog(
                    QStringLiteral("UR3e semi-fixed: restored last route \"%1\" (%2 ring(s)).")
                        .arg(route.displayName)
                        .arg(route.rings.size()));
                updateRobotUi();
                if (host_->capturePanel() != nullptr)
                    host_->capturePanel()->syncBfsAndMultiviewRgbCaptureControls();
                return;
            }
        }

        const QString planPath =
            host_->ur3eHemisphereScanSettings_->scanExecuteMode() == Ur3eScanExecuteMode::Fpp
                ? host_->ur3eHemisphereScanSettings_->rememberedFppPlanPath()
                : host_->ur3eHemisphereScanSettings_->rememberedSemiFixedPlanPath();
        if (!planPath.isEmpty() && QFile::exists(planPath))
        {
            onLoadPlannedRouteAsSemiFixedRequested(planPath);
            return;
        }
    }

    // Auto: prefer last named route when remember-last is on.
    if (host_->ur3eHemisphereScanSettings_->rememberLastPlan()
        && host_->ur3eHemisphereScanSettings_->scanExecuteMode()
               == Ur3eScanExecuteMode::AutoHemisphere)
    {
        const QString autoPath =
            host_->ur3eHemisphereScanSettings_->rememberedAutoRoutePath();
        if (!autoPath.isEmpty() && QFile::exists(autoPath))
        {
            onLoadScanRouteRequested(autoPath);
            return;
        }
    }

    const QString fingerprint = ur3eScanPlanFingerprint(hf::hardwareConfig().ur3e,
                                                        host_->ur3eHemisphereScanSettings_->params());
    Ur3eHemisphereScanPlan plan;
    QString error;
    if (!loadUr3eScanPlanCache(defaultUr3eScanPlanCachePath(), fingerprint, plan, &error))
    {
        if (QFile::exists(defaultUr3eScanPlanCachePath()))
            host_->appendLog(QStringLiteral("UR3e scan plan cache: not loaded — %1").arg(error));
        return;
    }

    plannedScanPlan_ = plan;
    scanPlanReady_ = plan.reachableCount > 0;

    if (host_->ur3eHemisphereScanSettings_ != nullptr)
        host_->ur3eHemisphereScanSettings_->setPlannedReachablePins(plan.reachableCount);

    host_->appendLog(
        QStringLiteral("UR3e scan plan cache: loaded %1 points (%2 reachable: "
                       "%3 home→pin, %4 chain-only; %5 unreachable).")
            .arg(plan.points.size())
            .arg(plan.reachableCount)
            .arg(plan.homePathOkCount)
            .arg(plan.chainOnlyCount)
            .arg(plan.unreachableCount));

    if (host_->ur3eScanRoutePlanWidget_ != nullptr)
    {
        host_->ur3eScanRoutePlanWidget_->setScanParams(
            host_->ur3eHemisphereScanSettings_->params());
        host_->ur3eScanRoutePlanWidget_->setScanPlan(plan);
    }

    updateRobotUi();
    if (host_->capturePanel() != nullptr)
        host_->capturePanel()->syncBfsAndMultiviewRgbCaptureControls();
}

void Ur3ePanelController::onExecuteHemisphereScanRequested()
{
    const bool bfsConnected =
        host_ != nullptr && host_->bfsPanel() != nullptr
        && host_->bfsPanel()->isCameraConnected();

    if (!bfsConnected)
    {
        startHemisphereScanExecute({});
        return;
    }

    const QString parentDir = QFileDialog::getExistingDirectory(
        host_,
        QStringLiteral("Save Multiview images"),
        AppSettingsStore::suggestedCaptureSaveStartDir(),
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (parentDir.isEmpty())
    {
        host_->appendLog(QStringLiteral("UR3e scan execute: cancelled (no save folder)."));
        return;
    }

    const QString stamp =
        QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss"));
    const QString captureDir =
        QDir(parentDir).filePath(QStringLiteral("multiview_%1").arg(stamp));
    if (!QDir().mkpath(captureDir))
    {
        host_->appendLog(
            QStringLiteral("UR3e scan execute rejected: could not create %1").arg(captureDir));
        QMessageBox::warning(host_,
                             QStringLiteral("UR3e Scan Execute"),
                             QStringLiteral("Could not create folder:\n%1").arg(captureDir));
        return;
    }

    HemisphereScanExecuteOptions opts;
    opts.captureOutputDir = captureDir;
    opts.stabilizeMs = hf::hardwareConfig().ur3e.scanCaptureStabilizeMs;
    if (!startHemisphereScanExecute(opts))
    {
        host_->appendLog(QStringLiteral("UR3e scan execute: capture start rejected."));
    }
}

bool Ur3ePanelController::startHemisphereScanExecute(const HemisphereScanExecuteOptions &options)
{
    if (host_ == nullptr || host_->ur3eHemisphereScanSettings_ == nullptr || busy_
        || !robotConnected_ || serverManager_ == nullptr || scanExecuting_)
        return false;

    if (host_->ur3eHemisphereScanSettings_->scanExecuteMode() == Ur3eScanExecuteMode::SemiFixed
        || host_->ur3eHemisphereScanSettings_->scanExecuteMode() == Ur3eScanExecuteMode::Fpp)
        return startSemiFixedScanExecute(options);

    if (!scanPlanReady_ || plannedScanPlan_.reachableCount == 0)
    {
        host_->appendLog(
            QStringLiteral("UR3e scan execute rejected: plan a route with reachable points first."));
        return false;
    }

    std::vector<int> order = buildHemisphereScanExecutionOrder(plannedScanPlan_);
    if (options.pinSet != HemisphereScanPinSet::All)
    {
        std::vector<int> filtered;
        filtered.reserve(order.size());
        for (const int index : order)
        {
            if (index < 0 || index >= static_cast<int>(plannedScanPlan_.points.size()))
                continue;
            const bool apex =
                std::abs(plannedScanPlan_.points[static_cast<std::size_t>(index)].gridPoint.thetaDeg)
                <= 1.0e-9;
            if (options.pinSet == HemisphereScanPinSet::ApexOnly && apex)
                filtered.push_back(index);
            else if (options.pinSet == HemisphereScanPinSet::RingsOnly && !apex)
                filtered.push_back(index);
        }
        order = std::move(filtered);
    }
    // Two full-360° pins on one latitude: keep one (simple path). Backup pairs stay.
    {
        QHash<int, bool> sweepTaken;
        std::vector<int> simple;
        simple.reserve(order.size());
        for (const int index : order)
        {
            if (index < 0 || index >= static_cast<int>(plannedScanPlan_.points.size()))
                continue;
            const Ur3ePlannedScanPoint &pt =
                plannedScanPlan_.points[static_cast<std::size_t>(index)];
            if (std::abs(pt.gridPoint.thetaDeg) < 0.75)
            {
                simple.push_back(index);
                continue;
            }
            if (pt.baseSweepOk)
            {
                const int key = static_cast<int>(std::lround(pt.gridPoint.thetaDeg * 2.0));
                if (sweepTaken.contains(key))
                    continue;
                sweepTaken.insert(key, true);
            }
            simple.push_back(index);
        }
        order = std::move(simple);
    }
    if (order.empty())
    {
        host_->appendLog(QStringLiteral("UR3e scan execute rejected: no stored joint solutions."));
        return false;
    }

    const QString captureDir = options.captureOutputDir.trimmed();
    const bool captureStills = !captureDir.isEmpty();
    const hf::HardwareConfig::Ur3eConfig &ur3eCfg = hf::hardwareConfig().ur3e;
    const int stabilizeMs =
        options.stabilizeMs > 0 ? options.stabilizeMs
                                : (ur3eCfg.scanCaptureStabilizeMs >= 0 ? ur3eCfg.scanCaptureStabilizeMs
                                                                      : kScanCaptureStabilizeMs);
    Ur3eWristSweepParams wristSweep;
    if (host_->ur3eHemisphereScanSettings_ != nullptr)
        wristSweep = host_->ur3eHemisphereScanSettings_->wristSweepParams();
    else
    {
        wristSweep.enabled = ur3eCfg.scanWristSweepEnabled;
        wristSweep.stepDeg = ur3eCfg.scanWristSweepStepDeg;
        wristSweep.stepsEachWay = ur3eCfg.scanWristSweepStepsEachWay;
        wristSweep.wrist1 = ur3eCfg.scanWristSweepWrist1;
        wristSweep.wrist2 = ur3eCfg.scanWristSweepWrist2;
        wristSweep.wrist3 = ur3eCfg.scanWristSweepWrist3;
    }
    const int wristPosesPerPin = wristSweep.imagesPerPin();

    if (captureStills)
    {
        if (host_->bfsPanel() == nullptr || !host_->bfsPanel()->isCameraConnected())
        {
            host_->appendLog(QStringLiteral(
                "UR3e scan execute rejected: BFS camera must be connected for Multiview capture."));
            return false;
        }
        QDir().mkpath(captureDir);
    }

    QString wristSummary;
    if (wristSweep.enabled && wristSweep.enabledAxisCount() > 0)
    {
        QStringList axes;
        if (wristSweep.wrist1)
            axes << QStringLiteral("w1");
        if (wristSweep.wrist2)
            axes << QStringLiteral("w2");
        if (wristSweep.wrist3)
            axes << QStringLiteral("w3");
        wristSummary = QStringLiteral(", wrist %1 ±%2×%3° → %4 pose(s)/pin")
                           .arg(axes.join(QLatin1Char('+')))
                           .arg(wristSweep.stepsEachWay)
                           .arg(wristSweep.stepDeg, 0, 'f', 0)
                           .arg(wristPosesPerPin);
    }

    const QString serverUrl = serverManager_->serverUrl();
    const bool driveStage = options.pinSet == HemisphereScanPinSet::All;
    const bool useStage = driveStage && stageConnectedForScan(host_);
    const double stageMm = host_->ur3eHemisphereScanSettings_->stagePositionMm();
    QString captureNote = captureStills ? QStringLiteral(", BFS stills → ") + captureDir
                                        : QStringLiteral(", motion-only");
    if (captureStills && host_->dlpPanel() != nullptr && host_->dlpPanel()->isConnected())
    {
        captureNote += QStringLiteral(", FPP %1 patterns × %2 new + %3 settle BFS frame(s)")
                           .arg(hf::dlp::kFppScanningStepCount)
                           .arg(hf::dlp::kFppCaptureMinNewFrames)
                           .arg(hf::dlp::kFppCaptureStabilizeFrames);
    }
    if (useStage)
    {
        captureNote += QStringLiteral(", stage %1 mm").arg(stageMm, 0, 'f', 0);
    }
    else if (driveStage)
    {
        captureNote += QStringLiteral(", stage not connected (GUI Stage ignored)");
    }
    host_->appendLog(
        QStringLiteral("UR3e scan execute: %1 reachable point(s), top-ring-first clockwise sweep "
                       "(%2 ms settle%3%4)…")
            .arg(order.size())
            .arg(stabilizeMs)
            .arg(captureNote)
            .arg(wristSummary));
    stopRequested_.store(false, std::memory_order_release);
    scanExecuting_ = true;
    scanExecuteSuppressUiSummary_ = options.suppressUiSummary;
    if (host_->ur3eScanRoutePlanWidget_ != nullptr)
    {
        const int plannedPins =
            static_cast<int>(order.size()) * std::max(1, wristPosesPerPin);
        host_->ur3eScanRoutePlanWidget_->beginScanExecution(plannedPins);
    }
    setBusy(true);
    setJointPollIntervalMs(kMotionPollIntervalMs);

    const Ur3eHemisphereScanPlan planCopy = plannedScanPlan_;
    const int sessionId = ++scanExecuteSessionId_;

    {
        std::lock_guard<std::mutex> lock(scanExecuteThreadMutex_);
        if (scanExecuteThread_.joinable())
            scanExecuteThread_.join();

        const int startFrameIndex = std::max(0, options.startFrameIndex);
        const bool appendTransformsJson = options.appendTransformsJson;

        scanExecuteThread_ = std::thread([this,
                                          serverUrl,
                                          order,
                                          planCopy,
                                          sessionId,
                                          captureDir,
                                          captureStills,
                                          stabilizeMs,
                                          wristSweep,
                                          startFrameIndex,
                                          appendTransformsJson,
                                          useStage,
                                          stageMm]() {
            int executed = 0;
            int skipped = 0;
            int captured = startFrameIndex;
            int wristSkipped = 0;
            QString errorMessage;
            bool ok = true;
            const int total = static_cast<int>(order.size());
            TransformsJsonDocument transformsDoc;
            const std::vector<double> wristOffsets =
                wristSweepOffsetsRad(wristSweep.stepsEachWay, wristSweep.stepDeg);
            const std::vector<double> axis1 =
                wristSweep.wrist1 ? wristOffsets : std::vector<double>{0.0};
            const std::vector<double> axis2 =
                wristSweep.wrist2 ? wristOffsets : std::vector<double>{0.0};
            const std::vector<double> axis3 =
                wristSweep.wrist3 ? wristOffsets : std::vector<double>{0.0};

            const auto sessionActive = [this, sessionId]() {
                return !shutdownRequested_.load(std::memory_order_acquire)
                       && sessionId == scanExecuteSessionId_.load(std::memory_order_acquire);
            };
            const auto captureContinue = [this, &sessionActive]() {
                return sessionActive() && !stopRequested_.load(std::memory_order_acquire);
            };

            const auto finishWithCapture = [this, &transformsDoc, captureStills, captureDir,
                                            appendTransformsJson, useStage, &sessionActive,
                                            stageMm](
                                               bool finishOk,
                                               const QString &finishError,
                                               int executedCount,
                                               bool stopped,
                                               int capturedCount,
                                               qint64 elapsedMs) {
                parkStageAtMvsAfterExecuteIfUsed(host_, useStage, sessionActive, stageMm);
                if (captureStills && !transformsDoc.frames.empty())
                {
                    QString writeError;
                    if (!writeTransformsJson(captureDir, transformsDoc, &writeError,
                                             appendTransformsJson))
                    {
                        QMetaObject::invokeMethod(
                            this,
                            [this, writeError]() {
                                host_->appendLog(
                                    QStringLiteral("UR3e scan capture: transforms.json failed — %1")
                                        .arg(writeError));
                            },
                            Qt::QueuedConnection);
                    }
                    else
                    {
                        QMetaObject::invokeMethod(
                            this,
                            [this, captureDir, capturedCount]() {
                                host_->appendLog(
                                    QStringLiteral(
                                        "UR3e scan capture: wrote %1 frame(s) + transforms.json → %2")
                                        .arg(capturedCount)
                                        .arg(captureDir));
                            },
                            Qt::QueuedConnection);
                    }
                }
                QMetaObject::invokeMethod(
                    this,
                    "scanExecuteFinish",
                    Qt::QueuedConnection,
                    Q_ARG(bool, finishOk),
                    Q_ARG(QString, finishError),
                    Q_ARG(int, executedCount),
                    Q_ARG(bool, stopped),
                    Q_ARG(int, capturedCount),
                    Q_ARG(qint64, elapsedMs));
            };

            const auto captureStillAtPose = [&](const Ur3eScanTcpPose &plannedTcp,
                                                const int pointIndex) -> bool {
                if (!captureStills)
                    return true;
                return capturePinStillsMaybeFpp(
                    this,
                    host_,
                    serverUrl,
                    captureDir,
                    plannedTcp,
                    &captured,
                    &transformsDoc,
                    &ok,
                    &errorMessage,
                    QStringLiteral("pin %1").arg(pointIndex),
                    captureContinue,
                    OutputPoseShift{});
            };

            QMetaObject::invokeMethod(
                this,
                [this]() {
                    host_->appendLog(
                        QStringLiteral("UR3e scan execute: verifying scan home before scan…"));
                    syncHomeJointTargetSliders();
                },
                Qt::BlockingQueuedConnection);

            const HomeEnsureOutcome preHomeOutcome =
                ensureRobotAtHomeSync(HomeEnsureContext::BeforeScanExecute);
            if (preHomeOutcome.cancelled)
            {
                finishWithCapture(false,
                                  QStringLiteral("Scan aborted — homing cancelled."),
                                  0,
                                  false,
                                  0,
                                  0);
                return;
            }

            QMetaObject::invokeMethod(
                this, &Ur3ePanelController::syncHomeJointTargetSliders, Qt::QueuedConnection);

            bool returnHomeAfterScan = true;
            const auto scanStartedAt = std::chrono::steady_clock::now();
            const auto stageSessionOk = [this, &sessionActive]() {
                return sessionActive() && !stopRequested_.load(std::memory_order_acquire);
            };
            if (useStage)
            {
                QString stageErr;
                if (!waitMoveStageAbsolute(host_, stageMm, QStringLiteral("stage"),
                                           stageSessionOk, &stageErr))
                {
                    finishWithCapture(false, stageErr, 0, false, captured, 0);
                    return;
                }
            }

            // Clear wrist_3 wind at home when |live−home|≥180°. Used before and after pins
            // so we never approach a pin already multi-turn wound (BFS USB cable risk).
            const auto runWrist3CableRewind = [&](const QString &whenLabel) -> bool {
                if (!sessionActive() || stopRequested_.load(std::memory_order_acquire))
                    return false;
                const Ur3eWrist3RewindResult rewind = ur3eRewindWrist3Cable(serverUrl);
                if (rewind.stopped)
                {
                    if (stopRequested_.load(std::memory_order_acquire))
                        ur3eStopMotion(serverUrl);
                    return false;
                }
                if (rewind.ok && rewind.rewound)
                {
                    const int turns = rewind.turns;
                    QMetaObject::invokeMethod(
                        this,
                        [this, turns, whenLabel]() {
                            host_->appendLog(
                                QStringLiteral(
                                    "UR3e scan: wrist_3 cable rewind (%1 turn(s), "
                                    "threshold ±180°) at home %2.")
                                    .arg(turns > 0 ? QStringLiteral("+%1").arg(turns)
                                                   : QString::number(turns))
                                    .arg(whenLabel));
                            syncHomeJointTargetSliders();
                        },
                        Qt::QueuedConnection);
                }
                else if (!rewind.ok)
                {
                    const QString reason = rewind.errorMessage.isEmpty()
                                              ? QStringLiteral("rewind failed")
                                              : rewind.errorMessage;
                    QMetaObject::invokeMethod(
                        this,
                        [this, reason]() {
                            host_->appendLog(
                                QStringLiteral(
                                    "UR3e scan warning: wrist_3 cable rewind — %1")
                                    .arg(reason));
                        },
                        Qt::QueuedConnection);
                }
                return true;
            };

            for (int step = 0; step < total; ++step)
            {
                if (!sessionActive() || stopRequested_.load(std::memory_order_acquire))
                {
                    ur3eStopMotion(serverUrl);
                    break;
                }

                const int pointIndex = order[static_cast<std::size_t>(step)];
                const Ur3ePlannedScanPoint &point =
                    planCopy.points[static_cast<std::size_t>(pointIndex)];
                const QString tcpSummary =
                    QStringLiteral("tcp=(%1, %2, %3) m theta=%4° phi=%5°")
                        .arg(point.tcp.xM, 0, 'f', 3)
                        .arg(point.tcp.yM, 0, 'f', 3)
                        .arg(point.tcp.zM, 0, 'f', 3)
                        .arg(point.gridPoint.thetaDeg, 0, 'f', 1)
                        .arg(point.gridPoint.phiDeg, 0, 'f', 1);
                const QString targetSummary = formatJointTargetsDeg(point.jointPositionsRad);

                QVariantList targetPositionsVariant;
                targetPositionsVariant.reserve(
                    static_cast<int>(point.jointPositionsRad.size()));
                for (const double value : point.jointPositionsRad)
                    targetPositionsVariant.append(value);

                if (!sessionActive())
                    break;

                if (!runWrist3CableRewind(QStringLiteral("before pin move")))
                    goto scan_execute_loop_done;

                QMetaObject::invokeMethod(
                    this,
                    "scanExecuteSetActivePoint",
                    Qt::QueuedConnection,
                    Q_ARG(int, pointIndex));
                QMetaObject::invokeMethod(
                    this,
                    "scanExecuteLogMoving",
                    Qt::QueuedConnection,
                    Q_ARG(int, step + 1),
                    Q_ARG(int, total),
                    Q_ARG(int, pointIndex),
                    Q_ARG(QString, tcpSummary),
                    Q_ARG(QString, targetSummary),
                    Q_ARG(QVariantList, targetPositionsVariant));

                bool homeThenPin = false;
                if (step > 0)
                {
                    const int prevIndex = order[static_cast<std::size_t>(step - 1)];
                    if (prevIndex >= 0
                        && prevIndex < static_cast<int>(planCopy.points.size()))
                    {
                        const Ur3ePlannedScanPoint &prev =
                            planCopy.points[static_cast<std::size_t>(prevIndex)];
                        // Via-home for backup 2-pin pairs, and when elbow family
                        // flips (direct PTP folds the payload through the arm).
                        const bool backupPair = sameHemisphereScanRing(prev, point)
                                                && !prev.baseSweepOk && !point.baseSweepOk;
                        const bool elbowFlip = !sameElbowFamily(
                            prev.jointPositionsRad, point.jointPositionsRad);
                        homeThenPin = backupPair || elbowFlip;
                    }
                }
                if (homeThenPin)
                {
                    QMetaObject::invokeMethod(
                        this,
                        [this, pointIndex]() {
                            host_->appendLog(
                                QStringLiteral(
                                    "UR3e scan execute: via home, then pin %1")
                                    .arg(pointIndex));
                        },
                        Qt::QueuedConnection);
                }
                const Ur3eScanWaypointMoveResult moveResult = ur3eExecuteScanWaypoint(
                    serverUrl,
                    point.jointPositionsRad,
                    &point.tcp,
                    nullptr,
                    homeThenPin,
                    false);
                if (moveResult.stopped)
                {
                    if (stopRequested_.load(std::memory_order_acquire))
                    {
                        ur3eStopMotion(serverUrl);
                    }
                    else
                    {
                        ok = false;
                        errorMessage = moveResult.errorMessage.isEmpty()
                                           ? QStringLiteral("MoveIt execution stopped before motion started.")
                                           : moveResult.errorMessage;
                    }
                    break;
                }
                if (moveResult.skipped)
                {
                    const int stepNum = step + 1;
                    const QString reason = moveResult.errorMessage.isEmpty()
                                               ? QStringLiteral("no collision-free path")
                                               : moveResult.errorMessage;
                    QMetaObject::invokeMethod(
                        this,
                        [this, stepNum, total, pointIndex, reason]() {
                            host_->appendLog(
                                QStringLiteral(
                                    "UR3e scan execute [%1/%2] pt %3: skipped — %4")
                                    .arg(stepNum)
                                    .arg(total)
                                    .arg(pointIndex)
                                    .arg(reason));
                        },
                        Qt::QueuedConnection);
                    QMetaObject::invokeMethod(
                        this,
                        "scanExecuteMarkFailed",
                        Qt::QueuedConnection,
                        Q_ARG(int, pointIndex));
                    // Entire base pin skipped → all center+sweep imaging poses failed.
                    const int poses = std::max(1, wristSweep.imagesPerPin());
                    QMetaObject::invokeMethod(
                        this,
                        [this, poses]() {
                            if (host_->ur3eScanRoutePlanWidget_ == nullptr)
                                return;
                            for (int i = 0; i < poses; ++i)
                                host_->ur3eScanRoutePlanWidget_->markPinFailed();
                        },
                        Qt::QueuedConnection);
                    ++skipped;
                    continue;
                }
                if (!moveResult.ok)
                {
                    ok = false;
                    errorMessage = moveResult.errorMessage;
                    // Still retreat home after a pin move failure (operator expects a known pose).
                    returnHomeAfterScan = true;
                    break;
                }

                if (!sessionActive())
                    break;

                const Ur3eJointsState joints = ur3eGetJoints(serverUrl);
                const Ur3ePoseResult pose = ur3eGetTcpPose(serverUrl);
                const QString arrivedJoints = joints.ok ? formatJointTargetsDeg(joints.positionsRad)
                                                        : QStringLiteral("(n/a)");
                const QString arrivedPose =
                    pose.ok ? formatTcpPose(pose.pose) : QStringLiteral("(n/a)");

                QVariantList positionsVariant;
                if (joints.ok)
                {
                    positionsVariant.reserve(static_cast<int>(joints.positionsRad.size()));
                    for (const double value : joints.positionsRad)
                        positionsVariant.append(value);
                }

                QMetaObject::invokeMethod(
                    this,
                    "scanExecuteLogArrived",
                    Qt::QueuedConnection,
                    Q_ARG(int, step + 1),
                    Q_ARG(int, total),
                    Q_ARG(int, pointIndex),
                    Q_ARG(QString, arrivedPose),
                    Q_ARG(QString, arrivedJoints),
                    Q_ARG(QVariantList, positionsVariant),
                    Q_ARG(QStringList, joints.names));

                std::this_thread::sleep_for(std::chrono::milliseconds(stabilizeMs));

                if (!sessionActive() || stopRequested_.load(std::memory_order_acquire))
                {
                    ur3eStopMotion(serverUrl);
                    break;
                }

                if (!captureStillAtPose(point.tcp, pointIndex))
                {
                    returnHomeAfterScan = true;
                    break;
                }

                // Count each imaging pose (center + wrist sweeps) for the progress bar.
                ++executed;
                QMetaObject::invokeMethod(
                    this,
                    [this]() {
                        if (host_->ur3eScanRoutePlanWidget_ != nullptr)
                            host_->ur3eScanRoutePlanWidget_->markPinCompleted();
                    },
                    Qt::QueuedConnection);

                if (wristSweep.enabled && wristSweep.enabledAxisCount() > 0
                    && point.jointPositionsRad.size() >= 6)
                {
                    for (const double dWrist1 : axis1)
                    {
                        for (const double dWrist2 : axis2)
                        {
                            for (const double dWrist3 : axis3)
                            {
                                if (dWrist1 == 0.0 && dWrist2 == 0.0 && dWrist3 == 0.0)
                                    continue; // center already captured

                                if (!sessionActive()
                                    || stopRequested_.load(std::memory_order_acquire))
                                {
                                    ur3eStopMotion(serverUrl);
                                    goto scan_execute_loop_done;
                                }

                                std::vector<double> wristJoints = point.jointPositionsRad;
                                wristJoints[3] += dWrist1;
                                wristJoints[4] += dWrist2;
                                wristJoints[5] += dWrist3;

                                // Hardware (not MoveIt): small wrist deltas at an already-reached
                                // pin — same policy as Semi pan / return-to-pin.
                                QString wristErr;
                                const Ur3eScanWaypointMoveResult wristMove =
                                    ur3eExecuteHardwareJointMove(
                                        serverUrl,
                                        wristJoints,
                                        true,
                                        &wristErr,
                                        QStringLiteral("auto wrist sweep offset"));
                                if (wristMove.stopped)
                                {
                                    if (stopRequested_.load(std::memory_order_acquire))
                                        ur3eStopMotion(serverUrl);
                                    else
                                    {
                                        ok = false;
                                        errorMessage = wristErr.isEmpty()
                                                           ? QStringLiteral(
                                                                 "Wrist sweep stopped before motion.")
                                                           : wristErr;
                                    }
                                    goto scan_execute_loop_done;
                                }
                                if (!wristMove.ok)
                                {
                                    ++wristSkipped;
                                    const QString reason =
                                        wristErr.isEmpty()
                                            ? QStringLiteral("hardware move failed")
                                            : wristErr;
                                    constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
                                    const double w1Deg = dWrist1 * kRadToDeg;
                                    const double w2Deg = dWrist2 * kRadToDeg;
                                    const double w3Deg = dWrist3 * kRadToDeg;
                                    QMetaObject::invokeMethod(
                                        this,
                                        [this, pointIndex, w1Deg, w2Deg, w3Deg, reason]() {
                                            host_->appendLog(
                                                QStringLiteral(
                                                    "UR3e scan wrist sweep pin %1: skip "
                                                    "Δw1=%2° Δw2=%3° Δw3=%4° — %5")
                                                    .arg(pointIndex)
                                                    .arg(w1Deg, 0, 'f', 1)
                                                    .arg(w2Deg, 0, 'f', 1)
                                                    .arg(w3Deg, 0, 'f', 1)
                                                    .arg(reason));
                                            if (host_->ur3eScanRoutePlanWidget_ != nullptr)
                                                host_->ur3eScanRoutePlanWidget_->markPinFailed();
                                        },
                                        Qt::QueuedConnection);
                                    continue;
                                }

                                std::this_thread::sleep_for(
                                    std::chrono::milliseconds(stabilizeMs));

                                if (!sessionActive()
                                    || stopRequested_.load(std::memory_order_acquire))
                                {
                                    ur3eStopMotion(serverUrl);
                                    goto scan_execute_loop_done;
                                }

                                if (!captureStillAtPose(point.tcp, pointIndex))
                                {
                                    // Restore nominal before abort so home retreat is not wrist-offset.
                                    ur3eExecuteHardwareJointMove(
                                        serverUrl,
                                        point.jointPositionsRad,
                                        true,
                                        nullptr,
                                        QStringLiteral("auto return-to-pin after sweep abort"));
                                    returnHomeAfterScan = true;
                                    goto scan_execute_loop_done;
                                }
                                QMetaObject::invokeMethod(
                                    this,
                                    [this]() {
                                        if (host_->ur3eScanRoutePlanWidget_ != nullptr)
                                            host_->ur3eScanRoutePlanWidget_->markPinCompleted();
                                    },
                                    Qt::QueuedConnection);
                            }
                        }
                    }

                    // Always restore nominal pin joints before the next pin (hardware —
                    // only wrists moved; do not continue from an offset pose).
                    if (sessionActive() && !stopRequested_.load(std::memory_order_acquire))
                    {
                        QMetaObject::invokeMethod(
                            this,
                            [this, pointIndex]() {
                                host_->appendLog(
                                    QStringLiteral(
                                        "UR3e scan wrist sweep pin %1: returning to "
                                        "nominal pin pose…")
                                        .arg(pointIndex));
                            },
                            Qt::QueuedConnection);

                        QString returnErr;
                        const Ur3eScanWaypointMoveResult returnPin =
                            ur3eExecuteHardwareJointMove(
                                serverUrl,
                                point.jointPositionsRad,
                                true,
                                &returnErr,
                                QStringLiteral("auto return-to-pin after wrist sweep"));
                        if (returnPin.stopped)
                        {
                            if (stopRequested_.load(std::memory_order_acquire))
                                ur3eStopMotion(serverUrl);
                            goto scan_execute_loop_done;
                        }
                        if (!returnPin.ok || returnPin.skipped)
                        {
                            ok = false;
                            errorMessage = returnErr.isEmpty()
                                               ? QStringLiteral(
                                                     "return to nominal pin after wrist sweep failed")
                                               : returnErr;
                            QMetaObject::invokeMethod(
                                this,
                                [this, pointIndex, reason = errorMessage]() {
                                    host_->appendLog(
                                        QStringLiteral(
                                            "UR3e scan wrist sweep pin %1: return-to-pin "
                                            "failed — %2 (aborting; will not start next pin "
                                            "from offset)")
                                            .arg(pointIndex)
                                            .arg(reason));
                                },
                                Qt::QueuedConnection);
                            returnHomeAfterScan = true;
                            goto scan_execute_loop_done;
                        }
                        QMetaObject::invokeMethod(
                            this,
                            [this, pointIndex]() {
                                host_->appendLog(
                                    QStringLiteral(
                                        "UR3e scan wrist sweep pin %1: back at "
                                        "nominal pin pose.")
                                        .arg(pointIndex));
                            },
                            Qt::QueuedConnection);
                        std::this_thread::sleep_for(
                            std::chrono::milliseconds(stabilizeMs));
                    }
                    else if (sessionActive())
                    {
                        // Stop mid-sweep: still try to recenter wrists before home retreat.
                        ur3eExecuteHardwareJointMove(
                            serverUrl,
                            point.jointPositionsRad,
                            true,
                            nullptr,
                            QStringLiteral("auto return-to-pin after stop"));
                    }
                }

                QMetaObject::invokeMethod(
                    this,
                    "scanExecuteMarkCompleted",
                    Qt::QueuedConnection,
                    Q_ARG(int, pointIndex));

                // After pin (+ wrist sweep): clear any wind accumulated on this pin.
                if (!runWrist3CableRewind(QStringLiteral("before next move")))
                    goto scan_execute_loop_done;
            }

        scan_execute_loop_done:
            if (!sessionActive())
                return;

            // User Stop must always retreat to home after cancelling the current motion.
            const bool userStopped = stopRequested_.load(std::memory_order_acquire);
            if (userStopped)
                returnHomeAfterScan = true;

            if (returnHomeAfterScan)
            {
                scanReturningHome_.store(true, std::memory_order_release);
                // Allow MoveIt home even though Stop was pressed (sidecar stop latch cleared
                // inside /execute_move_home). Extra Stop presses are ignored while this flag
                // is set so they cannot cancel the retreat.
                stopRequested_.store(false, std::memory_order_release);

                QMetaObject::invokeMethod(
                    this,
                    [this, userStopped]() {
                        host_->appendLog(
                            userStopped
                                ? QStringLiteral(
                                      "UR3e scan execute: stop — returning to home pose…")
                                : QStringLiteral(
                                      "UR3e scan execute: returning to home pose…"));
                        syncHomeJointTargetSliders();
                    },
                    Qt::BlockingQueuedConnection);

                // Sidecar post_scan_home retries + ignore_stop; C++ ignores extra Stop presses.
                const Ur3eScanWaypointMoveResult postHomeResult = ur3eExecuteMoveHome(serverUrl);

                scanReturningHome_.store(false, std::memory_order_release);

                if (postHomeResult.ok)
                {
                    QMetaObject::invokeMethod(
                        this,
                        &Ur3ePanelController::syncHomeJointTargetSliders,
                        Qt::QueuedConnection);
                }
                else
                {
                    const QString reason = postHomeResult.errorMessage.isEmpty()
                                               ? QStringLiteral("could not return to home")
                                               : postHomeResult.errorMessage;
                    QMetaObject::invokeMethod(
                        this,
                        [this, reason]() {
                            host_->appendLog(
                                QStringLiteral("UR3e scan execute warning: %1").arg(reason));
                        },
                        Qt::QueuedConnection);
                }
            }

            if (skipped > 0)
            {
                const int skippedCount = skipped;
                QMetaObject::invokeMethod(
                    this,
                    [this, skippedCount]() {
                        host_->appendLog(
                            QStringLiteral(
                                "UR3e scan execute: %1 pin(s) skipped (no collision-free path).")
                                .arg(skippedCount));
                    },
                    Qt::QueuedConnection);
            }
            if (wristSkipped > 0)
            {
                const int wristSkippedCount = wristSkipped;
                QMetaObject::invokeMethod(
                    this,
                    [this, wristSkippedCount]() {
                        host_->appendLog(
                            QStringLiteral(
                                "UR3e scan execute: %1 wrist pose(s) skipped (collision / no path).")
                                .arg(wristSkippedCount));
                    },
                    Qt::QueuedConnection);
            }

            const bool stopped = stopRequested_.load(std::memory_order_acquire);
            const qint64 elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                        std::chrono::steady_clock::now() - scanStartedAt)
                                        .count();
            finishWithCapture(ok, errorMessage, executed, stopped || userStopped, captured,
                              elapsedMs);
        });
    }

    return true;
}

void Ur3ePanelController::refreshSemiFixedPreview()
{
    if (host_ == nullptr || host_->ur3eHemisphereScanSettings_ == nullptr
        || host_->ur3eScanRoutePlanWidget_ == nullptr)
        return;
    if (host_->ur3eHemisphereScanSettings_->scanExecuteMode()
        != Ur3eScanExecuteMode::SemiFixed
        && host_->ur3eHemisphereScanSettings_->scanExecuteMode() != Ur3eScanExecuteMode::Fpp)
        return;

    QVector<Ur3eSemiFixedPreviewRing> rings;
    const Ur3eHemisphereScanParams params =
        host_->ur3eHemisphereScanSettings_->semiPlanParams();
    if (host_->ur3eScanRoutePlanWidget_ != nullptr)
        host_->ur3eScanRoutePlanWidget_->setScanParams(params);
    const Ur3eSemiFixedRoute route = host_->ur3eHemisphereScanSettings_->semiFixedRoute();
    const bool fppMode =
        host_->ur3eHemisphereScanSettings_->scanExecuteMode() == Ur3eScanExecuteMode::Fpp;
    // Canned default top is always injected; only real planned/added rings skip params.
    if (!route.rings.isEmpty() || (fppMode && (route.hasTopPose || route.hasRgbRing
                                               || route.hasRgbHome)))
        rings = fppMode ? inferFppPreviewPins(route) : inferSemiFixedPreviewRings(route);
    else if (!plannedScanPlan_.points.empty())
        rings = previewSemiFixedRingsFromHemispherePlan(plannedScanPlan_);
    else
        rings = previewSemiFixedRingsFromScanParams(params);
    host_->ur3eScanRoutePlanWidget_->setSemiFixedPreviewRings(rings);
}

void Ur3ePanelController::onAddSemiFixedRingRequested()
{
    if (host_ == nullptr || host_->ur3eHemisphereScanSettings_ == nullptr || !robotConnected_
        || serverManager_ == nullptr)
    {
        if (host_ != nullptr)
            host_->appendLog(QStringLiteral("UR3e semi-fixed: connect robot before adding a ring."));
        return;
    }

    const QString serverUrl = serverManager_->serverUrl();
    const Ur3eJointsState joints = ur3eGetJoints(serverUrl);
    if (!joints.ok || joints.positionsRad.size() != 6)
    {
        host_->appendLog(QStringLiteral("UR3e semi-fixed: could not read joints — %1")
                             .arg(joints.errorMessage));
        return;
    }

    Ur3eScanTcpPose tcp{};
    bool hasTcp = false;
    const Ur3ePoseResult pose = ur3eGetTcpPose(serverUrl);
    if (pose.ok)
    {
        tcp = scanTcpFromLivePose(pose.pose, Ur3eScanTcpPose{});
        hasTcp = true;
    }

    host_->ur3eHemisphereScanSettings_->appendSemiFixedRing(joints.positionsRad, tcp, hasTcp);
    host_->appendLog(QStringLiteral("UR3e semi-fixed: added ring entry from current pose."));
    refreshSemiFixedPreview();
    updateRobotUi();
}

bool Ur3ePanelController::startSemiFixedScanExecute(const HemisphereScanExecuteOptions &options)
{
    if (host_ == nullptr || host_->ur3eHemisphereScanSettings_ == nullptr || busy_
        || !robotConnected_ || serverManager_ == nullptr || scanExecuting_)
        return false;

    const Ur3eSemiFixedRoute routeRaw = host_->ur3eHemisphereScanSettings_->semiFixedRoute();
    Ur3eSemiFixedRoute route = routeRaw;
    pruneSemiFixedRedundantFullSpinPins(route);
    ensureSemiFixedTopPose(route);
    host_->ur3eHemisphereScanSettings_->setSemiFixedRoute(route);
    if (route.rings.isEmpty() && !route.hasTopPose && !route.hasRgbRing
        && !route.hasRgbHome)
    {
        host_->appendLog(
            QStringLiteral("UR3e semi-fixed execute rejected: add a ring or load an apex plan."));
        return false;
    }

    const QString captureDir = options.captureOutputDir.trimmed();
    const bool captureStills = !captureDir.isEmpty();
    const hf::HardwareConfig::Ur3eConfig &ur3eCfg = hf::hardwareConfig().ur3e;
    const int stabilizeMs =
        options.stabilizeMs > 0
            ? options.stabilizeMs
            : (ur3eCfg.scanCaptureStabilizeMs >= 0 ? ur3eCfg.scanCaptureStabilizeMs
                                                   : kScanCaptureStabilizeMs);

    if (captureStills)
    {
        if (host_->bfsPanel() == nullptr || !host_->bfsPanel()->isCameraConnected())
        {
            host_->appendLog(QStringLiteral(
                "UR3e semi-fixed execute rejected: BFS camera must be connected for Multiview capture."));
            return false;
        }
        QDir().mkpath(captureDir);
    }

    refreshSemiFixedPreview();

    const QString serverUrl = serverManager_->serverUrl();
    const bool driveStage = options.pinSet == HemisphereScanPinSet::All;
    const bool useStage = driveStage && stageConnectedForScan(host_);
    const bool fppMode =
        host_->ur3eHemisphereScanSettings_->scanExecuteMode() == Ur3eScanExecuteMode::Fpp;
    const double stageMm = host_->ur3eHemisphereScanSettings_->stagePositionMm();
    if (useStage)
    {
        host_->appendLog(QStringLiteral("UR3e %1: stage %2 mm")
                             .arg(fppMode ? QStringLiteral("FPP") : QStringLiteral("semi-fixed"))
                             .arg(stageMm, 0, 'f', 0));
    }
    else if (driveStage)
    {
        host_->appendLog(QStringLiteral(
            "UR3e Multiview: stage not connected (GUI Stage ignored)"));
    }
    stopRequested_.store(false, std::memory_order_release);
    scanExecuting_ = true;
    scanExecuteSuppressUiSummary_ = options.suppressUiSummary;

    double guiExposureUs = 15005.0;
    hf::bfs::BfsCameraSettings bfsUiSettings{};
    hf::bfs::BfsCameraSettings dlpCaptureSettings{};
    hf::bfs::BfsCameraSettings rgbCaptureSettings{};
    bool haveBfsCaptureSettings = false;
    if (host_->bfsPanel() != nullptr)
    {
        host_->bfsPanel()->setCaptureSettingsOverride(std::nullopt);
        if (host_->bfsPanel()->isCameraConnected())
        {
            // Snapshot on the GUI thread — never read QWidgets from the scan worker.
            bfsUiSettings = host_->bfsPanel()->settingsFromUi();
            guiExposureUs = bfsUiSettings.exposureTimeUs > 0.0 ? bfsUiSettings.exposureTimeUs
                                                              : guiExposureUs;
            haveBfsCaptureSettings = true;
        }
    }
    const double dlpExposureUs =
        route.hasDlpExposure && route.dlpExposureUs > 0.0 ? route.dlpExposureUs
                                                          : guiExposureUs;
    const double rgbExposureUs =
        route.hasRgbExposure && route.rgbExposureUs > 0.0 ? route.rgbExposureUs
                                                          : guiExposureUs;
    if (haveBfsCaptureSettings)
    {
        dlpCaptureSettings = hf::bfs::BfsPanelController::settingsWithCaptureExposure(
            bfsUiSettings, dlpExposureUs);
        rgbCaptureSettings = hf::bfs::BfsPanelController::settingsWithCaptureExposure(
            bfsUiSettings, rgbExposureUs);
    }
    if (captureStills && host_->bfsPanel() != nullptr && host_->bfsPanel()->isCameraConnected())
    {
        host_->appendLog(
            QStringLiteral("UR3e FPP/semi: BFS exposure DLP %1 µs%2, RGB %3 µs%4")
                .arg(dlpExposureUs, 0, 'f', 0)
                .arg(route.hasDlpExposure ? QStringLiteral(" (plan)")
                                          : QStringLiteral(" (GUI)"))
                .arg(rgbExposureUs, 0, 'f', 0)
                .arg(route.hasRgbExposure ? QStringLiteral(" (plan)")
                                          : QStringLiteral(" (GUI)")));

        // Apply DLP exposure on the GUI thread before motion/capture (avoids Spinnaker
        // stop/start racing the first FPP burst on the worker thread).
        const auto exposureAlreadySet = [](const hf::bfs::BfsCameraSettings &cur,
                                           const hf::bfs::BfsCameraSettings &want) {
            return cur.exposureMode.compare(want.exposureMode, Qt::CaseInsensitive) == 0
                   && cur.exposureAuto.compare(want.exposureAuto, Qt::CaseInsensitive) == 0
                   && std::abs(cur.exposureTimeUs - want.exposureTimeUs) < 0.5;
        };
        if (!exposureAlreadySet(bfsUiSettings, dlpCaptureSettings))
        {
            QString applyErr;
            if (!host_->bfsPanel()->applyCameraSettingsBlocking(dlpCaptureSettings, &applyErr))
            {
                host_->appendLog(QStringLiteral("UR3e: BFS DLP exposure apply failed — %1")
                                     .arg(applyErr));
                scanExecuting_ = false;
                setBusy(false);
                return false;
            }
            const std::uint64_t after = host_->bfsPanel()->lastFrameIndex();
            host_->bfsPanel()->waitForNewerFrame(after, 1, 4000);
            host_->appendLog(QStringLiteral("UR3e: BFS exposure → %1 µs (DLP/FPP)")
                                 .arg(dlpExposureUs, 0, 'f', 0));
        }
        else
        {
            host_->bfsPanel()->setCaptureSettingsOverride(dlpCaptureSettings);
            host_->appendLog(QStringLiteral(
                                 "UR3e: BFS exposure already %1 µs (DLP/FPP) — skip re-apply")
                                 .arg(dlpExposureUs, 0, 'f', 0));
        }
    }

    Ur3eWristSweepParams wristSweep{};
    if (host_->ur3eHemisphereScanSettings_ != nullptr)
        wristSweep = host_->ur3eHemisphereScanSettings_->wristSweepParams();
    if (host_->ur3eScanRoutePlanWidget_ != nullptr)
    {
        const double intervalDeg =
            route.intervalDeg > 0.0 ? route.intervalDeg : 10.0;
        int ringSamples = 0;
        for (const Ur3eSemiFixedRing &ring : route.rings)
        {
            ringSamples += semiFixedRingSampleCount(
                ring,
                resolveRingIntervalDeg(ring, intervalDeg),
                resolveRingPanRangeDeg(ring, route.panRangeDeg));
        }
        if (route.hasRgbRing)
        {
            ringSamples += semiFixedRingSampleCount(
                route.rgbRing,
                resolveRingIntervalDeg(route.rgbRing, intervalDeg),
                resolveRingPanRangeDeg(route.rgbRing, route.panRangeDeg));
        }
        const bool skipTop = options.pinSet == HemisphereScanPinSet::RingsOnly;
        const bool skipRings = options.pinSet == HemisphereScanPinSet::ApexOnly;
        const bool apexRingOnly =
            route.rings.size() == 1
            && (route.rings[0].noPan || std::abs(route.rings[0].thetaDeg) < 0.75);
        const bool doTop =
            !skipTop && route.hasTopPose && !(apexRingOnly && !skipRings);
        const int topSamples = doTop ? 1 : 0;
        const int basePins = topSamples + (skipRings ? 0 : ringSamples);
        const int plannedPins = basePins * std::max(1, wristSweep.imagesPerPin());
        host_->ur3eScanRoutePlanWidget_->beginScanExecution(plannedPins);
    }
    setBusy(true);
    setJointPollIntervalMs(kMotionPollIntervalMs);

    const int sessionId = ++scanExecuteSessionId_;

    if (wristSweep.enabled && wristSweep.enabledAxisCount() > 0)
    {
        QString axes;
        if (wristSweep.wrist1)
            axes += QStringLiteral("w1 ");
        if (wristSweep.wrist2)
            axes += QStringLiteral("w2 ");
        if (wristSweep.wrist3)
            axes += QStringLiteral("w3 ");
        host_->appendLog(QStringLiteral(
                             "UR3e semi-fixed: wrist sweep ON (%1±%2° %3)")
                             .arg(wristSweep.stepDeg, 0, 'f', 0)
                             .arg(wristSweep.stepsEachWay)
                             .arg(axes.trimmed()));
    }
    else
    {
        host_->appendLog(QStringLiteral("UR3e semi-fixed: wrist sweep OFF"));
    }

    {
        std::lock_guard<std::mutex> lock(scanExecuteThreadMutex_);
        if (scanExecuteThread_.joinable())
            scanExecuteThread_.join();

        const int startFrameIndex = std::max(0, options.startFrameIndex);
        const bool appendTransformsJson = options.appendTransformsJson;
        const bool skipTop = options.pinSet == HemisphereScanPinSet::RingsOnly;
        const bool skipRings = options.pinSet == HemisphereScanPinSet::ApexOnly;

        scanExecuteThread_ = std::thread([this, serverUrl, route, sessionId, captureDir,
                                          captureStills, stabilizeMs, wristSweep,
                                          startFrameIndex, appendTransformsJson, skipTop,
                                          skipRings, useStage, stageMm, rgbCaptureSettings,
                                          haveBfsCaptureSettings, rgbExposureUs]() {
            int captured = startFrameIndex;
            TransformsJsonDocument transformsDoc;
            bool rgbExposureApplied = false;

            const auto sessionActive = [this, sessionId]() {
                return !shutdownRequested_.load(std::memory_order_acquire)
                       && sessionId == scanExecuteSessionId_.load(std::memory_order_acquire);
            };
            const auto captureContinue = [this, &sessionActive]() {
                return sessionActive() && !stopRequested_.load(std::memory_order_acquire);
            };

            const auto clearBfsCaptureOverride = [this]() {
                QMetaObject::invokeMethod(
                    this,
                    [this]() {
                        if (host_ != nullptr && host_->bfsPanel() != nullptr)
                            host_->bfsPanel()->setCaptureSettingsOverride(std::nullopt);
                    },
                    Qt::QueuedConnection);
            };

            const auto ensureBfsExposure = [this, captureStills, &rgbExposureApplied,
                                           rgbCaptureSettings, haveBfsCaptureSettings,
                                           rgbExposureUs, &captureContinue](
                                              const bool rgbOnly,
                                              QString *errorOut) -> bool {
                if (!captureStills || !haveBfsCaptureSettings)
                    return true;
                hf::bfs::BfsPanelController *bfs =
                    host_ != nullptr ? host_->bfsPanel() : nullptr;
                if (bfs == nullptr || !bfs->isCameraConnected())
                    return true;
                if (!rgbOnly)
                    return true; // DLP exposure applied on GUI thread before this worker.
                if (rgbExposureApplied)
                    return true;

                // Apply RGB exposure on the GUI thread (Spinnaker + override bookkeeping).
                bool applyOk = false;
                QString applyErr;
                QMetaObject::invokeMethod(
                    this,
                    [bfs, rgbCaptureSettings, &applyOk, &applyErr]() {
                        applyOk = bfs->applyCameraSettingsBlocking(rgbCaptureSettings, &applyErr);
                    },
                    Qt::BlockingQueuedConnection);
                if (!applyOk)
                {
                    if (errorOut != nullptr)
                    {
                        *errorOut =
                            QStringLiteral("BFS RGB exposure %1 µs failed: %2")
                                .arg(rgbExposureUs, 0, 'f', 0)
                                .arg(applyErr);
                    }
                    return false;
                }
                const std::uint64_t after = bfs->lastFrameIndex();
                bfs->waitForNewerFrame(after, 1, 4000, nullptr, [&captureContinue]() {
                    return !captureContinue();
                });
                QMetaObject::invokeMethod(
                    this,
                    [this, rgbExposureUs]() {
                        host_->appendLog(QStringLiteral("UR3e: BFS exposure → %1 µs (RGB)")
                                             .arg(rgbExposureUs, 0, 'f', 0));
                    },
                    Qt::QueuedConnection);
                rgbExposureApplied = true;
                return true;
            };

            const auto finishWithCapture = [this, &transformsDoc, captureStills, captureDir,
                                            &captured, appendTransformsJson, useStage,
                                            &sessionActive, stageMm, clearBfsCaptureOverride](
                                               bool finishOk, const QString &finishError,
                                               int executedCount, bool stopped,
                                               int /*capturedFromExec*/,
                                               qint64 elapsedMs) {
                clearBfsCaptureOverride();
                parkStageAtMvsAfterExecuteIfUsed(host_, useStage, sessionActive, stageMm);
                if (captureStills && !transformsDoc.frames.empty())
                {
                    QString writeError;
                    if (!writeTransformsJson(captureDir, transformsDoc, &writeError,
                                             appendTransformsJson))
                    {
                        QMetaObject::invokeMethod(
                            this,
                            [this, writeError]() {
                                host_->appendLog(
                                    QStringLiteral(
                                        "UR3e semi-fixed capture: transforms.json failed — %1")
                                        .arg(writeError));
                            },
                            Qt::QueuedConnection);
                    }
                }
                QMetaObject::invokeMethod(this, "scanExecuteFinish", Qt::QueuedConnection,
                                          Q_ARG(bool, finishOk), Q_ARG(QString, finishError),
                                          Q_ARG(int, executedCount), Q_ARG(bool, stopped),
                                          Q_ARG(int, captured), Q_ARG(qint64, elapsedMs));
            };

            // Mirror execute: rgb_home then rgb_ring appended after FPP rings.
            Ur3eSemiFixedRoute captureRoute = route;
            foldRgbEntriesIntoRings(captureRoute);

            const auto captureStillAtPose = [&](const Ur3eScanTcpPose &plannedTcp,
                                                const int ringIndex,
                                                const int sampleIndex) -> bool {
                if (!captureStills)
                    return true;
                const QString skipContext =
                    ringIndex < 0
                        ? QStringLiteral("top")
                        : QStringLiteral("ring %1 sample %2").arg(ringIndex).arg(sampleIndex);
                bool rgbOnly = false;
                if (ringIndex >= 0 && ringIndex < captureRoute.rings.size())
                {
                    rgbOnly = captureRoute.rings[ringIndex].captureKind.trimmed().compare(
                                  QStringLiteral("rgb"), Qt::CaseInsensitive)
                              == 0;
                }
                QString exposureErr;
                if (!ensureBfsExposure(rgbOnly, &exposureErr))
                {
                    QMetaObject::invokeMethod(
                        this,
                        [this, exposureErr]() {
                            host_->appendLog(QStringLiteral("UR3e capture aborted: %1")
                                                 .arg(exposureErr));
                        },
                        Qt::QueuedConnection);
                    return false;
                }
                return capturePinStillsMaybeFpp(this,
                                                host_,
                                                serverUrl,
                                                captureDir,
                                                plannedTcp,
                                                &captured,
                                                &transformsDoc,
                                                nullptr,
                                                nullptr,
                                                skipContext,
                                                captureContinue,
                                                OutputPoseShift{},
                                                rgbOnly);
            };

            SemiFixedScanExecuteInput input;
            input.serverUrl = serverUrl;
            input.route = route;
            input.captureDir = captureDir;
            input.stabilizeMs = stabilizeMs;
            input.sessionId = sessionId;
            input.wristSweep = wristSweep;
            input.skipTop = skipTop;
            input.skipRings = skipRings;
            input.stageMm = stageMm;

            SemiFixedScanExecuteHost hostHooks;
            hostHooks.sessionActive = sessionActive;
            hostHooks.stopRequested = [this]() {
                return stopRequested_.load(std::memory_order_acquire);
            };
            hostHooks.clearStopRequested = [this]() {
                stopRequested_.store(false, std::memory_order_release);
            };
            hostHooks.setReturningHome = [this](const bool on) {
                scanReturningHome_.store(on, std::memory_order_release);
            };
            // Run on the worker thread (same as Auto). ensureRobotAtHomeSync may
            // BlockingQueued syncHomeJointTargetSliders — nesting that on the UI thread
            // deadlocks (Windows then kills the hung app.exe).
            hostHooks.ensureHomeBeforeScan = [this]() {
                const HomeEnsureOutcome outcome =
                    ensureRobotAtHomeSync(HomeEnsureContext::BeforeScanExecute);
                return !outcome.cancelled;
            };
            hostHooks.syncHomeSliders = [this]() {
                QMetaObject::invokeMethod(this, &Ur3ePanelController::syncHomeJointTargetSliders,
                                          Qt::QueuedConnection);
            };
            hostHooks.log = [this](const QString &msg) {
                QMetaObject::invokeMethod(
                    this, [this, msg]() { host_->appendLog(msg); }, Qt::QueuedConnection);
            };
            hostHooks.setActiveRing = [this](const int ringIndex) {
                QMetaObject::invokeMethod(this, "scanExecuteSetActivePoint", Qt::QueuedConnection,
                                          Q_ARG(int, ringIndex));
            };
            hostHooks.markRingCompleted = [this](const int ringIndex) {
                QMetaObject::invokeMethod(this, "scanExecuteMarkCompleted", Qt::QueuedConnection,
                                          Q_ARG(int, ringIndex));
            };
            hostHooks.markRingFailed = [this](const int ringIndex) {
                QMetaObject::invokeMethod(this, "scanExecuteMarkFailed", Qt::QueuedConnection,
                                          Q_ARG(int, ringIndex));
            };
            hostHooks.markPinCompleted = [this]() {
                QMetaObject::invokeMethod(
                    this,
                    [this]() {
                        if (host_ != nullptr && host_->ur3eScanRoutePlanWidget_ != nullptr)
                            host_->ur3eScanRoutePlanWidget_->markPinCompleted();
                    },
                    Qt::QueuedConnection);
            };
            hostHooks.markPinFailed = [this]() {
                QMetaObject::invokeMethod(
                    this,
                    [this]() {
                        if (host_ != nullptr && host_->ur3eScanRoutePlanWidget_ != nullptr)
                            host_->ur3eScanRoutePlanWidget_->markPinFailed();
                    },
                    Qt::QueuedConnection);
            };
            hostHooks.captureStill = captureStillAtPose;
            hostHooks.blankProjector = [this](QString *errorOut) -> bool {
                hf::dlp::DlpPanelController *dlp =
                    host_ != nullptr ? host_->dlpPanel() : nullptr;
                if (dlp == nullptr || !dlp->isConnected())
                    return true;
                return dlp->blankSync(errorOut);
            };
            hostHooks.confirmContinueRgb = [this]() -> bool {
                bool continueRgb = false;
                QMetaObject::invokeMethod(
                    this,
                    [this, &continueRgb]() {
                        if (host_ == nullptr)
                        {
                            continueRgb = false;
                            return;
                        }
                        QMessageBox box(host_);
                        box.setIcon(QMessageBox::Question);
                        box.setWindowTitle(QStringLiteral("Continue RGB scanning?"));
                        box.setText(QStringLiteral("FPP capture finished. DLP is blanked."));
                        box.setInformativeText(
                            QStringLiteral(
                                "Continue with the RGB color sweep now?\n\n"
                                "Choose Skip to finish the scan and return home."));
                        box.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
                        box.setDefaultButton(QMessageBox::Yes);
                        if (QAbstractButton *yesButton = box.button(QMessageBox::Yes))
                            yesButton->setText(QStringLiteral("Continue RGB"));
                        if (QAbstractButton *noButton = box.button(QMessageBox::No))
                            noButton->setText(QStringLiteral("Skip RGB"));
                        continueRgb = box.exec() == QMessageBox::Yes;
                    },
                    Qt::BlockingQueuedConnection);
                return continueRgb;
            };
            if (useStage)
            {
                hostHooks.moveStage = [this, &sessionActive](const double targetMm,
                                                             const QString &label,
                                                             QString *errorOut) {
                    return waitMoveStageAbsolute(
                        host_,
                        targetMm,
                        label,
                        [this, &sessionActive]() {
                            return sessionActive()
                                   && !stopRequested_.load(std::memory_order_acquire);
                        },
                        errorOut);
                };
            }
            hostHooks.finish = finishWithCapture;

            runSemiFixedScanExecute(input, hostHooks);
        });
    }

    return true;
}

void Ur3ePanelController::scanExecuteSetActivePoint(const int pointIndex)
{
    if (!scanExecuting_)
        return;

    if (host_->ur3eScanRoutePlanWidget_ != nullptr)
        host_->ur3eScanRoutePlanWidget_->setActiveScanPoint(pointIndex);
}

void Ur3ePanelController::scanExecuteMarkCompleted(const int pointIndex)
{
    if (!scanExecuting_)
        return;

    // Preview ring/pin color only — imaging-pose progress is marked per center/sweep.
    if (host_->ur3eScanRoutePlanWidget_ != nullptr)
        host_->ur3eScanRoutePlanWidget_->markScanPointCompleted(pointIndex);
}

void Ur3ePanelController::scanExecuteMarkFailed(const int pointIndex)
{
    if (!scanExecuting_)
        return;

    if (host_->ur3eScanRoutePlanWidget_ != nullptr)
        host_->ur3eScanRoutePlanWidget_->markScanPointFailed(pointIndex);
}

void Ur3ePanelController::scanExecuteLogMoving(const int step,
                                               const int total,
                                               const int pointIndex,
                                               const QString &tcpSummary,
                                               const QString &targetSummary,
                                               const QVariantList &targetPositionsRad)
{
    if (!scanExecuting_)
        return;

    if (!targetPositionsRad.isEmpty())
    {
        std::vector<double> targets;
        targets.reserve(static_cast<std::size_t>(targetPositionsRad.size()));
        for (const QVariant &value : targetPositionsRad)
            targets.push_back(value.toDouble());
        applyJointTargets(targets);
        pollJoints();
    }

    host_->appendLog(
        QStringLiteral("UR3e scan execute [%1/%2] pt %3: MoveIt moving — %4; target %5")
            .arg(step)
            .arg(total)
            .arg(pointIndex)
            .arg(tcpSummary)
            .arg(targetSummary));
}

void Ur3ePanelController::scanExecuteLogArrived(const int step,
                                              const int total,
                                              const int pointIndex,
                                              const QString &arrivedPose,
                                              const QString &arrivedJoints,
                                              const QVariantList &positionsRad,
                                              const QStringList &names)
{
    if (!scanExecuting_)
        return;

    host_->appendLog(
        QStringLiteral("UR3e scan execute [%1/%2] pt %3: arrived pose %4; joints %5")
            .arg(step)
            .arg(total)
            .arg(pointIndex)
            .arg(arrivedPose)
            .arg(arrivedJoints));

    if (positionsRad.isEmpty())
        return;

    std::vector<double> positions;
    positions.reserve(static_cast<std::size_t>(positionsRad.size()));
    for (const QVariant &value : positionsRad)
        positions.push_back(value.toDouble());

    applyJointPositions(positions, names, false);
}

void Ur3ePanelController::scanExecuteFinish(const bool ok,
                                            const QString &errorMessage,
                                            const int executedCount,
                                            const bool stopped,
                                            const int capturedFrameCount,
                                            const qint64 elapsedMs)
{
    if (!scanExecuting_)
        return;

    {
        std::lock_guard<std::mutex> lock(scanExecuteThreadMutex_);
        if (scanExecuteThread_.joinable())
            scanExecuteThread_.join();
    }

    scanExecuting_ = false;
    stopRequested_.store(false, std::memory_order_release);
    if (host_->ur3eScanRoutePlanWidget_ != nullptr)
        host_->ur3eScanRoutePlanWidget_->endScanExecution();

    QString detail;
    if (!ok)
    {
        if (errorMessage.isEmpty())
            detail = QStringLiteral("scan execute failed.");
        else if (executedCount > 0)
            detail = QStringLiteral("Failed after %1 completed waypoint(s): %2")
                         .arg(executedCount)
                         .arg(errorMessage);
        else
            detail = errorMessage;
    }
    else if (stopped)
        detail = QStringLiteral("scan stopped after %1 waypoint(s).").arg(executedCount);
    else
        detail = QStringLiteral("scan complete — %1 waypoint(s).").arg(executedCount);

    if (capturedFrameCount > 0)
        detail += QStringLiteral(" Captured %1 BFS frame(s).").arg(capturedFrameCount);
    if (elapsedMs > 0)
    {
        const qint64 totalSec = elapsedMs / 1000;
        const qint64 minutes = totalSec / 60;
        const qint64 seconds = totalSec % 60;
        detail += QStringLiteral(" Time %1:%2.")
                      .arg(minutes)
                      .arg(seconds, 2, 10, QLatin1Char('0'));
    }

    finishScanExecute(ok, detail, capturedFrameCount, executedCount, elapsedMs, stopped);
}

void Ur3ePanelController::finishScanExecute(const bool ok,
                                            const QString &detail,
                                            const int capturedFrameCount,
                                            const int successfulPins,
                                            const qint64 elapsedMs,
                                            const bool stopped)
{
    scanReturningHome_.store(false, std::memory_order_release);
    setBusy(false);
    setJointPollIntervalMs(kPosePollIntervalMs);
    host_->appendLog(QStringLiteral("UR3e scan execute: %1").arg(detail));

    const qint64 totalSec = (elapsedMs > 0 ? elapsedMs : qint64{0}) / 1000;
    const qint64 minutes = totalSec / 60;
    const qint64 seconds = totalSec % 60;
    const QString durationText =
        QStringLiteral("%1:%2").arg(minutes).arg(seconds, 2, 10, QLatin1Char('0'));

    QString summaryTitle;
    if (!ok)
        summaryTitle = QStringLiteral("UR3e Scan Failed");
    else if (stopped)
        summaryTitle = QStringLiteral("UR3e Scan Stopped");
    else
        summaryTitle = QStringLiteral("UR3e Scan Complete");

    const QString summaryBody =
        QStringLiteral("Successful pins: %1\n"
                       "Multiview images recorded: %2\n"
                       "Scanning time: %3")
            .arg(successfulPins)
            .arg(capturedFrameCount)
            .arg(durationText);

    // Capture-driven Multiview merges the success/stopped summary into Recording complete.
    // Still show failures immediately so the operator sees the error.
    if (!(scanExecuteSuppressUiSummary_ && ok))
    {
        if (!ok)
            QMessageBox::warning(host_, summaryTitle, summaryBody + QStringLiteral("\n\n") + detail);
        else
            QMessageBox::information(host_, summaryTitle, summaryBody);
    }

    scanExecuteSuppressUiSummary_ = false;

    if (ok)
        pollJoints();
    updateRobotUi();
    emit hemisphereScanExecuteFinished(ok, detail, capturedFrameCount, successfulPins, elapsedMs);
}

void Ur3ePanelController::onMoveItStateChanged(const bool running, const QString &detail)
{
    if (running)
        pushWorkspaceBoundaryToMoveIt();

    if (detail.isEmpty())
    {
        updateRobotUi();
        return;
    }

    const QStringList lines = detail.split(QLatin1Char('\n'), Qt::SkipEmptyParts);
    for (const QString &line : lines)
    {
        const QString trimmed = line.trimmed();
        if (!trimmed.isEmpty())
            host_->appendLog(QStringLiteral("UR3e MoveIt: %1").arg(trimmed));
    }
    updateRobotUi();
}

void Ur3ePanelController::onStartRvizRequested()
{
    if (rvizManager_ == nullptr || busy_)
        return;

    if (rvizManager_->isRunning())
    {
        host_->appendLog(QStringLiteral("UR3e: stopping RViz\u2026"));
        rvizManager_->stop();
        updateRobotUi();
        return;
    }

    if (!isSidecarRunning())
    {
        host_->appendLog(QStringLiteral("UR3e: sidecar not ready — cannot start RViz."));
        return;
    }

    if (!robotConnected_)
    {
        host_->appendLog(QStringLiteral(
            "UR3e: connect the robot first (RViz needs the running UR driver for the model)."));
        return;
    }

    host_->appendLog(QStringLiteral("UR3e: starting RViz (visualization only) in WSL\u2026"));
    rvizManager_->start();
    updateRobotUi();
}

void Ur3ePanelController::onRvizStateChanged(const bool running, const QString &detail)
{
    if (detail.isEmpty())
    {
        updateRobotUi();
        Q_UNUSED(running);
        return;
    }

    const QStringList lines = detail.split(QLatin1Char('\n'), Qt::SkipEmptyParts);
    for (const QString &line : lines)
    {
        const QString trimmed = line.trimmed();
        if (!trimmed.isEmpty())
            host_->appendLog(QStringLiteral("UR3e RViz: %1").arg(trimmed));
    }
    updateRobotUi();
    Q_UNUSED(running);
}

void Ur3ePanelController::onConnectRequested()
{
    if (serverManager_ == nullptr || busy_ || connectInProgress_)
        return;

    if (!serverManager_->isServerConnected())
    {
        host_->appendLog(QStringLiteral("UR3e: sidecar not ready yet — wait for WSL startup (see UR3e log tab)."));
        return;
    }

    const hf::HardwareConfig::Ur3eConfig &cfg = hf::hardwareConfig().ur3e;
    if (cfg.prestartDriver && !driverPrestartReady_)
    {
        host_->appendLog(
            cfg.useMockHardware
                ? QStringLiteral(
                      "UR3e: simulation driver still warming up — wait for \"simulation driver ready\" in the log.")
                : QStringLiteral(
                      "UR3e: robot driver still warming up — wait for \"robot driver ready\" in the log "
                      "(reverse port 50002 must be listening)."));
        return;
    }

    const QString robotIp = host_->ur3eRobotIpEdit_ != nullptr
                                ? host_->ur3eRobotIpEdit_->text().trimmed()
                                : cfg.robotIp;
    const int connectTimeoutMs = qMax(30000, cfg.connectTimeoutMs);
    const int connectTimeoutSec = connectTimeoutMs / 1000;

    host_->appendLog(
        cfg.useMockHardware
            ? QStringLiteral("UR3e: connecting (simulation)\u2026")
            : QStringLiteral(
                  "UR3e: connecting to %1 — press Play on External Control (%2:50002) within %3 min\u2026")
                  .arg(robotIp)
                  .arg(cfg.reverseIp)
                  .arg((connectTimeoutSec + 59) / 60));

    setBusy(true);
    connectInProgress_ = true;
    lastConnectStatusPhase_.clear();
    lastConnectStatusMessage_.clear();
    const int sessionId = ++connectSessionId_;
    connectDeadlineMs_ = QDateTime::currentMSecsSinceEpoch() + connectTimeoutMs;

    dismissConnectWaitDialog();
    connectWaitDialog_ = new Ur3eExternalControlWaitDialog(host_);
    connectWaitDialog_->configure(cfg.useMockHardware, cfg.reverseIp, connectTimeoutSec);
    connect(connectWaitDialog_,
            &Ur3eExternalControlWaitDialog::cancelRequested,
            this,
            &Ur3ePanelController::onConnectDialogCancelled);
    connectWaitDialog_->show();
    connectWaitDialog_->raise();
    connectWaitDialog_->activateWindow();

    const QString serverUrl = serverManager_->serverUrl();
    std::thread([this, serverUrl, robotIp, sessionId]() {
        QString error;
        Ur3eConnectAsyncStatus status;
        bool started = ur3eConnectStart(serverUrl, robotIp, &status, &error);
        bool legacyConnect = false;
        Ur3eConnectResult legacyResult;
        if (!started && error.contains(QStringLiteral("not found"), Qt::CaseInsensitive))
        {
            legacyConnect = true;
            legacyResult = ur3eConnectRobot(serverUrl, robotIp, &error);
        }
        const bool legacyOk = legacyConnect && legacyResult.ok;
        const QString legacyDetail =
            legacyOk ? (legacyResult.useMockHardware ? QStringLiteral("connected (simulation)")
                                                     : QStringLiteral("connected (hardware)"))
                     : error;
        QMetaObject::invokeMethod(
            this,
            [this, started, status, error, sessionId, legacyConnect, legacyOk, legacyDetail]() {
                if (sessionId != connectSessionId_.load())
                    return;
                if (legacyConnect)
                {
                    dismissConnectWaitDialog();
                    connectInProgress_ = false;
                    if (connectPollTimer_ != nullptr)
                        connectPollTimer_->stop();
                    if (connectCountdownTimer_ != nullptr)
                        connectCountdownTimer_->stop();
                    finishConnect(legacyOk, legacyDetail);
                    return;
                }
                if (!started)
                {
                    dismissConnectWaitDialog();
                    connectInProgress_ = false;
                    if (connectPollTimer_ != nullptr)
                        connectPollTimer_->stop();
                    if (connectCountdownTimer_ != nullptr)
                        connectCountdownTimer_->stop();
                    finishConnect(false, error);
                    return;
                }

                applyConnectAsyncStatus(status);
                if (status.ok && !status.inProgress)
                {
                    dismissConnectWaitDialog();
                    connectInProgress_ = false;
                    if (connectPollTimer_ != nullptr)
                        connectPollTimer_->stop();
                    if (connectCountdownTimer_ != nullptr)
                        connectCountdownTimer_->stop();
                    finishConnect(
                        true,
                        status.useMockHardware ? QStringLiteral("connected (simulation)")
                                               : QStringLiteral("connected (hardware)"));
                    return;
                }

                if (connectPollTimer_ != nullptr)
                    connectPollTimer_->start();
                if (connectCountdownTimer_ != nullptr)
                    connectCountdownTimer_->start();
                onConnectCountdownTick();
            },
            Qt::QueuedConnection);
    }).detach();
}

void Ur3ePanelController::dismissConnectWaitDialog()
{
    if (connectWaitDialog_ == nullptr)
        return;

    connectWaitDialog_->dismiss();
    connectWaitDialog_->deleteLater();
    connectWaitDialog_ = nullptr;
}

void Ur3ePanelController::applyConnectAsyncStatus(const Ur3eConnectAsyncStatus &status)
{
    if (connectWaitDialog_ == nullptr)
        return;

    const QString phase = status.phase.trimmed().toLower();
    if (!status.message.isEmpty() && status.message != lastConnectStatusMessage_)
    {
        lastConnectStatusMessage_ = status.message;
        connectWaitDialog_->setDetailText(status.message);
    }

    if (phase == lastConnectStatusPhase_)
        return;
    lastConnectStatusPhase_ = phase;

    const bool mock = status.useMockHardware;
    if (phase == QStringLiteral("waiting_external_control")
        || (status.scriptPortListening && !status.reverseConnected && !mock))
    {
        connectWaitDialog_->setPhase(Ur3eExternalControlWaitDialog::Phase::PressPlay);
    }
    else if (phase == QStringLiteral("bridge_setup") || phase == QStringLiteral("finishing")
             || status.reverseConnected)
    {
        connectWaitDialog_->setPhase(Ur3eExternalControlWaitDialog::Phase::Finishing);
    }
    else if (phase == QStringLiteral("starting_driver"))
    {
        connectWaitDialog_->setPhase(mock ? Ur3eExternalControlWaitDialog::Phase::SimulationStarting
                                          : Ur3eExternalControlWaitDialog::Phase::StartingDriver);
    }
}

void Ur3ePanelController::updateConnectDialogFromSidecarLine(const QString &line)
{
    if (!connectInProgress_ || connectWaitDialog_ == nullptr)
        return;

    const QString lower = line.toLower();
    if (lower.contains(QStringLiteral("port 50002 listening"))
        || lower.contains(QStringLiteral("press play on external control")))
    {
        connectWaitDialog_->setPhase(Ur3eExternalControlWaitDialog::Phase::PressPlay);
    }
    else if (lower.contains(QStringLiteral("external control connected"))
             || lower.contains(QStringLiteral("trajectory action server ready")))
    {
        connectWaitDialog_->setPhase(Ur3eExternalControlWaitDialog::Phase::Finishing);
    }
    else if (lower.contains(QStringLiteral("starting ur_robot_driver"))
             || lower.contains(QStringLiteral("waiting for controller manager")))
    {
        connectWaitDialog_->setPhase(Ur3eExternalControlWaitDialog::Phase::StartingDriver);
    }
    else if (lower.contains(QStringLiteral("connect phase=waiting_external_control")))
    {
        connectWaitDialog_->setPhase(Ur3eExternalControlWaitDialog::Phase::PressPlay);
    }
    else if (lower.contains(QStringLiteral("connect phase=bridge_setup"))
             || lower.contains(QStringLiteral("connect phase=finishing")))
    {
        connectWaitDialog_->setPhase(Ur3eExternalControlWaitDialog::Phase::Finishing);
    }

    if (line.contains(QStringLiteral("connect phase="), Qt::CaseInsensitive))
        connectWaitDialog_->setDetailText(line);
}

void Ur3ePanelController::onConnectPollTick()
{
    if (!connectInProgress_ || serverManager_ == nullptr)
        return;

    // One wsl.exe/curl at a time — spawning every 500 ms without this guard stalls the UI.
    if (connectPollInFlight_.exchange(true, std::memory_order_acq_rel))
        return;

    const int sessionId = connectSessionId_.load();
    const QString serverUrl = serverManager_->serverUrl();
    std::thread([this, serverUrl, sessionId]() {
        Ur3eConnectAsyncStatus status;
        QString error;
        const bool polled = ur3eConnectStatus(serverUrl, &status, &error);
        QMetaObject::invokeMethod(
            this,
            [this, polled, status, error, sessionId]() {
                connectPollInFlight_.store(false, std::memory_order_release);
                if (sessionId != connectSessionId_.load() || !connectInProgress_)
                    return;
                if (!polled)
                {
                    if (connectWaitDialog_ != nullptr)
                        connectWaitDialog_->setDetailText(error);
                    return;
                }

                applyConnectAsyncStatus(status);
                const QString phase = status.phase.trimmed().toLower();
                if (status.ok && phase == QStringLiteral("complete"))
                {
                    if (connectPollTimer_ != nullptr)
                        connectPollTimer_->stop();
                    if (connectCountdownTimer_ != nullptr)
                        connectCountdownTimer_->stop();
                    dismissConnectWaitDialog();
                    connectInProgress_ = false;
                    finishConnect(
                        true,
                        status.useMockHardware ? QStringLiteral("connected (simulation)")
                                               : QStringLiteral("connected (hardware)"));
                    return;
                }

                if (phase == QStringLiteral("failed"))
                {
                    if (connectPollTimer_ != nullptr)
                        connectPollTimer_->stop();
                    if (connectCountdownTimer_ != nullptr)
                        connectCountdownTimer_->stop();
                    dismissConnectWaitDialog();
                    connectInProgress_ = false;
                    finishConnect(false, status.errorMessage.isEmpty() ? status.message : status.errorMessage);
                    return;
                }

                if (phase == QStringLiteral("cancelled"))
                {
                    if (connectPollTimer_ != nullptr)
                        connectPollTimer_->stop();
                    if (connectCountdownTimer_ != nullptr)
                        connectCountdownTimer_->stop();
                    dismissConnectWaitDialog();
                    connectInProgress_ = false;
                    setBusy(false);
                    host_->appendLog(QStringLiteral("UR3e connect cancelled."));
                    updateRobotUi();
                }
            },
            Qt::QueuedConnection);
    }).detach();
}

void Ur3ePanelController::onConnectCountdownTick()
{
    if (!connectInProgress_ || connectWaitDialog_ == nullptr)
        return;

    const qint64 remainingMs = connectDeadlineMs_ - QDateTime::currentMSecsSinceEpoch();
    const int remainingSec = static_cast<int>((remainingMs + 999) / 1000);
    connectWaitDialog_->setRemainingSeconds(remainingSec);
    if (remainingSec <= 0)
        onConnectTimedOut();
}

void Ur3ePanelController::onConnectDialogCancelled()
{
    if (!connectInProgress_)
        return;

    ++connectSessionId_;
    connectPollInFlight_.store(false, std::memory_order_release);
    if (connectPollTimer_ != nullptr)
        connectPollTimer_->stop();
    if (connectCountdownTimer_ != nullptr)
        connectCountdownTimer_->stop();

    host_->appendLog(QStringLiteral("UR3e: connect cancelled by user."));
    dismissConnectWaitDialog();

    if (serverManager_ != nullptr)
    {
        const QString serverUrl = serverManager_->serverUrl();
        host_->appendLog(QStringLiteral("UR3e: cancelling connect\u2026"));
        std::thread([this, serverUrl]() {
            QString error;
            (void)ur3eConnectCancel(serverUrl, &error);

            // Short wait only — cancel keeps the warm driver and must not hang the UI.
            for (int i = 0; i < 20; ++i)
            {
                Ur3eConnectAsyncStatus status;
                QString statusError;
                if (ur3eConnectStatus(serverUrl, &status, &statusError) && !status.inProgress
                    && status.phase != QStringLiteral("cancelling"))
                {
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(250));
            }

            QMetaObject::invokeMethod(
                this,
                [this]() {
                    connectInProgress_ = false;
                    setBusy(false);
                    // Cancel no longer kills the prestarted driver — keep Connect enabled.
                    host_->appendLog(QStringLiteral("UR3e: connect cancel complete."));
                    updateRobotUi();
                },
                Qt::QueuedConnection);
        }).detach();
    }
    else
    {
        connectInProgress_ = false;
        setBusy(false);
        updateRobotUi();
    }
}

void Ur3ePanelController::onConnectTimedOut()
{
    if (!connectInProgress_)
        return;

    ++connectSessionId_;
    connectPollInFlight_.store(false, std::memory_order_release);
    if (connectPollTimer_ != nullptr)
        connectPollTimer_->stop();
    if (connectCountdownTimer_ != nullptr)
        connectCountdownTimer_->stop();

    dismissConnectWaitDialog();
    const bool useMockHardware = hf::hardwareConfig().ur3e.useMockHardware;
    host_->appendLog(
        useMockHardware
            ? QStringLiteral("UR3e: connect timed out waiting for simulation driver.")
            : QStringLiteral("UR3e: connect timed out waiting for External Control Play."));

    if (serverManager_ != nullptr)
    {
        const QString serverUrl = serverManager_->serverUrl();
        std::thread([this, serverUrl, useMockHardware]() {
            QString error;
            (void)ur3eConnectCancel(serverUrl, &error);
            for (int i = 0; i < 120; ++i)
            {
                Ur3eConnectAsyncStatus status;
                QString statusError;
                if (ur3eConnectStatus(serverUrl, &status, &statusError) && !status.inProgress
                    && status.phase != QStringLiteral("cancelling"))
                {
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
            QMetaObject::invokeMethod(
                this,
                [this, useMockHardware]() {
                    connectInProgress_ = false;
                    // Cancel keeps the warm driver; do not force a cold re-prestart.
                    finishConnect(
                        false,
                        useMockHardware
                            ? QStringLiteral(
                                  "Simulation connect timed out. The ROS mock driver may still be "
                                  "starting in WSL — wait for \"simulation driver ready\", then try again.")
                            : QStringLiteral(
                                  "Timed out waiting for External Control. On the teach pendant open "
                                  "External Control, confirm remote PC, and press Play while connecting."));
                },
                Qt::QueuedConnection);
        }).detach();
    }
    else
    {
        connectInProgress_ = false;
        finishConnect(false, QStringLiteral("Connect timed out."));
    }
}

void Ur3ePanelController::onDisconnectRequested()
{
    if (serverManager_ == nullptr || busy_ || !robotConnected_)
        return;

    host_->appendLog(QStringLiteral("UR3e: preparing to disconnect\u2026"));
    setBusy(true);
    beginHomeMotionUi();
    const QString serverUrl = serverManager_->serverUrl();
    std::thread([this, serverUrl]() {
        const HomeEnsureOutcome homeOutcome =
            ensureRobotAtHomeSync(HomeEnsureContext::BeforeDisconnect);
        if (homeOutcome.cancelled)
        {
            QMetaObject::invokeMethod(
                this,
                [this]() {
                    endHomeMotionUi();
                    setBusy(false);
                    host_->appendLog(
                        QStringLiteral("UR3e: disconnect cancelled (home positioning)."));
                    pollJointsSync();
                    syncTargetsFromCurrent();
                    updateRobotUi();
                },
                Qt::QueuedConnection);
            return;
        }

        if (!homeOutcome.success && !homeOutcome.alreadyAtHome)
        {
            host_->appendLog(
                QStringLiteral("UR3e: disconnecting without reaching scan home."));
        }

        const Ur3eConnectResult result = ur3eDisconnectRobot(serverUrl);
        const bool ok = result.ok;
        const QString detail = ok ? QStringLiteral("disconnected") : result.errorMessage;
        QMetaObject::invokeMethod(
            this,
            [this, ok, detail]() { finishDisconnect(ok, detail); },
            Qt::QueuedConnection);
    }).detach();
}

void Ur3ePanelController::onStopMotionRequested()
{
    requestStopMotion();
}

void Ur3ePanelController::requestStopMotion()
{
    if (serverManager_ == nullptr || !robotConnected_)
        return;

    // Do not cancel an in-progress post-scan retreat to home (double-Stop used to abort it).
    if (scanReturningHome_.load(std::memory_order_acquire))
    {
        host_->appendLog(
            QStringLiteral("UR3e: stop ignored — already returning to scan home."));
        return;
    }

    stopRequested_.store(true, std::memory_order_release);
    host_->appendLog(QStringLiteral("UR3e: stop requested"));
    updateRobotUi();

    const QString serverUrl = serverManager_->serverUrl();
    std::thread([this, serverUrl]() {
        QString error;
        const bool ok = ur3eStopMotion(serverUrl, &error);
        const QString detail = ok ? QStringLiteral("motion stopped") : error;
        QMetaObject::invokeMethod(
            this,
            [this, ok, detail]() { finishStop(ok, detail); },
            Qt::QueuedConnection);
    }).detach();
}

void Ur3ePanelController::finishConnect(const bool ok, const QString &detail)
{
    if (!ok)
    {
        setBusy(false);
        host_->appendLog(QStringLiteral("UR3e connect failed: %1").arg(detail));
        updateRobotUi();
        return;
    }

    robotConnected_ = true;
    host_->appendLog(QStringLiteral("UR3e: %1").arg(detail));
    pushWorkspaceBoundaryToMoveIt();
    if (host_->ur3ePosePollTimer_ != nullptr)
    {
        host_->ur3ePosePollTimer_->start(kPosePollIntervalMs);
        const Ur3eJointsState joints = ur3eGetJoints(serverManager_->serverUrl());
        if (joints.ok && joints.positionsRad.size() >= MainWindow::kUr3eJointCount)
            applyJointPositions(joints.positionsRad, joints.names, false);
        else
            pollJointsSync();

        const hf::HardwareConfig::Ur3eConfig &cfg = hf::hardwareConfig().ur3e;
        if (cfg.useMockHardware)
            applyConfiguredInitialJointTargets();
        else
            syncTargetsFromCurrent();
    }

    updateRobotUi();
    host_->appendLog(QStringLiteral("UR3e: verifying scan home position\u2026"));
    setBusy(true);
    beginHomeMotionUi();

    std::thread([this]() {
        const HomeEnsureOutcome outcome = ensureRobotAtHomeSync(HomeEnsureContext::AfterConnect);
        QMetaObject::invokeMethod(
            this,
            [this, outcome]() { finishHomeEnsureAfterConnect(outcome); },
            Qt::QueuedConnection);
    }).detach();
}

Ur3ePanelController::HomeEnsureOutcome Ur3ePanelController::ensureRobotAtHomeSync(
    const HomeEnsureContext context)
{
    HomeEnsureOutcome outcome;
    if (serverManager_ == nullptr || !serverManager_->isServerConnected() || !robotConnected_)
    {
        outcome.success = true;
        return outcome;
    }

    const QString serverUrl = serverManager_->serverUrl();
    for (;;)
    {
        if (context != HomeEnsureContext::BeforeShutdown
            && shutdownRequested_.load(std::memory_order_acquire))
        {
            outcome.cancelled = true;
            return outcome;
        }

        host_->appendLog(QStringLiteral("UR3e: MoveIt verifying scan home\u2026"));
        const Ur3eScanWaypointMoveResult moveResult = ur3eExecuteMoveHome(serverUrl);
        if (moveResult.stopped)
        {
            outcome.cancelled = true;
            return outcome;
        }

        if (moveResult.ok)
        {
            outcome.success = true;
            outcome.alreadyAtHome = moveResult.alreadyAtHome;
            outcome.atHomeVerified = true;
            host_->appendLog(moveResult.alreadyAtHome
                                ? QStringLiteral("UR3e: verified at scan home position.")
                                : QStringLiteral("UR3e: moved to scan home position."));
            pollJointsSync();
            if (QThread::currentThread() == thread())
                syncHomeJointTargetSliders();
            else
            {
                QMetaObject::invokeMethod(this,
                                          &Ur3ePanelController::syncHomeJointTargetSliders,
                                          Qt::BlockingQueuedConnection);
            }
            return outcome;
        }

        const QString reason = moveResult.errorMessage.trimmed().isEmpty()
                                   ? QStringLiteral("MoveIt could not reach scan home.")
                                   : moveResult.errorMessage.trimmed();
        const HomeEnsurePromptChoice choice = promptManualHomePositioning(reason, context);
        if (choice == HomeEnsurePromptChoice::Retry)
            continue;

        if (choice == HomeEnsurePromptChoice::ContinueWithoutHoming
            && (context == HomeEnsureContext::AfterConnect
                || context == HomeEnsureContext::BeforeScanExecute))
        {
            outcome.success = true;
            if (context == HomeEnsureContext::BeforeScanExecute)
                host_->appendLog(QStringLiteral("UR3e: starting scan without valid scan home."));
            else
                host_->appendLog(QStringLiteral("UR3e: continuing without scan home verification."));
            return outcome;
        }

        if (choice == HomeEnsurePromptChoice::ProceedAnyway
            && (context == HomeEnsureContext::BeforeDisconnect
                || context == HomeEnsureContext::BeforeShutdown))
        {
            outcome.success = true;
            host_->appendLog(QStringLiteral("UR3e: proceeding without reaching scan home."));
            return outcome;
        }

        outcome.cancelled = true;
        return outcome;
    }
}

Ur3ePanelController::HomeEnsurePromptChoice Ur3ePanelController::promptManualHomePositioning(
    const QString &reason,
    const HomeEnsureContext context)
{
    if (QThread::currentThread() == thread())
        return showManualHomePositioningDialog(reason, context);

    HomeEnsurePromptChoice choice = HomeEnsurePromptChoice::Cancel;
    QMetaObject::invokeMethod(
        this,
        [this, reason, context, &choice]() {
            choice = showManualHomePositioningDialog(reason, context);
        },
        Qt::BlockingQueuedConnection);
    return choice;
}

Ur3ePanelController::HomeEnsurePromptChoice Ur3ePanelController::showManualHomePositioningDialog(
    const QString &reason,
    const HomeEnsureContext context)
{
    if (host_ == nullptr)
        return HomeEnsurePromptChoice::Cancel;

    QMessageBox box(host_);
    box.setIcon(QMessageBox::Warning);
    if (context == HomeEnsureContext::AfterConnect)
        box.setWindowTitle(QStringLiteral("UR3e Homing Failed"));
    else if (context == HomeEnsureContext::BeforeScanExecute)
        box.setWindowTitle(QStringLiteral("UR3e Homing Failed — Cannot Start Scan"));
    else if (context == HomeEnsureContext::BeforeDisconnect)
        box.setWindowTitle(QStringLiteral("UR3e Homing Failed — Before Disconnect"));
    else
        box.setWindowTitle(QStringLiteral("UR3e Homing Failed — Before Closing"));

    const QString body =
        QStringLiteral(
            "The robot could not reach or verify the configured scan home pose.\n\n"
            "Reason: %1\n\n"
            "Configured home: %2\n\n"
            "Manually jog the robot closer to home using the joint controls or teach "
            "pendant, then click Retry.")
            .arg(reason, formatConfiguredHomeJointsDeg());
    box.setText(body);

    QPushButton *retryButton = box.addButton(QStringLiteral("Retry"), QMessageBox::AcceptRole);
    box.setDefaultButton(retryButton);

    QPushButton *secondaryButton = nullptr;
    QPushButton *cancelButton = nullptr;
    if (context == HomeEnsureContext::AfterConnect)
    {
        secondaryButton =
            box.addButton(QStringLiteral("Continue without homing"), QMessageBox::DestructiveRole);
        cancelButton = box.addButton(QStringLiteral("Disconnect"), QMessageBox::RejectRole);
    }
    else if (context == HomeEnsureContext::BeforeScanExecute)
    {
        secondaryButton =
            box.addButton(QStringLiteral("Start scan anyway"), QMessageBox::DestructiveRole);
        cancelButton = box.addButton(QStringLiteral("Abort scan"), QMessageBox::RejectRole);
    }
    else if (context == HomeEnsureContext::BeforeDisconnect)
    {
        secondaryButton =
            box.addButton(QStringLiteral("Disconnect anyway"), QMessageBox::DestructiveRole);
        cancelButton = box.addButton(QStringLiteral("Cancel"), QMessageBox::RejectRole);
    }
    else
    {
        secondaryButton = box.addButton(QStringLiteral("Close anyway"), QMessageBox::DestructiveRole);
        cancelButton = box.addButton(QStringLiteral("Cancel"), QMessageBox::RejectRole);
    }

    const bool wasBusy = busy_;
    setBusy(false);
    updateRobotUi();
    box.exec();
    setBusy(wasBusy);
    updateRobotUi();

    QAbstractButton *clicked = box.clickedButton();
    if (clicked == retryButton)
        return HomeEnsurePromptChoice::Retry;
    if (clicked == secondaryButton)
    {
        return context == HomeEnsureContext::AfterConnect ? HomeEnsurePromptChoice::ContinueWithoutHoming
                                                        : HomeEnsurePromptChoice::ProceedAnyway;
    }
    return HomeEnsurePromptChoice::Cancel;
}

void Ur3ePanelController::finishHomeEnsureAfterConnect(const HomeEnsureOutcome &outcome)
{
    if (outcome.cancelled)
    {
        host_->appendLog(QStringLiteral("UR3e: home verification cancelled — disconnecting."));
        setBusy(true);
        const QString serverUrl = serverManager_->serverUrl();
        std::thread([this, serverUrl]() {
            const Ur3eConnectResult result = ur3eDisconnectRobot(serverUrl);
            const bool ok = result.ok;
            const QString detail = ok ? QStringLiteral("disconnected") : result.errorMessage;
            QMetaObject::invokeMethod(
                this,
                [this, ok, detail]() { finishDisconnect(ok, detail); },
                Qt::QueuedConnection);
        }).detach();
        return;
    }

    endHomeMotionUi();
    setBusy(false);
    if (outcome.alreadyAtHome)
        host_->appendLog(QStringLiteral("UR3e: verified at scan home position."));
    else if (outcome.atHomeVerified)
        host_->appendLog(QStringLiteral("UR3e: moved to scan home position."));
    pollJointsSync();
    if (outcome.atHomeVerified)
        applyScanHomeJointTargets();
    updateRobotUi();
}

void Ur3ePanelController::finishDisconnect(const bool ok, const QString &detail)
{
    endHomeMotionUi();
    setBusy(false);
    robotConnected_ = false;
    stopRequested_.store(false, std::memory_order_release);
    if (host_->ur3ePosePollTimer_ != nullptr)
        host_->ur3ePosePollTimer_->stop();

    // Disconnect stops the UR driver; do not leave Connect enabled as if prestart
    // were still warm — wait for driver_ready again.
    const hf::HardwareConfig::Ur3eConfig &cfg = hf::hardwareConfig().ur3e;
    if (cfg.prestartDriver && isSidecarRunning())
    {
        driverPrestartReady_ = false;
        if (driverReadyPollTimer_ != nullptr)
            driverReadyPollTimer_->start();
        pollDriverPrestartReady();
        if (ok)
        {
            host_->appendLog(
                QStringLiteral(
                    "UR3e: disconnected — re-warming driver (Connect enables when ready)…"));
        }
    }

    if (ok)
        host_->appendLog(QStringLiteral("UR3e: %1").arg(detail));
    else
        host_->appendLog(QStringLiteral("UR3e disconnect failed: %1").arg(detail));

    updateRobotUi();
}

void Ur3ePanelController::finishMove(const bool ok, const QString &detail)
{
    const bool stoppedByUser = stopRequested_.exchange(false, std::memory_order_acq_rel);
    motionInProgress_ = false;
    setBusy(false);
    setJointPollIntervalMs(kPosePollIntervalMs);
    if (stoppedByUser)
    {
        host_->appendLog(QStringLiteral("UR3e: move interrupted by stop"));
        pollJointsSync();
        syncTargetsFromCurrent();
    }
    else if (ok)
    {
        host_->appendLog(QStringLiteral("UR3e: %1").arg(detail));
        pollJointsSync();
        syncTargetsFromCurrent();
    }
    else
    {
        host_->appendLog(QStringLiteral("UR3e move failed: %1").arg(detail));
        if (host_ != nullptr)
        {
            QMessageBox::warning(host_,
                                 QStringLiteral("UR3e Move Failed"),
                                 detail);
        }
    }
    updateRobotUi();
}

void Ur3ePanelController::finishStop(const bool ok, const QString &detail)
{
    motionInProgress_ = false;
    if (scanExecuting_)
    {
        // Keep stopRequested_ set and stay busy so the scan thread exits the pin loop
        // and returns to home. Clearing the flag here used to resume the scan in place.
        if (ok)
            host_->appendLog(QStringLiteral("UR3e: %1 — scan will return to home").arg(detail));
        else
            host_->appendLog(QStringLiteral("UR3e stop failed: %1").arg(detail));
        updateRobotUi();
        return;
    }

    setBusy(false);
    setJointPollIntervalMs(kPosePollIntervalMs);
    stopRequested_.store(false, std::memory_order_release);
    if (ok)
        host_->appendLog(QStringLiteral("UR3e: %1").arg(detail));
    else
        host_->appendLog(QStringLiteral("UR3e stop failed: %1").arg(detail));
    pollJointsSync();
    syncTargetsFromCurrent();
    updateRobotUi();
}

} // namespace hf::ur3e
