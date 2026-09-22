// Optical-TCP pose → nerfstudio-style transforms.json (Capture Multiview RGB).
// Capture poses are always BFS camera optical (cfg tool_tcp_*), even when MoveIt tips on DLP.

#include "backend/multiview/Ur3eCameraTransforms.hpp"

#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include <algorithm>
#include <cmath>

namespace hf::ur3e {
namespace {

void rotVecToMatrix(const double rx, const double ry, const double rz,
                    double R[3][3]) {
  const double angle = std::sqrt(rx * rx + ry * ry + rz * rz);
  if (angle < 1e-12) {
    R[0][0] = 1.0;
    R[0][1] = 0.0;
    R[0][2] = 0.0;
    R[1][0] = 0.0;
    R[1][1] = 1.0;
    R[1][2] = 0.0;
    R[2][0] = 0.0;
    R[2][1] = 0.0;
    R[2][2] = 1.0;
    return;
  }

  const double ax = rx / angle;
  const double ay = ry / angle;
  const double az = rz / angle;
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  const double t = 1.0 - c;

  R[0][0] = t * ax * ax + c;
  R[0][1] = t * ax * ay - s * az;
  R[0][2] = t * ax * az + s * ay;
  R[1][0] = t * ax * ay + s * az;
  R[1][1] = t * ay * ay + c;
  R[1][2] = t * ay * az - s * ax;
  R[2][0] = t * ax * az - s * ay;
  R[2][1] = t * ay * az + s * ax;
  R[2][2] = t * az * az + c;
}

Mat4 identityMat4() {
  Mat4 m{};
  m[0] = 1.0;
  m[5] = 1.0;
  m[10] = 1.0;
  m[15] = 1.0;
  return m;
}

QJsonArray matrix3ToJson(const double R[3][3]) {
  QJsonArray rows;
  for (int row = 0; row < 3; ++row) {
    QJsonArray cols;
    for (int col = 0; col < 3; ++col)
      cols.append(R[row][col]);
    rows.append(cols);
  }
  return rows;
}

QJsonArray vec3ToJson(const double t[3]) {
  QJsonArray arr;
  arr.append(t[0]);
  arr.append(t[1]);
  arr.append(t[2]);
  return arr;
}

QJsonObject intrinsicsToJson(const CameraIntrinsics &intrinsics) {
  QJsonObject obj;
  obj.insert(QStringLiteral("fx"), intrinsics.fx);
  obj.insert(QStringLiteral("fy"), intrinsics.fy);
  obj.insert(QStringLiteral("cx"), intrinsics.cx);
  obj.insert(QStringLiteral("cy"), intrinsics.cy);
  if (intrinsics.width > 0)
    obj.insert(QStringLiteral("width"), intrinsics.width);
  if (intrinsics.height > 0)
    obj.insert(QStringLiteral("height"), intrinsics.height);
  QJsonArray distortion;
  for (const double coeff : intrinsics.distortion)
    distortion.append(coeff);
  obj.insert(QStringLiteral("distortion"), distortion);
  return obj;
}

QJsonObject extrinsicsToJson(const CameraExtrinsicsRt &extrinsics) {
  QJsonObject obj;
  obj.insert(QStringLiteral("convention"),
             QStringLiteral("camera_to_base_link_opencv"));
  obj.insert(QStringLiteral("R"), matrix3ToJson(extrinsics.R));
  obj.insert(QStringLiteral("t"), vec3ToJson(extrinsics.t));
  return obj;
}

QJsonArray se3MatrixJson(const double R[3][3], const double t[3]) {
  QJsonArray T;
  for (int row = 0; row < 4; ++row) {
    QJsonArray rowArr;
    for (int col = 0; col < 4; ++col) {
      if (row < 3 && col < 3)
        rowArr.append(R[row][col]);
      else if (row < 3 && col == 3)
        rowArr.append(t[row]);
      else if (row == 3 && col == 3)
        rowArr.append(1.0);
      else
        rowArr.append(0.0);
    }
    T.append(rowArr);
  }
  return T;
}

QJsonObject baseTFlangeJson(const CalibrationCaptureExtras &calib) {
  double R[3][3]{};
  rotVecToMatrix(calib.flangeRx, calib.flangeRy, calib.flangeRz, R);
  const double t[3] = {calib.flangeX, calib.flangeY, calib.flangeZ};
  QJsonObject flange;
  flange.insert(QStringLiteral("frame"), QStringLiteral("tool0"));
  flange.insert(QStringLiteral("parent_frame"), QStringLiteral("base_link"));
  flange.insert(QStringLiteral("convention"), QStringLiteral("base_link_T_tool0"));
  flange.insert(QStringLiteral("x_m"), calib.flangeX);
  flange.insert(QStringLiteral("y_m"), calib.flangeY);
  flange.insert(QStringLiteral("z_m"), calib.flangeZ);
  QJsonObject rot;
  rot.insert(QStringLiteral("rx"), calib.flangeRx);
  rot.insert(QStringLiteral("ry"), calib.flangeRy);
  rot.insert(QStringLiteral("rz"), calib.flangeRz);
  flange.insert(QStringLiteral("rotvec_rad"), rot);
  flange.insert(QStringLiteral("T"), se3MatrixJson(R, t));
  return flange;
}

/// URDF / tf2 fixed RPY: R = Rz(yaw) * Ry(pitch) * Rx(roll).
void rpyDegToMatrix(const double rollDeg, const double pitchDeg, const double yawDeg,
                    double R[3][3]) {
  const double roll = rollDeg * (3.14159265358979323846 / 180.0);
  const double pitch = pitchDeg * (3.14159265358979323846 / 180.0);
  const double yaw = yawDeg * (3.14159265358979323846 / 180.0);
  const double cr = std::cos(roll);
  const double sr = std::sin(roll);
  const double cp = std::cos(pitch);
  const double sp = std::sin(pitch);
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);
  R[0][0] = cy * cp;
  R[0][1] = cy * sp * sr - sy * cr;
  R[0][2] = cy * sp * cr + sy * sr;
  R[1][0] = sy * cp;
  R[1][1] = sy * sp * sr + cy * cr;
  R[1][2] = sy * sp * cr - cy * sr;
  R[2][0] = -sp;
  R[2][1] = cp * sr;
  R[2][2] = cp * cr;
}

void matrixToRotVec(const double R[3][3], double &rx, double &ry, double &rz) {
  const double m00 = R[0][0];
  const double m01 = R[0][1];
  const double m02 = R[0][2];
  const double m10 = R[1][0];
  const double m11 = R[1][1];
  const double m12 = R[1][2];
  const double m20 = R[2][0];
  const double m21 = R[2][1];
  const double m22 = R[2][2];
  const double cosAngle = std::clamp(0.5 * (m00 + m11 + m22 - 1.0), -1.0, 1.0);
  const double angle = std::acos(cosAngle);
  if (angle < 1.0e-12) {
    rx = 0.0;
    ry = 0.0;
    rz = 0.0;
    return;
  }
  if (std::abs(3.14159265358979323846 - angle) < 1.0e-6) {
    // Near π: pick a stable axis from the diagonal.
    double axis[3] = {std::sqrt(std::max(0.0, (m00 + 1.0) * 0.5)),
                      std::sqrt(std::max(0.0, (m11 + 1.0) * 0.5)),
                      std::sqrt(std::max(0.0, (m22 + 1.0) * 0.5))};
    if (m01 < 0.0)
      axis[1] = -axis[1];
    if (m02 < 0.0)
      axis[2] = -axis[2];
    const double n =
        std::sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
    if (n > 1.0e-12) {
      rx = axis[0] / n * angle;
      ry = axis[1] / n * angle;
      rz = axis[2] / n * angle;
    } else {
      rx = angle;
      ry = 0.0;
      rz = 0.0;
    }
    return;
  }
  const double inv = 0.5 / std::sin(angle);
  rx = (m21 - m12) * inv * angle;
  ry = (m02 - m20) * inv * angle;
  rz = (m10 - m01) * inv * angle;
}

} // namespace

Ur3eScanTcpPose cameraOpticalTcpFromTool0(
    const Ur3eTcpPose &tool0,
    const hf::HardwareConfig::Ur3eConfig::ToolTcpMm &cameraTcp) {
  double Rf[3][3]{};
  rotVecToMatrix(tool0.rx, tool0.ry, tool0.rz, Rf);
  double Rc[3][3]{};
  rpyDegToMatrix(cameraTcp.rollDeg, cameraTcp.pitchDeg, cameraTcp.yawDeg, Rc);

  // base_T_cam = base_T_tool0 · tool0_T_camera
  double R[3][3]{};
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col)
      R[row][col] =
          Rf[row][0] * Rc[0][col] + Rf[row][1] * Rc[1][col] + Rf[row][2] * Rc[2][col];
  }
  const double tx = cameraTcp.xMm * 0.001;
  const double ty = cameraTcp.yMm * 0.001;
  const double tz = cameraTcp.zMm * 0.001;
  const double t0[3] = {tool0.x, tool0.y, tool0.z};
  const double t[3] = {Rf[0][0] * tx + Rf[0][1] * ty + Rf[0][2] * tz + t0[0],
                       Rf[1][0] * tx + Rf[1][1] * ty + Rf[1][2] * tz + t0[1],
                       Rf[2][0] * tx + Rf[2][1] * ty + Rf[2][2] * tz + t0[2]};

  Ur3eScanTcpPose out{};
  out.xM = t[0];
  out.yM = t[1];
  out.zM = t[2];
  matrixToRotVec(R, out.rxRad, out.ryRad, out.rzRad);
  out.toolZMx = R[0][2];
  out.toolZMy = R[1][2];
  out.toolZMz = R[2][2];
  return out;
}

CameraExtrinsicsRt cameraExtrinsicsOpenCvFromTcp(const Ur3eScanTcpPose &tcp) {
  CameraExtrinsicsRt out{};
  rotVecToMatrix(tcp.rxRad, tcp.ryRad, tcp.rzRad, out.R);
  out.t[0] = tcp.xM;
  out.t[1] = tcp.yM;
  out.t[2] = tcp.zM;
  return out;
}

Mat4 cameraToWorldOpenGlFromTcp(const Ur3eScanTcpPose &tcp) {
  const CameraExtrinsicsRt cv = cameraExtrinsicsOpenCvFromTcp(tcp);

  // OpenCV-style camera-to-parent: columns = camera X/Y/Z; +Z = optical axis
  // (tool +Z).
  Mat4 opencv = identityMat4();
  for (int row = 0; row < 3; ++row) {
    const int r = row * 4;
    opencv[static_cast<std::size_t>(r + 0)] = cv.R[row][0];
    opencv[static_cast<std::size_t>(r + 1)] = cv.R[row][1];
    opencv[static_cast<std::size_t>(r + 2)] = cv.R[row][2];
    opencv[static_cast<std::size_t>(r + 3)] = cv.t[row];
  }

  // OpenCV → OpenGL (nerfstudio): flip Y and Z of the camera basis.
  // c2w_gl = c2w_cv * diag(1, -1, -1, 1)
  Mat4 gl = identityMat4();
  for (int row = 0; row < 3; ++row) {
    const int r = row * 4;
    gl[static_cast<std::size_t>(r + 0)] =
        opencv[static_cast<std::size_t>(r + 0)];
    gl[static_cast<std::size_t>(r + 1)] =
        -opencv[static_cast<std::size_t>(r + 1)];
    gl[static_cast<std::size_t>(r + 2)] =
        -opencv[static_cast<std::size_t>(r + 2)];
    gl[static_cast<std::size_t>(r + 3)] =
        opencv[static_cast<std::size_t>(r + 3)];
  }
  return gl;
}

bool writeTransformsJson(const QString &directory,
                         const TransformsJsonDocument &doc,
                         QString *errorMessage,
                         const bool appendExisting) {
  if (directory.isEmpty()) {
    if (errorMessage != nullptr)
      *errorMessage = QStringLiteral("transforms.json directory is empty.");
    return false;
  }

  QDir dir(directory);
  if (!dir.exists() && !dir.mkpath(QStringLiteral("."))) {
    if (errorMessage != nullptr)
      *errorMessage =
          QStringLiteral("Could not create directory: %1").arg(directory);
    return false;
  }

  const bool haveFocal = doc.intrinsics.fx > 0.0 && doc.intrinsics.fy > 0.0;

  QJsonObject root;
  root.insert(QStringLiteral("camera_model"), QStringLiteral("OPENCV"));
  root.insert(QStringLiteral("coordinate_convention"),
              QStringLiteral("opengl"));
  root.insert(
      QStringLiteral("note"),
      haveFocal
          ? QStringLiteral(
                "Frames use BFS camera optical in base_link "
                "(base_T_tool0 · tool_tcp_*). MoveIt tip may be DLP when "
                "scan_tcp=dlp; capture poses stay on the camera. "
                "Extrinsics R/t are OpenCV camera-to-base_link.")
          : QStringLiteral(
                "Frames use BFS camera optical in base_link "
                "(base_T_tool0 · tool_tcp_*). Set bfs_camera_fx/fy (and "
                "cx/cy) in hyperfusion.cfg for full intrinsics; extrinsics "
                "R/t are OpenCV camera-to-base_link."));
  root.insert(QStringLiteral("parent_frame"), QStringLiteral("base_link"));
  root.insert(QStringLiteral("frame"), QStringLiteral("camera_optical"));
  root.insert(QStringLiteral("optical_tip"), QStringLiteral("camera"));
  root.insert(QStringLiteral("moveit_tip"),
              hf::hardwareConfig().ur3e.usesDlpScanTcp()
                  ? QStringLiteral("dlp")
                  : QStringLiteral("camera"));
  root.insert(QStringLiteral("intrinsics"), intrinsicsToJson(doc.intrinsics));
  if (doc.intrinsics.width > 0)
    root.insert(QStringLiteral("w"), doc.intrinsics.width);
  if (doc.intrinsics.height > 0)
    root.insert(QStringLiteral("h"), doc.intrinsics.height);
  // Nerfstudio-friendly flat keys when calibrated.
  if (haveFocal) {
    root.insert(QStringLiteral("fl_x"), doc.intrinsics.fx);
    root.insert(QStringLiteral("fl_y"), doc.intrinsics.fy);
    root.insert(QStringLiteral("cx"), doc.intrinsics.cx);
    root.insert(QStringLiteral("cy"), doc.intrinsics.cy);
    if (!doc.intrinsics.distortion.empty()) {
      const auto &d = doc.intrinsics.distortion;
      if (d.size() >= 1)
        root.insert(QStringLiteral("k1"), d[0]);
      if (d.size() >= 2)
        root.insert(QStringLiteral("k2"), d[1]);
      if (d.size() >= 3)
        root.insert(QStringLiteral("p1"), d[2]);
      if (d.size() >= 4)
        root.insert(QStringLiteral("p2"), d[3]);
      if (d.size() >= 5)
        root.insert(QStringLiteral("k3"), d[4]);
    }
  }

  QJsonArray frames;
  if (appendExisting) {
    const QString existingPath = dir.filePath(QStringLiteral("transforms.json"));
    QFile existing(existingPath);
    if (existing.exists() && existing.open(QIODevice::ReadOnly | QIODevice::Text)) {
      const QJsonDocument existingDoc = QJsonDocument::fromJson(existing.readAll());
      existing.close();
      if (existingDoc.isObject()) {
        const QJsonArray prior = existingDoc.object().value(QStringLiteral("frames")).toArray();
        for (const QJsonValue &value : prior)
          frames.append(value);
      }
    }
  }
  for (const TransformsJsonFrame &frame : doc.frames) {
    QJsonObject entry;
    entry.insert(QStringLiteral("file_path"), frame.filePathStem);

    QJsonArray matrix;
    for (int row = 0; row < 4; ++row) {
      QJsonArray rowArr;
      for (int col = 0; col < 4; ++col)
        rowArr.append(
            frame.transformMatrix[static_cast<std::size_t>(row * 4 + col)]);
      matrix.append(rowArr);
    }
    entry.insert(QStringLiteral("transform_matrix"), matrix);
    entry.insert(QStringLiteral("extrinsics"),
                 extrinsicsToJson(frame.extrinsics));
    if (frame.fppStepIndex >= 0)
    {
      entry.insert(QStringLiteral("fpp_step_index"), frame.fppStepIndex);
      if (!frame.fppStepLabel.isEmpty())
        entry.insert(QStringLiteral("fpp_step_label"), frame.fppStepLabel);
      if (!frame.fppPattern.isEmpty())
        entry.insert(QStringLiteral("fpp_pattern"), frame.fppPattern);
    }
    frames.append(entry);
  }
  root.insert(QStringLiteral("frames"), frames);

  const QString path = dir.filePath(QStringLiteral("transforms.json"));
  QFile file(path);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate |
                 QIODevice::Text)) {
    if (errorMessage != nullptr)
      *errorMessage = QStringLiteral("Could not write %1").arg(path);
    return false;
  }

  const QByteArray payload =
      QJsonDocument(root).toJson(QJsonDocument::Indented);
  if (file.write(payload) != payload.size()) {
    if (errorMessage != nullptr)
      *errorMessage = QStringLiteral("Incomplete write: %1").arg(path);
    return false;
  }
  return true;
}

bool writeCameraPoseJson(const QString &jsonPath, const Ur3eScanTcpPose &tcp,
                         const Mat4 &cameraToWorldOpenGl,
                         const CameraExtrinsicsRt &extrinsics,
                         const CameraIntrinsics &intrinsics,
                         const QString &imageFileName,
                         const QString &poseSource,
                         const Ur3eScanTcpPose *plannedTcp,
                         QString *errorMessage,
                         const CalibrationCaptureExtras *calib) {
  if (jsonPath.isEmpty()) {
    if (errorMessage != nullptr)
      *errorMessage = QStringLiteral("camera pose JSON path is empty.");
    return false;
  }

  // Optical axis in parent frame = OpenCV camera +Z = R column 2.
  const double opticalX = extrinsics.R[0][2];
  const double opticalY = extrinsics.R[1][2];
  const double opticalZ = extrinsics.R[2][2];

  QJsonObject root;
  root.insert(QStringLiteral("file_path"), imageFileName);
  root.insert(QStringLiteral("coordinate_convention"),
              QStringLiteral("opengl"));
  root.insert(QStringLiteral("frame"), QStringLiteral("camera_optical"));
  root.insert(QStringLiteral("optical_tip"), QStringLiteral("camera"));
  root.insert(QStringLiteral("moveit_tip"),
              hf::hardwareConfig().ur3e.usesDlpScanTcp()
                  ? QStringLiteral("dlp")
                  : QStringLiteral("camera"));
  root.insert(QStringLiteral("parent_frame"), QStringLiteral("base_link"));
  root.insert(QStringLiteral("units"), QStringLiteral("metres"));
  root.insert(QStringLiteral("pose_source"), poseSource);
  root.insert(
      QStringLiteral("note"),
      QStringLiteral(
          "BFS camera optical in base_link = live tool0 ⊗ cfg tool_tcp_* "
          "(xyz+rpy). Always camera — even when MoveIt tip hyperfusion_tcp "
          "is remapped to dlp_tcp_* (scan_tcp=dlp). base_T_flange is live TF "
          "base_link→tool0 (hand–eye gripper). Do not use this tip as the "
          "gripper for calibrateHandEye."));

  QJsonObject position;
  position.insert(QStringLiteral("x_m"), tcp.xM);
  position.insert(QStringLiteral("y_m"), tcp.yM);
  position.insert(QStringLiteral("z_m"), tcp.zM);
  root.insert(QStringLiteral("position_m"), position);

  if (plannedTcp != nullptr) {
    QJsonObject planned;
    planned.insert(QStringLiteral("x_m"), plannedTcp->xM);
    planned.insert(QStringLiteral("y_m"), plannedTcp->yM);
    planned.insert(QStringLiteral("z_m"), plannedTcp->zM);
    planned.insert(QStringLiteral("rx"), plannedTcp->rxRad);
    planned.insert(QStringLiteral("ry"), plannedTcp->ryRad);
    planned.insert(QStringLiteral("rz"), plannedTcp->rzRad);
    planned.insert(QStringLiteral("frame"), QStringLiteral("world"));
    planned.insert(QStringLiteral("note"),
                   QStringLiteral("MoveIt hemisphere pin (tray/world)"));
    root.insert(QStringLiteral("planned_world_position_m"), planned);
  }

  QJsonObject rotvec;
  rotvec.insert(QStringLiteral("rx"), tcp.rxRad);
  rotvec.insert(QStringLiteral("ry"), tcp.ryRad);
  rotvec.insert(QStringLiteral("rz"), tcp.rzRad);
  root.insert(QStringLiteral("rotation_vector_rad"), rotvec);

  QJsonObject toolZ;
  toolZ.insert(QStringLiteral("x"), opticalX);
  toolZ.insert(QStringLiteral("y"), opticalY);
  toolZ.insert(QStringLiteral("z"), opticalZ);
  root.insert(QStringLiteral("tool_z"), toolZ);
  root.insert(QStringLiteral("optical_axis_base_link"), toolZ);

  QJsonArray matrix;
  for (int row = 0; row < 4; ++row) {
    QJsonArray rowArr;
    for (int col = 0; col < 4; ++col)
      rowArr.append(
          cameraToWorldOpenGl[static_cast<std::size_t>(row * 4 + col)]);
    matrix.append(rowArr);
  }
  root.insert(QStringLiteral("transform_matrix"), matrix);

  root.insert(QStringLiteral("intrinsics"), intrinsicsToJson(intrinsics));
  root.insert(QStringLiteral("extrinsics"), extrinsicsToJson(extrinsics));

  if (intrinsics.width > 0)
    root.insert(QStringLiteral("w"), intrinsics.width);
  if (intrinsics.height > 0)
    root.insert(QStringLiteral("h"), intrinsics.height);

  const bool haveFlange = calib != nullptr && calib->haveFlange;
  root.insert(QStringLiteral("hand_eye_ready"), haveFlange);
  if (haveFlange)
    root.insert(QStringLiteral("base_T_flange"), baseTFlangeJson(*calib));
  else
    root.insert(QStringLiteral("base_T_flange"), QJsonValue::Null);

  if (calib != nullptr
      && (calib->fppStepIndex >= 0 || !calib->fppPattern.isEmpty()))
  {
    if (calib->fppStepIndex >= 0)
      root.insert(QStringLiteral("fpp_step_index"), calib->fppStepIndex);
    if (!calib->fppStepLabel.isEmpty())
      root.insert(QStringLiteral("fpp_step_label"), calib->fppStepLabel);
    if (!calib->fppPattern.isEmpty())
      root.insert(QStringLiteral("fpp_pattern"), calib->fppPattern);
  }

  if (calib != nullptr && calib->haveDlpLed)
  {
    QJsonObject led;
    led.insert(QStringLiteral("red_ma"), calib->dlpLedRedMa);
    led.insert(QStringLiteral("green_ma"), calib->dlpLedGreenMa);
    led.insert(QStringLiteral("blue_ma"), calib->dlpLedBlueMa);
    const int r = calib->dlpLedRedMa;
    const int g = calib->dlpLedGreenMa;
    const int b = calib->dlpLedBlueMa;
    const int mx = std::max({r, g, b});
    QString channel = QStringLiteral("luma");
    if (mx >= 50)
    {
      const int second = (r == mx) ? std::max(g, b) : (g == mx) ? std::max(r, b) : std::max(r, g);
      if (second * 2 <= mx)
      {
        if (r == mx)
          channel = QStringLiteral("r");
        else if (g == mx)
          channel = QStringLiteral("g");
        else
          channel = QStringLiteral("b");
      }
    }
    led.insert(QStringLiteral("decode_channel"), channel);
    root.insert(QStringLiteral("dlp_led"), led);
    root.insert(QStringLiteral("decode_channel"), channel);
  }

  if (calib != nullptr && calib->haveBfsCapture)
  {
    QJsonObject bfs;
    if (!calib->bfsCameraId.isEmpty())
      bfs.insert(QStringLiteral("camera_id"), calib->bfsCameraId);
    if (!calib->bfsExposureMode.isEmpty())
      bfs.insert(QStringLiteral("exposure_mode"), calib->bfsExposureMode);
    if (!calib->bfsExposureAuto.isEmpty())
      bfs.insert(QStringLiteral("exposure_auto"), calib->bfsExposureAuto);
    bfs.insert(QStringLiteral("exposure_time_us"), calib->bfsExposureTimeUs);
    if (!calib->bfsGainAuto.isEmpty())
      bfs.insert(QStringLiteral("gain_auto"), calib->bfsGainAuto);
    bfs.insert(QStringLiteral("gain_db"), calib->bfsGainDb);
    bfs.insert(QStringLiteral("gamma_enable"), calib->bfsGammaEnable);
    bfs.insert(QStringLiteral("gamma"), calib->bfsGamma);
    if (!calib->bfsBalanceWhiteAuto.isEmpty())
      bfs.insert(QStringLiteral("balance_white_auto"), calib->bfsBalanceWhiteAuto);
    if (!calib->bfsBalanceRatioSelector.isEmpty())
      bfs.insert(QStringLiteral("balance_ratio_selector"), calib->bfsBalanceRatioSelector);
    bfs.insert(QStringLiteral("balance_ratio"), calib->bfsBalanceRatio);
    bfs.insert(QStringLiteral("acquisition_frame_rate_enable"),
               calib->bfsAcquisitionFrameRateEnable);
    bfs.insert(QStringLiteral("acquisition_frame_rate_hz"), calib->bfsAcquisitionFrameRateHz);
    bfs.insert(QStringLiteral("device_link_throughput_limit"),
               calib->bfsDeviceLinkThroughputLimit);
    bfs.insert(QStringLiteral("black_level_percent"), calib->bfsBlackLevelPercent);
    bfs.insert(QStringLiteral("ev_compensation"), calib->bfsEvCompensation);
    bfs.insert(QStringLiteral("note"),
               QStringLiteral("BFS UI / applied settings at still capture time."));
    root.insert(QStringLiteral("bfs_capture"), bfs);
  }

  if (calib != nullptr && calib->haveOutputStageShift)
  {
    QJsonObject stage;
    stage.insert(QStringLiteral("capture_position_mm"), calib->stageCapturePositionMm);
    stage.insert(QStringLiteral("output_position_mm"), calib->stageOutputPositionMm);
    QJsonObject shift;
    shift.insert(QStringLiteral("x_m"), calib->outputShiftXM);
    shift.insert(QStringLiteral("y_m"), calib->outputShiftYM);
    shift.insert(QStringLiteral("z_m"), calib->outputShiftZM);
    stage.insert(QStringLiteral("output_translation_m"), shift);
    stage.insert(QStringLiteral("note"),
                 QStringLiteral("Camera t / extrinsics translated so the sample looks fixed at "
                                "output_position_mm (stage static); equivalent to arm motion "
                                "only. FPP undoes output_translation_m to recover room TF."));
    root.insert(QStringLiteral("stage_output"), stage);
  }

  if (calib != nullptr && !calib->jointsRad.empty()) {
    QJsonArray joints;
    for (double q : calib->jointsRad)
      joints.append(q);
    root.insert(QStringLiteral("joints_rad"), joints);
    QJsonArray names;
    for (const QString &name : calib->jointNames)
      names.append(name);
    if (!names.isEmpty())
      root.insert(QStringLiteral("joint_names"), names);
  }

  QFile file(jsonPath);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate |
                 QIODevice::Text)) {
    if (errorMessage != nullptr)
      *errorMessage = QStringLiteral("Could not write %1").arg(jsonPath);
    return false;
  }

  const QByteArray payload =
      QJsonDocument(root).toJson(QJsonDocument::Indented);
  if (file.write(payload) != payload.size()) {
    if (errorMessage != nullptr)
      *errorMessage = QStringLiteral("Incomplete write: %1").arg(jsonPath);
    return false;
  }
  return true;
}

} // namespace hf::ur3e
