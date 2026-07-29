// Optical-TCP pose → nerfstudio-style transforms.json (Capture 3D RGB).

#include "backend/3dscanning/Ur3eCameraTransforms.hpp"

#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

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

} // namespace

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
                         QString *errorMessage) {
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
                "Frames use optical TCP hyperfusion_tcp in base_link "
                "(live TF; includes tool_tcp_*). Intrinsics from "
                "hyperfusion.cfg; "
                "extrinsics R/t are OpenCV camera-to-base_link.")
          : QStringLiteral(
                "Frames use optical TCP hyperfusion_tcp in base_link "
                "(live TF; includes tool_tcp_*). Set bfs_camera_fx/fy (and "
                "cx/cy) "
                "in hyperfusion.cfg for full intrinsics; extrinsics R/t are "
                "OpenCV camera-to-base_link."));
  root.insert(QStringLiteral("parent_frame"), QStringLiteral("base_link"));
  root.insert(QStringLiteral("frame"), QStringLiteral("hyperfusion_tcp"));
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
                         QString *errorMessage) {
  if (jsonPath.isEmpty()) {
    if (errorMessage != nullptr)
      *errorMessage = QStringLiteral("camera pose JSON path is empty.");
    return false;
  }

  const bool haveFocal = intrinsics.fx > 0.0 && intrinsics.fy > 0.0;

  // Optical axis in parent frame = OpenCV camera +Z = R column 2.
  const double opticalX = extrinsics.R[0][2];
  const double opticalY = extrinsics.R[1][2];
  const double opticalZ = extrinsics.R[2][2];

  QJsonObject root;
  root.insert(QStringLiteral("file_path"), imageFileName);
  root.insert(QStringLiteral("coordinate_convention"),
              QStringLiteral("opengl"));
  root.insert(QStringLiteral("frame"), QStringLiteral("hyperfusion_tcp"));
  root.insert(QStringLiteral("parent_frame"), QStringLiteral("base_link"));
  root.insert(QStringLiteral("units"), QStringLiteral("metres"));
  root.insert(QStringLiteral("pose_source"), poseSource);
  root.insert(
      QStringLiteral("note"),
      haveFocal
          ? QStringLiteral(
                "Depth/pose: optical TCP hyperfusion_tcp in robot base_link "
                "(includes tool_tcp_*). Independent of tray/world mount. "
                "position_m / extrinsics.t = sensor-face centroid (m). "
                "extrinsics R/t = OpenCV camera-to-base_link (R col2 = optical "
                "+Z). "
                "transform_matrix = OpenGL c2w-style (Y/Z flipped). "
                "planned_world_position_m is MoveIt pin audit only. "
                "Intrinsics from hyperfusion.cfg.")
          : QStringLiteral(
                "Depth/pose: optical TCP hyperfusion_tcp in robot base_link "
                "(includes tool_tcp_*). Set bfs_camera_fx/fy in "
                "hyperfusion.cfg "
                "for full intrinsics."));

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
