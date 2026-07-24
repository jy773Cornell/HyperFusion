// Optical-TCP pose → nerfstudio-style transforms.json (Capture 3D RGB).

#include "backend/3dscanning/Ur3eCameraTransforms.hpp"

#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTextStream>

#include <cmath>

namespace hf::ur3e
{
namespace
{

void rotVecToMatrix(const double rx,
                    const double ry,
                    const double rz,
                    double R[3][3])
{
    const double angle = std::sqrt(rx * rx + ry * ry + rz * rz);
    if (angle < 1e-12)
    {
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

Mat4 identityMat4()
{
    Mat4 m{};
    m[0] = 1.0;
    m[5] = 1.0;
    m[10] = 1.0;
    m[15] = 1.0;
    return m;
}

} // namespace

Mat4 cameraToWorldOpenGlFromTcp(const Ur3eScanTcpPose &tcp)
{
    double R[3][3];
    rotVecToMatrix(tcp.rxRad, tcp.ryRad, tcp.rzRad, R);

    // OpenCV-style c2w: columns = camera X/Y/Z in world; +Z = optical axis (tool +Z).
    Mat4 cv = identityMat4();
    cv[0] = R[0][0];
    cv[1] = R[0][1];
    cv[2] = R[0][2];
    cv[3] = tcp.xM;
    cv[4] = R[1][0];
    cv[5] = R[1][1];
    cv[6] = R[1][2];
    cv[7] = tcp.yM;
    cv[8] = R[2][0];
    cv[9] = R[2][1];
    cv[10] = R[2][2];
    cv[11] = tcp.zM;

    // OpenCV → OpenGL (nerfstudio): flip Y and Z of the camera basis.
    // c2w_gl = c2w_cv * diag(1, -1, -1, 1)
    Mat4 gl = identityMat4();
    for (int row = 0; row < 3; ++row)
    {
        const int r = row * 4;
        gl[static_cast<std::size_t>(r + 0)] = cv[static_cast<std::size_t>(r + 0)];
        gl[static_cast<std::size_t>(r + 1)] = -cv[static_cast<std::size_t>(r + 1)];
        gl[static_cast<std::size_t>(r + 2)] = -cv[static_cast<std::size_t>(r + 2)];
        gl[static_cast<std::size_t>(r + 3)] = cv[static_cast<std::size_t>(r + 3)];
    }
    return gl;
}

bool writeTransformsJson(const QString &directory,
                         const TransformsJsonDocument &doc,
                         QString *errorMessage)
{
    if (directory.isEmpty())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("transforms.json directory is empty.");
        return false;
    }

    QDir dir(directory);
    if (!dir.exists() && !dir.mkpath(QStringLiteral(".")))
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Could not create directory: %1").arg(directory);
        return false;
    }

    QJsonObject root;
    root.insert(QStringLiteral("camera_model"), QStringLiteral("OPENCV"));
    root.insert(QStringLiteral("coordinate_convention"), QStringLiteral("opengl"));
    root.insert(QStringLiteral("note"),
                QStringLiteral("camera_to_world from HyperFusion optical TCP "
                               "(hyperfusion_tcp); intrinsics not set."));
    if (doc.width > 0)
        root.insert(QStringLiteral("w"), doc.width);
    if (doc.height > 0)
        root.insert(QStringLiteral("h"), doc.height);

    QJsonArray frames;
    for (const TransformsJsonFrame &frame : doc.frames)
    {
        QJsonObject entry;
        entry.insert(QStringLiteral("file_path"), frame.filePathStem);

        QJsonArray matrix;
        for (int row = 0; row < 4; ++row)
        {
            QJsonArray rowArr;
            for (int col = 0; col < 4; ++col)
                rowArr.append(frame.transformMatrix[static_cast<std::size_t>(row * 4 + col)]);
            matrix.append(rowArr);
        }
        entry.insert(QStringLiteral("transform_matrix"), matrix);
        frames.append(entry);
    }
    root.insert(QStringLiteral("frames"), frames);

    const QString path = dir.filePath(QStringLiteral("transforms.json"));
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text))
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Could not write %1").arg(path);
        return false;
    }

    const QByteArray payload = QJsonDocument(root).toJson(QJsonDocument::Indented);
    if (file.write(payload) != payload.size())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Incomplete write: %1").arg(path);
        return false;
    }
    return true;
}

bool writeCameraPoseJson(const QString &jsonPath,
                         const Ur3eScanTcpPose &tcp,
                         const Mat4 &cameraToWorldOpenGl,
                         const QString &imageFileName,
                         const int width,
                         const int height,
                         QString *errorMessage)
{
    if (jsonPath.isEmpty())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("camera pose JSON path is empty.");
        return false;
    }

    QJsonObject root;
    root.insert(QStringLiteral("file_path"), imageFileName);
    root.insert(QStringLiteral("coordinate_convention"), QStringLiteral("opengl"));
    root.insert(QStringLiteral("frame"), QStringLiteral("hyperfusion_tcp"));
    root.insert(QStringLiteral("note"),
                QStringLiteral("camera_to_world from optical TCP (UR rotvec); "
                               "position metres, rotvec radians."));

    QJsonObject position;
    position.insert(QStringLiteral("x_m"), tcp.xM);
    position.insert(QStringLiteral("y_m"), tcp.yM);
    position.insert(QStringLiteral("z_m"), tcp.zM);
    root.insert(QStringLiteral("position_m"), position);

    QJsonObject rotvec;
    rotvec.insert(QStringLiteral("rx"), tcp.rxRad);
    rotvec.insert(QStringLiteral("ry"), tcp.ryRad);
    rotvec.insert(QStringLiteral("rz"), tcp.rzRad);
    root.insert(QStringLiteral("rotation_vector_rad"), rotvec);

    QJsonObject toolZ;
    toolZ.insert(QStringLiteral("x"), tcp.toolZMx);
    toolZ.insert(QStringLiteral("y"), tcp.toolZMy);
    toolZ.insert(QStringLiteral("z"), tcp.toolZMz);
    root.insert(QStringLiteral("tool_z"), toolZ);

    QJsonArray matrix;
    for (int row = 0; row < 4; ++row)
    {
        QJsonArray rowArr;
        for (int col = 0; col < 4; ++col)
            rowArr.append(cameraToWorldOpenGl[static_cast<std::size_t>(row * 4 + col)]);
        matrix.append(rowArr);
    }
    root.insert(QStringLiteral("transform_matrix"), matrix);

    if (width > 0)
        root.insert(QStringLiteral("w"), width);
    if (height > 0)
        root.insert(QStringLiteral("h"), height);

    QFile file(jsonPath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text))
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Could not write %1").arg(jsonPath);
        return false;
    }

    const QByteArray payload = QJsonDocument(root).toJson(QJsonDocument::Indented);
    if (file.write(payload) != payload.size())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Incomplete write: %1").arg(jsonPath);
        return false;
    }
    return true;
}

} // namespace hf::ur3e
