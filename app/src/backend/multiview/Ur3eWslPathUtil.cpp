// Windows ↔ WSL path helpers for UR3e sidecar integration.
#include "backend/multiview/Ur3eWslPathUtil.hpp"

#include "backend/HyperFusionConfig.hpp"
#include "backend/camera/processing/GsamWslPathUtil.hpp"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>

namespace hf::ur3e
{
QString resolveUr3eResourcesWindowsPath()
{
    QDir dir(QCoreApplication::applicationDirPath());
    for (int depth = 0; depth < 8; ++depth)
    {
        const QString candidate = dir.filePath(QStringLiteral("sidecars/ur3e"));
        if (QFileInfo::exists(candidate))
            return QFileInfo(candidate).absoluteFilePath();

        const QString alt = dir.filePath(QStringLiteral("resources/ur3e"));
        if (QFileInfo::exists(alt))
            return QFileInfo(alt).absoluteFilePath();

        if (!dir.cdUp())
            break;
    }

    return {};
}

QString resolveUr3eRepoLinuxPath()
{
    const hf::HardwareConfig::Ur3eConfig &cfg = hf::hardwareConfig().ur3e;
    if (!cfg.ur3eRepoLinux.trimmed().isEmpty())
        return cfg.ur3eRepoLinux.trimmed();

    const QString windowsPath = resolveUr3eResourcesWindowsPath();
    if (windowsPath.isEmpty())
        return {};

    return hf::processing::windowsPathToWsl(windowsPath);
}

QString buildMountEnvExports(const hf::HardwareConfig::Ur3eConfig &cfg)
{
    return QStringLiteral("export HYPERFUSION_CEILING_MOUNT='true' && "
                          "export HYPERFUSION_CEILING_MOUNT_HEIGHT_M='%1' && "
                          "export HYPERFUSION_MOUNT_ROLL_DEG='%2' && "
                          "export HYPERFUSION_MOUNT_PITCH_DEG='%3' && "
                          "export HYPERFUSION_MOUNT_YAW_DEG='%4' && "
                          "export HYPERFUSION_MOUNT_OFFSET_X_M='%5' && "
                          "export HYPERFUSION_MOUNT_OFFSET_Y_M='%6' && ")
        .arg(QString::number(cfg.ceilingMountHeightMm / 1000.0, 'f', 6),
             QString::number(cfg.mountRollDeg, 'g', 6),
             QString::number(cfg.mountPitchDeg, 'g', 6),
             QString::number(cfg.mountYawDeg, 'g', 6),
             QString::number(cfg.mountOffsetXMm / 1000.0, 'f', 6),
             QString::number(cfg.mountOffsetYMm / 1000.0, 'f', 6));
}

QString buildToolPayloadEnvExports(const hf::HardwareConfig::Ur3eConfig &cfg)
{
    const QString shape =
        cfg.toolPayloadShape.trimmed().isEmpty() ? QStringLiteral("mesh") : cfg.toolPayloadShape.trimmed();
    const QString mesh =
        cfg.toolPayloadMesh.trimmed().isEmpty() ? QStringLiteral("ur_tool_payload.stl")
                                                : cfg.toolPayloadMesh.trimmed();
    const auto tcp = cfg.cameraToolTcpMm();
    return QStringLiteral("export HYPERFUSION_TOOL_PAYLOAD_ENABLED='true' && "
                          "export HYPERFUSION_TOOL_PAYLOAD_SHAPE='%1' && "
                          "export HYPERFUSION_TOOL_PAYLOAD_MESH_FILE='%2' && "
                          "export HYPERFUSION_TOOL_PAYLOAD_RADIUS_M='%3' && "
                          "export HYPERFUSION_TOOL_TCP_X_M='%4' && "
                          "export HYPERFUSION_TOOL_TCP_Y_M='%5' && "
                          "export HYPERFUSION_TOOL_TCP_Z_M='%6' && "
                          "export HYPERFUSION_TOOL_TCP_ROLL_DEG='%7' && "
                          "export HYPERFUSION_TOOL_TCP_PITCH_DEG='%8' && "
                          "export HYPERFUSION_TOOL_TCP_YAW_DEG='%9' && ")
        .arg(shape,
             mesh,
             QString::number(cfg.toolPayloadRadiusMm / 1000.0, 'f', 6),
             QString::number(tcp.xMm / 1000.0, 'f', 6),
             QString::number(tcp.yMm / 1000.0, 'f', 6),
             QString::number(tcp.zMm / 1000.0, 'f', 6),
             QString::number(tcp.rollDeg, 'f', 6),
             QString::number(tcp.pitchDeg, 'f', 6),
             QString::number(tcp.yawDeg, 'f', 6));
}

} // namespace hf::ur3e
