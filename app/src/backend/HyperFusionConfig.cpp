#include "backend/HyperFusionConfig.hpp"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QSaveFile>
#include <QTextStream>

#include <algorithm>
#include <cmath>

namespace
{
hf::HardwareConfig g_hardwareConfig;

QString trimmedLine(QString line)
{
    return line.trimmed();
}

bool isCommentOrEmpty(const QString &line)
{
    return line.isEmpty() || line.startsWith(QLatin1Char('#')) || line.startsWith(QLatin1Char(';'));
}

bool parseSectionName(const QString &line, QString &sectionOut)
{
    if (!line.startsWith(QLatin1Char('[')) || !line.endsWith(QLatin1Char(']')))
        return false;

    sectionOut = line.mid(1, line.size() - 2).trimmed().toLower();
    return !sectionOut.isEmpty();
}

bool parseKeyValue(const QString &line, QString &keyOut, QString &valueOut)
{
    const int equals = line.indexOf(QLatin1Char('='));
    if (equals <= 0)
        return false;

    keyOut = line.left(equals).trimmed().toLower();
    valueOut = line.mid(equals + 1).trimmed();
    return !keyOut.isEmpty() && !valueOut.isEmpty();
}

bool parseDouble(const QString &text, double &valueOut)
{
    bool ok = false;
    const double parsed = text.toDouble(&ok);
    if (!ok || !std::isfinite(parsed))
        return false;

    valueOut = parsed;
    return true;
}

bool parsePercent(const QString &text, int &valueOut)
{
    bool ok = false;
    const int parsed = text.toInt(&ok);
    if (!ok || parsed < 0 || parsed > 100)
        return false;

    valueOut = parsed;
    return true;
}

void assignCameraPosition(hf::HardwareConfig &config,
                          const int cameraIndex,
                          const double valueMm,
                          QStringList &warnings)
{
    if (cameraIndex < 0 || cameraIndex >= 2)
        return;

    if (valueMm < 0.0)
    {
        warnings.push_back(QStringLiteral("camera%1_mm must be >= 0 (ignored %2)")
                               .arg(cameraIndex + 1)
                               .arg(valueMm, 0, 'f', 3));
        return;
    }

    config.cameraPositionMm[static_cast<std::size_t>(cameraIndex)] = valueMm;
}

bool parseConfigLines(const QStringList &lines, hf::HardwareConfig &config, QStringList &warnings)
{
    QString section;

    for (const QString &rawLine : lines)
    {
        const QString line = trimmedLine(rawLine);
        if (isCommentOrEmpty(line))
            continue;

        QString sectionName;
        if (parseSectionName(line, sectionName))
        {
            section = sectionName;
            continue;
        }

        QString key;
        QString value;
        if (!parseKeyValue(line, key, value))
        {
            warnings.push_back(QStringLiteral("Ignored line: %1").arg(line));
            continue;
        }

        double numericValue = 0.0;
        const bool hasNumber = parseDouble(value, numericValue);

        if (section == QStringLiteral("camera_positions"))
        {
            if (key == QStringLiteral("fx10e_mm") || key == QStringLiteral("camera1_mm")
                || key == QStringLiteral("camera_1_mm"))
            {
                if (!hasNumber)
                    warnings.push_back(QStringLiteral("Invalid fx10e_mm value: %1").arg(value));
                else
                    assignCameraPosition(config, 0, numericValue, warnings);
            }
            else if (key == QStringLiteral("swir3_mm") || key == QStringLiteral("camera2_mm")
                     || key == QStringLiteral("camera_2_mm"))
            {
                if (!hasNumber)
                    warnings.push_back(QStringLiteral("Invalid swir3_mm value: %1").arg(value));
                else
                    assignCameraPosition(config, 1, numericValue, warnings);
            }
            else if (key == QStringLiteral("distance_dual_camera_mm"))
            {
                // Informational; derived from fx10e_mm - swir3_mm.
            }
            else
            {
                warnings.push_back(QStringLiteral("Unknown key in [camera_positions]: %1").arg(key));
            }
        }
        else if (section == QStringLiteral("sample_stage_position"))
        {
            if (key == QStringLiteral("front_edge_sample_window_mm"))
            {
                if (!hasNumber)
                    warnings.push_back(
                        QStringLiteral("Invalid front_edge_sample_window_mm: %1").arg(value));
                else if (numericValue < 0.0)
                    warnings.push_back(QStringLiteral("front_edge_sample_window_mm must be >= 0"));
                else
                    config.frontEdgeSampleWindowMm = numericValue;
            }
            else if (key == QStringLiteral("sample_window_length_mm"))
            {
                if (!hasNumber)
                    warnings.push_back(QStringLiteral("Invalid sample_window_length_mm: %1").arg(value));
                else if (numericValue <= 0.0)
                    warnings.push_back(QStringLiteral("sample_window_length_mm must be > 0"));
                else
                    config.sampleWindowLengthMm = numericValue;
            }
            else if (key == QStringLiteral("scanning_starting_position_mm"))
            {
                if (!hasNumber)
                    warnings.push_back(
                        QStringLiteral("Invalid scanning_starting_position_mm: %1").arg(value));
                else if (numericValue < 0.0)
                    warnings.push_back(QStringLiteral("scanning_starting_position_mm must be >= 0"));
                else
                    config.scanningStartingPositionMm = numericValue;
            }
            else
            {
                warnings.push_back(QStringLiteral("Unknown key in [sample_stage_position]: %1").arg(key));
            }
        }
        else if (section == QStringLiteral("scanning_settings"))
        {
            if (key == QStringLiteral("operation_scanning_speed_mm_per_sec"))
            {
                if (!hasNumber)
                    warnings.push_back(
                        QStringLiteral("Invalid operation_scanning_speed_mm_per_sec: %1").arg(value));
                else if (numericValue <= 0.0)
                    warnings.push_back(QStringLiteral("operation_scanning_speed_mm_per_sec must be > 0"));
                else
                    config.operationScanningSpeedMmPerSec = numericValue;
            }
            else if (key == QStringLiteral("record_scanning_speed_mm_per_sec")
                     || key == QStringLiteral("scanning_speed_mm_per_sec"))
            {
                if (!hasNumber)
                    warnings.push_back(QStringLiteral("Invalid record_scanning_speed_mm_per_sec: %1").arg(value));
                else if (numericValue <= 0.0)
                    warnings.push_back(QStringLiteral("record_scanning_speed_mm_per_sec must be > 0"));
                else
                    config.recordScanningSpeedMmPerSec = numericValue;
            }
            else if (key == QStringLiteral("white_reference_scanning_length_mm"))
            {
                if (!hasNumber)
                    warnings.push_back(
                        QStringLiteral("Invalid white_reference_scanning_length_mm: %1").arg(value));
                else if (numericValue <= 0.0)
                    warnings.push_back(QStringLiteral("white_reference_scanning_length_mm must be > 0"));
                else
                    config.whiteReferenceScanningLengthMm = numericValue;
            }
            else if (key == QStringLiteral("black_reference_frames"))
            {
                bool ok = false;
                const int frames = value.toInt(&ok);
                if (!ok || frames <= 0)
                    warnings.push_back(QStringLiteral("Invalid black_reference_frames: %1").arg(value));
                else
                    config.blackReferenceFrames = frames;
            }
            else
            {
                warnings.push_back(QStringLiteral("Unknown key in [scanning_settings]: %1").arg(key));
            }
        }
        else if (section == QStringLiteral("white_reference"))
        {
            if (key == QStringLiteral("position_mm"))
            {
                warnings.push_back(
                    QStringLiteral("Deprecated [white_reference] section — use [camera_positions] instead"));
            }
            else
            {
                warnings.push_back(QStringLiteral("Unknown key in [white_reference]: %1").arg(key));
            }
        }
        else if (section == QStringLiteral("calibration"))
        {
            if (key == QStringLiteral("spatial_mm_per_pixel") || key == QStringLiteral("spatial_distance_per_pixel_mm"))
            {
                if (!hasNumber)
                    warnings.push_back(QStringLiteral("Invalid spatial_mm_per_pixel: %1").arg(value));
                else if (numericValue <= 0.0)
                    warnings.push_back(QStringLiteral("spatial_mm_per_pixel must be > 0"));
                else
                    config.spatialMmPerPixel = numericValue;
            }
            else
            {
                warnings.push_back(QStringLiteral("Unknown key in [calibration]: %1").arg(key));
            }
        }
        else if (section == QStringLiteral("lighthouse"))
        {
            int percent = 0;
            if (key == QStringLiteral("idle_intensity_percent"))
            {
                if (!parsePercent(value, percent))
                    warnings.push_back(
                        QStringLiteral("Invalid lighthouse idle_intensity_percent: %1").arg(value));
                else
                    config.lighthouseIdleIntensityPercent = percent;
            }
            else if (key == QStringLiteral("reflectance_percent") || key == QStringLiteral("reflectance_percent_default")
                || key == QStringLiteral("reflectance_intensity_percent"))
            {
                if (!parsePercent(value, percent))
                    warnings.push_back(QStringLiteral("Invalid lighthouse reflectance_percent: %1").arg(value));
                else
                    config.lighthouseReflectancePercent = percent;
            }
            else if (key == QStringLiteral("transmittance_intensity_percent")
                     || key == QStringLiteral("transmission_percent")
                     || key == QStringLiteral("transmission_percent_default")
                     || key == QStringLiteral("transmission_intensity_percent"))
            {
                if (!parsePercent(value, percent))
                    warnings.push_back(
                        QStringLiteral("Invalid lighthouse transmittance_intensity_percent: %1").arg(value));
                else
                    config.lighthouseTransmittancePercent = percent;
            }
            else
            {
                warnings.push_back(QStringLiteral("Unknown key in [lighthouse]: %1").arg(key));
            }
        }
        else if (section == QStringLiteral("stage_motion"))
        {
            if (key == QStringLiteral("acceleration_mm_per_sec2"))
            {
                if (!hasNumber)
                    warnings.push_back(
                        QStringLiteral("Invalid stage_motion acceleration_mm_per_sec2: %1").arg(value));
                else if (numericValue <= 0.0)
                    warnings.push_back(QStringLiteral("stage_motion acceleration_mm_per_sec2 must be > 0"));
                else
                {
                    config.stageMotionAccelerationMmPerSec2 = numericValue;
                    if (numericValue < 20.0)
                    {
                        warnings.push_back(
                            QStringLiteral("stage_motion acceleration_mm_per_sec2=%1 may round to zero on NMS23; "
                                           "effective value will be raised at connect")
                                .arg(numericValue));
                    }
                }
            }
            else
            {
                warnings.push_back(QStringLiteral("Unknown key in [stage_motion]: %1").arg(key));
            }
        }
        else if (section.isEmpty())
        {
            warnings.push_back(QStringLiteral("Key outside a section (ignored): %1").arg(key));
        }
        else
        {
            warnings.push_back(QStringLiteral("Unknown section [%1]").arg(section));
        }
    }

    return true;
}
} // namespace

namespace hf
{
QStringList hyperFusionConfigSearchPaths()
{
    QStringList paths;

    if (QCoreApplication::instance() != nullptr)
        paths << QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("hyperfusion.cfg"));

#ifdef HF_APP_SOURCE_DIR
    paths << QDir(QString::fromUtf8(HF_APP_SOURCE_DIR)).filePath(QStringLiteral("hyperfusion.cfg"));
#endif

    paths.removeDuplicates();
    return paths;
}

bool writeDefaultHardwareConfigFile(const QString &path, QString *errorMessage)
{
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate))
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Could not write %1").arg(path);
        return false;
    }

    QTextStream out(&file);
    out << "# HyperFusion hardware configuration\n"
        << "# Edit this file manually. Values are reloaded on every app start.\n"
        << "# Distances are in millimetres unless noted.\n"
        << "\n"
        << "[camera_positions]\n"
        << "# Default stage position for each camera at the white reference.\n"
        << "fx10e_mm = 735\n"
        << "swir3_mm = 545\n"
        << "\n"
        << "[sample_stage_position]\n"
        << "front_edge_sample_window_mm = 760\n"
        << "sample_window_length_mm = 550\n"
        << "scanning_starting_position_mm = 810\n"
        << "\n"
        << "[scanning_settings]\n"
        << "# All stage move speeds during preview/record.\n"
        << "operation_scanning_speed_mm_per_sec = 100\n"
        << "white_reference_scanning_length_mm = 10\n"
        << "black_reference_frames = 100\n"
        << "# White-reference and sample scan legs during preview/record.\n"
        << "record_scanning_speed_mm_per_sec = 15\n"
        << "\n"
        << "[calibration]\n"
        << "# Spatial scale along the scan axis (mm per detector pixel).\n"
        << "spatial_mm_per_pixel = 0.050\n"
        << "\n"
        << "[lighthouse]\n"
        << "idle_intensity_percent = 0\n"
        << "reflectance_intensity_percent = 100\n"
        << "transmittance_intensity_percent = 40\n"
        << "\n"
        << "[stage_motion]\n"
        << "# Trapezoidal accel for lockstep moves and stop deceleration (mm/s²). Lower = gentler.\n"
        << "acceleration_mm_per_sec2 = 30\n";

    if (!file.commit())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Could not save %1").arg(path);
        return false;
    }

    return true;
}

HardwareConfig loadHardwareConfig()
{
    HardwareConfig config;
    QStringList warnings;

    QString resolvedPath;
    for (const QString &candidate : hyperFusionConfigSearchPaths())
    {
        if (QFile::exists(candidate))
        {
            resolvedPath = candidate;
            break;
        }
    }

    if (resolvedPath.isEmpty() && QCoreApplication::instance() != nullptr)
    {
        const QString defaultPath =
            QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("hyperfusion.cfg"));
        QString writeError;
        if (writeDefaultHardwareConfigFile(defaultPath, &writeError))
        {
            resolvedPath = defaultPath;
            warnings.push_back(QStringLiteral("Created default hyperfusion.cfg at %1").arg(defaultPath));
        }
        else
        {
            warnings.push_back(writeError);
        }
    }

    if (resolvedPath.isEmpty())
    {
        config.warnings = warnings;
        config.warnings.push_back(QStringLiteral("hyperfusion.cfg not found — using built-in defaults"));
        setHardwareConfig(config);
        return config;
    }

    QFile file(resolvedPath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        config.filePath = resolvedPath;
        config.warnings.push_back(QStringLiteral("Could not open %1").arg(resolvedPath));
        setHardwareConfig(config);
        return config;
    }

    QTextStream in(&file);
    QStringList lines;
    while (!in.atEnd())
        lines.push_back(in.readLine());

    parseConfigLines(lines, config, warnings);

    config.filePath = resolvedPath;
    config.loadedFromFile = true;
    config.warnings = warnings;
    setHardwareConfig(config);
    return config;
}

const HardwareConfig &hardwareConfig()
{
    return g_hardwareConfig;
}

void setHardwareConfig(HardwareConfig config)
{
    g_hardwareConfig = std::move(config);
}
} // namespace hf
