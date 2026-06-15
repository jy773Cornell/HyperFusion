#include "backend/HyperFusionConfig.hpp"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QSaveFile>
#include <QTextStream>

#include <algorithm>
#include <cmath>

#include <QHash>

namespace
{
hf::HardwareConfig g_hardwareConfig;

struct SampleStagePositionDraft
{
    QHash<QString, QString> rawValues;
};

bool parseDouble(const QString &text, double &valueOut);

bool lookupResolvedValue(const QHash<QString, double> &resolved, const QString &token, double &valueOut)
{
    const QString key = token.trimmed().toLower();
    if (!resolved.contains(key))
        return false;

    valueOut = resolved.value(key);
    return true;
}

bool evaluateSampleStageExpression(const QString &expression,
                                   const QHash<QString, double> &resolved,
                                   double &valueOut)
{
    const QString trimmed = expression.trimmed();
    if (trimmed.isEmpty())
        return false;

    bool ok = false;
    const double direct = trimmed.toDouble(&ok);
    if (ok && std::isfinite(direct))
    {
        valueOut = direct;
        return true;
    }

    const int minusIndex = trimmed.indexOf(QStringLiteral(" - "));
    if (minusIndex > 0)
    {
        double lhs = 0.0;
        double rhs = 0.0;
        if (!lookupResolvedValue(resolved, trimmed.left(minusIndex), lhs)
            || !lookupResolvedValue(resolved, trimmed.mid(minusIndex + 3), rhs))
        {
            return false;
        }

        valueOut = lhs - rhs;
        return std::isfinite(valueOut);
    }

    return lookupResolvedValue(resolved, trimmed, valueOut);
}

bool resolveSampleStagePositions(const SampleStagePositionDraft &draft,
                                 hf::HardwareConfig &config,
                                 QStringList &warnings)
{
    QHash<QString, double> resolved;
    for (auto it = draft.rawValues.cbegin(); it != draft.rawValues.cend(); ++it)
    {
        double numericValue = 0.0;
        if (parseDouble(it.value(), numericValue))
            resolved.insert(it.key(), numericValue);
    }

    bool changed = true;
    int passCount = 0;
    while (changed && passCount < static_cast<int>(draft.rawValues.size()) + 2)
    {
        changed = false;
        ++passCount;

        for (auto it = draft.rawValues.cbegin(); it != draft.rawValues.cend(); ++it)
        {
            if (resolved.contains(it.key()))
                continue;

            double numericValue = 0.0;
            if (!evaluateSampleStageExpression(it.value(), resolved, numericValue))
                continue;

            resolved.insert(it.key(), numericValue);
            changed = true;
        }
    }

    const auto require = [&](const QString &key, double &target, const QString &label) -> bool {
        if (!resolved.contains(key))
        {
            warnings.push_back(QStringLiteral("Missing or unresolved %1 in [sample_stage_position]")
                                   .arg(label));
            return false;
        }

        const double value = resolved.value(key);
        if (!std::isfinite(value) || value < 0.0)
        {
            warnings.push_back(QStringLiteral("Invalid %1 in [sample_stage_position]").arg(label));
            return false;
        }

        target = value;
        return true;
    };

    bool ok = true;
    ok = require(QStringLiteral("distance_dual_camera_mm"), config.distanceDualCameraMm,
                 QStringLiteral("distance_dual_camera_mm"))
         && ok;
    ok = require(QStringLiteral("white_ref_fx10e_mm"), config.whiteRefMm[0],
                 QStringLiteral("white_ref_fx10e_mm"))
         && ok;
    ok = require(QStringLiteral("white_ref_swir3_mm"), config.whiteRefMm[1],
                 QStringLiteral("white_ref_swir3_mm"))
         && ok;
    ok = require(QStringLiteral("bright_ref_fx10e_mm"), config.brightRefMm[0],
                 QStringLiteral("bright_ref_fx10e_mm"))
         && ok;
    ok = require(QStringLiteral("bright_ref_swir3_mm"), config.brightRefMm[1],
                 QStringLiteral("bright_ref_swir3_mm"))
         && ok;
    ok = require(QStringLiteral("sample_scanning_starting_position_fx10e_mm"),
                 config.sampleScanStartMm[0],
                 QStringLiteral("sample_scanning_starting_position_fx10e_mm"))
         && ok;
    ok = require(QStringLiteral("sample_scanning_starting_position_swir3_mm"),
                 config.sampleScanStartMm[1],
                 QStringLiteral("sample_scanning_starting_position_swir3_mm"))
         && ok;
    ok = require(QStringLiteral("temp_stop_position_mm"), config.tempStopPositionMm,
                 QStringLiteral("temp_stop_position_mm"))
         && ok;

    config.cameraPositionMm[0] = config.whiteRefMm[0];
    config.cameraPositionMm[1] = config.whiteRefMm[1];
    return ok;
}

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
    SampleStagePositionDraft sampleStageDraft;

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
            sampleStageDraft.rawValues.insert(key, value);
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
                warnings.push_back(
                    QStringLiteral("Deprecated record_scanning_speed_mm_per_sec — record scan speed is "
                                   "frame rate × spatial_mm_per_pixel"));
            }
            else if (key == QStringLiteral("white_reference_frames"))
            {
                bool ok = false;
                const int frames = value.toInt(&ok);
                if (!ok || frames <= 0)
                    warnings.push_back(QStringLiteral("Invalid white_reference_frames: %1").arg(value));
                else
                    config.whiteReferenceFrames = frames;
            }
            else if (key == QStringLiteral("white_reference_scanning_length_mm"))
            {
                warnings.push_back(
                    QStringLiteral("Deprecated white_reference_scanning_length_mm — use white_reference_frames instead"));
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
            else if (key == QStringLiteral("sample_window_max_length_mm")
                     || key == QStringLiteral("sample_window_length_mm"))
            {
                if (!hasNumber)
                    warnings.push_back(
                        QStringLiteral("Invalid sample_window_max_length_mm: %1").arg(value));
                else if (numericValue <= 0.0)
                    warnings.push_back(QStringLiteral("sample_window_max_length_mm must be > 0"));
                else
                    config.sampleWindowMaxLengthMm = numericValue;
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
        else if (section == QStringLiteral("preprocessing"))
        {
            if (key == QStringLiteral("illuminant_d"))
            {
                bool ok = false;
                const int illuminantD = value.toInt(&ok);
                if (!ok || (illuminantD != 50 && illuminantD != 55 && illuminantD != 65 && illuminantD != 75))
                    warnings.push_back(QStringLiteral("Invalid preprocessing illuminant_d: %1").arg(value));
                else
                    config.preprocessing.illuminantD = illuminantD;
            }
            else if (key == QStringLiteral("ffc_epsilon"))
            {
                if (!hasNumber || numericValue <= 0.0)
                    warnings.push_back(QStringLiteral("Invalid preprocessing ffc_epsilon: %1").arg(value));
                else
                    config.preprocessing.ffcEpsilon = numericValue;
            }
            else if (key == QStringLiteral("ffc_clamp_min"))
            {
                if (!hasNumber)
                    warnings.push_back(QStringLiteral("Invalid preprocessing ffc_clamp_min: %1").arg(value));
                else
                    config.preprocessing.ffcClampMin = numericValue;
            }
            else if (key == QStringLiteral("ffc_clamp_max"))
            {
                if (!hasNumber)
                    warnings.push_back(QStringLiteral("Invalid preprocessing ffc_clamp_max: %1").arg(value));
                else
                    config.preprocessing.ffcClampMax = numericValue;
            }
            else if (key == QStringLiteral("truncate_nm"))
            {
                if (!hasNumber || numericValue <= 0.0)
                    warnings.push_back(QStringLiteral("Invalid preprocessing truncate_nm: %1").arg(value));
                else
                    config.preprocessing.truncateNm = numericValue;
            }
            else
            {
                warnings.push_back(QStringLiteral("Unknown key in [preprocessing]: %1").arg(key));
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

    if (!sampleStageDraft.rawValues.isEmpty())
        resolveSampleStagePositions(sampleStageDraft, config, warnings);

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
        << "[sample_stage_position]\n"
        << "distance_dual_camera_mm = 190\n"
        << "white_ref_fx10e_mm = 735\n"
        << "white_ref_swir3_mm = white_ref_fx10e_mm - distance_dual_camera_mm\n"
        << "bright_ref_fx10e_mm = 760\n"
        << "bright_ref_swir3_mm = bright_ref_fx10e_mm - distance_dual_camera_mm\n"
        << "sample_scanning_starting_position_fx10e_mm = 840\n"
        << "sample_scanning_starting_position_swir3_mm = sample_scanning_starting_position_fx10e_mm - distance_dual_camera_mm\n"
        << "temp_stop_position_mm = 500\n"
        << "\n"
        << "[scanning_settings]\n"
        << "operation_scanning_speed_mm_per_sec = 80\n"
        << "white_reference_frames = 100\n"
        << "black_reference_frames = 100\n"
        << "sample_window_max_length_mm = 500\n"
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
        << "acceleration_mm_per_sec2 = 30\n"
        << "\n"
        << "[preprocessing]\n"
        << "illuminant_d = 65\n"
        << "ffc_epsilon = 1e-6\n"
        << "ffc_clamp_min = 0.0\n"
        << "ffc_clamp_max = 1.0\n"
        << "truncate_nm = 780.0\n";

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

double recordScanSpeedMmPerSec(const double frameRateHz, const double spatialMmPerPixel)
{
    if (frameRateHz <= 0.0 || spatialMmPerPixel <= 0.0)
        return 0.0;

    return frameRateHz * spatialMmPerPixel;
}

void setHardwareConfig(HardwareConfig config)
{
    g_hardwareConfig = std::move(config);
}
} // namespace hf
