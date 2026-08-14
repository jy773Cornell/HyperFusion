#include "backend/HyperFusionConfig.hpp"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QSaveFile>
#include <QTextStream>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

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
    ok = require(QStringLiteral("sample_3d_scanning_position_mm"),
                 config.sample3dScanningPositionMm,
                 QStringLiteral("sample_3d_scanning_position_mm"))
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

QString stripInlineComment(QString value)
{
    const int hashIndex = value.indexOf(QLatin1Char('#'));
    if (hashIndex >= 0)
        value = value.left(hashIndex);

    return value.trimmed();
}

bool parseKeyValue(const QString &line, QString &keyOut, QString &valueOut)
{
    const int equals = line.indexOf(QLatin1Char('='));
    if (equals <= 0)
        return false;

    keyOut = line.left(equals).trimmed().toLower();
    valueOut = stripInlineComment(line.mid(equals + 1));
    return !keyOut.isEmpty();
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

bool parseSixJointDegList(const QString &text, std::array<double, 6> &jointsOut)
{
    QStringList parts = text.split(QLatin1Char(','), Qt::KeepEmptyParts);
    if (parts.size() != 6)
        return false;

    for (int jointIndex = 0; jointIndex < 6; ++jointIndex)
    {
        double jointDeg = 0.0;
        if (!parseDouble(parts[jointIndex].trimmed(), jointDeg))
            return false;
        jointsOut[static_cast<std::size_t>(jointIndex)] = jointDeg;
    }
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

void assignFx10eSpatialMmPerPixel(hf::HardwareConfig &config,
                                    const double valueMm,
                                    QStringList &warnings)
{
    if (valueMm <= 0.0)
        warnings.push_back(QStringLiteral("fx10e_spatial_mm_per_pixel must be > 0"));
    else
        config.spatialMmPerPixel[0] = valueMm;
}

void assignSwir3SpatialMmPerPixel(hf::HardwareConfig &config,
                                  const double valueMm,
                                  QStringList &warnings)
{
    if (valueMm <= 0.0)
        warnings.push_back(QStringLiteral("swir3_spatial_mm_per_pixel must be > 0"));
    else
        config.spatialMmPerPixel[1] = valueMm;
}

void assignBothSpatialMmPerPixel(hf::HardwareConfig &config,
                                 const double valueMm,
                                 QStringList &warnings)
{
    if (valueMm <= 0.0)
        warnings.push_back(QStringLiteral("spatial_mm_per_pixel must be > 0"));
    else
    {
        config.spatialMmPerPixel[0] = valueMm;
        config.spatialMmPerPixel[1] = valueMm;
    }
}

bool parseSpatialMmPerPixelKey(const QString &key,
                             const QString &value,
                             const bool hasNumber,
                             const double numericValue,
                             hf::HardwareConfig &config,
                             QStringList &warnings)
{
    if (key == QStringLiteral("fx10e_spatial_mm_per_pixel")
        || key == QStringLiteral("fx10e_spatial_distance_per_pixel_mm"))
    {
        if (!hasNumber)
            warnings.push_back(QStringLiteral("Invalid fx10e_spatial_mm_per_pixel: %1").arg(value));
        else
            assignFx10eSpatialMmPerPixel(config, numericValue, warnings);
        return true;
    }

    if (key == QStringLiteral("swir3_spatial_mm_per_pixel")
        || key == QStringLiteral("swir3_spatial_distance_per_pixel_mm"))
    {
        if (!hasNumber)
            warnings.push_back(QStringLiteral("Invalid swir3_spatial_mm_per_pixel: %1").arg(value));
        else
            assignSwir3SpatialMmPerPixel(config, numericValue, warnings);
        return true;
    }

    if (key == QStringLiteral("spatial_mm_per_pixel")
        || key == QStringLiteral("spatial_distance_per_pixel_mm"))
    {
        if (!hasNumber)
            warnings.push_back(QStringLiteral("Invalid spatial_mm_per_pixel: %1").arg(value));
        else
            assignBothSpatialMmPerPixel(config, numericValue, warnings);
        return true;
    }

    return false;
}

bool parseSwir3PreprocessingKey(const QString &key,
                                const QString &value,
                                const bool hasNumber,
                                const double numericValue,
                                hf::HardwareConfig::PreprocessingConfig &preprocess,
                                QStringList &warnings)
{
    if (key == QStringLiteral("swir3_auto_nuc"))
    {
        if (value == QStringLiteral("true") || value == QStringLiteral("1")
            || value == QStringLiteral("yes") || value == QStringLiteral("on"))
            preprocess.swir3AutoNuc = true;
        else if (value == QStringLiteral("false") || value == QStringLiteral("0")
                 || value == QStringLiteral("no") || value == QStringLiteral("off"))
            preprocess.swir3AutoNuc = false;
        else
            warnings.push_back(QStringLiteral("Invalid swir3_auto_nuc (use true/false): %1").arg(value));
        return true;
    }

    if (key == QStringLiteral("swir3_column_profile_correct"))
    {
        if (value == QStringLiteral("true") || value == QStringLiteral("1")
            || value == QStringLiteral("yes") || value == QStringLiteral("on"))
            preprocess.swir3ColumnProfileCorrect = true;
        else if (value == QStringLiteral("false") || value == QStringLiteral("0")
                 || value == QStringLiteral("no") || value == QStringLiteral("off"))
            preprocess.swir3ColumnProfileCorrect = false;
        else
            warnings.push_back(
                QStringLiteral("Invalid swir3_column_profile_correct (use true/false): %1").arg(value));
        return true;
    }

    if (key == QStringLiteral("swir3_column_profile_baseline_radius"))
    {
        if (!hasNumber)
            warnings.push_back(
                QStringLiteral("Invalid swir3_column_profile_baseline_radius: %1").arg(value));
        else
            preprocess.swir3ColumnProfileBaselineRadius = static_cast<int>(numericValue);
        return true;
    }

    if (key == QStringLiteral("swir3_column_profile_valley_gain_min"))
    {
        if (!hasNumber)
            warnings.push_back(
                QStringLiteral("Invalid swir3_column_profile_valley_gain_min: %1").arg(value));
        else
            preprocess.swir3ColumnProfileValleyGainMin = numericValue;
        return true;
    }

    if (key == QStringLiteral("swir3_column_profile_min_band_dn"))
    {
        if (!hasNumber)
            warnings.push_back(
                QStringLiteral("Invalid swir3_column_profile_min_band_dn: %1").arg(value));
        else
            preprocess.swir3ColumnProfileMinBandDn = numericValue;
        return true;
    }

    if (key == QStringLiteral("swir3_column_profile_min_hits"))
    {
        if (!hasNumber)
            warnings.push_back(QStringLiteral("Invalid swir3_column_profile_min_hits: %1").arg(value));
        else
            preprocess.swir3ColumnProfileMinHits = static_cast<int>(numericValue);
        return true;
    }

    if (key == QStringLiteral("swir3_column_profile_min_valley_dn"))
    {
        if (!hasNumber)
            warnings.push_back(
                QStringLiteral("Invalid swir3_column_profile_min_valley_dn: %1").arg(value));
        else
            preprocess.swir3ColumnProfileMinValleyDn = numericValue;
        return true;
    }

    return false;
}

bool parseCameraCalibrationRecordKey(const QString &key,
                                     const QString &value,
                                     const bool hasNumber,
                                     const double numericValue,
                                     hf::HardwareConfig::CameraCalibrationConfig &calib,
                                     QStringList &warnings)
{
    const auto assignPositive = [&](const QString &label, double &target) {
        if (!hasNumber)
            warnings.push_back(QStringLiteral("Invalid %1: %2").arg(label, value));
        else if (numericValue <= 0.0)
            warnings.push_back(QStringLiteral("%1 must be > 0").arg(label));
        else
            target = numericValue;
    };

    if (key == QStringLiteral("fx10e_spatial_fwhm_mm"))
    {
        assignPositive(QStringLiteral("fx10e_spatial_fwhm_mm"), calib.spatialFwhmMm[0]);
        return true;
    }

    if (key == QStringLiteral("swir3_spatial_fwhm_mm"))
    {
        assignPositive(QStringLiteral("swir3_spatial_fwhm_mm"), calib.spatialFwhmMm[1]);
        return true;
    }

    if (key == QStringLiteral("fx10e_spectral_nm_per_pixel"))
    {
        assignPositive(QStringLiteral("fx10e_spectral_nm_per_pixel"), calib.spectralNmPerPixel[0]);
        return true;
    }

    if (key == QStringLiteral("swir3_spectral_nm_per_pixel"))
    {
        assignPositive(QStringLiteral("swir3_spectral_nm_per_pixel"), calib.spectralNmPerPixel[1]);
        return true;
    }

    if (key == QStringLiteral("fx10e_spectral_fwhm_nm"))
    {
        assignPositive(QStringLiteral("fx10e_spectral_fwhm_nm"), calib.spectralFwhmNm[0]);
        return true;
    }

    if (key == QStringLiteral("swir3_spectral_fwhm_nm"))
    {
        assignPositive(QStringLiteral("swir3_spectral_fwhm_nm"), calib.spectralFwhmNm[1]);
        return true;
    }

    return false;
}

bool parseCameraCalibrationSectionKey(const QString &key,
                                    const QString &value,
                                    const bool hasNumber,
                                    const double numericValue,
                                    hf::HardwareConfig &config,
                                    QStringList &warnings)
{
    if (parseSpatialMmPerPixelKey(key, value, hasNumber, numericValue, config, warnings))
        return true;

    if (parseSwir3PreprocessingKey(key, value, hasNumber, numericValue, config.preprocessing, warnings))
        return true;

    if (parseCameraCalibrationRecordKey(key, value, hasNumber, numericValue, config.cameraCalibration, warnings))
        return true;

    return false;
}

bool isCameraCalibrationSection(const QString &section)
{
    return section == QStringLiteral("camera_calibration")
           || section == QStringLiteral("camera_calibraiton")
           || section == QStringLiteral("calibration");
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
                    QStringLiteral("Deprecated record_scanning_speed_mm_per_sec \u2014 record scan speed is "
                                   "frame rate \u00D7 spatial_mm_per_pixel"));
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
                    QStringLiteral("Deprecated white_reference_scanning_length_mm \u2014 use white_reference_frames instead"));
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
            else if (key == QStringLiteral("black_reference_shutter_settle_ms"))
            {
                bool ok = false;
                const int settleMs = value.toInt(&ok);
                if (!ok || settleMs < 0)
                    warnings.push_back(
                        QStringLiteral("Invalid black_reference_shutter_settle_ms: %1").arg(value));
                else
                    config.blackReferenceShutterSettleMs = settleMs;
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
            else if (key == QStringLiteral("acceleration_mm_per_sec2"))
            {
                if (!hasNumber)
                    warnings.push_back(
                        QStringLiteral("Invalid acceleration_mm_per_sec2: %1").arg(value));
                else if (numericValue <= 0.0)
                    warnings.push_back(QStringLiteral("acceleration_mm_per_sec2 must be > 0"));
                else
                    config.stageMotionAccelerationMmPerSec2 = numericValue;
            }
            else if (parseSpatialMmPerPixelKey(key, value, hasNumber, numericValue, config, warnings))
            {
            }
            else if (key == QStringLiteral("fx10e_transmittance_exp"))
            {
                if (!hasNumber)
                    warnings.push_back(QStringLiteral("Invalid fx10e_transmittance_exp: %1").arg(value));
                else if (numericValue <= 0.0)
                    warnings.push_back(QStringLiteral("fx10e_transmittance_exp must be > 0"));
                else
                    config.transmittanceExposureMs[0] = numericValue;
            }
            else if (key == QStringLiteral("swir_transmittance_exp")
                     || key == QStringLiteral("swir3_transmittance_exp"))
            {
                if (!hasNumber)
                    warnings.push_back(QStringLiteral("Invalid swir_transmittance_exp: %1").arg(value));
                else if (numericValue <= 0.0)
                    warnings.push_back(QStringLiteral("swir_transmittance_exp must be > 0"));
                else
                    config.transmittanceExposureMs[1] = numericValue;
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
                    QStringLiteral("Deprecated [white_reference] section \u2014 use [camera_positions] instead"));
            }
            else
            {
                warnings.push_back(QStringLiteral("Unknown key in [white_reference]: %1").arg(key));
            }
        }
        else if (section == QStringLiteral("swir3"))
        {
            warnings.push_back(
                QStringLiteral("[swir3] section is deprecated \u2014 SWIR3 always uses SDK serial autoconnect"));
        }
        else if (isCameraCalibrationSection(section))
        {
            if (!parseCameraCalibrationSectionKey(key, value, hasNumber, numericValue, config, warnings))
                warnings.push_back(QStringLiteral("Unknown key in [%1]: %2").arg(section, key));
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
            else if (key == QStringLiteral("swir_false_color_red_nm_min"))
            {
                if (!hasNumber)
                    warnings.push_back(QStringLiteral("Invalid swir_false_color_red_nm_min: %1").arg(value));
                else
                    config.preprocessing.swirFalseColorRed.minNm = numericValue;
            }
            else if (key == QStringLiteral("swir_false_color_red_nm_max"))
            {
                if (!hasNumber)
                    warnings.push_back(QStringLiteral("Invalid swir_false_color_red_nm_max: %1").arg(value));
                else
                    config.preprocessing.swirFalseColorRed.maxNm = numericValue;
            }
            else if (key == QStringLiteral("swir_false_color_green_nm_min"))
            {
                if (!hasNumber)
                    warnings.push_back(QStringLiteral("Invalid swir_false_color_green_nm_min: %1").arg(value));
                else
                    config.preprocessing.swirFalseColorGreen.minNm = numericValue;
            }
            else if (key == QStringLiteral("swir_false_color_green_nm_max"))
            {
                if (!hasNumber)
                    warnings.push_back(QStringLiteral("Invalid swir_false_color_green_nm_max: %1").arg(value));
                else
                    config.preprocessing.swirFalseColorGreen.maxNm = numericValue;
            }
            else if (key == QStringLiteral("swir_false_color_blue_nm_min"))
            {
                if (!hasNumber)
                    warnings.push_back(QStringLiteral("Invalid swir_false_color_blue_nm_min: %1").arg(value));
                else
                    config.preprocessing.swirFalseColorBlue.minNm = numericValue;
            }
            else if (key == QStringLiteral("swir_false_color_blue_nm_max"))
            {
                if (!hasNumber)
                    warnings.push_back(QStringLiteral("Invalid swir_false_color_blue_nm_max: %1").arg(value));
                else
                    config.preprocessing.swirFalseColorBlue.maxNm = numericValue;
            }
            else if (parseSwir3PreprocessingKey(key, value, hasNumber, numericValue, config.preprocessing, warnings))
            {
            }
            else
            {
                warnings.push_back(QStringLiteral("Unknown key in [preprocessing]: %1").arg(key));
            }
        }
        else if (section == QStringLiteral("segmentation"))
        {
            if (key == QStringLiteral("wsl_distro"))
                config.segmentation.wslDistro = value;
            else if (key == QStringLiteral("wsl_bash_command"))
                config.segmentation.wslBashCommand = value;
            else if (key == QStringLiteral("sam2_repo_linux"))
                config.segmentation.sam2RepoLinux = value;
            else if (key == QStringLiteral("server_port"))
            {
                bool ok = false;
                const int port = value.toInt(&ok);
                if (!ok || port <= 0 || port > 65535)
                    warnings.push_back(QStringLiteral("Invalid segmentation server_port: %1").arg(value));
                else
                    config.segmentation.serverPort = port;
            }
            else if (key == QStringLiteral("box_threshold"))
            {
                if (!hasNumber || numericValue <= 0.0 || numericValue >= 1.0)
                    warnings.push_back(QStringLiteral("Invalid segmentation box_threshold: %1").arg(value));
                else
                    config.segmentation.boxThreshold = numericValue;
            }
            else if (key == QStringLiteral("multimask_output"))
            {
                const QString lower = value.trimmed().toLower();
                config.segmentation.multimaskOutput =
                    lower == QStringLiteral("true") || lower == QStringLiteral("1")
                    || lower == QStringLiteral("yes");
            }
            else if (key == QStringLiteral("warmup_on_start"))
            {
                const QString lower = value.trimmed().toLower();
                config.segmentation.warmupOnStart =
                    lower.isEmpty() || lower == QStringLiteral("true") || lower == QStringLiteral("1")
                    || lower == QStringLiteral("yes");
            }
            else if (key == QStringLiteral("hf_model_id"))
                config.segmentation.hfModelId = value;
            else if (key == QStringLiteral("sam2_config"))
                config.segmentation.sam2Config = value;
            else if (key == QStringLiteral("sam2_checkpoint"))
                config.segmentation.sam2Checkpoint = value;
            else if (key == QStringLiteral("detector_device"))
                config.segmentation.detectorDevice = value;
            else if (key == QStringLiteral("sam2_device"))
                config.segmentation.sam2Device = value;
            else
                warnings.push_back(QStringLiteral("Unknown key in [segmentation]: %1").arg(key));
        }
        else if (section == QStringLiteral("fusion"))
        {
            if (key == QStringLiteral("fusion_margin_mm"))
            {
                if (!hasNumber || numericValue <= 0.0)
                    warnings.push_back(QStringLiteral("Invalid fusion_margin_mm: %1").arg(value));
                else
                    config.fusion.defaultMarginMm = numericValue;
            }
            else if (key == QStringLiteral("fusion_timeout_ms"))
            {
                bool ok = false;
                const int timeoutMs = value.toInt(&ok);
                if (!ok || timeoutMs < 60000)
                    warnings.push_back(QStringLiteral("Invalid fusion_timeout_ms: %1").arg(value));
                else
                    config.fusion.subprocessTimeoutMs = timeoutMs;
            }
            else
                warnings.push_back(QStringLiteral("Unknown key in [fusion]: %1").arg(key));
        }
        else if (section == QStringLiteral("3d scanning") || section == QStringLiteral("ur3e"))
        {
            if (key == QStringLiteral("use_3d_scanning") || key == QStringLiteral("use_ur3e"))
                config.ur3e.use3dScanning =
                    value.trimmed().toLower() == QStringLiteral("true") || value.trimmed() == QStringLiteral("1")
                    || value.trimmed().toLower() == QStringLiteral("yes");
            else if (key == QStringLiteral("wsl_distro"))
                config.ur3e.wslDistro = value;
            else if (key == QStringLiteral("wsl_bash_command"))
                config.ur3e.wslBashCommand = value;
            else if (key == QStringLiteral("ur3e_repo_linux"))
                config.ur3e.ur3eRepoLinux = value;
            else if (key == QStringLiteral("server_port"))
            {
                bool ok = false;
                const int port = value.toInt(&ok);
                if (!ok || port <= 0 || port > 65535)
                    warnings.push_back(QStringLiteral("Invalid ur3e server_port: %1").arg(value));
                else
                    config.ur3e.serverPort = port;
            }
            else if (key == QStringLiteral("robot_ip"))
                config.ur3e.robotIp = value;
            else if (key == QStringLiteral("reverse_ip"))
                config.ur3e.reverseIp = value;
            else if (key == QStringLiteral("dashboard_port"))
            {
                bool ok = false;
                const int port = value.toInt(&ok);
                if (!ok || port <= 0 || port > 65535)
                    warnings.push_back(QStringLiteral("Invalid ur3e dashboard_port: %1").arg(value));
                else
                    config.ur3e.dashboardPort = port;
            }
            else if (key == QStringLiteral("rtde_port"))
            {
                bool ok = false;
                const int port = value.toInt(&ok);
                if (!ok || port <= 0 || port > 65535)
                    warnings.push_back(QStringLiteral("Invalid ur3e rtde_port: %1").arg(value));
                else
                    config.ur3e.rtdePort = port;
            }
            else if (key == QStringLiteral("prestart_driver"))
            {
                const QString lower = value.trimmed().toLower();
                config.ur3e.prestartDriver =
                    lower == QStringLiteral("true") || lower == QStringLiteral("1")
                    || lower == QStringLiteral("yes");
            }
            else if (key == QStringLiteral("connect_timeout_ms"))
            {
                bool ok = false;
                const int timeoutMs = value.toInt(&ok);
                if (!ok || timeoutMs < 30000)
                    warnings.push_back(
                        QStringLiteral("Invalid ur3e connect_timeout_ms (min 30000): %1").arg(value));
                else
                    config.ur3e.connectTimeoutMs = timeoutMs;
            }
            else if (key == QStringLiteral("plan_timeout_ms"))
            {
                bool ok = false;
                const int timeoutMs = value.toInt(&ok);
                if (!ok || timeoutMs < 60000)
                    warnings.push_back(
                        QStringLiteral("Invalid ur3e plan_timeout_ms (min 60000): %1").arg(value));
                else
                    config.ur3e.planTimeoutMs = timeoutMs;
            }
            else if (key == QStringLiteral("ros_distro"))
                config.ur3e.rosDistro = value;
            else if (key == QStringLiteral("ur_type"))
                config.ur3e.urType = value;
            else if (key == QStringLiteral("use_mock_hardware"))
            {
                const QString lower = value.trimmed().toLower();
                config.ur3e.useMockHardware =
                    lower.isEmpty() || lower == QStringLiteral("true") || lower == QStringLiteral("1")
                    || lower == QStringLiteral("yes");
            }
            else if (key == QStringLiteral("max_linear_speed_m_per_s"))
            {
                if (!hasNumber || numericValue <= 0.0)
                    warnings.push_back(QStringLiteral("Invalid ur3e max_linear_speed_m_per_s: %1").arg(value));
                else
                    config.ur3e.maxLinearSpeedMPerS = numericValue;
            }
            else if (key == QStringLiteral("max_linear_accel_m_per_s2"))
            {
                if (!hasNumber || numericValue <= 0.0)
                    warnings.push_back(QStringLiteral("Invalid ur3e max_linear_accel_m_per_s2: %1").arg(value));
                else
                    config.ur3e.maxLinearAccelMPerS2 = numericValue;
            }
            else if (key == QStringLiteral("max_joint_velocity_deg_s"))
            {
                if (!hasNumber || numericValue <= 0.0)
                    warnings.push_back(QStringLiteral("Invalid ur3e max_joint_velocity_deg_s: %1").arg(value));
                else
                    config.ur3e.maxJointVelocityDegS = qMin(numericValue, 190.0);
            }
            else if (key == QStringLiteral("tool_payload_radius_mm")
                     || key == QStringLiteral("tool_payload_pinch_radius_mm"))
            {
                if (!hasNumber || numericValue <= 0.0)
                    warnings.push_back(QStringLiteral("Invalid ur3e tool_payload_radius_mm: %1").arg(value));
                else
                    config.ur3e.toolPayloadRadiusMm = numericValue;
            }
            else if (key == QStringLiteral("tool_payload_shape"))
                config.ur3e.toolPayloadShape = value.trimmed().toLower();
            else if (key == QStringLiteral("tool_payload_mesh"))
                config.ur3e.toolPayloadMesh = value.trimmed();
            else if (key == QStringLiteral("tool_tcp_x_mm"))
            {
                if (!hasNumber)
                    warnings.push_back(QStringLiteral("Invalid ur3e tool_tcp_x_mm: %1").arg(value));
                else
                    config.ur3e.toolTcpXMm = numericValue;
            }
            else if (key == QStringLiteral("tool_tcp_y_mm"))
            {
                if (!hasNumber)
                    warnings.push_back(QStringLiteral("Invalid ur3e tool_tcp_y_mm: %1").arg(value));
                else
                    config.ur3e.toolTcpYMm = numericValue;
            }
            else if (key == QStringLiteral("tool_tcp_z_mm"))
            {
                if (!hasNumber)
                    warnings.push_back(QStringLiteral("Invalid ur3e tool_tcp_z_mm: %1").arg(value));
                else
                    config.ur3e.toolTcpZMm = numericValue;
            }
            else if (key == QStringLiteral("workspace_boundary_enabled"))
            {
                const QString lower = value.trimmed().toLower();
                config.ur3e.workspaceBoundaryEnabled =
                    lower.isEmpty() || lower == QStringLiteral("true") || lower == QStringLiteral("1")
                    || lower == QStringLiteral("yes");
            }
            else if (key == QStringLiteral("workspace_length_mm"))
            {
                if (!hasNumber || numericValue <= 0.0)
                    warnings.push_back(QStringLiteral("Invalid ur3e workspace_length_mm: %1").arg(value));
                else
                    config.ur3e.workspaceLengthMm = numericValue;
            }
            else if (key == QStringLiteral("workspace_width_mm"))
            {
                if (!hasNumber || numericValue <= 0.0)
                    warnings.push_back(QStringLiteral("Invalid ur3e workspace_width_mm: %1").arg(value));
                else
                    config.ur3e.workspaceWidthMm = numericValue;
            }
            else if (key == QStringLiteral("ceiling_mount_height_mm"))
            {
                if (!hasNumber || numericValue <= 0.0)
                    warnings.push_back(
                        QStringLiteral("Invalid ur3e ceiling_mount_height_mm: %1").arg(value));
                else
                    config.ur3e.ceilingMountHeightMm = numericValue;
            }
            else if (key == QStringLiteral("workspace_height_mm"))
            {
                if (!hasNumber || numericValue <= 0.0)
                    warnings.push_back(QStringLiteral("Invalid ur3e workspace_height_mm: %1").arg(value));
                else
                    config.ur3e.workspaceHeightMm = numericValue;
            }
            else if (key == QStringLiteral("workspace_ceiling_clearance_mm"))
            {
                if (!hasNumber || numericValue < 0.0)
                    warnings.push_back(QStringLiteral(
                        "Invalid ur3e workspace_ceiling_clearance_mm: %1").arg(value));
                else
                    config.ur3e.workspaceCeilingClearanceMm = numericValue;
            }
            else if (key == QStringLiteral("mount_roll_deg"))
            {
                if (hasNumber)
                    config.ur3e.mountRollDeg = numericValue;
                else
                    warnings.push_back(QStringLiteral("Invalid ur3e mount_roll_deg: %1").arg(value));
            }
            else if (key == QStringLiteral("mount_pitch_deg"))
            {
                if (hasNumber)
                    config.ur3e.mountPitchDeg = numericValue;
                else
                    warnings.push_back(QStringLiteral("Invalid ur3e mount_pitch_deg: %1").arg(value));
            }
            else if (key == QStringLiteral("mount_yaw_deg"))
            {
                if (hasNumber)
                    config.ur3e.mountYawDeg = numericValue;
                else
                    warnings.push_back(QStringLiteral("Invalid ur3e mount_yaw_deg: %1").arg(value));
            }
            else if (key == QStringLiteral("mount_offset_x_mm"))
            {
                if (hasNumber)
                    config.ur3e.mountOffsetXMm = numericValue;
                else
                    warnings.push_back(QStringLiteral("Invalid ur3e mount_offset_x_mm: %1").arg(value));
            }
            else if (key == QStringLiteral("mount_offset_y_mm"))
            {
                if (hasNumber)
                    config.ur3e.mountOffsetYMm = numericValue;
                else
                    warnings.push_back(QStringLiteral("Invalid ur3e mount_offset_y_mm: %1").arg(value));
            }
            else if (key == QStringLiteral("scan_center_offset_x_mm")
                     || key == QStringLiteral("scan_center_offset_y_mm"))
            {
                warnings.push_back(
                    QStringLiteral("Deprecated %1 — scan center is tray/base XY (0,0); "
                                   "use mount_offset_*_mm to shift the robot vs tray")
                        .arg(key));
            }
            else if (key == QStringLiteral("home_joints_deg"))
            {
                const QStringList parts = value.split(QLatin1Char(','), Qt::SkipEmptyParts);
                if (parts.size() != 6)
                {
                    warnings.push_back(
                        QStringLiteral("Invalid ur3e home_joints_deg (need 6 comma-separated values): %1")
                            .arg(value));
                }
                else
                {
                    bool allOk = true;
                    std::array<double, 6> parsed{};
                    for (int jointIndex = 0; jointIndex < 6; ++jointIndex)
                    {
                        bool ok = false;
                        const double jointDeg = parts[jointIndex].trimmed().toDouble(&ok);
                        if (!ok)
                        {
                            allOk = false;
                            break;
                        }
                        parsed[static_cast<std::size_t>(jointIndex)] = jointDeg;
                    }
                    if (allOk)
                        config.ur3e.homeJointsDeg = parsed;
                    else
                        warnings.push_back(
                            QStringLiteral("Invalid ur3e home_joints_deg (non-numeric value): %1")
                                .arg(value));
                }
            }
            else if (key == QStringLiteral("scan_wrist_sweep_enabled"))
            {
                const QString lower = value.trimmed().toLower();
                config.ur3e.scanWristSweepEnabled =
                    lower.isEmpty() || lower == QStringLiteral("true") || lower == QStringLiteral("1")
                    || lower == QStringLiteral("yes");
            }
            else if (key == QStringLiteral("scan_wrist_sweep_step_deg"))
            {
                if (!hasNumber || numericValue <= 0.0)
                    warnings.push_back(QStringLiteral(
                        "Invalid ur3e scan_wrist_sweep_step_deg: %1").arg(value));
                else
                    config.ur3e.scanWristSweepStepDeg = numericValue;
            }
            else if (key == QStringLiteral("scan_wrist_sweep_steps_each_way"))
            {
                if (!hasNumber || numericValue < 1.0)
                    warnings.push_back(QStringLiteral(
                        "Invalid ur3e scan_wrist_sweep_steps_each_way: %1").arg(value));
                else
                    config.ur3e.scanWristSweepStepsEachWay = static_cast<int>(numericValue);
            }
            else if (key == QStringLiteral("scan_wrist_sweep_wrist_1"))
            {
                const QString lower = value.trimmed().toLower();
                config.ur3e.scanWristSweepWrist1 =
                    lower.isEmpty() || lower == QStringLiteral("true") || lower == QStringLiteral("1")
                    || lower == QStringLiteral("yes");
            }
            else if (key == QStringLiteral("scan_wrist_sweep_wrist_2"))
            {
                const QString lower = value.trimmed().toLower();
                config.ur3e.scanWristSweepWrist2 =
                    lower.isEmpty() || lower == QStringLiteral("true") || lower == QStringLiteral("1")
                    || lower == QStringLiteral("yes");
            }
            else if (key == QStringLiteral("scan_wrist_sweep_wrist_3"))
            {
                const QString lower = value.trimmed().toLower();
                config.ur3e.scanWristSweepWrist3 =
                    lower.isEmpty() || lower == QStringLiteral("true") || lower == QStringLiteral("1")
                    || lower == QStringLiteral("yes");
            }
            else if (key == QStringLiteral("scan_capture_stabilize_ms"))
            {
                if (!hasNumber || numericValue < 0.0)
                    warnings.push_back(QStringLiteral(
                        "Invalid ur3e scan_capture_stabilize_ms: %1").arg(value));
                else
                    config.ur3e.scanCaptureStabilizeMs = static_cast<int>(numericValue);
            }
            else if (key == QStringLiteral("scan_camera_up_world_z"))
            {
                const QString lower = value.trimmed().toLower();
                config.ur3e.scanCameraUpWorldZ =
                    lower.isEmpty() || lower == QStringLiteral("true") || lower == QStringLiteral("1")
                    || lower == QStringLiteral("yes");
            }
            else if (key == QStringLiteral("pin_pose_tolerance_deg"))
            {
                if (!hasNumber || numericValue < 0.0 || numericValue > 45.0)
                    warnings.push_back(QStringLiteral(
                        "Invalid ur3e pin_pose_tolerance_deg: %1 (use 0…45)").arg(value));
                else
                    config.ur3e.pinPoseToleranceDeg = numericValue;
            }
            else if (key == QStringLiteral("pin_tcp_tilt_deg"))
            {
                if (!hasNumber || numericValue < -45.0 || numericValue > 45.0)
                    warnings.push_back(QStringLiteral(
                        "Invalid ur3e pin_tcp_tilt_deg: %1 (use −45…45)").arg(value));
                else
                    config.ur3e.pinTcpTiltDeg = numericValue;
            }
            else if (key == QStringLiteral("semi_ring_search_candidates")
                     || key == QStringLiteral("semi_ring_search_buffer_deg"))
            {
                // Legacy key semi_ring_search_buffer_deg was misnamed; both mean candidate count.
                if (!hasNumber || numericValue < 1.0 || numericValue > 720.0)
                    warnings.push_back(QStringLiteral(
                        "Invalid ur3e semi_ring_search_candidates: %1 (use 1…720)").arg(value));
                else
                    config.ur3e.semiRingSearchCandidates = static_cast<int>(std::lround(numericValue));
            }
            else if (key == QStringLiteral("remember_last_scan_plan"))
            {
                const QString lower = value.trimmed().toLower();
                config.ur3e.rememberLastScanPlan =
                    lower.isEmpty() || lower == QStringLiteral("true") || lower == QStringLiteral("1")
                    || lower == QStringLiteral("yes");
            }
            else if (key == QStringLiteral("bfs_camera_fx"))
            {
                if (!hasNumber || numericValue < 0.0)
                    warnings.push_back(QStringLiteral("Invalid ur3e bfs_camera_fx: %1").arg(value));
                else
                    config.ur3e.bfsCameraFx = numericValue;
            }
            else if (key == QStringLiteral("bfs_camera_fy"))
            {
                if (!hasNumber || numericValue < 0.0)
                    warnings.push_back(QStringLiteral("Invalid ur3e bfs_camera_fy: %1").arg(value));
                else
                    config.ur3e.bfsCameraFy = numericValue;
            }
            else if (key == QStringLiteral("bfs_camera_cx"))
            {
                if (!hasNumber || numericValue < 0.0)
                    warnings.push_back(QStringLiteral("Invalid ur3e bfs_camera_cx: %1").arg(value));
                else
                    config.ur3e.bfsCameraCx = numericValue;
            }
            else if (key == QStringLiteral("bfs_camera_cy"))
            {
                if (!hasNumber || numericValue < 0.0)
                    warnings.push_back(QStringLiteral("Invalid ur3e bfs_camera_cy: %1").arg(value));
                else
                    config.ur3e.bfsCameraCy = numericValue;
            }
            else if (key == QStringLiteral("bfs_camera_distortion"))
            {
                const QString trimmed = value.trimmed();
                if (trimmed.isEmpty())
                {
                    config.ur3e.bfsCameraDistortion.clear();
                }
                else
                {
                    const QStringList parts = trimmed.split(QLatin1Char(','), Qt::SkipEmptyParts);
                    std::vector<double> parsed;
                    parsed.reserve(static_cast<std::size_t>(parts.size()));
                    bool allOk = true;
                    for (const QString &part : parts)
                    {
                        bool ok = false;
                        const double coeff = part.trimmed().toDouble(&ok);
                        if (!ok)
                        {
                            allOk = false;
                            break;
                        }
                        parsed.push_back(coeff);
                    }
                    if (allOk)
                        config.ur3e.bfsCameraDistortion = std::move(parsed);
                    else
                        warnings.push_back(QStringLiteral(
                            "Invalid ur3e bfs_camera_distortion (comma-separated floats): %1")
                                               .arg(value));
                }
            }
            else
                warnings.push_back(QStringLiteral("Unknown key in [3d scanning]: %1").arg(key));
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
        << "[camera_calibration]\n"
        << "fx10e_spatial_mm_per_pixel = 0.205\n"
        << "swir3_spatial_mm_per_pixel = 0.4\n"
        << "fx10e_spatial_fwhm_mm = 0.982\n"
        << "swir3_spatial_fwhm_mm = 1.1\n"
        << "fx10e_spectral_nm_per_pixel = 1.35\n"
        << "swir3_spectral_nm_per_pixel = 5.6\n"
        << "fx10e_spectral_fwhm_nm = 5.5\n"
        << "swir3_spectral_fwhm_nm = 12\n"
        << "# SWIR3: enable Camera.AutoNUC when timing is applied.\n"
        << "swir3_auto_nuc = true\n"
        << "# SWIR3: column destripe from band-median spatial profile (disables SDK Camera.BPR).\n"
        << "swir3_column_profile_correct = true\n"
        << "swir3_column_profile_baseline_radius = 12\n"
        << "swir3_column_profile_valley_gain_min = 0.88\n"
        << "swir3_column_profile_min_band_dn = 64\n"
        << "swir3_column_profile_min_hits = 1\n"
        << "swir3_column_profile_min_valley_dn = 0\n"
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
        << "sample_3d_scanning_position_mm = 1600\n"
        << "\n"
        << "[scanning_settings]\n"
        << "operation_scanning_speed_mm_per_sec = 80\n"
        << "acceleration_mm_per_sec2 = 30\n"
        << "white_reference_frames = 100\n"
        << "black_reference_frames = 100\n"
        << "black_reference_shutter_settle_ms = 1500\n"
        << "sample_window_max_length_mm = 500\n"
        << "# Exposure (ms) for transmittance scan when both reflectance and transmittance are recorded.\n"
        << "fx10e_transmittance_exp = 12\n"
        << "swir_transmittance_exp = 8\n"
        << "\n"
        << "[lighthouse]\n"
        << "idle_intensity_percent = 0\n"
        << "reflectance_intensity_percent = 100\n"
        << "transmittance_intensity_percent = 40\n"
        << "\n"
        << "[stage_motion]\n"
        << "# Trapezoidal accel for lockstep moves and stop deceleration (mm/s\u00B2). Lower = gentler.\n"
        << "acceleration_mm_per_sec2 = 30\n"
        << "\n"
        << "[preprocessing]\n"
        << "illuminant_d = 65\n"
        << "ffc_epsilon = 1e-6\n"
        << "ffc_clamp_min = 0.0\n"
        << "ffc_clamp_max = 1.0\n"
        << "truncate_nm = 780.0\n"
        << "# SWIR false-color PNG (post-capture): mean reflectance per channel inside each nm range.\n"
        << "swir_false_color_red_nm_min = 1550\n"
        << "swir_false_color_red_nm_max = 1700\n"
        << "swir_false_color_green_nm_min = 1100\n"
        << "swir_false_color_green_nm_max = 1300\n"
        << "swir_false_color_blue_nm_min = 950\n"
        << "swir_false_color_blue_nm_max = 1050\n"
        << "\n"
        << "[segmentation]\n"
        << "# GSAM2 sidecar (WSL). sam2_repo_linux empty = auto from resources/gsam2.\n"
        << "wsl_distro = Ubuntu\n"
        << "# Optional extra shell before server start. Leave empty — app uses ./venv/bin/python.\n"
        << "wsl_bash_command = \n"
        << "sam2_repo_linux = \n"
        << "server_port = 8765\n"
        << "box_threshold = 0.30\n"
        << "multimask_output = false\n"
        << "warmup_on_start = true\n"
        << "hf_model_id = IDEA-Research/grounding-dino-base\n"
        << "sam2_config = configs/sam2.1/sam2.1_hiera_l.yaml\n"
        << "sam2_checkpoint = checkpoints/sam2.1_hiera_large.pt\n"
        << "detector_device = cuda\n"
        << "sam2_device = cuda\n"
        << "\n"
        << "[fusion]\n"
        << "# Offline FX10e + SWIR3 fusion. Venv: resources/hf_fusion/.venv (run setup_venv.ps1 once there).\n"
        << "fusion_margin_mm = 5.0\n"
        << "fusion_timeout_ms = 3600000\n"
        << "\n"
        << "[3d scanning]\n"
        << "# Set use_3d_scanning = false to hide 3D Scanning UI (UR3e + BFS) and skip WSL sidecar/driver.\n"
        << "# Legacy section [ur3e] / key use_ur3e still accepted.\n"
        << "use_3d_scanning = true\n"
        << "# UR3e WSL sidecar (ROS 2). See resources/ur3e/README.md.\n"
        << "wsl_distro = Ubuntu\n"
        << "wsl_bash_command = \n"
        << "ur3e_repo_linux = \n"
        << "server_port = 8766\n"
        << "robot_ip = 192.168.0.10\n"
        << "dashboard_port = 29999\n"
        << "rtde_port = 30004\n"
        << "prestart_driver = false\n"
        << "connect_timeout_ms = 120000\n"
        << "# MoveIt hemisphere Plan HTTP timeout (ms). Large grids often need 30–60 min.\n"
        << "plan_timeout_ms = 3600000\n"
        << "ros_distro = jazzy\n"
        << "ur_type = ur3e\n"
        << "use_mock_hardware = true\n"
        << "max_linear_speed_m_per_s = 0.05\n"
        << "max_linear_accel_m_per_s2 = 0.3\n"
        << "max_joint_velocity_deg_s = 60\n"
        << "# Real BFS tool collision mesh on tool0 (urdf/meshes/). Pinch uses flange-flat hemisphere of this radius.\n"
        << "tool_payload_shape = mesh\n"
        << "tool_payload_mesh = ur_bfs_tool_payload.stl\n"
        << "tool_payload_radius_mm = 77\n"
        << "# Optical TCP in tool0 (mm). Fusion CAD (0, 56.035, 20) after mesh pan-180 → (0, -56.035, 20).\n"
        << "tool_tcp_x_mm = 0\n"
        << "tool_tcp_y_mm = -56.035\n"
        << "tool_tcp_z_mm = 20\n"
        << "# Robot mount height (mm): world Z of base_link / ceiling plane. Tray/sample stage stays at Z=0.\n"
        << "ceiling_mount_height_mm = 650\n"
        << "# Workspace collision box (mm): X/Y centered on tray; Z depth extends downward from mount.\n"
        << "workspace_boundary_enabled = true\n"
        << "workspace_length_mm = 600\n"
        << "workspace_width_mm = 600\n"
        << "workspace_height_mm = 650\n"
        << "workspace_ceiling_clearance_mm = 40\n"
        << "# Mount transform (world -> robot base). Roll=180 = ceiling upside-down.\n"
        << "mount_roll_deg = 180\n"
        << "mount_pitch_deg = 0\n"
        << "mount_yaw_deg = 0\n"
        << "mount_offset_x_mm = 0\n"
        << "mount_offset_y_mm = 0\n"
        << "# Scan center = tray projection of optical TCP at home_joints_deg.\n"
        << "# Scan home pose (degrees): pan, lift, elbow, wrist_1, wrist_2, wrist_3.\n"
        << "home_joints_deg = 0,-150,120,0,90,0\n"
        << "scan_capture_stabilize_ms = 500\n"
        << "scan_camera_up_world_z = true\n"
        << "# Half-angle tip (deg) in vertical plane (look-at × camera-up); 0=off. No left/right.\n"
        << "pin_pose_tolerance_deg = 5\n"
        << "# TCP look-at tip angle from nominal point-at-center pose (deg), not a wrist joint.\n"
        << "# + tip toward camera-up; − toward tray. Apex stays exact look-down. Tolerance uses this axis.\n"
        << "pin_tcp_tilt_deg = 0\n"
        << "# Semi Plan: φ candidates per θ ring (evenly over 360°). e.g. 260 ≈ every 1.4°.\n"
        << "semi_ring_search_candidates = 360\n"
        << "remember_last_scan_plan = true\n"
        << "# BFS OpenCV intrinsics for multiview JSON (pixels). fx/fy=0 until calibrated.\n"
        << "bfs_camera_fx = 0\n"
        << "bfs_camera_fy = 0\n"
        << "bfs_camera_cx = 0\n"
        << "bfs_camera_cy = 0\n"
        << "# Optional Brown-Conrady: k1,k2,p1,p2,k3\n"
        << "bfs_camera_distortion =\n";

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
        config.warnings.push_back(QStringLiteral("hyperfusion.cfg not found \u2014 using built-in defaults"));
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

double spatialMmPerPixelForStageCamera(const HardwareConfig &config, const std::size_t stageCameraIndex)
{
    if (stageCameraIndex >= 2)
        return 0.0;

    return config.spatialMmPerPixel[stageCameraIndex];
}

int normalizedSpatialBinning(const int spatialBinning)
{
    if (spatialBinning == 1 || spatialBinning == 2 || spatialBinning == 4 || spatialBinning == 8)
        return spatialBinning;

    return 1;
}

double effectiveSpatialMmPerPixel(const double baseSpatialMmPerPixel, const int spatialBinning)
{
    if (baseSpatialMmPerPixel <= 0.0)
        return 0.0;

    return baseSpatialMmPerPixel * static_cast<double>(normalizedSpatialBinning(spatialBinning));
}

double effectiveSpatialMmPerPixelForStageCamera(const HardwareConfig &config,
                                                const std::size_t stageCameraIndex,
                                                const int spatialBinning)
{
    return effectiveSpatialMmPerPixel(spatialMmPerPixelForStageCamera(config, stageCameraIndex),
                                      spatialBinning);
}

void setHardwareConfig(HardwareConfig config)
{
    g_hardwareConfig = std::move(config);
}
} // namespace hf
