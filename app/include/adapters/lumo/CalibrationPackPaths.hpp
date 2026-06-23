// Bundled Specim .scp calibration pack discovery (fx10e/ and swir/ under app/calibration/).
#pragma once

#include "adapters/lumo/LumoDeviceTypes.hpp"

#include <QString>

namespace lumo
{
inline constexpr const char *kFx10eCalibrationFileName = "3210441_20211027_calpack.scp";
inline constexpr const char *kSwir3CalibrationFileName = "462111_OLES15_20250109_calpack.scp";
inline constexpr const char *kFx10eCalibrationSubdir = "fx10e";
inline constexpr const char *kSwir3CalibrationSubdir = "swir";

[[nodiscard]] QString calibrationSubdirForSensor(LumoSensorKind sensorKind);
[[nodiscard]] QString defaultCalibrationFileName(LumoSensorKind sensorKind);

/// Absolute path to a bundled .scp next to app.exe or in the source tree (sensor subdir first, then legacy flat).
[[nodiscard]] QString resolveBundledCalibrationPackPath(LumoSensorKind sensorKind);

/// Resolve stored UI path, bare filename, or default bundle for this sensor.
[[nodiscard]] QString resolveCalibrationPackPath(const QString &storedPathOrFileName, LumoSensorKind sensorKind);

[[nodiscard]] QString defaultCalibrationPackPathForProfile(const QString &profileName,
                                                           LumoSensorKind sensorKind);

} // namespace lumo
