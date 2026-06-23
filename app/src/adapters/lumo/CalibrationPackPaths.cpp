// Resolves bundled Specim .scp paths for FX10e and SWIR3 (calibration/fx10e, calibration/swir).
#include "adapters/lumo/CalibrationPackPaths.hpp"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>

namespace lumo
{
namespace
{
QStringList calibrationSearchRoots()
{
    const QString appDir = QCoreApplication::applicationDirPath();
    QStringList roots = {
        QDir(appDir).filePath(QStringLiteral("calibration")),
        QDir(appDir).filePath(QStringLiteral("../calibration")),
        QDir(appDir).filePath(QStringLiteral("../../calibration")),
        QDir(appDir).filePath(QStringLiteral("../../app/calibration")),
        QDir(appDir).filePath(QStringLiteral("../../../app/calibration")),
    };

#ifdef HF_APP_SOURCE_DIR
    roots.prepend(QDir(QString::fromUtf8(HF_APP_SOURCE_DIR)).filePath(QStringLiteral("calibration")));
#endif

    QStringList cleaned;
    for (const QString &root : roots)
    {
        const QString path = QDir::cleanPath(root);
        if (!cleaned.contains(path))
            cleaned.push_back(path);
    }
    return cleaned;
}

QString firstExistingPath(const QStringList &candidates)
{
    for (const QString &candidate : candidates)
    {
        if (QFileInfo::exists(candidate))
            return QDir::cleanPath(candidate);
    }
    return {};
}

QString resolveByFileName(const QString &fileName, const QString &preferredSubdir)
{
    if (fileName.isEmpty())
        return {};

    const QFileInfo direct(fileName);
    if (direct.isAbsolute() && direct.exists())
        return QDir::cleanPath(fileName);

    QStringList candidates;
    for (const QString &root : calibrationSearchRoots())
    {
        if (!preferredSubdir.isEmpty())
            candidates.push_back(QDir(root).filePath(preferredSubdir + QLatin1Char('/') + fileName));
        candidates.push_back(QDir(root).filePath(fileName));
    }

    return firstExistingPath(candidates);
}
} // namespace

QString calibrationSubdirForSensor(const LumoSensorKind sensorKind)
{
    return sensorKind == LumoSensorKind::Swir3Ni ? QString::fromLatin1(kSwir3CalibrationSubdir)
                                                 : QString::fromLatin1(kFx10eCalibrationSubdir);
}

QString defaultCalibrationFileName(const LumoSensorKind sensorKind)
{
    return sensorKind == LumoSensorKind::Swir3Ni ? QString::fromLatin1(kSwir3CalibrationFileName)
                                                : QString::fromLatin1(kFx10eCalibrationFileName);
}

QString resolveBundledCalibrationPackPath(const LumoSensorKind sensorKind)
{
    return resolveByFileName(defaultCalibrationFileName(sensorKind), calibrationSubdirForSensor(sensorKind));
}

QString resolveCalibrationPackPath(const QString &storedPathOrFileName, const LumoSensorKind sensorKind)
{
    const QString trimmed = storedPathOrFileName.trimmed();
    if (!trimmed.isEmpty())
    {
        const QFileInfo info(trimmed);
        if (info.isAbsolute() && info.exists())
            return QDir::cleanPath(trimmed);

        const QString byName =
            resolveByFileName(info.fileName(), calibrationSubdirForSensor(sensorKind));
        if (!byName.isEmpty())
            return byName;
    }

    return resolveBundledCalibrationPackPath(sensorKind);
}

QString defaultCalibrationPackPathForProfile(const QString &profileName, const LumoSensorKind sensorKind)
{
    if (profileName.contains(QStringLiteral("SWIR"), Qt::CaseInsensitive))
        return resolveBundledCalibrationPackPath(LumoSensorKind::Swir3Ni);
    if (profileName.contains(QStringLiteral("FX10e"), Qt::CaseInsensitive)
        || profileName.contains(QStringLiteral("FX10"), Qt::CaseInsensitive))
        return resolveBundledCalibrationPackPath(LumoSensorKind::Fx10ePleora);

    return resolveBundledCalibrationPackPath(sensorKind);
}

} // namespace lumo
