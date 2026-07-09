#include "backend/camera/processing/IlluminantTables.hpp"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include <algorithm>
#include <cmath>

namespace hf::processing
{
namespace
{
double pchipSlope(const double *x, const double *y, int n, int i)
{
    if (n == 1)
        return 0.0;

    if (i == 0)
    {
        const double h = x[1] - x[0];
        const double delta = (y[1] - y[0]) / std::max(h, 1e-12);
        return delta;
    }

    if (i == n - 1)
    {
        const double h = x[n - 1] - x[n - 2];
        const double delta = (y[n - 1] - y[n - 2]) / std::max(h, 1e-12);
        return delta;
    }

    const double h0 = x[i] - x[i - 1];
    const double h1 = x[i + 1] - x[i];
    const double d0 = (y[i] - y[i - 1]) / std::max(h0, 1e-12);
    const double d1 = (y[i + 1] - y[i]) / std::max(h1, 1e-12);

    if (d0 * d1 <= 0.0)
        return 0.0;

    const double w1 = 2.0 * h1 + h0;
    const double w2 = h1 + 2.0 * h0;
    return (w1 + w2) / (w1 / d0 + w2 / d1);
}

double pchipInterpolate(const std::vector<double> &x,
                        const std::vector<double> &y,
                        const double xq)
{
    if (x.empty() || y.empty())
        return 0.0;

    const int n = static_cast<int>(std::min(x.size(), y.size()));
    if (n == 1)
        return y[0];

    if (xq <= x.front())
        return y.front();
    if (xq >= x.back())
        return y.back();

    const auto upper = std::upper_bound(x.begin(), x.begin() + n, xq);
    const int i1 = static_cast<int>(upper - x.begin());
    const int i0 = i1 - 1;

    const double x0 = x[static_cast<std::size_t>(i0)];
    const double x1 = x[static_cast<std::size_t>(i1)];
    const double y0 = y[static_cast<std::size_t>(i0)];
    const double y1 = y[static_cast<std::size_t>(i1)];
    const double h = std::max(x1 - x0, 1e-12);

    const double m0 = pchipSlope(x.data(), y.data(), n, i0);
    const double m1 = pchipSlope(x.data(), y.data(), n, i1);
    const double t = (xq - x0) / h;
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
    const double h10 = t3 - 2.0 * t2 + t;
    const double h01 = -2.0 * t3 + 3.0 * t2;
    const double h11 = t3 - t2;
    return h00 * y0 + h10 * h * m0 + h01 * y1 + h11 * h * m1;
}

std::vector<double> jsonToDoubles(const QJsonArray &array)
{
    std::vector<double> values;
    values.reserve(static_cast<std::size_t>(array.size()));
    for (const QJsonValue &value : array)
        values.push_back(value.toDouble());
    return values;
}

QString illuminantColumnKey(const int illuminantD)
{
    switch (illuminantD)
    {
    case 50:
        return QStringLiteral("D50");
    case 55:
        return QStringLiteral("D55");
    case 65:
        return QStringLiteral("D65");
    case 75:
        return QStringLiteral("D75");
    default:
        return QStringLiteral("D65");
    }
}
} // namespace

QString defaultIlluminantsJsonPath()
{
    const QStringList candidates = {
        QDir(QCoreApplication::applicationDirPath())
            .filePath(QStringLiteral("backend/camera/processing/reference/D_illuminants.json")),
        QStringLiteral(HF_APP_SOURCE_DIR)
            + QStringLiteral("/src/backend/camera/processing/reference/D_illuminants.json"),
    };

    for (const QString &path : candidates)
    {
        if (QFileInfo::exists(path))
            return path;
    }

    return candidates.front();
}

bool loadIlluminantSpectra(const QString &jsonPath,
                           const int illuminantD,
                           const std::vector<double> &targetWavelengthsNm,
                           IlluminantSpectra &spectraOut,
                           QString *errorMessage)
{
    spectraOut = IlluminantSpectra{};

    QFile file(jsonPath);
    if (!file.open(QIODevice::ReadOnly))
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Could not open illuminant table: %1").arg(jsonPath);
        return false;
    }

    const QJsonDocument document = QJsonDocument::fromJson(file.readAll());
    if (!document.isObject())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Invalid illuminant JSON: %1").arg(jsonPath);
        return false;
    }

    const QJsonObject root = document.object();
    const std::vector<double> cmfWavelengths = jsonToDoubles(root.value(QStringLiteral("wxyz_wavelength_nm")).toArray());
    const std::vector<double> xBar = jsonToDoubles(root.value(QStringLiteral("xbar")).toArray());
    const std::vector<double> yBar = jsonToDoubles(root.value(QStringLiteral("ybar")).toArray());
    const std::vector<double> zBar = jsonToDoubles(root.value(QStringLiteral("zbar")).toArray());
    const std::vector<double> illumWavelengths = jsonToDoubles(root.value(QStringLiteral("D_wavelength_nm")).toArray());
    const std::vector<double> illumValues = jsonToDoubles(root.value(illuminantColumnKey(illuminantD)).toArray());

    if (cmfWavelengths.empty() || illumWavelengths.empty())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Illuminant JSON is missing wavelength tables.");
        return false;
    }

    spectraOut.wavelengthNm = targetWavelengthsNm;
    spectraOut.illuminant.resize(targetWavelengthsNm.size());
    spectraOut.xBar.resize(targetWavelengthsNm.size());
    spectraOut.yBar.resize(targetWavelengthsNm.size());
    spectraOut.zBar.resize(targetWavelengthsNm.size());

    for (std::size_t i = 0; i < targetWavelengthsNm.size(); ++i)
    {
        const double wavelength = targetWavelengthsNm[i];
        spectraOut.illuminant[i] = pchipInterpolate(illumWavelengths, illumValues, wavelength);
        spectraOut.xBar[i] = pchipInterpolate(cmfWavelengths, xBar, wavelength);
        spectraOut.yBar[i] = pchipInterpolate(cmfWavelengths, yBar, wavelength);
        spectraOut.zBar[i] = pchipInterpolate(cmfWavelengths, zBar, wavelength);
    }

    return true;
}

} // namespace hf::processing
