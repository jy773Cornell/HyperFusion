// Mean ± 1σ reference spectrum plot export (intensity DN vs wavelength).
#include "backend/camera/processing/ReferenceSpectrumPlot.hpp"

#include <QDir>
#include <QFileInfo>
#include <QFont>
#include <QImage>
#include <QPainter>
#include <QPen>
#include <QPolygonF>
#include <QSaveFile>

#include <algorithm>
#include <cmath>

namespace hf::processing
{
namespace
{
constexpr int kPlotWidth = 960;
constexpr int kPlotHeight = 540;
constexpr int kMarginLeft = 72;
constexpr int kMarginRight = 24;
constexpr int kMarginTop = 48;
constexpr int kMarginBottom = 56;
constexpr int kAxisTickCount = 10;

QPointF mapDataToPlot(const QRectF &plotRect,
                      const double wavelengthMin,
                      const double wavelengthMax,
                      const double yMin,
                      const double yMax,
                      const double wavelengthNm,
                      const double dn)
{
    const double xNorm =
        (wavelengthNm - wavelengthMin) / std::max(1e-9, wavelengthMax - wavelengthMin);
    const double yNorm = (dn - yMin) / std::max(1e-9, yMax - yMin);
    return QPointF(plotRect.left() + xNorm * plotRect.width(),
                   plotRect.bottom() - yNorm * plotRect.height());
}

QString formatDnAxisTickLabel(const double value)
{
    if (value >= 1000.0)
        return QString::number(static_cast<qint64>(std::llround(value)));
    return QString::number(value, 'f', 0);
}

QString yAxisLabelForMax(const double yAxisMax)
{
    if (yAxisMax > 4096.0)
        return QStringLiteral("Intensity (DN, 0\u201365535, 16-bit)");

    return QStringLiteral("Intensity (DN, 0\u20134096)");
}
} // namespace

bool saveReferenceMeanStdPlotPng(const std::vector<double> &wavelengthsNm,
                                 const std::vector<double> &meanDn,
                                 const std::vector<double> &stdDn,
                                 const QString &title,
                                 const QString &outputPath,
                                 const double yAxisMax,
                                 QString *errorMessage)
{
    if (wavelengthsNm.empty() || meanDn.empty() || stdDn.empty())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Reference spectrum plot has no data.");
        return false;
    }

    const std::size_t bandCount =
        std::min({wavelengthsNm.size(), meanDn.size(), stdDn.size()});
    if (bandCount == 0)
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Reference spectrum plot has no bands.");
        return false;
    }

    const double wavelengthMin = wavelengthsNm.front();
    const double wavelengthMax = wavelengthsNm[bandCount - 1];
    const double yMin = 0.0;
    const double yMax = std::max(1.0, yAxisMax);

    QImage image(kPlotWidth, kPlotHeight, QImage::Format_RGB32);
    image.fill(Qt::white);

    QPainter painter(&image);
    painter.setRenderHint(QPainter::Antialiasing, true);

    QFont titleFont = painter.font();
    titleFont.setBold(true);
    titleFont.setPointSize(11);
    painter.setFont(titleFont);
    painter.setPen(Qt::black);
    painter.drawText(QRect(0, 8, kPlotWidth, kMarginTop), Qt::AlignHCenter | Qt::AlignVCenter, title);

    const QRectF plotRect(kMarginLeft,
                          kMarginTop,
                          kPlotWidth - kMarginLeft - kMarginRight,
                          kPlotHeight - kMarginTop - kMarginBottom);

    QFont tickFont = painter.font();
    tickFont.setPointSize(8);
    painter.setFont(tickFont);

    painter.setPen(QPen(QColor(220, 220, 220)));
    for (int tick = 0; tick < kAxisTickCount; ++tick)
    {
        const double yValue =
            yMin + (static_cast<double>(tick) / static_cast<double>(kAxisTickCount - 1)) * (yMax - yMin);
        const QPointF left =
            mapDataToPlot(plotRect, wavelengthMin, wavelengthMax, yMin, yMax, wavelengthMin, yValue);
        const QPointF right =
            mapDataToPlot(plotRect, wavelengthMin, wavelengthMax, yMin, yMax, wavelengthMax, yValue);
        painter.drawLine(left, right);
    }

    painter.setPen(QPen(Qt::black, 1.2));
    painter.drawRect(plotRect);

    QPolygonF upper;
    QPolygonF lower;
    upper.reserve(static_cast<int>(bandCount));
    lower.reserve(static_cast<int>(bandCount));

    for (std::size_t band = 0; band < bandCount; ++band)
    {
        const double wavelength = wavelengthsNm[band];
        const double mean = meanDn[band];
        const double std = stdDn[band];
        upper.push_back(
            mapDataToPlot(plotRect, wavelengthMin, wavelengthMax, yMin, yMax, wavelength, mean + std));
        lower.push_front(
            mapDataToPlot(plotRect, wavelengthMin, wavelengthMax, yMin, yMax, wavelength, mean - std));
    }

    QPolygonF fill = upper;
    fill.append(lower);
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(120, 170, 230, 90));
    painter.drawPolygon(fill);

    painter.setBrush(Qt::NoBrush);
    painter.setPen(QPen(QColor(30, 90, 180), 2.0));
    QPolygonF meanLine;
    meanLine.reserve(static_cast<int>(bandCount));
    for (std::size_t band = 0; band < bandCount; ++band)
    {
        meanLine.push_back(mapDataToPlot(plotRect,
                                         wavelengthMin,
                                         wavelengthMax,
                                         yMin,
                                         yMax,
                                         wavelengthsNm[band],
                                         meanDn[band]));
    }
    painter.drawPolyline(meanLine);

    painter.setPen(Qt::black);
    for (int tick = 0; tick < kAxisTickCount; ++tick)
    {
        const double yValue =
            yMin + (static_cast<double>(tick) / static_cast<double>(kAxisTickCount - 1)) * (yMax - yMin);
        const QPointF plotPoint =
            mapDataToPlot(plotRect, wavelengthMin, wavelengthMax, yMin, yMax, wavelengthMin, yValue);
        painter.drawLine(QPointF(plotRect.left() - 5.0, plotPoint.y()),
                         QPointF(plotRect.left(), plotPoint.y()));
        painter.drawText(QRect(4,
                               static_cast<int>(std::lround(plotPoint.y())) - 8,
                               kMarginLeft - 12,
                               16),
                         Qt::AlignRight | Qt::AlignVCenter,
                         formatDnAxisTickLabel(yValue));
    }

    for (int tick = 0; tick < kAxisTickCount; ++tick)
    {
        const double wavelength =
            wavelengthMin
            + (static_cast<double>(tick) / static_cast<double>(kAxisTickCount - 1))
                  * (wavelengthMax - wavelengthMin);
        const QPointF plotPoint =
            mapDataToPlot(plotRect, wavelengthMin, wavelengthMax, yMin, yMax, wavelength, yMin);
        painter.drawLine(QPointF(plotPoint.x(), plotRect.bottom()),
                         QPointF(plotPoint.x(), plotRect.bottom() + 5.0));
        painter.drawText(QRect(static_cast<int>(std::lround(plotPoint.x())) - 28,
                               kPlotHeight - kMarginBottom + 6,
                               56,
                               20),
                         Qt::AlignHCenter | Qt::AlignTop,
                         QString::number(static_cast<int>(std::lround(wavelength))));
    }

    painter.drawText(QRect(kMarginLeft, kPlotHeight - kMarginBottom + 28, static_cast<int>(plotRect.width()), 24),
                     Qt::AlignHCenter | Qt::AlignTop,
                     QStringLiteral("Wavelength (nm)"));

    painter.save();
    painter.translate(18, kPlotHeight / 2);
    painter.rotate(-90.0);
    painter.drawText(QRect(-kPlotHeight / 2, 0, kPlotHeight, 20),
                     Qt::AlignHCenter | Qt::AlignTop,
                     yAxisLabelForMax(yMax));
    painter.restore();

    painter.end();

    QDir().mkpath(QFileInfo(outputPath).absolutePath());

    QSaveFile file(outputPath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate))
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Could not write plot PNG: %1").arg(outputPath);
        return false;
    }

    if (!image.save(&file, "PNG"))
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Failed to encode plot PNG: %1").arg(outputPath);
        return false;
    }

    if (!file.commit())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Failed to save plot PNG: %1").arg(outputPath);
        return false;
    }

    return true;
}

} // namespace hf::processing
