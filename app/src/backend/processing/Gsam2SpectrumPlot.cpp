// Multi-ROI reflectance spectrum plot export (mean ± 1σ vs wavelength).
#include "backend/processing/Gsam2SpectrumPlot.hpp"

#include <QColor>
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
constexpr int kMarginRight = 160;
constexpr int kMarginTop = 48;
constexpr int kMarginBottom = 56;

QPointF mapDataToPlot(const QRectF &plotRect,
                      const double wavelengthMin,
                      const double wavelengthMax,
                      const double yMin,
                      const double yMax,
                      const double wavelengthNm,
                      const double value)
{
    const double xNorm =
        (wavelengthNm - wavelengthMin) / std::max(1e-9, wavelengthMax - wavelengthMin);
    const double yNorm = (value - yMin) / std::max(1e-9, yMax - yMin);
    return QPointF(plotRect.left() + xNorm * plotRect.width(),
                   plotRect.bottom() - yNorm * plotRect.height());
}

QColor seriesColor(const std::size_t index)
{
    static const QColor palette[] = {
        QColor(31, 119, 180),
        QColor(255, 127, 14),
        QColor(44, 160, 44),
        QColor(214, 39, 40),
        QColor(148, 103, 189),
        QColor(140, 86, 75),
        QColor(227, 119, 194),
        QColor(127, 127, 127),
        QColor(188, 189, 34),
        QColor(23, 190, 207),
    };
    return palette[index % (sizeof(palette) / sizeof(palette[0]))];
}
} // namespace

bool saveRoiSpectrumMeanStdPlotPng(const std::vector<double> &wavelengthsNm,
                                   const std::vector<RoiSpectrumSeries> &series,
                                   const QString &title,
                                   const QString &yAxisLabel,
                                   const QString &outputPath,
                                   QString *errorMessage)
{
    if (wavelengthsNm.empty() || series.empty())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("ROI spectrum plot has no data.");
        return false;
    }

    const double wavelengthMin = wavelengthsNm.front();
    const double wavelengthMax = wavelengthsNm.back();

    constexpr double yMin = 0.0;
    constexpr double yMax = 1.0;

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

    constexpr int kYTickCount = 10;
    painter.setPen(QPen(QColor(220, 220, 220)));
    for (int tick = 0; tick < kYTickCount; ++tick)
    {
        const double yValue =
            yMin + (static_cast<double>(tick) / static_cast<double>(kYTickCount - 1)) * (yMax - yMin);
        const QPointF left =
            mapDataToPlot(plotRect, wavelengthMin, wavelengthMax, yMin, yMax, wavelengthMin, yValue);
        const QPointF right =
            mapDataToPlot(plotRect, wavelengthMin, wavelengthMax, yMin, yMax, wavelengthMax, yValue);
        painter.drawLine(left, right);
    }

    painter.setPen(QPen(Qt::black, 1.2));
    painter.drawRect(plotRect);

    painter.setPen(Qt::black);
    for (int tick = 0; tick < kYTickCount; ++tick)
    {
        const double yValue =
            yMin + (static_cast<double>(tick) / static_cast<double>(kYTickCount - 1)) * (yMax - yMin);
        const QPointF plotPoint =
            mapDataToPlot(plotRect, wavelengthMin, wavelengthMax, yMin, yMax, wavelengthMin, yValue);
        painter.drawLine(QPointF(plotRect.left() - 5.0, plotPoint.y()),
                         QPointF(plotRect.left(), plotPoint.y()));
        painter.drawText(QRect(4,
                               static_cast<int>(std::lround(plotPoint.y())) - 8,
                               kMarginLeft - 12,
                               16),
                         Qt::AlignRight | Qt::AlignVCenter,
                         QString::number(yValue, 'f', 1));
    }

    constexpr int kXTickCount = 10;
    for (int tick = 0; tick < kXTickCount; ++tick)
    {
        const double wavelength =
            wavelengthMin
            + (static_cast<double>(tick) / static_cast<double>(kXTickCount - 1))
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

    const std::size_t bandCount = wavelengthsNm.size();
    for (std::size_t seriesIndex = 0; seriesIndex < series.size(); ++seriesIndex)
    {
        const RoiSpectrumSeries &entry = series[seriesIndex];
        const QColor color = seriesColor(seriesIndex);
        const std::size_t count =
            std::min({bandCount, entry.mean.size(), entry.std.size()});
        if (count == 0)
            continue;

        QPolygonF upper;
        QPolygonF lower;
        upper.reserve(static_cast<int>(count));
        lower.reserve(static_cast<int>(count));

        for (std::size_t band = 0; band < count; ++band)
        {
            const double wavelength = wavelengthsNm[band];
            upper.push_back(mapDataToPlot(plotRect,
                                          wavelengthMin,
                                          wavelengthMax,
                                          yMin,
                                          yMax,
                                          wavelength,
                                          entry.mean[band] + entry.std[band]));
            lower.push_front(mapDataToPlot(plotRect,
                                           wavelengthMin,
                                           wavelengthMax,
                                           yMin,
                                           yMax,
                                           wavelength,
                                           entry.mean[band] - entry.std[band]));
        }

        QPolygonF fill = upper;
        fill.append(lower);
        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor(color.red(), color.green(), color.blue(), 50));
        painter.drawPolygon(fill);

        painter.setPen(QPen(color, 1.8));
        painter.setBrush(Qt::NoBrush);
        for (std::size_t band = 1; band < count; ++band)
        {
            const QPointF p0 = mapDataToPlot(plotRect,
                                             wavelengthMin,
                                             wavelengthMax,
                                             yMin,
                                             yMax,
                                             wavelengthsNm[band - 1],
                                             entry.mean[band - 1]);
            const QPointF p1 = mapDataToPlot(plotRect,
                                             wavelengthMin,
                                             wavelengthMax,
                                             yMin,
                                             yMax,
                                             wavelengthsNm[band],
                                             entry.mean[band]);
            painter.drawLine(p0, p1);
        }
    }

    QFont legendFont = painter.font();
    legendFont.setPointSize(9);
    painter.setFont(legendFont);
    int legendY = kMarginTop;
    for (std::size_t seriesIndex = 0; seriesIndex < series.size(); ++seriesIndex)
    {
        const QColor color = seriesColor(seriesIndex);
        const int legendX = kPlotWidth - kMarginRight + 8;
        painter.setPen(QPen(color, 2.0));
        painter.drawLine(legendX, legendY + 8, legendX + 18, legendY + 8);
        painter.setPen(Qt::black);
        painter.drawText(legendX + 24, legendY, kMarginRight - 32, 18, Qt::AlignVCenter, series[seriesIndex].label);
        legendY += 20;
    }

    painter.setPen(Qt::black);
    painter.drawText(QRect(kMarginLeft, kPlotHeight - kMarginBottom + 8, static_cast<int>(plotRect.width()), 24),
                     Qt::AlignHCenter | Qt::AlignTop,
                     QStringLiteral("Wavelength (nm)"));
    painter.save();
    painter.translate(18, kPlotHeight / 2);
    painter.rotate(-90);
    painter.drawText(-80, 0, 160, 20, Qt::AlignCenter, yAxisLabel);
    painter.restore();

    QSaveFile file(outputPath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate))
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Could not write ROI spectrum plot: %1").arg(outputPath);
        return false;
    }

    if (!image.save(&file, "PNG"))
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Could not encode ROI spectrum plot PNG.");
        return false;
    }

    return file.commit();
}

} // namespace hf::processing
