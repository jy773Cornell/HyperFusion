// Profile plot widget implementation.
#include "ui/ProfilePlotWidget.hpp"

#include <QPainter>
#include <QPainterPath>
#include <QPaintEvent>

#include <algorithm>
#include <cmath>

namespace ui
{
namespace
{
constexpr double kDnAxisMax = 4096.0;
constexpr int kTickCount = 9;
constexpr int kLeftMargin = 46;
constexpr int kBottomMargin = 28;
constexpr int kTopMargin = 12;
constexpr int kRightMargin = 10;
} // namespace

ProfilePlotWidget::ProfilePlotWidget(const Mode mode, QWidget *parent)
    : mode_(mode), QWidget(parent)
{
    setMinimumSize(280, 180);
    setStyleSheet(QStringLiteral("background-color: #2a2a2a;"));
}

void ProfilePlotWidget::setProfile(const std::vector<std::uint16_t> &dnValues,
                                 const int xMaxInclusive)
{
    dnValues_ = dnValues;
    xMaxInclusive_ = std::max(0, xMaxInclusive);
    hasProfile_ = !dnValues_.empty() && xMaxInclusive_ > 0;
    disconnectedMessage_.clear();
    update();
}

void ProfilePlotWidget::setWavelengthAxis(const std::vector<double> &wavelengthNmByBand)
{
    wavelengthNmByBand_ = wavelengthNmByBand;
    update();
}

void ProfilePlotWidget::setRgbBandMarkers(const int redBand,
                                          const int greenBand,
                                          const int blueBand)
{
    redBandMarker_ = redBand;
    greenBandMarker_ = greenBand;
    blueBandMarker_ = blueBand;
    update();
}

void ProfilePlotWidget::clearDisplay(const QString &message)
{
    dnValues_.clear();
    hasProfile_ = false;
    xMaxInclusive_ = 0;
    wavelengthNmByBand_.clear();
    disconnectedMessage_ = message;
    update();
}

std::vector<double> ProfilePlotWidget::nineTickValues(const double minValue, const double maxValue)
{
    std::vector<double> ticks(static_cast<std::size_t>(kTickCount));
    if (kTickCount <= 1)
    {
        ticks[0] = minValue;
        return ticks;
    }

    const double span = maxValue - minValue;
    for (int i = 0; i < kTickCount; ++i)
    {
        const double t = static_cast<double>(i) / static_cast<double>(kTickCount - 1);
        ticks[static_cast<std::size_t>(i)] = minValue + span * t;
    }
    return ticks;
}

void ProfilePlotWidget::drawGridAndAxes(QPainter &painter, const QRect &plotRect) const
{
    const auto yTicks = nineTickValues(0.0, kDnAxisMax);

    double xDomainMin = 0.0;
    double xDomainMax = std::max(1.0, static_cast<double>(xMaxInclusive_));
    if (mode_ == Mode::Wavelength && xMaxInclusive_ > 0 && wavelengthNmByBand_.size() > 1)
    {
        const int lastBand = std::min(xMaxInclusive_, static_cast<int>(wavelengthNmByBand_.size()) - 1);
        xDomainMin = wavelengthNmByBand_.front();
        xDomainMax = wavelengthNmByBand_[static_cast<std::size_t>(lastBand)];
        if (xDomainMax <= xDomainMin)
            xDomainMax = xDomainMin + 1.0;
    }

    const auto xTicks = nineTickValues(xDomainMin, xDomainMax);

    QPen gridPen(QColor(0x66, 0x66, 0x66));
    gridPen.setStyle(Qt::DashLine);
    painter.setPen(gridPen);

    for (const double yTick : yTicks)
    {
        const int y = plotRect.bottom()
                      - static_cast<int>((yTick / kDnAxisMax) * static_cast<double>(plotRect.height()));
        painter.drawLine(plotRect.left(), y, plotRect.right(), y);
    }

    const double xSpan = xDomainMax - xDomainMin;
    for (const double xTick : xTicks)
    {
        const double xNorm = (xSpan > 0.0) ? (xTick - xDomainMin) / xSpan : 0.0;
        const int x = plotRect.left() + static_cast<int>(xNorm * static_cast<double>(plotRect.width()));
        painter.drawLine(x, plotRect.top(), x, plotRect.bottom());
    }

    painter.setPen(QColor(0xcc, 0xcc, 0xcc));
    QFont axisFont = painter.font();
    axisFont.setPointSize(8);
    painter.setFont(axisFont);

    for (const double yTick : yTicks)
    {
        const int y = plotRect.bottom()
                      - static_cast<int>((yTick / kDnAxisMax) * static_cast<double>(plotRect.height()));
        painter.drawText(2, y + 4, QString::number(static_cast<int>(yTick)));
    }

    for (const double xTick : xTicks)
    {
        const double xNorm = (xSpan > 0.0) ? (xTick - xDomainMin) / xSpan : 0.0;
        const int x = plotRect.left() + static_cast<int>(xNorm * static_cast<double>(plotRect.width()));
        const QString label = QString::number(static_cast<int>(std::lround(xTick)));
        painter.drawText(x - 14, plotRect.bottom() + 18, label);
    }
}

void ProfilePlotWidget::drawBandMarkers(QPainter &painter, const QRect &plotRect) const
{
    if (mode_ != Mode::Wavelength || xMaxInclusive_ <= 0)
        return;

    double xDomainMin = 0.0;
    double xDomainMax = std::max(1.0, static_cast<double>(xMaxInclusive_));
    if (wavelengthNmByBand_.size() > 1)
    {
        const int lastBand = std::min(xMaxInclusive_, static_cast<int>(wavelengthNmByBand_.size()) - 1);
        xDomainMin = wavelengthNmByBand_.front();
        xDomainMax = wavelengthNmByBand_[static_cast<std::size_t>(lastBand)];
        if (xDomainMax <= xDomainMin)
            xDomainMax = xDomainMin + 1.0;
    }

    const double xSpan = xDomainMax - xDomainMin;
    const auto drawMarker = [&](const int band, const QColor &color) {
        if (band < 0 || band > xMaxInclusive_)
            return;
        double xNorm = (xMaxInclusive_ > 0)
                           ? static_cast<double>(band) / static_cast<double>(xMaxInclusive_)
                           : 0.0;
        if (wavelengthNmByBand_.size() > static_cast<std::size_t>(band) && xSpan > 0.0)
        {
            xNorm = (wavelengthNmByBand_[static_cast<std::size_t>(band)] - xDomainMin) / xSpan;
        }
        const int x = plotRect.left() + static_cast<int>(xNorm * static_cast<double>(plotRect.width()));
        QPen pen(color, 2);
        painter.setPen(pen);
        painter.drawLine(x, plotRect.top(), x, plotRect.bottom());
    };

    drawMarker(redBandMarker_, QColor(0xff, 0x55, 0x55));
    drawMarker(greenBandMarker_, QColor(0x55, 0xff, 0x55));
    drawMarker(blueBandMarker_, QColor(0x3d, 0xa8, 0xff));
}

void ProfilePlotWidget::drawProfile(QPainter &painter, const QRect &plotRect) const
{
    if (!hasProfile_ || dnValues_.empty() || xMaxInclusive_ <= 0)
        return;

    const int count = static_cast<int>(dnValues_.size()) - 1;
    if (count <= 0)
        return;

    double xDomainMin = 0.0;
    double xDomainMax = std::max(1.0, static_cast<double>(xMaxInclusive_));
    if (mode_ == Mode::Wavelength && wavelengthNmByBand_.size() > 1)
    {
        const int lastBand = std::min(xMaxInclusive_, static_cast<int>(wavelengthNmByBand_.size()) - 1);
        xDomainMin = wavelengthNmByBand_.front();
        xDomainMax = wavelengthNmByBand_[static_cast<std::size_t>(lastBand)];
        if (xDomainMax <= xDomainMin)
            xDomainMax = xDomainMin + 1.0;
    }
    const double xSpan = xDomainMax - xDomainMin;

    QPainterPath path;
    for (int i = 0; i <= count; ++i)
    {
        double xNorm = static_cast<double>(i) / static_cast<double>(count);
        if (mode_ == Mode::Wavelength && wavelengthNmByBand_.size() > static_cast<std::size_t>(i)
            && xSpan > 0.0)
        {
            xNorm = (wavelengthNmByBand_[static_cast<std::size_t>(i)] - xDomainMin) / xSpan;
        }
        const int x = plotRect.left() + static_cast<int>(xNorm * static_cast<double>(plotRect.width()));
        const double dn = static_cast<double>(dnValues_[static_cast<std::size_t>(i)]);
        const int y = plotRect.bottom()
                      - static_cast<int>((std::min(dn, kDnAxisMax) / kDnAxisMax)
                                         * static_cast<double>(plotRect.height()));
        const QPoint point(x, y);
        if (i == 0)
            path.moveTo(point);
        else
            path.lineTo(point);
    }

    QPen profilePen(Qt::white, 1);
    painter.setPen(profilePen);
    painter.setBrush(Qt::NoBrush);
    painter.drawPath(path);
}

void ProfilePlotWidget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);

    QPainter painter(this);
    painter.fillRect(rect(), QColor(0x2a, 0x2a, 0x2a));

    if (!hasProfile_)
    {
        painter.setPen(QColor(0x7e, 0xc8, 0xff));
        painter.drawText(rect(), Qt::AlignCenter, disconnectedMessage_);
        return;
    }

    QRect plotRect = rect().adjusted(kLeftMargin, kTopMargin, -kRightMargin, -kBottomMargin);
    if (plotRect.width() < 10 || plotRect.height() < 10)
        return;

    drawGridAndAxes(painter, plotRect);
    drawBandMarkers(painter, plotRect);
    drawProfile(painter, plotRect);
}
} // namespace ui
