#include "frontend/widgets/IntensityBarWidget.hpp"

#include <cmath>

#include <QMouseEvent>
#include <QPainter>
#include <QPaintEvent>

namespace ui
{
namespace
{
constexpr int kMarginH = 8;
constexpr int kMarginTop = 2;
constexpr int kMarginBottom = 4;
constexpr int kTickExtent = 5;
constexpr int kIndicatorWidth = 8;
constexpr int kIndicatorHeight = 14;
constexpr int kValueLabelHeight = 14;
constexpr int kMinWidgetHeight =
    kMarginTop + kIndicatorHeight + kValueLabelHeight + kMarginBottom + 4;
} // namespace

IntensityBarWidget::IntensityBarWidget(QWidget *parent) : QWidget(parent)
{
    setMinimumHeight(kMinWidgetHeight);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    setMouseTracking(true);
}

void IntensityBarWidget::setPercent(const int percent)
{
    percent_ = qBound(0, percent, 100);
    update();
}

int IntensityBarWidget::trackLeft() const
{
    return kMarginH;
}

int IntensityBarWidget::trackRight() const
{
    return qMax(trackLeft() + 1, width() - kMarginH);
}

int IntensityBarWidget::percentAtPosition(const int x) const
{
    const int left = trackLeft();
    const int right = trackRight();
    const int trackWidth = qMax(1, right - left);
    const double fraction = static_cast<double>(qBound(left, x, right) - left) / static_cast<double>(trackWidth);
    return static_cast<int>(std::lround(fraction * 100.0));
}

void IntensityBarWidget::setPercentFromInteraction(const int percent)
{
    const int clamped = qBound(0, percent, 100);
    if (clamped == percent_)
        return;

    percent_ = clamped;
    update();
    emit percentChanged(percent_);
}

void IntensityBarWidget::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton && isEnabled())
    {
        dragging_ = true;
        setPercentFromInteraction(percentAtPosition(static_cast<int>(event->position().x())));
        event->accept();
        return;
    }

    QWidget::mousePressEvent(event);
}

void IntensityBarWidget::mouseMoveEvent(QMouseEvent *event)
{
    if (dragging_ && isEnabled())
    {
        setPercentFromInteraction(percentAtPosition(static_cast<int>(event->position().x())));
        event->accept();
        return;
    }

    QWidget::mouseMoveEvent(event);
}

void IntensityBarWidget::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
        dragging_ = false;

    QWidget::mouseReleaseEvent(event);
}

void IntensityBarWidget::paintEvent(QPaintEvent *event)
{
    QWidget::paintEvent(event);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);

    const int trackLeftPx = trackLeft();
    const int trackRightPx = trackRight();
    const int trackWidth = trackRightPx - trackLeftPx;
    const int trackY = kMarginTop + kIndicatorHeight / 2 + 2;

    const double fraction = static_cast<double>(percent_) / 100.0;
    const int indicatorX = trackLeftPx + static_cast<int>(fraction * trackWidth);

    QPen axisPen(QColor(160, 160, 160));
    axisPen.setWidth(3);
    painter.setPen(axisPen);
    painter.drawLine(trackLeftPx, trackY, trackRightPx, trackY);
    painter.drawLine(trackLeftPx, trackY - kTickExtent, trackLeftPx, trackY + kTickExtent);
    painter.drawLine(trackRightPx, trackY - kTickExtent, trackRightPx, trackY + kTickExtent);

    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(230, 160, 50));
    painter.drawRoundedRect(indicatorX - kIndicatorWidth / 2,
                            trackY - kIndicatorHeight / 2,
                            kIndicatorWidth,
                            kIndicatorHeight,
                            2,
                            2);

    const int indicatorBottom = trackY + kIndicatorHeight / 2;

    const QString valueText = QStringLiteral("%1 %").arg(percent_);
    QFont valueFont = painter.font();
    valueFont.setBold(true);
    painter.setFont(valueFont);
    painter.setPen(QColor(40, 40, 40));

    const QFontMetrics valueMetrics(valueFont);
    const int textWidth = valueMetrics.horizontalAdvance(valueText);
    const int textHeight = valueMetrics.height();
    int textLeft = indicatorX - textWidth / 2;
    textLeft = qBound(kMarginH, textLeft, width() - kMarginH - textWidth);

    const QRect textRect(textLeft, indicatorBottom, textWidth, textHeight);
    painter.drawText(textRect, Qt::AlignHCenter | Qt::AlignTop, valueText);
}
} // namespace ui
