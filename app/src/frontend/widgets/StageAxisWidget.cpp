#include "frontend/widgets/StageAxisWidget.hpp"

#include <QPainter>
#include <QPaintEvent>

namespace ui
{
namespace
{
constexpr int kMarginH = 8;
constexpr int kMarginTop = 2;
constexpr int kMarginBottom = 4;
constexpr int kSideGap = 6;
constexpr int kTickExtent = 5;
constexpr int kIndicatorWidth = 8;
constexpr int kIndicatorHeight = 14;
constexpr int kPositionLabelHeight = 14;
constexpr int kMinWidgetHeight =
    kMarginTop + kIndicatorHeight + kPositionLabelHeight + kMarginBottom + 4;
} // namespace

StageAxisWidget::StageAxisWidget(QWidget *parent) : QWidget(parent)
{
    setMinimumHeight(kMinWidgetHeight);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
}

void StageAxisWidget::setTravelRangeMm(const double minimumMm, const double maximumMm)
{
    minimumMm_ = minimumMm;
    maximumMm_ = maximumMm;
    update();
}

void StageAxisWidget::setPositionMm(const double positionMm)
{
    positionMm_ = positionMm;
    update();
}

void StageAxisWidget::paintEvent(QPaintEvent *event)
{
    QWidget::paintEvent(event);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);

    const QFont endFont = painter.font();
    const QFontMetrics endMetrics(endFont);
    const QString minLabel = QString::number(minimumMm_, 'f', 0);
    const QString maxLabel = QString::number(maximumMm_, 'f', 0);
    const int minLabelWidth = endMetrics.horizontalAdvance(minLabel);
    const int maxLabelWidth = endMetrics.horizontalAdvance(maxLabel);
    const int endLabelHeight = endMetrics.height();

    const int trackLeft = kMarginH + minLabelWidth + kSideGap;
    const int trackRight = qMax(trackLeft + 1, width() - kMarginH - maxLabelWidth - kSideGap);
    const int trackWidth = trackRight - trackLeft;
    const int trackY = kMarginTop + kIndicatorHeight / 2 + 2;

    const double span = qMax(maximumMm_ - minimumMm_, 1.0);
    const double clampedPosition = qBound(minimumMm_, positionMm_, maximumMm_);
    const double fraction = (clampedPosition - minimumMm_) / span;
    const int indicatorX = trackLeft + static_cast<int>(fraction * trackWidth);

    painter.setFont(endFont);
    painter.setPen(QColor(80, 80, 80));

    const QRect minLabelRect(kMarginH, trackY - endLabelHeight / 2, minLabelWidth, endLabelHeight);
    const QRect maxLabelRect(width() - kMarginH - maxLabelWidth,
                             trackY - endLabelHeight / 2,
                             maxLabelWidth,
                             endLabelHeight);
    painter.drawText(minLabelRect, Qt::AlignLeft | Qt::AlignVCenter, minLabel);
    painter.drawText(maxLabelRect, Qt::AlignRight | Qt::AlignVCenter, maxLabel);

    QPen axisPen(QColor(160, 160, 160));
    axisPen.setWidth(3);
    painter.setPen(axisPen);
    painter.drawLine(trackLeft, trackY, trackRight, trackY);
    painter.drawLine(trackLeft, trackY - kTickExtent, trackLeft, trackY + kTickExtent);
    painter.drawLine(trackRight, trackY - kTickExtent, trackRight, trackY + kTickExtent);

    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(70, 120, 190));
    painter.drawRoundedRect(indicatorX - kIndicatorWidth / 2,
                            trackY - kIndicatorHeight / 2,
                            kIndicatorWidth,
                            kIndicatorHeight,
                            2,
                            2);

    const int indicatorBottom = trackY + kIndicatorHeight / 2;

    const QString positionText = QStringLiteral("%1 mm").arg(clampedPosition, 0, 'f', 1);
    QFont positionFont = painter.font();
    positionFont.setBold(true);
    painter.setFont(positionFont);
    painter.setPen(QColor(40, 40, 40));

    const QFontMetrics positionMetrics(positionFont);
    const int textWidth = positionMetrics.horizontalAdvance(positionText);
    const int textHeight = positionMetrics.height();
    int textLeft = indicatorX - textWidth / 2;
    textLeft = qBound(kMarginH, textLeft, width() - kMarginH - textWidth);

    const QRect textRect(textLeft, indicatorBottom, textWidth, textHeight);
    painter.drawText(textRect, Qt::AlignHCenter | Qt::AlignTop, positionText);
}
} // namespace ui
