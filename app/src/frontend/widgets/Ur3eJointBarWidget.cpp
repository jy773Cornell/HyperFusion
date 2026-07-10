// Draggable joint target bar with live current-position marker (frontend/ui layer).

#include "frontend/widgets/Ur3eJointBarWidget.hpp"



#include <QMouseEvent>

#include <QPainter>

#include <QPaintEvent>



#include <cmath>



namespace ui

{

namespace

{

constexpr int kBarHeight = 14;

constexpr int kBarRadius = 1;

constexpr int kCurrentTickWidth = 2;

} // namespace



Ur3eJointBarWidget::Ur3eJointBarWidget(QWidget *parent) : QWidget(parent)

{

    setFixedHeight(kBarHeight);

    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

    setMouseTracking(true);

}



void Ur3eJointBarWidget::setRangeRadians(const double minimumRad, const double maximumRad)

{

    minimumRad_ = minimumRad;

    maximumRad_ = qMax(minimumRad + 0.001, maximumRad);

    currentRad_ = qBound(minimumRad_, currentRad_, maximumRad_);

    targetRad_ = qBound(minimumRad_, targetRad_, maximumRad_);

    update();

}



void Ur3eJointBarWidget::setCurrentRadians(const double valueRad)

{

    const double clamped = qBound(minimumRad_, valueRad, maximumRad_);

    if (std::abs(clamped - currentRad_) < 0.0005)

        return;



    currentRad_ = clamped;

    update();

}



void Ur3eJointBarWidget::setValueRadians(const double valueRad)

{

    const double clamped = qBound(minimumRad_, valueRad, maximumRad_);

    if (std::abs(clamped - targetRad_) < 0.0005)

        return;



    targetRad_ = clamped;

    update();

    emit targetChanged(targetRad_);

}



void Ur3eJointBarWidget::syncTargetFromCurrent()

{

    if (std::abs(currentRad_ - targetRad_) < 0.0005)

        return;



    targetRad_ = currentRad_;

    update();

    emit targetChanged(targetRad_);

}



double Ur3eJointBarWidget::fractionForRadians(const double valueRad) const

{

    const double span = qMax(maximumRad_ - minimumRad_, 0.001);

    return qBound(0.0, (valueRad - minimumRad_) / span, 1.0);

}



QRect Ur3eJointBarWidget::barRect() const

{

    return QRect(0, 0, width(), kBarHeight);

}



void Ur3eJointBarWidget::setTargetFromPosition(const int x)

{

    const QRect track = barRect();

    if (track.width() <= 1)

        return;



    const double fraction =

        qBound(0.0, static_cast<double>(x - track.left()) / track.width(), 1.0);

    const double span = maximumRad_ - minimumRad_;

    const double newTargetRad = minimumRad_ + fraction * span;
    if (std::abs(newTargetRad - targetRad_) < 0.0005)
        return;

    targetRad_ = newTargetRad;

    update();

    emit targetChanged(targetRad_);

}



void Ur3eJointBarWidget::finishDrag()

{

    if (!dragging_)

        return;



    dragging_ = false;

    emit dragFinished();

}



void Ur3eJointBarWidget::paintEvent(QPaintEvent *event)

{

    QWidget::paintEvent(event);



    QPainter painter(this);

    painter.setRenderHint(QPainter::Antialiasing, true);



    const QRect track = barRect();

    const double targetFraction = fractionForRadians(targetRad_);

    const int fillWidth = qMax(1, static_cast<int>(targetFraction * track.width()));



    painter.setPen(QColor(170, 170, 170));

    painter.setBrush(QColor(255, 255, 255));

    painter.drawRoundedRect(track, kBarRadius, kBarRadius);



    const QRect fillRect(track.left(), track.top(), fillWidth, track.height());

    painter.setPen(Qt::NoPen);

    painter.setBrush(QColor(70, 120, 190));

    painter.drawRoundedRect(fillRect, kBarRadius, kBarRadius);



    const double currentFraction = fractionForRadians(currentRad_);

    const int tickCenterX = track.left() + static_cast<int>(currentFraction * track.width());

    const int tickLeft = qBound(track.left(), tickCenterX - kCurrentTickWidth / 2,

                                track.right() - kCurrentTickWidth + 1);

    const QRect tickRect(tickLeft, track.top(), kCurrentTickWidth, track.height());

    painter.setBrush(QColor(40, 40, 40));

    painter.drawRect(tickRect);



    const double degrees = targetRad_ * 180.0 / M_PI;

    const QString valueText = QStringLiteral("%1\u00B0").arg(static_cast<int>(std::lround(degrees)));



    QFont valueFont = painter.font();

    valueFont.setPixelSize(qMax(9, kBarHeight - 3));

    valueFont.setBold(true);

    painter.setFont(valueFont);

    painter.setPen(QColor(0, 0, 0));



    const QFontMetrics metrics(valueFont);

    const int textWidth = metrics.horizontalAdvance(valueText);

    int textX = fillRect.right() - textWidth - 6;

    if (textX < track.left() + 4)

        textX = track.left() + 4;



    painter.drawText(textX, track.top(), textWidth, track.height(), Qt::AlignVCenter, valueText);

}



void Ur3eJointBarWidget::mousePressEvent(QMouseEvent *event)

{

    if (!isEnabled() || event->button() != Qt::LeftButton)

    {

        QWidget::mousePressEvent(event);

        return;

    }



    dragging_ = true;

    emit dragStarted();

    setTargetFromPosition(static_cast<int>(event->position().x()));

    event->accept();

}



void Ur3eJointBarWidget::mouseMoveEvent(QMouseEvent *event)

{

    if (!dragging_ || !isEnabled())

    {

        QWidget::mouseMoveEvent(event);

        return;

    }



    setTargetFromPosition(static_cast<int>(event->position().x()));

    event->accept();

}



void Ur3eJointBarWidget::mouseReleaseEvent(QMouseEvent *event)

{

    if (dragging_ && event->button() == Qt::LeftButton)

    {

        finishDrag();

        event->accept();

        return;

    }



    QWidget::mouseReleaseEvent(event);

}



} // namespace ui

