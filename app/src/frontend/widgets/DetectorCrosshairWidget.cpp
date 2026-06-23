// Detector crosshair widget implementation.
#include "frontend/widgets/DetectorCrosshairWidget.hpp"

#include <QFont>
#include <QMouseEvent>
#include <QPainter>
#include <QPaintEvent>
#include <QResizeEvent>

#include <algorithm>
#include <cmath>

namespace ui
{
namespace
{
constexpr int kHitTolerancePx = 6;
constexpr int kFpsOverlayMarginPx = 6;
} // namespace

DetectorCrosshairWidget::DetectorCrosshairWidget(QWidget *parent) : QWidget(parent)
{
    setMinimumSize(320, 200);
    setMouseTracking(true);
    setStyleSheet(QStringLiteral("background-color: #111111;"));
}

void DetectorCrosshairWidget::setFrameSize(const int width, const int height)
{
    const bool firstSize = frameWidth_ <= 0 && frameHeight_ <= 0 && width > 0 && height > 0;
    frameWidth_ = std::max(0, width);
    frameHeight_ = std::max(0, height);
    if (frameWidth_ > 0 && frameHeight_ > 0)
    {
        if (firstSize)
        {
            spatialIndex_ = frameWidth_ / 2;
            bandIndex_ = frameHeight_ / 2;
            emitLinesIfChanged();
        }
        else
        {
            spatialIndex_ = std::min(spatialIndex_, frameWidth_ - 1);
            bandIndex_ = std::min(bandIndex_, frameHeight_ - 1);
        }
    }
    update();
}

void DetectorCrosshairWidget::setDetectorImage(const QImage &image)
{
    sourceImage_ = image;
    hasImage_ = !sourceImage_.isNull();
    disconnectedMessage_.clear();
    rebuildScaledPixmap();
    update();
}

void DetectorCrosshairWidget::setAcquisitionFps(const double fps)
{
    const double clamped = std::max(0.0, fps);
    if (std::abs(clamped - acquisitionFps_) < 0.05)
        return;

    acquisitionFps_ = clamped;
    update();
}

void DetectorCrosshairWidget::setDetectorImage(const QPixmap &pixmap)
{
    sourceImage_ = pixmap.toImage();
    pixmap_ = pixmap;
    hasImage_ = !pixmap_.isNull();
    disconnectedMessage_.clear();
    update();
}

void DetectorCrosshairWidget::rebuildScaledPixmap()
{
    if (sourceImage_.isNull() || width() <= 0 || height() <= 0)
    {
        pixmap_ = QPixmap();
        return;
    }

    const QSize target = sourceImage_.size().scaled(size(), Qt::KeepAspectRatio);
    if (target.isEmpty())
    {
        pixmap_ = QPixmap();
        return;
    }

    pixmap_ = QPixmap::fromImage(
        sourceImage_.scaled(target, Qt::KeepAspectRatio, Qt::FastTransformation));
}

void DetectorCrosshairWidget::clearDisplay(const QString &message)
{
    sourceImage_ = QImage();
    pixmap_ = QPixmap();
    hasImage_ = false;
    disconnectedMessage_ = message;
    frameWidth_ = 0;
    frameHeight_ = 0;
    acquisitionFps_ = 0.0;
    update();
}

void DetectorCrosshairWidget::drawStatusOverlay(QPainter &painter) const
{
    if (acquisitionFps_ <= 0.0)
        return;

    QFont font = painter.font();
    font.setBold(true);
    font.setPointSize(10);
    painter.setFont(font);
    painter.setPen(QColor(0xcc, 0xcc, 0xcc));

    const QFontMetrics metrics(font);
    const int lineHeight = metrics.height();
    const QRect widgetRect = rect().adjusted(kFpsOverlayMarginPx, kFpsOverlayMarginPx, 0, 0);

    const QString fpsText = QStringLiteral("%1 fps").arg(acquisitionFps_, 0, 'f', 1);
    painter.drawText(QRect(widgetRect.left(), widgetRect.top(), widgetRect.width(), lineHeight),
                     Qt::AlignTop | Qt::AlignLeft,
                     fpsText);
}

QRect DetectorCrosshairWidget::imageDrawRect() const
{
    if (!hasImage_ || pixmap_.isNull())
        return {};

    const QSize target = pixmap_.size().scaled(size(), Qt::KeepAspectRatio);
    const QPoint topLeft((width() - target.width()) / 2, (height() - target.height()) / 2);
    return QRect(topLeft, target);
}

bool DetectorCrosshairWidget::mapWidgetToImage(const QPoint &widgetPoint, int &outX, int &outY) const
{
    const QRect drawRect = imageDrawRect();
    if (drawRect.isEmpty() || frameWidth_ <= 0 || frameHeight_ <= 0)
        return false;

    if (!drawRect.contains(widgetPoint))
        return false;

    const double nx =
        static_cast<double>(widgetPoint.x() - drawRect.left()) / static_cast<double>(drawRect.width());
    const double ny =
        static_cast<double>(widgetPoint.y() - drawRect.top()) / static_cast<double>(drawRect.height());

    outX = std::clamp(static_cast<int>(nx * static_cast<double>(frameWidth_ - 1) + 0.5), 0,
                      frameWidth_ - 1);
    outY = std::clamp(static_cast<int>(ny * static_cast<double>(frameHeight_ - 1) + 0.5), 0,
                      frameHeight_ - 1);
    return true;
}

void DetectorCrosshairWidget::clampIndices()
{
    if (frameWidth_ > 0)
        spatialIndex_ = std::clamp(spatialIndex_, 0, frameWidth_ - 1);
    if (frameHeight_ > 0)
        bandIndex_ = std::clamp(bandIndex_, 0, frameHeight_ - 1);
}

void DetectorCrosshairWidget::emitLinesIfChanged()
{
    emit linesChanged(spatialIndex_, bandIndex_);
}

void DetectorCrosshairWidget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);

    QPainter painter(this);
    painter.fillRect(rect(), QColor(0x11, 0x11, 0x11));

    if (!hasImage_ || pixmap_.isNull())
    {
        painter.setPen(QColor(0x7e, 0xc8, 0xff));
        painter.drawText(rect(), Qt::AlignCenter, disconnectedMessage_);
        return;
    }

    const QRect drawRect = imageDrawRect();
    painter.drawPixmap(drawRect, pixmap_);
    drawStatusOverlay(painter);

    if (frameWidth_ <= 0 || frameHeight_ <= 0)
        return;

    const double xPos =
        drawRect.left()
        + (static_cast<double>(spatialIndex_) / static_cast<double>(std::max(1, frameWidth_ - 1)))
              * static_cast<double>(drawRect.width());
    const double yPos =
        drawRect.top()
        + (static_cast<double>(bandIndex_) / static_cast<double>(std::max(1, frameHeight_ - 1)))
              * static_cast<double>(drawRect.height());

    QPen crosshairPen(Qt::white);
    crosshairPen.setWidth(2);
    painter.setPen(crosshairPen);
    painter.drawLine(QPointF(xPos, drawRect.top()), QPointF(xPos, drawRect.bottom()));

    painter.setPen(crosshairPen);
    painter.drawLine(QPointF(drawRect.left(), yPos), QPointF(drawRect.right(), yPos));
}

void DetectorCrosshairWidget::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);
    rebuildScaledPixmap();
    update();
}

void DetectorCrosshairWidget::mousePressEvent(QMouseEvent *event)
{
    if (!hasImage_ || event->button() != Qt::LeftButton)
    {
        QWidget::mousePressEvent(event);
        return;
    }

    const QRect drawRect = imageDrawRect();
    if (drawRect.isEmpty())
        return;

    const double xPos =
        drawRect.left()
        + (static_cast<double>(spatialIndex_) / static_cast<double>(std::max(1, frameWidth_ - 1)))
              * static_cast<double>(drawRect.width());
    const double yPos =
        drawRect.top()
        + (static_cast<double>(bandIndex_) / static_cast<double>(std::max(1, frameHeight_ - 1)))
              * static_cast<double>(drawRect.height());

    const int dx = std::abs(event->pos().x() - static_cast<int>(xPos));
    const int dy = std::abs(event->pos().y() - static_cast<int>(yPos));

    if (dx <= kHitTolerancePx && dy <= kHitTolerancePx)
        dragMode_ = (dx < dy) ? DragMode::Vertical : DragMode::Horizontal;
    else if (dx <= kHitTolerancePx)
        dragMode_ = DragMode::Vertical;
    else if (dy <= kHitTolerancePx)
        dragMode_ = DragMode::Horizontal;
    else
    {
        dragMode_ = DragMode::None;
        int imageX = spatialIndex_;
        int imageY = bandIndex_;
        if (mapWidgetToImage(event->pos(), imageX, imageY))
        {
            spatialIndex_ = imageX;
            bandIndex_ = imageY;
            clampIndices();
            update();
            emitLinesIfChanged();
            event->accept();
            return;
        }
    }

    if (dragMode_ != DragMode::None)
        event->accept();
}

void DetectorCrosshairWidget::mouseMoveEvent(QMouseEvent *event)
{
    if (dragMode_ == DragMode::None || !hasImage_)
    {
        QWidget::mouseMoveEvent(event);
        return;
    }

    int imageX = spatialIndex_;
    int imageY = bandIndex_;
    if (!mapWidgetToImage(event->pos(), imageX, imageY))
        return;

    if (dragMode_ == DragMode::Vertical)
        spatialIndex_ = imageX;
    else if (dragMode_ == DragMode::Horizontal)
        bandIndex_ = imageY;

    clampIndices();
    update();
    emitLinesIfChanged();
    event->accept();
}

void DetectorCrosshairWidget::mouseReleaseEvent(QMouseEvent *event)
{
    dragMode_ = DragMode::None;
    QWidget::mouseReleaseEvent(event);
}
} // namespace ui
