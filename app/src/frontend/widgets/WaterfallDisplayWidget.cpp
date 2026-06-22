// Waterfall display widget implementation.
#include "frontend/widgets/WaterfallDisplayWidget.hpp"

#include <QPainter>
#include <QPaintEvent>
#include <QResizeEvent>

namespace ui
{
WaterfallDisplayWidget::WaterfallDisplayWidget(QWidget *parent) : QWidget(parent)
{
    setMinimumSize(160, 120);
    setStyleSheet(QStringLiteral("background-color: #1a1a1a;"));
}

void WaterfallDisplayWidget::setImage(QImage image)
{
    image_ = std::move(image);
    placeholder_.clear();
    scaledDirty_ = true;
    update();
}

void WaterfallDisplayWidget::clearDisplay(const QString &message)
{
    image_ = QImage();
    scaledImage_ = QImage();
    placeholder_ = message;
    cachedScaleSize_ = QSize();
    scaledDirty_ = false;
    update();
}

void WaterfallDisplayWidget::ensureScaledImage()
{
    if (!scaledDirty_ && !scaledImage_.isNull() && cachedScaleSize_ == rect().size())
        return;

    scaledDirty_ = false;
    cachedScaleSize_ = rect().size();

    if (image_.isNull() || width() <= 0 || height() <= 0)
    {
        scaledImage_ = QImage();
        return;
    }

    // Ignore aspect ratio: line count grows over time; KeepAspectRatio would shrink width as
    // the buffer gets taller (height-limited scaling), which looks like a narrowing waterfall.
    scaledImage_ = image_.scaled(cachedScaleSize_, Qt::IgnoreAspectRatio, Qt::FastTransformation);
}

void WaterfallDisplayWidget::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);
    scaledDirty_ = true;
    update();
}

void WaterfallDisplayWidget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);

    QPainter painter(this);
    painter.fillRect(rect(), QColor(0x1a, 0x1a, 0x1a));

    if (image_.isNull())
    {
        if (!placeholder_.isEmpty())
        {
            painter.setPen(QColor(0x7e, 0xc8, 0xff));
            painter.drawText(rect(), Qt::AlignCenter, placeholder_);
        }
        return;
    }

    ensureScaledImage();
    if (scaledImage_.isNull())
        return;

    painter.drawImage(rect(), scaledImage_);
}
} // namespace ui
