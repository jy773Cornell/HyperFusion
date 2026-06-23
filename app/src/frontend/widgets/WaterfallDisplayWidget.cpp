// Waterfall display: full pane width, height from aspect, bottom-aligned (newest lines at bottom).
#include "frontend/widgets/WaterfallDisplayWidget.hpp"

#include <QPainter>
#include <QPaintEvent>
#include <QResizeEvent>
#include <QShowEvent>

#include <cmath>

namespace ui
{
WaterfallDisplayWidget::WaterfallDisplayWidget(QWidget *parent) : QWidget(parent)
{
    setMinimumSize(160, 120);
    setStyleSheet(QStringLiteral("background-color: #1a1a1a;"));
}

void WaterfallDisplayWidget::setImage(QImage image)
{
    sharedImage_.reset();
    image_ = std::move(image);
    placeholder_.clear();
    scaledDirty_ = true;
    update();
}

void WaterfallDisplayWidget::setSharedImage(std::shared_ptr<const QImage> image)
{
    image_ = QImage();
    sharedImage_ = std::move(image);
    placeholder_.clear();
    scaledDirty_ = true;
    update();
}

void WaterfallDisplayWidget::clearDisplay(const QString &message)
{
    sharedImage_.reset();
    image_ = QImage();
    scaledImage_ = QImage();
    imageDrawRect_ = QRect();
    placeholder_ = message;
    cachedScaleSize_ = QSize();
    scaledDirty_ = false;
    update();
}

const QImage *WaterfallDisplayWidget::sourceImage() const
{
    if (sharedImage_ != nullptr && !sharedImage_->isNull())
        return sharedImage_.get();
    if (!image_.isNull())
        return &image_;
    return nullptr;
}

void WaterfallDisplayWidget::ensureScaledImage()
{
    if (!scaledDirty_ && !scaledImage_.isNull() && cachedScaleSize_ == rect().size())
        return;

    scaledDirty_ = false;
    cachedScaleSize_ = rect().size();
    imageDrawRect_ = QRect();

    const QImage *source = sourceImage();
    if (source == nullptr || width() <= 0 || height() <= 0)
    {
        scaledImage_ = QImage();
        return;
    }

    const int paneW = cachedScaleSize_.width();
    const int paneH = cachedScaleSize_.height();
    if (source->width() <= 0 || source->height() <= 0)
    {
        scaledImage_ = QImage();
        return;
    }

    const int imageW = source->width();
    const int imageH = source->height();
    const int linesForPane = std::max(
        1,
        static_cast<int>(std::lround(static_cast<double>(paneH) * static_cast<double>(imageW)
                                      / static_cast<double>(paneW))));
    const int linesToShow = std::min(imageH, linesForPane);
    const int startLine = imageH - linesToShow;

    const QImage slice = (startLine == 0 && linesToShow == imageH)
                             ? *source
                             : source->copy(0, startLine, imageW, linesToShow);

    const int fittedW = paneW;
    const int fittedH = std::max(
        1,
        static_cast<int>(std::lround(static_cast<double>(linesToShow) * static_cast<double>(paneW)
                                      / static_cast<double>(imageW))));

    scaledImage_ = slice.scaled(fittedW, fittedH, Qt::IgnoreAspectRatio, Qt::FastTransformation);
    imageDrawRect_ = QRect(0, paneH - fittedH, fittedW, fittedH);
}

void WaterfallDisplayWidget::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);
    scaledDirty_ = true;
    emit paneGeometryChanged();
    update();
}

void WaterfallDisplayWidget::showEvent(QShowEvent *event)
{
    QWidget::showEvent(event);
    scaledDirty_ = true;
    emit paneGeometryChanged();
    update();
}

void WaterfallDisplayWidget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);

    QPainter painter(this);
    painter.fillRect(rect(), QColor(0x1a, 0x1a, 0x1a));

    if (sourceImage() == nullptr)
    {
        if (!placeholder_.isEmpty())
        {
            painter.setPen(QColor(0x7e, 0xc8, 0xff));
            painter.drawText(rect(), Qt::AlignCenter, placeholder_);
        }
        return;
    }

    ensureScaledImage();
    if (scaledImage_.isNull() || imageDrawRect_.isEmpty())
        return;

    painter.drawImage(imageDrawRect_, scaledImage_);
}
} // namespace ui
