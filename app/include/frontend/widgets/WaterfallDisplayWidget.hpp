// Waterfall RGB stack display: full pane width, height from aspect, anchored to bottom edge.
#pragma once

#include <QImage>
#include <QRect>
#include <QSize>
#include <QWidget>

#include <memory>

namespace ui
{
class WaterfallDisplayWidget final : public QWidget
{
    Q_OBJECT

public:
    explicit WaterfallDisplayWidget(QWidget *parent = nullptr);

    void setImage(QImage image);
    void setSharedImage(std::shared_ptr<const QImage> image);
    void clearDisplay(const QString &message = QString());

signals:
    void paneGeometryChanged();

protected:
    void paintEvent(QPaintEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;
    void showEvent(QShowEvent *event) override;

private:
    void ensureScaledImage();
    [[nodiscard]] const QImage *sourceImage() const;

    QImage image_;
    std::shared_ptr<const QImage> sharedImage_;
    QImage scaledImage_;
    QRect imageDrawRect_;
    QString placeholder_;
    QSize cachedScaleSize_;
    bool scaledDirty_ = true;
};
} // namespace ui
