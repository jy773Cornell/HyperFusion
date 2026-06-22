// Waterfall RGB stack display: spatial axis fills width, line history fills height.
#pragma once

#include <QImage>
#include <QSize>
#include <QWidget>

namespace ui
{
class WaterfallDisplayWidget final : public QWidget
{
    Q_OBJECT

public:
    explicit WaterfallDisplayWidget(QWidget *parent = nullptr);

    void setImage(QImage image);
    void clearDisplay(const QString &message = QString());

protected:
    void paintEvent(QPaintEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;
    void showEvent(QShowEvent *event) override;

private:
    void ensureScaledImage();

    QImage image_;
    QImage scaledImage_;
    QString placeholder_;
    QSize cachedScaleSize_;
    bool scaledDirty_ = true;
};
} // namespace ui
