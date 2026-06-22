// Detector preview with draggable vertical (spatial) and horizontal (band) profile cursors.
#pragma once

#include <QImage>
#include <QPixmap>
#include <QString>
#include <QWidget>

#include <optional>

class QPainter;

namespace ui
{
class DetectorCrosshairWidget final : public QWidget
{
    Q_OBJECT

public:
    explicit DetectorCrosshairWidget(QWidget *parent = nullptr);

    void setFrameSize(int width, int height);
    void setDetectorImage(const QImage &image);
    void setDetectorImage(const QPixmap &pixmap);
    void setAcquisitionFps(double fps);
    void setCameraTemperatureCelsius(const std::optional<double> &celsius);
    void clearDisplay(const QString &message = QString());

    int spatialIndex() const { return spatialIndex_; }
    int bandIndex() const { return bandIndex_; }

signals:
    void linesChanged(int spatialIndex, int bandIndex);

protected:
    void paintEvent(QPaintEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;

private:
    enum class DragMode
    {
        None,
        Vertical,
        Horizontal
    };

    QRect imageDrawRect() const;
    bool mapWidgetToImage(const QPoint &widgetPoint, int &outX, int &outY) const;
    void clampIndices();
    void emitLinesIfChanged();
    void drawStatusOverlay(QPainter &painter) const;
    void rebuildScaledPixmap();

    QImage sourceImage_;
    QPixmap pixmap_;
    QString disconnectedMessage_;
    bool hasImage_ = false;
    int frameWidth_ = 0;
    int frameHeight_ = 0;
    int spatialIndex_ = 0;
    int bandIndex_ = 0;

    DragMode dragMode_ = DragMode::None;

    double acquisitionFps_ = 0.0;
    bool hasCameraTemperature_ = false;
    double cameraTemperatureCelsius_ = 0.0;
};
} // namespace ui
