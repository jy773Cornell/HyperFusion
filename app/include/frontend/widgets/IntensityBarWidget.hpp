#pragma once

#include <QWidget>

class QMouseEvent;

namespace ui
{
class IntensityBarWidget : public QWidget
{
    Q_OBJECT

public:
    explicit IntensityBarWidget(QWidget *parent = nullptr);

    void setPercent(int percent);
    int percent() const { return percent_; }

signals:
    void percentChanged(int percent);

protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;

private:
    int trackLeft() const;
    int trackRight() const;
    int percentAtPosition(int x) const;
    void setPercentFromInteraction(int percent);

    int percent_ = 0;
    bool dragging_ = false;
};
} // namespace ui
