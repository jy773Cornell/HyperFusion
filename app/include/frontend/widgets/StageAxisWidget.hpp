#pragma once

#include <QWidget>

namespace ui
{
class StageAxisWidget : public QWidget
{
    Q_OBJECT

public:
    explicit StageAxisWidget(QWidget *parent = nullptr);

    void setTravelRangeMm(double minimumMm, double maximumMm);
    void setPositionMm(double positionMm);

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    double minimumMm_ = 0.0;
    double maximumMm_ = 2000.0;
    double positionMm_ = 0.0;
};
} // namespace ui
