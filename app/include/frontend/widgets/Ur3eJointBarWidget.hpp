// Draggable joint target bar with live current-position marker (frontend/ui layer).

#pragma once



#include <QWidget>



namespace ui

{

class Ur3eJointBarWidget : public QWidget

{

    Q_OBJECT



public:

    explicit Ur3eJointBarWidget(QWidget *parent = nullptr);



    void setRangeRadians(const double minimumRad, const double maximumRad);

    void setCurrentRadians(const double valueRad);

    void setValueRadians(const double valueRad);

    void syncTargetFromCurrent();



    [[nodiscard]] double currentRadians() const { return currentRad_; }

    [[nodiscard]] double valueRadians() const { return targetRad_; }

    [[nodiscard]] bool isDragging() const { return dragging_; }



signals:

    void dragStarted();

    void dragFinished();



protected:

    void paintEvent(QPaintEvent *event) override;

    void mousePressEvent(QMouseEvent *event) override;

    void mouseMoveEvent(QMouseEvent *event) override;

    void mouseReleaseEvent(QMouseEvent *event) override;



private:

    [[nodiscard]] QRect barRect() const;

    [[nodiscard]] double fractionForRadians(const double valueRad) const;

    void setTargetFromPosition(const int x);

    void finishDrag();



    double minimumRad_ = -6.283185307179586;

    double maximumRad_ = 6.283185307179586;

    double currentRad_ = 0.0;

    double targetRad_ = 0.0;

    bool dragging_ = false;

};

} // namespace ui

