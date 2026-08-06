// 3D preview of UR3e hemisphere scan over the sample tray (frontend/ui layer).
#pragma once

#include "backend/3dscanning/Ur3eHemisphereScan.hpp"
#include "backend/3dscanning/Ur3eHemisphereScanReachability.hpp"
#include "backend/3dscanning/Ur3eMountTransform.hpp"
#include "backend/3dscanning/Ur3eWorkspaceBoundary.hpp"

#include <QPoint>
#include <QTimer>
#include <QWidget>

#include <vector>

class QMouseEvent;
class QPaintEvent;
class QWheelEvent;

namespace ui
{
class Ur3eHemisphereScanPreviewWidget : public QWidget
{
    Q_OBJECT

public:
    explicit Ur3eHemisphereScanPreviewWidget(QWidget *parent = nullptr);

    void setScanParams(const hf::ur3e::Ur3eHemisphereScanParams &params);
    void setWorkspaceBoundary(const hf::ur3e::Ur3eWorkspaceBoundary &boundary);
    void setSceneMount(const hf::ur3e::Ur3eMountTransform &mount);
    void setScanPlan(const hf::ur3e::Ur3eHemisphereScanPlan &plan);
    void clearScanPlan();
    void beginScanExecution();
    void setActiveScanPoint(int pointIndex);
    void markScanPointCompleted(int pointIndex);
    void markScanPointFailed(int pointIndex);
    void endScanExecution();

protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;

private:
    struct Vec3d
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct Vec3
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct ProjectedPoint
    {
        QPointF screen;
        double depth = 0.0;
    };

    struct PreviewScanPoint
    {
        hf::ur3e::Ur3eHemisphereScanPoint point;
        bool reachabilityKnown = false;
        bool reachable = false;
        /// Reachable but only via previous→pin chain (no home→pin at plan).
        bool homePathOk = true;
        bool executionCompleted = false;
        bool executionFailed = false;
    };

    [[nodiscard]] Vec3 sceneCenter() const;
    [[nodiscard]] Vec3 mapScenePoint(const Vec3 &point) const;
    [[nodiscard]] ProjectedPoint projectPoint(const Vec3 &point, const QRectF &bounds, double scale) const;
    [[nodiscard]] Vec3d rotateView(const Vec3d &point) const;
    void rebuildScanPoints();
    void drawTray(QPainter &painter, const QRectF &bounds, double scale) const;
    void drawWorkspaceBoundary(QPainter &painter, const QRectF &bounds, double scale) const;
    void drawHemisphere(QPainter &painter, const QRectF &bounds, double scale) const;
    void drawScanNormals(QPainter &painter, const QRectF &bounds, double scale) const;
    void drawScanPin(QPainter &painter,
                     const QRectF &bounds,
                     double scale,
                     const PreviewScanPoint &entry,
                     int pointIndex) const;
    void drawLegend(QPainter &painter) const;

    [[nodiscard]] double sceneScale(const QRectF &bounds) const;
    [[nodiscard]] bool hasReachabilityLegend() const;
    void resetCameraView();

    hf::ur3e::Ur3eHemisphereScanParams params_;
    hf::ur3e::Ur3eWorkspaceBoundary workspaceBoundary_;
    hf::ur3e::Ur3eMountTransform sceneMount_;
    std::vector<PreviewScanPoint> scanPoints_;
    double yawRad_ = 0.0;
    double pitchRad_ = 0.0;
    double zoomFactor_ = 1.25;
    bool executionActive_ = false;
    bool executionResultsVisible_ = false;
    int executionActivePointIndex_ = -1;
    int flashPulse_ = 0;
    QTimer *flashTimer_ = nullptr;
    bool dragging_ = false;
    QPoint lastDragPos_;
};
} // namespace ui
