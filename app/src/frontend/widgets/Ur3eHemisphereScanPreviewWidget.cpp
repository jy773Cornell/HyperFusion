// 3D preview of UR3e hemisphere scan over the sample tray (frontend/ui layer).
#include "frontend/widgets/Ur3eHemisphereScanPreviewWidget.hpp"

#include "backend/HyperFusionConfig.hpp"
#include "backend/multiview/Ur3eHemisphereScan.hpp"
#include "backend/multiview/Ur3eHemisphereScanReachability.hpp"
#include "backend/multiview/Ur3eMountTransform.hpp"
#include "backend/multiview/Ur3eWorkspaceBoundary.hpp"

#include <QWheelEvent>
#include <QFont>
#include <QFontMetrics>
#include <QPainter>
#include <QPainterPath>
#include <QPaintEvent>
#include <QPalette>
#include <QTimer>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace ui
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTrayLengthM = hf::ur3e::kSampleTrayLengthM;
constexpr double kTrayWidthM = hf::ur3e::kSampleTrayWidthM;
constexpr double kTrayHeightM = hf::ur3e::kSampleTrayHeightM;
constexpr double kNormalDisplayLengthM = 0.018;
// Default orbit: front-right, looking slightly down at tray + hemisphere (not inverted).
constexpr double kDefaultYawRad = 40.0 * kPi / 180.0;
constexpr double kDefaultPitchRad = -30.0 * kPi / 180.0;
constexpr double kDefaultZoomFactor = 1.25;
constexpr double kSceneFitPadding = 2.2;

hf::ur3e::Ur3eHemisphereScanPoint offsetScanPoint(const hf::ur3e::Ur3eHemisphereScanPoint &point)
{
    hf::ur3e::Ur3eHemisphereScanPoint shifted = point;
    shifted.zM += hf::ur3e::kSampleTrayHeightM;
    return shifted;
}
} // namespace

Ur3eHemisphereScanPreviewWidget::Ur3eHemisphereScanPreviewWidget(QWidget *parent) : QWidget(parent)
{
    setMinimumHeight(200);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setAutoFillBackground(true);
    setMouseTracking(true);
    setCursor(Qt::OpenHandCursor);

    QPalette palette = this->palette();
    palette.setColor(QPalette::Window, QColor(26, 26, 26));
    setPalette(palette);

    yawRad_ = kDefaultYawRad;
    pitchRad_ = kDefaultPitchRad;
    zoomFactor_ = kDefaultZoomFactor;
    params_ = hf::ur3e::Ur3eHemisphereScanParams{};
    workspaceBoundary_ = hf::ur3e::Ur3eWorkspaceBoundary{};
    sceneMount_ = hf::ur3e::Ur3eMountTransform::sceneAlignFromConfig(hf::hardwareConfig().ur3e);
    rebuildScanPoints();

    flashTimer_ = new QTimer(this);
    flashTimer_->setInterval(80);
    connect(flashTimer_, &QTimer::timeout, this, [this]() {
        if (!executionActive_ || executionActivePointIndex_ < 0)
            return;
        flashPulse_ = (flashPulse_ + 1) % 16;
        update();
    });
}

void Ur3eHemisphereScanPreviewWidget::resetCameraView()
{
    yawRad_ = kDefaultYawRad;
    pitchRad_ = kDefaultPitchRad;
    zoomFactor_ = kDefaultZoomFactor;
}

Ur3eHemisphereScanPreviewWidget::Vec3 Ur3eHemisphereScanPreviewWidget::sceneCenter() const
{
    constexpr double minZ = 0.0;
    double maxZ = kTrayHeightM + params_.sphereRadiusM;
    if (workspaceBoundary_.enabled)
        maxZ = std::max(maxZ, workspaceBoundary_.topZM());

    double centerXM = 0.0;
    double centerYM = 0.0;
    hf::ur3e::scanCenterOffsetM(centerXM, centerYM);
    const Vec3 nominalCenter{centerXM, centerYM, (minZ + maxZ) * 0.5};
    return mapScenePoint(nominalCenter);
}

Ur3eHemisphereScanPreviewWidget::Vec3d
Ur3eHemisphereScanPreviewWidget::rotateView(const Vec3d &point) const
{
    const double yawX = point.x * std::cos(yawRad_) - point.y * std::sin(yawRad_);
    const double yawY = point.x * std::sin(yawRad_) + point.y * std::cos(yawRad_);
    const double pitchY = yawY * std::cos(pitchRad_) - point.z * std::sin(pitchRad_);
    const double pitchZ = yawY * std::sin(pitchRad_) + point.z * std::cos(pitchRad_);
    return {yawX, pitchY, pitchZ};
}

void Ur3eHemisphereScanPreviewWidget::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
        dragging_ = true;
        lastDragPos_ = event->pos();
        setCursor(Qt::ClosedHandCursor);
        event->accept();
        return;
    }
    QWidget::mousePressEvent(event);
}

void Ur3eHemisphereScanPreviewWidget::mouseMoveEvent(QMouseEvent *event)
{
    if (dragging_)
    {
        const QPoint delta = event->pos() - lastDragPos_;
        lastDragPos_ = event->pos();
        yawRad_ += static_cast<double>(delta.x()) * 0.012;
        pitchRad_ += static_cast<double>(delta.y()) * 0.012;
        pitchRad_ = std::clamp(pitchRad_, -1.45, 1.45);
        update();
        event->accept();
        return;
    }
    QWidget::mouseMoveEvent(event);
}

void Ur3eHemisphereScanPreviewWidget::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton && dragging_)
    {
        dragging_ = false;
        setCursor(Qt::OpenHandCursor);
        event->accept();
        return;
    }
    QWidget::mouseReleaseEvent(event);
}

void Ur3eHemisphereScanPreviewWidget::wheelEvent(QWheelEvent *event)
{
    const QPoint angleDelta = event->angleDelta();
    if (angleDelta.y() == 0)
    {
        QWidget::wheelEvent(event);
        return;
    }

    constexpr double kZoomStep = 1.12;
    if (angleDelta.y() > 0)
        zoomFactor_ *= kZoomStep;
    else
        zoomFactor_ /= kZoomStep;

    zoomFactor_ = std::clamp(zoomFactor_, 0.2, 8.0);
    update();
    event->accept();
}

void Ur3eHemisphereScanPreviewWidget::setWorkspaceBoundary(
    const hf::ur3e::Ur3eWorkspaceBoundary &boundary)
{
    workspaceBoundary_ = boundary;
    workspaceBoundary_.normalize();
    resetCameraView();
    update();
}

void Ur3eHemisphereScanPreviewWidget::setSceneMount(const hf::ur3e::Ur3eMountTransform &mount)
{
    sceneMount_ = mount;
    update();
}

void Ur3eHemisphereScanPreviewWidget::setScanParams(const hf::ur3e::Ur3eHemisphereScanParams &params)
{
    params_ = params;
    hf::ur3e::normalizeHemisphereScanParams(params_);
    rebuildScanPoints();
    update();
}

void Ur3eHemisphereScanPreviewWidget::setScanPlan(const hf::ur3e::Ur3eHemisphereScanPlan &plan)
{
    semiFixedPreviewActive_ = false;
    semiFixedRings_.clear();
    executionActive_ = false;
    executionResultsVisible_ = false;
    executionActivePointIndex_ = -1;
    flashPulse_ = 0;
    if (flashTimer_ != nullptr)
        flashTimer_->stop();
    scanPoints_.clear();
    scanPoints_.reserve(plan.points.size());
    for (const hf::ur3e::Ur3ePlannedScanPoint &planned : plan.points)
    {
        PreviewScanPoint preview;
        preview.point = offsetScanPoint(planned.gridPoint);
        preview.reachabilityKnown = true;
        preview.reachable = planned.reachable;
        preview.homePathOk = planned.homePathOk;
        preview.executionCompleted = false;
        preview.executionFailed = false;
        scanPoints_.push_back(preview);
    }
    update();
}

void Ur3eHemisphereScanPreviewWidget::beginScanExecution()
{
    executionActive_ = true;
    executionResultsVisible_ = false;
    executionActivePointIndex_ = -1;
    flashPulse_ = 0;
    for (PreviewScanPoint &entry : scanPoints_)
    {
        entry.executionCompleted = false;
        entry.executionFailed = false;
    }
    for (PreviewSemiFixedRing &entry : semiFixedRings_)
    {
        entry.executionCompleted = false;
        entry.executionFailed = false;
    }
    update();
}

void Ur3eHemisphereScanPreviewWidget::setActiveScanPoint(const int pointIndex)
{
    if (!executionActive_)
        return;

    // Execute already sends preview indices (route.rings 0..N-1, apex in-list or N).
    const int previewIndex = pointIndex;

    const int maxIndex = semiFixedPreviewActive_
                             ? static_cast<int>(semiFixedRings_.size())
                             : static_cast<int>(scanPoints_.size());
    if (previewIndex < 0 || previewIndex >= maxIndex)
    {
        executionActivePointIndex_ = -1;
        if (flashTimer_ != nullptr)
            flashTimer_->stop();
        update();
        return;
    }

    executionActivePointIndex_ = previewIndex;
    flashPulse_ = 0;
    if (flashTimer_ != nullptr && !flashTimer_->isActive())
        flashTimer_->start();
    update();
}

void Ur3eHemisphereScanPreviewWidget::markScanPointCompleted(const int pointIndex)
{
    if (semiFixedPreviewActive_)
    {
        const int previewIndex = pointIndex;
        if (previewIndex < 0 || previewIndex >= static_cast<int>(semiFixedRings_.size()))
            return;
        PreviewSemiFixedRing &entry = semiFixedRings_[static_cast<std::size_t>(previewIndex)];
        entry.executionCompleted = true;
        entry.executionFailed = false;
        if (executionActivePointIndex_ == previewIndex)
        {
            executionActivePointIndex_ = -1;
            if (flashTimer_ != nullptr)
                flashTimer_->stop();
        }
    }
    else
    {
        if (pointIndex < 0 || pointIndex >= static_cast<int>(scanPoints_.size()))
            return;
        PreviewScanPoint &entry = scanPoints_[static_cast<std::size_t>(pointIndex)];
        entry.executionCompleted = true;
        entry.executionFailed = false;
        if (executionActivePointIndex_ == pointIndex)
        {
            executionActivePointIndex_ = -1;
            if (flashTimer_ != nullptr)
                flashTimer_->stop();
        }
    }
    update();
}

void Ur3eHemisphereScanPreviewWidget::markScanPointFailed(const int pointIndex)
{
    if (semiFixedPreviewActive_)
    {
        const int previewIndex = pointIndex;
        if (previewIndex < 0 || previewIndex >= static_cast<int>(semiFixedRings_.size()))
            return;
        PreviewSemiFixedRing &entry = semiFixedRings_[static_cast<std::size_t>(previewIndex)];
        entry.executionFailed = true;
        entry.executionCompleted = false;
        if (executionActivePointIndex_ == previewIndex)
        {
            executionActivePointIndex_ = -1;
            if (flashTimer_ != nullptr)
                flashTimer_->stop();
        }
    }
    else
    {
        if (pointIndex < 0 || pointIndex >= static_cast<int>(scanPoints_.size()))
            return;
        PreviewScanPoint &entry = scanPoints_[static_cast<std::size_t>(pointIndex)];
        entry.executionFailed = true;
        entry.executionCompleted = false;
        if (executionActivePointIndex_ == pointIndex)
        {
            executionActivePointIndex_ = -1;
            if (flashTimer_ != nullptr)
                flashTimer_->stop();
        }
    }
    update();
}

void Ur3eHemisphereScanPreviewWidget::endScanExecution()
{
    executionActive_ = false;
    executionResultsVisible_ = true;
    executionActivePointIndex_ = -1;
    flashPulse_ = 0;
    if (flashTimer_ != nullptr)
        flashTimer_->stop();
    update();
}

int Ur3eHemisphereScanPreviewWidget::plannedExecutionCount() const
{
    if (semiFixedPreviewActive_)
    {
        int count = 0;
        bool hasTop = false;
        for (const PreviewSemiFixedRing &entry : semiFixedRings_)
        {
            if (entry.ring.isTopPose)
            {
                hasTop = true;
                continue;
            }
            if (entry.ring.reachabilityKnown && !entry.ring.reachable)
                continue;
            ++count;
        }
        if (hasTop)
            ++count;
        return count;
    }

    int count = 0;
    for (const PreviewScanPoint &entry : scanPoints_)
    {
        if (entry.reachabilityKnown && !entry.reachable)
            continue;
        ++count;
    }
    return count;
}

int Ur3eHemisphereScanPreviewWidget::completedExecutionCount() const
{
    int count = 0;
    if (semiFixedPreviewActive_)
    {
        for (const PreviewSemiFixedRing &entry : semiFixedRings_)
        {
            if (entry.ring.reachabilityKnown && !entry.ring.reachable && !entry.ring.isTopPose)
                continue;
            if (entry.executionCompleted)
                ++count;
        }
        return count;
    }
    for (const PreviewScanPoint &entry : scanPoints_)
    {
        if (entry.reachabilityKnown && !entry.reachable)
            continue;
        if (entry.executionCompleted)
            ++count;
    }
    return count;
}

int Ur3eHemisphereScanPreviewWidget::failedExecutionCount() const
{
    int count = 0;
    if (semiFixedPreviewActive_)
    {
        for (const PreviewSemiFixedRing &entry : semiFixedRings_)
        {
            if (entry.ring.reachabilityKnown && !entry.ring.reachable && !entry.ring.isTopPose)
                continue;
            if (entry.executionFailed)
                ++count;
        }
        return count;
    }
    for (const PreviewScanPoint &entry : scanPoints_)
    {
        if (entry.reachabilityKnown && !entry.reachable)
            continue;
        if (entry.executionFailed)
            ++count;
    }
    return count;
}

void Ur3eHemisphereScanPreviewWidget::clearScanPlan()
{
    executionActive_ = false;
    executionResultsVisible_ = false;
    executionActivePointIndex_ = -1;
    flashPulse_ = 0;
    if (flashTimer_ != nullptr)
        flashTimer_->stop();
    rebuildScanPoints();
    update();
}

void Ur3eHemisphereScanPreviewWidget::setSemiFixedPreviewRings(
    const QVector<hf::ur3e::Ur3eSemiFixedPreviewRing> &rings)
{
    semiFixedPreviewActive_ = true;
    semiFixedRings_.clear();
    semiFixedRings_.reserve(static_cast<std::size_t>(rings.size()));
    for (const hf::ur3e::Ur3eSemiFixedPreviewRing &ring : rings)
    {
        PreviewSemiFixedRing entry;
        entry.ring = ring;
        semiFixedRings_.push_back(entry);
    }
    executionActive_ = false;
    executionResultsVisible_ = false;
    executionActivePointIndex_ = -1;
    update();
}

void Ur3eHemisphereScanPreviewWidget::clearSemiFixedPreviewRings()
{
    semiFixedPreviewActive_ = false;
    semiFixedRings_.clear();
    update();
}

void Ur3eHemisphereScanPreviewWidget::rebuildScanPoints()
{
    const std::vector<hf::ur3e::Ur3eHemisphereScanPoint> generated =
        hf::ur3e::generateHemisphereScanPoints(params_);
    scanPoints_.clear();
    scanPoints_.reserve(generated.size());
    for (const hf::ur3e::Ur3eHemisphereScanPoint &point : generated)
    {
        PreviewScanPoint preview;
        preview.point = offsetScanPoint(point);
        scanPoints_.push_back(preview);
    }
}

Ur3eHemisphereScanPreviewWidget::Vec3 Ur3eHemisphereScanPreviewWidget::mapScenePoint(
    const Vec3 &point) const
{
    double xM = point.x;
    double yM = point.y;
    double zM = point.z;
    sceneMount_.transformPoint(xM, yM, zM);
    return Vec3{xM, yM, zM};
}

Ur3eHemisphereScanPreviewWidget::ProjectedPoint
Ur3eHemisphereScanPreviewWidget::projectPoint(const Vec3 &point,
                                              const QRectF &bounds,
                                              const double scale) const
{
    const Vec3 mapped = mapScenePoint(point);
    const Vec3 center = sceneCenter();
    const Vec3d centered{mapped.x - center.x, mapped.y - center.y, mapped.z - center.z};
    const Vec3d rotated = rotateView(centered);
    ProjectedPoint projected;
    projected.depth = rotated.z;
    projected.screen =
        QPointF(bounds.center().x() + rotated.x * scale, bounds.center().y() - rotated.y * scale);
    return projected;
}

double Ur3eHemisphereScanPreviewWidget::sceneScale(const QRectF &bounds) const
{
    double extentX = kTrayLengthM * 0.5;
    double extentY = kTrayWidthM * 0.5;
    if (workspaceBoundary_.enabled)
    {
        extentX = std::max(extentX, workspaceBoundary_.halfLengthM());
        extentY = std::max(extentY, workspaceBoundary_.halfWidthM());
    }

    constexpr double minZ = 0.0;
    double maxZ = kTrayHeightM + params_.sphereRadiusM;
    if (workspaceBoundary_.enabled)
        maxZ = std::max(maxZ, workspaceBoundary_.topZM());

    const std::array<Vec3, 8> corners = {
        Vec3{-extentX, -extentY, minZ},
        Vec3{extentX, -extentY, minZ},
        Vec3{extentX, extentY, minZ},
        Vec3{-extentX, extentY, minZ},
        Vec3{-extentX, -extentY, maxZ},
        Vec3{extentX, -extentY, maxZ},
        Vec3{extentX, extentY, maxZ},
        Vec3{-extentX, extentY, maxZ},
    };

    const Vec3 center = sceneCenter();
    double maxExtent = 0.01;
    for (const Vec3 &corner : corners)
    {
        const Vec3 mapped = mapScenePoint(corner);
        maxExtent = std::max(maxExtent, std::abs(mapped.x - center.x));
        maxExtent = std::max(maxExtent, std::abs(mapped.y - center.y));
        maxExtent = std::max(maxExtent, std::abs(mapped.z - center.z));
    }

    return std::min(bounds.width(), bounds.height()) / (maxExtent * kSceneFitPadding);
}

void Ur3eHemisphereScanPreviewWidget::paintEvent(QPaintEvent *event)
{
    QWidget::paintEvent(event);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.fillRect(rect(), QColor(26, 26, 26));

    const QRectF bounds = rect().adjusted(16.0, 16.0, -16.0, -16.0);
    const double scale = sceneScale(bounds) * zoomFactor_;

    if (workspaceBoundary_.enabled)
        drawWorkspaceBoundary(painter, bounds, scale);
    drawTray(painter, bounds, scale);
    if (semiFixedPreviewActive_)
        drawSemiFixedRings(painter, bounds, scale);
    else
    {
        drawHemisphere(painter, bounds, scale);
        drawScanNormals(painter, bounds, scale);
    }
    drawLegend(painter);
}

bool Ur3eHemisphereScanPreviewWidget::hasReachabilityLegend() const
{
    if (semiFixedPreviewActive_)
    {
        for (const PreviewSemiFixedRing &entry : semiFixedRings_)
        {
            if (entry.ring.reachabilityKnown)
                return true;
        }
        return false;
    }
    for (const PreviewScanPoint &entry : scanPoints_)
    {
        if (entry.reachabilityKnown)
            return true;
    }
    return false;
}

void Ur3eHemisphereScanPreviewWidget::drawLegend(QPainter &painter) const
{
    if (!semiFixedPreviewActive_ && scanPoints_.empty())
        return;
    if (semiFixedPreviewActive_ && semiFixedRings_.empty())
        return;

    struct LegendEntry
    {
        QColor color;
        QString label;
    };

    std::vector<LegendEntry> entries;
    entries.push_back({QColor(140, 140, 140), QStringLiteral("Preview")});
    if (hasReachabilityLegend())
    {
        if (executionActive_)
        {
            entries.push_back({QColor(170, 90, 230), QStringLiteral("Current")});
            entries.push_back({QColor(220, 190, 40), QStringLiteral("Pending")});
            entries.push_back({QColor(60, 180, 75), QStringLiteral("Completed")});
            entries.push_back({QColor(210, 45, 45), QStringLiteral("Execute failed")});
            entries.push_back({QColor(70, 130, 220), QStringLiteral("Unreachable (plan)")});
        }
        else if (executionResultsVisible_)
        {
            entries.push_back({QColor(60, 180, 75), QStringLiteral("Completed")});
            entries.push_back({QColor(210, 45, 45), QStringLiteral("Execute failed")});
            entries.push_back({QColor(60, 180, 75), QStringLiteral("Reachable (home)")});
            entries.push_back({QColor(230, 150, 40), QStringLiteral("Chain-only")});
            entries.push_back({QColor(70, 130, 220), QStringLiteral("Unreachable (plan)")});
        }
        else
        {
            entries.push_back({QColor(60, 180, 75), QStringLiteral("Reachable (home)")});
            entries.push_back({QColor(230, 150, 40), QStringLiteral("Chain-only")});
            entries.push_back({QColor(70, 130, 220), QStringLiteral("Unreachable")});
        }
    }

    QFont legendFont = painter.font();
    legendFont.setPointSize(9);
    painter.setFont(legendFont);
    const QFontMetrics metrics(legendFont);

    constexpr int kSwatchSize = 10;
    constexpr int kRowSpacing = 6;
    constexpr int kItemSpacing = 14;
    constexpr int kPadH = 10;
    constexpr int kPadV = 6;

    int contentWidth = 0;
    for (const LegendEntry &entry : entries)
        contentWidth += kSwatchSize + 6 + metrics.horizontalAdvance(entry.label) + kItemSpacing;
    if (!entries.empty())
        contentWidth -= kItemSpacing;

    const int contentHeight = std::max(kSwatchSize, metrics.height());
    const QRect legendRect(12,
                           height() - contentHeight - kPadV * 2 - 12,
                           contentWidth + kPadH * 2,
                           contentHeight + kPadV * 2);

    painter.save();
    painter.setPen(QPen(QColor(80, 80, 80), 1.0));
    painter.setBrush(QColor(26, 26, 26, 210));
    painter.drawRoundedRect(legendRect, 4, 4);

    int x = legendRect.left() + kPadH;
    const int swatchY = legendRect.center().y() - kSwatchSize / 2;
    const int textY = legendRect.top();
    const int textHeight = legendRect.height();

    for (const LegendEntry &entry : entries)
    {
        painter.setPen(Qt::NoPen);
        painter.setBrush(entry.color);
        painter.drawRoundedRect(QRect(x, swatchY, kSwatchSize, kSwatchSize), 2, 2);
        x += kSwatchSize + kRowSpacing;

        painter.setPen(QColor(200, 200, 200));
        painter.drawText(QRect(x, textY, metrics.horizontalAdvance(entry.label), textHeight),
                         Qt::AlignVCenter,
                         entry.label);
        x += metrics.horizontalAdvance(entry.label) + kItemSpacing;
    }

    painter.restore();
}

void Ur3eHemisphereScanPreviewWidget::drawWorkspaceBoundary(QPainter &painter,
                                                            const QRectF &bounds,
                                                            const double scale) const
{
    const double halfLength = workspaceBoundary_.halfLengthM();
    const double halfWidth = workspaceBoundary_.halfWidthM();
    const double floorZM = workspaceBoundary_.floorZM();
    const double topZM = workspaceBoundary_.topZM();

    const std::array<Vec3, 8> vertices = {
        Vec3{-halfLength, -halfWidth, floorZM},
        Vec3{halfLength, -halfWidth, floorZM},
        Vec3{halfLength, halfWidth, floorZM},
        Vec3{-halfLength, halfWidth, floorZM},
        Vec3{-halfLength, -halfWidth, topZM},
        Vec3{halfLength, -halfWidth, topZM},
        Vec3{halfLength, halfWidth, topZM},
        Vec3{-halfLength, halfWidth, topZM},
    };

    const std::array<std::pair<int, int>, 12> edges = {{
        {0, 1}, {1, 2}, {2, 3}, {3, 0},
        {4, 5}, {5, 6}, {6, 7}, {7, 4},
        {0, 4}, {1, 5}, {2, 6}, {3, 7},
    }};

    struct Face
    {
        std::array<int, 4> indices;
        QColor fill;
        double depth = 0.0;
    };

    const std::array<Face, 6> faces = {{
        {{0, 1, 2, 3}, QColor(120, 120, 120, 35)},
        {{4, 5, 6, 7}, QColor(90, 110, 150, 55)},
        {{0, 1, 5, 4}, QColor(100, 100, 100, 25)},
        {{1, 2, 6, 5}, QColor(100, 100, 100, 25)},
        {{2, 3, 7, 6}, QColor(100, 100, 100, 25)},
        {{3, 0, 4, 7}, QColor(100, 100, 100, 25)},
    }};

    std::vector<Face> sortedFaces(faces.begin(), faces.end());
    for (Face &face : sortedFaces)
    {
        double depthSum = 0.0;
        for (const int index : face.indices)
            depthSum += projectPoint(vertices[static_cast<std::size_t>(index)], bounds, scale).depth;
        face.depth = depthSum / static_cast<double>(face.indices.size());
    }
    std::sort(sortedFaces.begin(),
              sortedFaces.end(),
              [](const Face &lhs, const Face &rhs) { return lhs.depth < rhs.depth; });

    for (const Face &face : sortedFaces)
    {
        QPainterPath path;
        bool first = true;
        for (const int index : face.indices)
        {
            const QPointF point =
                projectPoint(vertices[static_cast<std::size_t>(index)], bounds, scale).screen;
            if (first)
            {
                path.moveTo(point);
                first = false;
            }
            else
            {
                path.lineTo(point);
            }
        }
        path.closeSubpath();
        painter.setPen(Qt::NoPen);
        painter.setBrush(face.fill);
        painter.drawPath(path);
    }

    painter.setPen(QPen(QColor(60, 60, 60), 1.5, Qt::SolidLine));
    for (const auto &edge : edges)
    {
        const QPointF start =
            projectPoint(vertices[static_cast<std::size_t>(edge.first)], bounds, scale).screen;
        const QPointF end =
            projectPoint(vertices[static_cast<std::size_t>(edge.second)], bounds, scale).screen;
        painter.drawLine(start, end);
    }

    painter.setPen(QPen(QColor(70, 100, 160), 2.0, Qt::DashLine));
    const std::array<std::pair<int, int>, 4> topEdges = {{{4, 5}, {5, 6}, {6, 7}, {7, 4}}};
    for (const auto &edge : topEdges)
    {
        const QPointF start =
            projectPoint(vertices[static_cast<std::size_t>(edge.first)], bounds, scale).screen;
        const QPointF end =
            projectPoint(vertices[static_cast<std::size_t>(edge.second)], bounds, scale).screen;
        painter.drawLine(start, end);
    }
}

void Ur3eHemisphereScanPreviewWidget::drawTray(QPainter &painter,
                                               const QRectF &bounds,
                                               const double scale) const
{
    const double halfLength = kTrayLengthM * 0.5;
    const double halfWidth = kTrayWidthM * 0.5;
    double centerXM = 0.0;
    double centerYM = 0.0;
    hf::ur3e::scanCenterOffsetM(centerXM, centerYM);

    const std::array<Vec3, 8> vertices = {
        Vec3{centerXM - halfLength, centerYM - halfWidth, 0.0},
        Vec3{centerXM + halfLength, centerYM - halfWidth, 0.0},
        Vec3{centerXM + halfLength, centerYM + halfWidth, 0.0},
        Vec3{centerXM - halfLength, centerYM + halfWidth, 0.0},
        Vec3{centerXM - halfLength, centerYM - halfWidth, kTrayHeightM},
        Vec3{centerXM + halfLength, centerYM - halfWidth, kTrayHeightM},
        Vec3{centerXM + halfLength, centerYM + halfWidth, kTrayHeightM},
        Vec3{centerXM - halfLength, centerYM + halfWidth, kTrayHeightM},
    };

    struct Face
    {
        std::array<int, 4> indices;
        QColor color;
        double depth = 0.0;
    };

    const std::array<Face, 6> faces = {{
        {{0, 1, 2, 3}, QColor(150, 145, 138)},
        {{4, 5, 6, 7}, QColor(196, 190, 180)},
        {{0, 1, 5, 4}, QColor(176, 170, 162)},
        {{1, 2, 6, 5}, QColor(166, 160, 152)},
        {{2, 3, 7, 6}, QColor(156, 150, 142)},
        {{3, 0, 4, 7}, QColor(186, 180, 172)},
    }};

    std::vector<Face> sortedFaces(faces.begin(), faces.end());
    for (Face &face : sortedFaces)
    {
        double depthSum = 0.0;
        for (const int index : face.indices)
            depthSum += projectPoint(vertices[static_cast<std::size_t>(index)], bounds, scale).depth;
        face.depth = depthSum / static_cast<double>(face.indices.size());
    }
    std::sort(sortedFaces.begin(),
              sortedFaces.end(),
              [](const Face &lhs, const Face &rhs) { return lhs.depth < rhs.depth; });

    for (const Face &face : sortedFaces)
    {
        QPainterPath path;
        bool first = true;
        for (const int index : face.indices)
        {
            const QPointF point =
                projectPoint(vertices[static_cast<std::size_t>(index)], bounds, scale).screen;
            if (first)
            {
                path.moveTo(point);
                first = false;
            }
            else
            {
                path.lineTo(point);
            }
        }
        path.closeSubpath();
        painter.setPen(QPen(face.color.darker(115), 1.0));
        painter.setBrush(face.color);
        painter.drawPath(path);
    }
}

void Ur3eHemisphereScanPreviewWidget::drawSemiFixedRings(QPainter &painter,
                                                         const QRectF &bounds,
                                                         const double scale) const
{
    constexpr int kSegments = 64;
    for (int i = 0; i < static_cast<int>(semiFixedRings_.size()); ++i)
    {
        const PreviewSemiFixedRing &entry = semiFixedRings_[static_cast<std::size_t>(i)];
        const auto &ring = entry.ring;

        // Same execute palette as Auto pins: pending yellow, current purple pulse,
        // done green, failed red. Plan-unreachable latitudes stay blue.
        QColor color(80, 180, 255);
        const bool planUnreachable = ring.reachabilityKnown && !ring.reachable;
        const bool isActive = executionActive_ && executionActivePointIndex_ == i;
        double lineWidth = isActive ? 2.5 : 1.8;
        if (planUnreachable)
        {
            color = QColor(70, 130, 220);
        }
        else if (executionActive_ && (isActive || !entry.executionCompleted))
        {
            if (entry.executionFailed)
                color = QColor(210, 45, 45);
            else if (entry.executionCompleted)
                color = QColor(60, 180, 75);
            else if (isActive)
            {
                const double pulse =
                    0.5 + 0.5 * std::sin(static_cast<double>(flashPulse_) * kPi / 8.0);
                const int red = static_cast<int>(std::clamp(170.0 + pulse * 50.0, 0.0, 255.0));
                const int green = static_cast<int>(std::clamp(70.0 + pulse * 40.0, 0.0, 255.0));
                const int blue = static_cast<int>(std::clamp(210.0 + pulse * 45.0, 0.0, 255.0));
                color = QColor(red, green, blue);
                lineWidth = 3.0;
            }
            else
                color = QColor(220, 190, 40); // pending
        }
        else if (entry.executionFailed)
        {
            color = QColor(210, 45, 45);
        }
        else if (entry.executionCompleted)
        {
            color = QColor(60, 180, 75);
        }
        else if (ring.reachabilityKnown)
        {
            if (!ring.reachable)
                color = QColor(70, 130, 220);
            else if (!ring.homePathOk)
                color = QColor(230, 150, 40);
            else
                color = QColor(60, 180, 75);
        }
        else
        {
            color = QColor(140, 140, 140);
        }

        if (ring.isTopPose || ring.radiusM < 1.0e-3)
        {
            // Apex = north pole of the spin-ring sphere (same R), not the tray look-at.
            double centerXM = 0.0;
            double centerYM = 0.0;
            hf::ur3e::scanCenterOffsetM(centerXM, centerYM);
            double apexZM = 0.0;
            for (const PreviewSemiFixedRing &other : semiFixedRings_)
            {
                if (other.ring.isTopPose || other.ring.radiusM < 1.0e-3)
                    continue;
                apexZM = std::max(apexZM, std::hypot(other.ring.radiusM, other.ring.centerZM));
            }
            if (apexZM < 0.01)
                apexZM = params_.sphereRadiusM;
            const Vec3 surface{centerXM, centerYM, apexZM};
            const Vec3 sphereCenter{centerXM, centerYM, kTrayHeightM};
            double dirX = surface.x - sphereCenter.x;
            double dirY = surface.y - sphereCenter.y;
            double dirZ = surface.z - sphereCenter.z;
            const double length = std::sqrt(dirX * dirX + dirY * dirY + dirZ * dirZ);
            if (length > 1.0e-9)
            {
                dirX /= length;
                dirY /= length;
                dirZ /= length;
                const double halfLen = kNormalDisplayLengthM * 0.5;
                const Vec3 pinStart{surface.x - dirX * halfLen,
                                    surface.y - dirY * halfLen,
                                    surface.z - dirZ * halfLen};
                const Vec3 pinEnd{surface.x + dirX * halfLen,
                                  surface.y + dirY * halfLen,
                                  surface.z + dirZ * halfLen};
                painter.setPen(QPen(color, lineWidth, Qt::SolidLine, Qt::RoundCap));
                painter.drawLine(projectPoint(pinStart, bounds, scale).screen,
                                 projectPoint(pinEnd, bounds, scale).screen);
            }
            const QPointF tip = projectPoint(surface, bounds, scale).screen;
            painter.setBrush(color);
            painter.setPen(Qt::NoPen);
            painter.drawEllipse(tip, isActive ? 6.0 : 5.0, isActive ? 6.0 : 5.0);
            continue;
        }

        QPainterPath path;
        for (int s = 0; s <= kSegments; ++s)
        {
            const double ang = (2.0 * kPi * static_cast<double>(s)) / static_cast<double>(kSegments);
            const Vec3 p{ring.centerXM + ring.radiusM * std::cos(ang),
                         ring.centerYM + ring.radiusM * std::sin(ang),
                         ring.centerZM};
            const QPointF screen = projectPoint(p, bounds, scale).screen;
            if (s == 0)
                path.moveTo(screen);
            else
                path.lineTo(screen);
        }
        painter.setPen(QPen(color, lineWidth));
        painter.setBrush(Qt::NoBrush);
        painter.drawPath(path);
    }
}

void Ur3eHemisphereScanPreviewWidget::drawHemisphere(QPainter &painter,
                                                     const QRectF &bounds,
                                                     const double scale) const
{
    const double radius = params_.sphereRadiusM;
    const int latitudeSteps = 14;
    const int longitudeSteps = 24;
    double centerXM = 0.0;
    double centerYM = 0.0;
    hf::ur3e::scanCenterOffsetM(centerXM, centerYM);

    struct Triangle
    {
        std::array<QPointF, 3> points;
        double depth = 0.0;
        QColor color;
    };

    std::vector<Triangle> triangles;
    triangles.reserve(static_cast<std::size_t>(latitudeSteps * longitudeSteps * 2));

    const auto appendTriangle = [&](const Vec3 &a, const Vec3 &b, const Vec3 &c, const double shade) {
        Triangle triangle;
        const ProjectedPoint pa = projectPoint(a, bounds, scale);
        const ProjectedPoint pb = projectPoint(b, bounds, scale);
        const ProjectedPoint pc = projectPoint(c, bounds, scale);
        triangle.points = {pa.screen, pb.screen, pc.screen};
        triangle.depth = (pa.depth + pb.depth + pc.depth) / 3.0;
        const int blue = static_cast<int>(std::clamp(150.0 + shade * 70.0, 110.0, 220.0));
        triangle.color = QColor(90, blue, 235, 170);
        triangles.push_back(triangle);
    };

    for (int latIndex = 0; latIndex < latitudeSteps; ++latIndex)
    {
        // Always draw from the apex so the fixed top pin sits on the dome mesh.
        const double thetaMinRad = 0.0;
        const double thetaMaxRad = params_.thetaMaxDeg * kPi / 180.0;
        const double thetaA =
            thetaMinRad
            + (thetaMaxRad - thetaMinRad) * static_cast<double>(latIndex)
                  / static_cast<double>(latitudeSteps);
        const double thetaB =
            thetaMinRad
            + (thetaMaxRad - thetaMinRad) * static_cast<double>(latIndex + 1)
                  / static_cast<double>(latitudeSteps);

        for (int lonIndex = 0; lonIndex < longitudeSteps; ++lonIndex)
        {
            const double phiA =
                2.0 * kPi * static_cast<double>(lonIndex) / static_cast<double>(longitudeSteps);
            const double phiB =
                2.0 * kPi * static_cast<double>(lonIndex + 1) / static_cast<double>(longitudeSteps);

            const auto spherePoint = [&](const double theta, const double phi) -> Vec3 {
                const double sinTheta = std::sin(theta);
                return Vec3{centerXM + radius * sinTheta * std::cos(phi),
                          centerYM + radius * sinTheta * std::sin(phi),
                          kTrayHeightM + radius * std::cos(theta)};
            };

            const Vec3 p00 = spherePoint(thetaA, phiA);
            const Vec3 p01 = spherePoint(thetaA, phiB);
            const Vec3 p10 = spherePoint(thetaB, phiA);
            const Vec3 p11 = spherePoint(thetaB, phiB);
            const double shade = std::cos(thetaA);

            appendTriangle(p00, p10, p11, shade);
            appendTriangle(p00, p11, p01, shade);
        }
    }

    std::sort(triangles.begin(),
              triangles.end(),
              [](const Triangle &lhs, const Triangle &rhs) { return lhs.depth < rhs.depth; });

    for (const Triangle &triangle : triangles)
    {
        QPainterPath path;
        path.moveTo(triangle.points[0]);
        path.lineTo(triangle.points[1]);
        path.lineTo(triangle.points[2]);
        path.closeSubpath();
        painter.setPen(Qt::NoPen);
        painter.setBrush(triangle.color);
        painter.drawPath(path);
    }

    painter.setPen(QPen(QColor(70, 120, 200, 180), 1.0));
    painter.setBrush(Qt::NoBrush);
    for (int lonIndex = 0; lonIndex <= longitudeSteps; lonIndex += 3)
    {
        const double phi = 2.0 * kPi * static_cast<double>(lonIndex) / static_cast<double>(longitudeSteps);
        QPainterPath meridian;
        bool first = true;
        for (int latIndex = 0; latIndex <= latitudeSteps; ++latIndex)
        {
            const double thetaMinRad = 0.0;
            const double thetaMaxRad = params_.thetaMaxDeg * kPi / 180.0;
            const double theta =
                thetaMinRad
                + (thetaMaxRad - thetaMinRad) * static_cast<double>(latIndex)
                      / static_cast<double>(latitudeSteps);
            const double sinTheta = std::sin(theta);
            const Vec3 point{centerXM + radius * sinTheta * std::cos(phi),
                             centerYM + radius * sinTheta * std::sin(phi),
                             kTrayHeightM + radius * std::cos(theta)};
            const QPointF screen = projectPoint(point, bounds, scale).screen;
            if (first)
            {
                meridian.moveTo(screen);
                first = false;
            }
            else
            {
                meridian.lineTo(screen);
            }
        }
        painter.drawPath(meridian);
    }
}

void Ur3eHemisphereScanPreviewWidget::drawScanNormals(QPainter &painter,
                                                      const QRectF &bounds,
                                                      const double scale) const
{
    for (int pointIndex = 0; pointIndex < static_cast<int>(scanPoints_.size()); ++pointIndex)
    {
        if (executionActive_ && pointIndex == executionActivePointIndex_)
            continue;
        drawScanPin(painter, bounds, scale, scanPoints_[static_cast<std::size_t>(pointIndex)], pointIndex);
    }

    if (executionActive_ && executionActivePointIndex_ >= 0
        && executionActivePointIndex_ < static_cast<int>(scanPoints_.size()))
    {
        drawScanPin(painter,
                    bounds,
                    scale,
                    scanPoints_[static_cast<std::size_t>(executionActivePointIndex_)],
                    executionActivePointIndex_);
    }
}

void Ur3eHemisphereScanPreviewWidget::drawScanPin(QPainter &painter,
                                                  const QRectF &bounds,
                                                  const double scale,
                                                  const PreviewScanPoint &entry,
                                                  const int pointIndex) const
{
    Q_UNUSED(pointIndex);

    QColor pinColor(140, 140, 140);
    if (entry.reachabilityKnown)
    {
        if (!entry.reachable)
        {
            pinColor = QColor(70, 130, 220);
        }
        else if (executionActive_ || executionResultsVisible_)
        {
            if (entry.executionFailed)
                pinColor = QColor(210, 45, 45);
            else if (entry.executionCompleted)
                pinColor = QColor(60, 180, 75);
            else
                pinColor = QColor(220, 190, 40);
        }
        else if (!entry.homePathOk)
        {
            pinColor = QColor(230, 150, 40);
        }
        else
        {
            pinColor = QColor(60, 180, 75);
        }
    }

    const bool isActive = executionActive_ && pointIndex == executionActivePointIndex_;
    double lineWidth = 2.0;
    if (isActive)
    {
        const double pulse =
            0.5 + 0.5 * std::sin(static_cast<double>(flashPulse_) * kPi / 8.0);
        const int red = static_cast<int>(std::clamp(170.0 + pulse * 50.0, 0.0, 255.0));
        const int green = static_cast<int>(std::clamp(70.0 + pulse * 40.0, 0.0, 255.0));
        const int blue = static_cast<int>(std::clamp(210.0 + pulse * 45.0, 0.0, 255.0));
        pinColor = QColor(red, green, blue);
        lineWidth = 3.0;
    }

    double centerXM = 0.0;
    double centerYM = 0.0;
    hf::ur3e::scanCenterOffsetM(centerXM, centerYM);
    const Vec3 sphereCenter = mapScenePoint(Vec3{centerXM, centerYM, kTrayHeightM});
    const Vec3 surface{entry.point.xM, entry.point.yM, entry.point.zM};
    const Vec3 mappedSurface = mapScenePoint(surface);
    double dirX = mappedSurface.x - sphereCenter.x;
    double dirY = mappedSurface.y - sphereCenter.y;
    double dirZ = mappedSurface.z - sphereCenter.z;
    const double length = std::sqrt(dirX * dirX + dirY * dirY + dirZ * dirZ);
    if (length <= 1.0e-9)
        return;

    dirX /= length;
    dirY /= length;
    dirZ /= length;

    const double halfLen = kNormalDisplayLengthM * 0.5;
    const Vec3 pinStart{
        mappedSurface.x - dirX * halfLen,
        mappedSurface.y - dirY * halfLen,
        mappedSurface.z - dirZ * halfLen,
    };
    const Vec3 pinEnd{
        mappedSurface.x + dirX * halfLen,
        mappedSurface.y + dirY * halfLen,
        mappedSurface.z + dirZ * halfLen,
    };

    const QPointF start = projectPoint(pinStart, bounds, scale).screen;
    const QPointF end = projectPoint(pinEnd, bounds, scale).screen;

    painter.setPen(QPen(pinColor, lineWidth, Qt::SolidLine, Qt::RoundCap));
    painter.drawLine(start, end);
}
} // namespace ui
