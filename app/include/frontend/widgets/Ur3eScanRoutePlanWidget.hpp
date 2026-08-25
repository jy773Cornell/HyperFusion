// Scan-route planning pane for the UR3e stream tab (frontend/ui layer).
#pragma once

#include "backend/multiview/Ur3eHemisphereScan.hpp"
#include "backend/multiview/Ur3eHemisphereScanReachability.hpp"
#include "backend/multiview/Ur3eMountTransform.hpp"
#include "backend/multiview/Ur3eSemiFixedScan.hpp"
#include "backend/multiview/Ur3eWorkspaceBoundary.hpp"

#include <QVector>
#include <QWidget>

class QLabel;

namespace ui
{
class Ur3eHemisphereScanPreviewWidget;

class Ur3eScanRoutePlanWidget : public QWidget
{
    Q_OBJECT

public:
    explicit Ur3eScanRoutePlanWidget(QWidget *parent = nullptr);

    void setScanParams(const hf::ur3e::Ur3eHemisphereScanParams &params);
    void setWorkspaceBoundary(const hf::ur3e::Ur3eWorkspaceBoundary &boundary);
    void setSceneMount(const hf::ur3e::Ur3eMountTransform &mount);
    void setScanPlan(const hf::ur3e::Ur3eHemisphereScanPlan &plan);
    void clearScanPlan();
    void setSemiFixedPreviewRings(const QVector<hf::ur3e::Ur3eSemiFixedPreviewRing> &rings);
    void clearSemiFixedPreviewRings();
    /// *plannedPins* = imaging poses: base pins × wrist imagesPerPin (center + sweeps).
    void beginScanExecution(int plannedPins);
    void setActiveScanPoint(int pointIndex);
    void markScanPointCompleted(int pointIndex);
    void markScanPointFailed(int pointIndex);
    void markPinCompleted();
    void markPinFailed();
    void endScanExecution();

private:
    void refreshProgressUi();

    Ur3eHemisphereScanPreviewWidget *previewWidget_ = nullptr;
    QWidget *progressBar_ = nullptr;
    QLabel *progressLabel_ = nullptr;
    int plannedPins_ = 0;
    int completedPins_ = 0;
    int failedPins_ = 0;
    bool progressActive_ = false;
};
} // namespace ui
