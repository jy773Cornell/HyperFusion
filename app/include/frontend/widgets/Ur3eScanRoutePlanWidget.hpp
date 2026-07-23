// Scan-route planning pane for the UR3e stream tab (frontend/ui layer).
#pragma once

#include "backend/3dscanning/Ur3eHemisphereScan.hpp"
#include "backend/3dscanning/Ur3eHemisphereScanReachability.hpp"
#include "backend/3dscanning/Ur3eMountTransform.hpp"
#include "backend/3dscanning/Ur3eWorkspaceBoundary.hpp"

#include <QWidget>

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
    void beginScanExecution();
    void setActiveScanPoint(int pointIndex);
    void markScanPointCompleted(int pointIndex);
    void markScanPointFailed(int pointIndex);
    void endScanExecution();

private:
    Ur3eHemisphereScanPreviewWidget *previewWidget_ = nullptr;
};
} // namespace ui
