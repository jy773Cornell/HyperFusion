// Scan-route planning pane for the UR3e stream tab (frontend/ui layer).
#include "frontend/widgets/Ur3eScanRoutePlanWidget.hpp"

#include "frontend/widgets/Ur3eHemisphereScanPreviewWidget.hpp"

#include <QVBoxLayout>

namespace ui
{
Ur3eScanRoutePlanWidget::Ur3eScanRoutePlanWidget(QWidget *parent) : QWidget(parent)
{
    setAttribute(Qt::WA_StyledBackground, true);
    setStyleSheet(QStringLiteral("background-color: transparent;"));

    auto *layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);

    previewWidget_ = new Ur3eHemisphereScanPreviewWidget(this);
    layout->addWidget(previewWidget_, 1);
}

void Ur3eScanRoutePlanWidget::setScanParams(const hf::ur3e::Ur3eHemisphereScanParams &params)
{
    if (previewWidget_ != nullptr)
        previewWidget_->setScanParams(params);
}

void Ur3eScanRoutePlanWidget::setWorkspaceBoundary(
    const hf::ur3e::Ur3eWorkspaceBoundary &boundary)
{
    if (previewWidget_ != nullptr)
        previewWidget_->setWorkspaceBoundary(boundary);
}

void Ur3eScanRoutePlanWidget::setScanPlan(const hf::ur3e::Ur3eHemisphereScanPlan &plan)
{
    if (previewWidget_ != nullptr)
        previewWidget_->setScanPlan(plan);
}

void Ur3eScanRoutePlanWidget::clearScanPlan()
{
    if (previewWidget_ != nullptr)
        previewWidget_->clearScanPlan();
}

void Ur3eScanRoutePlanWidget::beginScanExecution()
{
    if (previewWidget_ != nullptr)
        previewWidget_->beginScanExecution();
}

void Ur3eScanRoutePlanWidget::setActiveScanPoint(const int pointIndex)
{
    if (previewWidget_ != nullptr)
        previewWidget_->setActiveScanPoint(pointIndex);
}

void Ur3eScanRoutePlanWidget::markScanPointCompleted(const int pointIndex)
{
    if (previewWidget_ != nullptr)
        previewWidget_->markScanPointCompleted(pointIndex);
}

void Ur3eScanRoutePlanWidget::endScanExecution()
{
    if (previewWidget_ != nullptr)
        previewWidget_->endScanExecution();
}
} // namespace ui
