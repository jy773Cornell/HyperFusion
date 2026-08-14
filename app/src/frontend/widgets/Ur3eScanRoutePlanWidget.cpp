// Scan-route planning pane for the UR3e stream tab (frontend/ui layer).
#include "frontend/widgets/Ur3eScanRoutePlanWidget.hpp"

#include "frontend/widgets/Ur3eHemisphereScanPreviewWidget.hpp"

#include <algorithm>
#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QPaintEvent>
#include <QVBoxLayout>

namespace ui
{
namespace
{
class Ur3eScanProgressBar final : public QWidget
{
public:
    explicit Ur3eScanProgressBar(QWidget *parent = nullptr) : QWidget(parent)
    {
        setMinimumHeight(10);
        setMaximumHeight(12);
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    }

    void setCounts(const int total, const int completed, const int failed)
    {
        total_ = std::max(0, total);
        completed_ = std::max(0, completed);
        failed_ = std::max(0, failed);
        update();
    }

protected:
    void paintEvent(QPaintEvent *event) override
    {
        Q_UNUSED(event);
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing, true);
        const QRectF r = QRectF(rect()).adjusted(0.5, 0.5, -0.5, -0.5);
        p.setPen(Qt::NoPen);
        p.setBrush(QColor(40, 44, 52));
        p.drawRoundedRect(r, 3.0, 3.0);

        if (total_ <= 0)
            return;

        const double doneFrac =
            static_cast<double>(completed_) / static_cast<double>(total_);
        const double failFrac =
            static_cast<double>(failed_) / static_cast<double>(total_);
        const double doneW = r.width() * std::clamp(doneFrac, 0.0, 1.0);
        const double failW = r.width() * std::clamp(failFrac, 0.0, 1.0 - doneFrac);

        if (doneW > 0.5)
        {
            p.setBrush(QColor(70, 175, 95));
            p.drawRoundedRect(QRectF(r.left(), r.top(), doneW, r.height()), 3.0, 3.0);
        }
        if (failW > 0.5)
        {
            p.setBrush(QColor(210, 70, 70));
            p.drawRoundedRect(QRectF(r.left() + doneW, r.top(), failW, r.height()), 3.0, 3.0);
        }
    }

private:
    int total_ = 0;
    int completed_ = 0;
    int failed_ = 0;
};
} // namespace

Ur3eScanRoutePlanWidget::Ur3eScanRoutePlanWidget(QWidget *parent) : QWidget(parent)
{
    setAttribute(Qt::WA_StyledBackground, true);
    setStyleSheet(QStringLiteral("background-color: transparent;"));

    auto *layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(4);

    auto *statusRow = new QWidget(this);
    auto *statusLayout = new QHBoxLayout(statusRow);
    statusLayout->setContentsMargins(4, 2, 4, 0);
    statusLayout->setSpacing(8);

    progressLabel_ = new QLabel(QStringLiteral("Scan idle"), statusRow);
    progressLabel_->setStyleSheet(
        QStringLiteral("color: #c8cdd5; font-size: 11px; background: transparent;"));
    statusLayout->addWidget(progressLabel_, 1);

    layout->addWidget(statusRow, 0);

    progressBar_ = new Ur3eScanProgressBar(this);
    progressBar_->setVisible(false);
    layout->addWidget(progressBar_, 0);

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

void Ur3eScanRoutePlanWidget::setSceneMount(const hf::ur3e::Ur3eMountTransform &mount)
{
    if (previewWidget_ != nullptr)
        previewWidget_->setSceneMount(mount);
}

void Ur3eScanRoutePlanWidget::setScanPlan(const hf::ur3e::Ur3eHemisphereScanPlan &plan)
{
    if (previewWidget_ != nullptr)
        previewWidget_->setScanPlan(plan);
    if (!progressActive_)
    {
        plannedPins_ = previewWidget_ != nullptr ? previewWidget_->plannedExecutionCount() : 0;
        completedPins_ = 0;
        failedPins_ = 0;
        refreshProgressUi();
    }
}

void Ur3eScanRoutePlanWidget::clearScanPlan()
{
    if (previewWidget_ != nullptr)
        previewWidget_->clearScanPlan();
    if (!progressActive_)
    {
        plannedPins_ = 0;
        completedPins_ = 0;
        failedPins_ = 0;
        refreshProgressUi();
    }
}

void Ur3eScanRoutePlanWidget::setSemiFixedPreviewRings(
    const QVector<hf::ur3e::Ur3eSemiFixedPreviewRing> &rings)
{
    if (previewWidget_ != nullptr)
        previewWidget_->setSemiFixedPreviewRings(rings);
    if (!progressActive_)
    {
        plannedPins_ = 0;
        completedPins_ = 0;
        failedPins_ = 0;
        refreshProgressUi();
    }
}

void Ur3eScanRoutePlanWidget::clearSemiFixedPreviewRings()
{
    if (previewWidget_ != nullptr)
        previewWidget_->clearSemiFixedPreviewRings();
    if (!progressActive_)
    {
        plannedPins_ = 0;
        completedPins_ = 0;
        failedPins_ = 0;
        refreshProgressUi();
    }
}

void Ur3eScanRoutePlanWidget::beginScanExecution(const int plannedPins)
{
    plannedPins_ = std::max(0, plannedPins);
    completedPins_ = 0;
    failedPins_ = 0;
    progressActive_ = true;
    if (previewWidget_ != nullptr)
        previewWidget_->beginScanExecution();
    refreshProgressUi();
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

void Ur3eScanRoutePlanWidget::markScanPointFailed(const int pointIndex)
{
    if (previewWidget_ != nullptr)
        previewWidget_->markScanPointFailed(pointIndex);
}

void Ur3eScanRoutePlanWidget::markPinCompleted()
{
    ++completedPins_;
    refreshProgressUi();
}

void Ur3eScanRoutePlanWidget::markPinFailed()
{
    ++failedPins_;
    refreshProgressUi();
}

void Ur3eScanRoutePlanWidget::endScanExecution()
{
    progressActive_ = false;
    if (previewWidget_ != nullptr)
        previewWidget_->endScanExecution();
    refreshProgressUi();
}

void Ur3eScanRoutePlanWidget::refreshProgressUi()
{
    if (progressBar_ == nullptr || progressLabel_ == nullptr)
        return;

    auto *bar = static_cast<Ur3eScanProgressBar *>(progressBar_);
    const int total = plannedPins_;
    const int completed = completedPins_;
    const int failed = failedPins_;
    const int finished = completed + failed;

    if (bar != nullptr)
        bar->setCounts(total, completed, failed);
    progressBar_->setVisible(total > 0 && (progressActive_ || finished > 0));

    if (total <= 0)
    {
        progressLabel_->setText(QStringLiteral("Scan idle"));
        return;
    }

    if (progressActive_)
    {
        progressLabel_->setText(
            QStringLiteral("Scanning %1 / %2 pins  ·  %3 done  ·  %4 failed")
                .arg(finished)
                .arg(total)
                .arg(completed)
                .arg(failed));
    }
    else if (finished > 0)
    {
        progressLabel_->setText(
            QStringLiteral("Scan result  %1 / %2 pins  ·  %3 done  ·  %4 failed")
                .arg(finished)
                .arg(total)
                .arg(completed)
                .arg(failed));
    }
    else
    {
        progressLabel_->setText(QStringLiteral("Ready  ·  %1 pins planned").arg(total));
    }
}
} // namespace ui
