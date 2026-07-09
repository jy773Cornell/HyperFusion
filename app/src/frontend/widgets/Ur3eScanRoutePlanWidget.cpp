// Scan-route planning pane for the UR3e stream tab (frontend/ui layer).
#include "frontend/widgets/Ur3eScanRoutePlanWidget.hpp"

#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QVBoxLayout>

namespace ui
{
Ur3eScanRoutePlanWidget::Ur3eScanRoutePlanWidget(QWidget *parent) : QWidget(parent)
{
    auto *layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(6);

    auto *canvasFrame = new QFrame(this);
    canvasFrame->setFrameShape(QFrame::StyledPanel);
    canvasFrame->setMinimumHeight(180);
    canvasFrame->setStyleSheet(QStringLiteral("background-color: #111111;"));

    canvasLabel_ = new QLabel(
        QStringLiteral("Scan route planner\n(robot path preview — coming soon)"),
        canvasFrame);
    canvasLabel_->setAlignment(Qt::AlignCenter);
    canvasLabel_->setWordWrap(true);
    canvasLabel_->setStyleSheet(QStringLiteral("color: #9ec8ff; background-color: transparent;"));

    auto *canvasLayout = new QVBoxLayout(canvasFrame);
    canvasLayout->setContentsMargins(8, 8, 8, 8);
    canvasLayout->addWidget(canvasLabel_, 1);

    routeList_ = new QListWidget(this);
    routeList_->setMinimumHeight(96);
    routeList_->setSelectionMode(QAbstractItemView::SingleSelection);

    addBtn_ = new QPushButton(QStringLiteral("Add waypoint"), this);
    removeBtn_ = new QPushButton(QStringLiteral("Remove"), this);
    clearBtn_ = new QPushButton(QStringLiteral("Clear route"), this);
    removeBtn_->setEnabled(false);

    auto *buttonRow = new QHBoxLayout();
    buttonRow->setSpacing(8);
    buttonRow->addWidget(addBtn_, 1);
    buttonRow->addWidget(removeBtn_, 1);
    buttonRow->addWidget(clearBtn_, 1);

    statusLabel_ = new QLabel(QStringLiteral("No waypoints — GUI only (motion not wired)"), this);
    statusLabel_->setWordWrap(true);
    statusLabel_->setStyleSheet(QStringLiteral("color: #888888;"));

    layout->addWidget(canvasFrame, 2);
    layout->addWidget(routeList_, 1);
    layout->addLayout(buttonRow);
    layout->addWidget(statusLabel_);

    connect(addBtn_, &QPushButton::clicked, this, [this]() { onAddWaypoint(); });
    connect(removeBtn_, &QPushButton::clicked, this, [this]() { onRemoveWaypoint(); });
    connect(clearBtn_, &QPushButton::clicked, this, [this]() { onClearRoute(); });
    connect(routeList_, &QListWidget::itemSelectionChanged, this, [this]() {
        removeBtn_->setEnabled(routeList_->currentItem() != nullptr);
    });
}

void Ur3eScanRoutePlanWidget::onAddWaypoint()
{
    routeList_->addItem(
        QStringLiteral("Waypoint %1 — joint pose TBD").arg(nextWaypointId_++));
    updateStatus();
}

void Ur3eScanRoutePlanWidget::onRemoveWaypoint()
{
    const int row = routeList_->currentRow();
    if (row >= 0)
        delete routeList_->takeItem(row);
    removeBtn_->setEnabled(routeList_->currentItem() != nullptr);
    updateStatus();
}

void Ur3eScanRoutePlanWidget::onClearRoute()
{
    routeList_->clear();
    removeBtn_->setEnabled(false);
    updateStatus();
}

void Ur3eScanRoutePlanWidget::updateStatus()
{
    const int count = routeList_->count();
    if (count == 0)
    {
        statusLabel_->setText(QStringLiteral("No waypoints — GUI only (motion not wired)"));
        return;
    }

    statusLabel_->setText(
        QStringLiteral("%1 waypoint(s) — save/run route not wired yet").arg(count));
}
} // namespace ui
