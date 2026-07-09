// Scan-route planning pane for the UR3e stream tab (frontend/ui layer).
#pragma once

#include <QWidget>

class QLabel;
class QListWidget;
class QPushButton;

namespace ui
{
class Ur3eScanRoutePlanWidget : public QWidget
{
    Q_OBJECT

public:
    explicit Ur3eScanRoutePlanWidget(QWidget *parent = nullptr);

    [[nodiscard]] QListWidget *routeList() const { return routeList_; }

private:
    void onAddWaypoint();
    void onRemoveWaypoint();
    void onClearRoute();
    void updateStatus();

    QLabel *canvasLabel_ = nullptr;
    QListWidget *routeList_ = nullptr;
    QLabel *statusLabel_ = nullptr;
    QPushButton *addBtn_ = nullptr;
    QPushButton *removeBtn_ = nullptr;
    QPushButton *clearBtn_ = nullptr;
    int nextWaypointId_ = 1;
};
} // namespace ui
