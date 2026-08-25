// Multiview live stream tab: scan-route planner (left) + RGB preview (right).
#include "frontend/widgets/MainWindow.hpp"
#include "frontend/widgets/StreamPaneHelpers.hpp"
#include "frontend/widgets/Ur3eScanRoutePlanWidget.hpp"

#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QWidget>

QWidget *MainWindow::createUr3eStreamTab() {
  ur3eStreamPage_ = new QWidget(this);
  auto *grid = new QGridLayout(ur3eStreamPage_);
  grid->setContentsMargins(8, 8, 8, 8);
  grid->setSpacing(10);

  ur3eScanRoutePlanWidget_ = new ui::Ur3eScanRoutePlanWidget(ur3eStreamPage_);
  auto *routePane = ui::createStreamPane(
      ur3eStreamPage_, QStringLiteral("Scan route"), ur3eScanRoutePlanWidget_);

  QLabel *rgbLabel = nullptr;
  auto *rgbPane =
      ui::createPreviewPane(ur3eStreamPage_, QStringLiteral("BFS"), rgbLabel);
  ur3eRgbPreviewLabel_ = rgbLabel;
  ui::setPreviewDisconnectedText(ur3eRgbPreviewLabel_, QStringLiteral("stream"),
                                 QStringLiteral("BFS"));

  grid->addWidget(routePane, 0, 0);
  grid->addWidget(rgbPane, 0, 1);
  grid->setColumnStretch(0, 1);
  grid->setColumnStretch(1, 1);
  grid->setRowStretch(0, 1);

  return ur3eStreamPage_;
}
