// Builds the left-hand settings tab panel (camera, stage, light, UR3e, capture).
// MainWindow method definitions extracted from MainWindow.cpp for clarity.
#include "frontend/widgets/MainWindow.hpp"
#include <QTabWidget>
#include <QVBoxLayout>
#include <QWidget>
QWidget *MainWindow::createSettingsPanel()
{
    auto *panel = new QWidget(this);
    panel->setMinimumWidth(380);
    panel->setMaximumWidth(520);
    auto *layout = new QVBoxLayout(panel);

    settingsTabs_ = new QTabWidget(panel);
    settingsTabs_->addTab(createCameraSettingsTab(), QStringLiteral("Camera"));
    settingsTabs_->addTab(createStageSettingsTab(), QStringLiteral("Stage"));
    settingsTabs_->addTab(createLightSettingsTab(), QStringLiteral("Light"));
    settingsTabs_->addTab(createUr3eSettingsTab(), QStringLiteral("UR3e"));
    settingsTabs_->addTab(createCaptureSettingsTab(), QStringLiteral("Capture"));

    connect(settingsTabs_,
            &QTabWidget::currentChanged,
            this,
            &MainWindow::onSettingsTabChanged);

    layout->addWidget(settingsTabs_, 1);
    return panel;
}
