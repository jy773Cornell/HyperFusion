// Builds the left-hand settings tab panel (camera, stage, light, Multiview, capture).
// MainWindow method definitions extracted from MainWindow.cpp for clarity.
#include "frontend/widgets/MainWindow.hpp"
#include <QTabWidget>
#include <QVBoxLayout>
#include <QWidget>
QWidget *MainWindow::createSettingsPanel()
{
    auto *panel = new QWidget(this);
    // Keep the settings column compact; UR3e route combo no longer forces panel width.
    panel->setMinimumWidth(360);
    panel->setMaximumWidth(480);
    auto *layout = new QVBoxLayout(panel);
    layout->setContentsMargins(4, 4, 4, 4);

    settingsTabs_ = new QTabWidget(panel);
    settingsTabs_->addTab(createCameraSettingsTab(), QStringLiteral("Cameras"));
    settingsTabs_->addTab(createStageSettingsTab(), QStringLiteral("Stage"));
    settingsTabs_->addTab(createLightSettingsTab(), QStringLiteral("Light"));
    if (useMultiview_)
    {
        ur3eSettingsTabIndex_ = settingsTabs_->count();
        settingsTabs_->addTab(createUr3eSettingsTab(), QStringLiteral("Multiview"));
    }
    else
    {
        ur3eSettingsTabIndex_ = -1;
    }
    captureSettingsTabIndex_ = settingsTabs_->count();
    settingsTabs_->addTab(createCaptureSettingsTab(), QStringLiteral("Capture"));

    connect(settingsTabs_,
            &QTabWidget::currentChanged,
            this,
            &MainWindow::onSettingsTabChanged);

    layout->addWidget(settingsTabs_, 1);
    return panel;
}
