// Builds the left-hand settings tab widget (camera, stage, light, Multiview, capture).
#pragma once

class MainWindow;
class QWidget;

namespace ui
{
class SettingsTabPanelBuilder
{
public:
    static QWidget *buildSettingsPanel(MainWindow *window);
};
} // namespace ui
