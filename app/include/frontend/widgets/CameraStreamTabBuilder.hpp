// Builds per-camera stream tab pages (detector, waterfall, wavelength, pixel panes).
#pragma once

#include "frontend/widgets/LumoCameraUi.hpp"

#include <QString>
#include <functional>

class MainWindow;
class QWidget;

namespace ui
{
class CameraStreamTabBuilder
{
public:
    struct StreamTabHooks
    {
        std::function<void(LumoCameraUi &, int spatialIndex, int bandIndex)> onProfileLinesChanged;
        std::function<void(LumoCameraUi &)> syncProfileRgbMarkers;
        std::function<void(LumoCameraUi &)> updateStreamPaneTitles;
        std::function<QString(const LumoCameraUi &)> profileTabName;
    };

    static QWidget *buildCameraStreamTab(MainWindow *window,
                                         const QString &cameraName,
                                         LumoCameraUi &cameraUi,
                                         const StreamTabHooks &hooks);
};
} // namespace ui
