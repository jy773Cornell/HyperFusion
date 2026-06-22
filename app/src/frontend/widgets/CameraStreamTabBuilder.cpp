// Per-camera stream tab page builder implementation.
#include "frontend/widgets/CameraStreamTabBuilder.hpp"

#include "adapters/lumo/LumoDeviceTypes.hpp"
#include "frontend/processing/Overexposure.hpp"
#include "frontend/widgets/DetectorCrosshairWidget.hpp"
#include "frontend/widgets/MainWindow.hpp"
#include "frontend/widgets/ProfilePlotWidget.hpp"
#include "frontend/widgets/StreamPaneHelpers.hpp"
#include "frontend/widgets/WaterfallDisplayWidget.hpp"

#include <QGridLayout>
#include <QGroupBox>
#include <QWidget>

namespace ui
{
QWidget *CameraStreamTabBuilder::buildCameraStreamTab(MainWindow *window,
                                                      const QString &cameraName,
                                                      LumoCameraUi &cameraUi,
                                                      const StreamTabHooks &hooks)
{
    auto *tab = new QWidget(window);
    auto *grid = new QGridLayout(tab);
    grid->setContentsMargins(8, 8, 8, 8);
    grid->setSpacing(10);

    cameraUi.detectorView = new DetectorCrosshairWidget(tab);
    cameraUi.wavelengthView = new ProfilePlotWidget(ProfilePlotWidget::Mode::Wavelength, tab);
    cameraUi.pixelStreamView = new ProfilePlotWidget(ProfilePlotWidget::Mode::Spatial, tab);

    const double dnAxisMax =
        cameraUi.sensorKind == LumoSensorKind::Swir3Ni ? kMono16DnAxisMax : kMono12DnAxisMax;
    cameraUi.wavelengthView->setDnAxisMax(dnAxisMax);
    cameraUi.pixelStreamView->setDnAxisMax(dnAxisMax);

    ui::WaterfallDisplayWidget *waterfallWidget = nullptr;
    auto *detectorPane = createStreamPane(tab, QStringLiteral("Detector"), cameraUi.detectorView);
    auto *waterfallPane = createWaterfallPane(tab, QStringLiteral("Waterfall"), waterfallWidget);
    auto *wavelengthPane = createStreamPane(tab, QStringLiteral("Wavelength"), cameraUi.wavelengthView);
    auto *pixelStreamPane = createStreamPane(tab, QStringLiteral("Pixel"), cameraUi.pixelStreamView);

    cameraUi.detectorPane = detectorPane;
    cameraUi.waterfallPane = waterfallPane;
    cameraUi.wavelengthPane = wavelengthPane;
    cameraUi.pixelStreamPane = pixelStreamPane;
    cameraUi.waterfallView = waterfallWidget;

    QObject::connect(cameraUi.detectorView,
                     &DetectorCrosshairWidget::linesChanged,
                     window,
                     [hooks, &cameraUi](const int spatialIndex, const int bandIndex) {
                         if (hooks.onProfileLinesChanged)
                             hooks.onProfileLinesChanged(cameraUi, spatialIndex, bandIndex);
                     });

    if (hooks.updateStreamPaneTitles)
        hooks.updateStreamPaneTitles(cameraUi);

    const QString detectorMsg = cameraName + QStringLiteral(" detector (disconnected)");
    const QString wavelengthMsg = cameraName + QStringLiteral(" wavelength (disconnected)");
    const QString pixelMsg = cameraName + QStringLiteral(" pixel (disconnected)");
    cameraUi.detectorView->clearDisplay(detectorMsg);
    cameraUi.wavelengthView->clearDisplay(wavelengthMsg);
    cameraUi.pixelStreamView->clearDisplay(pixelMsg);
    setWaterfallDisconnectedText(cameraUi.waterfallView, QStringLiteral("waterfall"), cameraName);

    if (hooks.syncProfileRgbMarkers)
        hooks.syncProfileRgbMarkers(cameraUi);

    grid->addWidget(detectorPane, 0, 0);
    grid->addWidget(waterfallPane, 0, 1);
    grid->addWidget(wavelengthPane, 1, 0);
    grid->addWidget(pixelStreamPane, 1, 1);
    grid->setColumnStretch(0, 1);
    grid->setColumnStretch(1, 1);
    grid->setRowStretch(0, 1);
    grid->setRowStretch(1, 1);

    return tab;
}
} // namespace ui
