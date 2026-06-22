// Per-camera Qt widget handles for settings and stream panels.
#pragma once

#include "adapters/lumo/LumoDeviceTypes.hpp"
#include "adapters/lumo/CalpackBandCatalog.hpp"
#include "backend/CameraTypes.hpp"

#include <cstddef>
#include <memory>
#include <vector>

class LumoCamera;
class QComboBox;
class QDoubleSpinBox;
class QGroupBox;
class QLabel;
class QLineEdit;
class QPushButton;

namespace ui
{
class DetectorCrosshairWidget;
class ProfilePlotWidget;
class WaterfallDisplayWidget;
} // namespace ui

struct LumoCameraUi
{
    std::shared_ptr<LumoCamera> camera;
    LumoSensorKind sensorKind = LumoSensorKind::Fx10ePleora;
    std::size_t cameraIndex = 0;
    CameraState state = CameraState::Disconnected;
    bool autoStreamStarted = false;
    bool connectAttemptActive = false;
    bool shutterReportedOpen = false;

    QComboBox *deviceCombo = nullptr;
    QLineEdit *calibrationPackEdit = nullptr;
    QPushButton *calibrationPackBrowseBtn = nullptr;
    QPushButton *connectBtn = nullptr;
    QPushButton *applyBtn = nullptr;
    QDoubleSpinBox *exposureSpin = nullptr;
    int frameWidth = 0;
    int frameHeight = 0;

    QGroupBox *detectorPane = nullptr;
    QGroupBox *waterfallPane = nullptr;
    QGroupBox *wavelengthPane = nullptr;
    QGroupBox *pixelStreamPane = nullptr;
    ui::DetectorCrosshairWidget *detectorView = nullptr;
    ui::WaterfallDisplayWidget *waterfallView = nullptr;
    ui::ProfilePlotWidget *wavelengthView = nullptr;
    ui::ProfilePlotWidget *pixelStreamView = nullptr;
    QDoubleSpinBox *frameRateSpin = nullptr;
    QComboBox *spectralBinningCombo = nullptr;
    QComboBox *spatialBinningCombo = nullptr;
    QLabel *shutterIndicator = nullptr;
    QLabel *shutterStatusLabel = nullptr;
    QPushButton *shutterToggleBtn = nullptr;
    QComboBox *redBandCombo = nullptr;
    QComboBox *greenBandCombo = nullptr;
    QComboBox *blueBandCombo = nullptr;
    QLabel *sessionUptimeLabel = nullptr;
    std::vector<SpectralBand> spectralBands;

    /// Cached wavelength axis for profile plot (invalidated when spectralBands / frame height changes).
    std::vector<double> wavelengthAxisCache;
    int wavelengthAxisBandCount = -1;
};
