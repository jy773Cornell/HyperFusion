// Per-camera Qt widget handles for the settings and detector panels.
#pragma once

#include <cstddef>
#include <memory>

#include "backend/CameraTypes.hpp"

class LumoCamera;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPushButton;

struct LumoCameraUi
{
    std::shared_ptr<LumoCamera> camera;
    std::size_t cameraIndex = 0;
    CameraState state = CameraState::Disconnected;

    QComboBox *deviceCombo = nullptr;
    QLineEdit *calibrationPackEdit = nullptr;
    QPushButton *calibrationPackBrowseBtn = nullptr;
    QPushButton *connectBtn = nullptr;
    QPushButton *applyBtn = nullptr;
    QDoubleSpinBox *exposureSpin = nullptr;
    int frameWidth = 0;
    int frameHeight = 0;

    class QGroupBox *detectorPane = nullptr;
    class QGroupBox *wavelengthPane = nullptr;
    class QGroupBox *pixelStreamPane = nullptr;
    QLabel *detectorView = nullptr;
    QLabel *waterfallView = nullptr;
    QLabel *wavelengthView = nullptr;
    QLabel *pixelStreamView = nullptr;
    QDoubleSpinBox *frameRateSpin = nullptr;
    QComboBox *spectralBinningCombo = nullptr;
    QComboBox *spatialBinningCombo = nullptr;
    QLabel *shutterIndicator = nullptr;
    QLabel *shutterStatusLabel = nullptr;
    QPushButton *shutterToggleBtn = nullptr;
    QComboBox *redBandCombo = nullptr;
    QComboBox *greenBandCombo = nullptr;
    QComboBox *blueBandCombo = nullptr;
};
