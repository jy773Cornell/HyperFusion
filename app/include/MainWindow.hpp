// Main application window UI layout and control wiring.
#pragma once

#include <QMainWindow>

#include <memory>

#include "core/CameraTypes.hpp"
#include "orchestrator/CameraCoordinator.hpp"

class LumoCamera;
class QComboBox;
class QDoubleSpinBox;
class QGroupBox;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QTabWidget;
class QWidget;

struct LumoCameraUi
{
    std::shared_ptr<LumoCamera> camera;
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
    QComboBox *triggerCombo = nullptr;
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

private:
    QWidget *createStreamTabsPanel();
    QWidget *createSettingsPanel();
    QWidget *createCameraSettingsTab();
    QWidget *createStageSettingsTab();
    QWidget *createLightSettingsTab();
    QWidget *createUr3eSettingsTab();
    QWidget *createCaptureSettingsTab();
    QWidget *createStreamTabPage(const QString &cameraName, LumoCameraUi &cameraUi);
    QWidget *createRgbUr3eStreamTab();
    QGroupBox *createPreviewPane(const QString &title, QLabel *&labelOut);

    QGroupBox *createLumoCameraGroup(QWidget *parent, const QString &title, LumoCameraUi &ui);
    CameraSettings buildCameraSettings(const LumoCameraUi &ui) const;
    static QString defaultFx10eCalibrationPackPath();
    static QString calibrationPackPath(const LumoCameraUi &ui);
    static void setCalibrationPackDisplay(QLineEdit *edit, const QString &fullPath);

    void appendLog(const QString &message);
    void refreshLumoDeviceLists();
    void updateCameraControls(LumoCameraUi &ui, CameraState state);
    void updateShutterDisplay(LumoCameraUi &ui, bool isOpen);
    void onCameraStateChanged(LumoCameraUi &ui, CameraState state);
    void onShutterStateChanged(LumoCameraUi &ui, bool isOpen);
    void onCameraError(LumoCameraUi &ui, const CameraError &error);
    void updateDetectorFrame(const FramePacket &frame);
    void clearDetectorView(LumoCameraUi &ui);

    QPlainTextEdit *logOutput_ = nullptr;
    std::unique_ptr<CameraCoordinator> coordinator_;

    LumoCameraUi camera1Ui_;
    LumoCameraUi camera2Ui_;
};
