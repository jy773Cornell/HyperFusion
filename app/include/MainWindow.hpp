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
class QPlainTextEdit;
class QPushButton;
class QTabWidget;
class QWidget;

struct LumoCameraUi
{
    std::shared_ptr<LumoCamera> camera;
    std::size_t cameraIndex = 0;
    CameraState state = CameraState::Disconnected;

    QComboBox *deviceCombo = nullptr;
    QPushButton *connectBtn = nullptr;
    QPushButton *applyBtn = nullptr;
    QPushButton *startBtn = nullptr;
    QPushButton *stopBtn = nullptr;
    QDoubleSpinBox *exposureSpin = nullptr;
    QDoubleSpinBox *frameRateSpin = nullptr;
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
    QWidget *createStreamTabPage(const QString &cameraName);
    QWidget *createRgbUr3eStreamTab();
    QGroupBox *createPreviewPane(const QString &title, QLabel *&labelOut);

    QGroupBox *createLumoCameraGroup(QWidget *parent, const QString &title, LumoCameraUi &ui);
    CameraSettings buildCameraSettings(const LumoCameraUi &ui) const;

    void appendLog(const QString &message);
    void refreshLumoDeviceLists();
    void updateCameraControls(LumoCameraUi &ui, CameraState state);

    QPlainTextEdit *logOutput_ = nullptr;
    std::unique_ptr<CameraCoordinator> coordinator_;

    LumoCameraUi camera1Ui_;
    LumoCameraUi camera2Ui_;
};
