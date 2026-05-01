#pragma once

#include <QMainWindow>

class QGroupBox;
class QLabel;
class QPlainTextEdit;
class QTabWidget;
class QWidget;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override = default;

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
    QWidget *createDualSensorStackTab();
    QGroupBox *createPreviewPane(const QString &title, QLabel *&labelOut);
    void appendLog(const QString &message);

    QPlainTextEdit *logOutput_ = nullptr;
};
