// Blackfly S (BFS) camera settings panel UI (frontend/ui layer).
// Layout mirrors SpinView; values persist via QSettings. Backend wired by BfsPanelController.
#pragma once

#include "backend/multiview/BfsCameraTypes.hpp"

#include <QWidget>

#include <vector>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QPushButton;
class QSpinBox;

namespace ui
{
class BfsCameraSettingsWidget : public QWidget
{
    Q_OBJECT

public:
    explicit BfsCameraSettingsWidget(QWidget *parent = nullptr);

    [[nodiscard]] QPushButton *refreshButton() const { return refreshCamerasBtn_; }
    [[nodiscard]] QPushButton *connectButton() const { return connectBtn_; }
    [[nodiscard]] QPushButton *disconnectButton() const { return disconnectBtn_; }
    [[nodiscard]] QPushButton *captureButton() const { return captureBtn_; }

    [[nodiscard]] hf::bfs::BfsCameraSettings currentSettings() const;
    void setDevices(const std::vector<hf::bfs::BfsDeviceInfo> &devices, const QString &preferredId);
    void setConnectionStatus(const QString &text);
    void setConnectedUi(bool connected);
    void setCaptureEnabled(bool enabled);

signals:
    void settingsEdited();

private:
    void loadFromSettings();
    void saveToSettings() const;
    void onParameterChanged();
    void setComboText(QComboBox *combo, const QString &text) const;
    void setParameterControlsEnabled(bool enabled);

    QComboBox *cameraCombo_ = nullptr;
    QPushButton *refreshCamerasBtn_ = nullptr;
    QPushButton *connectBtn_ = nullptr;
    QPushButton *disconnectBtn_ = nullptr;
    QPushButton *captureBtn_ = nullptr;
    QLabel *connectionStatusLabel_ = nullptr;

    QComboBox *acquisitionModeCombo_ = nullptr;
    QCheckBox *acquisitionFrameRateEnableCheck_ = nullptr;
    QDoubleSpinBox *acquisitionFrameRateSpin_ = nullptr;
    QDoubleSpinBox *evCompensationSpin_ = nullptr;
    QComboBox *exposureModeCombo_ = nullptr;
    QComboBox *exposureAutoCombo_ = nullptr;
    QDoubleSpinBox *exposureTimeSpin_ = nullptr;
    QSpinBox *exposureTimeLowerLimitMinSpin_ = nullptr;
    QSpinBox *exposureTimeLowerLimitMaxSpin_ = nullptr;
    QComboBox *gainAutoCombo_ = nullptr;
    QDoubleSpinBox *gainSpin_ = nullptr;
    QCheckBox *gammaEnableCheck_ = nullptr;
    QDoubleSpinBox *gammaSpin_ = nullptr;
    QComboBox *blackLevelSelectorCombo_ = nullptr;
    QDoubleSpinBox *blackLevelSpin_ = nullptr;
    QComboBox *balanceRatioSelectorCombo_ = nullptr;
    QDoubleSpinBox *balanceRatioSpin_ = nullptr;
    QComboBox *balanceWhiteAutoCombo_ = nullptr;
    QSpinBox *deviceLinkThroughputLimitSpin_ = nullptr;
};
} // namespace ui
