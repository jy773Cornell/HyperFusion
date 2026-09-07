// DLP3010EVM-LC projector settings panel (frontend/ui).
// Connect also arms. Blank is the fail-safe. FPP test is HDMI sine only (no flash/splash).
#pragma once

#include "backend/fpp/DlpTypes.hpp"

#include <QWidget>

#include <vector>

class QComboBox;
class QLabel;
class QPushButton;
class QSpinBox;

namespace ui
{
class DlpProjectorSettingsWidget : public QWidget
{
    Q_OBJECT

public:
    explicit DlpProjectorSettingsWidget(QWidget *parent = nullptr);

    [[nodiscard]] QPushButton *refreshButton() const { return refreshBtn_; }
    [[nodiscard]] QPushButton *connectButton() const { return connectBtn_; }
    [[nodiscard]] QPushButton *disconnectButton() const { return disconnectBtn_; }
    [[nodiscard]] QPushButton *blankButton() const { return blankBtn_; }
    [[nodiscard]] QPushButton *testPatternButton() const { return testPatternBtn_; }

    [[nodiscard]] hf::dlp::DlpProjectorSettings currentSettings() const;
    void setDevices(const std::vector<hf::dlp::DlpDeviceInfo> &devices, const QString &preferredId);
    void setConnectionStatus(const QString &text);
    void applyState(hf::dlp::DlpProjectorState state);
    void setLedMaxMilliamp(int maxMa);

signals:
    void settingsEdited();

private:
    void loadFromSettings();
    void saveToSettings() const;
    void onParameterChanged();
    void setComboText(QComboBox *combo, const QString &text) const;
    void clampLedSpins();

    QComboBox *deviceCombo_ = nullptr;
    QPushButton *refreshBtn_ = nullptr;
    QPushButton *connectBtn_ = nullptr;
    QPushButton *disconnectBtn_ = nullptr;
    QPushButton *blankBtn_ = nullptr;
    QLabel *connectionStatusLabel_ = nullptr;
    QComboBox *testPatternCombo_ = nullptr;
    QPushButton *testPatternBtn_ = nullptr;
    QSpinBox *ledRedSpin_ = nullptr;
    QSpinBox *ledGreenSpin_ = nullptr;
    QSpinBox *ledBlueSpin_ = nullptr;
    int ledMaxMa_ = 2400;
};
} // namespace ui
