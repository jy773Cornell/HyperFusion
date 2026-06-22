// Persisted UI settings: hardware config, camera/stage/light form values.
#pragma once

#include "frontend/widgets/LumoCameraUi.hpp"

#include <QObject>

class MainWindow;
class QComboBox;

namespace hf::settings
{
class UiSettingsController : public QObject
{
    Q_OBJECT

public:
    explicit UiSettingsController(MainWindow *host, QObject *parent = nullptr);

    void loadHardwareConfig();
    void applyHardwareConfigToUi();
    void loadPersistedUiSettings();
    void savePersistedUiSettings();
    void schedulePersistedUiSettingsSave();
    void connectAutosave();
    void applyPersistedCameraProfilesAndBands();
    void applyPersistedCameraUiValues(LumoCameraUi &ui);
    void applyPersistedCameraProfileSelection(LumoCameraUi &ui);
    void savePersistedCameraSettings(const LumoCameraUi &ui);
    bool selectDeviceProfileByName(QComboBox *combo, const QString &profileName) const;

private:
    MainWindow *host_ = nullptr;
    class QTimer *saveTimer_ = nullptr;
};
} // namespace hf::settings
