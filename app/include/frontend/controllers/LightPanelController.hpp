// Light / lighthouse tab orchestration: DAQ worker and lamp UI sync.
#pragma once

#include "backend/light/LighthouseTypes.hpp"

#include <QElapsedTimer>
#include <QObject>

#include <array>
#include <memory>

class MainWindow;
class LighthouseWorker;

namespace hf::light
{
class LightPanelController : public QObject
{
    Q_OBJECT

public:
    explicit LightPanelController(MainWindow *host, QObject *parent = nullptr);

    void initializeWorker();
    void shutdownSync();

    [[nodiscard]] LighthouseWorker *worker() const;
    [[nodiscard]] bool isSessionActive() const;

    void syncUiFromBackend();
    void updateConnectionDisplay();
    void updateControlsEnabled();
    void updateLampUptimeDisplay();
    void resetLampUptimes();
    void updatePowerDisplay(const LighthouseControllerPowerStatus &status);
    void applySettingsToUi(const LighthouseSettings &settings);
    void applyPersistedUiValues();
    void savePersistedSettings() const;
    LighthouseSettings buildConnectDefaults() const;
    void setRowIntensity(int rowIndex, int percent);

public slots:
    void onStateChanged(LighthouseState state);
    void onDeviceInfoChanged(const LighthouseDeviceInfo &info);
    void onSettingsChanged(const LighthouseSettings &settings);
    void onPowerStatusChanged(const LighthouseControllerPowerStatus &status);
    void onError(const LighthouseError &error);

private:
    static int partnerIndex(int rowIndex);

    MainWindow *host_ = nullptr;
    std::unique_ptr<LighthouseWorker> lighthouseWorker_;
    class QTimer *powerPollTimer_ = nullptr;
    std::array<QElapsedTimer, kLighthouseLampCount> lampUptimeElapsed_{};
};
} // namespace hf::light
