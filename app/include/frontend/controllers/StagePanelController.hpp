// Stage tab orchestration: Zaber worker, homing, motion, and position polling.
#pragma once

#include "backend/StageTypes.hpp"

#include <QObject>
#include <QString>

#include <memory>

class MainWindow;
class StageWorker;

namespace hf::stage
{
class StagePanelController : public QObject
{
    Q_OBJECT

public:
    explicit StagePanelController(MainWindow *host, QObject *parent = nullptr);

    void initializeWorker();
    void shutdownSync(bool homeBeforeDisconnect);

    [[nodiscard]] StageWorker *worker() const;
    [[nodiscard]] bool isSessionActive() const;

    void refreshComPortList();
    [[nodiscard]] QString selectedPortName() const;
    void updateConnectionControls(StageState state, bool refreshCaptureControls = true);
    void updateMotionControls(StageState state);
    void updatePositionDisplay(double positionMm);
    void pollPosition();
    void wireSettingsTabConnections();

public slots:
    void onStateChanged(StageState state);
    void onTopologyChanged(const StageTopology &topology);
    void onError(const StageError &error);

private:
    void updateDeviceDisplay(const StageTopology &topology);
    void clearDeviceDisplay();

    MainWindow *host_ = nullptr;
    std::unique_ptr<StageWorker> stageWorker_;
    bool positionPollInFlight_ = false;
    class QTimer *positionTimer_ = nullptr;
};
} // namespace hf::stage
