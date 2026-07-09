// Stage tab orchestration: Zaber worker, homing, motion, and position polling.
#pragma once

#include "backend/stage/StageTypes.hpp"

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
    void syncPositionPollInterval();
    void onManualMotionStarted();
    void onManualMotionStopped();
    void onStopMotionRequested();
    void wireSettingsTabConnections();

public slots:
    void onStateChanged(StageState state);
    void onTopologyChanged(const StageTopology &topology);
    void onError(const StageError &error);

private:
    void updateDeviceDisplay(const StageTopology &topology);
    void clearDeviceDisplay();
    void beginManualMotionCoastWatch();
    void onManualMotionCoastPositionSample(double positionMm);
    void finishManualMotionCoast();

    static constexpr int kIdlePositionPollIntervalMs = 100;
    static constexpr int kManualPositionPollIntervalMs = 50;
    static constexpr int kManualMotionCoastStablePollsRequired = 4;
    static constexpr double kManualMotionCoastStableToleranceMm = 0.03;

    MainWindow *host_ = nullptr;
    std::unique_ptr<StageWorker> stageWorker_;
    int manualMotionDepth_ = 0;
    bool manualMotionCoastActive_ = false;
    int manualMotionCoastStableCount_ = 0;
    double manualMotionCoastLastPositionMm_ = 0.0;
};
} // namespace hf::stage
