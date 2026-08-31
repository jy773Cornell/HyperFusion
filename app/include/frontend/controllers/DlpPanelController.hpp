// DLP Multiview tab: worker, connect (auto-arm), Blank, test pattern (frontend).
#pragma once

#include "backend/fpp/DlpTypes.hpp"

#include <QObject>
#include <QTimer>

#include <memory>

class MainWindow;

namespace hf::dlp
{
class DlpProjectorWorker;

class DlpPanelController : public QObject
{
    Q_OBJECT

public:
    explicit DlpPanelController(MainWindow *host, QObject *parent = nullptr);
    ~DlpPanelController() override;

    void initializeWorker();
    void shutdownSync();
    void wireSettingsTabConnections();
    [[nodiscard]] bool isConnected() const;

private:
    void onRefreshClicked();
    void onConnectClicked();
    void onDisconnectClicked();
    void onBlankClicked();
    void onTestPatternClicked();
    void onSettingsEdited();
    void applyLedCurrentsFromUi();
    void onStateChanged(DlpProjectorState state);
    void onError(const DlpError &error);
    void onDevices(const std::vector<DlpDeviceInfo> &devices);
    void onWorkerLog(const QString &message);
    [[nodiscard]] DlpProjectorSettings settingsFromUi() const;

    MainWindow *host_ = nullptr;
    std::unique_ptr<DlpProjectorWorker> worker_;
    QTimer *ledApplyTimer_ = nullptr;
};
} // namespace hf::dlp
